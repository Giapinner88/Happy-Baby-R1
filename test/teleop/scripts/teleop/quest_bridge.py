#!/usr/bin/env python3
"""T001 live bridge: Quest XR telemetry to normalized R1TeleopCommand JSONL.

This process owns the vendor boundary only. It runs in the `tv` environment,
which has the Quest vendor wrapper but no IsaacLab, and writes newline-delimited
`R1TeleopCommand` JSON to stdout for `run_r1_quest3_live.py` to consume over a
pipe. The default path imports no simulator, DDS, ROS, Unitree SDK, or hardware
code; the explicit `--robot-camera` mode adds a read-only local videohub client.

stdout carries the command stream and nothing else; all human-readable progress
and connection logging goes to stderr and to --connection-log.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict, dataclass
import json
import math
import os
import platform
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
TELEVUER_SOURCE = ROOT / "third_party" / "xr_teleoperate" / "teleop" / "televuer" / "src"
sys.path.insert(0, str(ROOT))

from teleop.r1 import BridgeConfig, QuestCommandBridge, QuestTransportSample  # noqa: E402


DEADMAN_SOURCES = ("right_trigger", "left_trigger", "either_trigger")
TRIGGER_VALUE_MIN = 0.0
TRIGGER_VALUE_MAX = 10.0
DEFAULT_TRIGGER_VALUE_THRESHOLD = 5.0
CAMERA_TOGGLE_FIELD = "right_ctrl_aButton"


@dataclass(frozen=True)
class TriggerState:
    """One TeleVuer controller trigger with a fail-closed analog fallback.

    The vendored TeleVuer wrapper reports trigger pull depth on an inverted
    10.0 (released) to 0.0 (fully pressed) scale. Some Quest Browser/WebXR
    sessions update that analog value without ever asserting the companion
    boolean ``trigger`` field. Values outside the declared wrapper range are
    recorded but cannot enable motion.
    """

    digital_pressed: bool
    analog_value: float | None
    analog_valid: bool
    analog_pressed: bool
    effective_pressed: bool

    def as_log_dict(self) -> dict[str, object]:
        return {
            "digital_pressed": self.digital_pressed,
            "analog_value": self.analog_value,
            "analog_valid": self.analog_valid,
            "analog_pressed": self.analog_pressed,
            "effective_pressed": self.effective_pressed,
        }


@dataclass(frozen=True)
class RobotCameraConfig:
    """Robot-local, read-only camera settings for the Quest background."""

    interface: str
    capture_hz: float
    width: int
    height: int
    enable_max_age_s: float
    fallback_max_age_s: float
    media_port: int = 8013
    startup_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        if not self.interface or any(char.isspace() for char in self.interface):
            raise ValueError("robot camera interface must be non-empty and contain no whitespace")
        if not math.isfinite(self.capture_hz) or self.capture_hz < 0:
            raise ValueError("robot camera capture rate must be >=0; 0 means adaptive FPS")
        if not math.isfinite(self.startup_timeout_s) or self.startup_timeout_s <= 0:
            raise ValueError("camera startup timeout must be positive")
        if not 1 <= self.media_port <= 65535:
            raise ValueError("camera media port must be in [1, 65535]")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("robot camera dimensions must be positive")
        if (not math.isfinite(self.enable_max_age_s) or not math.isfinite(self.fallback_max_age_s)
                or self.enable_max_age_s <= 0.0 or self.fallback_max_age_s < self.enable_max_age_s):
            raise ValueError("camera age limits must be positive and fallback >= enable")


from quest_camera_webrtc import WebRTCCameraClient as RobotCameraSubscriber, build_camera_wrapper

def _compact_log_line(record: dict[str, object]) -> str:
    """Return one short operator line; keep the full record in JSONL evidence."""
    event = str(record.get("event", "event"))
    if event == "bridge_start":
        return f"[BRIDGE] ready url={record.get('vuer_url')} trigger={record.get('deadman_source')}"
    if event == "controller_trigger_transition":
        return "[BRIDGE] trigger=" + ("ON" if record.get("deadman_enabled") else "OFF")
    if event == "connected":
        return "[BRIDGE] Quest connected"
    if event == "disconnected":
        return f"[BRIDGE] Quest disconnected: {record.get('detail', 'pose unavailable')}"
    if event == "rejected":
        return f"[BRIDGE] ERROR rejected: {record.get('detail', 'invalid sample')}"
    if event == "bridge_failed":
        return f"[BRIDGE] ERROR {record.get('reason', 'startup failed')}"
    if event == "bridge_stop":
        return (
            f"[BRIDGE] stop={record.get('stop_reason', 'unknown')} "
            f"cmd={record.get('emitted_command_count', 0)} "
            f"drop={record.get('dropped_sample_count', 0)}"
        )
    if event == "camera_view_changed":
        return "[CAMERA] " + str(record.get("view", "unknown"))
    if event == "camera_capture_requested":
        return "[CAMERA] requesting fresh robot frame"
    if event in {"camera_enable_rejected", "camera_runtime_fallback"}:
        return f"[CAMERA] {event}: {record.get('reason', 'unknown')}"
    return f"[BRIDGE] {event}: {record.get('detail', record.get('reason', ''))}".rstrip(": ")


def _log(stream, connection_log, record: dict[str, object]) -> None:
    record.setdefault("timestamp_monotonic_s", time.monotonic())
    line = json.dumps(record, sort_keys=True)
    # Full JSON remains available in bridge_connection.jsonl.  Terminal output
    # is intentionally compact so a real failure is visible at a glance.
    if os.environ.get("HB_TELEOP_VERBOSE_LOG") == "1":
        print(line, file=stream, flush=True)
    else:
        print(_compact_log_line(record), file=stream, flush=True)
    if connection_log is not None:
        connection_log.write(line + "\n")
        connection_log.flush()


def _trigger_state(telemetry: object, side: str, analog_threshold: float) -> TriggerState:
    digital_pressed = bool(getattr(telemetry, f"{side}_ctrl_trigger", False))
    analog_value: float | None
    try:
        analog_value = float(getattr(telemetry, f"{side}_ctrl_triggerValue"))
    except (AttributeError, TypeError, ValueError):
        analog_value = None
    analog_valid = bool(
        analog_value is not None
        and math.isfinite(analog_value)
        and TRIGGER_VALUE_MIN <= analog_value <= TRIGGER_VALUE_MAX
    )
    analog_pressed = bool(analog_valid and analog_value is not None and analog_value <= analog_threshold)
    return TriggerState(
        digital_pressed=digital_pressed,
        analog_value=analog_value,
        analog_valid=analog_valid,
        analog_pressed=analog_pressed,
        effective_pressed=digital_pressed or analog_pressed,
    )


def _deadman_pressed(left: TriggerState, right: TriggerState, source: str) -> bool:
    if source == "right_trigger":
        return right.effective_pressed
    if source == "left_trigger":
        return left.effective_pressed
    return left.effective_pressed or right.effective_pressed


def _install_stop_handlers() -> tuple[threading.Event, dict[str, str], dict[int, object]]:
    """Turn termination signals into a loop-level, evidence-preserving stop."""

    requested = threading.Event()
    detail: dict[str, str] = {"reason": ""}
    previous: dict[int, object] = {}

    def request_stop(signum: int, _frame: object) -> None:
        detail["reason"] = f"signal_{signal.Signals(signum).name}"
        requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return requested, detail, previous


def _restore_stop_handlers(previous: dict[int, object]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip", required=True, help="Host IP shown in the Quest connection URL.")
    parser.add_argument("--port", type=int, default=8012, help="HTTPS/WebXR port.")
    parser.add_argument("--duration-s", type=float, default=180.0, help="Bridge lifetime after the endpoint starts.")
    parser.add_argument("--frequency-hz", type=float, default=30.0, help="Command emission rate.")
    parser.add_argument(
        "--deadman-source",
        choices=DEADMAN_SOURCES,
        default="right_trigger",
        help="Controller input that enables command dispatch; releasing it must hold the simulator.",
    )
    parser.add_argument(
        "--trigger-value-threshold",
        type=float,
        default=DEFAULT_TRIGGER_VALUE_THRESHOLD,
        help=(
            "TeleVuer analog trigger threshold on its inverted 10=released, 0=fully-pressed scale. "
            "A valid value at or below this threshold counts as pressed when WebXR leaves the boolean false."
        ),
    )
    parser.add_argument("--connection-log", type=Path, help="Optional JSONL copy of the connection/status log.")
    parser.add_argument(
        "--stop-file",
        type=Path,
        help="A path that must not exist at startup; create it to request a graceful live stop.",
    )
    parser.add_argument("--cert-file", type=Path, help="HTTPS certificate for the Quest endpoint.")
    parser.add_argument("--key-file", type=Path, help="Private key paired with --cert-file.")
    parser.add_argument("--robot-camera", action="store_true",
                        help="Read the R1 videohub camera locally; A right toggles the Quest view.")
    parser.add_argument("--robot-camera-interface", default="eth10")
    parser.add_argument("--robot-camera-capture-hz", type=float, default=0.0)
    parser.add_argument("--robot-camera-width", type=int, default=640)
    parser.add_argument("--robot-camera-media-port", type=int, default=8013)
    parser.add_argument("--robot-camera-height", type=int, default=360)
    parser.add_argument("--camera-startup-timeout-s", type=float, default=5.0)
    parser.add_argument("--camera-enable-max-age-s", type=float, default=0.5)
    parser.add_argument("--camera-fallback-max-age-s", type=float, default=1.0)
    args = parser.parse_args()

    if args.duration_s < 0.0 or args.frequency_hz <= 0.0 or not 1 <= args.port <= 65535:
        raise SystemExit("--duration-s must be non-negative; 0 means run forever.")
    if not math.isfinite(args.trigger_value_threshold) or not (
        TRIGGER_VALUE_MIN <= args.trigger_value_threshold < TRIGGER_VALUE_MAX
    ):
        raise SystemExit("--trigger-value-threshold must be finite and in [0, 10).")
    if (args.cert_file is None) != (args.key_file is None):
        raise SystemExit("Specify both --cert-file and --key-file, or neither.")
    if args.stop_file is not None and args.stop_file.expanduser().exists():
        raise SystemExit(f"Refusing to start: --stop-file already exists: {args.stop_file}")
    camera_config: RobotCameraConfig | None = None
    if args.robot_camera:
        try:
            camera_config = RobotCameraConfig(
                interface=args.robot_camera_interface,
                media_port=args.robot_camera_media_port,
                startup_timeout_s=args.camera_startup_timeout_s,
                capture_hz=args.robot_camera_capture_hz,
                width=args.robot_camera_width,
                height=args.robot_camera_height,
                enable_max_age_s=args.camera_enable_max_age_s,
                fallback_max_age_s=args.camera_fallback_max_age_s,
            )
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    connection_log = None
    if args.connection_log is not None:
        args.connection_log.parent.mkdir(parents=True, exist_ok=True)
        from teleop.hardware.stream_io import AsyncFileLog
        connection_log = AsyncFileLog(args.connection_log)

    bridge_config = BridgeConfig()
    vuer_url = f"https://{args.host_ip}:{args.port}/?ws=wss://{args.host_ip}:{args.port}"
    _log(
        sys.stderr,
        connection_log,
        {
            "event": "bridge_start",
            "utc": datetime.now(timezone.utc).isoformat(),
            "vuer_url": vuer_url,
            "deadman_source": args.deadman_source,
            "trigger_input": {
                "policy": "digital_or_valid_analog",
                "analog_scale": "10.0=released, 0.0=fully_pressed",
                "analog_valid_range": [TRIGGER_VALUE_MIN, TRIGGER_VALUE_MAX],
                "analog_pressed_at_or_below": args.trigger_value_threshold,
            },
            "frequency_hz": args.frequency_hz,
            "duration_s": args.duration_s,
            "robot_camera": None if camera_config is None else {
                "source": "r1_videohub_dds_local",
                "transport": "webrtc_vp8_separate_process",
                "toggle_button": "right_a",
                "initial_view": "quest_passthrough",
                "interface": camera_config.interface,
                "frame_size": [camera_config.width, camera_config.height],
            },
            "bridge_config": asdict(bridge_config),
            "python_version": sys.version,
            "platform": platform.platform(),
            "command": [sys.executable, *sys.argv],
        },
    )

    sys.path.insert(0, str(TELEVUER_SOURCE))
    try:
        from televuer import TeleVuerWrapper
        from televuer import tv_wrapper as vendor_frame_module
    except ImportError as exc:
        _log(sys.stderr, connection_log, {"event": "bridge_failed", "reason": f"Cannot import TeleVuer: {exc}"})
        raise SystemExit(f"Cannot import TeleVuer in this environment: {exc}") from exc

    wrapper_args = dict(
        use_hand_tracking=False,
        binocular=True,
        img_shape=(480, 1280),
        display_fps=args.frequency_hz,
        display_mode="pass-through",
        zmq=False,
        webrtc=False,
        cert_file=str(args.cert_file.resolve()) if args.cert_file else None,
        key_file=str(args.key_file.resolve()) if args.key_file else None,
        port=args.port,
    )
    camera_subscriber: RobotCameraSubscriber | None = None
    if camera_config is None:
        wrapper = build_camera_wrapper(vendor_frame_module, None, **wrapper_args)
    else:
        wrapper = build_camera_wrapper(vendor_frame_module, camera_config, **wrapper_args)
        camera_subscriber = RobotCameraSubscriber(camera_config)
        camera_subscriber.start()
    print(
        f"Open {vuer_url} in Quest Browser, accept the certificate, then ENTER VR before moving.",
        file=sys.stderr,
        flush=True,
    )

    bridge = QuestCommandBridge(bridge_config)
    previous_reset_pressed = False
    previous_trigger_pressed: tuple[bool, bool] | None = None
    period_s = 1.0 / args.frequency_hz
    deadline = None if args.duration_s == 0.0 else time.monotonic() + args.duration_s
    emitted = 0
    reported_transitions = 0
    camera_previous_pressed: bool | None = None
    camera_enabled = False
    camera_enable_requested_at: float | None = None
    camera_sequence = -1
    camera_visible_at = 0.0
    last_diagnostics_at = 0.0
    stop_requested, stop_detail, previous_handlers = _install_stop_handlers()
    stop_reason = "duration_elapsed"
    try:
        while deadline is None or time.monotonic() < deadline:
            if stop_requested.is_set():
                stop_reason = stop_detail["reason"]
                break
            if args.stop_file is not None and args.stop_file.expanduser().exists():
                stop_reason = "stop_file_requested"
                break
            loop_start = time.monotonic()
            with wrapper.tvuer.tracking.get_lock():
                telemetry = wrapper.get_tele_data()
                tracking_seen, tracking_received, tracking_valid = wrapper.tvuer.tracking[:3]
                arm_tracking = dict(zip(("left", "right"), map(bool, wrapper.tvuer.arm_tracking[:])))
            loop_start = time.monotonic()  # receipt cannot be newer than this sample time
            if camera_subscriber is not None:
                frame, sequence, age_s, camera_error = camera_subscriber.snapshot()
                if bool(telemetry.motion_data_ready):
                    camera_pressed = bool(getattr(telemetry, CAMERA_TOGGLE_FIELD, False))
                    edge = camera_previous_pressed is not None and camera_pressed and not camera_previous_pressed
                    if camera_previous_pressed is not None and camera_pressed != camera_previous_pressed:
                        _log(sys.stderr, connection_log, {
                            "event": "camera_button_transition",
                            "button": "right_a",
                            "pressed": camera_pressed,
                            "camera_enabled": camera_enabled,
                        })
                    camera_previous_pressed = camera_pressed
                    if edge:
                        if camera_enabled or camera_enable_requested_at is not None:
                            camera_enabled = False
                            camera_enable_requested_at = None
                            wrapper.tvuer.set_robot_camera_enabled(False)
                            camera_subscriber.set_capture_requested(False)
                            camera_sequence = -1
                            _log(sys.stderr, connection_log, {"event": "camera_view_changed", "view": "quest_passthrough"})
                        else:
                            camera_subscriber.set_capture_requested(True)
                            camera_enable_requested_at = loop_start
                            _log(sys.stderr, connection_log, {"event": "camera_capture_requested"})
                if camera_enable_requested_at is not None:
                    if frame and age_s <= camera_config.enable_max_age_s:
                        camera_enabled = True
                        camera_enable_requested_at = None
                        camera_sequence = sequence
                        camera_visible_at = loop_start
                        wrapper.tvuer.set_robot_camera_enabled(True)
                        _log(sys.stderr, connection_log, {"event": "camera_view_changed", "view": "robot_camera"})
                    elif loop_start - camera_enable_requested_at > camera_config.startup_timeout_s:
                        camera_enable_requested_at = None
                        camera_subscriber.set_capture_requested(False)
                        _log(sys.stderr, connection_log, {
                            "event": "camera_enable_rejected", "reason": "no_fresh_frame",
                            "camera_age_s": age_s, "camera_error": camera_error,
                        })
                peer_failed = (camera_enabled and loop_start - camera_visible_at > 8.0
                               and camera_subscriber.peer_status().get("connected_peers", 0) == 0)
                if camera_enabled and (not frame or age_s > camera_config.fallback_max_age_s or peer_failed):
                    camera_enabled = False
                    wrapper.tvuer.set_robot_camera_enabled(False)
                    camera_subscriber.set_capture_requested(False)
                    camera_sequence = -1
                    _log(sys.stderr, connection_log, {
                        "event": "camera_runtime_fallback", "reason": "peer_not_connected" if peer_failed else "stale_frame",
                        "camera_age_s": age_s, "camera_error": camera_error,
                    })
            diagnostics = wrapper.tvuer.input_diagnostics()
            if loop_start - last_diagnostics_at >= 1.0:
                if connection_log is not None:
                    connection_log.write(json.dumps({"event": "input_diagnostics", "timestamp_monotonic_s": loop_start,
                                                    "camera_enabled": camera_enabled, **diagnostics}) + "\n")
                    connection_log.flush()
                last_diagnostics_at = loop_start
            left_trigger = _trigger_state(telemetry, "left", args.trigger_value_threshold)
            right_trigger = _trigger_state(telemetry, "right", args.trigger_value_threshold)
            trigger_pressed = (left_trigger.effective_pressed, right_trigger.effective_pressed)
            if bool(telemetry.motion_data_ready) and trigger_pressed != previous_trigger_pressed:
                _log(
                    sys.stderr,
                    connection_log,
                    {
                        "event": "controller_trigger_transition",
                        "timestamp_monotonic_s": loop_start,
                        "input_diagnostics": diagnostics,
                        "left": left_trigger.as_log_dict(),
                        "right": right_trigger.as_log_dict(),
                        "deadman_enabled": _deadman_pressed(left_trigger, right_trigger, args.deadman_source),
                    },
                )
                previous_trigger_pressed = trigger_pressed
            reset_pressed = left_trigger.effective_pressed
            transport = QuestTransportSample(
                tracking_received_monotonic_s=tracking_received if tracking_seen else None,
                tracking_valid=bool(tracking_valid),
                motion_data_ready=bool(telemetry.motion_data_ready),
                head_pose_matrix=telemetry.head_pose.tolist(),
                left_wrist_pose_matrix=telemetry.left_wrist_pose.tolist(),
                right_wrist_pose_matrix=telemetry.right_wrist_pose.tolist(),
                deadman_pressed=_deadman_pressed(left_trigger, right_trigger, args.deadman_source),
                reset_requested=reset_pressed and not previous_reset_pressed,
            )
            previous_reset_pressed = reset_pressed
            command = bridge.build(transport, loop_start)
            if command is not None:
                try:
                    record = command.as_dict()
                    record["arm_tracking"] = arm_tracking
                    print(json.dumps(record, sort_keys=True), flush=True)
                except BrokenPipeError:
                    # A downstream deadman/watchdog stop is a normal foreground
                    # shutdown, not a bridge failure. Redirect stdout so Python's
                    # interpreter-finalization flush cannot emit a second error.
                    sys.stdout = open("/dev/null", "w", encoding="utf-8")
                    stop_reason = "downstream_closed"
                    break
                emitted += 1
            while reported_transitions < len(bridge.state.transitions):
                _log(sys.stderr, connection_log, bridge.state.transitions[reported_transitions])
                reported_transitions += 1
            remaining = period_s - (time.monotonic() - loop_start)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        if camera_subscriber is not None:
            camera_subscriber.close()
        wrapper.close()
        _restore_stop_handlers(previous_handlers)

    _log(
        sys.stderr,
        connection_log,
        {
            "event": "bridge_stop",
            "stop_reason": stop_reason,
            "utc": datetime.now(timezone.utc).isoformat(),
            "emitted_command_count": emitted,
            "connect_count": bridge.state.connect_count,
            "disconnect_count": bridge.state.disconnect_count,
            "dropped_sample_count": bridge.state.dropped_sample_count,
            "rejected_sample_count": bridge.state.rejected_sample_count,
        },
    )
    if connection_log is not None:
        connection_log.close()
        stats = connection_log.stats()
        if stats['dropped'] or stats['error'] or stats['pending']:
            print(f'[BRIDGE LOG] {json.dumps(stats)}', file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
