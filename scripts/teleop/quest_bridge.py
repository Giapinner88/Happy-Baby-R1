#!/usr/bin/env python3
"""Quest XR telemetry to normalized R1TeleopCommand JSONL.

This process owns the vendor boundary only. It runs in the `tv` environment,
which has the Quest vendor wrapper but no IsaacLab, and writes newline-delimited
`R1TeleopCommand` JSON to stdout for `run_r1_quest3_live.py` to consume over a
pipe. It imports no simulator, DDS, ROS, Unitree SDK, or hardware code.

stdout carries the command stream and nothing else; all human-readable progress
and connection logging goes to stderr and to --connection-log.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json
import math
import platform
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit


ROOT = Path(__file__).resolve().parents[2]
TELEVUER_SOURCE = ROOT / "third_party" / "xr_teleoperate" / "teleop" / "televuer" / "src"
sys.path.insert(0, str(ROOT))

from teleop.r1 import BridgeConfig, QuestCommandBridge, QuestTransportSample  # noqa: E402
from teleop.r1.frame_contract import validate_vendor_frame_contract  # noqa: E402
from scripts.teleop.robot_camera_vuer import (  # noqa: E402
    CAMERA_TOGGLE_BUTTONS,
    CameraToggleLatch,
    RobotCameraConfig,
    RobotCameraFrameConfig,
    build_frame_switchable_televuer_wrapper,
    build_switchable_televuer_wrapper,
)
from scripts.teleop.robot_camera_stream import (  # noqa: E402
    HttpRobotCameraSubscriber,
    RobotCameraSnapshot,
)


DEADMAN_SOURCES = ("right_trigger", "left_trigger", "either_trigger")
TRIGGER_VALUE_MIN = 0.0
TRIGGER_VALUE_MAX = 10.0
DEFAULT_TRIGGER_VALUE_THRESHOLD = 5.0


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


def _log(stream, connection_log, record: dict[str, object]) -> None:
    line = json.dumps(record, sort_keys=True)
    print(line, file=stream, flush=True)
    if connection_log is not None:
        connection_log.write(line + "\n")
        connection_log.flush()


def _redacted_command(argv: list[str]) -> list[str]:
    """Keep camera endpoint query parameters out of the connection log."""

    result = list(argv)
    for flag in ("--robot-camera-webrtc-url", "--robot-camera-preview-url"):
        start = 0
        while True:
            try:
                index = result.index(flag, start) + 1
            except ValueError:
                break
            if index < len(result):
                parsed = urlsplit(result[index])
                result[index] = urlunsplit((parsed.scheme, parsed.netloc, parsed.path, "", ""))
            start = index + 1
    return result


class LocalCameraViewController:
    """Apply local preview frames and enforce view freshness independently."""

    def __init__(
        self,
        display: object,
        config: RobotCameraFrameConfig,
        *,
        enable_max_age_s: float,
        fallback_max_age_s: float,
    ) -> None:
        self._display = display
        self._config = config
        self._toggle = CameraToggleLatch(config.toggle_button)
        self._enable_max_age_s = float(enable_max_age_s)
        self._fallback_max_age_s = float(fallback_max_age_s)
        self._enabled = False
        self._last_sequence = -1

    @staticmethod
    def _ages(
        snapshot: RobotCameraSnapshot | None, now_s: float
    ) -> tuple[float, float]:
        if snapshot is None:
            return math.inf, math.inf
        receive_age_s = max(0.0, float(now_s) - snapshot.received_monotonic_s)
        source_age_s = snapshot.robot_source_age_s + receive_age_s
        return source_age_s, receive_age_s

    @staticmethod
    def _stale_reason(source_age_s: float, receive_age_s: float, limit_s: float) -> str:
        source_stale = source_age_s > limit_s
        receive_stale = receive_age_s > limit_s
        if source_stale and receive_stale:
            return "robot_source_and_workstation_receive_stale"
        if source_stale:
            return "robot_source_stale"
        if receive_stale:
            return "workstation_receive_stale"
        return "fresh"

    def update(
        self,
        telemetry: object,
        snapshot: RobotCameraSnapshot | None,
        *,
        now_s: float,
        motion_data_ready: bool = True,
    ) -> list[dict[str, object]]:
        events: list[dict[str, object]] = []
        if snapshot is not None and snapshot.sequence > self._last_sequence:
            if self._display.set_robot_camera_frame(snapshot.bgr, snapshot.sequence):
                self._last_sequence = snapshot.sequence

        source_age_s, receive_age_s = self._ages(snapshot, now_s)
        transition = self._toggle.update(telemetry) if motion_data_ready else None
        if transition is True:
            reason = self._stale_reason(
                source_age_s, receive_age_s, self._enable_max_age_s
            )
            if snapshot is None or reason != "fresh":
                self._toggle.set_enabled(False)
                events.append(
                    {
                        "event": "camera_enable_rejected",
                        "timestamp_monotonic_s": now_s,
                        "button": self._config.toggle_button,
                        "reason": "no_frame" if snapshot is None else reason,
                        "robot_source_age_s": source_age_s,
                        "workstation_receive_age_s": receive_age_s,
                    }
                )
            else:
                self._display.set_robot_camera_enabled(True)
                self._enabled = True
                events.append(
                    {
                        "event": "camera_view_changed",
                        "timestamp_monotonic_s": now_s,
                        "button": self._config.toggle_button,
                        "view": "robot_camera",
                        "robot_source_age_s": source_age_s,
                        "workstation_receive_age_s": receive_age_s,
                    }
                )
        elif transition is False:
            self._display.set_robot_camera_enabled(False)
            self._enabled = False
            events.append(
                {
                    "event": "camera_view_changed",
                    "timestamp_monotonic_s": now_s,
                    "button": self._config.toggle_button,
                    "view": "quest_passthrough",
                }
            )

        if self._enabled:
            reason = self._stale_reason(
                source_age_s, receive_age_s, self._fallback_max_age_s
            )
            if snapshot is None or reason != "fresh":
                self._display.set_robot_camera_enabled(False)
                self._enabled = False
                self._toggle.set_enabled(False)
                events.append(
                    {
                        "event": "camera_runtime_fallback",
                        "timestamp_monotonic_s": now_s,
                        "view": "quest_passthrough",
                        "reason": "no_frame" if snapshot is None else reason,
                        "robot_source_age_s": source_age_s,
                        "workstation_receive_age_s": receive_age_s,
                    }
                )
        return events


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip", required=True, help="Host IP shown in the Quest connection URL.")
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
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument(
        "--robot-camera-webrtc-url",
        help="WebRTC offer URL for the robot camera; omitted keeps Quest passthrough only.",
    )
    camera_group.add_argument(
        "--robot-camera-preview-url",
        help="Loopback HTTP preview URL forwarded from the R1 camera gateway.",
    )
    parser.add_argument("--robot-camera-http-timeout-s", type=float, default=0.4)
    parser.add_argument("--robot-camera-poll-hz", type=float, default=10.0)
    parser.add_argument("--camera-enable-max-age-s", type=float, default=0.5)
    parser.add_argument("--camera-fallback-max-age-s", type=float, default=1.0)
    parser.add_argument(
        "--camera-toggle-button",
        choices=tuple(CAMERA_TOGGLE_BUTTONS),
        default="right_a",
        help="Controller button whose press edge toggles Quest/robot view.",
    )
    parser.add_argument(
        "--robot-camera-layout",
        choices=("mono", "stereo-sbs"),
        default="mono",
        help="Layout emitted by the robot camera WebRTC stream.",
    )
    parser.add_argument("--robot-camera-aspect", type=float, default=16.0 / 9.0)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)

    if (
        not math.isfinite(args.duration_s)
        or not math.isfinite(args.frequency_hz)
        or args.duration_s <= 0.0
        or args.frequency_hz <= 0.0
    ):
        raise SystemExit("--duration-s and --frequency-hz must be positive.")
    if not math.isfinite(args.trigger_value_threshold) or not (
        TRIGGER_VALUE_MIN <= args.trigger_value_threshold < TRIGGER_VALUE_MAX
    ):
        raise SystemExit("--trigger-value-threshold must be finite and in [0, 10).")
    if (args.cert_file is None) != (args.key_file is None):
        raise SystemExit("Specify both --cert-file and --key-file, or neither.")
    if args.stop_file is not None and args.stop_file.expanduser().exists():
        raise SystemExit(f"Refusing to start: --stop-file already exists: {args.stop_file}")
    if not math.isfinite(args.robot_camera_http_timeout_s) or args.robot_camera_http_timeout_s <= 0:
        raise SystemExit("--robot-camera-http-timeout-s must be finite and positive.")
    if not math.isfinite(args.robot_camera_poll_hz) or not 1 <= args.robot_camera_poll_hz <= 15:
        raise SystemExit("--robot-camera-poll-hz must be finite and in [1, 15].")
    if (
        not math.isfinite(args.camera_enable_max_age_s)
        or not math.isfinite(args.camera_fallback_max_age_s)
        or args.camera_enable_max_age_s <= 0
        or args.camera_fallback_max_age_s < args.camera_enable_max_age_s
    ):
        raise SystemExit(
            "camera age limits must be finite, positive, and fallback >= enable."
        )
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    camera_config: RobotCameraConfig | None = None
    frame_camera_config: RobotCameraFrameConfig | None = None
    camera_subscriber: HttpRobotCameraSubscriber | None = None
    if args.robot_camera_webrtc_url:
        try:
            camera_config = RobotCameraConfig(
                webrtc_url=args.robot_camera_webrtc_url,
                toggle_button=args.camera_toggle_button,
                layout=args.robot_camera_layout,
                aspect_ratio=args.robot_camera_aspect,
            )
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
    elif args.robot_camera_preview_url:
        try:
            frame_camera_config = RobotCameraFrameConfig(
                toggle_button=args.camera_toggle_button,
                width=640,
                height=360,
                aspect_ratio=args.robot_camera_aspect,
            )
            camera_subscriber = HttpRobotCameraSubscriber(
                args.robot_camera_preview_url,
                poll_hz=args.robot_camera_poll_hz,
                timeout_s=args.robot_camera_http_timeout_s,
                expected_width=frame_camera_config.width,
                expected_height=frame_camera_config.height,
            )
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    connection_log = None
    if args.connection_log is not None:
        args.connection_log.parent.mkdir(parents=True, exist_ok=True)
        connection_log = args.connection_log.open("w", encoding="utf-8")

    bridge_config = BridgeConfig()
    vuer_url = f"https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012"
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
            "robot_camera": (
                None
                if camera_config is None and frame_camera_config is None
                else {
                    "transport": "webrtc" if camera_config is not None else "local_http_preview",
                    "endpoint": (
                        camera_config.log_url
                        if camera_config is not None
                        else _redacted_command(
                            ["--robot-camera-preview-url", args.robot_camera_preview_url]
                        )[1]
                    ),
                    "initial_view": "quest_passthrough",
                    "toggle_button": (
                        camera_config.toggle_button
                        if camera_config is not None
                        else frame_camera_config.toggle_button
                    ),
                    "layout": camera_config.layout if camera_config is not None else "mono",
                    "aspect_ratio": (
                        camera_config.aspect_ratio
                        if camera_config is not None
                        else frame_camera_config.aspect_ratio
                    ),
                }
            ),
            "bridge_config": asdict(bridge_config),
            "python_version": sys.version,
            "platform": platform.platform(),
            "command": [sys.executable, *_redacted_command(sys.argv)],
        },
    )

    sys.path.insert(0, str(TELEVUER_SOURCE))
    try:
        from televuer import TeleVuerWrapper
        from televuer import tv_wrapper as vendor_frame_module
    except ImportError as exc:
        _log(sys.stderr, connection_log, {"event": "bridge_failed", "reason": f"Cannot import TeleVuer: {exc}"})
        raise SystemExit(f"Cannot import TeleVuer in this environment: {exc}") from exc

    frame_contract = validate_vendor_frame_contract(vendor_frame_module)
    _log(
        sys.stderr,
        connection_log,
        {"event": "vendor_frame_contract_verified", **frame_contract},
    )

    wrapper_args = {
        "use_hand_tracking": False,
        "binocular": True,
        "img_shape": (480, 1280),
        "display_fps": args.frequency_hz,
        "cert_file": str(args.cert_file.resolve()) if args.cert_file else None,
        "key_file": str(args.key_file.resolve()) if args.key_file else None,
        "arm_reference_mode": bridge_config.arm_reference_mode,
    }
    local_camera_controller: LocalCameraViewController | None = None
    if camera_config is None and frame_camera_config is None:
        wrapper = TeleVuerWrapper(
            display_mode="pass-through", zmq=False, webrtc=False, **wrapper_args
        )
        camera_toggle = None
    elif camera_config is not None:
        wrapper = build_switchable_televuer_wrapper(
            vendor_frame_module, camera_config, **wrapper_args
        )
        camera_toggle = CameraToggleLatch(camera_config.toggle_button)
    else:
        assert frame_camera_config is not None
        assert camera_subscriber is not None
        wrapper = build_frame_switchable_televuer_wrapper(
            vendor_frame_module, frame_camera_config, **wrapper_args
        )
        camera_subscriber.start()
        local_camera_controller = LocalCameraViewController(
            wrapper.tvuer,
            frame_camera_config,
            enable_max_age_s=args.camera_enable_max_age_s,
            fallback_max_age_s=args.camera_fallback_max_age_s,
        )
        camera_toggle = None
    print(
        f"Open {vuer_url} in Quest Browser, accept the certificate, then ENTER VR before moving.",
        file=sys.stderr,
        flush=True,
    )

    bridge = QuestCommandBridge(bridge_config)
    previous_reset_pressed = False
    previous_trigger_pressed: tuple[bool, bool] | None = None
    period_s = 1.0 / args.frequency_hz
    deadline = time.monotonic() + args.duration_s
    emitted = 0
    reported_transitions = 0
    stop_requested, stop_detail, previous_handlers = _install_stop_handlers()
    stop_reason = "duration_elapsed"
    try:
        while time.monotonic() < deadline:
            if stop_requested.is_set():
                stop_reason = stop_detail["reason"]
                break
            if args.stop_file is not None and args.stop_file.expanduser().exists():
                stop_reason = "stop_file_requested"
                break
            loop_start = time.monotonic()
            telemetry = wrapper.get_tele_data()
            if local_camera_controller is not None:
                assert camera_subscriber is not None
                for camera_event in local_camera_controller.update(
                    telemetry,
                    camera_subscriber.snapshot(),
                    now_s=loop_start,
                    motion_data_ready=bool(telemetry.motion_data_ready),
                ):
                    _log(sys.stderr, connection_log, camera_event)
            elif camera_toggle is not None and bool(telemetry.motion_data_ready):
                camera_enabled = camera_toggle.update(telemetry)
                if camera_enabled is not None:
                    wrapper.tvuer.set_robot_camera_enabled(camera_enabled)
                    _log(
                        sys.stderr,
                        connection_log,
                        {
                            "event": "camera_view_changed",
                            "timestamp_monotonic_s": loop_start,
                            "view": "robot_camera" if camera_enabled else "quest_passthrough",
                            "button": camera_config.toggle_button,
                        },
                    )
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
                        "left": left_trigger.as_log_dict(),
                        "right": right_trigger.as_log_dict(),
                        "deadman_enabled": _deadman_pressed(left_trigger, right_trigger, args.deadman_source),
                    },
                )
                previous_trigger_pressed = trigger_pressed
            reset_pressed = left_trigger.effective_pressed
            transport = QuestTransportSample(
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
                    print(json.dumps(command.as_dict(), sort_keys=True), flush=True)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
