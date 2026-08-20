#!/usr/bin/env python3
"""T001 live bridge: Quest XR telemetry to normalized R1TeleopCommand JSONL.

This process owns the vendor boundary only. It runs in the `tv` environment,
which has the Quest vendor wrapper but no IsaacLab, and writes newline-delimited
`R1TeleopCommand` JSON to stdout for `run_r1_quest3_live.py` to consume over a
pipe. It imports no simulator, DDS, ROS, Unitree SDK, or hardware code.

stdout carries the command stream and nothing else; all human-readable progress
and connection logging goes to stderr and to --connection-log.
"""

from __future__ import annotations

import argparse
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


ROOT = Path(__file__).resolve().parents[2]
TELEVUER_SOURCE = ROOT / "third_party" / "xr_teleoperate" / "teleop" / "televuer" / "src"
sys.path.insert(0, str(ROOT))

from teleop.r1 import BridgeConfig, QuestCommandBridge, QuestTransportSample  # noqa: E402


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
    args = parser.parse_args()

    if args.duration_s <= 0.0 or args.frequency_hz <= 0.0:
        raise SystemExit("--duration-s and --frequency-hz must be positive.")
    if not math.isfinite(args.trigger_value_threshold) or not (
        TRIGGER_VALUE_MIN <= args.trigger_value_threshold < TRIGGER_VALUE_MAX
    ):
        raise SystemExit("--trigger-value-threshold must be finite and in [0, 10).")
    if (args.cert_file is None) != (args.key_file is None):
        raise SystemExit("Specify both --cert-file and --key-file, or neither.")
    if args.stop_file is not None and args.stop_file.expanduser().exists():
        raise SystemExit(f"Refusing to start: --stop-file already exists: {args.stop_file}")

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
            "bridge_config": asdict(bridge_config),
            "python_version": sys.version,
            "platform": platform.platform(),
            "command": [sys.executable, *sys.argv],
        },
    )

    sys.path.insert(0, str(TELEVUER_SOURCE))
    try:
        from televuer import TeleVuerWrapper
    except ImportError as exc:
        _log(sys.stderr, connection_log, {"event": "bridge_failed", "reason": f"Cannot import TeleVuer: {exc}"})
        raise SystemExit(f"Cannot import TeleVuer in this environment: {exc}") from exc

    wrapper = TeleVuerWrapper(
        use_hand_tracking=False,
        binocular=True,
        img_shape=(480, 1280),
        display_fps=args.frequency_hz,
        display_mode="pass-through",
        zmq=False,
        webrtc=False,
        cert_file=str(args.cert_file.resolve()) if args.cert_file else None,
        key_file=str(args.key_file.resolve()) if args.key_file else None,
    )
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
