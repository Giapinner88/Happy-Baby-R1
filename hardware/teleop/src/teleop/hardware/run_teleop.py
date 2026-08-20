"""Bounded R1-A5 arms/head hardware pilot.

This is deliberately not the continuous Quest runtime yet.  The default mode
subscribes to ``rt/lowstate`` and exits without creating a publisher.  The
actuating pilot requires independent interlocks.  ``arm_sdk`` is the default;
``lowcmd`` additionally requires an explicit Dev Mode confirmation.

Joint slots and gains follow the pinned Unitree R1-A5 implementation in
``third_party/xr_teleoperate_v1_6/teleop/robot_control/robot_arm.py``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from typing import Any


ARM_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
HEAD_INDICES = (29, 30)  # pitch, yaw in the pinned R1-A5 interface
WAIST_HOLD_INDICES = (12, 13)  # slot 12 unused on A5; yaw at slot 13
ALLOWED_INDICES = frozenset(ARM_INDICES + HEAD_INDICES + WAIST_HOLD_INDICES)
MOTOR_COUNT = 35
COMMAND_TOPICS = {"arm_sdk": "rt/arm_sdk", "lowcmd": "rt/lowcmd"}
STATE_TOPIC = "rt/lowstate"
CONTROL_HZ = 250.0
MAX_PILOT_DELTA_RAD = 0.05
STREAM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)
STREAM_MOTOR_INDICES = ARM_INDICES + HEAD_INDICES


def smoothstep(value: float) -> float:
    """C1-continuous interpolation on [0, 1]."""
    x = min(1.0, max(0.0, float(value)))
    return x * x * (3.0 - 2.0 * x)


def pilot_offsets(scale: float, amplitude_rad: float) -> dict[int, float]:
    """Return the only moving slots: symmetric wrists and head yaw."""
    if not math.isfinite(amplitude_rad) or not 0.0 < amplitude_rad <= MAX_PILOT_DELTA_RAD:
        raise ValueError(f"amplitude must be in (0, {MAX_PILOT_DELTA_RAD}] rad")
    s = smoothstep(scale)
    return {19: amplitude_rad * s, 26: -amplitude_rad * s, 30: amplitude_rad * s}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth10")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--execute-pilot", action="store_true")
    action.add_argument("--stream-stdin", action="store_true")
    parser.add_argument("--confirm-suspended-with-estop", action="store_true")
    parser.add_argument("--transport", choices=tuple(COMMAND_TOPICS), default="arm_sdk")
    parser.add_argument("--confirm-dev-mode", action="store_true")
    parser.add_argument("--amplitude-rad", type=float, default=0.03)
    parser.add_argument("--phase-seconds", type=float, default=1.5)
    parser.add_argument("--expected-mode-machine", type=int, default=1)
    parser.add_argument("--state-timeout-seconds", type=float, default=0.20)
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--stream-duration-s", type=float, default=120.0)
    parser.add_argument("--stream-first-input-timeout-s", type=float, default=120.0)
    parser.add_argument("--stream-input-timeout-s", type=float, default=0.35)
    parser.add_argument("--stream-max-offset-rad", type=float, default=0.15)
    parser.add_argument("--stream-max-rate-rad-s", type=float, default=0.30)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    pilot_offsets(1.0, args.amplitude_rad)
    if not 0.5 <= args.phase_seconds <= 5.0:
        raise SystemExit("--phase-seconds must be between 0.5 and 5.0")
    if not 0.05 <= args.state_timeout_seconds <= 1.0:
        raise SystemExit("--state-timeout-seconds must be between 0.05 and 1.0")
    if args.stream_stdin:
        if args.transport != "lowcmd":
            raise SystemExit("--stream-stdin currently requires --transport lowcmd")
        if args.stream_duration_s <= 0.0:
            raise SystemExit("--stream-duration-s must be positive")
        if not 5.0 <= args.stream_first_input_timeout_s <= 300.0:
            raise SystemExit("--stream-first-input-timeout-s must be between 5 and 300")
        if not 0.1 <= args.stream_input_timeout_s <= 1.0:
            raise SystemExit("--stream-input-timeout-s must be between 0.1 and 1.0")
        if not 0.02 <= args.stream_max_offset_rad <= 0.30:
            raise SystemExit("--stream-max-offset-rad must be between 0.02 and 0.30")
        if not 0.05 <= args.stream_max_rate_rad_s <= 0.50:
            raise SystemExit("--stream-max-rate-rad-s must be between 0.05 and 0.50")
    if args.execute_pilot or args.stream_stdin:
        if os.environ.get("HB_TELEOP_ALLOW_MOTOR_WRITE", "0") != "1":
            raise SystemExit("motor write denied: set HB_TELEOP_ALLOW_MOTOR_WRITE=1 explicitly")
        if not args.confirm_suspended_with_estop:
            raise SystemExit("motor write denied: --confirm-suspended-with-estop is required")
        if args.transport == "lowcmd" and not args.confirm_dev_mode:
            raise SystemExit("lowcmd denied: --confirm-dev-mode is required")


def _wait_for_state(subscriber: Any, timeout_s: float = 5.0) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = subscriber.Read()
        if message is not None:
            return message
        time.sleep(0.01)
    raise TimeoutError(f"no {STATE_TOPIC} sample within {timeout_s:.1f}s")


def _configure_selected_command(message: Any, state: Any, transport: str) -> tuple[int, ...]:
    arm_set = set(ARM_INDICES)
    head_set = set(HEAD_INDICES)
    # A LowCmd is a complete message. Non-selected slots are explicitly passive:
    # measured q, zero gain, zero feed-forward torque, and disabled mode.
    for index in range(MOTOR_COUNT):
        command = message.motor_cmd[index]
        command.mode = 0
        command.q = float(state.motor_state[index].q)
        command.dq = 0.0
        command.tau = 0.0
        command.kp = 0.0
        command.kd = 0.0
    selected = tuple(sorted(ALLOWED_INDICES if transport == "arm_sdk" else arm_set | head_set))
    for index in selected:
        command = message.motor_cmd[index]
        command.mode = 1
        command.q = float(state.motor_state[index].q)
        command.dq = 0.0
        command.tau = 0.0
        if index in head_set:
            command.kp, command.kd = 15.0, 1.0
        elif index in arm_set:
            if index in (19, 26):
                command.kp, command.kd = 30.0, 2.0
            elif index in (17, 18, 24, 25):
                command.kp, command.kd = 40.0, 2.0
            else:
                command.kp, command.kd = 50.0, 2.0
        else:
            command.kp, command.kd = 50.0, 3.0
    return selected


def _write(publisher: Any, crc: Any, message: Any) -> None:
    message.crc = crc.Crc(message)
    publisher.Write(message)


def _stdin_reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def _run_stream(
    args: argparse.Namespace,
    subscriber: Any,
    state: Any,
    channel_publisher: Any,
    low_cmd_type: Any,
    default_low_cmd: Any,
    crc_type: Any,
) -> int:
    run_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "_r1_arms_head_stream"
    run_dir = args.log_dir / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    log_path = run_dir / "samples.jsonl"
    metadata = {
        "run_id": run_id,
        "status": "waiting_for_input",
        "transport": "lowcmd",
        "command_topic": COMMAND_TOPICS["lowcmd"],
        "scope": "arms_head",
        "joint_names": STREAM_JOINT_NAMES,
        "motor_indices": STREAM_MOTOR_INDICES,
        "max_offset_from_start_rad": args.stream_max_offset_rad,
        "max_rate_rad_s": args.stream_max_rate_rad_s,
        "input_timeout_s": args.stream_input_timeout_s,
        "duration_s": args.stream_duration_s,
        "control_hz": CONTROL_HZ,
    }
    metadata_path = run_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(lines,), daemon=True).start()
    source_zero: list[float] | None = None
    latest_source: list[float] | None = None
    latest_sequence = -1
    input_closed = False
    first_deadline = time.monotonic() + args.stream_first_input_timeout_s
    last_input_at = 0.0
    while time.monotonic() < first_deadline and source_zero is None:
        try:
            line = lines.get(timeout=0.1)
        except queue.Empty:
            continue
        if line is None:
            input_closed = True
            break
        try:
            payload = json.loads(line)
            names = tuple(str(value) for value in payload["joint_names"])
            positions = [float(value) for value in payload["positions_rad"]]
            sequence = int(payload["sequence_id"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            print(f"[WARN] invalid target stream line: {exc}", file=sys.stderr)
            continue
        if names != STREAM_JOINT_NAMES or len(positions) != len(STREAM_MOTOR_INDICES):
            print("[WARN] rejected target with unexpected joint schema", file=sys.stderr)
            continue
        if not all(math.isfinite(value) for value in positions):
            print("[WARN] rejected non-finite target", file=sys.stderr)
            continue
        source_zero = positions
        latest_source = positions
        latest_sequence = sequence
        last_input_at = time.monotonic()
    if source_zero is None:
        metadata["status"] = "no_input"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print("[SAFE] no valid target received; no publisher was created")
        return 2

    publisher = channel_publisher(COMMAND_TOPICS["lowcmd"], low_cmd_type)
    publisher.Init()
    command = default_low_cmd()
    command.mode_machine = int(state.mode_machine)
    command.mode_pr = 0
    selected = _configure_selected_command(command, state, "lowcmd")
    if selected != tuple(sorted(STREAM_MOTOR_INDICES)):
        raise RuntimeError("internal arms/head motor ordering mismatch")
    base_gains = {i: (float(command.motor_cmd[i].kp), float(command.motor_cmd[i].kd)) for i in selected}
    start_q = [float(state.motor_state[index].q) for index in STREAM_MOTOR_INDICES]
    commanded = start_q.copy()
    crc = crc_type()
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    dt = 1.0 / CONTROL_HZ
    last_state_at = time.monotonic()
    latest_state = state

    def set_authority(percent: float) -> None:
        scale = min(1.0, max(0.0, percent / 100.0))
        for index in selected:
            kp, kd = base_gains[index]
            command.motor_cmd[index].kp = kp * scale
            command.motor_cmd[index].kd = kd * scale

    def refresh_state() -> None:
        nonlocal latest_state, last_state_at
        observed = subscriber.Read()
        if observed is not None:
            latest_state = observed
            last_state_at = time.monotonic()
        if time.monotonic() - last_state_at > args.state_timeout_seconds:
            raise TimeoutError("lowstate watchdog expired")

    status = "completed"
    stop_reason = "duration_elapsed"
    started_at = time.monotonic()
    sample_index = 0
    try:
        acquire_steps = round(1.0 * CONTROL_HZ)
        for step in range(acquire_steps):
            refresh_state()
            set_authority(100.0 * (step + 1) / acquire_steps)
            _write(publisher, crc, command)
            time.sleep(dt)
        metadata["status"] = "running"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        while time.monotonic() - started_at < args.stream_duration_s:
            if stop_requested:
                stop_reason = "signal"
                break
            while True:
                try:
                    line = lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    input_closed = True
                    break
                try:
                    payload = json.loads(line)
                    names = tuple(str(value) for value in payload["joint_names"])
                    positions = [float(value) for value in payload["positions_rad"]]
                    sequence = int(payload["sequence_id"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    continue
                if (
                    names == STREAM_JOINT_NAMES
                    and len(positions) == len(STREAM_MOTOR_INDICES)
                    and all(math.isfinite(value) for value in positions)
                    and sequence > latest_sequence
                ):
                    latest_source = positions
                    latest_sequence = sequence
                    last_input_at = time.monotonic()
            if input_closed:
                stop_reason = "stream_closed"
                break
            if time.monotonic() - last_input_at > args.stream_input_timeout_s:
                stop_reason = "input_watchdog"
                break
            assert latest_source is not None
            max_step = args.stream_max_rate_rad_s * dt
            for offset, motor_index in enumerate(STREAM_MOTOR_INDICES):
                relative = latest_source[offset] - source_zero[offset]
                relative = min(args.stream_max_offset_rad, max(-args.stream_max_offset_rad, relative))
                desired = start_q[offset] + relative
                delta = min(max_step, max(-max_step, desired - commanded[offset]))
                commanded[offset] += delta
                command.motor_cmd[motor_index].q = commanded[offset]
            refresh_state()
            _write(publisher, crc, command)
            if sample_index % 25 == 0:
                record = {
                    "monotonic_s": time.monotonic(),
                    "sequence_id": latest_sequence,
                    "target_q": commanded,
                    "observed_q": [float(latest_state.motor_state[i].q) for i in STREAM_MOTOR_INDICES],
                }
                with log_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, separators=(",", ":")) + "\n")
            sample_index += 1
            time.sleep(dt)
    except Exception:
        status = "failed"
        stop_reason = "exception"
        raise
    finally:
        release_steps = round(0.5 * CONTROL_HZ)
        for step in range(release_steps):
            set_authority(100.0 * (1.0 - (step + 1) / release_steps))
            _write(publisher, crc, command)
            time.sleep(dt)
        for index in selected:
            command.motor_cmd[index].mode = 0
            command.motor_cmd[index].kp = 0.0
            command.motor_cmd[index].kd = 0.0
        for _ in range(25):
            _write(publisher, crc, command)
            time.sleep(dt)
        metadata.update(
            status=status,
            stop_reason=stop_reason,
            last_sequence_id=latest_sequence,
            finished_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        )
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[{status.upper()}] stop={stop_reason} evidence: {run_dir}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _validate_args(args)

    # Imports stay inside main so static checks and --help do not initialize DDS.
    from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # type: ignore[import-not-found]
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # type: ignore[import-not-found]
    from unitree_sdk2py.utils.crc import CRC  # type: ignore[import-not-found]

    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init()
    state = _wait_for_state(subscriber)
    if int(state.mode_machine) != args.expected_mode_machine:
        raise SystemExit(
            f"mode_machine={state.mode_machine}, expected {args.expected_mode_machine}; refusing pilot"
        )
    print(f"[OK] {STATE_TOPIC}: mode_machine={state.mode_machine}, motors={len(state.motor_state)}")

    if not args.execute_pilot:
        if args.stream_stdin:
            return _run_stream(
                args, subscriber, state, ChannelPublisher, LowCmd_,
                unitree_hg_msg_dds__LowCmd_, CRC,
            )
        print("[SAFE] read-only check complete; no publisher was created")
        return 0

    run_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "_r1_arms_head_pilot"
    run_dir = args.log_dir / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    log_path = run_dir / "samples.jsonl"
    metadata = {
        "run_id": run_id,
        "command_topic": COMMAND_TOPICS[args.transport],
        "transport": args.transport,
        "state_topic": STATE_TOPIC,
        "interface": args.interface,
        "scope": "arms_head",
        "arm_indices": ARM_INDICES,
        "head_indices_pitch_yaw": HEAD_INDICES,
        "waist_hold_indices": WAIST_HOLD_INDICES if args.transport == "arm_sdk" else (),
        "amplitude_rad": args.amplitude_rad,
        "phase_seconds": args.phase_seconds,
        "control_hz": CONTROL_HZ,
        "status": "running",
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    publisher = ChannelPublisher(COMMAND_TOPICS[args.transport], LowCmd_)
    publisher.Init()
    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_machine = int(state.mode_machine)
    command.mode_pr = 0
    selected_indices = _configure_selected_command(command, state, args.transport)
    base_gains = {
        index: (float(command.motor_cmd[index].kp), float(command.motor_cmd[index].kd))
        for index in selected_indices
    }
    start_q = {index: float(state.motor_state[index].q) for index in selected_indices}
    crc = CRC()
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    dt = 1.0 / CONTROL_HZ
    last_state_at = time.monotonic()
    latest_state = state

    def cycle(scale: float, weight: int, sample_index: int) -> None:
        nonlocal latest_state, last_state_at
        observed = subscriber.Read()
        if observed is not None:
            latest_state = observed
            last_state_at = time.monotonic()
        if time.monotonic() - last_state_at > args.state_timeout_seconds:
            raise TimeoutError("lowstate watchdog expired")
        authority = max(0, min(100, int(weight)))
        if args.transport == "arm_sdk":
            command.mode_pr = authority
        else:
            command.mode_pr = 0
            for index in selected_indices:
                kp, kd = base_gains[index]
                command.motor_cmd[index].kp = kp * authority / 100.0
                command.motor_cmd[index].kd = kd * authority / 100.0
        for index, delta in pilot_offsets(scale, args.amplitude_rad).items():
            command.motor_cmd[index].q = start_q[index] + delta
        _write(publisher, crc, command)
        if sample_index % 25 == 0:
            record = {
                "monotonic_s": time.monotonic(),
                "authority_percent": authority,
                "observed_mode_pr": int(latest_state.mode_pr),
                "scale": scale,
                "target_q": {str(i): command.motor_cmd[i].q for i in selected_indices},
                "observed_q": {str(i): float(latest_state.motor_state[i].q) for i in selected_indices},
            }
            with log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")

    status = "completed"
    try:
        phase_steps = max(1, round(args.phase_seconds * CONTROL_HZ))
        # Acquire overlay while holding every selected joint at measured state.
        for step in range(phase_steps):
            if stop_requested:
                raise KeyboardInterrupt
            cycle(0.0, round(100 * (step + 1) / phase_steps), step)
            time.sleep(dt)
        # Smooth out, then smooth back to the exact captured posture.
        for direction in (1, -1):
            for step in range(phase_steps):
                if stop_requested:
                    raise KeyboardInterrupt
                fraction = (step + 1) / phase_steps
                scale = fraction if direction == 1 else 1.0 - fraction
                cycle(scale, 100, step)
                time.sleep(dt)
    except KeyboardInterrupt:
        status = "aborted"
        print("[STOP] operator/watchdog stop requested", file=sys.stderr)
    except Exception:
        status = "failed"
        raise
    finally:
        # Return targets to the captured posture and release the arm overlay.
        for index in selected_indices:
            command.motor_cmd[index].q = start_q[index]
        release_steps = max(1, round(args.phase_seconds * CONTROL_HZ))
        for step in range(release_steps):
            cycle(0.0, round(100 * (1.0 - (step + 1) / release_steps)), step)
            time.sleep(dt)
        if args.transport == "lowcmd":
            for index in selected_indices:
                command.motor_cmd[index].mode = 0
                command.motor_cmd[index].kp = 0.0
                command.motor_cmd[index].kd = 0.0
            for _ in range(25):
                _write(publisher, crc, command)
                time.sleep(dt)
        metadata["status"] = status
        metadata["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[{status.upper()}] evidence: {run_dir}")
    return 0 if status == "completed" else 130


if __name__ == "__main__":
    raise SystemExit(main())
