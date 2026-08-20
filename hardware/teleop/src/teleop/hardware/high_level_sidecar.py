"""Forward validated arms/head targets to the sole high-level lowcmd owner.

This process subscribes to ``rt/lowstate`` for mode/watchdog/evidence only.  It
creates no DDS publisher. Motor authority remains inside ``hb_high_level``;
the only output here is a loopback UDP packet consumed by that same process.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import socket
import struct
import sys
import threading
import time
from typing import Any

STATE_TOPIC = "rt/lowstate"
TELEOP_MAGIC = 0x314C5455
PACKET = struct.Struct("<IIBBBB12f")
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)
MOTOR_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5560)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--first-input-timeout-s", type=float, default=120.0)
    parser.add_argument("--input-timeout-s", type=float, default=0.75)
    parser.add_argument("--state-timeout-s", type=float, default=0.20)
    parser.add_argument("--send-hz", type=float, default=100.0)
    parser.add_argument("--max-offset-rad", type=float, default=0.15)
    parser.add_argument("--expected-mode-machine", type=int, default=1)
    parser.add_argument("--confirm-suspended-with-estop", action="store_true")
    parser.add_argument("--confirm-dev-mode", action="store_true")
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if os.environ.get("HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP", "0") != "1":
        raise SystemExit("high-level teleop denied: set HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1")
    if not args.confirm_suspended_with_estop:
        raise SystemExit("high-level teleop denied: --confirm-suspended-with-estop is required")
    if not args.confirm_dev_mode:
        raise SystemExit("high-level teleop denied: --confirm-dev-mode is required")
    if args.duration_s <= 0 or not 5 <= args.first_input_timeout_s <= 300:
        raise SystemExit("invalid duration or first-input timeout")
    if not 0.1 <= args.input_timeout_s <= 1.0:
        raise SystemExit("--input-timeout-s must be in [0.1, 1.0]")
    if not 0.05 <= args.state_timeout_s <= 1.0:
        raise SystemExit("--state-timeout-s must be in [0.05, 1.0]")
    if not 10 <= args.send_hz <= 250 or not 0.02 <= args.max_offset_rad <= 0.30:
        raise SystemExit("invalid send rate or session envelope")


def parse_target(line: str, previous_sequence: int) -> tuple[int, list[float]] | None:
    try:
        payload = json.loads(line)
        names = tuple(str(value) for value in payload["joint_names"])
        positions = [float(value) for value in payload["positions_rad"]]
        sequence = int(payload["sequence_id"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if names != JOINT_NAMES or len(positions) != len(MOTOR_INDICES):
        return None
    if sequence <= previous_sequence or not all(math.isfinite(value) for value in positions):
        return None
    return sequence, positions


def encode_target(sequence: int, positions: list[float]) -> bytes:
    """Encode arms[10], head pitch/yaw stream as UTL1 arms, yaw, pitch."""
    if len(positions) != 12:
        raise ValueError("expected 12 arms/head positions")
    arms = positions[:10]
    head_pitch, head_yaw = positions[10:]
    return PACKET.pack(
        TELEOP_MAGIC, sequence, 1, 1, 1, 0, *arms, head_yaw, head_pitch
    )


def encode_stop(sequence: int) -> bytes:
    return PACKET.pack(TELEOP_MAGIC, sequence, 0, 0, 0, 0, *([0.0] * 12))


def stdin_reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def wait_for_state(subscriber: Any, timeout_s: float = 5.0) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = subscriber.Read()
        if state is not None:
            return state
        time.sleep(0.01)
    raise TimeoutError(f"no {STATE_TOPIC} sample within {timeout_s:.1f}s")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    validate_args(args)
    if args.udp_host not in {"127.0.0.1", "localhost"}:
        raise SystemExit("high-level teleop transport is restricted to loopback")
    if not 1024 <= args.udp_port <= 65535:
        raise SystemExit("invalid UDP port")

    # Imports stay here so --help and static tests never initialize DDS.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init()
    state = wait_for_state(subscriber)
    if int(state.mode_machine) != args.expected_mode_machine:
        raise SystemExit(
            f"mode_machine={state.mode_machine}, expected {args.expected_mode_machine}; refusing sidecar"
        )

    run_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "_r1_high_level_teleop"
    run_dir = args.log_dir / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    metadata_path = run_dir / "metadata.json"
    samples_path = run_dir / "samples.jsonl"
    metadata = {
        "run_id": run_id,
        "status": "waiting_for_input",
        "transport": "loopback_udp_high_level_owner",
        "udp_endpoint": f"{args.udp_host}:{args.udp_port}",
        "state_topic": STATE_TOPIC,
        "joint_names": JOINT_NAMES,
        "motor_indices": MOTOR_INDICES,
        "max_offset_from_start_rad": args.max_offset_rad,
        "send_hz": args.send_hz,
        "input_timeout_s": args.input_timeout_s,
        "state_timeout_s": args.state_timeout_s,
        "duration_s": args.duration_s,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=stdin_reader, args=(lines,), daemon=True).start()
    source_zero: list[float] | None = None
    latest_source: list[float] | None = None
    upstream_sequence = -1
    local_sequence = 0
    last_input_at = 0.0
    input_closed = False
    first_deadline = time.monotonic() + args.first_input_timeout_s
    while time.monotonic() < first_deadline and source_zero is None:
        try:
            line = lines.get(timeout=0.1)
        except queue.Empty:
            continue
        if line is None:
            input_closed = True
            break
        parsed = parse_target(line, upstream_sequence)
        if parsed is None:
            continue
        upstream_sequence, latest_source = parsed
        source_zero = latest_source.copy()
        last_input_at = time.monotonic()

    if source_zero is None:
        metadata.update(status="no_input", stop_reason="stream_closed" if input_closed else "first_input_timeout")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print("[SAFE] no valid target; high-level stayed in Damping")
        return 2

    start_q = [float(state.motor_state[index].q) for index in MOTOR_INDICES]
    latest_state = state
    last_state_at = time.monotonic()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.connect((args.udp_host, args.udp_port))
    status = "completed"
    stop_reason = "duration_elapsed"
    started_at = time.monotonic()
    period = 1.0 / args.send_hz
    sample_index = 0
    try:
        metadata["status"] = "running"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        while time.monotonic() - started_at < args.duration_s:
            loop_start = time.monotonic()
            while True:
                try:
                    line = lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    input_closed = True
                    break
                parsed = parse_target(line, upstream_sequence)
                if parsed is not None:
                    upstream_sequence, latest_source = parsed
                    last_input_at = time.monotonic()
            if input_closed:
                stop_reason = "stream_closed"
                break
            if time.monotonic() - last_input_at > args.input_timeout_s:
                stop_reason = "input_watchdog"
                break
            observed = subscriber.Read()
            if observed is not None:
                latest_state = observed
                last_state_at = time.monotonic()
            if time.monotonic() - last_state_at > args.state_timeout_s:
                stop_reason = "lowstate_watchdog"
                status = "failed"
                break
            if int(latest_state.mode_machine) != args.expected_mode_machine:
                stop_reason = "mode_machine_changed"
                status = "failed"
                break
            assert latest_source is not None
            local_sequence += 1
            desired = [
                start + min(args.max_offset_rad, max(-args.max_offset_rad, source - zero))
                for start, source, zero in zip(start_q, latest_source, source_zero)
            ]
            client.send(encode_target(local_sequence, desired))
            if sample_index % max(1, round(args.send_hz / 10.0)) == 0:
                record = {
                    "monotonic_s": time.monotonic(),
                    "upstream_sequence_id": upstream_sequence,
                    "ipc_sequence_id": local_sequence,
                    "target_q": desired,
                    "observed_q": [float(latest_state.motor_state[index].q) for index in MOTOR_INDICES],
                }
                with samples_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, separators=(",", ":")) + "\n")
            sample_index += 1
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except Exception:
        status = "failed"
        stop_reason = "exception"
        raise
    finally:
        local_sequence += 1
        try:
            client.send(encode_stop(local_sequence))
        except OSError:
            pass
        client.close()
        metadata.update(
            status=status,
            stop_reason=stop_reason,
            last_ipc_sequence_id=local_sequence,
            finished_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        )
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[{status.upper()}] stop={stop_reason} evidence: {run_dir}")
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
