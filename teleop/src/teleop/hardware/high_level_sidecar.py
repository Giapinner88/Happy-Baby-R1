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
from typing import Any, NamedTuple

STATE_TOPIC = "rt/lowstate"
TELEOP_MAGIC = 0x314C5455
PACKET = struct.Struct("<IIBBBB12f")
# Unitree R1 SDK: idl 29 = head_pitch, 30 = head_yaw. UTL1 mang thu tu ngu
# nghia (yaw, pitch), vi vay hai index cuoi phai doc theo thu tu (30, 29).
# ``teleop/r1`` giu quy uoc noi bo
# cua workspace nguon (co cho dung [pitch, yaw]) va bi sync_from_workspace.sh ghi
# de, nen moi quy doi sang chuan controller nam trong ``hardware/`` — dung cho
# bien gioi, vi day cung la noi dong goi UTL1.
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_yaw_joint", "head_pitch_joint",
)
ARM_MOTOR_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
HEAD_YAW_IDL = 30
HEAD_PITCH_IDL = 29
MOTOR_INDICES = ARM_MOTOR_INDICES + (HEAD_YAW_IDL, HEAD_PITCH_IDL)


class H4Target(NamedTuple):
    sequence_id: int
    upstream_sequence_id: int
    enabled: bool
    positions_rad: list[float] | None


class SourceSequenceWatchdog:
    """Only an advancing Quest sequence refreshes the deadman lease."""

    def __init__(self, timeout_s: float) -> None:
        if not 0.0 < timeout_s <= 1.0:
            raise ValueError("source timeout must be in (0, 1] seconds")
        self.timeout_s = timeout_s
        self.last_sequence = -1
        self.last_advance_at = float("-inf")

    def observe(self, sequence: int, now_s: float) -> bool:
        if sequence > self.last_sequence:
            self.last_sequence = sequence
            self.last_advance_at = now_s
        return sequence >= self.last_sequence and now_s - self.last_advance_at <= self.timeout_s


class H4SourceSession:
    """Translate a deadman-aware source into encoder-relative joint targets."""

    def __init__(self, max_offset_rad: float | None, source_timeout_s: float) -> None:
        self.max_offset_rad = max_offset_rad
        self.watchdog = SourceSequenceWatchdog(source_timeout_s)
        self.source_zero: list[float] | None = None
        self.robot_zero: list[float] | None = None
        self.last_desired: list[float] | None = None
        self.last_used_source_sequence = -1
        self.needs_release = False

    def consume(self, target: H4Target, observed_q: list[float], now_s: float) -> list[float] | None:
        if not target.enabled:
            self.source_zero = self.robot_zero = self.last_desired = None
            self.needs_release = False
            return None
        if self.needs_release or target.positions_rad is None:
            return None
        if len(observed_q) != len(MOTOR_INDICES) or not all(math.isfinite(q) for q in observed_q):
            self.needs_release = True
            return None
        if not self.watchdog.observe(target.upstream_sequence_id, now_s):
            self.needs_release = True
            self.source_zero = self.robot_zero = self.last_desired = None
            return None
        if target.upstream_sequence_id == self.last_used_source_sequence:
            return self.last_desired
        self.last_used_source_sequence = target.upstream_sequence_id
        if self.source_zero is None:
            self.source_zero = target.positions_rad.copy()
            self.robot_zero = observed_q.copy()
        assert self.robot_zero is not None
        if self.max_offset_rad is None:
            self.last_desired = list(target.positions_rad)
        else:
            self.last_desired = [
                base + min(self.max_offset_rad, max(-self.max_offset_rad, source - zero))
                for base, source, zero in zip(self.robot_zero, target.positions_rad, self.source_zero)
            ]
        return self.last_desired


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream-contract", choices=("legacy", "h4"), default="legacy")
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5560)
    parser.add_argument("--duration-s", type=float, default=0.0,
                        help="Session duration in seconds; 0 keeps the runtime alive indefinitely.")
    parser.add_argument("--first-input-timeout-s", type=float, default=120.0)
    parser.add_argument("--input-timeout-s", type=float, default=0.75)
    parser.add_argument("--source-timeout-s", type=float, default=0.30)
    parser.add_argument("--state-timeout-s", type=float, default=0.20)
    parser.add_argument("--send-hz", type=float, default=100.0)
    parser.add_argument("--max-offset-rad", type=float, default=0.15)
    parser.add_argument("--head-yaw-max-rad", type=float, default=0.60)
    parser.add_argument("--head-pitch-max-rad", type=float, default=0.35)
    parser.add_argument(
        "--joint-limits-only", action="store_true",
        help=(
            "Trust the upstream URDF-clamped target positions and skip the sidecar "
            "offset/head envelopes. Deadman, watchdogs and finite-value checks remain active."
        ),
    )
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
    if args.duration_s < 0 or not 5 <= args.first_input_timeout_s <= 300:
        raise SystemExit("duration must be non-negative (0=infinite) and first-input timeout must be 5..300 s")
    if not 0.1 <= args.input_timeout_s <= 1.0:
        raise SystemExit("--input-timeout-s must be in [0.1, 1.0]")
    if not 0.1 <= args.source_timeout_s <= args.input_timeout_s:
        raise SystemExit("--source-timeout-s must be in [0.1, input-timeout-s]")
    if not 0.05 <= args.state_timeout_s <= 1.0:
        raise SystemExit("--state-timeout-s must be in [0.05, 1.0]")
    if not 10 <= args.send_hz <= 250:
        raise SystemExit("invalid send rate")
    # Tran cu 0.30 rad (17 deg) qua chat de kiem chieu khop: tay chi "dao dong quanh vi
    # tri init" nen mot truc bi dao dau cung khong lo ra.  Noi len 3.2 rad = gan het dai
    # khop.  Day KHONG phai bo an toan, vi ba lop cung hon van chan trong moi truong hop:
    #   1. IK clamp theo gioi han khop trong R1.urdf -> muc tieu khong bao gio vuot dai that
    #   2. TeleopReceiver::Valid() bo nguyen goi neu |arm_q| > 3.5 rad
    #   3. LowCmdSender slew theo teleop_max_rate_rad_s = 0.30 rad/s (~17 deg/giay)
    # Lop 3 moi la thu giu an toan that su: du lenh nhay bao xa, tay van bo tu tu.
    if not 0.02 <= args.max_offset_rad <= 3.2:
        raise SystemExit("invalid session envelope (0.02..3.2 rad)")
    if not 0.05 <= args.head_yaw_max_rad <= 1.0:
        raise SystemExit("--head-yaw-max-rad must be in [0.05, 1.0]")
    # Tran nay phuc vu HAI viec khac nhau: (a) gate "dau co o giua khong" luc khoi
    # dong, va (b) clamp vi tri dau khi chay (xem desired[-1] ben duoi).  O ZERO
    # TORQUE dau limp ru xuong ~0.62 rad nen tran 0.6 cu lam gate khong bao gio
    # qua duoc.  Noi len 1.0, van duoi tran C++ teleop_head_pitch_max (1.5).
    if not 0.05 <= args.head_pitch_max_rad <= 1.0:
        raise SystemExit("--head-pitch-max-rad must be in [0.05, 1.0]")


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


def parse_h4_target(line: str, previous_sequence: int) -> H4Target | None:
    """H4 refuses the legacy stream, which never carried a real deadman bit."""
    try:
        payload = json.loads(line)
        sequence = payload["sequence_id"]
        upstream_sequence = payload["upstream_sequence_id"]
        enabled = payload["enabled"]
        if type(sequence) is not int or type(upstream_sequence) is not int or type(enabled) is not bool:
            return None
        if sequence <= previous_sequence or upstream_sequence < 0:
            return None
        if not enabled:
            return H4Target(sequence, upstream_sequence, False, None)
        names = tuple(str(value) for value in payload["joint_names"])
        positions = [float(value) for value in payload["positions_rad"]]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if names != JOINT_NAMES or len(positions) != len(MOTOR_INDICES):
        return None
    if not all(math.isfinite(value) for value in positions):
        return None
    return H4Target(sequence, upstream_sequence, True, positions)


def encode_target(sequence: int, positions: list[float], *, explicit_deadman: bool = False) -> bytes:
    """Encode arms[10] + head (yaw, pitch); stream order already matches UTL1."""
    if len(positions) != 12:
        raise ValueError("expected 12 arms/head positions")
    arms = positions[:10]
    head_yaw, head_pitch = positions[10:]
    return PACKET.pack(
        TELEOP_MAGIC, sequence, 1, 1, 1, int(explicit_deadman), *arms, head_yaw, head_pitch
    )


def encode_stop(sequence: int, *, explicit_deadman: bool = False) -> bytes:
    return PACKET.pack(TELEOP_MAGIC, sequence, 0, 0, 0, int(explicit_deadman), *([0.0] * 12))


def encode_h4_frame(
    session: H4SourceSession, target: H4Target, observed_q: list[float], now_s: float,
    head_yaw_max: float | None = None, head_pitch_max: float | None = None,
) -> bytes:
    """Encode only targets backed by a live, explicit upstream deadman."""
    desired = session.consume(target, observed_q, now_s)
    if desired is None:
        return encode_stop(target.sequence_id, explicit_deadman=True)
    if head_yaw_max is not None:
        desired[-2] = min(head_yaw_max, max(-head_yaw_max, desired[-2]))
    if head_pitch_max is not None:
        desired[-1] = min(head_pitch_max, max(-head_pitch_max, desired[-1]))
    return encode_target(target.sequence_id, desired, explicit_deadman=True)


class H4StreamPump:
    """Sequence the local UTL1 stream without extending a stale Quest lease."""

    def __init__(self, max_offset_rad: float | None, source_timeout_s: float,
                 input_timeout_s: float, head_yaw_max: float | None = None,
                 head_pitch_max: float | None = None) -> None:
        self.session = H4SourceSession(max_offset_rad, source_timeout_s)
        self.input_timeout_s = input_timeout_s
        self.latest: H4Target | None = None
        self.last_input_at = float("-inf")
        self.packet_sequence = 0
        self.source_sequence = -1
        self.head_yaw_max = head_yaw_max
        self.head_pitch_max = head_pitch_max
        self.release_stop_pending = False

    def accept(self, line: str, now_s: float) -> bool:
        parsed = parse_h4_target(line, self.source_sequence)
        if parsed is None:
            return False
        self.source_sequence = parsed.sequence_id
        self.latest = parsed
        self.last_input_at = now_s
        if not parsed.enabled:
            self.session.consume(parsed, [], now_s)
            self.release_stop_pending = True
        return True

    def frame(self, observed_q: list[float], now_s: float) -> bytes:
        self.packet_sequence += 1
        if self.release_stop_pending:
            self.release_stop_pending = False
            return encode_stop(self.packet_sequence, explicit_deadman=True)
        if self.latest is None or now_s - self.last_input_at > self.input_timeout_s:
            if self.latest is not None:
                self.session.needs_release = True
            return encode_stop(self.packet_sequence, explicit_deadman=True)
        target = self.latest._replace(sequence_id=self.packet_sequence)
        return encode_h4_frame(self.session, target, observed_q, now_s,
                               self.head_yaw_max, self.head_pitch_max)


def stdin_reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


class LatestLowState:
    """Callback mailbox; repeated reads cannot extend a stale DDS sample."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: Any = None
        self._received_at = 0.0

    def receive(self, state: Any) -> None:
        with self._lock:
            self._state = state
            self._received_at = time.monotonic()

    def snapshot(self) -> tuple[Any, float]:
        with self._lock:
            return self._state, self._received_at


def wait_for_state(states: LatestLowState, timeout_s: float = 5.0,
                   max_age_s: float = 0.2) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state, received_at = states.snapshot()
        if state is not None and time.monotonic() - received_at <= max_age_s:
            return state
        time.sleep(0.01)
    raise TimeoutError(f"no {STATE_TOPIC} sample within {timeout_s:.1f}s")


def run_h4_stream(args: argparse.Namespace, states: LatestLowState,
                  lines: "queue.Queue[str | None]", metadata: dict[str, Any],
                  metadata_path: Path, samples_path: Path) -> int:
    """The opt-in H4 sender; never publishes DDS and always ends with UTL1 STOP."""
    # Keep direct callers and older test fixtures source-compatible when the
    # optional joint-limits-only flag is absent from their Namespace.
    joint_limits_only = getattr(args, "joint_limits_only", False)
    offset_limit = None if joint_limits_only else args.max_offset_rad
    head_yaw_limit = None if joint_limits_only else args.head_yaw_max_rad
    head_pitch_limit = None if joint_limits_only else args.head_pitch_max_rad
    pump = H4StreamPump(offset_limit, args.source_timeout_s,
                        args.input_timeout_s, head_yaw_limit,
                        head_pitch_limit)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.connect((args.udp_host, args.udp_port))
    period = 1.0 / args.send_hz
    first_deadline = time.monotonic() + args.first_input_timeout_s
    started_at: float | None = None
    sample_index = 0
    stop_reason = "duration_elapsed"
    status = "completed"
    try:
        client.send(pump.frame([], time.monotonic()))  # STOP before any target
        while True:
            loop_start = time.monotonic()
            while True:
                try:
                    line = lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    # The upstream mapper intentionally closes its stdout after
                    # the release debounce.  It is a normal deadman release,
                    # not an H4 transport failure; the finally block below still
                    # emits an explicit UTL1 STOP with pad=1.
                    stop_reason, status = "stream_closed", "completed"
                    break
                if pump.accept(line, time.monotonic()) and started_at is None:
                    started_at = time.monotonic()
                    metadata["status"] = "running"
                    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            if status == "failed":
                break
            now_s = time.monotonic()
            if started_at is None and now_s >= first_deadline:
                stop_reason, status = "first_input_timeout", "no_input"
                break
            if started_at is not None and args.duration_s > 0 and now_s - started_at >= args.duration_s:
                break
            state, received_at = states.snapshot()
            if state is None or now_s - received_at > args.state_timeout_s:
                stop_reason, status = "lowstate_watchdog", "failed"
                break
            if int(state.mode_machine) != args.expected_mode_machine:
                stop_reason, status = "mode_machine_changed", "failed"
                break
            observed_q = [float(state.motor_state[index].q) for index in MOTOR_INDICES]
            packet = pump.frame(observed_q, now_s)
            client.send(packet)
            if sample_index % max(1, round(args.send_hz / 10.0)) == 0:
                fields = PACKET.unpack(packet)
                record = {
                    "monotonic_s": now_s,
                    "source_sequence_id": pump.source_sequence,
                    "upstream_sequence_id": pump.latest.upstream_sequence_id if pump.latest else None,
                    "ipc_sequence_id": pump.packet_sequence,
                    "enabled": bool(fields[2]),
                    "observed_q": observed_q,
                    "target_q": list(fields[6:]) if fields[2] else None,
                }
                with samples_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, separators=(",", ":")) + "\n")
            sample_index += 1
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except Exception:
        status, stop_reason = "failed", "exception"
        raise
    finally:
        pump.packet_sequence += 1
        try:
            client.send(encode_stop(pump.packet_sequence, explicit_deadman=True))
        except OSError:
            pass
        client.close()
        metadata.update(status=status, stop_reason=stop_reason,
                        last_ipc_sequence_id=pump.packet_sequence,
                        finished_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[{status.upper()}] stop={stop_reason} evidence: {metadata_path.parent}")
    return 0 if status == "completed" else 2 if status == "no_input" else 1


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
    states = LatestLowState()
    subscriber.Init(states.receive)
    state = wait_for_state(states, max_age_s=args.state_timeout_s)
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
        "max_offset_from_start_rad": None if args.joint_limits_only else args.max_offset_rad,
        "position_limit_mode": "joint_limits_only" if args.joint_limits_only else "session_offset",
        "send_hz": args.send_hz,
        "input_timeout_s": args.input_timeout_s,
        "state_timeout_s": args.state_timeout_s,
        "duration_s": args.duration_s,
        "stream_contract": args.stream_contract,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=stdin_reader, args=(lines,), daemon=True).start()
    if args.stream_contract == "h4":
        return run_h4_stream(args, states, lines, metadata, metadata_path, samples_path)
    source_zero: list[float] | None = None
    latest_source: list[float] | None = None
    upstream_sequence = -1
    valid_target_count = 0
    rejected_target_count = 0
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
            rejected_target_count += 1
            continue
        upstream_sequence, latest_source = parsed
        valid_target_count += 1
        source_zero = latest_source.copy()
        last_input_at = time.monotonic()

    if source_zero is None:
        metadata.update(status="no_input", stop_reason="stream_closed" if input_closed else "first_input_timeout",
                        valid_target_count=valid_target_count, rejected_target_count=rejected_target_count)
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[SAFE] no valid target; rejected={rejected_target_count}; high-level stayed in Damping")
        return 2

    # Mau state doc luc khoi dong co the da cu trong khi cho Quest toi 120 s.
    # Chot neutral tu mot mau moi ngay sau target dau tien.
    state = wait_for_state(states, max_age_s=args.state_timeout_s)
    if int(state.mode_machine) != args.expected_mode_machine:
        metadata.update(status="unsafe_start", stop_reason="mode_machine_changed_before_start")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[SAFE] mode_machine={state.mode_machine} before first command; refusing teleop")
        return 3
    start_q = [float(state.motor_state[index].q) for index in MOTOR_INDICES]
    start_head_yaw, start_head_pitch = start_q[-2:]
    if (not args.joint_limits_only and
            (abs(start_head_yaw) > args.head_yaw_max_rad or
             abs(start_head_pitch) > args.head_pitch_max_rad)):
        metadata.update(
            status="unsafe_start",
            stop_reason="head_not_neutral",
            start_head_yaw_rad=start_head_yaw,
            start_head_pitch_rad=start_head_pitch,
            head_yaw_max_rad=args.head_yaw_max_rad,
            head_pitch_max_rad=args.head_pitch_max_rad,
        )
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(
            "[SAFE] head not neutral: "
            f"yaw(IDL30)={start_head_yaw:.3f}, pitch(IDL29)={start_head_pitch:.3f}; "
            "manually center the limp head before squeezing the trigger"
        )
        return 3
    latest_state = state
    _, last_state_at = states.snapshot()
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
        while args.duration_s == 0 or time.monotonic() - started_at < args.duration_s:
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
                    valid_target_count += 1
                    last_input_at = time.monotonic()
                else:
                    rejected_target_count += 1
            if input_closed:
                stop_reason = "stream_closed"
                break
            if time.monotonic() - last_input_at > args.input_timeout_s:
                stop_reason = "input_watchdog"
                break
            observed, received_at = states.snapshot()
            if observed is not None and received_at > last_state_at:
                latest_state = observed
                last_state_at = received_at
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
            if args.joint_limits_only:
                desired = list(latest_source)
            else:
                desired = [
                    start + min(args.max_offset_rad, max(-args.max_offset_rad, source - zero))
                    for start, source, zero in zip(start_q, latest_source, source_zero)
                ]
                desired[-2] = min(args.head_yaw_max_rad, max(-args.head_yaw_max_rad, desired[-2]))
                desired[-1] = min(args.head_pitch_max_rad, max(-args.head_pitch_max_rad, desired[-1]))
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
            last_target_sequence_id=upstream_sequence,
            valid_target_count=valid_target_count,
            rejected_target_count=rejected_target_count,
            last_valid_target_age_s=time.monotonic() - last_input_at,
            finished_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        )
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(
            f"[{status.upper()}] stop={stop_reason} valid={valid_target_count} "
            f"rejected={rejected_target_count} last_target_seq={upstream_sequence} "
            f"age_s={time.monotonic() - last_input_at:.3f} evidence: {run_dir}"
        )
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
