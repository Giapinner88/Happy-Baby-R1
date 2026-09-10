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
from typing import Any, Sequence

STATE_TOPIC = "rt/lowstate"
TELEOP_MAGIC = 0x314C5455
PACKET = struct.Struct("<IIBBBB12f")
ABSOLUTE_ENVELOPE_TOLERANCE_RAD = 0.002
# Unitree R1 SDK: idl 29 = head_pitch, 30 = head_yaw. UTL1 mang thu tu ngu
# nghia (yaw, pitch), vi vay hai index cuoi phai doc theo thu tu (30, 29).
# ``teleop/r1`` giu quy uoc noi bo
# cua workspace nguon (co cho dung [pitch, yaw]) va bi sync_from_workspace.sh ghi
# de, nen moi quy doi sang chuan high_level_2 nam trong ``hardware/`` — dung cho
# bien gioi, vi day cung la noi dong goi UTL1.
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_yaw_joint", "head_pitch_joint",
)
ARM_JOINT_NAMES = JOINT_NAMES[:10]
ARM_MOTOR_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
HEAD_YAW_IDL = 30
HEAD_PITCH_IDL = 29
MOTOR_INDICES = ARM_MOTOR_INDICES + (HEAD_YAW_IDL, HEAD_PITCH_IDL)


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
    parser.add_argument(
        "--max-offset-rad-shoulders", type=float, default=None,
        help=(
            "Envelope riêng cho 6 khớp vai. Bỏ trống thì dùng chung "
            "--max-offset-rad. Vai là nơi tầm hoạt động lớn nhất, và giới hạn "
            "khớp theo asset đã được producer kẹp trước rồi."
        ),
    )
    home = parser.add_mutually_exclusive_group()
    home.add_argument(
        "--home-to-nominal", action="store_true",
        help="Ramp arms/head to the nominal pose before teleop, then anchor there.",
    )
    home.add_argument(
        "--home-to-source", action="store_true",
        help=(
            "Ramp to the first validated relative-source target, then use that "
            "same vector as both robot and source anchor."
        ),
    )
    parser.add_argument(
        "--home-pose-rad", type=float, nargs=12, default=None,
        help="Home pose in wire order; default is the arms_head sim nominal (all zeros).",
    )
    parser.add_argument("--home-rate-rad-s", type=float, default=0.15)
    parser.add_argument("--home-timeout-s", type=float, default=45.0)
    parser.add_argument("--home-tolerance-rad", type=float, default=0.02)
    parser.add_argument("--head-yaw-max-rad", type=float, default=0.60)
    parser.add_argument("--head-pitch-max-rad", type=float, default=0.35)
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
    if not 10 <= args.send_hz <= 250:
        raise SystemExit("invalid send rate")
    # Trần nâng 0.30 -> 1.0 sau phiên 2026-08-29: ở 0.15 rad/s thì 10/12 khớp
    # bão hoà envelope, tức người vận hành liên tục đụng tường. 1.0 rad (57 độ)
    # đủ gấp duỗi khuỷu và đưa tay quanh thân mà vẫn là bao an toàn thật — một
    # lệnh sai không thể quăng tay qua cả tầm.
    if not 0.02 <= args.max_offset_rad <= 1.0:
        raise SystemExit("invalid session envelope (0.02..1.0 rad)")
    if args.max_offset_rad_shoulders is not None:
        # 3.2 phủ trọn khớp rộng nhất (shoulder pitch ±3.142 trong asset), nên
        # ở mức đó envelope thôi ràng buộc vai và giới hạn khớp của asset —
        # producer đã kẹp theo R1.urdf — trở thành lớp chặn duy nhất.
        if not 0.02 <= args.max_offset_rad_shoulders <= 3.2:
            raise SystemExit("invalid shoulder envelope (0.02..3.2 rad)")
    if not 0.05 <= args.head_yaw_max_rad <= 1.0:
        raise SystemExit("--head-yaw-max-rad must be in [0.05, 1.0]")
    if not 0.05 <= args.head_pitch_max_rad <= 0.6:
        raise SystemExit("--head-pitch-max-rad must be in [0.05, 0.6]")
    home_enabled = args.home_to_nominal or args.home_to_source
    if args.home_to_source and args.home_pose_rad is not None:
        raise SystemExit("--home-pose-rad is only valid with --home-to-nominal")
    if home_enabled:
        # Chậm hơn hẳn trần teleop: robot đang tự đi chứ không bám theo người,
        # nên tốc độ phải là tốc độ nhìn thấy kịp và nhả cò kịp.
        if not 0.02 <= args.home_rate_rad_s <= 0.30:
            raise SystemExit("--home-rate-rad-s must be in [0.02, 0.30]")
        if not 5.0 <= args.home_timeout_s <= 120.0:
            raise SystemExit("--home-timeout-s must be in [5.0, 120.0]")
        if not 0.005 <= args.home_tolerance_rad <= 0.10:
            raise SystemExit("--home-tolerance-rad must be in [0.005, 0.10]")
        if args.home_to_nominal:
            pose = args.home_pose_rad if args.home_pose_rad is not None else [0.0] * len(JOINT_NAMES)
            if len(pose) != len(JOINT_NAMES) or not all(math.isfinite(v) for v in pose):
                raise SystemExit("--home-pose-rad must be 12 finite values")
            # Không có URDF ở phía robot, nên chặn bằng một biên thô: tư thế home là
            # một hằng số đã biết, không phải luồng, nên cái này chỉ để bắt lỗi gõ.
            if max(abs(v) for v in pose) > 1.6:
                raise SystemExit("--home-pose-rad outside +-1.6 rad; refusing")
            args.home_pose_rad = list(pose)


def parse_target(
    line: str, previous_sequence: int
) -> tuple[int, list[float], bool, str, bool] | None:
    try:
        payload = json.loads(line)
        names = tuple(str(value) for value in payload["joint_names"])
        positions = [float(value) for value in payload["positions_rad"]]
        sequence = int(payload["sequence_id"])
        schema_version = int(payload.get("schema_version", 1))
        target_mode = str(payload.get("target_mode", "relative_source"))
        rehome = bool(payload.get("rehome", False))
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if schema_version != 1 or target_mode not in {"relative_source", "absolute_robot"}:
        return None
    if names == ARM_JOINT_NAMES and len(positions) == len(ARM_MOTOR_INDICES):
        head_valid = False
    elif names == JOINT_NAMES and len(positions) == len(MOTOR_INDICES):
        head_valid = True
    else:
        return None
    if sequence <= previous_sequence or not all(math.isfinite(value) for value in positions):
        return None
    return sequence, positions, head_valid, target_mode, rehome


def encode_target(sequence: int, positions: list[float], head_valid: bool = True) -> bytes:
    """Encode arm-only or arms+head target in the fixed UTL1 packet ABI."""
    expected = 12 if head_valid else 10
    if len(positions) != expected:
        raise ValueError(f"expected {expected} positions for head_valid={head_valid}")
    arms = positions[:10]
    head_yaw, head_pitch = positions[10:] if head_valid else (0.0, 0.0)
    return PACKET.pack(
        TELEOP_MAGIC, sequence, 1, 1, int(head_valid), 0, *arms, head_yaw, head_pitch
    )


def encode_stop(sequence: int) -> bytes:
    return PACKET.pack(TELEOP_MAGIC, sequence, 0, 0, 0, 0, *([0.0] * 12))


def sequence_seed(send_hz: float, monotonic_s: float | None = None) -> int:
    """Create a UTL1 sequence that remains newer across sidecar restarts."""
    now_s = time.monotonic() if monotonic_s is None else monotonic_s
    return int(now_s * send_hz) & 0xFFFFFFFF


def next_sequence(sequence: int) -> int:
    return (sequence + 1) & 0xFFFFFFFF


def constrain_absolute_target(
    target: list[float],
    start: list[float],
    max_offset_rad: float,
    tolerance_rad: float = ABSOLUTE_ENVELOPE_TOLERANCE_RAD,
) -> tuple[list[float], int, float, bool]:
    """Validate an absolute target and clamp only numerical boundary drift.

    LeRobot and this robot-side process intentionally enforce the same session
    envelope. They sample the encoder anchor at slightly different times, so a
    target already clamped to exactly ``max_offset_rad`` by LeRobot can exceed
    the robot-side comparison by a few encoder ticks. A small fixed tolerance
    prevents that false stop; the packet sent to the high-level owner remains
    clamped to the exact robot-side envelope.
    """
    if len(target) != len(start) or not target:
        raise ValueError("absolute target/start shape mismatch")
    offsets = [value - anchor for value, anchor in zip(target, start)]
    worst_index = max(range(len(offsets)), key=lambda index: abs(offsets[index]))
    worst_offset = abs(offsets[worst_index])
    if worst_offset > max_offset_rad + tolerance_rad:
        raise ValueError("absolute target outside session envelope")
    bounded = [
        anchor + min(max_offset_rad, max(-max_offset_rad, offset))
        for anchor, offset in zip(start, offsets)
    ]
    was_clamped = any(abs(raw - safe) > 1e-12 for raw, safe in zip(target, bounded))
    return bounded, worst_index, worst_offset, was_clamped


SHOULDER_JOINT_NAMES = tuple(n for n in JOINT_NAMES if "shoulder" in n)


def per_joint_envelope(names: Sequence[str], default_rad: float, shoulder_rad: float | None) -> list[float]:
    """Envelope cho từng khớp, vai có thể rộng hơn phần còn lại.

    Một con số chung cho cả 12 khớp khiến vai — nơi tầm hoạt động lớn nhất —
    bị bó theo khớp hẹp nhất. Tách ra để nới vai mà không đụng tới cổ tay hay
    đầu, vốn không cần và không nên đi xa.
    """

    if shoulder_rad is None:
        return [default_rad] * len(names)
    return [shoulder_rad if "shoulder" in n else default_rad for n in names]


def validate_source_home_goal(source: Sequence[float], envelope: Sequence[float]) -> tuple[int, float]:
    """Refuse a first vendor posture outside the declared zero-centred bounds.

    Source homing is the one intentional move that may cross the session offset
    from the limp encoder pose. It is nevertheless a named, bounded posture:
    the workstation already clamps it to the R1 asset limits, and this robot-side
    check independently requires every absolute angle to fit the declared
    per-joint envelope. Nothing is silently clipped during alignment.
    """

    values = [float(value) for value in source]
    bounds = [float(value) for value in envelope]
    if not values or len(values) != len(bounds):
        raise ValueError("source home goal/envelope shape mismatch")
    if not all(math.isfinite(value) for value in values + bounds) or min(bounds) <= 0.0:
        raise ValueError("source home goal/envelope must be finite and positive")
    ratios = [abs(value) / bound for value, bound in zip(values, bounds)]
    worst_index = max(range(len(values)), key=ratios.__getitem__)
    if ratios[worst_index] > 1.0:
        raise ValueError("source home goal outside zero-centred envelope")
    return worst_index, abs(values[worst_index])


def relative_session_target(
    start: Sequence[float],
    source: Sequence[float],
    source_zero: Sequence[float],
    envelope: Sequence[float],
) -> tuple[list[float], tuple[int, ...], float]:
    """Map a relative source into the robot session and expose all clipping.

    When source homing makes ``start == source_zero``, this reduces exactly to
    ``target == source`` until an envelope is reached. That identity is the
    contract which makes the sidecar reproduce the producer's upstream joint
    stream without adding an affine offset. The hardware producer may still
    rate-limit the raw vendor stream before it reaches this process.
    """

    lengths = {len(start), len(source), len(source_zero), len(envelope)}
    if len(lengths) != 1 or len(start) == 0:
        raise ValueError("relative session vectors must have one non-empty shape")
    raw_offsets = [value - zero for value, zero in zip(source, source_zero)]
    bounded_offsets = [
        min(bound, max(-bound, offset))
        for offset, bound in zip(raw_offsets, envelope)
    ]
    clamped = tuple(
        index
        for index, (raw, bounded) in enumerate(zip(raw_offsets, bounded_offsets))
        if abs(raw - bounded) > 1e-12
    )
    desired = [anchor + offset for anchor, offset in zip(start, bounded_offsets)]
    worst_offset = max(abs(value) for value in raw_offsets)
    return desired, clamped, worst_offset


def constrain_head_target(
    yaw_rad: float,
    pitch_rad: float,
    yaw_max_rad: float,
    pitch_max_rad: float,
) -> tuple[float, float, tuple[str, ...]]:
    """Apply the explicit hardware head gate and report every affected axis."""

    bounded_yaw = min(yaw_max_rad, max(-yaw_max_rad, yaw_rad))
    bounded_pitch = min(pitch_max_rad, max(-pitch_max_rad, pitch_rad))
    clamped = []
    if abs(yaw_rad - bounded_yaw) > 1e-12:
        clamped.append("head_yaw_joint")
    if abs(pitch_rad - bounded_pitch) > 1e-12:
        clamped.append("head_pitch_joint")
    return bounded_yaw, bounded_pitch, tuple(clamped)


def head_outside_gate(yaw_rad: float, pitch_rad: float, yaw_max: float, pitch_max: float) -> bool:
    """Đầu có lệch quá mức cho phép lúc chốt mốc phiên không.

    Tách riêng vì cùng một phép kiểm được dùng ở hai chỗ với hai ý nghĩa khác
    nhau: trước homing nó kiểm điều kiện ban đầu, sau homing nó kiểm kết quả.
    """

    return abs(yaw_rad) > yaw_max or abs(pitch_rad) > pitch_max


def ramp_toward(current: list[float], goal: list[float], max_step: float) -> list[float]:
    """One rate-limited step from `current` toward `goal`.

    The homing move is generated here rather than streamed in, which is what
    keeps the teleop envelope out of it: the target is one named pose and the
    only freedom is how fast it is approached. Nothing a producer sends can
    widen it.
    """

    stepped = []
    for now, want in zip(current, goal):
        delta = want - now
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        stepped.append(now + delta)
    return stepped


def stdin_reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


class LatestLowState:
    """DDS callback mailbox; reading never waits for a DDS sample.

    Receipt time belongs to the callback, not the control loop. Re-reading an
    old sample must never refresh the watchdog.
    """

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


def wait_for_state(states: LatestLowState, timeout_s: float = 5.0, max_age_s: float = 0.2) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state, received_at = states.snapshot()
        if state is not None and time.monotonic() - received_at <= max_age_s:
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
        "max_offset_from_start_rad": args.max_offset_rad,
        "max_offset_from_start_shoulders_rad": args.max_offset_rad_shoulders,
        "absolute_envelope_tolerance_rad": ABSOLUTE_ENVELOPE_TOLERANCE_RAD,
        "send_hz": args.send_hz,
        "input_timeout_s": args.input_timeout_s,
        "state_timeout_s": args.state_timeout_s,
        "duration_s": args.duration_s,
        "home_mode": (
            "source_aligned" if args.home_to_source
            else "nominal" if args.home_to_nominal
            else "disabled"
        ),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=stdin_reader, args=(lines,), daemon=True).start()
    source_zero: list[float] | None = None
    latest_source: list[float] | None = None
    head_valid = False
    target_mode = "relative_source"
    upstream_sequence = -1
    local_sequence = sequence_seed(args.send_hz)
    metadata["initial_ipc_sequence_id"] = local_sequence
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
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
        upstream_sequence, latest_source, head_valid, target_mode, _ = parsed
        source_zero = latest_source.copy()
        last_input_at = time.monotonic()

    if source_zero is None:
        metadata.update(status="no_input", stop_reason="stream_closed" if input_closed else "first_input_timeout")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print("[SAFE] no valid target; high-level stayed in Damping")
        return 2

    # Mau state doc luc khoi dong co the da cu trong khi cho Quest toi 120 s.
    # Chot neutral tu mot mau moi ngay sau target dau tien.
    state = wait_for_state(states, max_age_s=args.state_timeout_s)
    if int(state.mode_machine) != args.expected_mode_machine:
        metadata.update(status="unsafe_start", stop_reason="mode_machine_changed_before_start")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[SAFE] mode_machine={state.mode_machine} before first command; refusing teleop")
        return 3
    selected_names = JOINT_NAMES if head_valid else ARM_JOINT_NAMES
    envelope = per_joint_envelope(
        selected_names, args.max_offset_rad, args.max_offset_rad_shoulders
    )
    selected_motor_indices = MOTOR_INDICES if head_valid else ARM_MOTOR_INDICES
    start_q = [float(state.motor_state[index].q) for index in selected_motor_indices]
    if head_valid:
        start_head_yaw, start_head_pitch = start_q[-2:]
    else:
        start_head_yaw, start_head_pitch = 0.0, 0.0
    home_enabled = args.home_to_nominal or args.home_to_source
    # Gate này tồn tại vì phiên là tương đối: đầu lệch bao nhiêu lúc chốt thì
    # lệch bấy nhiêu suốt phiên. Khi homing bật thì tiền đề đó không còn — đầu
    # được đưa về goal đã kiểm tra trước khi chốt, nên gate chuyển xuống chạy SAU homing.
    #
    # Encoder là tuyệt đối, nên robot biết chính xác đầu đang ở đâu ngay khi bật;
    # thứ nó thiếu để tự về chỉ là mô-men và một lệnh. Chặn ở đây tức là chặn
    # đúng cái cơ chế sửa được vấn đề. Và khi đầu đang tì vào chặn cơ khí thì đi
    # về giữa là RỜI KHỎI giới hạn — hướng an toàn nhất.
    if head_valid and not home_enabled and head_outside_gate(
        start_head_yaw, start_head_pitch, args.head_yaw_max_rad, args.head_pitch_max_rad
    ):
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
    if args.home_to_source and target_mode != "relative_source":
        metadata.update(status="unsafe_start", stop_reason="source_home_requires_relative_stream")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print("[SAFE] --home-to-source requires target_mode=relative_source")
        return 3
    if args.home_to_source:
        try:
            validate_source_home_goal(source_zero, envelope)
        except ValueError:
            ratios = [abs(value) / bound for value, bound in zip(source_zero, envelope)]
            worst_index = max(range(len(ratios)), key=ratios.__getitem__)
            metadata.update(
                status="unsafe_start",
                stop_reason="source_home_goal_outside_envelope",
                violating_joint=selected_names[worst_index],
                source_home_goal_rad=source_zero[worst_index],
                source_home_limit_rad=envelope[worst_index],
            )
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            print(
                f"[SAFE] source home goal {selected_names[worst_index]}="
                f"{source_zero[worst_index]:.3f} rad exceeds its {envelope[worst_index]:.3f} rad bound"
            )
            return 3
    if target_mode == "absolute_robot":
        try:
            _, worst_index, worst_initial_error, _ = constrain_absolute_target(
                source_zero, start_q, args.max_offset_rad
            )
        except ValueError:
            offsets = [abs(target - state_q) for target, state_q in zip(source_zero, start_q)]
            worst_index = max(range(len(offsets)), key=offsets.__getitem__)
            worst_initial_error = offsets[worst_index]
            metadata.update(
                status="unsafe_start",
                stop_reason="absolute_target_outside_session_envelope",
                worst_initial_error_rad=worst_initial_error,
                violating_joint=selected_names[worst_index],
                envelope_limit_rad=args.max_offset_rad,
            )
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            print(
                f"[SAFE] absolute target starts {worst_initial_error:.3f} rad from encoders; "
                f"limit is {args.max_offset_rad:.3f} rad"
            )
            return 3
    # Mở transport và các mốc watchdog TRƯỚC pha homing: homing cũng gửi gói và
    # cũng phải chịu đúng những watchdog đó, không phải một đường vòng.
    latest_state = state
    latest_state, last_state_at = states.snapshot()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.connect((args.udp_host, args.udp_port))
    status = "completed"
    stop_reason = "duration_elapsed"

    # --- Pha homing ---------------------------------------------------------
    # Đường phần cứng vẫn là phiên tương đối: target = start_q + lệch, có
    # envelope. Với upstream, goal là target vendor đầu tiên và hai mốc được
    # chốt cùng một vector; vì thế công thức rút gọn thành target = source q,
    # không cộng offset posture. Producer hardware vẫn rate-limit q vendor trước
    # khi gửi source. Coupled giữ cách
    # cũ là về nominal rồi chốt lại source mới nhất.
    #
    # Source goal phụ thuộc neutral của người vận hành, nên không giả định chỉ
    # khuỷu chuyển động: mọi khớp tay và đầu đều có thể đi. Launcher phải yêu cầu
    # dọn khoảng trống quanh toàn bộ upper body trước khi bóp cò.
    #
    # Homing phải nằm TRONG phiên này chứ không thể là một tool riêng chạy trước:
    # owner nhả về ZERO TORQUE sau teleop_timeout_ms khi ngừng nhận gói, nên tay
    # vừa gập xong sẽ limp và rơi lại trước khi teleop kịp tiếp quản.
    home_report: dict[str, object] | None = None
    if home_enabled:
        goal_kind = "source vendor ban đầu" if args.home_to_source else "nominal"
        goal = list(source_zero) if args.home_to_source else list(args.home_pose_rad)
        if not head_valid:
            goal = goal[: len(ARM_MOTOR_INDICES)]
        commanded = list(start_q)
        step = args.home_rate_rad_s / args.send_hz
        period = 1.0 / args.send_hz
        started = time.monotonic()
        worst = max(abs(g - s) for g, s in zip(goal, start_q))
        command_error = worst
        measured_error = worst
        accepted_home_input_count = 0
        print(
            f"[HOME] đưa {len(goal)} khớp về {goal_kind}, lệch lớn nhất {worst:.3f} rad, "
            f"{args.home_rate_rad_s:.2f} rad/s -> khoảng {worst / args.home_rate_rad_s:.1f}s. "
            "GIỮ NGUYÊN cò; robot đang tự đi, chưa bám theo tay."
        )
        while True:
            loop_start = time.monotonic()
            # Vẫn phải rút stdin: producer bơm đều, không đọc thì đầy pipe và nó
            # chặn. Giữ mẫu mới nhất; goal source ban đầu vẫn được đóng băng để
            # pha tự chạy không đuổi theo một tay người đang di chuyển.
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
                    (parsed_sequence, parsed_source, parsed_head_valid,
                     parsed_target_mode, _) = parsed
                    if parsed_head_valid != head_valid or parsed_target_mode != target_mode:
                        stop_reason = "home_aborted_stream_contract_changed"
                        status = "failed"
                        input_closed = True
                        break
                    upstream_sequence, latest_source = parsed_sequence, parsed_source
                    accepted_home_input_count += 1
                    last_input_at = time.monotonic()
            input_age_s = time.monotonic() - last_input_at
            if input_closed or input_age_s > args.input_timeout_s:
                # EOF và watchdog từng bị gộp thành home_aborted_input, khiến
                # không thể biết tầng workstation đóng pipe hay chỉ ngừng phát.
                # Tách chúng ra nhưng giữ nguyên hành vi fail-closed.
                if stop_reason != "home_aborted_stream_contract_changed":
                    stop_reason = (
                        "home_aborted_stream_closed"
                        if input_closed
                        else "home_aborted_input_watchdog"
                    )
                    status = "aborted"
                home_report = {
                    "reached": False,
                    "elapsed_s": time.monotonic() - started,
                    "initial_worst_error_rad": worst,
                    "final_command_error_rad": command_error,
                    "final_measured_error_rad": measured_error,
                    "goal_kind": goal_kind,
                    "goal_q_rad": goal,
                    "abort_input_closed": input_closed,
                    "last_input_age_s": input_age_s,
                    "accepted_input_count": accepted_home_input_count,
                    "last_upstream_sequence_id": upstream_sequence,
                }
                print(
                    f"[SAFE] homing mất target: {stop_reason}; "
                    f"input_age={input_age_s:.3f}s, accepted={accepted_home_input_count}, "
                    f"last_seq={upstream_sequence}."
                )
                break
            latest_state, last_state_at = states.snapshot()
            if time.monotonic() - last_state_at > args.state_timeout_s:
                stop_reason = "home_aborted_lowstate"
                status = "failed"
                break
            if int(latest_state.mode_machine) != args.expected_mode_machine:
                stop_reason = "home_aborted_mode_machine"
                status = "failed"
                break
            commanded = ramp_toward(commanded, goal, step)
            local_sequence = next_sequence(local_sequence)
            client.send(encode_target(local_sequence, commanded, head_valid))
            command_error = max(abs(g - c) for g, c in zip(goal, commanded))
            measured_q = [
                float(latest_state.motor_state[index].q)
                for index in selected_motor_indices
            ]
            measured_error = max(abs(g - q) for g, q in zip(goal, measured_q))
            if command_error <= args.home_tolerance_rad:
                home_report = {
                    "reached": True,
                    "elapsed_s": time.monotonic() - started,
                    "initial_worst_error_rad": worst,
                    "final_command_error_rad": command_error,
                    "final_measured_error_rad": measured_error,
                    "encoder_within_home_tolerance": measured_error <= args.home_tolerance_rad,
                    "goal_kind": goal_kind,
                    "goal_q_rad": goal,
                }
                print(
                    f"[HOME] ramp tới goal sau {home_report['elapsed_s']:.1f}s; "
                    f"encoder residual {measured_error:.3f} rad; chốt mốc phiên."
                )
                break
            if time.monotonic() - started > args.home_timeout_s:
                home_report = {
                    "reached": False,
                    "elapsed_s": time.monotonic() - started,
                    "initial_worst_error_rad": worst,
                    "final_command_error_rad": command_error,
                    "final_measured_error_rad": measured_error,
                    "goal_kind": goal_kind,
                    "goal_q_rad": goal,
                }
                stop_reason = "home_timeout"
                status = "failed"
                print(
                    f"[SAFE] homing quá {args.home_timeout_s:.0f}s; "
                    f"lệnh còn {command_error:.3f} rad, encoder còn {measured_error:.3f} rad; dừng."
                )
                break
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

        if home_report is None or not home_report.get("reached"):
            metadata.update(status=status, stop_reason=stop_reason, home=home_report)
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            try:
                client.send(encode_stop(next_sequence(local_sequence)))
            except OSError:
                pass
            client.close()
            print("[SAFE] homing không hoàn tất; không vào teleop.")
            return 3

        # Với source homing, giữ nguyên source_zero đầu tiên: start_q và
        # source_zero cùng bằng target vendor đó, nên ánh xạ sau homing đồng nhất
        # với q tuyệt đối trong sim. Nominal legacy vẫn re-anchor mẫu mới nhất.
        start_q = list(commanded)
        assert latest_source is not None
        if args.home_to_nominal:
            source_zero = latest_source.copy()

        # Gate đầu chạy ở đây thay vì trước homing: bây giờ nó kiểm KẾT QUẢ chứ
        # không kiểm điều kiện ban đầu. Goal đầu đã qua bound nên bình thường
        # phải đạt; không đạt nghĩa là đầu bị kẹt hoặc owner đang kẹp lệnh, và
        # đó mới là lúc phải dừng.
        if head_valid:
            # Đọc encoder, KHÔNG đọc giá trị vừa ra lệnh. Ramp ở đây là vòng hở,
            # và owner còn kẹp lệnh đầu ở teleop_head_yaw_max (1.0 rad) trước
            # khi slew, nên "đã ra lệnh 0" không chứng minh được "đầu đã về 0".
            # Kiểm bằng chính cái mình ra lệnh thì gate này luôn pass và vô dụng.
            latest_state, last_state_at = states.snapshot()
            if time.monotonic() - last_state_at > args.state_timeout_s:
                metadata.update(status="failed", stop_reason="lowstate_watchdog_after_home", home=home_report)
                metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
                client.send(encode_stop(next_sequence(local_sequence)))
                client.close()
                print("[SAFE] lowstate stale after homing; refusing teleop.", flush=True)
                return 3
            measured = [float(latest_state.motor_state[i].q) for i in selected_motor_indices]
            reached_yaw, reached_pitch = measured[-2:]
            if head_outside_gate(
                reached_yaw, reached_pitch, args.head_yaw_max_rad, args.head_pitch_max_rad
            ):
                metadata.update(
                    status="unsafe_start",
                    stop_reason="head_not_neutral_after_home",
                    start_head_yaw_rad=reached_yaw,
                    start_head_pitch_rad=reached_pitch,
                    commanded_head_yaw_rad=start_q[-2],
                    commanded_head_pitch_rad=start_q[-1],
                    home=home_report,
                )
                metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
                try:
                    client.send(encode_stop(next_sequence(local_sequence)))
                except OSError:
                    pass
                client.close()
                print(
                    "[SAFE] homing xong mà đầu vẫn lệch: "
                    f"yaw(IDL30)={reached_yaw:.3f}, pitch(IDL29)={reached_pitch:.3f}; "
                    "kiểm đầu có bị vướng không."
                )
                return 3

    metadata.update(
        per_joint_envelope_rad=envelope,
        joint_names=selected_names,
        motor_indices=selected_motor_indices,
        head_valid=head_valid,
        target_mode=target_mode,
        home=home_report,
    )
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    started_at = time.monotonic()
    period = 1.0 / args.send_hz
    sample_index = 0
    # Cò trái không còn là dừng phiên. Nó xin đưa robot về lại goal của mode
    # homing rồi chốt lại mốc — thứ người vận hành thực sự cần khi tay đã trôi tới rìa
    # envelope và muốn bắt đầu lại mà không phải chạy lại cả pipeline.
    rehome_requested = False
    rehome_goal: list[float] | None = None
    rehome_count = 0
    relative_envelope_clamp_count = 0
    relative_envelope_clamped_joints: set[str] = set()
    maximum_relative_source_offset_rad = 0.0
    head_gate_clamp_count = 0
    head_gate_clamped_joints: set[str] = set()
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
                    (parsed_sequence, parsed_source, parsed_head_valid,
                     parsed_target_mode, parsed_rehome) = parsed
                    if parsed_head_valid != head_valid or parsed_target_mode != target_mode:
                        status = "failed"
                        stop_reason = "stream_contract_changed"
                        input_closed = True
                        break
                    upstream_sequence, latest_source = parsed_sequence, parsed_source
                    last_input_at = time.monotonic()
                    if parsed_rehome:
                        rehome_requested = True
            if input_closed:
                if stop_reason != "stream_contract_changed":
                    stop_reason = "stream_closed"
                break
            if time.monotonic() - last_input_at > args.input_timeout_s:
                stop_reason = "input_watchdog"
                break
            latest_state, last_state_at = states.snapshot()
            if time.monotonic() - last_state_at > args.state_timeout_s:
                stop_reason = "lowstate_watchdog"
                status = "failed"
                break
            if int(latest_state.mode_machine) != args.expected_mode_machine:
                stop_reason = "mode_machine_changed"
                status = "failed"
                break
            assert latest_source is not None
            if rehome_requested and home_enabled and rehome_goal is None:
                goal = list(latest_source) if args.home_to_source else list(args.home_pose_rad)
                if not head_valid:
                    goal = goal[: len(ARM_MOTOR_INDICES)]
                if args.home_to_source:
                    try:
                        validate_source_home_goal(goal, envelope)
                    except ValueError:
                        status = "failed"
                        stop_reason = "rehome_source_goal_outside_envelope"
                        break
                commanded = [
                    float(latest_state.motor_state[index].q)
                    for index in selected_motor_indices
                ]
                rehome_goal = goal
                rehome_started_at = time.monotonic()
                rehome_requested = False
            if rehome_goal is not None:
                # One step per outer tick: stdin, lowstate and mode watchdogs
                # remain active throughout rehoming as well as initial homing.
                if time.monotonic() - rehome_started_at > args.home_timeout_s:
                    status, stop_reason = "failed", "rehome_timeout"
                    break
                commanded = ramp_toward(commanded, rehome_goal, args.home_rate_rad_s / args.send_hz)
                local_sequence = next_sequence(local_sequence)
                client.send(encode_target(local_sequence, commanded, head_valid))
                if max(abs(g - c) for g, c in zip(rehome_goal, commanded)) <= args.home_tolerance_rad:
                    start_q = list(commanded)
                    source_zero = list(rehome_goal if args.home_to_source else latest_source)
                    rehome_goal = None
                    rehome_count += 1
                    print(f"[HOME] cò trái: đã về {goal_kind} và chốt lại mốc (lần {rehome_count}).", flush=True)
                rehome_requested = False
                remaining = period - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
                continue
            rehome_requested = False
            local_sequence = next_sequence(local_sequence)
            if target_mode == "relative_source":
                desired, clamped_indices, worst_offset = relative_session_target(
                    start_q, latest_source, source_zero, envelope
                )
                maximum_relative_source_offset_rad = max(
                    maximum_relative_source_offset_rad, worst_offset
                )
                if clamped_indices:
                    relative_envelope_clamp_count += 1
                    relative_envelope_clamped_joints.update(
                        selected_names[index] for index in clamped_indices
                    )
            else:
                try:
                    desired, worst_index, worst_offset, boundary_clamped = constrain_absolute_target(
                        latest_source, start_q, args.max_offset_rad
                    )
                except ValueError:
                    offsets = [
                        abs(target - start)
                        for target, start in zip(latest_source, start_q)
                    ]
                    worst_index = max(range(len(offsets)), key=offsets.__getitem__)
                    status = "failed"
                    stop_reason = "absolute_target_outside_session_envelope"
                    metadata["envelope_violation"] = {
                        "joint": selected_names[worst_index],
                        "offset_rad": offsets[worst_index],
                        "limit_rad": args.max_offset_rad,
                        "tolerance_rad": ABSOLUTE_ENVELOPE_TOLERANCE_RAD,
                    }
                    break
                if boundary_clamped:
                    metadata["numerical_boundary_clamp_seen"] = True
                    metadata["maximum_preclamp_offset_rad"] = max(
                        float(metadata.get("maximum_preclamp_offset_rad", 0.0)),
                        worst_offset,
                    )
            if head_valid:
                desired[-2], desired[-1], head_clamped = constrain_head_target(
                    desired[-2], desired[-1], args.head_yaw_max_rad, args.head_pitch_max_rad
                )
                if head_clamped:
                    head_gate_clamp_count += 1
                    head_gate_clamped_joints.update(head_clamped)
            client.send(encode_target(local_sequence, desired, head_valid=head_valid))
            if sample_index % max(1, round(args.send_hz / 10.0)) == 0:
                record = {
                    "monotonic_s": time.monotonic(),
                    "upstream_sequence_id": upstream_sequence,
                    "ipc_sequence_id": local_sequence,
                    "joint_names": selected_names,
                    "target_q": desired,
                    "observed_q": [
                        float(latest_state.motor_state[index].q)
                        for index in selected_motor_indices
                    ],
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
        local_sequence = next_sequence(local_sequence)
        try:
            client.send(encode_stop(local_sequence))
        except OSError:
            pass
        client.close()
        metadata.update(
            status=status,
            stop_reason=stop_reason,
            rehome_count=rehome_count,
            relative_envelope_clamp_count=relative_envelope_clamp_count,
            relative_envelope_clamped_joints=sorted(relative_envelope_clamped_joints),
            maximum_relative_source_offset_rad=maximum_relative_source_offset_rad,
            head_gate_clamp_count=head_gate_clamp_count,
            head_gate_clamped_joints=sorted(head_gate_clamped_joints),
            last_ipc_sequence_id=local_sequence,
            finished_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        )
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[{status.upper()}] stop={stop_reason} evidence: {run_dir}")
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
