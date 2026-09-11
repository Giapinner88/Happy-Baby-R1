#!/usr/bin/env python3
"""Map Quest JSONL to relative-session R1-A5 arms/head joint targets.

This workstation process performs no DDS writes. The joints arrive already
solved by the unmodified vendor ``R1_A5_ArmIK`` and leave as a 12-joint JSONL
stream for the fail-closed robot receiver. Nothing here solves IK.

The envelope this process enforces on the vendor path is not optional. Upstream
constrains against its own `r1_a5.urdf` and ships no rate limiter -- in
simulation that is exactly the point, and joint velocity is merely recorded,
but a robot needs the limit applied rather than measured. Both modes therefore
leave here through the same velocity/acceleration limiter and the same asset
joint limits.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1 import R1TeleopCommand, R1TeleopMapper, TeleopCalibration, TeleopLimits, Vector3  # noqa: E402
from teleop.r1.rate_limit import OnlineJointLimiter  # noqa: E402
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model  # noqa: E402

# Model order: what the URDF, the solvers and the limiter all use.
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)
# Wire order: what the robot receiver accepts. The R1-A5 IDL puts head pitch at
# 29 and yaw at 30, but the UTL1 packet carries them semantically as (yaw,
# pitch), and the receiver names its vector to match the packet it builds. The
# swap therefore happens once, here at the boundary, and never inside a solver.
# The names travel with the values, so the receiver rejects a stream that
# forgets it rather than silently driving the wrong head joint.
RECEIVER_JOINT_NAMES = JOINT_NAMES[:10] + ("head_yaw_joint", "head_pitch_joint")


def to_receiver_order(positions_rad) -> list[float]:
    values = [float(value) for value in positions_rad]
    if len(values) != len(JOINT_NAMES):
        raise SystemExit(f"expected {len(JOINT_NAMES)} joint values, got {len(values)}")
    return values[:10] + [values[11], values[10]]


def make_receiver_payload(sequence_id, positions, solution_kind, rehome=False) -> dict:
    """One wire contract for both solvers; inputs are already safety-limited."""
    payload = {
        "schema_version": 1,
        "target_mode": "relative_source",
        "sequence_id": sequence_id,
        "sent_monotonic_s": time.monotonic(),
        "joint_names": RECEIVER_JOINT_NAMES,
        "positions_rad": to_receiver_order(positions),
        "solution_kind": solution_kind,
    }
    if rehome:
        payload["rehome"] = True
    return payload


def upstream_solution(payload: dict) -> np.ndarray | None:
    """The vendor-solved joint vector carried by a passthrough command line.

    Returns ``None`` when the line carries no solved vector at all. A vector
    under the wrong joint names is refused outright rather than reordered:
    applying correct numbers to the wrong joints is the one failure this path
    must never be able to produce, and on hardware it is not recoverable.
    """

    positions = payload.get("upstream_joint_position_rad")
    if positions is None:
        return None
    names = tuple(str(value) for value in payload.get("upstream_joint_names", ()))
    if names != JOINT_NAMES:
        raise SystemExit(f"upstream joint names do not match the R1-A5 arms/head order: {names}")
    values = np.asarray(positions, dtype=float)
    if values.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(values)):
        raise SystemExit("upstream joint vector must hold 12 finite values")
    return values


def _reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--command-log", type=Path, default=None,
                        help="Record emitted targets and workstation monotonic timestamps.")
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/baseline/config/upstream_stream.json",
    )
    parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
        default=1.0,
        help=(
            "Hardware joint speed ceiling; applies in both solver modes. Kept "
            "above the owner's own slew so the owner stays the binding limit "
            "rather than this process silently becoming it."
        ),
    )
    parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
        default=2.0,
        help="Hardware joint acceleration ceiling; applies in both solver modes.",
    )
    args = parser.parse_args()
    if (
        not math.isfinite(args.max_joint_velocity_rad_s)
        or not math.isfinite(args.max_joint_acceleration_rad_s2)
        or args.max_joint_velocity_rad_s <= 0.0
        or args.max_joint_acceleration_rad_s2 <= 0.0
    ):
        raise SystemExit("joint velocity and acceleration ceilings must be finite and positive")
    if (
        not math.isfinite(args.control_hz)
        or not math.isfinite(args.duration_s)
        or args.control_hz <= 0.0
        or args.duration_s <= 0.0
    ):
        raise SystemExit("control rate and duration must be finite and positive")

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    declared = dict(profile["model"])
    model = load_r1_a5_upper_body_model(
        (ROOT / str(declared["urdf_path"])).resolve(), control_waist_yaw=False
    )
    if tuple(model.joint_names) != JOINT_NAMES:
        raise SystemExit(f"unexpected arms_head joint order: {model.joint_names}")
    limiter = OnlineJointLimiter(
        max_velocity_rad_s=args.max_joint_velocity_rad_s,
        max_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        dt_s=1.0 / args.control_hz,
        lower_limits=model.lower_limits,
        upper_limits=model.upper_limits,
    )
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(0.0, 0.0, 0.0),
            yaw_rad=0.0,
            source_frame="quest_headset",
            robot_frame="neutral_waist_yaw_link",
        ),
        TeleopLimits(command_timeout_s=0.5, allow_velocity=False),
    )
    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=_reader, args=(lines,), daemon=True).start()
    deadline = time.monotonic() + args.duration_s
    previous_sequence = -1
    stream_started = False
    rehome_pending = False
    period = 1.0 / args.control_hz
    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        newest: R1TeleopCommand | None = None
        newest_payload: dict | None = None
        closed = False
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                closed = True
                break
            try:
                document = json.loads(line)
                candidate = R1TeleopCommand.from_dict(document)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                print(f"[WARN] invalid Quest command: {exc}", file=sys.stderr, flush=True)
                continue
            if candidate.sequence_id <= previous_sequence:
                continue
            previous_sequence = candidate.sequence_id
            newest = candidate
            newest_payload = document
        if newest is not None:
            if newest.reset_requested:
                # Cò trái từng là dừng phiên: người vận hành phải chạy lại cả
                # pipeline chỉ để lập lại mốc. Nay nó xin sidecar căn lại theo
                # home mode đang chọn, và phiên chạy tiếp. Cờ đi kèm chính command nên
                # nó tới receiver đúng thứ tự với dòng lệnh, không cần kênh phụ.
                rehome_pending = True
                limiter.hold()
            positions = None
            solution_kind = None
            mapped_at = time.monotonic()
            target = mapper.map(newest, mapped_at)
            if not target.enabled:
                if stream_started:
                    print(
                        json.dumps(
                            {
                                "event": "hardware_target_stop",
                                "reason": target.reason,
                                "sequence_id": newest.sequence_id,
                                "command_age_s": mapped_at - newest.timestamp_monotonic_s,
                            },
                            separators=(",", ":"),
                        ),
                        file=sys.stderr,
                        flush=True,
                    )
                    return 0
            else:
                assert newest_payload is not None
                solved = upstream_solution(newest_payload)
                if solved is None:
                    # Solved vectors ride on every line once the vendor solver
                    # has produced one, so a line without one means the stream
                    # is not what this mode requires -- not a dropped sample.
                    raise SystemExit(
                        "--upstream-joint-stream expects lines carrying upstream_joint_position_rad; "
                        "pipe the Quest bridge through "
                        "`run_r1_upstream_ik_stream.py --passthrough` first"
                    )
                positions = limiter.step(model.clamp(solved))
                solution_kind = "upstream_xr_teleoperate_R1_A5_ArmIK"
            if positions is not None:
                payload = make_receiver_payload(
                    newest.sequence_id, positions, solution_kind, rehome_pending
                )
                rehome_pending = False
                try:
                    print(json.dumps(payload, separators=(",", ":")), flush=True)
                    if args.command_log is not None:
                        with args.command_log.open("a", encoding="utf-8") as log:
                            log.write(json.dumps(payload, separators=(",", ":")) + "\n")
                except BrokenPipeError:
                    print(
                        json.dumps(
                            {
                                "event": "hardware_target_stop",
                                "reason": "downstream_closed",
                                "sequence_id": newest.sequence_id,
                            },
                            separators=(",", ":"),
                        ),
                        file=sys.stderr,
                        flush=True,
                    )
                    sys.stdout = open("/dev/null", "w", encoding="utf-8")
                    return 0
                stream_started = True
        if closed:
            print(
                json.dumps(
                    {
                        "event": "hardware_target_stop",
                        "reason": "upstream_stream_closed",
                        "last_sequence_id": previous_sequence,
                    },
                    separators=(",", ":"),
                ),
                file=sys.stderr,
                flush=True,
            )
            return 0
        remaining = period - (time.monotonic() - loop_start)
        if remaining > 0.0:
            time.sleep(remaining)
    print(
        json.dumps(
            {
                "event": "hardware_target_stop",
                "reason": "duration_elapsed",
                "last_sequence_id": previous_sequence,
            },
            separators=(",", ":"),
        ),
        file=sys.stderr,
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
