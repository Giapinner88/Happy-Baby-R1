#!/usr/bin/env python3
"""Map Quest JSONL to relative-session R1-A5 arms/head joint targets.

This workstation process performs no DDS writes.  It reuses the coupled IK
from the simulation pilot in ``arms_head`` mode and emits a 12-joint JSONL
stream for the fail-closed robot receiver.
"""
from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1 import (  # noqa: E402
    R1A5WholeUpperBodyOwnership,
    R1TeleopCommand,
    R1TeleopMapper,
    TeleopCalibration,
    TeleopLimits,
    UpperBodyIKConfig,
    Vector3,
)
from teleop.r1.upper_body_kinematics import retarget_nominal  # noqa: E402
from teleop.r1.whole_upper_body import (  # noqa: E402
    WholeUpperBodyIsaacLabSink,
    WholeUpperBodyLiveConfig,
)

JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)


class TargetHandle:
    def __init__(self, initial: tuple[float, ...]) -> None:
        self.positions = np.asarray(initial, dtype=float)

    def joint_positions(self, joint_names: object) -> tuple[float, ...]:
        return tuple(float(value) for value in self.positions)

    def write_joint_targets(self, joint_names: object, positions_rad: object) -> None:
        self.positions = np.asarray(positions_rad, dtype=float)


def _reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_whole_upper_body_live.json",
    )
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T001/config/r1_quest3_sim_v1.json",
    )
    args = parser.parse_args()
    if args.control_hz <= 0.0 or args.duration_s <= 0.0:
        raise SystemExit("control rate and duration must be positive")

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    declared = dict(profile["whole_upper_body"])
    nominal, fixed_waist_yaw = retarget_nominal(
        tuple(float(value) for value in declared["nominal_joint_position_rad"]),
        str(declared.get("body_mode", "waist_yaw")),
        "arms_head",
    )
    config = WholeUpperBodyLiveConfig(
        urdf_path=(ROOT / str(declared["urdf_path"])).resolve(),
        nominal_joint_position_rad=nominal,
        max_joint_velocity_rad_s=min(0.5, float(declared["max_joint_velocity_rad_s"])),
        max_joint_acceleration_rad_s2=min(1.0, float(declared["max_joint_acceleration_rad_s2"])),
        control_dt_s=1.0 / args.control_hz,
        ik=UpperBodyIKConfig(**dict(declared["ik"])),
        source_target_frame=str(declared["source_target_frame"]),
        allow_nonconverged_solution=False,
        body_mode="arms_head",
        fixed_waist_yaw_rad=fixed_waist_yaw,
        seed_restart_residual_m=(
            float(declared["seed_restart_residual_m"])
            if declared.get("seed_restart_residual_m") is not None else None
        ),
        allow_projected_position_solution=bool(declared.get("allow_projected_position_solution", False)),
    )
    handle = TargetHandle(nominal)
    sink = WholeUpperBodyIsaacLabSink(handle, config)
    if tuple(sink.model.joint_names) != JOINT_NAMES:
        raise SystemExit(f"unexpected arms_head joint order: {sink.model.joint_names}")

    mapping = json.loads(args.mapping_config.read_text(encoding="utf-8"))
    translation = mapping.get("calibration", {}).get("translation_m", [0.0, 0.0, 0.0])
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(mapping.get("calibration", {}).get("yaw_rad", 0.0)),
            source_frame=str(mapping.get("source_frame", "quest_headset")),
            robot_frame=str(mapping.get("robot_frame", "r1_base")),
        ),
        TeleopLimits(command_timeout_s=float(mapping.get("command_timeout_s", 0.5)), allow_velocity=False),
    )
    ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head")
    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=_reader, args=(lines,), daemon=True).start()
    deadline = time.monotonic() + args.duration_s
    previous_sequence = -1
    stream_started = False
    period = 1.0 / args.control_hz
    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        newest: R1TeleopCommand | None = None
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
                candidate = R1TeleopCommand.from_dict(json.loads(line))
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                print(f"[WARN] invalid Quest command: {exc}", file=sys.stderr, flush=True)
                continue
            if candidate.sequence_id <= previous_sequence:
                continue
            previous_sequence = candidate.sequence_id
            newest = candidate
        if newest is not None:
            if newest.reset_requested:
                if stream_started:
                    print("[STOP] left-trigger reset requested; restart to establish a new neutral", file=sys.stderr)
                    return 0
                sink.reset_session()
                stream_started = False
            target = mapper.map(newest, time.monotonic())
            if not target.enabled:
                if stream_started:
                    print("[STOP] deadman released or command disabled", file=sys.stderr, flush=True)
                    return 0
            else:
                sink.apply_upper_body(target, ownership.upper_body)
                application = sink.last_application or {}
                if application.get("accepted"):
                    payload = {
                        "schema_version": 1,
                        "sequence_id": newest.sequence_id,
                        "sent_monotonic_s": time.monotonic(),
                        "joint_names": JOINT_NAMES,
                        "positions_rad": [float(value) for value in handle.positions],
                        "solution_kind": application.get("solver_solution_kind"),
                    }
                    try:
                        print(json.dumps(payload, separators=(",", ":")), flush=True)
                    except BrokenPipeError:
                        sys.stdout = open("/dev/null", "w", encoding="utf-8")
                        return 0
                    stream_started = True
        if closed:
            return 0
        remaining = period - (time.monotonic() - loop_start)
        if remaining > 0.0:
            time.sleep(remaining)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
