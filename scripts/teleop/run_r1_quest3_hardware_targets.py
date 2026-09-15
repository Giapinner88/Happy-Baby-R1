#!/usr/bin/env python3
"""Map Quest JSONL to relative-session R1-A5 arms/head joint targets.

This workstation process performs no DDS writes. It emits a 12-joint JSONL
stream for the fail-closed robot receiver, in one of two solver modes.

``--upstream-joint-stream`` is the default hardware path. The joints are solved
further upstream by the unmodified vendor `xr_teleoperate` R1_A5_ArmIK, in the
`tv` environment where CasADi lives, and arrive here already solved; this
process then only enforces the hardware envelope. Nothing here solves.

Without that flag the coupled in-repo IK runs in this process, as it did before
the vendor solver became the baseline. Kept so the two can be compared on one
trace, not because the hardware path needs it.

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

from teleop.r1 import (  # noqa: E402
    R1A5WholeUpperBodyOwnership,
    R1TeleopCommand,
    R1TeleopMapper,
    TeleopCalibration,
    TeleopLimits,
    UpperBodyIKConfig,
    Vector3,
)
from teleop.r1.rate_limit import OnlineJointLimiter  # noqa: E402
from teleop.r1.upper_body_kinematics import (  # noqa: E402
    load_r1_a5_upper_body_model,
    retarget_nominal,
)
from teleop.r1.whole_upper_body import (  # noqa: E402
    WholeUpperBodyIsaacLabSink,
    WholeUpperBodyLiveConfig,
)

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


class TargetHandle:
    def __init__(self, initial: tuple[float, ...]) -> None:
        self.positions = np.asarray(initial, dtype=float)

    def joint_positions(self, joint_names: object) -> tuple[float, ...]:
        return tuple(float(value) for value in self.positions)

    def write_joint_targets(self, joint_names: object, positions_rad: object) -> None:
        self.positions = np.asarray(positions_rad, dtype=float)


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
    parser.add_argument(
        "--upstream-joint-stream",
        dest="upstream_joint_stream",
        action="store_true",
        default=True,
        help="Take joints already solved by the vendor xr_teleoperate IK (default).",
    )
    parser.add_argument(
        "--coupled-ik",
        dest="upstream_joint_stream",
        action="store_false",
        help="Solve here with this repository's coupled IK instead of the vendor solver.",
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
    declared = dict(profile["whole_upper_body"])
    model = load_r1_a5_upper_body_model(
        (ROOT / str(declared["urdf_path"])).resolve(), control_waist_yaw=False
    )
    if tuple(model.joint_names) != JOINT_NAMES:
        raise SystemExit(f"unexpected arms_head joint order: {model.joint_names}")
    limiter = OnlineJointLimiter(
        max_velocity_rad_s=min(args.max_joint_velocity_rad_s, float(declared["max_joint_velocity_rad_s"])),
        max_acceleration_rad_s2=min(
            args.max_joint_acceleration_rad_s2, float(declared["max_joint_acceleration_rad_s2"])
        ),
        dt_s=1.0 / args.control_hz,
        lower_limits=model.lower_limits,
        upper_limits=model.upper_limits,
    )
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
        # T007 may dispatch projected targets in simulation. The hardware gate
        # has not accepted that behavior, so the robot path always refuses it.
        allow_projected_position_solution=False,
    )
    handle = TargetHandle(nominal)
    # The coupled solver is loaded only when it is the one being used: on the
    # vendor path it would otherwise sit in the process holding an IK model that
    # nothing consults, which is exactly the confusion this mode exists to end.
    sink = None if args.upstream_joint_stream else WholeUpperBodyIsaacLabSink(handle, config)

    mapping = json.loads(args.mapping_config.read_text(encoding="utf-8"))
    translation = mapping.get("calibration", {}).get("translation_m", [0.0, 0.0, 0.0])
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(mapping.get("calibration", {}).get("yaw_rad", 0.0)),
            source_frame=str(mapping.get("source_frame", "quest_headset")),
            robot_frame=str(mapping.get("robot_frame", "neutral_waist_yaw_link")),
        ),
        TeleopLimits(command_timeout_s=float(mapping.get("command_timeout_s", 0.5)), allow_velocity=False),
    )
    ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head")
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
                # pipeline chỉ để lập lại mốc. Nay nó xin robot về nominal rồi
                # chốt lại mốc, và phiên chạy tiếp. Cờ đi kèm chính command nên
                # nó tới receiver đúng thứ tự với dòng lệnh, không cần kênh phụ.
                rehome_pending = True
                if sink is not None:
                    sink.reset_session()
                limiter.hold()
            target = mapper.map(newest, time.monotonic())
            if not target.enabled:
                if stream_started:
                    print("[STOP] deadman released or command disabled", file=sys.stderr, flush=True)
                    return 0
            elif args.upstream_joint_stream:
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
                limited = limiter.step(model.clamp(solved))
                payload = {
                    "schema_version": 1,
                    "sequence_id": newest.sequence_id,
                    "sent_monotonic_s": time.monotonic(),
                    "joint_names": RECEIVER_JOINT_NAMES,
                    "positions_rad": to_receiver_order(limited),
                    "solution_kind": "upstream_xr_teleoperate_R1_A5_ArmIK",
                }
                if rehome_pending:
                    payload["rehome"] = True
                    rehome_pending = False
                try:
                    print(json.dumps(payload, separators=(",", ":")), flush=True)
                except BrokenPipeError:
                    sys.stdout = open("/dev/null", "w", encoding="utf-8")
                    return 0
                stream_started = True
            else:
                assert sink is not None
                sink.apply_upper_body(target, ownership.upper_body)
                application = sink.last_application or {}
                if application.get("accepted"):
                    payload = {
                        "schema_version": 1,
                        "sequence_id": newest.sequence_id,
                        "sent_monotonic_s": time.monotonic(),
                        "joint_names": RECEIVER_JOINT_NAMES,
                        # No limiter call here: the coupled sink already limits
                        # with these same ceilings, and a second one in series
                        # would only add lag to a path this change is not meant
                        # to alter.
                        "positions_rad": to_receiver_order(handle.positions),
                        "solution_kind": application.get("solver_solution_kind"),
                    }
                    if rehome_pending:
                        payload["rehome"] = True
                        rehome_pending = False
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
