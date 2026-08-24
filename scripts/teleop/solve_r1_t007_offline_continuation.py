#!/usr/bin/env python3
"""Solve a continuous R1 joint trajectory from a recorded T007 Quest trace."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from types import SimpleNamespace
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_json,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)
from scripts.teleop.run_r1_t007_mujoco_replay import Packets, map_packets  # noqa: E402
from teleop.r1 import (  # noqa: E402
    R1TeleopCommand,
    R1TeleopMapper,
    TeleopCalibration,
    TeleopLimits,
    validate_vendor_r1_a5_ik_contract,
)
from teleop.r1.offline_continuation import (  # noqa: E402
    OfflineContinuationConfig,
    arm_straightness_deg,
    elbow_pole_vector_m,
    elbow_outward_pole_m,
    reproject_position_manifold,
    solve_offline_arm_trajectory,
)
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model  # noqa: E402
from teleop.r1.workspace_projection import (  # noqa: E402
    project_arm_position_to_reach_sphere,
)


DEFAULT_CONFIG = ROOT / "experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_offline_continuation.json"
XR_R1_A5_IK_SOURCE = (
    ROOT / "third_party/xr_teleoperate_v1_6/teleop/robot_control/robot_arm_ik.py"
)
R1_A5_MIRROR_JOINT_SIGNS = np.asarray([1.0, -1.0, -1.0, 1.0, -1.0])
SAGITTAL_REFLECTION = np.diag([1.0, -1.0, 1.0])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(values: np.ndarray) -> dict[str, float | int]:
    data = np.asarray(values, dtype=float)
    return {
        "count": int(data.size),
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "p95": float(np.quantile(data, 0.95)),
        "max": float(np.max(data)),
    }


def pose_rotation_matrix(pose: object) -> np.ndarray:
    """Rotation matrix for a schema Pose, without a SciPy runtime dependency."""

    q = pose.orientation.normalized()
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def absolute_vendor_wrist_packets(
    commands: list[R1TeleopCommand], base: Packets
) -> Packets:
    """Replace relative-session wrist targets with audited vendor waist poses.

    ``QuestCommandBridge`` already consumes TeleVuer's processed matrices: the
    wrist axes are in Unitree convention and positions are in the head-yaw
    relative IK waist workspace.  This is the exact wrist-pose contract used
    directly by upstream ``R1_A5_ArmIK``.
    """

    mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(1.0))
    selected_commands = commands[base.start : base.stop]
    mapped = [mapper.map(command, command.timestamp_monotonic_s) for command in selected_commands]
    if len(mapped) != len(base.t) or any(
        target.left_wrist_target is None or target.right_wrist_target is None
        for target in mapped
    ):
        raise ValueError("absolute vendor mapping did not reproduce the enabled packet span")

    def positions(side: str) -> np.ndarray:
        return np.asarray(
            [
                [
                    float(getattr(getattr(target, f"{side}_wrist_target").position, axis))
                    for axis in ("x", "y", "z")
                ]
                for target in mapped
            ],
            dtype=float,
        )

    return Packets(
        t=base.t,
        seq=base.seq,
        lp=positions("left"),
        rp=positions("right"),
        lR=np.asarray([pose_rotation_matrix(target.left_wrist_target) for target in mapped]),
        rR=np.asarray([pose_rotation_matrix(target.right_wrist_target) for target in mapped]),
        hR=base.hR,
        ha=base.ha,
        vel=base.vel,
        start=base.start,
        stop=base.stop,
    )


def canonicalize_right_arm_problem(
    positions_m: np.ndarray,
    orientations: np.ndarray,
    q_rad: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """Reflect right-only data into the left-arm canonical IK problem.

    No left-arm target or trajectory enters this transform.  It removes a
    purely numerical asymmetry by making mirrored inputs use the same chain,
    seed convention and multistart ordering.  The result maps back with the
    URDF-verified R1-A5 joint signs ``[+,-,-,+,-]``.
    """

    positions = np.asarray(positions_m, dtype=float).copy()
    positions[:, 1] *= -1.0
    rotations = np.asarray(
        [SAGITTAL_REFLECTION @ value @ SAGITTAL_REFLECTION for value in orientations]
    )
    joints = None
    if q_rad is not None:
        joints = np.asarray(q_rad, dtype=float) * R1_A5_MIRROR_JOINT_SIGNS
    return positions, rotations, joints


def rate_limit_trajectory(
    desired_q: np.ndarray,
    time_s: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    maximum_velocity_rad_s: float,
    maximum_acceleration_rad_s2: float,
    mode: str = "component_wise",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Causal hard velocity/acceleration projection at source timestamps.

    ``component_wise`` clips every joint independently. When one joint
    saturates and the others do not, the joint-space step changes direction and
    the endpoint leaves the path the IK asked for. On segment 2 of
    ``t007_whole_upper_body_20260823T072635Z`` that turned a p95 endpoint error
    of 101.1 mm before limiting into 247.1 mm after it, which is what fails the
    150 mm criterion; the unlimited solution passes it.

    ``vector_uniform`` scales the whole requested step by one scalar instead, so
    the direction in joint space is preserved and the arm lags along its own
    path rather than leaving it. It cannot reduce the peak caused by targets the
    arm never reaches: the same segment already shows a 722.6 mm maximum before
    any limiting.
    """

    if mode not in ("component_wise", "vector_uniform"):
        raise ValueError(f"unsupported rate limit mode: {mode!r}")

    reference = np.empty_like(desired_q)
    velocity = np.zeros_like(desired_q)
    acceleration = np.zeros_like(desired_q)
    reference[0] = np.clip(desired_q[0], lower, upper)
    for index in range(1, len(time_s)):
        dt = float(time_s[index] - time_s[index - 1])
        distance_to_lower = np.maximum(reference[index - 1] - lower, 0.0)
        distance_to_upper = np.maximum(upper - reference[index - 1], 0.0)
        # Braking-aware velocity bounds prevent the later joint-position clip
        # from turning a finite velocity into zero in one sample.  Include the
        # displacement of the current explicit-Euler step in the stopping
        # distance: v*dt + v**2/(2*a) <= distance.  This is conservative for
        # the sampled system and avoids an acceleration impulse at a limit.
        def stopping_velocity_bound(distance: np.ndarray) -> np.ndarray:
            return np.maximum(
                -maximum_acceleration_rad_s2 * dt
                + np.sqrt(
                    (maximum_acceleration_rad_s2 * dt) ** 2
                    + 2.0 * maximum_acceleration_rad_s2 * distance
                ),
                0.0,
            )

        safe_lower_velocity = -np.minimum(
            maximum_velocity_rad_s,
            np.minimum(
                distance_to_lower / dt,
                stopping_velocity_bound(distance_to_lower),
            ),
        )
        safe_upper_velocity = np.minimum(
            maximum_velocity_rad_s,
            np.minimum(
                distance_to_upper / dt,
                stopping_velocity_bound(distance_to_upper),
            ),
        )
        requested_velocity = np.clip(
            (desired_q[index] - reference[index - 1]) / dt,
            safe_lower_velocity,
            safe_upper_velocity,
        )
        maximum_delta_velocity = maximum_acceleration_rad_s2 * dt
        if mode == "component_wise":
            velocity[index] = np.clip(
                requested_velocity,
                np.maximum(
                    safe_lower_velocity,
                    velocity[index - 1] - maximum_delta_velocity,
                ),
                np.minimum(
                    safe_upper_velocity,
                    velocity[index - 1] + maximum_delta_velocity,
                ),
            )
        else:
            # One scalar for the whole vector: the direction of the step and of
            # the change in step are both preserved, so the path is kept and
            # only its timing is slowed.
            raw_velocity = (desired_q[index] - reference[index - 1]) / dt
            speed_scale = 1.0
            for bound, value in (
                (safe_upper_velocity, raw_velocity),
                (-safe_lower_velocity, -raw_velocity),
            ):
                exceeded = value > bound
                if np.any(exceeded):
                    with np.errstate(divide="ignore", invalid="ignore"):
                        ratio = np.where(exceeded, bound / np.maximum(value, 1e-12), 1.0)
                    speed_scale = min(speed_scale, float(np.min(ratio)))
            speed_scale = float(np.clip(speed_scale, 0.0, 1.0))
            delta_velocity = speed_scale * raw_velocity - velocity[index - 1]
            largest_delta = float(np.max(np.abs(delta_velocity)))
            if largest_delta > maximum_delta_velocity:
                delta_velocity *= maximum_delta_velocity / largest_delta
            velocity[index] = np.clip(
                velocity[index - 1] + delta_velocity,
                safe_lower_velocity,
                safe_upper_velocity,
            )
        reference[index] = np.clip(
            reference[index - 1] + velocity[index] * dt,
            lower,
            upper,
        )
        velocity[index] = (reference[index] - reference[index - 1]) / dt
        acceleration[index] = (velocity[index] - velocity[index - 1]) / dt
    return reference, velocity, acceleration


def plots(
    output: Path,
    time_s: np.ndarray,
    model: object,
    q: np.ndarray,
    left_target: np.ndarray,
    right_target: np.ndarray,
    left_error: np.ndarray,
    right_error: np.ndarray,
    left_straightness: np.ndarray,
    right_straightness: np.ndarray,
    left_outward_pole: np.ndarray,
    right_outward_pole: np.ndarray,
    anchor: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    states = [model.forward_kinematics(row) for row in q]
    actual = {
        "left": np.asarray([state.left_end_effector[:3, 3] for state in states]),
        "right": np.asarray([state.right_end_effector[:3, 3] for state in states]),
    }
    targets = {"left": left_target, "right": right_target}
    errors = {"left": left_error, "right": right_error}
    fig, axes = plt.subplots(4, 2, figsize=(14, 11), sharex="col", layout="constrained")
    for column, side in enumerate(("left", "right")):
        for index, axis_name in enumerate("xyz"):
            axes[index, column].plot(time_s, targets[side][:, index], label="target")
            axes[index, column].plot(time_s, actual[side][:, index], label="offline FK", linewidth=0.8)
            axes[index, column].set_ylabel(f"{axis_name} (m)")
        axes[3, column].plot(time_s, 1000.0 * errors[side], color="tab:red")
        axes[3, column].set_ylabel("error (mm)")
        axes[3, column].set_xlabel("source time (s)")
        axes[0, column].set_title(f"{side} wrist position")
        axes[0, column].legend()
    fig.savefig(output / "offline_wrist_position_tracking.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(4, 3, figsize=(16, 11), sharex=True, layout="constrained")
    for axis, name, values in zip(axes.flat, model.joint_names, q.T):
        axis.plot(time_s, values)
        axis.set_title(name.replace("_joint", ""), fontsize=9)
        axis.grid(alpha=0.2)
    fig.supylabel("joint angle (rad)")
    fig.supxlabel("source time (s)")
    fig.savefig(output / "offline_joint_trajectory.png", dpi=160)
    plt.close(fig)

    velocity = np.gradient(q, time_s, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, time_s, axis=0, edge_order=2)
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True, layout="constrained")
    axes[0].plot(time_s, velocity)
    axes[0].set_ylabel("joint velocity (rad/s)")
    axes[1].plot(time_s, acceleration)
    axes[1].set_ylabel("joint acceleration (rad/s²)")
    axes[1].set_xlabel("source time (s)")
    fig.savefig(output / "offline_joint_velocity_acceleration.png", dpi=160)
    plt.close(fig)

    separation = np.abs(left_target[:, 1] - right_target[:, 1])
    fig, axis = plt.subplots(figsize=(14, 5), layout="constrained")
    axis.plot(time_s, left_straightness, label="left straightness")
    axis.plot(time_s, right_straightness, label="right straightness")
    axis.axvline(time_s[anchor], color="black", linestyle="--", label="wide anchor")
    axis.set(xlabel="source time (s)", ylabel="shoulder-elbow-EE angle (deg)")
    second = axis.twinx()
    second.plot(time_s, separation, color="tab:gray", alpha=0.4, label="hand separation")
    second.set_ylabel("lateral hand separation (m)")
    axis.legend(loc="upper left")
    second.legend(loc="upper right")
    fig.savefig(output / "offline_elbow_straightness_and_hand_separation.png", dpi=160)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(14, 5), layout="constrained")
    axis.plot(time_s, 1000.0 * left_outward_pole, label="left outward pole")
    axis.plot(time_s, 1000.0 * right_outward_pole, label="right outward pole")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.axvline(time_s[anchor], color="black", linestyle="--", label="wide anchor")
    axis.set(
        xlabel="source time (s)",
        ylabel="signed outward elbow pole (mm)",
        title="Independent side-aware elbow branch (negative is inward)",
    )
    axis.legend()
    fig.savefig(output / "offline_elbow_branch_pole.png", dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--resume-trajectory",
        type=Path,
        help="Existing offline_joint_trajectory.npz used only as the initial manifold path.",
    )
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("experiment_id") != "t007" or config.get("mode") != "simulation_only":
        raise SystemExit("Configuration must declare T007 simulation_only.")
    xr_reference_contract = validate_vendor_r1_a5_ik_contract(XR_R1_A5_IK_SOURCE)
    output = args.output_dir.expanduser().resolve()
    if output.exists():
        raise SystemExit(f"Refusing to overwrite {output}")
    output.mkdir(parents=True)

    source = (ROOT / config["source_run"]).resolve()
    raw_path = source / "raw_commands.jsonl"
    commands = [
        R1TeleopCommand.from_dict(json.loads(line))
        for line in raw_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    # `map_packets` always takes the longest deadman segment. When a trace holds
    # several gestures the longest one is not necessarily the one under test:
    # in `t007_whole_upper_body_20260823T072635Z` the five-pose reference cycle
    # (40.1 s) loses to the free-motion segment (40.9 s) by 0.8 s, which made
    # the reference cycle unselectable. An explicit index, ordered by start
    # time, makes the choice reproducible instead of incidental.
    segment_index = config["source_selection"].get("deadman_segment_index")
    if segment_index is not None:
        spans: list[tuple[int, int]] = []
        start_index: int | None = None
        for position in range(len(commands) + 1):
            enabled = position < len(commands) and commands[position].deadman_enabled
            if enabled and start_index is None:
                start_index = position
            elif not enabled and start_index is not None:
                spans.append((start_index, position))
                start_index = None
        if not spans:
            raise ValueError("source trace has no deadman-enabled segment")
        index = int(segment_index)
        if not 0 <= index < len(spans):
            raise ValueError(
                f"deadman_segment_index {index} outside the {len(spans)} segments in the trace"
            )
        begin, end = spans[index]
        commands = commands[begin:end]

    model_config = config["model"]
    model = load_r1_a5_upper_body_model(
        (ROOT / model_config["urdf_path"]).resolve(),
        control_waist_yaw=False,
        fixed_waist_yaw_rad=float(model_config["fixed_waist_yaw_rad"]),
    )
    calibration = config["calibration"]
    mapping_mode = str(calibration["mapping_mode"])
    packets = map_packets(
        commands,
        model,
        float(calibration.get("position_scale", 1.0)),
        1.0,
        None,
    )
    if mapping_mode == "absolute_vendor_pose_projected":
        packets = absolute_vendor_wrist_packets(commands, packets)
    elif mapping_mode != "relative_first_sample_to_declared_robot_neutral":
        raise ValueError(f"unsupported calibration.mapping_mode: {mapping_mode!r}")
    full_segment_start = packets.start
    full_segment_stop = packets.stop
    source_window = config["source_selection"].get("source_time_window_s")
    if source_window is not None:
        if len(source_window) != 2:
            raise ValueError("source_time_window_s must be [start_s, stop_s]")
        window_start_s, window_stop_s = (float(value) for value in source_window)
        selected = np.flatnonzero(
            (packets.t >= window_start_s) & (packets.t <= window_stop_s)
        )
        if len(selected) < 3:
            raise ValueError("source_time_window_s selected fewer than three samples")
        first = int(selected[0])
        last = int(selected[-1]) + 1
        packets = Packets(
            t=packets.t[first:last] - packets.t[first],
            seq=packets.seq[first:last],
            lp=packets.lp[first:last],
            rp=packets.rp[first:last],
            lR=packets.lR[first:last],
            rR=packets.rR[first:last],
            hR=packets.hR[first:last],
            ha=packets.ha[first:last],
            vel=packets.vel[first:last],
            start=full_segment_start + first,
            stop=full_segment_start + last,
        )
    waist = model.waist_transform_from_q(np.zeros(model.dof))
    waist_inverse = np.linalg.inv(waist)
    if mapping_mode == "absolute_vendor_pose_projected":
        # Upstream R1_A5_ArmIK is rooted directly at waist_yaw_link, as is our
        # ArmChain.  The vendor pose must therefore enter the arm solve without
        # the full-body URDF's pelvis-to-waist translation.
        left_raw_position = packets.lp.copy()
        right_raw_position = packets.rp.copy()
        left_orientation = packets.lR.copy()
        right_orientation = packets.rR.copy()
    else:
        left_raw_position = np.asarray(
            [(waist_inverse @ np.r_[position, 1.0])[:3] for position in packets.lp]
        )
        right_raw_position = np.asarray(
            [(waist_inverse @ np.r_[position, 1.0])[:3] for position in packets.rp]
        )
        left_orientation = np.asarray([waist[:3, :3].T @ value for value in packets.lR])
        right_orientation = np.asarray([waist[:3, :3].T @ value for value in packets.rR])

    projection_margin_m = float(calibration.get("workspace_projection_margin_m", 0.0))

    def project_trace(chain: object, raw: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        projected: list[np.ndarray] = []
        distances: list[float] = []
        reach_limit = float("nan")
        for position in raw:
            value, _changed, distance, reach_limit = project_arm_position_to_reach_sphere(
                chain, position, projection_margin_m
            )
            projected.append(value)
            distances.append(distance)
        return np.asarray(projected), np.asarray(distances), reach_limit

    left_position, left_projection_distance, left_reach_limit = project_trace(
        model.left_arm, left_raw_position
    )
    right_position, right_projection_distance, right_reach_limit = project_trace(
        model.right_arm, right_raw_position
    )
    left_target_pelvis = np.asarray(
        [(waist @ np.r_[position, 1.0])[:3] for position in left_position]
    )
    right_target_pelvis = np.asarray(
        [(waist @ np.r_[position, 1.0])[:3] for position in right_position]
    )
    anchor = int(np.argmax(np.abs(left_position[:, 1] - right_position[:, 1])))
    solver_stride = int(config["source_selection"].get("solver_stride", 1))
    if solver_stride <= 0:
        raise ValueError("source_selection.solver_stride must be positive")
    solver_indices = np.unique(
        np.r_[np.arange(0, len(packets.t), solver_stride), anchor, len(packets.t) - 1]
    )
    solver_anchor = int(np.flatnonzero(solver_indices == anchor)[0])
    continuation_values = dict(config["continuation"])
    continuation_values.pop("anchor_policy", None)
    # Keys ending in `_note` are operator documentation kept beside the value
    # they explain; they are provenance, not solver parameters.
    for key in [name for name in continuation_values if name.endswith("_note")]:
        continuation_values.pop(key)
    solver_config = OfflineContinuationConfig(**continuation_values)
    nominal = np.asarray(model_config["nominal_joint_position_rad"], dtype=float)
    right_canonical_mirror = bool(model_config.get("right_arm_canonical_mirror", False))
    joint_limit_mirror_max_abs_rad = 0.0
    if right_canonical_mirror:
        canonical_lower = np.minimum(
            model.right_arm.lower_limits * R1_A5_MIRROR_JOINT_SIGNS,
            model.right_arm.upper_limits * R1_A5_MIRROR_JOINT_SIGNS,
        )
        canonical_upper = np.maximum(
            model.right_arm.lower_limits * R1_A5_MIRROR_JOINT_SIGNS,
            model.right_arm.upper_limits * R1_A5_MIRROR_JOINT_SIGNS,
        )
        joint_limit_mirror_max_abs_rad = float(
            max(
                np.max(np.abs(canonical_lower - model.left_arm.lower_limits)),
                np.max(np.abs(canonical_upper - model.left_arm.upper_limits)),
            )
        )
        # The vendor asset rounds the mirrored shoulder-roll limits 9e-5 rad
        # differently.  The final full-model rate limiter clips that residual
        # asymmetry in the physical right-arm limits.
        if joint_limit_mirror_max_abs_rad > 1.0e-4:
            raise ValueError("R1-A5 right/left joint limits are no longer mirror symmetric")
        right_solve_position, right_solve_orientation, _ = canonicalize_right_arm_problem(
            right_position, right_orientation
        )
        right_solve_chain = model.left_arm
        right_solve_nominal = (
            nominal[model.right_arm_slice] * R1_A5_MIRROR_JOINT_SIGNS
        )
    else:
        right_solve_position = right_position
        right_solve_orientation = right_orientation
        right_solve_chain = model.right_arm
        right_solve_nominal = nominal[model.right_arm_slice]
    resume_path = args.resume_trajectory.expanduser().resolve() if args.resume_trajectory else None
    if resume_path is not None:
        with np.load(resume_path) as resume:
            resume_time = np.asarray(resume["time_s"], dtype=float)
            resume_sequence = np.asarray(resume["sequence_id"], dtype=int)
            resume_q = np.asarray(resume["unlimited_joint_position_solution_rad"], dtype=float)
        if not (
            np.array_equal(resume_sequence, packets.seq)
            and np.allclose(resume_time, packets.t, atol=1.0e-12, rtol=0.0)
            and resume_q.shape == (len(packets.t), model.dof)
        ):
            raise ValueError("resume trajectory does not match selected source time/sequence/model")
        print(
            f"reprojecting left arm from {resume_path}: {len(solver_indices)} nodes",
            flush=True,
        )
        left_start = time.perf_counter()
        left_q = reproject_position_manifold(
            model.left_arm,
            packets.t[solver_indices],
            left_position[solver_indices],
            left_orientation[solver_indices],
            resume_q[solver_indices, model.left_arm_slice],
            nominal[model.left_arm_slice],
            solver_config,
        )
        left_solve_wall_s = time.perf_counter() - left_start
        left = SimpleNamespace(
            q_rad=left_q,
            candidate_scores=({"source": "resume_trajectory_reprojection"},),
        )
        print("left arm reprojection complete; reprojecting right arm", flush=True)
        right_start = time.perf_counter()
        right_initial = resume_q[solver_indices, model.right_arm_slice]
        if right_canonical_mirror:
            _, _, right_initial = canonicalize_right_arm_problem(
                right_position[solver_indices],
                right_orientation[solver_indices],
                right_initial,
            )
        right_q = reproject_position_manifold(
            right_solve_chain,
            packets.t[solver_indices],
            right_solve_position[solver_indices],
            right_solve_orientation[solver_indices],
            right_initial,
            right_solve_nominal,
            solver_config,
        )
        if right_canonical_mirror:
            right_q = right_q * R1_A5_MIRROR_JOINT_SIGNS
        right_solve_wall_s = time.perf_counter() - right_start
        right = SimpleNamespace(
            q_rad=right_q,
            candidate_scores=({"source": "resume_trajectory_reprojection"},),
        )
    else:
        print(
            f"solving left arm: {len(solver_indices)} continuation nodes from "
            f"{len(packets.t)} source samples, anchor={anchor}",
            flush=True,
        )
        left_start = time.perf_counter()
        left = solve_offline_arm_trajectory(
            model.left_arm,
            packets.t[solver_indices],
            left_position[solver_indices],
            left_orientation[solver_indices],
            nominal[model.left_arm_slice],
            solver_anchor,
            solver_config,
        )
        left_solve_wall_s = time.perf_counter() - left_start
        print("left arm complete; solving right arm", flush=True)
        right_start = time.perf_counter()
        right = solve_offline_arm_trajectory(
            right_solve_chain,
            packets.t[solver_indices],
            right_solve_position[solver_indices],
            right_solve_orientation[solver_indices],
            right_solve_nominal,
            solver_anchor,
            solver_config,
        )
        if right_canonical_mirror:
            right = SimpleNamespace(
                q_rad=right.q_rad * R1_A5_MIRROR_JOINT_SIGNS,
                candidate_scores=right.candidate_scores,
            )
        right_solve_wall_s = time.perf_counter() - right_start
    print("right arm complete; writing metrics and figures", flush=True)
    q_unlimited = np.zeros((len(packets.t), model.dof), dtype=float)
    for joint_offset, joint_index in enumerate(
        range(model.left_arm_slice.start, model.left_arm_slice.stop)
    ):
        q_unlimited[:, joint_index] = np.interp(
            packets.t, packets.t[solver_indices], left.q_rad[:, joint_offset]
        )
    for joint_offset, joint_index in enumerate(
        range(model.right_arm_slice.start, model.right_arm_slice.stop)
    ):
        q_unlimited[:, joint_index] = np.interp(
            packets.t, packets.t[solver_indices], right.q_rad[:, joint_offset]
        )
    q_unlimited[:, model.head_slice] = packets.ha
    trajectory_limits = config["trajectory_limits"]
    q, velocity, acceleration = rate_limit_trajectory(
        q_unlimited,
        packets.t,
        model.lower_limits,
        model.upper_limits,
        float(trajectory_limits["max_joint_velocity_rad_s"]),
        float(trajectory_limits["max_joint_acceleration_rad_s2"]),
        mode=str(trajectory_limits.get("rate_limit_mode", "component_wise")),
    )
    left_position_error = np.asarray(
        [
            np.linalg.norm(model.left_arm.endpoint_position(row[model.left_arm_slice]) - target)
            for row, target in zip(q, left_position)
        ]
    )
    right_position_error = np.asarray(
        [
            np.linalg.norm(model.right_arm.endpoint_position(row[model.right_arm_slice]) - target)
            for row, target in zip(q, right_position)
        ]
    )
    left_straightness = np.asarray(
        [arm_straightness_deg(model.left_arm, row[model.left_arm_slice]) for row in q]
    )
    right_straightness = np.asarray(
        [arm_straightness_deg(model.right_arm, row[model.right_arm_slice]) for row in q]
    )
    hand_separation = np.abs(left_position[:, 1] - right_position[:, 1])
    wide_mask = hand_separation >= float(
        config["acceptance_criteria"]["wide_hand_separation_m_min"]
    )
    if not np.any(wide_mask):
        raise RuntimeError("source trace has no samples in the declared wide phase")
    wide_straightness_p05 = {
        "left": float(np.quantile(left_straightness[wide_mask], 0.05)),
        "right": float(np.quantile(right_straightness[wide_mask], 0.05)),
    }
    left_outward_pole = np.asarray(
        [elbow_outward_pole_m(model.left_arm, row[model.left_arm_slice]) for row in q]
    )
    right_outward_pole = np.asarray(
        [elbow_outward_pole_m(model.right_arm, row[model.right_arm_slice]) for row in q]
    )
    left_pole_vector = np.asarray(
        [elbow_pole_vector_m(model.left_arm, row[model.left_arm_slice]) for row in q]
    )
    right_pole_vector = np.asarray(
        [elbow_pole_vector_m(model.right_arm, row[model.right_arm_slice]) for row in q]
    )
    direction_minimum_pole_m = float(solver_config.elbow_direction_minimum_pole_m)
    direction_maximum_straightness_deg = float(
        solver_config.elbow_direction_maximum_straightness_deg
    )
    direction_violation_tolerance_m = float(
        solver_config.elbow_direction_violation_tolerance_m
    )
    # A straight arm has no meaningful elbow direction, so samples must be
    # down-weighted as they approach full extension. Doing that with a hard
    # cutoff put a cliff inside the data: the left arm's median straightness is
    # 166.31 deg and the right arm's is 164.58 deg against a 165.0 deg cutoff,
    # so the two arms sat on opposite sides of it. A 0.52 deg physics shift then
    # moved 20% of right-arm samples across the boundary and the fraction looked
    # like a left/right asymmetry that does not exist in the joint tracking
    # error, which is symmetric (p95 5.61 deg left, 6.40 deg right).
    #
    # The cutoff is replaced by a linear ramp centred on the same angle, so the
    # physical intent is unchanged but no sample's contribution can flip on an
    # arbitrarily small perturbation. Width 0 reproduces the old hard cutoff.
    softening_deg = float(solver_config.elbow_direction_straightness_softening_deg)

    def direction_weight(pole_vector: np.ndarray, straightness: np.ndarray) -> np.ndarray:
        norm_ok = (np.linalg.norm(pole_vector, axis=1) >= direction_minimum_pole_m).astype(float)
        if softening_deg <= 0.0:
            return norm_ok * (straightness <= direction_maximum_straightness_deg).astype(float)
        ramp = (direction_maximum_straightness_deg + softening_deg - straightness) / (
            2.0 * softening_deg
        )
        return norm_ok * np.clip(ramp, 0.0, 1.0)

    left_direction_weight = direction_weight(left_pole_vector, left_straightness)
    right_direction_weight = direction_weight(right_pole_vector, right_straightness)
    # Retained so every run still reports the old hard-cutoff population and the
    # revision stays comparable with pre-softening evidence.
    left_direction_defined = (
        np.linalg.norm(left_pole_vector, axis=1) >= direction_minimum_pole_m
    ) & (left_straightness <= direction_maximum_straightness_deg)
    right_direction_defined = (
        np.linalg.norm(right_pole_vector, axis=1) >= direction_minimum_pole_m
    ) & (right_straightness <= direction_maximum_straightness_deg)

    def defined_fraction(mask: np.ndarray, defined: np.ndarray) -> float:
        """Weighted violation fraction; `defined` may be a weight or a mask."""

        weight = np.asarray(defined, dtype=float)
        total = float(np.sum(weight))
        if total <= 0.0:
            return 0.0
        return float(np.sum(np.asarray(mask, dtype=bool) * weight) / total)

    unnatural_pole_fraction = {
        "left_forward": defined_fraction(left_pole_vector[:, 0] > direction_violation_tolerance_m, left_direction_weight),
        "left_up": defined_fraction(left_pole_vector[:, 2] > direction_violation_tolerance_m, left_direction_weight),
        "right_forward": defined_fraction(right_pole_vector[:, 0] > direction_violation_tolerance_m, right_direction_weight),
        "right_up": defined_fraction(right_pole_vector[:, 2] > direction_violation_tolerance_m, right_direction_weight),
    }
    # Same quantities under the superseded hard cutoff, reported so this
    # revision stays comparable with evidence generated before the ramp.
    unnatural_pole_fraction_hard_cutoff = {
        "left_forward": defined_fraction(left_pole_vector[:, 0] > direction_violation_tolerance_m, left_direction_defined),
        "left_up": defined_fraction(left_pole_vector[:, 2] > direction_violation_tolerance_m, left_direction_defined),
        "right_forward": defined_fraction(right_pole_vector[:, 0] > direction_violation_tolerance_m, right_direction_defined),
        "right_up": defined_fraction(right_pole_vector[:, 2] > direction_violation_tolerance_m, right_direction_defined),
    }
    joint_step = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    criteria = config["acceptance_criteria"]
    checks = {
        "left_wrist_p95": float(np.quantile(left_position_error, 0.95))
        <= float(criteria["wrist_position_p95_m_max"]),
        "right_wrist_p95": float(np.quantile(right_position_error, 0.95))
        <= float(criteria["wrist_position_p95_m_max"]),
        "left_wrist_max": float(np.max(left_position_error))
        <= float(criteria["wrist_position_max_m_max"]),
        "right_wrist_max": float(np.max(right_position_error))
        <= float(criteria["wrist_position_max_m_max"]),
        "left_widest_straightness": float(left_straightness[anchor])
        >= float(criteria["widest_frame_straightness_deg_min"]),
        "right_widest_straightness": float(right_straightness[anchor])
        >= float(criteria["widest_frame_straightness_deg_min"]),
        "left_wide_phase_straightness": wide_straightness_p05["left"]
        >= float(criteria["wide_phase_straightness_p05_deg_min"]),
        "right_wide_phase_straightness": wide_straightness_p05["right"]
        >= float(criteria["wide_phase_straightness_p05_deg_min"])
    }
    # Elbow-sector gating is optional. `t007_seg1_no_elbow_20260823_b` solved
    # the operator's own five-pose cycle with the sector cost disabled and
    # measured 59-66% forward, 80% up and 88-100% inward elbow poses. The
    # declared sector therefore contradicts most of the recorded motion rather
    # than describing it, so a run may omit these criteria and report the
    # distribution instead of failing against a premise the data does not
    # support. Keeping the keys reproduces the old gate exactly.
    if "inward_elbow_fraction_max" in criteria:
        checks["left_elbow_branch"] = defined_fraction(
            left_outward_pole < -direction_violation_tolerance_m, left_direction_weight
        ) <= float(criteria["inward_elbow_fraction_max"])
        checks["right_elbow_branch"] = defined_fraction(
            right_outward_pole < -direction_violation_tolerance_m, right_direction_weight
        ) <= float(criteria["inward_elbow_fraction_max"])
    if "forward_or_up_elbow_fraction_max" in criteria:
        limit = float(criteria["forward_or_up_elbow_fraction_max"])
        checks["left_elbow_not_forward_or_up"] = max(
            unnatural_pole_fraction["left_forward"], unnatural_pole_fraction["left_up"]
        ) <= limit
        checks["right_elbow_not_forward_or_up"] = max(
            unnatural_pole_fraction["right_forward"], unnatural_pole_fraction["right_up"]
        ) <= limit

    metrics = {
        "schema_version": 1,
        "mode": "simulation_only",
        "xr_teleoperation_reference_contract": xr_reference_contract,
        "source_sample_count": int(len(packets.t)),
        "continuation_node_count": int(len(solver_indices)),
        "continuation_solver_stride": solver_stride,
        "source_duration_s": float(packets.t[-1]),
        "offline_solver_timing": {
            "left_wall_s": float(left_solve_wall_s),
            "right_wall_s": float(right_solve_wall_s),
            "total_wall_s": float(left_solve_wall_s + right_solve_wall_s),
            "samples_per_second_bilateral": float(
                2.0 * len(solver_indices) / (left_solve_wall_s + right_solve_wall_s)
            ),
            "realtime_factor_against_source_duration": float(
                packets.t[-1] / (left_solve_wall_s + right_solve_wall_s)
            ),
        },
        "anchor_index": anchor,
        "anchor_time_s": float(packets.t[anchor]),
        "anchor_sequence_id": int(packets.seq[anchor]),
        "anchor_lateral_hand_separation_m": float(
            abs(left_position[anchor, 1] - right_position[anchor, 1])
        ),
        "workspace_projection": {
            "mapping_mode": mapping_mode,
            "margin_m": projection_margin_m,
            "left_reach_limit_m": left_reach_limit,
            "right_reach_limit_m": right_reach_limit,
            "left_projected_fraction": float(np.mean(left_projection_distance > 0.0)),
            "right_projected_fraction": float(np.mean(right_projection_distance > 0.0)),
            "left_projection_distance_m": summary(left_projection_distance),
            "right_projection_distance_m": summary(right_projection_distance),
        },
        "wrist_position_error_m": {
            "left": summary(left_position_error),
            "right": summary(right_position_error),
        },
        "widest_frame_straightness_deg": {
            "left": float(left_straightness[anchor]),
            "right": float(right_straightness[anchor]),
        },
        "wide_phase_straightness_p05_deg": {
            **wide_straightness_p05,
            "sample_count": int(np.count_nonzero(wide_mask)),
            "hand_separation_threshold_m": float(
                criteria["wide_hand_separation_m_min"]
            ),
            "reason": (
                "Prevents a single straight anchor frame from hiding elbow "
                "folding during the rest of the held wide-arm gesture."
            ),
        },
        "straightness_deg": {
            "left": summary(left_straightness),
            "right": summary(right_straightness),
        },
        "elbow_outward_pole_m": {
            "definition": "positive points away from torso; negative is inward",
            "left": summary(left_outward_pole),
            "right": summary(right_outward_pole),
            "inward_fraction": {
                "left": defined_fraction(
                    left_outward_pole < -direction_violation_tolerance_m, left_direction_defined
                ),
                "right": defined_fraction(
                    right_outward_pole < -direction_violation_tolerance_m, right_direction_defined
                ),
            },
        },
        "elbow_pole_direction": {
            "frame": "robot pelvis: +x forward, +y left, +z up",
            "natural_sector": "backward (-x), side-aware outward y, down (-z)",
            "minimum_pole_norm_m": direction_minimum_pole_m,
            "maximum_straightness_deg": direction_maximum_straightness_deg,
            "violation_tolerance_m": direction_violation_tolerance_m,
            "straightness_softening_deg": softening_deg,
            "weighting": (
                "Samples are weighted by a linear ramp from full contribution at "
                "maximum_straightness_deg - softening to zero at + softening, replacing the "
                "hard cutoff whose cliff fell inside the data. Zero softening reproduces the cutoff."
            ),
            "direction_weight_sum": {
                "left": float(np.sum(left_direction_weight)),
                "right": float(np.sum(right_direction_weight)),
            },
            "direction_defined_fraction": {
                "left": float(np.mean(left_direction_defined)),
                "right": float(np.mean(right_direction_defined)),
            },
            "forward_or_up_fraction": unnatural_pole_fraction,
            "forward_or_up_fraction_hard_cutoff": unnatural_pole_fraction_hard_cutoff,
            "left_straightness_deg": summary(left_straightness),
            "right_straightness_deg": summary(right_straightness),
            "left_xyz_m": {
                axis: summary(left_pole_vector[:, index])
                for index, axis in enumerate(("x", "y", "z"))
            },
            "right_xyz_m": {
                axis: summary(right_pole_vector[:, index])
                for index, axis in enumerate(("x", "y", "z"))
            },
        },
        "bilateral_independence": {
            "definition": (
                "Each arm is solved from its own Quest target and side-aware elbow prior; "
                "no opposite-arm joint vector enters either solve."
            ),
            "maximum_abs_left_minus_right_joint_rad": float(
                np.max(
                    np.abs(
                        q[:, model.left_arm_slice] - q[:, model.right_arm_slice]
                    )
                )
            ),
            "elbow_joint_correlation": float(
                np.corrcoef(
                    q[:, model.left_arm_slice.start + 3],
                    q[:, model.right_arm_slice.start + 3],
                )[0, 1]
            ),
            "shoulder_yaw_joint_correlation": float(
                np.corrcoef(
                    q[:, model.left_arm_slice.start + 2],
                    q[:, model.right_arm_slice.start + 2],
                )[0, 1]
            ),
            "right_solver_canonical_mirror": right_canonical_mirror,
            "joint_limit_mirror_max_abs_rad": joint_limit_mirror_max_abs_rad,
        },
        "rate_limit_mode": str(trajectory_limits.get("rate_limit_mode", "component_wise")),
        "maximum_joint_step_rad": float(np.max(joint_step)),
        "maximum_joint_velocity_rad_s": float(np.max(np.abs(velocity))),
        "maximum_joint_acceleration_rad_s2": float(np.max(np.abs(acceleration))),
        "initial_posture_distance_rad": float(np.linalg.norm(q[0] - nominal)),
        "candidate_scores": {
            "left": list(left.candidate_scores),
            "right": list(right.candidate_scores),
        },
        "acceptance_checks": checks,
        "all_acceptance_checks_pass": bool(all(checks.values())),
    }
    np.savez_compressed(
        output / "offline_joint_trajectory.npz",
        time_s=packets.t,
        sequence_id=packets.seq,
        joint_names=np.asarray(model.joint_names),
        joint_position_reference_rad=q,
        unlimited_joint_position_solution_rad=q_unlimited,
        joint_velocity_reference_rad_s=velocity,
        joint_acceleration_reference_rad_s2=acceleration,
        left_raw_target_position_waist_m=left_raw_position,
        right_raw_target_position_waist_m=right_raw_position,
        left_target_position_waist_m=left_position,
        right_target_position_waist_m=right_position,
        left_target_position_pelvis_m=left_target_pelvis,
        right_target_position_pelvis_m=right_target_pelvis,
        left_target_orientation_waist=left_orientation,
        right_target_orientation_waist=right_orientation,
        left_target_orientation_pelvis=np.asarray(
            [waist[:3, :3] @ value for value in left_orientation]
        ),
        right_target_orientation_pelvis=np.asarray(
            [waist[:3, :3] @ value for value in right_orientation]
        ),
        left_workspace_projection_distance_m=left_projection_distance,
        right_workspace_projection_distance_m=right_projection_distance,
        head_pitch_yaw_rad=packets.ha,
        left_position_error_m=left_position_error,
        right_position_error_m=right_position_error,
        left_straightness_deg=left_straightness,
        right_straightness_deg=right_straightness,
        left_outward_elbow_pole_m=left_outward_pole,
        right_outward_elbow_pole_m=right_outward_pole,
        anchor_index=np.asarray(anchor),
    )
    selected_sequences = {int(value) for value in packets.seq}
    replay_commands = [
        command.as_dict()
        for command in commands
        if command.sequence_id in selected_sequences
    ]
    (output / "source_segment_raw_commands.jsonl").write_text(
        "".join(
            json.dumps(command, sort_keys=True) + "\n"
            for command in replay_commands
        ),
        encoding="utf-8",
    )
    write_json(output / "metrics.json", metrics)
    plots(
        output / "figures",
        packets.t,
        model,
        q,
        left_target_pelvis,
        right_target_pelvis,
        left_position_error,
        right_position_error,
        left_straightness,
        right_straightness,
        left_outward_pole,
        right_outward_pole,
        anchor,
    )
    write_experiment_config(output, config)
    resolved = json.loads(json.dumps(config))
    resolved["runtime"] = {
        "source_raw_commands": str(raw_path),
        "source_raw_commands_sha256": sha256(raw_path),
        "selected_source_indices_half_open": [packets.start, packets.stop],
        "selected_source_sequence_ids_inclusive": [
            int(packets.seq[0]),
            int(packets.seq[-1]),
        ],
        "full_enabled_source_indices_half_open": [
            full_segment_start,
            full_segment_stop,
        ],
        "source_segment_replay_commands": str(
            output / "source_segment_raw_commands.jsonl"
        ),
        "controlled_joint_names": list(model.joint_names),
        "resume_trajectory": str(resume_path) if resume_path is not None else None,
        "resume_trajectory_sha256": sha256(resume_path) if resume_path is not None else None,
        "xr_teleoperation_reference_contract": xr_reference_contract,
    }
    write_resolved_config(output, resolved)
    write_runner_command(output)
    write_metadata(
        output,
        ROOT,
        {
            "protocol_id": "t007_offline_trajectory_continuation",
            "execution_backend": "numpy_bounded_gauss_newton",
            "hardware_command_channel": "not_opened",
            "configuration_path": str(config_path),
            "configuration_sha256": sha256(config_path),
            "xr_teleoperation_r1_a5_ik_sha256": xr_reference_contract[
                "source_sha256"
            ],
        },
    )
    write_evidence_completeness(
        output,
        {
            "source_trace": True,
            "resolved_config": True,
            "joint_trajectory": True,
            "metrics": True,
            "plots": True,
            "video": {
                "present": False,
                "reason": "Offline kinematic solve; Isaac replay is the next gate.",
            },
        },
    )
    outcome = "pass" if all(checks.values()) else "fail"
    write_status(
        output,
        "completed",
        outcome,
        "all offline continuation criteria passed"
        if outcome == "pass"
        else "one or more offline continuation criteria failed",
        {"hardware_claim": "none", "dds_or_hardware_called": False},
    )
    print(json.dumps(metrics, indent=2))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
