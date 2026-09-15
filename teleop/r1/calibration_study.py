"""Deterministic calibration-sensitivity evaluation for the T004 study.

The study intentionally reuses the recorded T003-A waypoint trace as a
synthetic source-frame trace.  It evaluates the public mapper transform and the
project IK solver without importing Isaac Lab, Quest, DDS, or hardware code.
This isolates geometric calibration sensitivity from transport latency,
controller tracking, contact modelling, and hardware behaviour.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .ik import ArmIKConfig, solve_arm_ik
from .kinematics import ArmChain
from .mapping import R1TeleopMapper, TeleopCalibration, TeleopLimits
from .schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3


@dataclass(frozen=True)
class CalibrationCaseResult:
    """Per-case trace rows and summaries; no scientific judgement is implied."""

    rows: list[dict[str, object]]
    all_converged: bool
    any_joint_clamped: bool
    max_position_residual_m: float
    max_target_displacement_m: float
    minimum_limit_margin_rad: float | None
    identity_mapping_max_error_m: float
    base_velocity_nonzero_count: int


def load_t003_waypoints(path: Path, side: str) -> list[dict[str, object]]:
    """Load move waypoints and mirror their Y coordinate for the left arm.

    T003-A is a right-arm trace.  The T004 left condition is a declared mirror
    of its source positions, not observed left-hand Quest or Isaac data.
    """

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Cannot load T003 waypoint evidence {path}: {exc}") from exc
    if side not in {"left", "right"}:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    if not isinstance(payload, list):
        raise ValueError("T003 waypoints must be a JSON list.")

    trace: list[dict[str, object]] = []
    for item in payload:
        if not isinstance(item, dict) or item.get("kind") != "move":
            continue
        position = item.get("target_position_m")
        roll = item.get("target_wrist_roll_rad")
        if not isinstance(position, list) or len(position) != 3 or roll is None:
            raise ValueError(f"Malformed T003 move waypoint: {item!r}")
        source = np.asarray(position, dtype=float)
        if not np.all(np.isfinite(source)):
            raise ValueError(f"Non-finite T003 waypoint position: {item!r}")
        if side == "left":
            source[1] *= -1.0
        trace.append(
            {
                "name": str(item.get("name", "unnamed")),
                "source_position_m": source,
                "wrist_roll_rad": float(roll),
            }
        )
    if not trace:
        raise ValueError("T003 source evidence contains no move waypoints.")
    return trace


def evaluate_calibration_case(
    *,
    source_trace: list[dict[str, object]],
    side: str,
    calibration_translation_m: np.ndarray,
    calibration_yaw_rad: float,
    chain: ArmChain,
    ik_config: ArmIKConfig,
    seed_q: np.ndarray,
    nominal_q: np.ndarray,
) -> CalibrationCaseResult:
    """Map and solve every source waypoint under one declared calibration."""

    translation = np.asarray(calibration_translation_m, dtype=float)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("calibration_translation_m must contain three finite values.")
    if side not in {"left", "right"}:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    ik_config.validate()
    mapper = R1TeleopMapper(
        TeleopCalibration(Vector3(*translation.tolist()), float(calibration_yaw_rad)),
        TeleopLimits(command_timeout_s=0.5),
    )
    current_seed = chain.clamp(np.asarray(seed_q, dtype=float))
    nominal = chain.clamp(np.asarray(nominal_q, dtype=float))
    if current_seed.shape != (chain.dof,) or nominal.shape != (chain.dof,):
        raise ValueError(f"seed_q and nominal_q must have shape ({chain.dof},)")

    rows: list[dict[str, object]] = []
    for sequence_id, waypoint in enumerate(source_trace):
        source = np.asarray(waypoint["source_position_m"], dtype=float)
        pose = Pose(Vector3(*source.tolist()), Quaternion(0.0, 0.0, 0.0, 1.0))
        command = R1TeleopCommand(
            sequence_id=sequence_id,
            timestamp_monotonic_s=float(sequence_id),
            deadman_enabled=True,
            head_pose=pose,
            left_wrist_pose=pose,
            right_wrist_pose=pose,
            base_velocity=BaseVelocity.zero(),
            source_frame="quest_headset",
        )
        mapped = mapper.map(command, received_monotonic_s=float(sequence_id))
        target_pose = mapped.left_wrist_target if side == "left" else mapped.right_wrist_target
        if not mapped.enabled or target_pose is None:
            raise RuntimeError(f"A valid synthetic T004 command was disabled: {mapped.reason}")
        target = np.asarray(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z], dtype=float
        )
        result = solve_arm_ik(
            chain,
            target,
            float(waypoint["wrist_roll_rad"]),
            current_seed,
            nominal,
            ik_config,
        )
        if result.converged:
            current_seed = result.joint_positions.copy()
        rows.append(
            {
                "waypoint_index": sequence_id,
                "waypoint": str(waypoint["name"]),
                "source_x_m": float(source[0]),
                "source_y_m": float(source[1]),
                "source_z_m": float(source[2]),
                "mapped_x_m": float(target[0]),
                "mapped_y_m": float(target[1]),
                "mapped_z_m": float(target[2]),
                "target_displacement_m": float(np.linalg.norm(target - source)),
                "wrist_roll_rad": float(waypoint["wrist_roll_rad"]),
                "converged": result.converged,
                "solver_status": result.status,
                "iterations": result.iterations,
                "position_residual_m": result.position_residual_m,
                "roll_residual_rad": result.roll_residual_rad,
                "limit_margin_rad": result.limit_margin_rad,
                "clamped_joints": list(result.clamped_joints),
                "joint_positions_rad": result.joint_positions.tolist(),
                "base_velocity_zero": mapped.base_velocity == BaseVelocity.zero(),
            }
        )

    margins = [float(row["limit_margin_rad"]) for row in rows]
    is_identity = bool(np.allclose(translation, 0.0) and calibration_yaw_rad == 0.0)
    identity_errors = [float(row["target_displacement_m"]) for row in rows] if is_identity else [0.0]
    return CalibrationCaseResult(
        rows=rows,
        all_converged=all(bool(row["converged"]) for row in rows),
        any_joint_clamped=any(bool(row["clamped_joints"]) for row in rows),
        max_position_residual_m=max(float(row["position_residual_m"]) for row in rows),
        max_target_displacement_m=max(float(row["target_displacement_m"]) for row in rows),
        minimum_limit_margin_rad=min(margins) if margins else None,
        identity_mapping_max_error_m=max(identity_errors),
        base_velocity_nonzero_count=sum(not bool(row["base_velocity_zero"]) for row in rows),
    )


__all__ = ["CalibrationCaseResult", "evaluate_calibration_case", "load_t003_waypoints"]
