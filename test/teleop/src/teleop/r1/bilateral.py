"""Deterministic bilateral IK screening for the T006 protocol.

This module deliberately stops at kinematics.  It can establish that the two
arm solvers receive disjoint joint sets, solve a declared simultaneous trace,
and report wrist-endpoint separation.  A wrist-endpoint distance is *not* a
collision distance, and this module must not be used to make a collision-free
claim: the R1 USD self-collision defect is documented in the arm IK method.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ik import ArmIKConfig, solve_arm_ik
from .kinematics import ArmChain
from .mapping import R1JointOwnership


class BilateralConfigError(ValueError):
    """Raised when a bilateral trace cannot be evaluated safely."""


@dataclass(frozen=True)
class BilateralCaseResult:
    """Raw rows and direct screening facts for one simultaneous arm trace."""

    rows: list[dict[str, object]]
    all_converged: bool
    any_joint_clamped: bool
    minimum_limit_margin_rad: float
    minimum_endpoint_separation_m: float
    ownership_disjoint: bool
    ownership_complete_for_arms: bool


def arm_ownership_audit(
    ownership: R1JointOwnership, left: ArmChain, right: ArmChain
) -> dict[str, object]:
    """Audit the dispatch sets needed by simultaneous bilateral IK.

    This verifies the two chain joint sets are both disjoint from lower-body
    ownership and exactly covered by the upper-body owner.  Head joints may
    also live in the latter but are not dispatched here.
    """

    ownership.validate()
    left_joints = tuple(joint.name for joint in left.joints)
    right_joints = tuple(joint.name for joint in right.joints)
    left_set, right_set = set(left_joints), set(right_joints)
    upper_set, lower_set = set(ownership.upper_body), set(ownership.lower_body)
    expected = left_set | right_set
    return {
        "left_ik_joints": list(left_joints),
        "right_ik_joints": list(right_joints),
        "left_right_overlap": sorted(left_set & right_set),
        "arm_lower_body_overlap": sorted(expected & lower_set),
        "arm_joints_missing_from_upper_body": sorted(expected - upper_set),
        "ownership_disjoint": not (left_set & right_set) and not (expected & lower_set),
        "ownership_complete_for_arms": expected <= upper_set,
    }


def _target(step: dict[str, object], side: str) -> tuple[np.ndarray, float]:
    target = step.get(f"{side}_target_position_m")
    roll = step.get(f"{side}_wrist_roll_rad")
    if not isinstance(target, (list, tuple)) or len(target) != 3 or roll is None:
        raise BilateralConfigError(f"Step {step.get('name', '<unnamed>')!r} has no valid {side} target.")
    position = np.asarray(target, dtype=float)
    if not np.all(np.isfinite(position)) or not np.isfinite(float(roll)):
        raise BilateralConfigError(f"Step {step.get('name', '<unnamed>')!r} has a non-finite {side} target.")
    return position, float(roll)


def evaluate_bilateral_case(
    *,
    trace: list[dict[str, object]],
    left_chain: ArmChain,
    right_chain: ArmChain,
    ik_config: ArmIKConfig,
    left_seed_q: np.ndarray,
    right_seed_q: np.ndarray,
    left_nominal_q: np.ndarray,
    right_nominal_q: np.ndarray,
    ownership: R1JointOwnership | None = None,
) -> BilateralCaseResult:
    """Solve every declared simultaneous target pair using continuous seeds.

    A failed side retains its last converged seed, modelling the method's hold
    boundary for the next input sample without treating a partial solution as a
    command.  The raw row still retains the partial numerical result.
    """

    if not trace:
        raise BilateralConfigError("Bilateral trace must contain at least one simultaneous step.")
    ik_config.validate()
    audit = arm_ownership_audit(ownership or R1JointOwnership(), left_chain, right_chain)
    left_current = left_chain.clamp(np.asarray(left_seed_q, dtype=float))
    right_current = right_chain.clamp(np.asarray(right_seed_q, dtype=float))
    left_nominal = left_chain.clamp(np.asarray(left_nominal_q, dtype=float))
    right_nominal = right_chain.clamp(np.asarray(right_nominal_q, dtype=float))
    expected_shape = (left_chain.dof,)
    if left_current.shape != expected_shape or left_nominal.shape != expected_shape:
        raise BilateralConfigError(f"Left seed and nominal posture must have shape {expected_shape}.")
    expected_shape = (right_chain.dof,)
    if right_current.shape != expected_shape or right_nominal.shape != expected_shape:
        raise BilateralConfigError(f"Right seed and nominal posture must have shape {expected_shape}.")

    rows: list[dict[str, object]] = []
    for index, step in enumerate(trace):
        left_target, left_roll = _target(step, "left")
        right_target, right_roll = _target(step, "right")
        left_result = solve_arm_ik(left_chain, left_target, left_roll, left_current, left_nominal, ik_config)
        right_result = solve_arm_ik(right_chain, right_target, right_roll, right_current, right_nominal, ik_config)
        if left_result.converged:
            left_current = left_result.joint_positions.copy()
        if right_result.converged:
            right_current = right_result.joint_positions.copy()
        endpoint_separation = float(
            np.linalg.norm(
                left_chain.endpoint_position(left_result.joint_positions)
                - right_chain.endpoint_position(right_result.joint_positions)
            )
        )
        rows.append(
            {
                "step_index": index,
                "step": str(step.get("name", f"step_{index}")),
                "source": str(step.get("source", "unspecified")),
                "left_target_position_m": left_target.tolist(),
                "right_target_position_m": right_target.tolist(),
                "left_wrist_roll_rad": left_roll,
                "right_wrist_roll_rad": right_roll,
                "left_converged": left_result.converged,
                "right_converged": right_result.converged,
                "left_solver_status": left_result.status,
                "right_solver_status": right_result.status,
                "left_position_residual_m": left_result.position_residual_m,
                "right_position_residual_m": right_result.position_residual_m,
                "left_roll_residual_rad": left_result.roll_residual_rad,
                "right_roll_residual_rad": right_result.roll_residual_rad,
                "left_limit_margin_rad": left_result.limit_margin_rad,
                "right_limit_margin_rad": right_result.limit_margin_rad,
                "left_clamped_joints": list(left_result.clamped_joints),
                "right_clamped_joints": list(right_result.clamped_joints),
                "left_joint_positions_rad": left_result.joint_positions.tolist(),
                "right_joint_positions_rad": right_result.joint_positions.tolist(),
                "endpoint_separation_m": endpoint_separation,
            }
        )

    all_converged = all(bool(row["left_converged"]) and bool(row["right_converged"]) for row in rows)
    any_joint_clamped = any(bool(row["left_clamped_joints"]) or bool(row["right_clamped_joints"]) for row in rows)
    return BilateralCaseResult(
        rows=rows,
        all_converged=all_converged,
        any_joint_clamped=any_joint_clamped,
        minimum_limit_margin_rad=min(
            min(float(row["left_limit_margin_rad"]), float(row["right_limit_margin_rad"])) for row in rows
        ),
        minimum_endpoint_separation_m=min(float(row["endpoint_separation_m"]) for row in rows),
        ownership_disjoint=bool(audit["ownership_disjoint"]),
        ownership_complete_for_arms=bool(audit["ownership_complete_for_arms"]),
    )


__all__ = [
    "BilateralCaseResult",
    "BilateralConfigError",
    "arm_ownership_audit",
    "evaluate_bilateral_case",
]
