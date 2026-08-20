"""Damped-least-squares IK for the R1 arm, per `docs/teleop/r1_arm_wrist_ik.md`.

The task is four scalars — a 3-D endpoint position and the wrist-roll angle — on
a five-joint chain, so one degree of freedom is redundant and is resolved by a
null-space bias towards a declared nominal posture. That bias is what makes the
solution repeatable: without it two runs commanded to the same target can settle
on different elbow configurations and their traces stop being comparable.

Wrist roll is the last joint's own coordinate, so it is imposed directly rather
than through the Jacobian, and the position task is solved on the remaining
freedom. Solving position and roll in one least-squares system would let the
solver trade roll error for position error, which the method does not permit:
roll is commanded, not negotiated.

**No numeric threshold has a default here.** Tolerances, iteration budgets,
damping and rate limits are declared by the experiment that runs the solver. The
method record deliberately leaves them open because the project has no baseline
from which to derive safe teleoperation values, and a default in this file would
put an unaudited number into every downstream result.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmChain


class IKConfigError(ValueError):
    """Raised when a solver configuration is unusable."""


@dataclass(frozen=True)
class ArmIKConfig:
    """Solver settings. Every field must be supplied by the experiment config."""

    position_tolerance_m: float
    roll_tolerance_rad: float
    max_iterations: int
    damping: float
    posture_weight: float
    max_joint_step_rad: float
    posture_tolerance_rad: float
    """Settling tolerance for the redundant degree of freedom.

    Convergence on the position task alone is not enough. The position residual
    reaches tolerance in a handful of iterations while the null-space posture
    bias is still moving the elbow, so stopping there leaves the redundant joint
    wherever the seed happened to put it and two solves of the same target
    disagree. Iteration continues until the applied joint step is below this
    value, which is what makes the solution a function of the target rather than
    of the seed.
    """

    def validate(self) -> None:
        if self.position_tolerance_m <= 0.0:
            raise IKConfigError("position_tolerance_m must be positive.")
        if self.roll_tolerance_rad <= 0.0:
            raise IKConfigError("roll_tolerance_rad must be positive.")
        if self.posture_tolerance_rad <= 0.0:
            raise IKConfigError("posture_tolerance_rad must be positive.")
        if self.max_iterations < 1:
            raise IKConfigError("max_iterations must be at least 1.")
        if self.damping <= 0.0:
            raise IKConfigError("damping must be positive; an undamped pseudo-inverse is unbounded near singularities.")
        if self.posture_weight < 0.0:
            raise IKConfigError("posture_weight must be non-negative.")
        if self.max_joint_step_rad <= 0.0:
            raise IKConfigError("max_joint_step_rad must be positive.")


@dataclass(frozen=True)
class IKResult:
    """One solve. `converged` is the only field a caller may treat as success."""

    joint_positions: np.ndarray
    converged: bool
    status: str
    iterations: int
    position_residual_m: float
    roll_residual_rad: float
    limit_margin_rad: float
    clamped_joints: tuple[str, ...]

    @property
    def hit_a_limit(self) -> bool:
        return bool(self.clamped_joints)


def solve_arm_ik(
    chain: ArmChain,
    target_position_m: np.ndarray,
    target_roll_rad: float,
    seed_q: np.ndarray,
    nominal_q: np.ndarray,
    config: ArmIKConfig,
) -> IKResult:
    """Solve for joint positions reaching a target endpoint position and roll.

    `seed_q` is the previous solution, which keeps successive solves continuous.
    `nominal_q` is the posture the redundant degree of freedom is biased towards.

    Returns a result whose `converged` flag is false when the residuals are not
    within tolerance inside the iteration budget. Non-convergence is a valid
    scientific outcome about reachability, not an error to retry silently, so the
    caller holds rather than dispatching a partial solution.
    """

    config.validate()
    target_position = np.asarray(target_position_m, dtype=float)
    if target_position.shape != (3,):
        raise IKConfigError(f"target_position_m must have shape (3,), got {target_position.shape}")

    lower, upper = chain.lower_limits, chain.upper_limits
    q = chain.clamp(np.asarray(seed_q, dtype=float).copy())
    nominal = chain.clamp(np.asarray(nominal_q, dtype=float))

    # Roll is the last joint's coordinate, so it is imposed, not solved for.
    roll_clamped = float(np.clip(target_roll_rad, lower[-1], upper[-1]))
    q[-1] = roll_clamped

    position_indices = np.arange(chain.dof - 1)
    identity = np.eye(len(position_indices))
    status = "converged"
    iterations = 0
    best_q = q.copy()
    best_position_residual = float(np.linalg.norm(target_position - chain.endpoint_position(q)))
    stagnant_iterations = 0

    for iterations in range(1, config.max_iterations + 1):
        error = target_position - chain.endpoint_position(q)
        position_ok = float(np.linalg.norm(error)) <= config.position_tolerance_m

        jacobian = chain.position_jacobian(q)[:, position_indices]
        # Damped least squares: J^T (J J^T + lambda^2 I)^-1 stays bounded at
        # singularities, where a plain pseudo-inverse would demand an infinite step.
        jjt = jacobian @ jacobian.T + (config.damping**2) * np.eye(3)
        try:
            primary = jacobian.T @ np.linalg.solve(jjt, error)
        except np.linalg.LinAlgError:
            status = "singular_system"
            break

        # Null-space posture bias: only the component that does not disturb the
        # position task is applied, so redundancy is resolved without cost to it.
        if config.posture_weight > 0.0:
            pseudo = jacobian.T @ np.linalg.solve(jjt, jacobian)
            null_projector = identity - pseudo
            bias = config.posture_weight * (nominal[position_indices] - q[position_indices])
            step = primary + null_projector @ bias
        else:
            step = primary

        norm = float(np.linalg.norm(step))
        if norm > config.max_joint_step_rad:
            step *= config.max_joint_step_rad / norm

        previous = q[position_indices].copy()
        q[position_indices] = np.clip(previous + step, lower[position_indices], upper[position_indices])
        candidate_residual = float(np.linalg.norm(target_position - chain.endpoint_position(q)))
        if candidate_residual < best_position_residual - 1e-10:
            best_position_residual = candidate_residual
            best_q = q.copy()
            stagnant_iterations = 0
        elif candidate_residual > config.position_tolerance_m:
            stagnant_iterations += 1
        # The *applied* change is the settling measure, not the desired step: at a
        # joint limit the desired posture bias never shrinks, but the change it
        # produces does, so this terminates instead of exhausting the budget.
        applied = float(np.linalg.norm(q[position_indices] - previous))
        if position_ok and applied <= config.posture_tolerance_rad:
            break
        if stagnant_iterations >= 30:
            # A Cartesian target outside the bounded joint workspace has no
            # exact root. Preserve the closest iterate so the live simulation
            # profile can explicitly project to the reachable boundary instead
            # of returning an arbitrary late oscillation.
            q = best_q.copy()
            status = "projected_to_reachable_boundary"
            break
    else:
        status = "iteration_budget_exhausted"
        q = best_q.copy()

    position_residual = float(np.linalg.norm(target_position - chain.endpoint_position(q)))
    roll_residual = float(abs(roll_clamped - chain.wrist_roll(q)))
    roll_request_residual = float(abs(target_roll_rad - roll_clamped))

    converged = (
        status == "converged"
        and position_residual <= config.position_tolerance_m
        and roll_residual <= config.roll_tolerance_rad
    )
    if status == "converged" and not converged:
        status = "tolerance_not_met"
    if roll_request_residual > config.roll_tolerance_rad:
        # The requested roll was outside the joint range; the endpoint may still
        # be reached but the commanded orientation was not honoured.
        converged = False
        status = "roll_target_clamped_to_limit"

    margins = np.minimum(q - lower, upper - q)
    clamped = tuple(
        joint.name
        for joint, margin in zip(chain.joints, margins)
        if margin <= 1e-9
    )
    return IKResult(
        joint_positions=q,
        converged=converged,
        status=status,
        iterations=iterations,
        position_residual_m=position_residual,
        roll_residual_rad=max(roll_residual, roll_request_residual),
        limit_margin_rad=float(np.min(margins)),
        clamped_joints=clamped,
    )


__all__ = ["ArmIKConfig", "IKConfigError", "IKResult", "solve_arm_ik"]
