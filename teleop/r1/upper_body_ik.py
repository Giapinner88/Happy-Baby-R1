"""Coupled damped-least-squares IK for the R1-A5 upper body.

Unlike the legacy per-arm solver, this method lets the physical waist-yaw joint
participate in both wrist tasks and matches the head orientation at the same
time.  Wrist orientation is a weighted best-fit target because each R1-A5 arm
has only five joints; the solver reports the residual instead of claiming an
arbitrary 6-DoF wrist pose was realized.

A target outside the bounded joint workspace has no exact root. As in the
single-arm solver (`ik.py`), stagnation of the weighted task error is treated
as "as close as this iterate is going to get" and the closest iterate found so
far is returned with status `projected_to_reachable_boundary`, rather than
letting the solver exhaust its iteration budget while an unrelated redundant
joint keeps drifting. The caller decides whether a projected, non-exact result
is dispatched or held.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .upper_body_kinematics import R1A5UpperBodyModel


class UpperBodyIKConfigError(ValueError):
    """Raised when a coupled-IK target or configuration is malformed."""


# Matches the single-arm solver's stagnation threshold (`ik.py`).
_STAGNATION_ITERATIONS = 30
# Progress is measured relatively, not as an absolute score delta. The weighted
# score for an out-of-reach target is O(1e3), so an absolute epsilon counts
# asymptotic crawl toward an unreachable point as genuine progress and the
# stagnation counter never fires. A dimensionless ratio carries no physical
# scale, so it stays a numerical-method constant rather than a hidden tolerance.
_RELATIVE_IMPROVEMENT = 1e-4


def _proper_rotation(value: np.ndarray, name: str) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise UpperBodyIKConfigError(f"{name} must be a finite 3x3 rotation matrix.")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-6
    ):
        raise UpperBodyIKConfigError(f"{name} must be a proper rotation matrix.")
    return rotation


def so3_log(rotation: np.ndarray) -> np.ndarray:
    """Rotation-vector logarithm with stable small-angle and pi branches."""

    matrix = _proper_rotation(rotation, "rotation")
    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    vee = np.array(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]],
        dtype=float,
    )
    if angle < 1e-8:
        return 0.5 * vee
    if np.pi - angle < 1e-5:
        diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        pivot = int(np.argmax(axis))
        if axis[pivot] < 1e-8:
            return np.zeros(3)
        if pivot == 0:
            axis[1] = (matrix[0, 1] + matrix[1, 0]) / (4.0 * axis[0])
            axis[2] = (matrix[0, 2] + matrix[2, 0]) / (4.0 * axis[0])
        elif pivot == 1:
            axis[0] = (matrix[0, 1] + matrix[1, 0]) / (4.0 * axis[1])
            axis[2] = (matrix[1, 2] + matrix[2, 1]) / (4.0 * axis[1])
        else:
            axis[0] = (matrix[0, 2] + matrix[2, 0]) / (4.0 * axis[2])
            axis[1] = (matrix[1, 2] + matrix[2, 1]) / (4.0 * axis[2])
        axis /= np.linalg.norm(axis)
        return angle * axis
    return angle * vee / (2.0 * np.sin(angle))


def quaternion_xyzw_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """Convert an ``[x, y, z, w]`` quaternion to a proper rotation."""

    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise UpperBodyIKConfigError("quaternion_xyzw must be a finite 4-vector.")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise UpperBodyIKConfigError("quaternion_xyzw must have non-zero norm.")
    x, y, z, w = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class UpperBodyIKTarget:
    """Bilateral wrist and head targets, all expressed in ``pelvis_link``."""

    left_position_m: np.ndarray
    left_orientation: np.ndarray
    right_position_m: np.ndarray
    right_orientation: np.ndarray
    head_orientation: np.ndarray

    def validate(self) -> None:
        for name, raw in (
            ("left_position_m", self.left_position_m),
            ("right_position_m", self.right_position_m),
        ):
            value = np.asarray(raw, dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise UpperBodyIKConfigError(f"{name} must be a finite 3-vector.")
        _proper_rotation(self.left_orientation, "left_orientation")
        _proper_rotation(self.right_orientation, "right_orientation")
        _proper_rotation(self.head_orientation, "head_orientation")


@dataclass(frozen=True)
class UpperBodyIKConfig:
    """All numerical choices are supplied by the caller or experiment."""

    position_tolerance_m: float
    wrist_orientation_tolerance_rad: float
    head_orientation_tolerance_rad: float
    max_iterations: int
    damping: float
    posture_weight: float
    max_joint_step_rad: float
    finite_difference_rad: float
    position_weight: float
    wrist_orientation_weight: float
    head_orientation_weight: float

    def validate(self) -> None:
        positive = (
            "position_tolerance_m",
            "wrist_orientation_tolerance_rad",
            "head_orientation_tolerance_rad",
            "damping",
            "max_joint_step_rad",
            "finite_difference_rad",
            "position_weight",
            "wrist_orientation_weight",
            "head_orientation_weight",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise UpperBodyIKConfigError(f"{name} must be finite and positive.")
        if self.max_iterations < 1:
            raise UpperBodyIKConfigError("max_iterations must be at least one.")
        if not np.isfinite(self.posture_weight) or self.posture_weight < 0.0:
            raise UpperBodyIKConfigError("posture_weight must be finite and non-negative.")


@dataclass(frozen=True)
class UpperBodyIKResult:
    joint_positions: np.ndarray
    converged: bool
    status: str
    iterations: int
    left_position_residual_m: float
    right_position_residual_m: float
    left_orientation_residual_rad: float
    right_orientation_residual_rad: float
    head_orientation_residual_rad: float
    limit_margin_rad: float
    clamped_joints: tuple[str, ...]

    @property
    def hit_a_limit(self) -> bool:
        return bool(self.clamped_joints)


def _task_error(
    model: R1A5UpperBodyModel, q: np.ndarray, target: UpperBodyIKTarget
) -> np.ndarray:
    state = model.forward_kinematics(q)
    return np.concatenate(
        (
            np.asarray(target.left_position_m) - state.left_end_effector[:3, 3],
            np.asarray(target.right_position_m) - state.right_end_effector[:3, 3],
            so3_log(np.asarray(target.left_orientation) @ state.left_end_effector[:3, :3].T),
            so3_log(np.asarray(target.right_orientation) @ state.right_end_effector[:3, :3].T),
            so3_log(np.asarray(target.head_orientation) @ state.head[:3, :3].T),
        )
    )


def upper_body_task_jacobian(
    model: R1A5UpperBodyModel,
    q: np.ndarray,
    target: UpperBodyIKTarget,
    finite_difference_rad: float,
) -> np.ndarray:
    """Central-difference Jacobian of the 15-vector task error."""

    if not np.isfinite(finite_difference_rad) or finite_difference_rad <= 0.0:
        raise UpperBodyIKConfigError("finite_difference_rad must be finite and positive.")
    values = np.asarray(q, dtype=float)
    if values.shape != (model.dof,):
        raise UpperBodyIKConfigError(f"q must have shape ({model.dof},), got {values.shape}.")
    target.validate()
    jacobian = np.zeros((15, model.dof), dtype=float)
    for index in range(model.dof):
        plus, minus = values.copy(), values.copy()
        plus[index] += finite_difference_rad
        minus[index] -= finite_difference_rad
        jacobian[:, index] = (
            _task_error(model, plus, target) - _task_error(model, minus, target)
        ) / (2.0 * finite_difference_rad)
    return jacobian


def _residuals(error: np.ndarray) -> tuple[float, float, float, float, float]:
    return (
        float(np.linalg.norm(error[0:3])),
        float(np.linalg.norm(error[3:6])),
        float(np.linalg.norm(error[6:9])),
        float(np.linalg.norm(error[9:12])),
        float(np.linalg.norm(error[12:15])),
    )


def solve_upper_body_ik(
    model: R1A5UpperBodyModel,
    target: UpperBodyIKTarget,
    seed_q: np.ndarray,
    nominal_q: np.ndarray,
    config: UpperBodyIKConfig,
) -> UpperBodyIKResult:
    """Solve the coupled 13-DoF upper-body task with hard joint limits."""

    config.validate()
    target.validate()
    q = model.clamp(np.asarray(seed_q, dtype=float).copy())
    nominal = model.clamp(np.asarray(nominal_q, dtype=float))
    weights = np.concatenate(
        (
            np.full(6, config.position_weight),
            np.full(6, config.wrist_orientation_weight),
            np.full(3, config.head_orientation_weight),
        )
    )
    identity = np.eye(model.dof)
    best_q = q.copy()
    best_score = float("inf")
    status = "iteration_budget_exhausted"
    iterations = 0
    stagnant = 0

    for iterations in range(1, config.max_iterations + 1):
        error = _task_error(model, q, target)
        weighted_error = weights * error
        score = float(weighted_error @ weighted_error)
        if score < best_score - 1e-14:
            # Any improvement is kept as the best iterate; only *meaningful*
            # improvement resets the stagnation counter.
            meaningful = score < best_score * (1.0 - _RELATIVE_IMPROVEMENT)
            best_score = score
            best_q = q.copy()
            stagnant = 0 if meaningful else stagnant + 1
        else:
            stagnant += 1
        left_pos, right_pos, left_ori, right_ori, head_ori = _residuals(error)

        # Posture is a secondary cost, not a task constraint. Continuing to
        # iterate after all physical task tolerances pass can make a valid live
        # target miss its bounded iteration budget merely because the
        # null-space drift toward nominal has not fully decayed.
        tasks_ok = (
            max(left_pos, right_pos) <= config.position_tolerance_m
            and max(left_ori, right_ori) <= config.wrist_orientation_tolerance_rad
            and head_ori <= config.head_orientation_tolerance_rad
        )
        if tasks_ok:
            status = "converged"
            break

        jacobian = upper_body_task_jacobian(model, q, target, config.finite_difference_rad)
        weighted_jacobian = weights[:, None] * jacobian
        hessian = weighted_jacobian.T @ weighted_jacobian + (config.damping**2) * identity
        try:
            pseudo = np.linalg.solve(hessian, weighted_jacobian.T)
        except np.linalg.LinAlgError:
            status = "singular_system"
            break
        primary = -pseudo @ weighted_error
        null_projector = identity - pseudo @ weighted_jacobian
        step = primary + config.posture_weight * (null_projector @ (nominal - q))
        step_norm = float(np.linalg.norm(step))
        if step_norm > config.max_joint_step_rad:
            step *= config.max_joint_step_rad / step_norm
        q = np.clip(q + step, model.lower_limits, model.upper_limits)
        if stagnant >= _STAGNATION_ITERATIONS:
            # The closest iterate found so far, not the current (possibly
            # oscillating) one: preserve it explicitly rather than returning
            # an arbitrary late point.
            q = best_q.copy()
            status = "projected_to_reachable_boundary"
            break
    else:
        q = best_q.copy()

    final_error = _task_error(model, q, target)
    left_pos, right_pos, left_ori, right_ori, head_ori = _residuals(final_error)
    converged = (
        status == "converged"
        and max(left_pos, right_pos) <= config.position_tolerance_m
        and max(left_ori, right_ori) <= config.wrist_orientation_tolerance_rad
        and head_ori <= config.head_orientation_tolerance_rad
    )
    margins = np.minimum(q - model.lower_limits, model.upper_limits - q)
    clamped = tuple(
        name for name, margin in zip(model.joint_names, margins) if margin <= 1e-9
    )
    return UpperBodyIKResult(
        joint_positions=q,
        converged=converged,
        status=status,
        iterations=iterations,
        left_position_residual_m=left_pos,
        right_position_residual_m=right_pos,
        left_orientation_residual_rad=left_ori,
        right_orientation_residual_rad=right_ori,
        head_orientation_residual_rad=head_ori,
        limit_margin_rad=float(np.min(margins)),
        clamped_joints=clamped,
    )


__all__ = [
    "UpperBodyIKConfig",
    "UpperBodyIKConfigError",
    "UpperBodyIKResult",
    "UpperBodyIKTarget",
    "quaternion_xyzw_to_matrix",
    "so3_log",
    "solve_upper_body_ik",
    "upper_body_task_jacobian",
]
