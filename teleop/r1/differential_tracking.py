"""Velocity-feedforward Cartesian tracking for the R1 upper body.

This is the control-law layer described in the teleoperation theory note.  It
does not own transport, calibration, interpolation, or a simulator.  Callers
provide the current Cartesian target and its 15-vector spatial velocity in the
same task ordering as :mod:`teleop.r1.upper_body_ik`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .upper_body_ik import (
    UpperBodyIKTarget,
    upper_body_task_error,
    upper_body_task_jacobian,
)
from .upper_body_kinematics import R1A5UpperBodyModel


@dataclass(frozen=True)
class DifferentialTrackingConfig:
    """Explicit numerical and physical choices for one tracking controller."""

    dt_s: float
    position_gain_s: float
    wrist_orientation_gain_s: float
    head_orientation_gain_s: float
    damping: float
    finite_difference_rad: float
    position_weight: float
    wrist_orientation_weight: float
    head_orientation_weight: float
    max_joint_velocity_rad_s: float
    max_joint_acceleration_rad_s2: float
    max_reference_lead_rad: float
    posture_gain_s: float = 0.0
    task_priority: str = "weighted"

    def validate(self) -> None:
        positive = (
            "dt_s",
            "position_gain_s",
            "wrist_orientation_gain_s",
            "head_orientation_gain_s",
            "damping",
            "finite_difference_rad",
            "position_weight",
            "wrist_orientation_weight",
            "head_orientation_weight",
            "max_joint_velocity_rad_s",
            "max_joint_acceleration_rad_s2",
            "max_reference_lead_rad",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        if not np.isfinite(self.posture_gain_s) or self.posture_gain_s < 0.0:
            raise ValueError("posture_gain_s must be finite and non-negative.")
        if self.task_priority not in ("weighted", "position_then_orientation"):
            raise ValueError(
                "task_priority must be weighted or position_then_orientation."
            )


@dataclass(frozen=True)
class DifferentialTrackingStep:
    joint_position_reference_rad: np.ndarray
    joint_velocity_reference_rad_s: np.ndarray
    task_error: np.ndarray
    task_velocity_command: np.ndarray
    minimum_singular_value: float
    velocity_saturated: np.ndarray
    acceleration_saturated: np.ndarray
    reference_lead_clamped: np.ndarray
    joint_limit_active: np.ndarray


class DifferentialUpperBodyTracker:
    """Integrate a constrained weighted-DLS velocity command into ``q_ref``."""

    def __init__(
        self,
        model: R1A5UpperBodyModel,
        nominal_q: np.ndarray,
        initial_q: np.ndarray,
        config: DifferentialTrackingConfig,
    ) -> None:
        config.validate()
        self.model = model
        self.config = config
        self.nominal_q = self._vector(nominal_q, "nominal_q")
        self.q_reference = model.clamp(self._vector(initial_q, "initial_q"))
        self.dq_reference = np.zeros(model.dof, dtype=float)

    def reset(self, measured_q: np.ndarray) -> None:
        """Reset the online reference to a measured/session-boundary state."""

        self.q_reference = self.model.clamp(self._vector(measured_q, "measured_q"))
        self.dq_reference.fill(0.0)

    def hold(self) -> None:
        """Stop stored reference velocity without moving the held reference."""

        self.dq_reference.fill(0.0)

    def _vector(self, value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (self.model.dof,) or not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must be a finite ({self.model.dof},)-vector.")
        return array.copy()

    @staticmethod
    def _bounded_quadratic_solve(
        hessian: np.ndarray,
        rhs: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        initial: np.ndarray,
        *,
        sweeps: int = 80,
    ) -> np.ndarray:
        """Solve a small positive-definite box QP by coordinate descent.

        The controller has at most 14 variables.  Cyclic exact coordinate
        minimization is deterministic, dependency-free, and—unlike clipping an
        unconstrained DLS solution—keeps re-optimizing the other joints after a
        shoulder or elbow reaches a bound.
        """

        solution = np.clip(np.asarray(initial, dtype=float), lower, upper)
        diagonal = np.diag(hessian)
        if np.any(diagonal <= 0.0):
            raise ValueError("bounded DLS Hessian must have a positive diagonal")
        for _ in range(sweeps):
            previous = solution.copy()
            for index in range(len(solution)):
                residual = rhs[index] - (
                    hessian[index] @ solution
                    - hessian[index, index] * solution[index]
                )
                solution[index] = np.clip(
                    residual / diagonal[index], lower[index], upper[index]
                )
            if float(np.max(np.abs(solution - previous))) < 1.0e-9:
                break
        return solution

    def step(
        self,
        measured_q: np.ndarray,
        target: UpperBodyIKTarget,
        desired_task_velocity: np.ndarray,
        *,
        velocity_feedforward: bool = True,
    ) -> DifferentialTrackingStep:
        q = self.model.clamp(self._vector(measured_q, "measured_q"))
        velocity = np.asarray(desired_task_velocity, dtype=float)
        if velocity.shape != (15,) or not np.all(np.isfinite(velocity)):
            raise ValueError("desired_task_velocity must be a finite 15-vector.")

        error = upper_body_task_error(self.model, q, target)
        gains = np.concatenate(
            (
                np.full(6, self.config.position_gain_s),
                np.full(6, self.config.wrist_orientation_gain_s),
                np.full(3, self.config.head_orientation_gain_s),
            )
        )
        task_command = gains * error
        if velocity_feedforward:
            task_command = task_command + velocity

        # upper_body_task_jacobian differentiates target-current and is thus
        # the negative of the conventional geometric Jacobian used in V=J dq.
        geometric_jacobian = -upper_body_task_jacobian(
            self.model, q, target, self.config.finite_difference_rad
        )
        weights = np.concatenate(
            (
                np.full(6, self.config.position_weight),
                np.full(6, self.config.wrist_orientation_weight),
                np.full(3, self.config.head_orientation_weight),
            )
        )
        weighted_jacobian = weights[:, None] * geometric_jacobian
        weighted_command = weights * task_command
        identity = np.eye(self.model.dof)
        if self.config.task_priority == "weighted":
            hessian = weighted_jacobian.T @ weighted_jacobian
            hessian += (self.config.damping**2) * identity
            rhs = weighted_jacobian.T @ weighted_command
            dq_unbounded = np.linalg.solve(hessian, rhs)
            pseudo = np.linalg.solve(hessian, weighted_jacobian.T)
            null_projector = identity - pseudo @ weighted_jacobian
            bounded_hessian = hessian
            bounded_rhs = rhs
        else:
            # The R1 arm has only five DoF.  Solve wrist position (and the
            # independent head task) first, then spend the remaining arm
            # null-space on wrist orientation.  This is the strict-priority
            # construction from section 28 of the teleoperation method note;
            # it prevents an infeasible 6-D wrist orientation request from
            # stealing accuracy from the 3-D trajectory while still recruiting
            # shoulder and wrist-roll joints for natural controller rotations.
            primary_indices = np.r_[0:6, 12:15]
            secondary_indices = np.r_[6:12]
            primary_jacobian = weighted_jacobian[primary_indices]
            primary_command = weighted_command[primary_indices]
            primary_hessian = primary_jacobian.T @ primary_jacobian
            primary_hessian += (self.config.damping**2) * identity
            primary_pseudo = np.linalg.solve(
                primary_hessian, primary_jacobian.T
            )
            dq_primary = primary_pseudo @ primary_command
            primary_null = identity - primary_pseudo @ primary_jacobian

            orientation_jacobian = weighted_jacobian[secondary_indices]
            projected_orientation_jacobian = orientation_jacobian @ primary_null
            orientation_residual = (
                weighted_command[secondary_indices]
                - orientation_jacobian @ dq_primary
            )
            secondary_hessian = (
                projected_orientation_jacobian.T
                @ projected_orientation_jacobian
                + (self.config.damping**2) * identity
            )
            secondary_pseudo = np.linalg.solve(
                secondary_hessian, projected_orientation_jacobian.T
            )
            dq_unbounded = (
                dq_primary
                + primary_null @ (secondary_pseudo @ orientation_residual)
            )
            # Posture competes with the lower-priority wrist orientation, but
            # remains projected out of the primary wrist-position/head task.
            # With wrist-only sensing this is the explicit elbow-branch prior:
            # the human elbow/swivel is not observable from a controller pose.
            null_projector = primary_null
            # Under a box constraint, preserve the primary Cartesian task and
            # use the unconstrained hierarchical answer only as a damped
            # preference.  This prevents the old solve-then-clip path from
            # changing the direction of the commanded endpoint velocity.
            bounded_hessian = primary_hessian
            bounded_rhs = (
                primary_jacobian.T @ primary_command
                + (self.config.damping**2) * dq_unbounded
            )

        if self.config.posture_gain_s > 0.0:
            dq_unbounded += self.config.posture_gain_s * (
                null_projector @ (self.nominal_q - q)
            )

        max_v = self.config.max_joint_velocity_rad_s
        max_delta_v = self.config.max_joint_acceleration_rad_s2 * self.config.dt_s
        velocity_lower = np.full(self.model.dof, -max_v)
        velocity_upper = np.full(self.model.dof, max_v)
        acceleration_lower = self.dq_reference - max_delta_v
        acceleration_upper = self.dq_reference + max_delta_v
        joint_lower = (
            self.model.lower_limits - self.q_reference
        ) / self.config.dt_s
        joint_upper = (
            self.model.upper_limits - self.q_reference
        ) / self.config.dt_s
        physical_lower = np.maximum(velocity_lower, joint_lower)
        physical_upper = np.minimum(velocity_upper, joint_upper)
        lower_velocity = np.maximum(physical_lower, acceleration_lower)
        upper_velocity = np.minimum(physical_upper, acceleration_upper)
        # If the stored reference is already on a joint boundary, numerical
        # lag can make the acceleration interval unable to return it to the
        # feasible set in one step. Joint and velocity safety take precedence;
        # acceleration is relaxed only on those inconsistent coordinates.
        inconsistent = lower_velocity > upper_velocity + 1.0e-10
        lower_velocity[inconsistent] = physical_lower[inconsistent]
        upper_velocity[inconsistent] = physical_upper[inconsistent]
        if np.any(lower_velocity > upper_velocity + 1.0e-10):
            raise RuntimeError("inconsistent physical joint-velocity bounds")
        dq_limited = self._bounded_quadratic_solve(
            bounded_hessian,
            bounded_rhs,
            lower_velocity,
            upper_velocity,
            dq_unbounded,
        )
        tolerance = 1.0e-7
        active_lower = dq_limited <= lower_velocity + tolerance
        active_upper = dq_limited >= upper_velocity - tolerance
        # Report which unconstrained request each physical box would limit,
        # even when a tighter box (usually acceleration) becomes the active
        # face of their intersection.
        velocity_saturated = (
            (dq_unbounded < velocity_lower - tolerance)
            | (dq_unbounded > velocity_upper + tolerance)
        )
        acceleration_saturated = (
            (dq_unbounded < acceleration_lower - tolerance)
            | (dq_unbounded > acceleration_upper + tolerance)
        )
        joint_limit_active = (
            (active_lower & np.isclose(lower_velocity, joint_lower))
            | (active_upper & np.isclose(upper_velocity, joint_upper))
        )
        next_reference = self.model.clamp(
            self.q_reference + dq_limited * self.config.dt_s
        )
        # Anti-windup is a bound on the internal position reference rather than
        # a physical qdot constraint. Applying it after the box-QP avoids an
        # infeasible intersection with acceleration during actuator lag.
        lead = next_reference - q
        clamped_lead = np.clip(
            lead,
            -self.config.max_reference_lead_rad,
            self.config.max_reference_lead_rad,
        )
        reference_lead_clamped = np.abs(clamped_lead - lead) > 1.0e-12
        next_reference = self.model.clamp(q + clamped_lead)
        effective_dq = (next_reference - self.q_reference) / self.config.dt_s

        self.q_reference = next_reference
        self.dq_reference = effective_dq
        singular_values = np.linalg.svd(weighted_jacobian, compute_uv=False)
        minimum = float(singular_values[-1]) if len(singular_values) else 0.0
        return DifferentialTrackingStep(
            joint_position_reference_rad=self.q_reference.copy(),
            joint_velocity_reference_rad_s=self.dq_reference.copy(),
            task_error=error.copy(),
            task_velocity_command=task_command.copy(),
            minimum_singular_value=minimum,
            velocity_saturated=velocity_saturated,
            acceleration_saturated=acceleration_saturated,
            reference_lead_clamped=reference_lead_clamped,
            joint_limit_active=joint_limit_active,
        )


__all__ = [
    "DifferentialTrackingConfig",
    "DifferentialTrackingStep",
    "DifferentialUpperBodyTracker",
]
