"""Time-scaled minimum-jerk joint trajectories for simulation-only teleop.

The planner is deliberately independent of Isaac Lab and the Quest stack.  A
caller supplies *all* numeric limits through an experiment configuration; this
module merely chooses the shortest fifth-order segment whose analytic velocity,
acceleration and jerk envelopes do not exceed them.  It is therefore useful for
the T003 replay and directly unit-testable without a GPU.

This is a command-target planner, not a hardware safety controller.  A
fail-closed input event still takes priority over trajectory continuity: the
caller must freeze the last issued target and record that hold separately.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class TrajectoryConfigError(ValueError):
    """Raised when a trajectory limit or sample request is unusable."""


@dataclass(frozen=True)
class JointTrajectoryLimits:
    """Per-joint positive envelopes, in rad/s, rad/s² and rad/s³."""

    max_velocity_rad_s: np.ndarray
    max_acceleration_rad_s2: np.ndarray
    max_jerk_rad_s3: np.ndarray

    def validate(self) -> None:
        values = (
            ("max_velocity_rad_s", self.max_velocity_rad_s),
            ("max_acceleration_rad_s2", self.max_acceleration_rad_s2),
            ("max_jerk_rad_s3", self.max_jerk_rad_s3),
        )
        shape: tuple[int, ...] | None = None
        for name, raw in values:
            array = np.asarray(raw, dtype=float)
            if array.ndim != 1 or array.size == 0:
                raise TrajectoryConfigError(f"{name} must be a non-empty one-dimensional array.")
            if not np.all(np.isfinite(array)) or np.any(array <= 0.0):
                raise TrajectoryConfigError(f"{name} must contain only finite positive values.")
            if shape is None:
                shape = array.shape
            elif array.shape != shape:
                raise TrajectoryConfigError("All joint-limit arrays must have the same shape.")

    @property
    def dof(self) -> int:
        self.validate()
        return int(np.asarray(self.max_velocity_rad_s).size)


@dataclass(frozen=True)
class JointTrajectorySample:
    """Position and first three time derivatives at one segment time."""

    position_rad: np.ndarray
    velocity_rad_s: np.ndarray
    acceleration_rad_s2: np.ndarray
    jerk_rad_s3: np.ndarray


@dataclass(frozen=True)
class MinimumJerkSegment:
    """One rest-to-rest quintic joint segment with analytic derivatives."""

    start_rad: np.ndarray
    goal_rad: np.ndarray
    duration_s: float
    limits: JointTrajectoryLimits

    @classmethod
    def from_limits(
        cls, start_rad: np.ndarray, goal_rad: np.ndarray, limits: JointTrajectoryLimits
    ) -> "MinimumJerkSegment":
        """Construct the shortest segment within the declared component limits.

        For ``f(u)=10u³−15u⁴+6u⁵``, maxima of ``|f'|``, ``|f''|`` and
        ``|f'''|`` over ``[0, 1]`` are respectively 15/8, 10/sqrt(3), and
        60.  Scaling each joint displacement by these closed-form envelopes
        gives the shared duration required for a synchronized multi-joint move.
        """

        limits.validate()
        start = np.asarray(start_rad, dtype=float)
        goal = np.asarray(goal_rad, dtype=float)
        if start.shape != (limits.dof,) or goal.shape != (limits.dof,):
            raise TrajectoryConfigError(
                f"start_rad and goal_rad must both have shape ({limits.dof},), got {start.shape} and {goal.shape}."
            )
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
            raise TrajectoryConfigError("start_rad and goal_rad must be finite.")
        displacement = np.abs(goal - start)
        if not np.any(displacement):
            duration = 0.0
        else:
            velocity_duration = (15.0 / 8.0) * displacement / np.asarray(limits.max_velocity_rad_s)
            acceleration_duration = np.sqrt(
                (10.0 / np.sqrt(3.0)) * displacement / np.asarray(limits.max_acceleration_rad_s2)
            )
            jerk_duration = np.cbrt(60.0 * displacement / np.asarray(limits.max_jerk_rad_s3))
            duration = float(np.max(np.maximum.reduce((velocity_duration, acceleration_duration, jerk_duration))))
        return cls(start.copy(), goal.copy(), duration, limits)

    def sample(self, elapsed_s: float) -> JointTrajectorySample:
        """Sample at ``elapsed_s``; times outside the segment clamp to endpoints."""

        if not np.isfinite(elapsed_s):
            raise TrajectoryConfigError("elapsed_s must be finite.")
        self.limits.validate()
        if self.duration_s == 0.0:
            zeros = np.zeros_like(self.goal_rad)
            return JointTrajectorySample(self.goal_rad.copy(), zeros, zeros, zeros)
        u = float(np.clip(elapsed_s / self.duration_s, 0.0, 1.0))
        u2, u3, u4, u5 = u * u, u * u * u, u**4, u**5
        blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        d_blend = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
        dd_blend = 60.0 * u - 180.0 * u2 + 120.0 * u3
        ddd_blend = 60.0 - 360.0 * u + 360.0 * u2
        delta = self.goal_rad - self.start_rad
        return JointTrajectorySample(
            position_rad=self.start_rad + delta * blend,
            velocity_rad_s=delta * d_blend / self.duration_s,
            acceleration_rad_s2=delta * dd_blend / (self.duration_s**2),
            jerk_rad_s3=delta * ddd_blend / (self.duration_s**3),
        )


__all__ = [
    "JointTrajectoryLimits",
    "JointTrajectorySample",
    "MinimumJerkSegment",
    "TrajectoryConfigError",
]
