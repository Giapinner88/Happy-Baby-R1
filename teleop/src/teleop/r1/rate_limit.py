"""Transport- and simulator-independent online joint target rate limiter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class OnlineJointLimiter:
    """Acceleration- and velocity-limited follower for a stream of joint targets.

    Supplying ``lower_limits``/``upper_limits`` makes the limiter respect the
    joint range. This is not redundant with clamping the solver's output: the
    limiter is second order, so stored velocity carries the command past a
    target that reverses direction, and an unbounded limiter can therefore emit
    a position outside the joint range even when every requested target was
    inside it. Clamping also zeroes the velocity of the clamped joints, so the
    limiter stops integrating velocity into a limit it cannot pass.
    """

    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    dt_s: float
    position: np.ndarray | None = None
    velocity: np.ndarray | None = None
    lower_limits: np.ndarray | None = None
    upper_limits: np.ndarray | None = None

    def __post_init__(self) -> None:
        if min(self.max_velocity_rad_s, self.max_acceleration_rad_s2, self.dt_s) <= 0.0:
            raise ValueError("Joint limiter velocity, acceleration and timestep must be positive.")
        if (self.lower_limits is None) != (self.upper_limits is None):
            raise ValueError("Joint limiter position limits must be supplied as a pair.")
        if self.lower_limits is not None:
            self.lower_limits = np.asarray(self.lower_limits, dtype=float)
            self.upper_limits = np.asarray(self.upper_limits, dtype=float)
            if self.lower_limits.shape != self.upper_limits.shape:
                raise ValueError("Joint limiter position limits must have the same shape.")
            if np.any(self.lower_limits > self.upper_limits):
                raise ValueError("A joint limiter lower limit exceeds its upper limit.")

    def reset(self, position: Sequence[float]) -> None:
        values = np.asarray(position, dtype=float)
        if values.ndim != 1 or not np.all(np.isfinite(values)):
            raise ValueError("Joint limiter position must be a finite vector.")
        self.position = values.copy()
        self.velocity = np.zeros_like(self.position)

    def step(self, desired: Sequence[float]) -> np.ndarray:
        goal = np.asarray(desired, dtype=float)
        if self.position is None or self.velocity is None:
            self.reset(goal)
            return goal.copy()
        if goal.shape != self.position.shape or not np.all(np.isfinite(goal)):
            raise ValueError("Rate limiter target is malformed.")
        requested_v = np.clip(
            (goal - self.position) / self.dt_s,
            -self.max_velocity_rad_s,
            self.max_velocity_rad_s,
        )
        velocity = self.velocity + np.clip(
            requested_v - self.velocity,
            -self.max_acceleration_rad_s2 * self.dt_s,
            self.max_acceleration_rad_s2 * self.dt_s,
        )
        velocity = np.clip(velocity, -self.max_velocity_rad_s, self.max_velocity_rad_s)
        next_position = self.position + velocity * self.dt_s
        overshot = (goal - self.position) * (goal - next_position) <= 0.0
        self.position = np.where(overshot, goal, next_position)
        self.velocity = np.where(overshot, 0.0, velocity)
        if self.lower_limits is not None:
            clamped = np.clip(self.position, self.lower_limits, self.upper_limits)
            # Stop the joints that hit a limit; carrying their velocity forward
            # would keep integrating against the limit and delay the reversal.
            self.velocity = np.where(clamped != self.position, 0.0, self.velocity)
            self.position = clamped
        return self.position.copy()

    def hold(self) -> None:
        if self.velocity is not None:
            self.velocity.fill(0.0)


__all__ = ["OnlineJointLimiter"]
