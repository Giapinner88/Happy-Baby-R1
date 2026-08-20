"""Unit checks for the analytic T003 joint-trajectory limiter."""

from __future__ import annotations

import unittest

import numpy as np

from teleop.r1.trajectory import (
    JointTrajectoryLimits,
    MinimumJerkSegment,
    TrajectoryConfigError,
)


class MinimumJerkSegmentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = JointTrajectoryLimits(
            max_velocity_rad_s=np.full(5, 1.0),
            max_acceleration_rad_s2=np.full(5, 2.0),
            max_jerk_rad_s3=np.full(5, 10.0),
        )

    def test_analytic_samples_respect_declared_envelopes(self) -> None:
        segment = MinimumJerkSegment.from_limits(
            np.zeros(5), np.array([0.7, -0.4, 0.2, 1.1, -0.8]), self.limits
        )
        samples = [segment.sample(t) for t in np.linspace(0.0, segment.duration_s, 5001)]
        self.assertGreater(segment.duration_s, 0.0)
        self.assertLessEqual(max(np.max(np.abs(sample.velocity_rad_s)) for sample in samples), 1.0 + 1e-12)
        self.assertLessEqual(max(np.max(np.abs(sample.acceleration_rad_s2)) for sample in samples), 2.0 + 1e-12)
        self.assertLessEqual(max(np.max(np.abs(sample.jerk_rad_s3)) for sample in samples), 10.0 + 1e-12)

    def test_segment_is_rest_to_rest_at_its_endpoints(self) -> None:
        segment = MinimumJerkSegment.from_limits(np.zeros(5), np.ones(5), self.limits)
        for elapsed in (0.0, segment.duration_s):
            sample = segment.sample(elapsed)
            np.testing.assert_allclose(sample.velocity_rad_s, 0.0, atol=1e-12)
            np.testing.assert_allclose(sample.acceleration_rad_s2, 0.0, atol=1e-12)
        np.testing.assert_allclose(segment.sample(-1.0).position_rad, np.zeros(5))
        np.testing.assert_allclose(segment.sample(segment.duration_s + 1.0).position_rad, np.ones(5))

    def test_zero_displacement_is_a_stationary_segment(self) -> None:
        segment = MinimumJerkSegment.from_limits(np.ones(5), np.ones(5), self.limits)
        self.assertEqual(segment.duration_s, 0.0)
        np.testing.assert_allclose(segment.sample(12.0).position_rad, np.ones(5))

    def test_rejects_mismatched_or_nonpositive_limits(self) -> None:
        bad = JointTrajectoryLimits(np.ones(5), np.ones(4), np.ones(5))
        with self.assertRaises(TrajectoryConfigError):
            bad.validate()
        bad = JointTrajectoryLimits(np.ones(5), np.ones(5), np.array([1.0, 1.0, 1.0, 1.0, 0.0]))
        with self.assertRaises(TrajectoryConfigError):
            MinimumJerkSegment.from_limits(np.zeros(5), np.ones(5), bad)


if __name__ == "__main__":
    unittest.main()
