from __future__ import annotations

import unittest

import numpy as np

from scripts.teleop.plot_r1_quest3_telemetry import differentiate_position, segment_ids


class TelemetryKinematicsTests(unittest.TestCase):
    def test_quadratic_position_has_expected_velocity_and_acceleration(self) -> None:
        timestamps = np.array([0.0, 1.0, 2.0, 3.0])
        positions = np.column_stack((timestamps**2, np.zeros(4), np.zeros(4)))
        velocity, acceleration = differentiate_position(timestamps, positions, segment_ids(timestamps, 1.1))
        self.assertTrue(np.allclose(velocity[:, 0], [0.0, 2.0, 4.0, 6.0]))
        self.assertTrue(np.allclose(acceleration[:, 0], [2.0, 2.0, 2.0, 2.0]))

    def test_gap_prevents_cross_segment_derivative(self) -> None:
        timestamps = np.array([0.0, 0.1, 0.2, 1.0, 1.1])
        positions = np.column_stack((timestamps, np.zeros(5), np.zeros(5)))
        segments = segment_ids(timestamps, 0.2)
        velocity, acceleration = differentiate_position(timestamps, positions, segments)
        self.assertEqual(segments.tolist(), [0, 0, 0, 1, 1])
        self.assertTrue(np.allclose(velocity[:3, 0], 1.0))
        self.assertTrue(np.isnan(velocity[3:]).all())
        self.assertTrue(np.isnan(acceleration[3:]).all())
