from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from scripts.teleop.solve_r1_t007_offline_continuation import (
    R1_A5_MIRROR_JOINT_SIGNS,
    canonicalize_right_arm_problem,
    rate_limit_trajectory,
)
from teleop.r1.kinematics import load_arm_chain
from teleop.r1.offline_continuation import (
    OfflineContinuationConfig,
    elbow_pole_vector_m,
    solve_offline_arm_trajectory,
)
from teleop.r1.workspace_projection import project_arm_position_to_reach_sphere


ROOT = Path(__file__).resolve().parents[2]


class OfflineContinuationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.chain = load_arm_chain("left", ROOT / "assets/R1.urdf")

    def config(self, **overrides: object) -> OfflineContinuationConfig:
        values = {
            "anchor_multistart_count": 8,
            "retained_anchor_candidates": 2,
            "continuation_max_nfev": 30,
            "refinement_sweeps": 1,
            "random_seed": 11,
            "minimum_anchor_straightness_deg": 0.0,
            "anchor_straightness_weights": (0.0,),
        }
        values.update(overrides)
        return OfflineContinuationConfig(**values)

    def test_bidirectional_continuation_tracks_one_smooth_reachable_path(self) -> None:
        time = np.linspace(0.0, 0.7, 15)
        q_truth = np.zeros((len(time), self.chain.dof))
        q_truth[:, 0] = np.linspace(0.0, -0.35, len(time))
        q_truth[:, 1] = np.linspace(0.0, 0.55, len(time))
        q_truth[:, 2] = np.linspace(0.0, 0.45, len(time))
        q_truth[:, 3] = np.linspace(0.0, 0.65, len(time))
        poses = [self.chain.forward_kinematics(q) for q in q_truth]
        positions = np.asarray([pose[:3, 3] for pose in poses])
        orientations = np.asarray([pose[:3, :3] for pose in poses])
        result = solve_offline_arm_trajectory(
            self.chain,
            time,
            positions,
            orientations,
            np.zeros(self.chain.dof),
            len(time) - 1,
            self.config(manifold_reprojection_sweeps=1),
        )
        self.assertEqual(result.q_rad.shape, q_truth.shape)
        self.assertLess(float(np.quantile(result.position_error_m, 0.95)), 0.01)
        self.assertLess(float(np.max(np.abs(np.diff(result.q_rad, axis=0)))), 0.5)
        self.assertTrue(np.all(result.q_rad >= self.chain.lower_limits))
        self.assertTrue(np.all(result.q_rad <= self.chain.upper_limits))

    def test_rejects_nonincreasing_time(self) -> None:
        pose = self.chain.forward_kinematics(np.zeros(self.chain.dof))
        with self.assertRaises(ValueError):
            solve_offline_arm_trajectory(
                self.chain,
                np.array([0.0, 0.1, 0.1]),
                np.tile(pose[:3, 3], (3, 1)),
                np.tile(pose[:3, :3], (3, 1, 1)),
                np.zeros(self.chain.dof),
                1,
                self.config(),
            )

    def test_neutral_r1_elbow_pole_is_backward_outward_and_down(self) -> None:
        left = elbow_pole_vector_m(self.chain, np.zeros(self.chain.dof))
        self.assertLess(left[0], 0.0)  # backward, never forward
        self.assertGreater(left[1], 0.0)  # outward for the left arm
        self.assertLess(left[2], 0.0)  # down, never up

    def test_absolute_vendor_target_is_projected_before_arm_ik(self) -> None:
        shoulder = self.chain.shoulder_origin()
        raw = shoulder + np.array([0.0, 0.9, 0.0])
        projected, changed, distance, reach = project_arm_position_to_reach_sphere(
            self.chain, raw, margin_m=0.01
        )
        self.assertTrue(changed)
        self.assertAlmostEqual(
            float(np.linalg.norm(projected - shoulder)), reach, places=12
        )
        self.assertAlmostEqual(distance, 0.9 - reach, places=12)

    def test_right_canonicalization_is_fk_mirror_equivariant(self) -> None:
        right = load_arm_chain("right", ROOT / "assets/R1.urdf")
        left_q = np.array([0.3, 0.7, -0.4, 0.9, 0.2])
        right_q = left_q * R1_A5_MIRROR_JOINT_SIGNS
        right_pose = right.forward_kinematics(right_q)
        positions, orientations, canonical_q = canonicalize_right_arm_problem(
            right_pose[None, :3, 3], right_pose[None, :3, :3], right_q[None]
        )
        left_pose = self.chain.forward_kinematics(left_q)
        self.assertTrue(np.allclose(positions[0], left_pose[:3, 3], atol=1e-12))
        self.assertTrue(np.allclose(orientations[0], left_pose[:3, :3], atol=1e-12))
        self.assertTrue(np.allclose(canonical_q[0], left_q, atol=1e-12))

    def test_rate_limiter_brakes_before_joint_limit(self) -> None:
        time = np.linspace(0.0, 1.0, 31)
        desired = np.full((len(time), 1), 2.0)
        desired[0, 0] = 0.0
        q, velocity, acceleration = rate_limit_trajectory(
            desired,
            time,
            np.array([-1.0]),
            np.array([1.0]),
            maximum_velocity_rad_s=4.0,
            maximum_acceleration_rad_s2=12.0,
        )
        self.assertLessEqual(float(np.max(q)), 1.0 + 1.0e-12)
        self.assertLessEqual(float(np.max(np.abs(velocity))), 4.0 + 1.0e-12)
        self.assertLessEqual(float(np.max(np.abs(acceleration))), 12.0 + 1.0e-10)


if __name__ == "__main__":
    unittest.main()
