from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from scripts.teleop.run_r1_quest3_live import _arm_straightness_deg
from teleop.r1.differential_tracking import (
    DifferentialTrackingConfig,
    DifferentialUpperBodyTracker,
)
from teleop.r1.upper_body_ik import UpperBodyIKTarget, upper_body_task_error
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model


ROOT = Path(__file__).resolve().parents[2]


def tracker_config(**overrides: object) -> DifferentialTrackingConfig:
    values = {
        "dt_s": 0.01,
        "position_gain_s": 8.0,
        "wrist_orientation_gain_s": 1.0,
        "head_orientation_gain_s": 8.0,
        "damping": 0.02,
        "finite_difference_rad": 1e-5,
        "position_weight": 10.0,
        "wrist_orientation_weight": 0.1,
        "head_orientation_weight": 0.5,
        "max_joint_velocity_rad_s": 3.5,
        "max_joint_acceleration_rad_s2": 20.0,
        "max_reference_lead_rad": 0.25,
        "posture_gain_s": 0.0,
    }
    values.update(overrides)
    return DifferentialTrackingConfig(**values)


class DifferentialTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_r1_a5_upper_body_model(
            ROOT / "assets/R1.urdf", control_waist_yaw=False
        )
        cls.nominal = np.zeros(cls.model.dof)

    def target_from_q(self, q: np.ndarray) -> UpperBodyIKTarget:
        state = self.model.forward_kinematics(q)
        return UpperBodyIKTarget(
            state.left_end_effector[:3, 3],
            state.left_end_effector[:3, :3],
            state.right_end_effector[:3, 3],
            state.right_end_effector[:3, :3],
            state.head[:3, :3],
        )

    def test_zero_error_and_velocity_holds_reference(self) -> None:
        controller = DifferentialUpperBodyTracker(
            self.model, self.nominal, self.nominal, tracker_config()
        )
        step = controller.step(
            self.nominal, self.target_from_q(self.nominal), np.zeros(15)
        )
        np.testing.assert_allclose(step.joint_position_reference_rad, self.nominal, atol=1e-10)
        np.testing.assert_allclose(step.joint_velocity_reference_rad_s, 0.0, atol=1e-10)

    def test_pose_feedback_reduces_a_reachable_error(self) -> None:
        desired_q = self.nominal.copy()
        desired_q[0] = 0.12
        desired_q[3] = 0.18
        controller = DifferentialUpperBodyTracker(
            self.model,
            self.nominal,
            self.nominal,
            tracker_config(max_joint_acceleration_rad_s2=1000.0),
        )
        target = self.target_from_q(desired_q)
        before = np.linalg.norm(upper_body_task_error(self.model, self.nominal, target)[:6])
        step = controller.step(self.nominal, target, np.zeros(15), velocity_feedforward=False)
        after = np.linalg.norm(
            upper_body_task_error(
                self.model, step.joint_position_reference_rad, target
            )[:6]
        )
        self.assertLess(after, before)

    def test_velocity_acceleration_and_joint_bounds_are_enforced(self) -> None:
        config = tracker_config(
            max_joint_velocity_rad_s=0.2,
            max_joint_acceleration_rad_s2=0.5,
        )
        controller = DifferentialUpperBodyTracker(
            self.model, self.nominal, self.nominal, config
        )
        target = self.target_from_q(self.model.upper_limits)
        step = controller.step(self.nominal, target, np.full(15, 100.0))
        self.assertTrue(np.any(step.velocity_saturated))
        self.assertTrue(np.any(step.acceleration_saturated))
        self.assertLessEqual(
            float(np.max(np.abs(step.joint_velocity_reference_rad_s))),
            config.max_joint_acceleration_rad_s2 * config.dt_s + 1e-12,
        )
        self.assertTrue(np.all(step.joint_position_reference_rad <= self.model.upper_limits))
        self.assertTrue(np.all(step.joint_position_reference_rad >= self.model.lower_limits))

    def test_bounded_solver_reoptimizes_free_joints_instead_of_clipping(self) -> None:
        # min (x0 + x1 - 1)^2 with x0 <= 0.2.  Clipping the unconstrained
        # [0.5, 0.5] answer gives only 0.7 task output; the box optimum recruits
        # the free second joint and retains the requested output.
        hessian = np.array([[1.0001, 1.0], [1.0, 1.0001]])
        rhs = np.ones(2)
        result = DifferentialUpperBodyTracker._bounded_quadratic_solve(
            hessian,
            rhs,
            np.zeros(2),
            np.array([0.2, 1.0]),
            np.array([0.5, 0.5]),
        )
        self.assertAlmostEqual(result[0], 0.2, places=6)
        self.assertAlmostEqual(float(np.sum(result)), 1.0, places=3)

    def test_arm_straightness_is_finite_and_independent_of_head_pose(self) -> None:
        baseline = _arm_straightness_deg(self.model, self.nominal, "left")
        with_head_motion = self.nominal.copy()
        with_head_motion[self.model.head_slice] = [0.3, -0.4]
        self.assertGreaterEqual(baseline, 0.0)
        self.assertLessEqual(baseline, 180.0)
        self.assertAlmostEqual(
            _arm_straightness_deg(self.model, with_head_motion, "left"), baseline
        )

    def test_strict_priority_uses_arm_nullspace_for_wrist_orientation(self) -> None:
        state = self.model.forward_kinematics(self.nominal)
        angle = 0.4
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        target = UpperBodyIKTarget(
            state.left_end_effector[:3, 3],
            rotation @ state.left_end_effector[:3, :3],
            state.right_end_effector[:3, 3],
            state.right_end_effector[:3, :3],
            state.head[:3, :3],
        )
        controller = DifferentialUpperBodyTracker(
            self.model,
            self.nominal,
            self.nominal,
            tracker_config(
                task_priority="position_then_orientation",
                wrist_orientation_weight=0.5,
                max_joint_acceleration_rad_s2=1000.0,
            ),
        )
        before = upper_body_task_error(self.model, self.nominal, target)
        step = controller.step(
            self.nominal, target, np.zeros(15), velocity_feedforward=False
        )
        after = upper_body_task_error(
            self.model, step.joint_position_reference_rad, target
        )
        self.assertGreater(
            np.linalg.norm(step.joint_velocity_reference_rad_s[:5]), 0.1
        )
        self.assertLess(np.linalg.norm(after[6:9]), np.linalg.norm(before[6:9]))
        self.assertLess(np.linalg.norm(after[:6]), 1e-4)

    def test_strict_priority_box_qp_preserves_posture_preference(self) -> None:
        posture = self.nominal.copy()
        posture[3] = 1.5
        posture[8] = 1.5
        controller = DifferentialUpperBodyTracker(
            self.model,
            posture,
            self.nominal,
            tracker_config(
                task_priority="position_then_orientation",
                posture_gain_s=2.0,
                max_joint_acceleration_rad_s2=1000.0,
            ),
        )
        target = self.target_from_q(self.nominal)
        step = controller.step(
            self.nominal, target, np.zeros(15), velocity_feedforward=False
        )
        self.assertGreater(
            np.linalg.norm(step.joint_velocity_reference_rad_s[:10]), 1e-3
        )
        after = upper_body_task_error(
            self.model, step.joint_position_reference_rad, target
        )
        self.assertLess(np.linalg.norm(after[:6]), 1e-3)

    def test_wide_gesture_activates_soft_elbow_pole_objective(self) -> None:
        state = self.model.forward_kinematics(self.nominal)
        left_position = state.left_end_effector[:3, 3].copy()
        right_position = state.right_end_effector[:3, 3].copy()
        left_position[1] = 0.55
        right_position[1] = -0.55
        target = UpperBodyIKTarget(
            left_position,
            state.left_end_effector[:3, :3],
            right_position,
            state.right_end_effector[:3, :3],
            state.head[:3, :3],
        )
        baseline = DifferentialUpperBodyTracker(
            self.model,
            self.nominal,
            self.nominal,
            tracker_config(
                task_priority="position_then_orientation",
                max_joint_acceleration_rad_s2=1000.0,
            ),
        ).step(self.nominal, target, np.zeros(15), velocity_feedforward=False)
        extension = DifferentialUpperBodyTracker(
            self.model,
            self.nominal,
            self.nominal,
            tracker_config(
                task_priority="position_then_orientation",
                max_joint_acceleration_rad_s2=1000.0,
                wide_elbow_pole_gain_s=6.0,
                wide_elbow_pole_weight=2.0,
                wide_hand_separation_start_m=0.75,
                wide_hand_separation_full_m=1.0,
            ),
        ).step(self.nominal, target, np.zeros(15), velocity_feedforward=False)
        self.assertEqual(extension.wide_elbow_pole_activation, 1.0)
        self.assertGreater(np.linalg.norm(extension.wide_elbow_pole_error_m), 0.0)
        self.assertGreater(
            np.linalg.norm(
                extension.joint_velocity_reference_rad_s
                - baseline.joint_velocity_reference_rad_s
            ),
            1e-3,
        )

    def test_limit_trap_activates_hysteretic_single_arm_branch_recovery(self) -> None:
        trapped = self.nominal.copy()
        right = self.model.right_arm_slice
        trapped[right.start + 0] = self.model.lower_limits[right.start + 0]
        trapped[right.start + 1] = self.model.lower_limits[right.start + 1]
        trapped[right.start + 3] = self.model.upper_limits[right.start + 3]
        controller = DifferentialUpperBodyTracker(
            self.model,
            self.nominal,
            trapped,
            tracker_config(
                task_priority="position_then_orientation",
                max_joint_acceleration_rad_s2=1000.0,
                branch_recovery_position_error_m=0.15,
                branch_recovery_exit_error_m=0.05,
                branch_recovery_limit_margin_rad=0.03,
                branch_recovery_gain_s=3.0,
                branch_recovery_weight=5.0,
            ),
        )
        step = controller.step(
            trapped,
            self.target_from_q(self.nominal),
            np.zeros(15),
            velocity_feedforward=False,
        )
        np.testing.assert_array_equal(step.branch_recovery_active, [False, True])
        self.assertGreater(step.branch_recovery_velocity_rad_s[right.start + 0], 0.0)
        self.assertGreater(step.branch_recovery_velocity_rad_s[right.start + 1], 0.0)
        self.assertLess(step.branch_recovery_velocity_rad_s[right.start + 3], 0.0)


if __name__ == "__main__":
    unittest.main()
