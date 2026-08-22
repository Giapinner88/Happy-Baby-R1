from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from teleop.r1.bridge import rotation_matrix_to_quaternion
from teleop.r1.differential_live import (
    DifferentialWholeUpperBodyIsaacLabSink,
    DifferentialWholeUpperBodyLiveConfig,
)
from teleop.r1.differential_tracking import DifferentialTrackingConfig
from teleop.r1.mapping import R1A5WholeUpperBodyOwnership, R1TeleopTargets
from teleop.r1.schema import BaseVelocity, Pose, Vector3
from teleop.r1.upper_body_ik import UpperBodyIKTarget
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model
from teleop.r1.workspace_projection import project_upper_body_target


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "assets" / "R1.urdf"


class FakeHandle:
    def __init__(self, dof: int = 12) -> None:
        self.position = np.zeros(dof)
        self.writes: list[np.ndarray] = []

    def write_joint_targets(self, joint_names, positions_rad) -> None:
        self.position = np.asarray(positions_rad, dtype=float).copy()
        self.writes.append(self.position.copy())

    def joint_positions(self, joint_names):
        return tuple(float(value) for value in self.position)


def pose(transform: np.ndarray) -> Pose:
    return Pose(
        Vector3(*(float(value) for value in transform[:3, 3])),
        rotation_matrix_to_quaternion(transform),
    )


def config() -> DifferentialWholeUpperBodyLiveConfig:
    return DifferentialWholeUpperBodyLiveConfig(
        urdf_path=URDF,
        nominal_joint_position_rad=tuple(np.zeros(12)),
        body_mode="arms_head",
        mapping_mode="absolute",
        tracker=DifferentialTrackingConfig(
            dt_s=0.02,
            position_gain_s=8.0,
            wrist_orientation_gain_s=1.0,
            head_orientation_gain_s=8.0,
            damping=0.1,
            finite_difference_rad=1e-5,
            position_weight=10.0,
            wrist_orientation_weight=0.01,
            head_orientation_weight=0.5,
            max_joint_velocity_rad_s=1.5,
            max_joint_acceleration_rad_s2=10.0,
            max_reference_lead_rad=0.25,
            posture_gain_s=0.2,
        ),
        velocity_feedforward_task="head_only",
    )


def relative_config() -> DifferentialWholeUpperBodyLiveConfig:
    absolute = config()
    return DifferentialWholeUpperBodyLiveConfig(
        urdf_path=absolute.urdf_path,
        nominal_joint_position_rad=absolute.nominal_joint_position_rad,
        tracker=absolute.tracker,
        body_mode=absolute.body_mode,
        mapping_mode="relative_session",
        position_scale=0.4,
        velocity_feedforward_task=absolute.velocity_feedforward_task,
    )


class WorkspaceProjectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_r1_a5_upper_body_model(URDF, control_waist_yaw=False)
        cls.q = np.zeros(12)
        state = cls.model.forward_kinematics(cls.q)
        cls.neutral = UpperBodyIKTarget(
            state.left_end_effector[:3, 3],
            state.left_end_effector[:3, :3],
            state.right_end_effector[:3, 3],
            state.right_end_effector[:3, :3],
            state.head[:3, :3],
        )

    def test_inside_target_is_unchanged(self) -> None:
        result = project_upper_body_target(self.model, self.q, self.neutral, 0.01)
        self.assertFalse(result.any_projected)
        np.testing.assert_allclose(
            result.target.left_position_m, self.neutral.left_position_m
        )

    def test_far_targets_are_projected_before_control(self) -> None:
        far = UpperBodyIKTarget(
            np.array([3.0, 3.0, 3.0]),
            self.neutral.left_orientation,
            np.array([3.0, -3.0, 3.0]),
            self.neutral.right_orientation,
            self.neutral.head_orientation,
        )
        result = project_upper_body_target(self.model, self.q, far, 0.01)
        self.assertTrue(result.left_projected)
        self.assertTrue(result.right_projected)
        waist = self.model.waist_transform_from_q(self.q)
        for side in ("left", "right"):
            chain = getattr(self.model, f"{side}_arm")
            shoulder = (waist @ np.append(chain.shoulder_origin(), 1.0))[:3]
            distance = np.linalg.norm(
                getattr(result.target, f"{side}_position_m") - shoulder
            )
            self.assertAlmostEqual(
                float(distance), getattr(result, f"{side}_reach_limit_m"), places=10
            )


class DifferentialLiveSinkTests(unittest.TestCase):
    def far_targets(self, sequence_id: int = 1) -> R1TeleopTargets:
        transform = np.eye(4)
        transform[:3, 3] = [3.0, 3.0, 3.0]
        return R1TeleopTargets(
            sequence_id=sequence_id,
            enabled=True,
            reason=None,
            left_wrist_target=pose(transform),
            right_wrist_target=pose(transform),
            head_yaw_rad=0.2,
            head_pitch_rad=-0.1,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame="r1_base",
        )

    def test_far_target_is_projected_and_dispatched_in_one_step(self) -> None:
        handle = FakeHandle()
        sink = DifferentialWholeUpperBodyIsaacLabSink(handle, config())
        sink.apply_upper_body(
            self.far_targets(),
            R1A5WholeUpperBodyOwnership(body_mode="arms_head").upper_body,
        )
        application = sink.last_application
        self.assertTrue(application["accepted"])
        self.assertEqual(application["controller_type"], "differential_dls")
        self.assertEqual(application["solver_solution_kind"], "workspace_projected")
        self.assertTrue(application["workspace_projection"]["left_projected"])
        self.assertNotIn("iterations", application)
        self.assertGreater(application["controller_compute_ms"], 0.0)
        self.assertEqual(len(handle.writes[-1]), 12)

    def test_hold_clears_reference_velocity(self) -> None:
        handle = FakeHandle()
        sink = DifferentialWholeUpperBodyIsaacLabSink(handle, config())
        sink.apply_upper_body(
            self.far_targets(),
            R1A5WholeUpperBodyOwnership(body_mode="arms_head").upper_body,
        )
        self.assertGreater(np.max(np.abs(sink.tracker.dq_reference)), 0.0)
        held = handle.position.copy()
        sink.hold("deadman_released")
        np.testing.assert_allclose(sink.tracker.dq_reference, 0.0)
        np.testing.assert_allclose(handle.position, held)

    def test_relative_session_latches_first_sample_and_scales_translation(self) -> None:
        handle = FakeHandle()
        sink = DifferentialWholeUpperBodyIsaacLabSink(handle, relative_config())
        ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head").upper_body
        sink.apply_upper_body(self.far_targets(sequence_id=1), ownership)
        first = dict(sink.last_application)
        np.testing.assert_allclose(
            first["raw_left_target_position_pelvis_m"],
            sink.model.forward_kinematics(np.zeros(12)).left_end_effector[:3, 3],
            atol=1e-12,
        )
        self.assertFalse(first["workspace_projection"]["left_projected"])

        moved = self.far_targets(sequence_id=2)
        assert moved.left_wrist_target is not None
        moved = replace(
            moved,
            left_wrist_target=replace(
                moved.left_wrist_target,
                position=replace(
                    moved.left_wrist_target.position,
                    x=moved.left_wrist_target.position.x + 0.25,
                ),
            ),
        )
        sink.apply_upper_body(moved, ownership)
        second = dict(sink.last_application)
        displacement = np.asarray(second["raw_left_target_position_pelvis_m"]) - np.asarray(
            first["raw_left_target_position_pelvis_m"]
        )
        np.testing.assert_allclose(displacement, [0.1, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
