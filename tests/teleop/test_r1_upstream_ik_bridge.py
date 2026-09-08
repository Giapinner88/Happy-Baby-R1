"""Checks for the bridge around the vendor `xr_teleoperate` IK.

The vendor solver itself is not under test here: it lives in `third_party` and
is run unmodified. What is under test is everything this repository puts around
it, which is where the mistakes were. Two in particular are worth pinning:

* The vendor `r1_a5.urdf` gives `waist_yaw_joint` a zero origin, so its root
  frame coincides with `waist_yaw_link`. This project's `R1.urdf` puts a real
  `waist_roll_link` between pelvis and waist and carries a 58.8 mm offset there.
  Converting targets between the two assets as though the offset applied to
  both displaced every target by exactly that distance, and the resulting error
  looked like poor tracking rather than a frame bug.
* Head limits are outside the vendor reduced model, which locks both head
  joints. They must come from the asset; a transcribed pair was wrong on both
  joints the first time it was written.

These run without CasADi so they stay in the default suite, which is the point:
the vendor solver only runs in the `tv` environment, but the frame reasoning
around it must be checked everywhere.
"""

from __future__ import annotations

import importlib.util
import json
import math
import re
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
VENDOR_URDF = ROOT / "third_party" / "xr_teleoperate_v1_6" / "assets" / "r1" / "r1_a5.urdf"
PROJECT_URDF = ROOT / "assets" / "R1.urdf"


def load_stream_module():
    """Import the streamer without running it.

    It is a script, not a package module, and importing it must not trigger the
    working-directory change the solver load performs.
    """

    path = ROOT / "scripts" / "teleop" / "run_r1_upstream_ik_stream.py"
    spec = importlib.util.spec_from_file_location("r1_upstream_ik_stream", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a proper rotation, via the trace-positive branch.

    The rotations here are head poses well away from a 180 degree turn, so the
    branch is always valid and a general converter would add nothing.
    """

    w = math.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0
    return (
        float((rotation[2, 1] - rotation[1, 2]) / (4.0 * w)),
        float((rotation[0, 2] - rotation[2, 0]) / (4.0 * w)),
        float((rotation[1, 0] - rotation[0, 1]) / (4.0 * w)),
        float(w),
    )


def pose_dict(transform: np.ndarray) -> dict[str, dict[str, float]]:
    return {
        "position": dict(zip("xyz", (float(value) for value in transform[:3, 3]))),
        "orientation": dict(zip("xyzw", quaternion_from_rotation(transform[:3, :3]))),
    }


def yaw_rotation(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def vendor_head_relative_wrist(world_wrist: np.ndarray, head: np.ndarray) -> np.ndarray:
    """The audited TeleVuer ``head_yaw`` transform, reproduced for a fixture."""

    head_yaw = load_stream_module().head_yaw_rotation(head)
    result = np.eye(4)
    result[:3, :3] = head_yaw.T @ world_wrist[:3, :3]
    result[:3, 3] = (
        head_yaw.T @ (world_wrist[:3, 3] - head[:3, 3])
        + np.array([0.15, 0.0, 0.45])
    )
    return result


def joint_origin(urdf_path: Path, joint_name: str) -> np.ndarray:
    text = urdf_path.read_text(encoding="utf-8")
    match = re.search(
        rf'<joint name="{joint_name}".*?<origin xyz="([^"]+)"', text, re.DOTALL
    )
    if match is None:
        raise AssertionError(f"{urdf_path.name} declares no origin for {joint_name}")
    return np.array([float(value) for value in match.group(1).split()])


class VendorAssetFrameTest(unittest.TestCase):
    def test_vendor_waist_yaw_origin_is_zero(self):
        """The premise behind passing targets to the vendor unconverted."""

        origin = joint_origin(VENDOR_URDF, "waist_yaw_joint")
        np.testing.assert_allclose(origin, np.zeros(3), atol=1e-12)

    def test_project_waist_yaw_origin_is_not_zero(self):
        """The offset that must NOT be applied to vendor targets.

        If this ever becomes zero the two assets have converged and the
        distinction the driver draws is no longer load-bearing, which is worth
        being told about rather than silently carrying dead code.
        """

        origin = joint_origin(PROJECT_URDF, "waist_yaw_joint")
        self.assertGreater(float(np.linalg.norm(origin)), 0.05)

    def test_both_assets_agree_on_arm_joint_names(self):
        vendor = set(re.findall(r'<joint name="([^"]+)"', VENDOR_URDF.read_text(encoding="utf-8")))
        project = set(re.findall(r'<joint name="([^"]+)"', PROJECT_URDF.read_text(encoding="utf-8")))
        module = load_stream_module()
        for name in module.ARM_JOINT_NAMES:
            self.assertIn(name, vendor, f"{name} missing from the vendor asset")
            self.assertIn(name, project, f"{name} missing from the project asset")


class StreamHelperTest(unittest.TestCase):
    def setUp(self):
        self.module = load_stream_module()

    def test_head_limits_are_read_from_the_asset(self):
        limits = self.module.head_limits_rad(PROJECT_URDF)
        self.assertEqual(len(limits), 2)
        for lower, upper in limits:
            self.assertLess(lower, upper)
        # Pinned against the asset so a transcription drifting back in fails.
        pitch, yaw = limits
        self.assertAlmostEqual(pitch[1], 0.62832, places=5)
        self.assertAlmostEqual(yaw[1], 2.0071, places=4)

    def test_head_angles_are_clipped_to_the_asset(self):
        limits = ((-0.1, 0.1), (-0.2, 0.2))
        # Yaw of +90 degrees, far outside the tightened bound below.
        pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)},
        }
        pitch, yaw = self.module.head_angles(pose, limits)
        self.assertLessEqual(yaw, 0.2 + 1e-12)
        self.assertGreaterEqual(pitch, -0.1 - 1e-12)

    def test_head_angles_invert_the_asset_head_chain(self):
        """The angles must aim the head where the headset looks.

        R1 applies `head_pitch` before `head_yaw`, so a textbook ZYX reading is
        right on a pure yaw and on a pure pitch and wrong on every combination
        of the two. The pairs below are all mixed for that reason; a ZYX reading
        misses the 30/20 deg case by several degrees.
        """

        from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model

        model = load_r1_a5_upper_body_model(control_waist_yaw=False)
        limits = self.module.head_limits_rad(PROJECT_URDF)
        for pitch_deg, yaw_deg in ((20.0, 30.0), (-15.0, -45.0), (30.0, -60.0), (-5.0, 70.0)):
            with self.subTest(pitch=pitch_deg, yaw=yaw_deg):
                pitch_rad, yaw_rad = math.radians(pitch_deg), math.radians(yaw_deg)
                rotation = model.head_rotation(pitch_rad, yaw_rad)
                pose = {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": dict(zip("xyzw", quaternion_from_rotation(rotation))),
                }
                read_pitch, read_yaw = self.module.head_angles(pose, limits)
                self.assertAlmostEqual(read_pitch, max(min(pitch_rad, limits[0][1]), limits[0][0]), places=9)
                self.assertAlmostEqual(read_yaw, yaw_rad, places=9)
                # The commanded pair must reproduce the headset's forward axis.
                np.testing.assert_allclose(
                    model.head_rotation(read_pitch, read_yaw)[:, 0], rotation[:, 0], atol=1e-9
                )

    def test_relative_head_angles_make_initial_pose_neutral(self):
        from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model

        model = load_r1_a5_upper_body_model(control_waist_yaw=False)
        limits = self.module.head_limits_rad(PROJECT_URDF)
        initial_rotation = model.head_rotation(math.radians(-14.0), math.radians(-53.0))
        initial_pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": dict(zip("xyzw", quaternion_from_rotation(initial_rotation))),
        }
        neutral = self.module._head_angles_unbounded(initial_pose)

        self.assertEqual(self.module.relative_head_angles(initial_pose, neutral, limits), (0.0, 0.0))

        moved_rotation = model.head_rotation(math.radians(-4.0), math.radians(-33.0))
        moved_pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": dict(zip("xyzw", quaternion_from_rotation(moved_rotation))),
        }
        pitch, yaw = self.module.relative_head_angles(moved_pose, neutral, limits)
        self.assertAlmostEqual(pitch, math.radians(10.0), places=9)
        self.assertAlmostEqual(yaw, math.radians(20.0), places=9)

    def test_relative_head_angles_clip_after_neutral_is_removed(self):
        limits = ((-0.1, 0.1), (-0.2, 0.2))
        pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(math.radians(40.0) / 2.0),
                "w": math.cos(math.radians(40.0) / 2.0),
            },
        }
        pitch, yaw = self.module.relative_head_angles(pose, (0.0, 0.0), limits)
        self.assertAlmostEqual(pitch, 0.0, places=12)
        self.assertAlmostEqual(yaw, 0.2, places=12)

    def test_head_neutral_ignores_a_one_sample_deadman_pulse(self):
        limits = self.module.head_limits_rad(PROJECT_URDF)
        calibrator = self.module.HeadNeutralCalibrator(limits, confirm_samples=3)
        pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        self.assertEqual(calibrator.update(pose, True), (0.0, 0.0))
        self.assertIsNone(calibrator.neutral)
        self.assertFalse(calibrator.ready)
        calibrator.update(pose, False)
        self.assertEqual(calibrator.candidate_count, 0)

    def test_head_neutral_is_captured_after_sustained_deadman(self):
        limits = self.module.head_limits_rad(PROJECT_URDF)
        calibrator = self.module.HeadNeutralCalibrator(limits, confirm_samples=3)
        pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(math.radians(-50.0) / 2.0),
                "w": math.cos(math.radians(-50.0) / 2.0),
            },
        }

        for _ in range(3):
            self.assertEqual(calibrator.update(pose, True), (0.0, 0.0))
        self.assertIsNotNone(calibrator.neutral)
        self.assertTrue(calibrator.ready)
        np.testing.assert_allclose(calibrator.anchor_pose_matrix, self.module.pose_to_matrix(pose))

        # Deadman release holds the last target rather than following the headset.
        turned_pose = dict(pose)
        turned_pose["orientation"] = {
            "x": 0.0, "y": 0.0,
            "z": math.sin(math.radians(20.0) / 2.0),
            "w": math.cos(math.radians(20.0) / 2.0),
        }
        self.assertEqual(calibrator.update(turned_pose, False), (0.0, 0.0))
        _pitch, yaw = calibrator.update(turned_pose, True)
        self.assertAlmostEqual(yaw, math.radians(70.0), places=9)

    def test_wrist_reanchor_removes_current_head_translation_and_yaw(self):
        world_wrist = np.eye(4)
        world_wrist[:3, :3] = yaw_rotation(math.radians(15.0))
        world_wrist[:3, 3] = [0.62, -0.18, 1.20]

        anchor_head = np.eye(4)
        anchor_head[:3, :3] = yaw_rotation(math.radians(-35.0))
        anchor_head[:3, 3] = [0.10, 0.05, 1.68]

        moved_head = np.eye(4)
        moved_head[:3, :3] = yaw_rotation(math.radians(42.0))
        moved_head[:3, 3] = [-0.08, 0.21, 1.55]

        wrist_at_anchor = vendor_head_relative_wrist(world_wrist, anchor_head)
        wrist_after_head_motion = vendor_head_relative_wrist(world_wrist, moved_head)
        self.assertGreater(
            float(np.linalg.norm(wrist_after_head_motion - wrist_at_anchor)),
            0.1,
            "fixture must reproduce visible vendor head coupling",
        )

        reanchored = self.module.reanchor_wrist_matrix(
            wrist_after_head_motion, moved_head, anchor_head
        )
        np.testing.assert_allclose(reanchored, wrist_at_anchor, atol=1e-12)

    def test_wrist_reanchor_preserves_real_controller_motion(self):
        anchor_head = np.eye(4)
        anchor_head[:3, :3] = yaw_rotation(math.radians(20.0))
        anchor_head[:3, 3] = [0.0, 0.0, 1.65]
        moved_head = np.eye(4)
        moved_head[:3, :3] = yaw_rotation(math.radians(-25.0))
        moved_head[:3, 3] = [0.12, -0.04, 1.58]

        first_world_wrist = np.eye(4)
        first_world_wrist[:3, 3] = [0.55, 0.25, 1.20]
        moved_world_wrist = first_world_wrist.copy()
        moved_world_wrist[:3, 3] += [0.08, -0.03, 0.05]

        expected = vendor_head_relative_wrist(moved_world_wrist, anchor_head)
        observed = self.module.reanchor_wrist_matrix(
            vendor_head_relative_wrist(moved_world_wrist, moved_head),
            moved_head,
            anchor_head,
        )
        np.testing.assert_allclose(observed, expected, atol=1e-12)

    def test_pose_to_matrix_is_a_rotation_plus_translation(self):
        pose = {
            "position": {"x": 0.3, "y": -0.1, "z": 0.05},
            "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
        }
        transform = self.module.pose_to_matrix(pose)
        rotation = transform[:3, :3]
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)
        np.testing.assert_allclose(transform[:3, 3], [0.3, -0.1, 0.05], atol=1e-12)
        np.testing.assert_allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_pose_to_matrix_rejects_a_zero_quaternion(self):
        pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
        }
        with self.assertRaises(ValueError):
            self.module.pose_to_matrix(pose)

    def test_emitted_joint_order_matches_the_project_model(self):
        from teleop.r1.upper_body_kinematics import ARMS_HEAD_JOINT_NAMES

        emitted = tuple(self.module.ARM_JOINT_NAMES) + tuple(self.module.HEAD_JOINT_NAMES)
        self.assertEqual(emitted, tuple(ARMS_HEAD_JOINT_NAMES))


class DriverContractTest(unittest.TestCase):
    """The offline driver must keep producing what the Isaac replay reads."""

    def test_driver_declares_the_keys_the_replay_sink_requires(self):
        source = (ROOT / "scripts" / "teleop" / "solve_r1_t007_upstream_ik.py").read_text(
            encoding="utf-8"
        )
        for key in (
            "time_s",
            "sequence_id",
            "joint_names",
            "joint_position_reference_rad",
            "left_target_position_pelvis_m",
            "right_target_position_pelvis_m",
        ):
            self.assertIn(key, source, f"driver no longer writes {key}")
        # The replay sink leaves the torso soft unless the config says otherwise,
        # which cost about 22 degrees of waist deflection when it was missing.
        self.assertIn("hold_uncontrolled_waist_joints", source)


if __name__ == "__main__":
    unittest.main()
