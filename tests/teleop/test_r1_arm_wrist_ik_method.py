"""Gate M audit for `docs/teleop/r1_arm_wrist_ik.md`.

Gate M is the method gate that must pass before any T002 IK run. It checks four
things: the unit/frame conventions the mapper actually uses, the zero/identity
mapping case, that the method's joint list and limits match the real asset, and
that joint ownership is disjoint and complete.

These tests read `assets/R1.urdf` directly rather than trusting the numbers
transcribed into the method record, so a silent asset change fails the gate
instead of quietly invalidating every downstream workspace claim. They need no
Quest, no Isaac Sim and no GPU.
"""

from __future__ import annotations

import math
import re
import unittest
from pathlib import Path

import numpy as np

from teleop.r1 import (
    Pose,
    Quaternion,
    R1JointOwnership,
    R1TeleopMapper,
    TeleopCalibration,
    TeleopLimits,
    Vector3,
)
from teleop.r1.schema import BaseVelocity, R1TeleopCommand
from teleop.r1.kinematics import R1_A5_END_EFFECTOR_OFFSET_M, load_arm_chain


REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = REPO_ROOT / "assets" / "R1.urdf"
METHOD_PATH = REPO_ROOT / "docs" / "teleop" / "r1_arm_wrist_ik.md"
VENDOR_R1_A5_URDF_PATH = REPO_ROOT / "third_party" / "xr_teleoperate_v1_6" / "assets" / "r1" / "r1_a5.urdf"

ARM_JOINTS = tuple(
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in ("shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint")
)
HEAD_JOINTS = ("head_yaw_joint", "head_pitch_joint")
ENDPOINT_LINKS = ("left_wrist_roll_link", "right_wrist_roll_link")


def urdf_revolute_joints() -> dict[str, dict[str, float]]:
    """Name -> limits for every revolute joint in the asset."""

    text = URDF_PATH.read_text(encoding="utf-8")
    joints: dict[str, dict[str, float]] = {}
    for name, body in re.findall(r'<joint\s+name="([^"]+)"\s+type="revolute">(.*?)</joint>', text, re.S):
        limit = re.search(r"<limit([^/]*)/>", body)
        attributes = dict(re.findall(r'(\w+)="([^"]+)"', limit.group(1))) if limit else {}
        joints[name] = {key: float(value) for key, value in attributes.items()}
    return joints


def urdf_links() -> set[str]:
    return set(re.findall(r'<link\s+name="([^"]+)"', URDF_PATH.read_text(encoding="utf-8")))


class MethodRecordTests(unittest.TestCase):
    def test_method_record_exists(self) -> None:
        """T002 may not run without the method record Gate M audits."""

        self.assertTrue(METHOD_PATH.is_file(), f"Gate M method record missing: {METHOD_PATH}")

    def test_method_record_refuses_a_six_dof_orientation_target(self) -> None:
        text = METHOD_PATH.read_text(encoding="utf-8")
        self.assertIn("must not be read as a 6-DOF orientation target", text)


class UnitAndFrameAuditTests(unittest.TestCase):
    """Gate M item 1."""

    def test_endpoint_links_exist_in_the_asset(self) -> None:
        links = urdf_links()
        for link in ENDPOINT_LINKS:
            self.assertIn(link, links)

    def test_local_arm_geometry_and_position_limits_match_vendored_r1_a5(self) -> None:
        """The virtual tool offset is only valid for the matching R1-A5 chain."""

        self.assertTrue(VENDOR_R1_A5_URDF_PATH.is_file())
        local = URDF_PATH.read_text(encoding="utf-8")
        vendor = VENDOR_R1_A5_URDF_PATH.read_text(encoding="utf-8")

        def joint_body(text: str, name: str) -> str:
            match = re.search(r'<joint\s+name="' + re.escape(name) + r'"[^>]*>(.*?)</joint>', text, re.S)
            self.assertIsNotNone(match, name)
            return match.group(1)

        def attributes(body: str, tag: str) -> dict[str, str]:
            match = re.search(r'<'+ tag + r'([^/>]*)/?>', body)
            return dict(re.findall(r'(\w+)="([^"]+)"', match.group(1))) if match else {}

        for name in ARM_JOINTS:
            local_body, vendor_body = joint_body(local, name), joint_body(vendor, name)
            for tag in ("parent", "child", "origin", "axis"):
                self.assertEqual(attributes(local_body, tag), attributes(vendor_body, tag), f"{name}:{tag}")
            local_limit, vendor_limit = attributes(local_body, "limit"), attributes(vendor_body, "limit")
            for bound in ("lower", "upper"):
                self.assertAlmostEqual(float(local_limit[bound]), float(vendor_limit[bound]), places=12)

    def test_arm_chain_is_rooted_at_the_waist_not_the_pelvis(self) -> None:
        """The IK base moves with the waist, which locomotion owns."""

        text = URDF_PATH.read_text(encoding="utf-8")
        body = re.search(
            r'<joint\s+name="left_shoulder_pitch_joint"[^>]*>(.*?)</joint>', text, re.S
        ).group(1)
        parent = re.search(r'<parent link="([^"]+)"', body).group(1)
        self.assertEqual(parent, "waist_yaw_link")

    def test_mapper_uses_the_declared_frames(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(command_timeout_s=0.5))
        self.assertEqual(mapper.calibration.source_frame, "quest_headset")
        self.assertEqual(mapper.calibration.robot_frame, "r1_base")

    def test_foreign_source_frame_fails_closed_before_ik(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(command_timeout_s=0.5))
        target = mapper.map(self.command(source_frame="other_frame"), 1.0)
        self.assertFalse(target.enabled)
        self.assertEqual(target.reason, "source_frame_mismatch")
        self.assertIsNone(target.left_wrist_target)

    @staticmethod
    def command(*, source_frame: str = "quest_headset", deadman: bool = True) -> R1TeleopCommand:
        pose = Pose(Vector3(0.3, 0.2, 1.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        return R1TeleopCommand(
            sequence_id=1,
            timestamp_monotonic_s=1.0,
            deadman_enabled=deadman,
            head_pose=pose,
            left_wrist_pose=pose,
            right_wrist_pose=pose,
            base_velocity=BaseVelocity.zero(),
            source_frame=source_frame,
        )


class ZeroIdentityMappingTests(unittest.TestCase):
    """Gate M item 2."""

    def mapper(self) -> R1TeleopMapper:
        return R1TeleopMapper(TeleopCalibration(), TeleopLimits(command_timeout_s=0.5))

    def test_identity_calibration_maps_a_pose_to_itself(self) -> None:
        position = Vector3(0.31, -0.22, 1.05)
        pose = Pose(position, Quaternion(0.0, 0.0, 0.0, 1.0))
        command = R1TeleopCommand(
            sequence_id=1,
            timestamp_monotonic_s=1.0,
            deadman_enabled=True,
            head_pose=pose,
            left_wrist_pose=pose,
            right_wrist_pose=pose,
            base_velocity=BaseVelocity.zero(),
        )
        target = self.mapper().map(command, 1.0)
        self.assertTrue(target.enabled)
        for solved in (target.left_wrist_target, target.right_wrist_target):
            self.assertAlmostEqual(solved.position.x, position.x, places=12)
            self.assertAlmostEqual(solved.position.y, position.y, places=12)
            self.assertAlmostEqual(solved.position.z, position.z, places=12)

    def test_zero_pose_under_identity_calibration_stays_zero(self) -> None:
        pose = Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        command = R1TeleopCommand(
            sequence_id=1,
            timestamp_monotonic_s=1.0,
            deadman_enabled=True,
            head_pose=pose,
            left_wrist_pose=pose,
            right_wrist_pose=pose,
            base_velocity=BaseVelocity.zero(),
        )
        target = self.mapper().map(command, 1.0)
        self.assertAlmostEqual(target.head_yaw_rad, 0.0, places=12)
        self.assertAlmostEqual(target.head_pitch_rad, 0.0, places=12)
        self.assertAlmostEqual(target.left_wrist_target.position.x, 0.0, places=12)

    def test_no_scale_factor_is_applied(self) -> None:
        """Operator motion maps 1:1; a scale would change T002's workspace validity."""

        far = Vector3(1.7, -0.9, 0.4)
        pose = Pose(far, Quaternion(0.0, 0.0, 0.0, 1.0))
        command = R1TeleopCommand(
            sequence_id=1,
            timestamp_monotonic_s=1.0,
            deadman_enabled=True,
            head_pose=pose,
            left_wrist_pose=pose,
            right_wrist_pose=pose,
            base_velocity=BaseVelocity.zero(),
        )
        solved = self.mapper().map(command, 1.0).left_wrist_target.position
        self.assertAlmostEqual(solved.x, far.x, places=12)
        self.assertAlmostEqual(solved.y, far.y, places=12)
        self.assertAlmostEqual(solved.z, far.z, places=12)

    def test_r1_a5_controlled_endpoint_matches_vendor_virtual_frame(self) -> None:
        np.testing.assert_allclose(R1_A5_END_EFFECTOR_OFFSET_M, [0.20, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(
            load_arm_chain("left").endpoint_position(np.zeros(5)),
            [0.328371, 0.13860574468679437, -0.018035188737750694],
            atol=1e-12,
        )


class AssetJointListTests(unittest.TestCase):
    """Gate M item 3."""

    def test_every_arm_joint_declared_by_the_method_exists(self) -> None:
        joints = urdf_revolute_joints()
        for name in ARM_JOINTS:
            self.assertIn(name, joints)

    def test_each_arm_has_exactly_five_degrees_of_freedom(self) -> None:
        joints = urdf_revolute_joints()
        for side in ("left", "right"):
            arm = [
                name
                for name in joints
                if name.startswith(f"{side}_")
                and any(key in name for key in ("shoulder", "elbow", "wrist"))
            ]
            self.assertEqual(len(arm), 5, f"{side} arm: {sorted(arm)}")

    def test_the_asset_has_no_finger_or_gripper_joint(self) -> None:
        joints = urdf_revolute_joints()
        self.assertEqual([n for n in joints if "finger" in n or "gripper" in n], [])

    def test_wrist_roll_cannot_wrap(self) -> None:
        """The method's no-unwrapping claim depends on this staying true."""

        joints = urdf_revolute_joints()
        for side in ("left", "right"):
            limits = joints[f"{side}_wrist_roll_joint"]
            self.assertGreater(limits["lower"], -math.pi)
            self.assertLess(limits["upper"], math.pi)

    def test_shoulder_roll_range_is_mirrored_between_arms(self) -> None:
        """A mapping assuming a shared range drives one arm into its limit."""

        joints = urdf_revolute_joints()
        left = joints["left_shoulder_roll_joint"]
        right = joints["right_shoulder_roll_joint"]
        self.assertGreater(left["upper"], 0.0)
        self.assertLess(right["lower"], 0.0)
        self.assertAlmostEqual(left["upper"], -right["lower"], places=3)
        self.assertAlmostEqual(left["lower"], -right["upper"], places=3)

    def test_method_record_limits_match_the_asset(self) -> None:
        """The transcribed table must not drift from the asset it describes."""

        text = METHOD_PATH.read_text(encoding="utf-8")
        joints = urdf_revolute_joints()
        for name, expected in (
            ("left_shoulder_pitch_joint", ("−3.1416", "2.0944")),
            ("left_shoulder_roll_joint", ("−0.2269", "2.4784")),
            ("left_wrist_roll_joint", ("−1.9199", "1.9199")),
        ):
            lower, upper = expected
            self.assertIn(lower, text, name)
            self.assertIn(upper, text, name)
            self.assertAlmostEqual(joints[name]["lower"], -float(lower.replace("−", "")), places=4)
            self.assertAlmostEqual(joints[name]["upper"], float(upper), places=4)


class JointOwnershipTests(unittest.TestCase):
    """Gate M item 4."""

    def test_ownership_sets_are_disjoint(self) -> None:
        ownership = R1JointOwnership()
        ownership.validate()
        self.assertEqual(set(ownership.lower_body) & set(ownership.upper_body), set())

    def test_ownership_union_equals_the_asset_joint_set(self) -> None:
        """The gate's completeness check: no joint unowned, none invented."""

        ownership = R1JointOwnership()
        owned = set(ownership.lower_body) | set(ownership.upper_body)
        asset = set(urdf_revolute_joints())
        self.assertEqual(owned - asset, set(), "ownership names joints the asset does not have")
        self.assertEqual(asset - owned, set(), "asset has joints no owner claims")

    def test_ik_owns_the_ten_arm_joints(self) -> None:
        upper = set(R1JointOwnership().upper_body)
        self.assertTrue(set(ARM_JOINTS).issubset(upper))
        self.assertEqual(len(ARM_JOINTS), 10)

    def test_head_joints_are_dispatched_with_the_upper_body(self) -> None:
        upper = set(R1JointOwnership().upper_body)
        for name in HEAD_JOINTS:
            self.assertIn(name, upper)

    def test_locomotion_owns_the_waist(self) -> None:
        """Reassigning the waist would move the IK base frame; it needs a method change."""

        lower = set(R1JointOwnership().lower_body)
        self.assertIn("waist_roll_joint", lower)
        self.assertIn("waist_yaw_joint", lower)
        self.assertNotIn("waist_yaw_joint", set(R1JointOwnership().upper_body))

    def test_no_arm_joint_is_owned_by_locomotion(self) -> None:
        lower = set(R1JointOwnership().lower_body)
        self.assertEqual(set(ARM_JOINTS) & lower, set())


if __name__ == "__main__":
    unittest.main()
