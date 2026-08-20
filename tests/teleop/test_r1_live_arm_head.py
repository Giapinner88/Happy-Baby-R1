"""CPU-only contract checks for the T007 bilateral arm/head sink."""
from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from teleop.r1.ik import ArmIKConfig
from teleop.r1.kinematics import load_arm_chain
from teleop.r1.live_arm_head import ARM_HEAD_JOINT_NAMES, ArmHeadIsaacLabSink, ArmHeadLiveConfig
from teleop.r1.mapping import R1JointOwnership, R1TeleopTargets
from teleop.r1.schema import BaseVelocity, Pose, Quaternion, Vector3
from scripts.teleop.plot_r1_t007_dynamics import _endpoint_tracking

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_arm_head_live.json"


class Handle:
    def __init__(self) -> None:
        self.head = (0.0, 0.0); self.left = (0.0,) * 5; self.right = (0.0,) * 5
        self.arm_writes: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
    def write_head_targets(self, yaw_rad: float, pitch_rad: float) -> None: self.head = (yaw_rad, pitch_rad)
    def head_joint_positions(self) -> tuple[float, float]: return self.head
    def write_arm_targets(self, left_rad, right_rad) -> None:
        self.left, self.right = tuple(left_rad), tuple(right_rad); self.arm_writes.append((self.left, self.right))
    def arm_joint_positions(self): return self.left, self.right


def config() -> ArmHeadLiveConfig:
    raw = json.loads(CONFIG.read_text())["arm_head"]
    ik = ArmIKConfig(**raw["ik"])
    return ArmHeadLiveConfig(
        np.asarray(raw["left_neutral_position_m"]), np.asarray(raw["right_neutral_position_m"]),
        np.asarray(raw["left_workspace"]["lower_m"]), np.asarray(raw["left_workspace"]["upper_m"]),
        np.asarray(raw["right_workspace"]["lower_m"]), np.asarray(raw["right_workspace"]["upper_m"]),
        raw["position_scale"], raw["max_joint_velocity_rad_s"], raw["max_joint_acceleration_rad_s2"], 0.05, ik,
        enforce_workspace=raw.get("enforce_workspace", True),
        allow_converged_joint_limit_solution=raw.get("allow_converged_joint_limit_solution", False),
        mapping_mode=raw.get("mapping_mode", "relative_session"),
        allow_projected_position_solution=raw.get("allow_projected_position_solution", False),
        allow_clamped_roll_solution=raw.get("allow_clamped_roll_solution", False),
        neutral_joint_position_rad=tuple(raw["neutral_joint_position_rad"]),
    )


def targets(left: tuple[float, float, float], right: tuple[float, float, float], *, seq: int = 1) -> R1TeleopTargets:
    pose_l = Pose(Vector3(*left), Quaternion(0.0, 0.0, 0.0, 1.0)); pose_r = Pose(Vector3(*right), Quaternion(0.0, 0.0, 0.0, 1.0))
    return R1TeleopTargets(seq, True, None, pose_l, pose_r, 0.0, 0.0, BaseVelocity.zero(), False, "r1_base")


class ArmHeadSinkTests(unittest.TestCase):
    def test_declared_neutral_is_fk_of_straight_zero_joint_pose(self) -> None:
        cfg = config()
        np.testing.assert_allclose(cfg.left_neutral_position_m, load_arm_chain("left").endpoint_position(np.zeros(5)), atol=1e-12)
        np.testing.assert_allclose(cfg.right_neutral_position_m, load_arm_chain("right").endpoint_position(np.zeros(5)), atol=1e-12)
        self.assertEqual(cfg.position_scale, 1.0)

    def test_first_valid_sample_calibrates_then_relative_motion_drives_all_12_joints(self) -> None:
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, config())
        sink.apply_upper_body(targets((0.4, 0.2, 0.1), (0.4, -0.2, 0.1)), R1JointOwnership().upper_body)
        self.assertTrue(sink.calibrated); self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["left_target_position_m"], [0.4, 0.2, 0.1])
        self.assertEqual(sink.last_application["right_target_position_m"], [0.4, -0.2, 0.1])
        self.assertEqual(len(sink.last_application["left_ik_joint_target_rad"]), 5)
        self.assertEqual(len(sink.last_application["right_ik_joint_target_rad"]), 5)
        self.assertEqual(len(sink.last_application["left_rate_limiter_velocity_rad_s"]), 5)
        self.assertEqual(len(sink.last_application["right_rate_limiter_velocity_rad_s"]), 5)
        self.assertLess(abs(sink.left_seed[3]), 0.2)
        self.assertLess(abs(sink.right_seed[3]), 0.2)
        sink.apply_upper_body(targets((0.4, 0.25, 0.1), (0.4, -0.25, 0.1), seq=2), R1JointOwnership().upper_body)
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.acknowledgements[-1]["accepted_joints"], list(ARM_HEAD_JOINT_NAMES))
        self.assertEqual(len(handle.arm_writes), 2)

    def test_wide_lateral_pose_uses_the_full_vendor_endpoint_reach(self) -> None:
        """Regression for the observed curled-arm failure in the schema-1 run."""
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, config())
        left_target = np.asarray((0.25, 0.55, 0.10))
        right_target = np.asarray((0.25, -0.55, 0.10))
        sink.apply_upper_body(targets(tuple(left_target), tuple(right_target)), R1JointOwnership().upper_body)
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["left_solution_kind"], "exact")
        self.assertEqual(sink.last_application["right_solution_kind"], "exact")
        np.testing.assert_allclose(
            sink.left_chain.endpoint_position(sink.left_seed), left_target,
            atol=config().ik.position_tolerance_m,
        )
        np.testing.assert_allclose(
            sink.right_chain.endpoint_position(sink.right_seed), right_target,
            atol=config().ik.position_tolerance_m,
        )

    def test_optional_workspace_prefilter_holds_both_arms_and_head(self) -> None:
        bounded = replace(
            config(),
            enforce_workspace=True,
            left_upper_m=np.asarray((0.5, 0.4, 0.35)),
            right_upper_m=np.asarray((0.5, -0.05, 0.35)),
        )
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, bounded)
        sink.apply_upper_body(targets((0.4, 0.2, 0.1), (0.4, -0.2, 0.1)), R1JointOwnership().upper_body)
        old_writes = len(handle.arm_writes)
        sink.apply_upper_body(targets((1.0, 0.2, 0.1), (1.0, -0.2, 0.1), seq=2), R1JointOwnership().upper_body)
        self.assertEqual(sink.events[-1]["event"], "hold")
        self.assertEqual(sink.events[-1]["reason"], "workspace_outside_selected_envelope")
        self.assertEqual(len(handle.arm_writes), old_writes + 1)

    def test_ik_only_workspace_policy_does_not_hold_before_the_solver(self) -> None:
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, replace(config(), enforce_workspace=False))
        sink.apply_upper_body(targets((0.4, 0.2, 0.1), (0.4, -0.2, 0.1)), R1JointOwnership().upper_body)
        sink.apply_upper_body(targets((1.0, 0.2, 0.1), (1.0, -0.2, 0.1), seq=2), R1JointOwnership().upper_body)
        self.assertNotEqual(sink.events[-1].get("reason"), "workspace_outside_selected_envelope")
        self.assertEqual(sink.events[-1]["left_solution_kind"], "projected")
        self.assertIn("right_target_position_m", sink.events[-1])

    def test_converged_elbow_limit_solution_is_an_accepted_simulation_target_when_enabled(self) -> None:
        handle = Handle()
        sink = ArmHeadIsaacLabSink(
            handle,
            replace(config(), enforce_workspace=False, allow_converged_joint_limit_solution=True),
        )
        sink.apply_upper_body(targets((0.4, 0.2, 0.1), (0.4, -0.2, 0.1)), R1JointOwnership().upper_body)
        left_q = np.zeros(5); left_q[3] = load_arm_chain("left").lower_limits[3]
        right_q = np.zeros(5); right_q[3] = load_arm_chain("right").lower_limits[3]
        sink.left_seed = left_q.copy(); sink.right_seed = right_q.copy()
        left_source = np.asarray((-0.05, 0.26857142857142857, -0.1))
        right_source = np.asarray((-0.05, -0.26857142857142857, -0.1))
        sink.apply_upper_body(
            targets(tuple(left_source), tuple(right_source), seq=2),
            R1JointOwnership().upper_body,
        )
        self.assertTrue(sink.last_application["accepted"])
        self.assertIn("left_shoulder_roll_joint", sink.last_application["left_clamped_joints"])
        self.assertIn("right_shoulder_roll_joint", sink.last_application["right_clamped_joints"])

    def test_unreachable_absolute_target_projects_to_reachable_boundary(self) -> None:
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, config())
        sink.apply_upper_body(targets((0.8, 0.1, 0.3), (0.8, -0.1, 0.3)), R1JointOwnership().upper_body)
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["left_solution_kind"], "projected")
        self.assertGreater(sink.last_application["left_position_residual_m"], config().ik.position_tolerance_m)

    def test_operator_reset_returns_to_the_declared_neutral_pose(self) -> None:
        handle = Handle(); sink = ArmHeadIsaacLabSink(handle, config())
        sink.apply_upper_body(targets((0.4, 0.2, 0.1), (0.4, -0.2, 0.1)), R1JointOwnership().upper_body)
        sink.reset_session()
        np.testing.assert_allclose(sink.session_left_neutral, config().left_neutral_position_m)
        np.testing.assert_allclose(sink.session_right_neutral, config().right_neutral_position_m)
        self.assertEqual(sink.last_head, (0.0, 0.0))
        np.testing.assert_allclose(handle.left, np.zeros(5), atol=0.0)
        np.testing.assert_allclose(handle.right, np.zeros(5), atol=0.0)
        self.assertFalse(sink.calibrated)
        self.assertEqual(sink.events[-1]["reset_kind"], "declared_joint_pose")

    def test_base_is_not_in_the_arm_head_dispatch_set(self) -> None:
        self.assertFalse(set(ARM_HEAD_JOINT_NAMES) & set(R1JointOwnership().lower_body))


class T007EndpointFigureDataTests(unittest.TestCase):
    def test_schema2_reconstructs_observed_vendor_endpoint(self) -> None:
        q = np.zeros(5)
        requested = load_arm_chain("left").endpoint_position(q).tolist()
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            (run_dir / "experiment_config.json").write_text(
                json.dumps({"schema_version": 2, "experiment_id": "t007"}), encoding="utf-8"
            )
            (run_dir / "targets.json").write_text(json.dumps([{
                "elapsed_s": 1.0,
                "arm_head": {
                    "accepted": True,
                    "left_target_position_m": requested,
                    "left_solution_kind": "exact",
                    "left_limited_joint_target_rad": q.tolist(),
                    "left_position_residual_m": 0.0,
                },
                "post_physics_arm_position_rad": {"left": q.tolist()},
            }]), encoding="utf-8")
            reconstructed = _endpoint_tracking(run_dir)["left"]
        np.testing.assert_allclose(reconstructed["requested_m"], reconstructed["achieved_m"], atol=1e-12)
        np.testing.assert_allclose(reconstructed["commanded_m"], reconstructed["achieved_m"], atol=1e-12)
        np.testing.assert_allclose(reconstructed["solver_residual_m"], [0.0], atol=0.0)

    def test_schema1_is_not_reinterpreted_with_schema2_fk(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            (run_dir / "experiment_config.json").write_text(
                json.dumps({"schema_version": 1}), encoding="utf-8"
            )
            (run_dir / "targets.json").write_text("[]", encoding="utf-8")
            self.assertEqual(_endpoint_tracking(run_dir), {})


if __name__ == "__main__": unittest.main()
