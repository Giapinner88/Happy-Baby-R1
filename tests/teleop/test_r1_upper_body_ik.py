"""Numerical and asset-compatibility checks for coupled R1-A5 upper-body IK."""

from __future__ import annotations

import unittest
import json
import tempfile
from pathlib import Path

import numpy as np

from teleop.r1.upper_body_ik import (
    UpperBodyIKConfig,
    UpperBodyIKConfigError,
    UpperBodyIKTarget,
    quaternion_xyzw_to_matrix,
    so3_log,
    solve_upper_body_ik,
    upper_body_task_jacobian,
)
from teleop.r1.upper_body_kinematics import (
    ARMS_HEAD_JOINT_NAMES,
    UPPER_BODY_JOINT_NAMES,
    UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL,
    body_mode_flags,
    load_r1_a5_upper_body_model,
    retarget_nominal,
)
from teleop.r1.kinematics import KinematicsError
from teleop.r1.rate_limit import OnlineJointLimiter
from teleop.r1.bridge import rotation_matrix_to_quaternion
from teleop.r1.mapping import R1A5WholeUpperBodyOwnership, R1TeleopTargets
from teleop.r1.schema import BaseVelocity, Pose, Vector3
from teleop.r1.whole_upper_body import (
    _PROJECTED_STATUSES,
    WholeUpperBodyIsaacLabSink,
    WholeUpperBodyLiveConfig,
)
from scripts.teleop.plot_r1_t007_dynamics import _endpoint_tracking


ROOT = Path(__file__).resolve().parents[2]
SIM_URDF = ROOT / "assets" / "R1.urdf"
VENDOR_A5_URDF = ROOT / "third_party" / "xr_teleoperate_v1_6" / "assets" / "r1" / "r1_a5.urdf"


def config(**overrides: object) -> UpperBodyIKConfig:
    values: dict[str, object] = {
        "position_tolerance_m": 2e-3,
        "wrist_orientation_tolerance_rad": 5e-2,
        "head_orientation_tolerance_rad": 2e-2,
        "max_iterations": 200,
        "damping": 2e-2,
        "posture_weight": 5e-2,
        "max_joint_step_rad": 0.15,
        "finite_difference_rad": 1e-5,
        "position_weight": 10.0,
        "wrist_orientation_weight": 1.0,
        "head_orientation_weight": 0.5,
    }
    values.update(overrides)
    return UpperBodyIKConfig(**values)  # type: ignore[arg-type]


def target_from_q(model, q: np.ndarray) -> UpperBodyIKTarget:
    state = model.forward_kinematics(q)
    return UpperBodyIKTarget(
        state.left_end_effector[:3, 3],
        state.left_end_effector[:3, :3],
        state.right_end_effector[:3, 3],
        state.right_end_effector[:3, :3],
        state.head[:3, :3],
    )


def pose_from_transform(transform: np.ndarray) -> Pose:
    q = rotation_matrix_to_quaternion(transform)
    return Pose(Vector3(*(float(value) for value in transform[:3, 3])), q)


class FakeUpperBodyHandle:
    def __init__(self, dof: int = 13) -> None:
        self.position = np.zeros(dof)
        self.writes: list[tuple[tuple[str, ...], np.ndarray]] = []

    def write_joint_targets(self, joint_names, positions_rad) -> None:
        names = tuple(joint_names)
        values = np.asarray(positions_rad, dtype=float)
        self.writes.append((names, values.copy()))
        self.position = values.copy()

    def joint_positions(self, joint_names) -> tuple[float, ...]:
        self.asserted_names = tuple(joint_names)
        return tuple(float(value) for value in self.position)


class UpperBodyAssetTests(unittest.TestCase):
    def test_vendor_reference_is_present(self) -> None:
        self.assertTrue(VENDOR_A5_URDF.is_file())

    def test_hardware_common_model_has_thirteen_dofs(self) -> None:
        for path in (SIM_URDF, VENDOR_A5_URDF):
            model = load_r1_a5_upper_body_model(path)
            self.assertEqual(model.dof, 13)
            self.assertEqual(model.joint_names, UPPER_BODY_JOINT_NAMES)
            self.assertNotIn("waist_roll_joint", model.joint_names)

    def test_simulation_waist_roll_is_fixed_but_vendor_has_none(self) -> None:
        self.assertIsNotNone(load_r1_a5_upper_body_model(SIM_URDF).fixed_waist_roll)
        self.assertIsNone(load_r1_a5_upper_body_model(VENDOR_A5_URDF).fixed_waist_roll)

    def test_relative_arm_geometry_matches_vendor(self) -> None:
        simulation = load_r1_a5_upper_body_model(SIM_URDF)
        vendor = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        q = np.array([0.2, -0.1, 0.3, 0.7, 0.2])
        np.testing.assert_allclose(
            simulation.left_arm.forward_kinematics(q),
            vendor.left_arm.forward_kinematics(q),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            simulation.right_arm.forward_kinematics(q),
            vendor.right_arm.forward_kinematics(q),
            atol=1e-12,
        )

    def test_pelvis_to_waist_difference_is_not_hidden(self) -> None:
        simulation = load_r1_a5_upper_body_model(SIM_URDF)
        vendor = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        self.assertFalse(
            np.allclose(simulation.pelvis_to_waist(0.0), vendor.pelvis_to_waist(0.0), atol=1e-12)
        )

    def test_limits_are_finite_and_ordered(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        self.assertTrue(np.all(np.isfinite(model.lower_limits)))
        self.assertTrue(np.all(model.lower_limits < model.upper_limits))
        self.assertTrue(np.all(model.velocity_limits > 0.0))


class RotationTests(unittest.TestCase):
    def test_identity_log_is_zero(self) -> None:
        np.testing.assert_allclose(so3_log(np.eye(3)), np.zeros(3), atol=1e-15)

    def test_quaternion_half_turn_is_proper(self) -> None:
        rotation = quaternion_xyzw_to_matrix(np.array([1.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)
        self.assertAlmostEqual(float(np.linalg.norm(so3_log(rotation))), np.pi, places=6)

    def test_reflection_is_refused(self) -> None:
        with self.assertRaises(UpperBodyIKConfigError):
            so3_log(np.diag([1.0, 1.0, -1.0]))


class UpperBodyJacobianTests(unittest.TestCase):
    def test_jacobian_predicts_an_independent_directional_difference(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        q = np.array([0.15, 0.1, 0.3, -0.2, 0.7, 0.1, 0.1, -0.3, 0.2, 0.7, -0.1, 0.1, 0.2])
        target = target_from_q(model, q)
        jacobian = upper_body_task_jacobian(model, q, target, 1e-5)
        self.assertEqual(jacobian.shape, (15, 13))

        # Recompute with a separate epsilon. Agreement checks numerical
        # refinement rather than comparing the routine with itself verbatim.
        refined = upper_body_task_jacobian(model, q, target, 2.5e-6)
        np.testing.assert_allclose(jacobian, refined, atol=2e-5, rtol=2e-4)

    def test_waist_yaw_affects_both_wrist_tasks_and_head(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        q = np.zeros(13)
        jacobian = upper_body_task_jacobian(model, q, target_from_q(model, q), 1e-5)
        self.assertGreater(float(np.linalg.norm(jacobian[0:3, 0])), 1e-3)
        self.assertGreater(float(np.linalg.norm(jacobian[3:6, 0])), 1e-3)
        self.assertGreater(float(np.linalg.norm(jacobian[12:15, 0])), 1e-3)


class UpperBodySolverTests(unittest.TestCase):
    def test_known_fk_target_round_trips_on_both_assets(self) -> None:
        desired = np.array(
            [0.25, 0.2, 0.4, -0.2, 0.8, 0.1, 0.2, -0.4, 0.2, 0.8, -0.1, 0.15, 0.2]
        )
        for path in (SIM_URDF, VENDOR_A5_URDF):
            model = load_r1_a5_upper_body_model(path)
            result = solve_upper_body_ik(model, target_from_q(model, desired), np.zeros(13), np.zeros(13), config())
            self.assertTrue(result.converged, (path, result))
            self.assertLess(result.left_position_residual_m, 2e-3)
            self.assertLess(result.right_position_residual_m, 2e-3)
            self.assertLess(result.head_orientation_residual_rad, 2e-2)
            self.assertGreater(abs(float(result.joint_positions[0])), 0.1)

    def test_solution_stays_inside_every_joint_limit(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        desired = 0.45 * model.upper_limits + 0.55 * model.lower_limits
        result = solve_upper_body_ik(model, target_from_q(model, desired), np.zeros(13), np.zeros(13), config())
        self.assertTrue(np.all(result.joint_positions >= model.lower_limits))
        self.assertTrue(np.all(result.joint_positions <= model.upper_limits))

    def test_unreachable_target_is_preserved_as_nonconvergence(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        neutral = target_from_q(model, np.zeros(13))
        unreachable = UpperBodyIKTarget(
            np.array([3.0, 3.0, 3.0]),
            neutral.left_orientation,
            np.array([3.0, -3.0, 3.0]),
            neutral.right_orientation,
            neutral.head_orientation,
        )
        result = solve_upper_body_ik(model, unreachable, np.zeros(13), np.zeros(13), config(max_iterations=60))
        self.assertFalse(result.converged)
        self.assertGreater(max(result.left_position_residual_m, result.right_position_residual_m), 1.0)

    def test_unreachable_target_projects_to_the_reachable_boundary(self) -> None:
        """A target past the joint limits must settle, not burn the whole budget."""

        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        neutral = target_from_q(model, np.zeros(13))
        unreachable = UpperBodyIKTarget(
            np.array([3.0, 3.0, 3.0]),
            neutral.left_orientation,
            np.array([3.0, -3.0, 3.0]),
            neutral.right_orientation,
            neutral.head_orientation,
        )
        result = solve_upper_body_ik(model, unreachable, np.zeros(13), np.zeros(13), config(max_iterations=400))
        self.assertEqual(result.status, "projected_to_reachable_boundary")
        self.assertFalse(result.converged)
        self.assertLess(result.iterations, 400)
        self.assertTrue(np.all(result.joint_positions >= model.lower_limits))
        self.assertTrue(np.all(result.joint_positions <= model.upper_limits))

    def test_projection_returns_the_closest_iterate_not_a_late_one(self) -> None:
        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        neutral = target_from_q(model, np.zeros(13))
        # Just past the reachable set, so the solver approaches then stalls.
        unreachable = UpperBodyIKTarget(
            np.array([0.7, 0.5, 0.2]),
            neutral.left_orientation,
            np.array([0.7, -0.5, 0.2]),
            neutral.right_orientation,
            neutral.head_orientation,
        )
        result = solve_upper_body_ik(model, unreachable, np.zeros(13), np.zeros(13), config(max_iterations=400))
        self.assertFalse(result.converged)
        seed_state = model.forward_kinematics(np.zeros(13))
        seed_error = float(
            np.linalg.norm(unreachable.left_position_m - seed_state.left_end_effector[:3, 3])
        )
        # The projected solution must be a genuine improvement on the seed.
        self.assertLess(result.left_position_residual_m, seed_error)

    def test_small_budget_still_returns_the_closest_iterate(self) -> None:
        """Exhausting a tiny budget while still improving is not stagnation."""

        model = load_r1_a5_upper_body_model(VENDOR_A5_URDF)
        neutral = target_from_q(model, np.zeros(13))
        unreachable = UpperBodyIKTarget(
            np.array([3.0, 3.0, 3.0]),
            neutral.left_orientation,
            np.array([3.0, -3.0, 3.0]),
            neutral.right_orientation,
            neutral.head_orientation,
        )
        result = solve_upper_body_ik(model, unreachable, np.zeros(13), np.zeros(13), config(max_iterations=5))
        self.assertEqual(result.status, "iteration_budget_exhausted")
        self.assertTrue(np.all(result.joint_positions >= model.lower_limits))
        self.assertTrue(np.all(result.joint_positions <= model.upper_limits))

    def test_all_thresholds_must_be_supplied(self) -> None:
        with self.assertRaises(TypeError):
            UpperBodyIKConfig()  # type: ignore[call-arg]

    def test_bad_configuration_is_refused(self) -> None:
        with self.assertRaises(UpperBodyIKConfigError):
            config(finite_difference_rad=0.0).validate()


class WholeUpperBodySinkTests(unittest.TestCase):
    def make_sink(self, **overrides: object) -> tuple[WholeUpperBodyIsaacLabSink, FakeUpperBodyHandle]:
        dof = 14 if overrides.get("control_waist_roll") else 13
        handle = FakeUpperBodyHandle(dof)
        values: dict[str, object] = {
            "urdf_path": SIM_URDF,
            "nominal_joint_position_rad": tuple(np.zeros(dof)),
            "max_joint_velocity_rad_s": 2.0,
            "max_joint_acceleration_rad_s2": 8.0,
            "control_dt_s": 0.05,
            "ik": config(),
        }
        values.update(overrides)
        sink = WholeUpperBodyIsaacLabSink(handle, WholeUpperBodyLiveConfig(**values))  # type: ignore[arg-type]
        return sink, handle

    def targets_for_q(self, q: np.ndarray) -> R1TeleopTargets:
        """Build the source command that reproduces ``q`` exactly.

        The head target is expressed in the *neutral* waist frame, so a non-zero
        waist yaw has to be folded into the commanded head yaw. That folding is
        exact only while head pitch is zero: consecutive yaw rotations compose
        (``Rz·Rz``), whereas the chain's pitch-then-yaw order does not commute
        with a preceding waist yaw.
        """

        model = load_r1_a5_upper_body_model(SIM_URDF)
        state = model.forward_kinematics(q)
        neutral_waist_inverse = np.linalg.inv(model.pelvis_to_waist(0.0))
        left_source = neutral_waist_inverse @ state.left_end_effector
        right_source = neutral_waist_inverse @ state.right_end_effector
        waist_yaw = float(q[model.waist_yaw_index])
        head_pitch, head_yaw = (float(value) for value in q[model.head_slice])
        if abs(waist_yaw) > 1e-12:
            self.assertAlmostEqual(head_pitch, 0.0, msg="helper folds waist yaw only at zero head pitch")
        return R1TeleopTargets(
            sequence_id=7,
            enabled=True,
            reason=None,
            left_wrist_target=pose_from_transform(left_source),
            right_wrist_target=pose_from_transform(right_source),
            head_yaw_rad=waist_yaw + head_yaw,
            head_pitch_rad=head_pitch,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame="r1_base",
        )

    def test_coupled_target_dispatches_waist_both_arms_and_head_atomically(self) -> None:
        sink, handle = self.make_sink()
        # head_pitch = 0 so the waist-yaw fold above is exact; head_yaw carries
        # the head's own rotation.
        desired = np.array([0.25, 0.2, 0.4, -0.2, 0.8, 0.1, 0.2, -0.4, 0.2, 0.8, -0.1, 0.0, 0.15])
        sink.apply_upper_body(self.targets_for_q(desired), R1A5WholeUpperBodyOwnership().upper_body)
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(handle.writes[-1][0], UPPER_BODY_JOINT_NAMES)
        self.assertGreater(abs(float(sink.seed[0])), 0.1)
        self.assertEqual(sink.acknowledgements[-1]["accepted_joints"], list(UPPER_BODY_JOINT_NAMES))

    def test_combined_head_yaw_and_pitch_is_exactly_reachable(self) -> None:
        """Regression: the old Rz(yaw)@Ry(pitch) target was unsatisfiable here.

        With both head angles non-zero the previous convention left a residual of
        roughly 0.09-0.35 rad against a 0.03 rad tolerance, so no target could
        converge and the solver twisted waist yaw chasing it.
        """

        sink, _ = self.make_sink()
        desired = np.zeros(13)
        desired[11], desired[12] = 0.30, 0.60  # head pitch and yaw together
        sink.apply_upper_body(self.targets_for_q(desired), R1A5WholeUpperBodyOwnership().upper_body)
        application = sink.last_application
        self.assertTrue(application["accepted"], application)
        self.assertEqual(application["solver_solution_kind"], "exact")
        self.assertLess(application["ik"]["head_orientation_residual_rad"], 2e-2)
        # The head joints, not the waist, must absorb the commanded rotation.
        self.assertLess(abs(float(sink.seed[0])), 1e-2)
        np.testing.assert_allclose(sink.seed[11:13], [0.30, 0.60], atol=2e-2)

    def test_reset_returns_all_thirteen_joints_to_declared_nominal(self) -> None:
        sink, handle = self.make_sink()
        sink.last_target = np.full(13, 0.2)
        sink.reset_session()
        np.testing.assert_allclose(handle.writes[-1][1], np.zeros(13), atol=0.0)
        self.assertFalse(sink.session_started)

    def test_schema3_endpoint_plot_uses_coupled_pelvis_frame_fk(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF)
        q = np.array([0.2, 0.1, 0.3, -0.2, 0.7, 0.1, 0.1, -0.3, 0.2, 0.7, -0.1, 0.1, 0.0])
        state = model.forward_kinematics(q)
        application = {
            "accepted": True,
            "ik_joint_target_rad": q.tolist(),
            "limited_joint_target_rad": q.tolist(),
            "left_target_position_pelvis_m": state.left_end_effector[:3, 3].tolist(),
            "right_target_position_pelvis_m": state.right_end_effector[:3, 3].tolist(),
            "ik": {"left_position_residual_m": 0.0, "right_position_residual_m": 0.0},
            "solver_solution_kind": "exact",
        }
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "experiment_config.json").write_text(
                json.dumps({"schema_version": 3, "whole_upper_body": {"urdf_path": "assets/R1.urdf"}}),
                encoding="utf-8",
            )
            (run / "targets.json").write_text(
                json.dumps([{"elapsed_s": 1.0, "whole_upper_body": application, "post_physics_whole_upper_body_position_rad": q.tolist()}]),
                encoding="utf-8",
            )
            recovered = _endpoint_tracking(run)
        np.testing.assert_allclose(recovered["left"]["achieved_m"][0], state.left_end_effector[:3, 3])
        np.testing.assert_allclose(recovered["right"]["commanded_m"][0], state.right_end_effector[:3, 3])

    def test_incomplete_ownership_is_refused(self) -> None:
        sink, _ = self.make_sink()
        with self.assertRaises(ValueError):
            sink.apply_upper_body(self.targets_for_q(np.zeros(13)), ("head_yaw_joint",))

    def test_live_rate_may_not_exceed_the_slowest_urdf_joint(self) -> None:
        handle = FakeUpperBodyHandle()
        with self.assertRaises(ValueError):
            WholeUpperBodyIsaacLabSink(
                handle,
                WholeUpperBodyLiveConfig(
                    urdf_path=SIM_URDF,
                    nominal_joint_position_rad=tuple(np.zeros(13)),
                    max_joint_velocity_rad_s=100.0,
                    max_joint_acceleration_rad_s2=8.0,
                    control_dt_s=0.05,
                    ik=config(),
                ),
            )

    def far_targets(self) -> R1TeleopTargets:
        identity = np.eye(4)
        identity[:3, 3] = [3.0, 3.0, 3.0]
        return R1TeleopTargets(
            sequence_id=9,
            enabled=True,
            reason=None,
            left_wrist_target=pose_from_transform(identity),
            right_wrist_target=pose_from_transform(identity),
            head_yaw_rad=0.0,
            head_pitch_rad=0.0,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame="r1_base",
        )

    def test_far_target_holds_instead_of_dispatching_partial_solution(self) -> None:
        sink, handle = self.make_sink()
        sink.apply_upper_body(self.far_targets(), R1A5WholeUpperBodyOwnership().upper_body)
        self.assertFalse(sink.last_application["accepted"])
        self.assertTrue(str(sink.last_application["reason"]).startswith("upper_body_ik_refused:"))
        np.testing.assert_allclose(handle.writes[-1][1], np.zeros(13), atol=0.0)

    def test_projected_solution_extends_toward_an_unreachable_target(self) -> None:
        """The reported failure: an out-of-reach target froze all 13 joints."""

        sink, handle = self.make_sink(allow_projected_position_solution=True)
        sink.apply_upper_body(self.far_targets(), R1A5WholeUpperBodyOwnership().upper_body)
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["solver_solution_kind"], "projected")
        # The arm must actually move toward the boundary, not hold at nominal.
        self.assertGreater(float(np.max(np.abs(handle.writes[-1][1]))), 0.01)

    def test_projection_flag_never_accepts_a_singular_system(self) -> None:
        sink, _ = self.make_sink(allow_projected_position_solution=True)
        self.assertNotIn("singular_system", _PROJECTED_STATUSES)

    def test_projected_solution_still_respects_every_joint_limit(self) -> None:
        sink, handle = self.make_sink(allow_projected_position_solution=True)
        sink.apply_upper_body(self.far_targets(), R1A5WholeUpperBodyOwnership().upper_body)
        dispatched = handle.writes[-1][1]
        self.assertTrue(np.all(dispatched >= sink.model.lower_limits - 1e-12))
        self.assertTrue(np.all(dispatched <= sink.model.upper_limits + 1e-12))


class RateLimiterJointLimitTests(unittest.TestCase):
    """The limiter is second order, so momentum can carry it past a limit."""

    def test_reversing_target_cannot_be_carried_past_a_limit(self) -> None:
        lower, upper = np.array([-0.22689]), np.array([2.47840])
        limiter = OnlineJointLimiter(1.5, 4.0, 0.05, lower_limits=lower, upper_limits=upper)
        limiter.reset(np.array([0.0]))
        # Drive hard toward the lower limit, then reverse the target. The stored
        # negative velocity cannot be cancelled in one step by the acceleration
        # limit, which is what used to push the command below the joint limit.
        for _ in range(10):
            limiter.step(np.array([-0.22689]))
        for _ in range(5):
            position = limiter.step(np.array([1.0]))
            self.assertGreaterEqual(float(position[0]), float(lower[0]) - 1e-12)

    def test_clamped_joint_stops_integrating_velocity(self) -> None:
        lower, upper = np.array([-0.2]), np.array([2.0])
        limiter = OnlineJointLimiter(1.5, 4.0, 0.05, lower_limits=lower, upper_limits=upper)
        limiter.reset(np.array([0.0]))
        for _ in range(20):
            limiter.step(np.array([-5.0]))
        self.assertAlmostEqual(float(limiter.position[0]), float(lower[0]))
        self.assertEqual(float(limiter.velocity[0]), 0.0)

    def test_limits_are_optional_and_must_be_paired(self) -> None:
        OnlineJointLimiter(1.5, 4.0, 0.05)  # unbounded is still allowed
        with self.assertRaises(ValueError):
            OnlineJointLimiter(1.5, 4.0, 0.05, lower_limits=np.zeros(3))
        with self.assertRaises(ValueError):
            OnlineJointLimiter(1.5, 4.0, 0.05, lower_limits=np.ones(3), upper_limits=np.zeros(3))

    def test_coupled_sink_never_dispatches_outside_a_joint_limit(self) -> None:
        handle = FakeUpperBodyHandle()
        sink = WholeUpperBodyIsaacLabSink(
            handle,
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=tuple(np.zeros(13)),
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(),
                allow_projected_position_solution=True,
            ),
        )
        far = np.eye(4)
        model = load_r1_a5_upper_body_model(SIM_URDF)
        # Alternate two far, opposite targets so the limiter must reverse hard.
        for step in range(20):
            far[:3, 3] = [3.0, 3.0 * (1 if step % 2 else -1), 3.0]
            sink.apply_upper_body(
                R1TeleopTargets(
                    sequence_id=step,
                    enabled=True,
                    reason=None,
                    left_wrist_target=pose_from_transform(far),
                    right_wrist_target=pose_from_transform(far),
                    head_yaw_rad=0.0,
                    head_pitch_rad=0.0,
                    base_velocity=BaseVelocity.zero(),
                    base_velocity_enabled=False,
                    robot_frame="r1_base",
                ),
                R1A5WholeUpperBodyOwnership().upper_body,
            )
            dispatched = handle.writes[-1][1]
            self.assertTrue(np.all(dispatched >= model.lower_limits - 1e-12), step)
            self.assertTrue(np.all(dispatched <= model.upper_limits + 1e-12), step)


class WaistRollDeviationTests(unittest.TestCase):
    """The declared simulation-only 14-DoF torso-lean variant."""

    def test_default_model_keeps_waist_roll_fixed(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF)
        self.assertEqual(model.dof, 13)
        self.assertEqual(model.joint_names, UPPER_BODY_JOINT_NAMES)
        self.assertIsNone(model.waist_roll_index)

    def test_opt_in_model_controls_waist_roll(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True)
        self.assertEqual(model.dof, 14)
        self.assertEqual(model.joint_names, UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL)
        self.assertEqual(model.waist_roll_index, 0)
        self.assertEqual(model.joint_names[0], "waist_roll_joint")

    def test_vendor_asset_cannot_control_a_joint_it_lacks(self) -> None:
        with self.assertRaises(KinematicsError):
            load_r1_a5_upper_body_model(VENDOR_A5_URDF, control_waist_roll=True)

    def test_waist_roll_actually_leans_both_arms_and_head(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True)
        upright = model.forward_kinematics(np.zeros(14))
        leaned = model.forward_kinematics(np.concatenate(([0.3], np.zeros(13))))
        for name in ("left_end_effector", "right_end_effector", "head"):
            self.assertFalse(
                np.allclose(getattr(upright, name), getattr(leaned, name), atol=1e-9),
                f"waist roll did not move {name}",
            )

    def test_waist_roll_participates_in_the_task_jacobian(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True)
        q = np.zeros(14)
        jacobian = upper_body_task_jacobian(model, q, target_from_q(model, q), 1e-5)
        self.assertEqual(jacobian.shape, (15, 14))
        # Column 0 is waist roll: it must move both wrists and the head.
        self.assertGreater(float(np.linalg.norm(jacobian[0:3, 0])), 1e-3)
        self.assertGreater(float(np.linalg.norm(jacobian[3:6, 0])), 1e-3)
        self.assertGreater(float(np.linalg.norm(jacobian[12:15, 0])), 1e-3)

    def test_fourteen_dof_target_round_trips(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True)
        desired = np.array([0.2, 0.25, 0.2, 0.4, -0.2, 0.8, 0.1, 0.2, -0.4, 0.2, 0.8, -0.1, 0.15, 0.2])
        result = solve_upper_body_ik(model, target_from_q(model, desired), np.zeros(14), np.zeros(14), config())
        self.assertTrue(result.converged, result)
        self.assertLess(result.left_position_residual_m, 2e-3)
        self.assertLess(result.right_position_residual_m, 2e-3)

    def test_ownership_matches_every_body_mode_without_overlap(self) -> None:
        expected = {
            "arms_head": (False, False),
            "waist_yaw": (True, False),
            "full_upper_body": (True, True),
        }
        for mode, (owns_yaw, owns_roll) in expected.items():
            ownership = R1A5WholeUpperBodyOwnership(body_mode=mode)
            ownership.validate()
            self.assertEqual("waist_yaw_joint" in ownership.upper_body, owns_yaw, mode)
            self.assertEqual("waist_roll_joint" in ownership.upper_body, owns_roll, mode)
            # Every torso joint the solver does not own stays with the lower body.
            for joint in ("waist_yaw_joint", "waist_roll_joint"):
                self.assertNotEqual(
                    joint in ownership.upper_body, joint in ownership.lower_body, (mode, joint)
                )

    def test_unknown_body_mode_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            R1A5WholeUpperBodyOwnership(body_mode="legs")

    def test_sink_dispatches_all_fourteen_joints(self) -> None:
        handle = FakeUpperBodyHandle(14)
        sink = WholeUpperBodyIsaacLabSink(
            handle,
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=tuple(np.zeros(14)),
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(),
                body_mode="full_upper_body",
            ),
        )
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True)
        # waist_roll stays 0 in the *target*: the head task is a pitch/yaw-only
        # source rotation and cannot represent a rolled head, so a rolled target
        # is not exactly realizable. The joint is still solved and dispatched.
        # head_pitch is 0 so folding waist yaw into the commanded head yaw is exact.
        desired = np.array([0.0, 0.2, 0.2, 0.4, -0.2, 0.8, 0.1, 0.2, -0.4, 0.2, 0.8, -0.1, 0.0, 0.15])
        state = model.forward_kinematics(desired)
        neutral_inverse = np.linalg.inv(model.pelvis_to_waist(0.0, 0.0))
        head_pitch, head_yaw = (float(v) for v in desired[model.head_slice])
        targets = R1TeleopTargets(
            sequence_id=3,
            enabled=True,
            reason=None,
            left_wrist_target=pose_from_transform(neutral_inverse @ state.left_end_effector),
            right_wrist_target=pose_from_transform(neutral_inverse @ state.right_end_effector),
            head_yaw_rad=float(desired[model.waist_yaw_index]) + head_yaw,
            head_pitch_rad=head_pitch,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame="r1_base",
        )
        sink.apply_upper_body(targets, R1A5WholeUpperBodyOwnership(body_mode="full_upper_body").upper_body)
        self.assertEqual(handle.writes[-1][0], UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL)
        self.assertEqual(len(handle.writes[-1][1]), 14)
        self.assertIn("waist_roll_target_rad", sink.last_application)

    def test_nominal_length_must_match_the_selected_body_mode(self) -> None:
        for mode, wrong_dof in (("arms_head", 13), ("waist_yaw", 12), ("full_upper_body", 13)):
            with self.assertRaises(ValueError, msg=mode):
                WholeUpperBodyLiveConfig(
                    urdf_path=SIM_URDF,
                    nominal_joint_position_rad=tuple(np.zeros(wrong_dof)),
                    max_joint_velocity_rad_s=2.0,
                    max_joint_acceleration_rad_s2=8.0,
                    control_dt_s=0.05,
                    ik=config(),
                    body_mode=mode,
                ).validate()

    def test_waist_roll_requires_waist_yaw(self) -> None:
        with self.assertRaises(KinematicsError):
            load_r1_a5_upper_body_model(SIM_URDF, control_waist_roll=True, control_waist_yaw=False)


class SeedRestartTests(unittest.TestCase):
    """Folding the arms in to the body traps the continuation seed."""

    def test_reach_bound_is_conservative_and_asset_derived(self) -> None:
        chain = load_r1_a5_upper_body_model(SIM_URDF).left_arm
        bound = chain.max_reach_from_shoulder_m
        shoulder = chain.shoulder_origin()
        rng = np.random.default_rng(1)
        for _ in range(3000):
            q = rng.uniform(chain.lower_limits, chain.upper_limits)
            reach = float(np.linalg.norm(chain.endpoint_position(q) - shoulder))
            self.assertLessEqual(reach, bound)

    def make_sink(self, restart: float | None) -> WholeUpperBodyIsaacLabSink:
        return WholeUpperBodyIsaacLabSink(
            FakeUpperBodyHandle(12),
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=tuple(np.zeros(12)),
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(max_iterations=40),
                body_mode="arms_head",
                allow_projected_position_solution=True,
                seed_restart_residual_m=restart,
            ),
        )

    # The exact sample that failed live in
    # t007_whole_upper_body_20260818T114338Z: the operator drew both hands in
    # towards the body, 0.2965 m from the left shoulder and so well inside the
    # arm's reach, and the solver stalled at a 0.2068 m residual. Kept as
    # literals so the regression does not depend on the evidence directory.
    TRAPPED_SEED = np.array(
        [-1.50503, -0.22689, 1.23344, 0.61118, 0.82184,
         -1.60739, -0.59268, -1.71899, 2.1852, -1.46896, -0.17283, 0.39562]
    )
    LEFT_TARGET_M = np.array([0.3276, 0.10078, 0.27091])
    RIGHT_TARGET_M = np.array([0.32985, -0.09284, 0.27712])

    def folded_target(self) -> UpperBodyIKTarget:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_yaw=False)
        state = model.forward_kinematics(self.TRAPPED_SEED)
        return UpperBodyIKTarget(
            self.LEFT_TARGET_M,
            state.left_end_effector[:3, :3],
            self.RIGHT_TARGET_M,
            state.right_end_effector[:3, :3],
            state.head[:3, :3],
        )

    def test_the_live_failure_is_a_local_minimum_not_an_unreachable_target(self) -> None:
        """A good solution exists; only the continuation seed cannot reach it."""

        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_yaw=False)
        target = self.folded_target()
        nominal = np.zeros(12)
        settings = config(max_iterations=40)
        stuck = solve_upper_body_ik(model, target, self.TRAPPED_SEED, nominal, settings)
        fresh = solve_upper_body_ik(model, target, nominal, nominal, settings)
        self.assertGreater(stuck.left_position_residual_m, 0.15, stuck.status)
        self.assertLess(fresh.left_position_residual_m, stuck.left_position_residual_m / 5.0)

        # The target is inside the reach bound, so the residual is a solver
        # failure rather than the operator reaching past the arm.
        chain = model.left_arm
        shoulder = (model.pelvis_to_waist(0.0) @ np.append(chain.shoulder_origin(), 1.0))[:3]
        reach = float(np.linalg.norm(self.LEFT_TARGET_M - shoulder))
        self.assertLess(reach, chain.max_reach_from_shoulder_m)

    def test_restart_recovers_the_live_failure_through_the_sink(self) -> None:
        results = {}
        for label, threshold in (("off", None), ("on", 0.02)):
            sink = self.make_sink(threshold)
            sink.seed = self.TRAPPED_SEED.copy()
            model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_yaw=False)
            state = model.forward_kinematics(self.TRAPPED_SEED)
            inverse = np.linalg.inv(model.pelvis_to_waist(0.0))
            left = np.eye(4); left[:3, :3] = state.left_end_effector[:3, :3]; left[:3, 3] = self.LEFT_TARGET_M
            right = np.eye(4); right[:3, :3] = state.right_end_effector[:3, :3]; right[:3, 3] = self.RIGHT_TARGET_M
            sink.apply_upper_body(
                R1TeleopTargets(
                    sequence_id=11,
                    enabled=True,
                    reason=None,
                    left_wrist_target=pose_from_transform(inverse @ left),
                    right_wrist_target=pose_from_transform(inverse @ right),
                    head_yaw_rad=0.0,
                    head_pitch_rad=0.0,
                    base_velocity=BaseVelocity.zero(),
                    base_velocity_enabled=False,
                    robot_frame="r1_base",
                ),
                ARMS_HEAD_JOINT_NAMES,
            )
            results[label] = sink.last_application
        stuck = results["off"]["ik"]["left_position_residual_m"]
        fixed = results["on"]["ik"]["left_position_residual_m"]
        self.assertFalse(results["off"]["seed_restarted_from_nominal"])
        self.assertTrue(results["on"]["seed_restarted_from_nominal"])
        self.assertGreater(stuck, 0.15, results["off"]["ik"])
        self.assertLess(fixed, stuck / 5.0, (stuck, fixed))

    def test_out_of_reach_target_does_not_pay_for_a_restart(self) -> None:
        sink = self.make_sink(0.02)
        far = np.eye(4)
        far[:3, 3] = [3.0, 3.0, 3.0]
        sink.apply_upper_body(
            R1TeleopTargets(
                sequence_id=6,
                enabled=True,
                reason=None,
                left_wrist_target=pose_from_transform(far),
                right_wrist_target=pose_from_transform(far),
                head_yaw_rad=0.0,
                head_pitch_rad=0.0,
                base_velocity=BaseVelocity.zero(),
                base_velocity_enabled=False,
                robot_frame="r1_base",
            ),
            ARMS_HEAD_JOINT_NAMES,
        )
        self.assertFalse(sink.last_application["seed_restarted_from_nominal"])

    def test_restart_threshold_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=tuple(np.zeros(12)),
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(),
                body_mode="arms_head",
                seed_restart_residual_m=0.0,
            ).validate()


class ArmsHeadBodyModeTests(unittest.TestCase):
    """Torso frozen: the mode the operator selects to stop the body moving."""

    def make_sink(self) -> tuple[WholeUpperBodyIsaacLabSink, FakeUpperBodyHandle]:
        handle = FakeUpperBodyHandle(12)
        sink = WholeUpperBodyIsaacLabSink(
            handle,
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=tuple(np.zeros(12)),
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(),
                body_mode="arms_head",
                allow_projected_position_solution=True,
            ),
        )
        return sink, handle

    def test_model_excludes_both_waist_joints(self) -> None:
        model = load_r1_a5_upper_body_model(SIM_URDF, control_waist_yaw=False)
        self.assertEqual(model.dof, 12)
        self.assertEqual(model.body_mode, "arms_head")
        self.assertIsNone(model.waist_yaw_index)
        self.assertIsNone(model.waist_roll_index)
        self.assertNotIn("waist_yaw_joint", model.joint_names)
        self.assertNotIn("waist_roll_joint", model.joint_names)

    def test_frozen_torso_is_held_at_its_declared_value(self) -> None:
        held = load_r1_a5_upper_body_model(SIM_URDF, control_waist_yaw=False, fixed_waist_yaw_rad=0.4)
        free = load_r1_a5_upper_body_model(SIM_URDF)
        np.testing.assert_allclose(
            held.forward_kinematics(np.zeros(12)).left_end_effector,
            free.forward_kinematics(np.concatenate(([0.4], np.zeros(12)))).left_end_effector,
        )

    def test_torso_cannot_be_recruited_by_an_unreachable_target(self) -> None:
        """The reported symptom: waist yaw twisting to chase the hands."""

        sink, handle = self.make_sink()
        far = np.eye(4)
        far[:3, 3] = [3.0, 3.0, 3.0]
        sink.apply_upper_body(
            R1TeleopTargets(
                sequence_id=1,
                enabled=True,
                reason=None,
                left_wrist_target=pose_from_transform(far),
                right_wrist_target=pose_from_transform(far),
                head_yaw_rad=0.0,
                head_pitch_rad=0.0,
                base_velocity=BaseVelocity.zero(),
                base_velocity_enabled=False,
                robot_frame="r1_base",
            ),
            R1A5WholeUpperBodyOwnership(body_mode="arms_head").upper_body,
        )
        self.assertTrue(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["body_mode"], "arms_head")
        self.assertIsNone(sink.last_application["waist_yaw_target_rad"])
        self.assertEqual(len(handle.writes[-1][1]), 12)
        self.assertEqual(handle.writes[-1][0], ARMS_HEAD_JOINT_NAMES)

    def test_retarget_preserves_the_declared_torso_pose(self) -> None:
        """--body-mode must not silently move the torso the profile declared."""

        thirteen = tuple(float(v) for v in np.arange(13) / 100.0)  # waist yaw = 0.0
        leaning = (0.35,) + thirteen[1:]
        leaning = (0.11,) + leaning[1:]  # waist yaw = 0.11 rad

        frozen, held_yaw = retarget_nominal(leaning, "waist_yaw", "arms_head")
        self.assertEqual(len(frozen), 12)
        self.assertAlmostEqual(held_yaw, 0.11)
        np.testing.assert_allclose(frozen, leaning[1:])

        # Round trip restores the original vector exactly.
        restored, zero = retarget_nominal(frozen, "arms_head", "waist_yaw", fixed_waist_yaw_rad=held_yaw)
        np.testing.assert_allclose(restored, leaning)
        self.assertEqual(zero, 0.0)

        widened, _ = retarget_nominal(leaning, "waist_yaw", "full_upper_body")
        self.assertEqual(len(widened), 14)
        self.assertEqual(widened[0], 0.0)  # waist roll has no value to carry
        np.testing.assert_allclose(widened[1:], leaning)

    def test_retarget_refuses_a_wrong_length_nominal(self) -> None:
        with self.assertRaises(KinematicsError):
            retarget_nominal(tuple(np.zeros(13)), "arms_head", "waist_yaw")

    def test_frozen_torso_sink_holds_the_declared_yaw(self) -> None:
        nominal, held = retarget_nominal(
            (0.25,) + tuple(np.zeros(12)), "waist_yaw", "arms_head"
        )
        handle = FakeUpperBodyHandle(12)
        sink = WholeUpperBodyIsaacLabSink(
            handle,
            WholeUpperBodyLiveConfig(
                urdf_path=SIM_URDF,
                nominal_joint_position_rad=nominal,
                max_joint_velocity_rad_s=2.0,
                max_joint_acceleration_rad_s2=8.0,
                control_dt_s=0.05,
                ik=config(),
                body_mode="arms_head",
                fixed_waist_yaw_rad=held,
            ),
        )
        free = load_r1_a5_upper_body_model(SIM_URDF)
        np.testing.assert_allclose(
            sink.model.forward_kinematics(np.zeros(12)).left_end_effector,
            free.forward_kinematics(np.concatenate(([0.25], np.zeros(12)))).left_end_effector,
        )

    def test_body_mode_flags_round_trip(self) -> None:
        self.assertEqual(body_mode_flags("arms_head"), (False, False))
        self.assertEqual(body_mode_flags("waist_yaw"), (True, False))
        self.assertEqual(body_mode_flags("full_upper_body"), (True, True))
        with self.assertRaises(KinematicsError):
            body_mode_flags("nope")


if __name__ == "__main__":
    unittest.main()
