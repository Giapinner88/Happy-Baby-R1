"""Verification of the R1 arm forward kinematics and the DLS IK solver.

Everything here is checked against something independent of the code under test:
the Jacobian against finite differences of the forward kinematics, the forward
kinematics against the URDF geometry read separately, and the solver against
round trips through poses that are reachable by construction.

No Isaac Sim, no Quest, no GPU. Cross-checking the forward kinematics against the
simulator's own articulation is T002's job; this suite establishes internal
consistency first, so a T002 mismatch points at the simulator rather than here.
"""

from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

import numpy as np

from teleop.r1.ik import ArmIKConfig, IKConfigError, solve_arm_ik
from teleop.r1.kinematics import (
    ARM_JOINT_ORDER,
    CHAIN_ROOT_LINK,
    KinematicsError,
    R1_A5_END_EFFECTOR_OFFSET_M,
    axis_angle_to_matrix,
    load_arm_chain,
)
from teleop.r1.workspace import (
    GridSpec,
    WorkspaceError,
    grid_spacing_m,
    max_consecutive_step_m,
    serpentine_targets,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = REPO_ROOT / "assets" / "R1.urdf"


def test_config(**overrides: float) -> ArmIKConfig:
    """A solver config for tests only; T002 declares its own."""

    values = {
        "position_tolerance_m": 1e-4,
        "roll_tolerance_rad": 1e-6,
        "max_iterations": 200,
        "damping": 1e-3,
        "posture_weight": 0.05,
        "max_joint_step_rad": 0.2,
        "posture_tolerance_rad": 1e-6,
    }
    values.update(overrides)
    return ArmIKConfig(**values)  # type: ignore[arg-type]


class ChainLoadingTests(unittest.TestCase):
    def test_both_arms_load_with_five_joints(self) -> None:
        for side in ("left", "right"):
            chain = load_arm_chain(side)
            self.assertEqual(chain.dof, 5)
            self.assertEqual(
                chain.joint_names, tuple(f"{side}_{joint}_joint" for joint in ARM_JOINT_ORDER)
            )

    def test_limits_match_the_urdf(self) -> None:
        text = URDF_PATH.read_text(encoding="utf-8")
        for side in ("left", "right"):
            for joint in load_arm_chain(side).joints:
                body = re.search(
                    r'<joint\s+name="' + re.escape(joint.name) + r'"[^>]*>(.*?)</joint>', text, re.S
                ).group(1)
                limit = dict(re.findall(r'(\w+)="([^"]+)"', re.search(r"<limit([^/]*)/>", body).group(1)))
                self.assertAlmostEqual(joint.lower_rad, float(limit["lower"]), places=9)
                self.assertAlmostEqual(joint.upper_rad, float(limit["upper"]), places=9)

    def test_shoulder_origins_carry_a_non_zero_rotation(self) -> None:
        """A translation-only chain would drop the ~15 degree shoulder tilt."""

        chain = load_arm_chain("left")
        shoulder_pitch = chain.joints[0]
        self.assertFalse(np.allclose(shoulder_pitch.origin_rotation, np.eye(3)))

    def test_axes_are_unit_vectors(self) -> None:
        for side in ("left", "right"):
            for joint in load_arm_chain(side).joints:
                self.assertAlmostEqual(float(np.linalg.norm(joint.axis)), 1.0, places=12)

    def test_chain_root_is_the_waist(self) -> None:
        text = URDF_PATH.read_text(encoding="utf-8")
        parent = re.search(
            r'<joint\s+name="left_shoulder_pitch_joint"[^>]*>.*?<parent link="([^"]+)"', text, re.S
        ).group(1)
        self.assertEqual(parent, CHAIN_ROOT_LINK)

    def test_unknown_side_is_refused(self) -> None:
        with self.assertRaises(KinematicsError):
            load_arm_chain("middle")


class RotationTests(unittest.TestCase):
    def test_axis_angle_is_orthonormal_with_unit_determinant(self) -> None:
        axis = np.array([1.0, 2.0, -0.5])
        axis /= np.linalg.norm(axis)
        rotation = axis_angle_to_matrix(axis, 0.7)
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)

    def test_rotation_about_an_axis_leaves_that_axis_fixed(self) -> None:
        axis = np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(axis_angle_to_matrix(axis, 1.3) @ axis, axis, atol=1e-12)


class ForwardKinematicsTests(unittest.TestCase):
    def test_zero_pose_endpoint_matches_summed_urdf_origins(self) -> None:
        """Independent check: with all joints at zero, FK is the fixed chain."""

        chain = load_arm_chain("left")
        expected = np.eye(4)
        for joint in chain.joints:
            fixed = np.eye(4)
            fixed[:3, :3] = joint.origin_rotation
            fixed[:3, 3] = joint.origin_translation
            expected = expected @ fixed
        expected[:3, 3] += expected[:3, :3] @ R1_A5_END_EFFECTOR_OFFSET_M
        np.testing.assert_allclose(chain.forward_kinematics(np.zeros(5)), expected, atol=1e-12)

    def test_endpoint_pose_is_a_valid_rigid_transform(self) -> None:
        chain = load_arm_chain("right")
        pose = chain.forward_kinematics(np.array([0.3, -0.4, 0.2, 0.5, -0.1]))
        rotation = pose[:3, :3]
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)
        np.testing.assert_allclose(pose[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-15)

    def test_arms_are_mirrored_in_y_at_the_zero_pose(self) -> None:
        left = load_arm_chain("left").endpoint_position(np.zeros(5))
        right = load_arm_chain("right").endpoint_position(np.zeros(5))
        self.assertAlmostEqual(left[0], right[0], places=6)
        self.assertAlmostEqual(left[1], -right[1], places=6)
        self.assertAlmostEqual(left[2], right[2], places=6)

    def test_wrist_roll_does_not_move_the_endpoint_position(self) -> None:
        """Roll is the last joint, so it rotates the endpoint without translating it."""

        chain = load_arm_chain("left")
        base = np.array([0.2, 0.3, -0.1, 0.4, 0.0])
        rolled = base.copy()
        rolled[-1] = 1.1
        np.testing.assert_allclose(
            chain.endpoint_position(base), chain.endpoint_position(rolled), atol=1e-12
        )

    def test_wrong_joint_count_is_refused(self) -> None:
        with self.assertRaises(KinematicsError):
            load_arm_chain("left").forward_kinematics(np.zeros(4))


class JacobianTests(unittest.TestCase):
    """The Jacobian is the solver's only model of the chain; verify it directly."""

    def test_jacobian_matches_finite_differences(self) -> None:
        for side in ("left", "right"):
            chain = load_arm_chain(side)
            for q in (
                np.zeros(5),
                np.array([0.3, 0.2, -0.4, 0.6, 0.1]),
                np.array([-0.8, 0.1, 0.9, 1.2, -0.7]),
            ):
                q = chain.clamp(q)
                analytic = chain.position_jacobian(q)
                numeric = np.zeros_like(analytic)
                step = 1e-7
                for index in range(chain.dof):
                    forward, backward = q.copy(), q.copy()
                    forward[index] += step
                    backward[index] -= step
                    numeric[:, index] = (
                        chain.endpoint_position(forward) - chain.endpoint_position(backward)
                    ) / (2.0 * step)
                np.testing.assert_allclose(analytic, numeric, atol=1e-6, err_msg=f"{side} at {q}")

    def test_wrist_roll_column_is_zero(self) -> None:
        """Rolling the last joint cannot translate the endpoint."""

        chain = load_arm_chain("left")
        jacobian = chain.position_jacobian(np.array([0.2, 0.3, -0.1, 0.4, 0.5]))
        np.testing.assert_allclose(jacobian[:, -1], np.zeros(3), atol=1e-12)


class IKSolverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.chain = load_arm_chain("left")
        self.nominal = np.zeros(5)

    def test_reachable_target_round_trips(self) -> None:
        """A pose generated by FK is reachable by construction."""

        truth = self.chain.clamp(np.array([0.4, 0.3, -0.2, 0.7, 0.35]))
        target = self.chain.endpoint_position(truth)
        result = solve_arm_ik(
            self.chain, target, truth[-1], np.zeros(5), self.nominal, test_config()
        )
        self.assertTrue(result.converged, result.status)
        self.assertLess(result.position_residual_m, 1e-4)
        np.testing.assert_allclose(self.chain.endpoint_position(result.joint_positions), target, atol=1e-4)

    def test_roll_is_imposed_exactly(self) -> None:
        truth = self.chain.clamp(np.array([0.2, 0.4, 0.1, 0.9, -0.6]))
        target = self.chain.endpoint_position(truth)
        result = solve_arm_ik(
            self.chain, target, -0.6, np.zeros(5), self.nominal,
            test_config(max_iterations=400),
        )
        self.assertTrue(result.converged, result.status)
        self.assertAlmostEqual(self.chain.wrist_roll(result.joint_positions), -0.6, places=12)

    def test_solution_stays_inside_joint_limits(self) -> None:
        truth = self.chain.clamp(np.array([1.5, 2.0, 1.5, 2.0, 1.5]))
        target = self.chain.endpoint_position(truth)
        result = solve_arm_ik(self.chain, target, truth[-1], np.zeros(5), self.nominal, test_config())
        self.assertTrue(np.all(result.joint_positions >= self.chain.lower_limits - 1e-12))
        self.assertTrue(np.all(result.joint_positions <= self.chain.upper_limits + 1e-12))

    def test_unreachable_target_does_not_converge(self) -> None:
        """Far outside the workspace is a valid scientific result, not an error."""

        result = solve_arm_ik(
            self.chain, np.array([5.0, 5.0, 5.0]), 0.0, np.zeros(5), self.nominal, test_config()
        )
        self.assertFalse(result.converged)
        self.assertGreater(result.position_residual_m, 1.0)
        self.assertIn(result.status, ("iteration_budget_exhausted", "tolerance_not_met", "projected_to_reachable_boundary"))

    def test_roll_outside_the_joint_range_is_reported_not_silently_clamped(self) -> None:
        truth = self.chain.clamp(np.array([0.3, 0.3, 0.0, 0.8, 0.0]))
        target = self.chain.endpoint_position(truth)
        beyond = self.chain.upper_limits[-1] + 0.5
        result = solve_arm_ik(self.chain, target, beyond, np.zeros(5), self.nominal, test_config())
        self.assertFalse(result.converged)
        self.assertEqual(result.status, "roll_target_clamped_to_limit")
        self.assertGreater(result.roll_residual_rad, 0.4)

    def test_posture_bias_makes_the_redundant_solution_repeatable(self) -> None:
        """Same target, different seeds, same posture: solutions must agree."""

        truth = self.chain.clamp(np.array([0.35, 0.5, -0.3, 0.9, 0.2]))
        target = self.chain.endpoint_position(truth)
        config = test_config(posture_weight=0.2, max_iterations=2000)
        first = solve_arm_ik(self.chain, target, truth[-1], np.zeros(5), self.nominal, config)
        second = solve_arm_ik(
            self.chain, target, truth[-1], np.array([0.1, 0.2, 0.3, 0.4, 0.2]), self.nominal, config
        )
        self.assertTrue(first.converged and second.converged)
        np.testing.assert_allclose(first.joint_positions, second.joint_positions, atol=5e-3)

    def test_a_distant_seed_can_reach_a_different_elbow_branch(self) -> None:
        """Repeatability is per basin of attraction, not global.

        The posture bias makes the redundant degree of freedom a function of the
        target for seeds in the same basin, but the chain has more than one
        elbow configuration reaching a given endpoint. A seed far from nominal
        converges to the other branch: same endpoint, different posture. Teleop
        avoids this by seeding with the previous solution, so successive targets
        stay in one basin; a reset or a large jump can switch branch, and T002
        must record that rather than treat it as noise.
        """

        truth = self.chain.clamp(np.array([0.35, 0.5, -0.3, 0.9, 0.2]))
        target = self.chain.endpoint_position(truth)
        config = test_config(posture_weight=0.2, max_iterations=2000)
        near = solve_arm_ik(self.chain, target, 0.2, np.zeros(5), self.nominal, config)
        far = solve_arm_ik(
            self.chain, target, 0.2, np.array([-0.5, 0.8, -0.9, 1.5, 0.2]), self.nominal, config
        )
        self.assertTrue(near.converged and far.converged)
        # Both reach the commanded endpoint.
        for result in (near, far):
            np.testing.assert_allclose(
                self.chain.endpoint_position(result.joint_positions), target, atol=1e-4
            )
        # But not with the same posture.
        self.assertGreater(
            float(np.max(np.abs(near.joint_positions - far.joint_positions))), 0.5
        )

    def test_continuous_seeding_keeps_one_branch(self) -> None:
        """Seeding with the previous solution is what keeps a trace comparable."""

        config = test_config(posture_weight=0.2, max_iterations=2000)
        q = np.zeros(5)
        previous_solution = None
        for step in range(6):
            truth = self.chain.clamp(np.array([0.30 + 0.02 * step, 0.5, -0.3, 0.9, 0.2]))
            target = self.chain.endpoint_position(truth)
            result = solve_arm_ik(self.chain, target, 0.2, q, self.nominal, config)
            self.assertTrue(result.converged, result.status)
            if previous_solution is not None:
                self.assertLess(
                    float(np.max(np.abs(result.joint_positions - previous_solution))),
                    0.3,
                    "a small target change produced a branch switch",
                )
            previous_solution = result.joint_positions
            q = result.joint_positions

    def test_limit_margin_is_reported(self) -> None:
        truth = self.chain.clamp(np.array([0.2, 0.3, 0.0, 0.6, 0.0]))
        target = self.chain.endpoint_position(truth)
        result = solve_arm_ik(self.chain, target, 0.0, np.zeros(5), self.nominal, test_config())
        self.assertGreaterEqual(result.limit_margin_rad, 0.0)
        self.assertIsInstance(result.hit_a_limit, bool)

    def test_step_clamp_bounds_each_iteration(self) -> None:
        truth = self.chain.clamp(np.array([1.0, 1.5, 1.0, 1.8, 0.0]))
        target = self.chain.endpoint_position(truth)
        config = test_config(max_joint_step_rad=0.01, max_iterations=3)
        result = solve_arm_ik(self.chain, target, 0.0, np.zeros(5), self.nominal, config)
        self.assertLessEqual(float(np.max(np.abs(result.joint_positions[:4]))), 3 * 0.01 + 1e-9)


class WorkspaceGridTests(unittest.TestCase):
    """The sweep order is what keeps the solver inside one elbow branch."""

    def spec(self, **overrides) -> GridSpec:
        values = {
            "x_range_m": (0.05, 0.30),
            "y_range_m": (0.05, 0.40),
            "z_range_m": (-0.05, 0.35),
            "counts": (4, 4, 4),
            "wrist_roll_rad": 0.0,
        }
        values.update(overrides)
        return GridSpec(**values)  # type: ignore[arg-type]

    def test_grid_produces_the_declared_number_of_targets(self) -> None:
        targets = serpentine_targets(self.spec())
        self.assertEqual(len(targets), 64)
        self.assertEqual(self.spec().target_count, 64)

    def test_every_grid_cell_appears_exactly_once(self) -> None:
        cells = [target.grid_index for target in serpentine_targets(self.spec())]
        self.assertEqual(len(set(cells)), 64)

    def test_consecutive_targets_never_jump_more_than_one_cell(self) -> None:
        spec = self.spec()
        targets = serpentine_targets(spec)
        largest_spacing = max(grid_spacing_m(spec))
        self.assertLessEqual(max_consecutive_step_m(targets), largest_spacing + 1e-12)

    def test_targets_stay_inside_the_declared_bounds(self) -> None:
        spec = self.spec()
        for target in serpentine_targets(spec):
            x, y, z = target.position_m
            self.assertGreaterEqual(x, spec.x_range_m[0] - 1e-12)
            self.assertLessEqual(x, spec.x_range_m[1] + 1e-12)
            self.assertGreaterEqual(y, spec.y_range_m[0] - 1e-12)
            self.assertLessEqual(y, spec.y_range_m[1] + 1e-12)
            self.assertGreaterEqual(z, spec.z_range_m[0] - 1e-12)
            self.assertLessEqual(z, spec.z_range_m[1] + 1e-12)

    def test_single_sample_axis_is_allowed(self) -> None:
        targets = serpentine_targets(self.spec(counts=(1, 1, 3)))
        self.assertEqual(len(targets), 3)
        self.assertEqual(grid_spacing_m(self.spec(counts=(1, 1, 3)))[0], 0.0)

    def test_inverted_or_empty_specification_is_refused(self) -> None:
        with self.assertRaises(WorkspaceError):
            self.spec(x_range_m=(0.30, 0.05)).validate()
        with self.assertRaises(WorkspaceError):
            self.spec(counts=(0, 4, 4)).validate()

    def configured_spec(self, side: str) -> GridSpec:
        config = json.loads((REPO_ROOT / "experiments/r1_teleop/quest3_sim_v1/T002/config/r1_t002_workspace.json").read_text(encoding="utf-8"))
        declared = config["workspace_grid"][side]
        spec = GridSpec(
            x_range_m=tuple(declared["x_range_m"]),
            y_range_m=tuple(declared["y_range_m"]),
            z_range_m=tuple(declared["z_range_m"]),
            counts=tuple(declared["counts"]),
            wrist_roll_rad=float(declared["wrist_roll_rad"]),
        )
        spec.validate()
        return spec

    @staticmethod
    def configured_chain(side: str):
        config = json.loads((REPO_ROOT / "experiments/r1_teleop/quest3_sim_v1/T002/config/r1_t002_workspace.json").read_text(encoding="utf-8"))
        offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
        return load_arm_chain(side, end_effector_offset_m=offset)

    @staticmethod
    def sampled_reach_m(side: str, samples: int = 4000) -> float:
        """Largest endpoint distance from the waist over random valid postures."""

        chain = WorkspaceGridTests.configured_chain(side)
        rng = np.random.default_rng(0)
        q = rng.uniform(chain.lower_limits, chain.upper_limits, size=(samples, chain.dof))
        return float(max(np.linalg.norm(chain.endpoint_position(row)) for row in q))

    def test_configured_grid_straddles_the_measured_reach_boundary(self) -> None:
        """The declared grid must probe the boundary, which is its stated purpose.

        Most targets must be plausibly reachable for the sweep to measure a
        workspace, and at least one must lie beyond the measured reach so the
        run records an unreachable result rather than only successes.
        """

        for side in ("left", "right"):
            spec = self.configured_spec(side)
            reach = self.sampled_reach_m(side)
            distances = [float(np.linalg.norm(t.position_m)) for t in serpentine_targets(spec)]
            beyond = [d for d in distances if d > reach]
            within = [d for d in distances if d <= reach]
            self.assertGreater(len(beyond), 0, f"{side}: grid never probes past reach {reach:.3f} m")
            self.assertGreater(len(within), len(beyond), f"{side}: grid is mostly out of reach")

    def test_configured_grid_is_not_absurdly_outside_the_asset(self) -> None:
        """A target far beyond reach wastes the sweep on foregone conclusions."""

        for side in ("left", "right"):
            reach = self.sampled_reach_m(side)
            for target in serpentine_targets(self.configured_spec(side)):
                self.assertLess(float(np.linalg.norm(target.position_m)), 1.5 * reach, side)


class IKConfigTests(unittest.TestCase):
    def test_every_threshold_must_be_supplied(self) -> None:
        """No numeric default: the experiment declares them, not this module."""

        with self.assertRaises(TypeError):
            ArmIKConfig()  # type: ignore[call-arg]

    def test_non_positive_values_are_refused(self) -> None:
        for field in (
            "position_tolerance_m",
            "roll_tolerance_rad",
            "damping",
            "max_joint_step_rad",
            "posture_tolerance_rad",
        ):
            with self.assertRaises(IKConfigError, msg=field):
                test_config(**{field: 0.0}).validate()

    def test_zero_damping_is_refused(self) -> None:
        """An undamped pseudo-inverse is unbounded at a singularity."""

        with self.assertRaises(IKConfigError):
            test_config(damping=0.0).validate()

    def test_negative_posture_weight_is_refused(self) -> None:
        with self.assertRaises(IKConfigError):
            test_config(posture_weight=-0.1).validate()

    def test_bad_target_shape_is_refused(self) -> None:
        with self.assertRaises(IKConfigError):
            solve_arm_ik(
                load_arm_chain("left"), np.zeros(2), 0.0, np.zeros(5), np.zeros(5), test_config()
            )


if __name__ == "__main__":
    unittest.main()
