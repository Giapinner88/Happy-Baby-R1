"""Regression checks for the declared T006 bilateral screening protocol."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

from scripts.teleop.run_r1_t006_bilateral import build_trace, load_config, select_cases
from teleop.r1.bilateral import arm_ownership_audit, evaluate_bilateral_case
from teleop.r1.ik import ArmIKConfig
from teleop.r1.kinematics import load_arm_chain
from teleop.r1.mapping import R1JointOwnership


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "experiments/r1_teleop/quest3_sim_v1/T006/config/r1_t006_bilateral_kinematic_cases.json"


class T006BilateralTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(CONFIG_PATH)
        raw = dict(self.config["ik"])
        self.ik = ArmIKConfig(**{key: raw[key] for key in (
            "position_tolerance_m", "roll_tolerance_rad", "max_iterations", "damping",
            "posture_weight", "max_joint_step_rad", "posture_tolerance_rad",
        )})
        self.raw = raw

    def evaluate(self, case_id: str):
        case = select_cases(self.config, case_id)[0]
        trace, _ = build_trace(case, self.config)
        offset = tuple(float(value) for value in self.config["frames"]["end_effector_offset_m"])
        return evaluate_bilateral_case(
            trace=trace,
            left_chain=load_arm_chain("left", end_effector_offset_m=offset),
            right_chain=load_arm_chain("right", end_effector_offset_m=offset),
            ik_config=self.ik,
            left_seed_q=np.asarray(self.raw["left_seed_q_rad"]), right_seed_q=np.asarray(self.raw["right_seed_q_rad"]),
            left_nominal_q=np.asarray(self.raw["left_nominal_q_rad"]), right_nominal_q=np.asarray(self.raw["right_nominal_q_rad"]),
        )

    def test_manifest_is_a_small_fixed_three_case_protocol(self) -> None:
        cases = select_cases(self.config, "all")
        self.assertEqual([case["case_id"] for case in cases], ["mirror_t003", "asymmetric_t003", "inward_t002_grid"])

    def test_bilateral_ownership_is_disjoint_and_complete(self) -> None:
        audit = arm_ownership_audit(R1JointOwnership(), load_arm_chain("left"), load_arm_chain("right"))
        self.assertTrue(audit["ownership_disjoint"])
        self.assertTrue(audit["ownership_complete_for_arms"])
        self.assertEqual(audit["left_right_overlap"], [])
        self.assertEqual(audit["arm_lower_body_overlap"], [])

    def test_all_declared_cases_are_finite_and_kinematically_solvable(self) -> None:
        for case in select_cases(self.config, "all"):
            result = self.evaluate(str(case["case_id"]))
            self.assertTrue(result.all_converged, case["case_id"])
            self.assertFalse(result.any_joint_clamped, case["case_id"])
            self.assertGreater(result.minimum_endpoint_separation_m, 0.0, case["case_id"])
            self.assertTrue(result.ownership_disjoint)
            self.assertTrue(result.ownership_complete_for_arms)

    def test_inward_targets_remain_in_the_declared_t002_grid(self) -> None:
        case = select_cases(self.config, "inward_t002_grid")[0]
        trace, _ = build_trace(case, self.config)
        self.assertTrue(all(row["source"] == "T002 declared workspace grid" for row in trace))


if __name__ == "__main__":
    unittest.main()
