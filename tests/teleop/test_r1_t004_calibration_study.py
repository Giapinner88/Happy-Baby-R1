"""Regression checks for the deterministic T004 calibration study."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

from scripts.teleop.run_r1_t004_calibration_study import load_config, select_cases
from teleop.r1.calibration_study import evaluate_calibration_case, load_t003_waypoints
from teleop.r1.ik import ArmIKConfig
from teleop.r1.kinematics import load_arm_chain


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "experiments/r1_teleop/quest3_sim_v1/T004/config/r1_t004_calibration_study.json"
T004_ROOT = CONFIG_PATH.parents[1]


def ik_config(config: dict[str, object]) -> ArmIKConfig:
    raw = dict(config["ik"])
    return ArmIKConfig(**{key: raw[key] for key in (
        "position_tolerance_m", "roll_tolerance_rad", "max_iterations", "damping",
        "posture_weight", "max_joint_step_rad", "posture_tolerance_rad",
    )})


class T004CalibrationStudyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(CONFIG_PATH)
        self.ik = ik_config(self.config)
        self.source = ROOT / str(self.config["source_t003_run"]) / str(self.config["source_waypoints_file"])

    def evaluate(self, side: str, translation: list[float], yaw: float):
        raw = dict(self.config["ik"])
        offset = tuple(float(value) for value in self.config["frames"]["end_effector_offset_m"])
        return evaluate_calibration_case(
            source_trace=load_t003_waypoints(self.source, side),
            side=side,
            calibration_translation_m=np.asarray(translation),
            calibration_yaw_rad=yaw,
            chain=load_arm_chain(side, end_effector_offset_m=offset),
            ik_config=self.ik,
            seed_q=np.asarray(raw["seed_q_rad"]),
            nominal_q=np.asarray(raw["nominal_q_rad"]),
        )

    def test_case_manifest_has_the_declared_oat_matrix(self) -> None:
        cases = select_cases(self.config, "all")
        self.assertEqual(len(cases), 22)
        self.assertEqual(len({case["case_id"] for case in cases}), 22)
        self.assertEqual({case["side"] for case in cases}, {"left", "right"})

    def test_identity_is_exactly_mapped_and_reaches_both_mirrored_traces(self) -> None:
        tolerance = float(dict(self.config["case_validity"])["identity_mapping_tolerance_m"])
        for side in ("left", "right"):
            result = self.evaluate(side, [0.0, 0.0, 0.0], 0.0)
            self.assertTrue(result.all_converged, side)
            self.assertFalse(result.any_joint_clamped, side)
            self.assertEqual(result.base_velocity_nonzero_count, 0)
            self.assertLessEqual(result.identity_mapping_max_error_m, tolerance)

    def test_translation_changes_the_target_by_its_declared_magnitude(self) -> None:
        result = self.evaluate("right", [0.03, 0.0, 0.0], 0.0)
        self.assertTrue(all(abs(float(row["target_displacement_m"]) - 0.03) < 1e-12 for row in result.rows))

    def test_left_source_trace_is_a_y_mirror_of_the_recorded_right_trace(self) -> None:
        right = load_t003_waypoints(self.source, "right")
        left = load_t003_waypoints(self.source, "left")
        self.assertEqual(len(left), len(right))
        for left_waypoint, right_waypoint in zip(left, right):
            np.testing.assert_allclose(
                left_waypoint["source_position_m"],
                np.asarray([right_waypoint["source_position_m"][0], -right_waypoint["source_position_m"][1], right_waypoint["source_position_m"][2]]),
            )

    def test_initial_selection_and_aggregate_keep_every_declared_case(self) -> None:
        selection = json.loads((T004_ROOT / "metadata/initial_sweep_selection.json").read_text(encoding="utf-8"))
        selected = selection["case_to_run_id"]
        manifest_cases = {case["case_id"] for case in self.config["cases"]}
        self.assertEqual(set(selected), manifest_cases)
        for run_id in selected.values():
            self.assertTrue((T004_ROOT / "runs" / run_id).is_dir(), run_id)
        aggregate = json.loads(
            (T004_ROOT / "figures/t004_initial_20260811T044700Z/aggregate_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(aggregate["case_count_expected_and_selected"], len(manifest_cases))
        self.assertEqual(aggregate["nonconverged_case_count"], 0)
        self.assertEqual(aggregate["clamped_case_count"], 0)


if __name__ == "__main__":
    unittest.main()
