"""The workstation stage that feeds the robot sidecar on the vendor path.

The robot side receives joint angles and nothing else, so every check that a
number is fit to send has to happen here. Three of them are pinned below
because each has a silent failure mode:

* A vector under the wrong joint names must be refused, never reordered.
  Correct numbers on the wrong joints is the one output that cannot be
  recovered from once a robot has acted on it.
* The wire format the sidecar parses is `positions_rad`, not the
  `joint_position_rad` the solver emits. The two streams meet here and nowhere
  else, so a rename on either side has to fail a test rather than a run.
* The vendor ships no rate limiter, on purpose. Simulation records the joint
  speed it produces; hardware has to bound it instead.
"""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)


def load_module(relative: str, name: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HardwareTargetProducerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.producer = load_module(
            "scripts/teleop/run_r1_quest3_hardware_targets.py", "r1_hardware_targets"
        )
        cls.sidecar = load_module(
            "hardware/teleop/src/teleop/hardware/high_level_sidecar.py", "hb_high_level_sidecar"
        )

    def test_joint_order_matches_the_robot_receiver(self):
        self.assertEqual(self.producer.JOINT_NAMES, JOINT_NAMES)
        self.assertEqual(self.sidecar.JOINT_NAMES, JOINT_NAMES)
        self.assertEqual(len(self.sidecar.MOTOR_INDICES), len(JOINT_NAMES))

    def test_a_solved_vector_is_read_back_unchanged(self):
        values = [0.1 * index for index in range(12)]
        payload = {"upstream_joint_names": list(JOINT_NAMES), "upstream_joint_position_rad": values}
        np.testing.assert_allclose(self.producer.upstream_solution(payload), values)

    def test_a_line_without_a_solution_is_not_an_error(self):
        self.assertIsNone(self.producer.upstream_solution({"sequence_id": 4}))

    def test_wrong_joint_names_are_refused_rather_than_reordered(self):
        swapped = list(JOINT_NAMES)
        swapped[0], swapped[5] = swapped[5], swapped[0]
        payload = {
            "upstream_joint_names": swapped,
            "upstream_joint_position_rad": [0.0] * 12,
        }
        with self.assertRaises(SystemExit):
            self.producer.upstream_solution(payload)

    def test_a_short_or_non_finite_vector_is_refused(self):
        for values in ([0.0] * 11, [float("nan")] + [0.0] * 11):
            with self.subTest(values=values[:1]):
                with self.assertRaises(SystemExit):
                    self.producer.upstream_solution(
                        {"upstream_joint_names": list(JOINT_NAMES), "upstream_joint_position_rad": values}
                    )

    def test_the_emitted_payload_is_what_the_sidecar_parses(self):
        payload = {
            "schema_version": 1,
            "sequence_id": 7,
            "sent_monotonic_s": 1.0,
            "joint_names": JOINT_NAMES,
            "positions_rad": [0.01 * index for index in range(12)],
            "solution_kind": "upstream_xr_teleoperate_R1_A5_ArmIK",
        }
        parsed = self.sidecar.parse_target(json.dumps(payload), previous_sequence=6)
        self.assertIsNotNone(parsed)
        sequence, positions = parsed
        self.assertEqual(sequence, 7)
        self.assertEqual(len(positions), 12)
        self.sidecar.encode_target(sequence, positions)

    def test_the_hardware_ceilings_bound_a_vendor_sized_step(self):
        """A step the vendor path really produces must leave here bounded.

        The live simulation run reached 5.94 rad/s. Asking for a step that size
        and checking what comes out is the whole point of putting a limiter on
        this path.
        """

        from teleop.r1.rate_limit import OnlineJointLimiter
        from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model

        model = load_r1_a5_upper_body_model(control_waist_yaw=False)
        limiter = OnlineJointLimiter(
            max_velocity_rad_s=0.5,
            max_acceleration_rad_s2=1.0,
            dt_s=1.0 / 10.0,
            lower_limits=model.lower_limits,
            upper_limits=model.upper_limits,
        )
        start = np.zeros(12)
        limiter.step(start)
        previous = start
        for _ in range(5):
            current = limiter.step(model.clamp(previous + 0.594))  # 5.94 rad/s at 10 Hz
            speed = np.abs(current - previous) / 0.1
            self.assertLessEqual(float(speed.max()), 0.5 + 1e-9)
            previous = current


if __name__ == "__main__":
    unittest.main()
