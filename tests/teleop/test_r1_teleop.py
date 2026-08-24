from __future__ import annotations

import math
import json
import tempfile
import unittest
from pathlib import Path
import subprocess
import sys

from teleop.r1 import (
    BaseVelocity,
    FakeIsaacLabSink,
    Pose,
    Quaternion,
    R1TeleopCommand,
    R1TeleopMapper,
    R1JointOwnership,
    SimulationOnlyAdapter,
    TeleopCalibration,
    TeleopLimits,
    Vector3,
    PolicyGateError,
    validate_isaaclab_velocity_policy,
)


def command(*, deadman: bool = True, timestamp: float = 1.0, sequence: int = 1) -> R1TeleopCommand:
    pose = Pose(Vector3(1.0, 0.0, 1.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    return R1TeleopCommand(
        sequence_id=sequence,
        timestamp_monotonic_s=timestamp,
        deadman_enabled=deadman,
        head_pose=pose,
        left_wrist_pose=pose,
        right_wrist_pose=pose,
        base_velocity=BaseVelocity(2.0, -2.0, 3.0),
    )


class R1TeleopMappingTests(unittest.TestCase):
    def test_calibration_and_velocity_clamps(self) -> None:
        mapper = R1TeleopMapper(
            TeleopCalibration(translation_m=Vector3(1.0, 2.0, 0.0), yaw_rad=math.pi / 2.0),
            TeleopLimits(0.5, allow_velocity=True, max_vx_mps=0.5, max_vy_mps=0.25, max_yaw_rate_radps=1.0),
        )
        target = mapper.map(command(), 1.1)
        self.assertTrue(target.enabled)
        self.assertAlmostEqual(target.left_wrist_target.position.x, 1.0)
        self.assertAlmostEqual(target.left_wrist_target.position.y, 3.0)
        self.assertEqual(target.base_velocity, BaseVelocity(0.5, -0.25, 1.0))

    def test_head_angles_invert_the_asset_head_chain(self) -> None:
        """Mapped head angles must aim the head down the headset forward axis.

        R1 applies `head_pitch` before `head_yaw`, so the mapper's inverse has
        to match that order. A textbook ZYX reading agrees on a pure yaw and a
        pure pitch and drifts off-axis on every mixture of the two, which shows
        up in a run as a head that never quite points where the operator looks.
        """

        from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model

        model = load_r1_a5_upper_body_model(control_waist_yaw=False)
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        for pitch_deg, yaw_deg in ((20.0, 30.0), (-15.0, -45.0), (25.0, -60.0)):
            with self.subTest(pitch=pitch_deg, yaw=yaw_deg):
                pitch_rad, yaw_rad = math.radians(pitch_deg), math.radians(yaw_deg)
                rotation = model.head_rotation(pitch_rad, yaw_rad)
                w = math.sqrt(max(0.0, 1.0 + rotation[0][0] + rotation[1][1] + rotation[2][2])) / 2.0
                orientation = Quaternion(
                    float((rotation[2][1] - rotation[1][2]) / (4.0 * w)),
                    float((rotation[0][2] - rotation[2][0]) / (4.0 * w)),
                    float((rotation[1][0] - rotation[0][1]) / (4.0 * w)),
                    float(w),
                )
                head_pose = Pose(Vector3(0.0, 0.0, 1.4), orientation)
                target = mapper.map(
                    R1TeleopCommand(
                        sequence_id=1,
                        timestamp_monotonic_s=1.0,
                        deadman_enabled=True,
                        head_pose=head_pose,
                        left_wrist_pose=head_pose,
                        right_wrist_pose=head_pose,
                        base_velocity=BaseVelocity.zero(),
                    ),
                    1.0,
                )
                self.assertAlmostEqual(target.head_pitch_rad, pitch_rad, places=9)
                self.assertAlmostEqual(target.head_yaw_rad, yaw_rad, places=9)

    def test_deadman_and_timeout_fail_closed(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5, allow_velocity=True, max_vx_mps=1.0, max_vy_mps=1.0, max_yaw_rate_radps=1.0))
        released = mapper.map(command(deadman=False), 1.1)
        stale = mapper.map(command(timestamp=1.0), 1.6)
        self.assertFalse(released.enabled)
        self.assertEqual(released.reason, "deadman_released")
        self.assertEqual(released.base_velocity, BaseVelocity.zero())
        self.assertFalse(stale.enabled)
        self.assertEqual(stale.reason, "command_timeout")

    def test_simulation_adapter_never_dispatches_disabled_command(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        sink = FakeIsaacLabSink()
        SimulationOnlyAdapter(sink).apply(mapper.map(command(deadman=False), 1.1))
        self.assertEqual(sink.events, [("hold", "deadman_released")])

    def test_velocity_disabled_never_dispatches_lower_body(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        sink = FakeIsaacLabSink()
        SimulationOnlyAdapter(sink).apply(mapper.map(command(), 1.1))
        self.assertEqual([event[0] for event in sink.events], ["upper_body"])

    def test_velocity_enabled_dispatches_lower_body(self) -> None:
        mapper = R1TeleopMapper(
            TeleopCalibration(),
            TeleopLimits(0.5, allow_velocity=True, max_vx_mps=1.0, max_vy_mps=1.0, max_yaw_rate_radps=1.0),
        )
        sink = FakeIsaacLabSink()
        SimulationOnlyAdapter(sink).apply(mapper.map(command(), 1.1))
        self.assertEqual([event[0] for event in sink.events], ["upper_body", "base_velocity"])

    def test_schema_rejects_zero_quaternion(self) -> None:
        payload = command().as_dict()
        payload["head_pose"]["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        with self.assertRaises(ValueError):
            R1TeleopCommand.from_dict(payload)

    def test_schema_rejects_non_finite_values(self) -> None:
        payload = command().as_dict()
        payload["left_wrist_pose"]["position"]["x"] = float("nan")
        with self.assertRaises(ValueError):
            R1TeleopCommand.from_dict(payload)

    def test_future_command_timestamp_fails_closed(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        target = mapper.map(command(timestamp=2.0), 1.0)
        self.assertFalse(target.enabled)
        self.assertEqual(target.reason, "command_timestamp_in_future")

    def test_velocity_policy_gate_requires_matching_evaluation(self) -> None:
        asset = Path(__file__).resolve().parents[2] / "assets" / "R1" / "R1.usd"
        import hashlib

        asset_hash = hashlib.sha256(asset.read_bytes()).hexdigest()
        signature = {"inputs": "obs-v1", "outputs": "actions-v1"}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            evaluation = root / "evaluation.json"
            evaluation.write_text(json.dumps({
                "status": "passed", "framework": "rl_lab", "task": "Unitree-R1-Velocity",
                "r1_usd_sha256": asset_hash, "observation_action_signature": signature,
            }), encoding="utf-8")
            promotion = root / "promotion_manifest.json"
            promotion.write_text(json.dumps({
                "framework": "rl_lab", "task": "Unitree-R1-Velocity", "r1_usd_sha256": asset_hash,
                "observation_action_signature": signature, "evaluation_manifest": str(evaluation),
            }), encoding="utf-8")
            self.assertEqual(
                validate_isaaclab_velocity_policy(promotion, asset)["task"], "Unitree-R1-Velocity"
            )
            evaluation.write_text(json.dumps({"status": "not_assessed"}), encoding="utf-8")
            with self.assertRaises(PolicyGateError):
                validate_isaaclab_velocity_policy(promotion, asset)

    def test_joint_ownership_rejects_overlap(self) -> None:
        with self.assertRaises(ValueError):
            R1JointOwnership(lower_body=("head_yaw_joint",), upper_body=("head_yaw_joint",)).validate()

    def test_trace_replay_is_deterministic_with_fake_sink(self) -> None:
        repo = Path(__file__).resolve().parents[2]
        trace = repo / "experiments" / "r1_teleop" / "quest3_sim_v1" / "inputs" / "example_trace.jsonl"
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "replay"
            subprocess.run(
                [
                    sys.executable,
                    "scripts/teleop/run_r1_quest3_sim.py",
                    "--input-trace",
                    str(trace),
                    "--output-dir",
                    str(output),
                ],
                cwd=repo,
                check=True,
                text=True,
                capture_output=True,
            )
            metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
            self.assertEqual(metrics["latency_s"], [0.0])
            self.assertEqual(metrics["sink_events"], ["upper_body"])
            # Provenance now uses the shared run-contract name; see experiments/README.md.
            provenance = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(provenance["schema_version"], 1)
            self.assertEqual(provenance["record_type"], "experiment_run_provenance")
            self.assertEqual(provenance["run"]["id"], "replay")
            self.assertEqual(provenance["execution"]["working_directory"], str(repo))
            self.assertEqual(provenance["inputs"]["trace"]["path"], str(trace))

    def test_runner_rejects_non_increasing_sequence(self) -> None:
        repo = Path(__file__).resolve().parents[2]
        trace = repo / "experiments" / "r1_teleop" / "quest3_sim_v1" / "inputs" / "t001_sequence_violation_trace.jsonl"
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "replay"
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/teleop/run_r1_quest3_sim.py",
                    "--input-trace",
                    str(trace),
                    "--output-dir",
                    str(output),
                ],
                cwd=repo,
                text=True,
                capture_output=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("sequence_id must increase strictly", result.stderr)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
