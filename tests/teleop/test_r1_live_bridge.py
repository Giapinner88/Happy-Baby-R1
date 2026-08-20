"""Unit tests for the T001 live bridge and the head-only IsaacLab sink.

These tests use no Quest, no Isaac Sim, and no hardware. The simulator handle is
a recording stub, so the sink's ownership and fail-closed contract is checked
independently of whether Isaac Sim is installed.
"""

from __future__ import annotations

import math
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from teleop.r1 import (
    HEAD_JOINT_NAMES,
    BridgeConfig,
    BridgeError,
    HeadOnlyIsaacLabSink,
    Pose,
    Quaternion,
    QuestCommandBridge,
    QuestTransportSample,
    R1JointOwnership,
    R1TeleopMapper,
    SimulationOnlyAdapter,
    TeleopCalibration,
    TeleopLimits,
    Vector3,
    VelocityDispatchError,
    pose_from_matrix,
    rotation_matrix_to_quaternion,
)
from teleop.r1.mapping import R1TeleopTargets
from teleop.r1.schema import BaseVelocity
from scripts.teleop.quest_bridge import _deadman_pressed, _trigger_state
from scripts.teleop.run_r1_quest3_live import _head_tracking_error, build_parser


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def yaw_matrix(yaw_rad: float, translation: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> list[list[float]]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [
        [c, -s, 0.0, translation[0]],
        [s, c, 0.0, translation[1]],
        [0.0, 0.0, 1.0, translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def sample(**overrides: object) -> QuestTransportSample:
    values: dict[str, object] = {
        "motion_data_ready": True,
        "head_pose_matrix": IDENTITY,
        "left_wrist_pose_matrix": IDENTITY,
        "right_wrist_pose_matrix": IDENTITY,
        "deadman_pressed": True,
    }
    values.update(overrides)
    return QuestTransportSample(**values)  # type: ignore[arg-type]


class RecordingHandle:
    """Stub simulator handle; it records writes instead of stepping physics."""

    def __init__(self) -> None:
        self.writes: list[tuple[float, float]] = []
        self.position = (0.0, 0.0)

    def write_head_targets(self, yaw_rad: float, pitch_rad: float) -> None:
        self.writes.append((yaw_rad, pitch_rad))
        self.position = (yaw_rad, pitch_rad)

    def head_joint_positions(self) -> tuple[float, float]:
        return self.position


class PoseConversionTests(unittest.TestCase):
    def test_identity_matrix_maps_to_identity_pose(self) -> None:
        pose = pose_from_matrix(IDENTITY)
        self.assertEqual(pose.position, Vector3(0.0, 0.0, 0.0))
        self.assertAlmostEqual(pose.orientation.w, 1.0)
        self.assertAlmostEqual(pose.orientation.x, 0.0)
        self.assertAlmostEqual(pose.orientation.y, 0.0)
        self.assertAlmostEqual(pose.orientation.z, 0.0)

    def test_translation_is_read_from_the_last_column(self) -> None:
        pose = pose_from_matrix(yaw_matrix(0.0, (1.5, -2.5, 0.25)))
        self.assertEqual(pose.position, Vector3(1.5, -2.5, 0.25))

    def test_yaw_rotation_round_trips_through_the_mapper(self) -> None:
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        bridge = QuestCommandBridge()
        command = bridge.build(sample(head_pose_matrix=yaw_matrix(math.radians(30.0))), 1.0)
        self.assertIsNotNone(command)
        target = mapper.map(command, 1.0)
        self.assertTrue(target.enabled)
        self.assertAlmostEqual(target.head_yaw_rad, math.radians(30.0), places=9)

    def test_quaternion_is_stable_for_a_half_turn(self) -> None:
        # A pi yaw drives the trace-based branch of Shepperd's method to zero,
        # which is exactly the case a naive trace formula divides by.
        quaternion = rotation_matrix_to_quaternion(yaw_matrix(math.pi))
        self.assertAlmostEqual(abs(quaternion.z), 1.0, places=9)
        self.assertAlmostEqual(quaternion.w, 0.0, places=9)

    def test_non_finite_matrix_is_rejected(self) -> None:
        broken = [row[:] for row in IDENTITY]
        broken[0][3] = float("nan")
        with self.assertRaises(BridgeError):
            pose_from_matrix(broken)


class BridgeConnectionTests(unittest.TestCase):
    def test_sequence_ids_increase_strictly(self) -> None:
        # Poses must differ between samples: a repeated pose is a frozen headset.
        bridge = QuestCommandBridge()
        ids = [
            bridge.build(sample(head_pose_matrix=yaw_matrix(0.01 * index)), 1.0 + index).sequence_id
            for index in range(5)
        ]
        self.assertEqual(ids, [0, 1, 2, 3, 4])

    def test_sample_without_motion_data_yields_no_command(self) -> None:
        bridge = QuestCommandBridge()
        self.assertIsNone(bridge.build(sample(motion_data_ready=False), 1.0))
        self.assertEqual(bridge.state.dropped_sample_count, 1)
        self.assertFalse(bridge.state.connected)

    def test_disconnect_and_reconnect_are_recorded(self) -> None:
        bridge = QuestCommandBridge()
        bridge.build(sample(head_pose_matrix=yaw_matrix(0.1)), 1.0)
        bridge.build(sample(head_pose_matrix=yaw_matrix(0.1), motion_data_ready=False), 2.0)
        bridge.build(sample(head_pose_matrix=yaw_matrix(0.2)), 3.0)
        self.assertEqual(bridge.state.connect_count, 2)
        self.assertEqual(bridge.state.disconnect_count, 1)
        self.assertEqual(
            [transition["event"] for transition in bridge.state.transitions],
            ["connected", "disconnected", "connected"],
        )

    def test_dropped_samples_do_not_consume_sequence_ids(self) -> None:
        bridge = QuestCommandBridge()
        first = bridge.build(sample(head_pose_matrix=yaw_matrix(0.1)), 1.0)
        bridge.build(sample(head_pose_matrix=yaw_matrix(0.1), motion_data_ready=False), 2.0)
        second = bridge.build(sample(head_pose_matrix=yaw_matrix(0.2)), 3.0)
        self.assertEqual((first.sequence_id, second.sequence_id), (0, 1))

    def test_non_rotation_matrix_is_rejected_without_a_command(self) -> None:
        bridge = QuestCommandBridge()
        skewed = [row[:] for row in IDENTITY]
        skewed[0][0] = 2.0
        self.assertIsNone(bridge.build(sample(head_pose_matrix=skewed), 1.0))
        self.assertEqual(bridge.state.rejected_sample_count, 1)
        self.assertEqual(bridge.state.transitions[-1]["event"], "rejected")

    def test_reflection_matrix_is_rejected_without_a_command(self) -> None:
        bridge = QuestCommandBridge()
        reflection = [row[:] for row in IDENTITY]
        reflection[2][2] = -1.0
        self.assertIsNone(bridge.build(sample(head_pose_matrix=reflection), 1.0))
        self.assertEqual(bridge.state.rejected_sample_count, 1)
        self.assertIn("determinant", str(bridge.state.transitions[-1]["detail"]))

    def test_deadman_release_is_carried_into_the_command(self) -> None:
        bridge = QuestCommandBridge()
        self.assertFalse(bridge.build(sample(deadman_pressed=False), 1.0).deadman_enabled)

    def test_reset_request_survives_normalization_even_when_the_pose_is_static(self) -> None:
        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))
        frozen = sample(head_pose_matrix=yaw_matrix(0.1))
        self.assertIsNotNone(bridge.build(frozen, 1.0))
        command = bridge.build(sample(head_pose_matrix=yaw_matrix(0.1), reset_requested=True), 2.0)
        self.assertIsNotNone(command)
        self.assertTrue(command.reset_requested)

    def test_base_velocity_is_always_zero(self) -> None:
        bridge = QuestCommandBridge()
        self.assertEqual(bridge.build(sample(), 1.0).base_velocity, BaseVelocity.zero())

    def test_non_zero_velocity_source_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            QuestCommandBridge(BridgeConfig(base_velocity_source="thumbstick"))


class QuestControllerTriggerTests(unittest.TestCase):
    def telemetry(self, **overrides: object) -> SimpleNamespace:
        values: dict[str, object] = {
            "left_ctrl_trigger": False,
            "left_ctrl_triggerValue": 10.0,
            "right_ctrl_trigger": False,
            "right_ctrl_triggerValue": 10.0,
        }
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_digital_trigger_still_enables_at_released_analog_value(self) -> None:
        right = _trigger_state(self.telemetry(right_ctrl_trigger=True), "right", 5.0)
        self.assertTrue(right.digital_pressed)
        self.assertFalse(right.analog_pressed)
        self.assertTrue(right.effective_pressed)

    def test_analog_trigger_falls_back_when_webxr_boolean_is_false(self) -> None:
        right = _trigger_state(self.telemetry(right_ctrl_triggerValue=4.5), "right", 5.0)
        self.assertFalse(right.digital_pressed)
        self.assertTrue(right.analog_valid)
        self.assertTrue(right.analog_pressed)
        self.assertTrue(right.effective_pressed)

    def test_released_analog_value_does_not_enable(self) -> None:
        right = _trigger_state(self.telemetry(right_ctrl_triggerValue=10.0), "right", 5.0)
        self.assertFalse(right.effective_pressed)

    def test_out_of_range_or_non_finite_analog_value_fails_closed(self) -> None:
        for value in (-0.1, 10.1, float("nan"), float("inf"), "not-a-number"):
            with self.subTest(value=value):
                right = _trigger_state(self.telemetry(right_ctrl_triggerValue=value), "right", 5.0)
                self.assertFalse(right.analog_valid)
                self.assertFalse(right.effective_pressed)

    def test_missing_analog_value_falls_back_to_digital_only(self) -> None:
        telemetry = SimpleNamespace(right_ctrl_trigger=False)
        self.assertFalse(_trigger_state(telemetry, "right", 5.0).effective_pressed)
        telemetry.right_ctrl_trigger = True
        self.assertTrue(_trigger_state(telemetry, "right", 5.0).effective_pressed)

    def test_deadman_source_uses_effective_trigger_state(self) -> None:
        telemetry = self.telemetry(left_ctrl_triggerValue=4.0, right_ctrl_triggerValue=10.0)
        left = _trigger_state(telemetry, "left", 5.0)
        right = _trigger_state(telemetry, "right", 5.0)
        self.assertFalse(_deadman_pressed(left, right, "right_trigger"))
        self.assertTrue(_deadman_pressed(left, right, "left_trigger"))
        self.assertTrue(_deadman_pressed(left, right, "either_trigger"))


class FrozenPoseDisconnectTests(unittest.TestCase):
    """A removed headset keeps `motion_data_ready` true and freezes the pose.

    The vendor flag latches on the first motion event and is never cleared, so
    it cannot report a lost session. Run `t001_b_20260802T111348Z` recorded the
    consequence: the head pose was bit-identical for 5.9 s while the bridge
    emitted 177 commands with fresh timestamps, which also kept the mapper's
    `command_timeout` from ever firing.
    """

    def test_frozen_pose_stops_command_emission(self) -> None:
        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))
        frozen = sample(head_pose_matrix=yaw_matrix(0.3))
        self.assertIsNotNone(bridge.build(frozen, 1.0))
        # Within the stale window the pose may legitimately repeat.
        self.assertIsNotNone(bridge.build(frozen, 1.4))
        # Beyond it, the session is treated as lost.
        self.assertIsNone(bridge.build(frozen, 1.6))

    def test_frozen_pose_records_a_disconnect(self) -> None:
        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))
        frozen = sample(head_pose_matrix=yaw_matrix(0.3))
        bridge.build(frozen, 1.0)
        bridge.build(frozen, 2.0)
        self.assertEqual(bridge.state.disconnect_count, 1)
        self.assertEqual(bridge.state.transitions[-1]["event"], "disconnected")
        self.assertIn("unchanged", str(bridge.state.transitions[-1]["detail"]))

    def test_pose_changing_again_reconnects(self) -> None:
        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))
        frozen = sample(head_pose_matrix=yaw_matrix(0.3))
        bridge.build(frozen, 1.0)
        bridge.build(frozen, 2.0)
        self.assertIsNotNone(bridge.build(sample(head_pose_matrix=yaw_matrix(0.4)), 2.1))
        self.assertEqual(bridge.state.connect_count, 2)
        self.assertEqual(bridge.state.disconnect_count, 1)
        self.assertEqual(
            [transition["event"] for transition in bridge.state.transitions],
            ["connected", "disconnected", "connected"],
        )

    def test_observed_headset_removal_is_caught(self) -> None:
        """Replay of the recorded failure: 0.2 s stillness fine, 5.9 s not."""

        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))
        moving = 0.0
        timestamp = 1.0
        for _ in range(10):  # normal operation, pose changes every sample
            moving += 0.01
            self.assertIsNotNone(bridge.build(sample(head_pose_matrix=yaw_matrix(moving)), timestamp))
            timestamp += 1.0 / 30.0
        still = sample(head_pose_matrix=yaw_matrix(moving))
        for _ in range(6):  # 0.2 s of genuine stillness must not disconnect
            self.assertIsNotNone(bridge.build(still, timestamp))
            timestamp += 1.0 / 30.0
        self.assertEqual(bridge.state.disconnect_count, 0)
        emitted_during_freeze = 0
        for _ in range(177):  # the recorded 5.9 s removal
            if bridge.build(still, timestamp) is not None:
                emitted_during_freeze += 1
            timestamp += 1.0 / 30.0
        self.assertEqual(bridge.state.disconnect_count, 1)
        self.assertLess(emitted_during_freeze, 10, "commands kept flowing through a frozen pose")

    def test_stale_window_is_configurable_and_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.0))

    def test_disconnect_then_timeout_reaches_the_sink_as_a_hold(self) -> None:
        """The full chain the T001-B criterion asks for, with no Quest present."""

        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        adapter = SimulationOnlyAdapter(sink, mapper.ownership)
        bridge = QuestCommandBridge(BridgeConfig(max_pose_stale_s=0.5))

        moving = sample(head_pose_matrix=yaw_matrix(0.3))
        command = bridge.build(moving, 1.0)
        adapter.apply(mapper.map(command, 1.0))
        self.assertEqual(sink.events[-1]["event"], "upper_body")

        # Headset removed: pose freezes, so the bridge emits nothing further.
        self.assertIsNone(bridge.build(moving, 2.0))
        # The last command therefore ages past the mapper timeout.
        adapter.apply(mapper.map(command, 2.0))
        self.assertEqual(sink.events[-1]["event"], "hold")
        self.assertEqual(sink.events[-1]["reason"], "command_timeout")


class HeadOnlySinkTests(unittest.TestCase):
    def targets(self, *, enabled: bool = True, yaw: float = 0.2, pitch: float = -0.1) -> R1TeleopTargets:
        pose = Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        return R1TeleopTargets(
            sequence_id=7,
            enabled=enabled,
            reason=None if enabled else "deadman_released",
            left_wrist_target=pose if enabled else None,
            right_wrist_target=pose if enabled else None,
            head_yaw_rad=yaw,
            head_pitch_rad=pitch,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame="r1_base",
        )

    def test_enabled_command_writes_only_head_joints(self) -> None:
        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        sink.apply_upper_body(self.targets(), R1JointOwnership().upper_body)
        self.assertEqual(handle.writes, [(0.2, -0.1)])
        self.assertEqual(sink.acknowledgements[0]["accepted_joints"], list(HEAD_JOINT_NAMES))
        self.assertEqual(sink.arm_targets_withheld, 1)
        self.assertFalse(sink.events[0]["arm_target_written"])

    def test_arm_joints_are_reported_as_withheld(self) -> None:
        sink = HeadOnlyIsaacLabSink(RecordingHandle())
        sink.apply_upper_body(self.targets(), R1JointOwnership().upper_body)
        withheld = sink.acknowledgements[0]["withheld_joints"]
        self.assertIn("left_elbow_joint", withheld)
        self.assertNotIn("head_yaw_joint", withheld)

    def test_ownership_without_head_joints_is_refused(self) -> None:
        sink = HeadOnlyIsaacLabSink(RecordingHandle())
        with self.assertRaises(ValueError):
            sink.apply_upper_body(self.targets(), ("left_elbow_joint",))

    def test_hold_freezes_the_last_commanded_target(self) -> None:
        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        sink.apply_upper_body(self.targets(yaw=0.4, pitch=0.3), R1JointOwnership().upper_body)
        sink.hold("deadman_released")
        self.assertEqual(handle.writes, [(0.4, 0.3), (0.4, 0.3)])
        self.assertEqual(sink.events[-1]["held_head_target_rad"], [0.4, 0.3])

    def test_hold_before_any_command_writes_nothing(self) -> None:
        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        sink.hold("command_timeout")
        self.assertEqual(handle.writes, [])
        self.assertIsNone(sink.events[-1]["held_head_target_rad"])

    def test_base_velocity_dispatch_is_an_invariant_breach(self) -> None:
        sink = HeadOnlyIsaacLabSink(RecordingHandle())
        with self.assertRaises(VelocityDispatchError):
            sink.apply_base_velocity(self.targets(), R1JointOwnership().lower_body)

    def test_adapter_holds_and_never_dispatches_velocity(self) -> None:
        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        adapter = SimulationOnlyAdapter(sink)
        adapter.apply(self.targets(enabled=False))
        self.assertEqual(handle.writes, [])
        self.assertEqual(sink.events[-1]["reason"], "deadman_released")


class EndToEndFailClosedTests(unittest.TestCase):
    """Bridge -> mapper -> adapter -> sink, with no simulator and no Quest."""

    def build_chain(self) -> tuple[QuestCommandBridge, R1TeleopMapper, SimulationOnlyAdapter, HeadOnlyIsaacLabSink, RecordingHandle]:
        handle = RecordingHandle()
        sink = HeadOnlyIsaacLabSink(handle)
        mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
        return QuestCommandBridge(), mapper, SimulationOnlyAdapter(sink, mapper.ownership), sink, handle

    def test_quest_disconnect_produces_a_timeout_hold(self) -> None:
        bridge, mapper, adapter, sink, handle = self.build_chain()
        command = bridge.build(sample(head_pose_matrix=yaw_matrix(0.3)), 1.0)
        adapter.apply(mapper.map(command, 1.0))
        # The Quest stops delivering motion, so the bridge emits nothing and the
        # last command ages past the mapper timeout.
        self.assertIsNone(bridge.build(sample(motion_data_ready=False), 1.2))
        adapter.apply(mapper.map(command, 1.2 + mapper.limits.command_timeout_s))
        self.assertEqual(sink.events[-1]["event"], "hold")
        self.assertEqual(sink.events[-1]["reason"], "command_timeout")
        self.assertEqual(handle.writes[-1], handle.writes[0])

    def test_deadman_release_holds_without_moving_the_head(self) -> None:
        bridge, mapper, adapter, sink, handle = self.build_chain()
        adapter.apply(mapper.map(bridge.build(sample(head_pose_matrix=yaw_matrix(0.3)), 1.0), 1.0))
        adapter.apply(mapper.map(bridge.build(sample(head_pose_matrix=yaw_matrix(0.9), deadman_pressed=False), 1.1), 1.1))
        self.assertEqual(sink.events[-1]["reason"], "deadman_released")
        self.assertEqual(len(set(handle.writes)), 1)

    def test_no_sink_event_is_a_base_velocity_dispatch(self) -> None:
        bridge, mapper, adapter, sink, _ = self.build_chain()
        for index in range(10):
            moment = 1.0 + index * 0.01
            command = bridge.build(sample(head_pose_matrix=yaw_matrix(0.01 * index)), moment)
            adapter.apply(mapper.map(command, moment))
        self.assertEqual({event["event"] for event in sink.events}, {"upper_body"})


class RunnerEvidenceTests(unittest.TestCase):
    def test_live_runner_accepts_a_graceful_stop_file(self) -> None:
        args = build_parser().parse_args(["--output-dir", "example-run", "--stop-file", "example-run.stop"])
        self.assertEqual(args.stop_file, Path("example-run.stop"))

    def test_legacy_and_coupled_upper_body_modes_are_mutually_exclusive(self) -> None:
        with self.assertRaises(SystemExit):
            build_parser().parse_args([
                "--output-dir", "example-run",
                "--arm-head-config", "legacy.json",
                "--whole-upper-body-config", "coupled.json",
            ])

    def test_tracking_error_uses_post_physics_observation(self) -> None:
        metric = _head_tracking_error(
            [
                {
                    "enabled": True,
                    "head_yaw_rad": 0.5,
                    "head_pitch_rad": -0.2,
                    "post_physics_head_position_rad": [0.4, -0.1],
                },
                {"enabled": False, "head_yaw_rad": 0.0, "head_pitch_rad": 0.0},
            ]
        )
        self.assertEqual(metric["count"], 1)
        self.assertAlmostEqual(metric["max_yaw"], 0.1)
        self.assertAlmostEqual(metric["max_pitch"], 0.1)

    def test_tracking_error_uses_relative_applied_head_target_when_present(self) -> None:
        metric = _head_tracking_error([{
            "enabled": True, "head_yaw_rad": 0.9, "head_pitch_rad": 0.3,
            "applied_head_target_rad": [0.2, -0.1], "post_physics_head_position_rad": [0.1, -0.2],
        }])
        self.assertAlmostEqual(metric["max_yaw"], 0.1)
        self.assertAlmostEqual(metric["max_pitch"], 0.1)

    def test_tracking_error_excludes_an_ik_target_that_was_not_dispatched(self) -> None:
        metric = _head_tracking_error([{
            "enabled": True,
            "head_yaw_rad": 0.9,
            "head_pitch_rad": 0.3,
            "whole_upper_body": {"accepted": False, "reason": "upper_body_ik_refused"},
            "post_physics_head_position_rad": [0.0, 0.0],
        }])
        self.assertEqual(metric["count"], 0)

    def test_live_runner_help_needs_no_isaaclab_arguments(self) -> None:
        repo = Path(__file__).resolve().parents[2]
        result = subprocess.run(
            [sys.executable, "scripts/teleop/run_r1_quest3_live.py", "--help"],
            cwd=repo,
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("--output-dir", result.stdout)


if __name__ == "__main__":
    unittest.main()
