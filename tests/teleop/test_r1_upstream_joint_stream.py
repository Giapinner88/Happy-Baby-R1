"""Behaviour of the sink that applies vendor-solved joint vectors.

This sink is thin on purpose, so most of what is worth testing is what it
refuses to do: reorder joints it does not recognise, invent a target when none
arrived, or let a solved vector past the asset's limits. Those are the failures
that would be invisible in a video and only show up as a robot moving somewhere
the operator did not ask for.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from teleop.r1.mapping import R1TeleopTargets
from teleop.r1.upper_body_kinematics import ARMS_HEAD_JOINT_NAMES
from teleop.r1.upstream_joint_stream import (
    UpstreamJointStreamConfig,
    UpstreamJointStreamSink,
)

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "assets" / "R1.urdf"


class RecordingHandle:
    def __init__(self) -> None:
        self.writes: list[tuple[tuple[str, ...], tuple[float, ...]]] = []

    def write_joint_targets(self, joint_names, positions_rad) -> None:
        self.writes.append((tuple(joint_names), tuple(float(v) for v in positions_rad)))

    def joint_positions(self, joint_names):
        return tuple(0.0 for _ in joint_names)


def make_sink(**overrides):
    handle = RecordingHandle()
    config = UpstreamJointStreamConfig(urdf_path=URDF, **overrides)
    return handle, UpstreamJointStreamSink(handle, config)


def targets(sequence_id: int) -> R1TeleopTargets:
    class _Targets:
        pass

    value = _Targets()
    value.sequence_id = sequence_id
    return value


class UpstreamJointStreamSinkTest(unittest.TestCase):
    def test_applies_the_vector_it_was_given(self):
        handle, sink = make_sink()
        solved = [0.1] * len(ARMS_HEAD_JOINT_NAMES)
        sink.ingest(7, ARMS_HEAD_JOINT_NAMES, solved)
        sink.apply_upper_body(targets(7), list(ARMS_HEAD_JOINT_NAMES))

        self.assertTrue(sink.last_application["accepted"])
        np.testing.assert_allclose(sink.last_application["joint_target_rad"], solved)
        names, values = handle.writes[-1]
        # Held waist joints are appended to every dispatch, never solved.
        self.assertEqual(names[: len(ARMS_HEAD_JOINT_NAMES)], tuple(ARMS_HEAD_JOINT_NAMES))
        self.assertEqual(names[len(ARMS_HEAD_JOINT_NAMES):], ("waist_yaw_joint", "waist_roll_joint"))
        self.assertEqual(values[len(ARMS_HEAD_JOINT_NAMES):], (0.0, 0.0))

    def test_holds_when_no_solution_arrived_for_that_sequence(self):
        handle, sink = make_sink()
        sink.ingest(1, ARMS_HEAD_JOINT_NAMES, [0.2] * len(ARMS_HEAD_JOINT_NAMES))
        sink.apply_upper_body(targets(1), list(ARMS_HEAD_JOINT_NAMES))
        applied = list(sink.last_target)

        sink.apply_upper_body(targets(2), list(ARMS_HEAD_JOINT_NAMES))
        self.assertFalse(sink.last_application["accepted"])
        self.assertEqual(sink.last_application["reason"], "no_upstream_solution_for_sequence")
        # A hold repeats the last applied pose rather than falling to zero.
        np.testing.assert_allclose(sink.last_target, applied)
        np.testing.assert_allclose(handle.writes[-1][1][: len(applied)], applied)

    def test_rejects_a_vector_whose_joint_names_disagree(self):
        _handle, sink = make_sink()
        shuffled = list(ARMS_HEAD_JOINT_NAMES)
        shuffled[0], shuffled[1] = shuffled[1], shuffled[0]
        sink.ingest(3, shuffled, [0.3] * len(shuffled))

        # Rejected, not reordered: applying it would put right numbers on wrong joints.
        sink.apply_upper_body(targets(3), list(ARMS_HEAD_JOINT_NAMES))
        self.assertFalse(sink.last_application["accepted"])
        self.assertIn(
            "upstream_joint_names_rejected", [event["event"] for event in sink.events]
        )

    def test_rejects_a_wrong_length_or_non_finite_vector(self):
        _handle, sink = make_sink()
        sink.ingest(4, ARMS_HEAD_JOINT_NAMES, [0.1] * (len(ARMS_HEAD_JOINT_NAMES) - 1))
        sink.ingest(5, ARMS_HEAD_JOINT_NAMES, [float("nan")] * len(ARMS_HEAD_JOINT_NAMES))
        reasons = [e["event"] for e in sink.events]
        self.assertEqual(reasons.count("upstream_joint_vector_rejected"), 2)

    def test_clamps_a_vector_past_the_asset_limits_and_says_so(self):
        _handle, sink = make_sink()
        beyond = sink.model.upper_limits + 0.5
        sink.ingest(9, ARMS_HEAD_JOINT_NAMES, beyond.tolist())
        sink.apply_upper_body(targets(9), list(ARMS_HEAD_JOINT_NAMES))

        self.assertTrue(sink.last_application["accepted"])
        np.testing.assert_array_less(
            sink.last_target, sink.model.upper_limits + 1e-9
        )
        self.assertAlmostEqual(sink.last_application["joint_limit_clamp_rad"], 0.5, places=9)
        self.assertEqual(sink.summary()["joint_limit_clamped_sample_count"], 1)

    def test_base_velocity_is_refused(self):
        _handle, sink = make_sink()
        with self.assertRaises(RuntimeError):
            sink.apply_base_velocity(targets(1), [])

    def test_pending_vectors_are_bounded(self):
        _handle, sink = make_sink(pending_capacity=4)
        for sequence in range(20):
            sink.ingest(sequence, ARMS_HEAD_JOINT_NAMES, [0.0] * len(ARMS_HEAD_JOINT_NAMES))
        self.assertEqual(sink.summary()["pending_unapplied_count"], 4)
        # The four kept are the newest, since the oldest are the ones already passed by.
        sink.apply_upper_body(targets(19), list(ARMS_HEAD_JOINT_NAMES))
        self.assertTrue(sink.last_application["accepted"])

    def test_a_non_arms_head_body_mode_is_refused(self):
        with self.assertRaises(ValueError):
            UpstreamJointStreamSink(
                RecordingHandle(),
                UpstreamJointStreamConfig(urdf_path=URDF, body_mode="waist_yaw"),
            )

    def test_summary_reports_no_rate_limiter(self):
        _handle, sink = make_sink()
        self.assertIn("none", str(sink.summary()["rate_limiter"]))

    def test_reset_returns_to_the_nominal_the_solver_warm_starts_from(self):
        handle, sink = make_sink()
        sink.ingest(1, ARMS_HEAD_JOINT_NAMES, [0.4] * len(ARMS_HEAD_JOINT_NAMES))
        sink.apply_upper_body(targets(1), list(ARMS_HEAD_JOINT_NAMES))
        sink.reset_session()
        np.testing.assert_allclose(sink.last_target, np.zeros(sink.model.dof))
        np.testing.assert_allclose(
            sink.nominal_joint_position_rad, np.zeros(sink.model.dof)
        )


if __name__ == "__main__":
    unittest.main()


class RepeatedSequenceTest(unittest.TestCase):
    """The control loop re-applies the last command when none is fresh.

    A lookup that consumed its entry turned that ordinary case into a hold, and
    the hold was indistinguishable in evidence from the stream genuinely
    dropping a sample.
    """

    def test_the_same_sequence_can_be_applied_twice(self):
        _handle, sink = make_sink()
        # Midway between the asset's limits, so the clamp cannot fire and mask
        # what this test is actually about. A flat constant does not work:
        # right_shoulder_roll_joint tops out at 0.2268 rad.
        solved = (0.5 * (sink.model.lower_limits + sink.model.upper_limits)).tolist()
        sink.ingest(11, ARMS_HEAD_JOINT_NAMES, solved)

        sink.apply_upper_body(targets(11), list(ARMS_HEAD_JOINT_NAMES))
        self.assertTrue(sink.last_application["accepted"])

        sink.apply_upper_body(targets(11), list(ARMS_HEAD_JOINT_NAMES))
        self.assertTrue(sink.last_application["accepted"])
        np.testing.assert_allclose(sink.last_application["joint_target_rad"], solved)
