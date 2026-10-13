"""Hardware R1 head/frame contract; catches ZYX extraction on a Ry@Rz head."""

from __future__ import annotations

from math import cos, nan, sin

import pytest

from teleop.r1.mapping import R1TeleopMapper, TeleopCalibration, TeleopLimits
from teleop.r1.schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3


def _head_pose(pitch: float, yaw: float) -> Pose:
    # R1 URDF: head_pitch_joint is parent of head_yaw_joint, so R = Ry @ Rz.
    return Pose(
        Vector3(0.0, 0.0, 0.0),
        Quaternion(
            sin(pitch / 2.0) * sin(yaw / 2.0),
            sin(pitch / 2.0) * cos(yaw / 2.0),
            cos(pitch / 2.0) * sin(yaw / 2.0),
            cos(pitch / 2.0) * cos(yaw / 2.0),
        ),
    )


def _command(pitch: float = 0.3, yaw: float = 0.4, timestamp: float = 10.0) -> R1TeleopCommand:
    head = _head_pose(pitch, yaw)
    wrist = Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    return R1TeleopCommand(1, timestamp, True, head, wrist, wrist, BaseVelocity.zero())


@pytest.mark.parametrize("pitch,yaw", [(0.3, 0.4), (0.3, 0.0), (0.0, 0.4)])
def test_head_fk_round_trip_for_r1_axis_order(pitch: float, yaw: float) -> None:
    mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
    target = mapper.map(_command(pitch, yaw), 10.1)
    assert target.enabled
    assert target.robot_frame == "neutral_waist_yaw_link"
    assert target.head_pitch_rad == pytest.approx(pitch, abs=1e-6)
    assert target.head_yaw_rad == pytest.approx(yaw, abs=1e-6)


def test_future_command_does_not_gain_authority() -> None:
    mapper = R1TeleopMapper(TeleopCalibration(), TeleopLimits(0.5))
    result = mapper.map(_command(timestamp=11.0), 10.0)
    assert not result.enabled
    assert result.reason == "command_timestamp_in_future"


def test_nonfinite_calibration_and_reversed_limits_are_rejected() -> None:
    with pytest.raises(ValueError):
        TeleopCalibration(yaw_rad=nan)
    with pytest.raises(ValueError):
        TeleopLimits(0.5, head_yaw_range_rad=(1.0, -1.0))
