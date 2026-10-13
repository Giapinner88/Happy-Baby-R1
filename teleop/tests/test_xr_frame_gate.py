"""Complete tracked frames with safe trigger hysteresis and lease boundaries."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/teleop'))
from quest_tracking import TrackingGate


def frame(sequence, analog=0, digital=False):
    matrix=np.eye(4).flatten(order='F').tolist()
    return dict(sequence=sequence,valid=True,camera=dict(matrix=matrix),left=matrix,right=matrix,
                leftState=dict(trigger=False,triggerValue=0),
                rightState=dict(trigger=digital,triggerValue=analog))


def test_partial_trigger_jitter_does_not_release_but_real_release_does():
    gate=TrackingGate()
    assert not gate.update(frame(1),0)['rightState']['trigger']
    assert gate.update(frame(2,1,True),.03)['rightState']['trigger']
    for seq, analog in enumerate((.452,.411,.379,.301,.216),3):
        assert gate.update(frame(seq,analog),seq*.03)['rightState']['trigger']
    assert not gate.update(frame(9,0),.3)['rightState']['trigger']
    assert not gate.update(frame(10,.4),.33)['rightState']['trigger']
    assert gate.update(frame(11,.8),.36)['rightState']['trigger']


def test_tracking_loss_or_replayed_frame_requires_physical_release():
    gate=TrackingGate()
    gate.update(frame(1),0)
    assert gate.update(frame(2,1,True),.03)['rightState']['trigger']
    assert gate.update(dict(sequence=3,valid=False,reason='right_pose_missing'),.06) is None
    assert gate.reason=='right_pose_missing'
    assert not gate.update(frame(4,1,True),.09)['rightState']['trigger']
    gate.update(frame(5),.12)
    assert gate.update(frame(6,1,True),.15)['rightState']['trigger']
    assert gate.update(frame(6,1,True),.18) is None
    assert gate.release_required
    gate.update(frame(7),.21)
    assert not gate.update(frame(8,1,True),1)['rightState']['trigger']


def test_lost_left_controller_does_not_release_tracked_right_arm():
    gate = TrackingGate()
    gate.update(frame(1), 0)
    assert gate.update(frame(2, 1, True), .03)['arm_tracking']['left']
    partial = frame(3, 1, True)
    partial.update(arm_tracking={'left': False, 'right': True}, left=None)
    value = gate.update(partial, .06)
    assert value['rightState']['trigger']
    assert value['arm_tracking'] == {'left': False, 'right': True}
    # Reappearance while held must not reactivate the left arm.
    value = gate.update(frame(4, 1, True), .09)
    assert value['rightState']['trigger'] and not value['arm_tracking']['left']
    gate.update(frame(5), .12)
    assert gate.update(frame(6, 1, True), .15)['arm_tracking']['left']
    bad = frame(7, 1, True)
    bad.update(arm_tracking={'left': True, 'right': False}, right=None)
    assert gate.update(bad, .18) is None
    assert gate.release_required


def test_right_only_session_can_release_and_arm_without_left_controller():
    gate = TrackingGate()
    for sequence, analog in enumerate((0, 1, 1, 1), 1):
        value = frame(sequence, analog, bool(analog))
        value.update(arm_tracking={'left': False, 'right': True}, left=None)
        normalized = gate.update(value, sequence / 30)
        assert normalized is not None
        assert normalized['rightState']['trigger'] == bool(analog)
