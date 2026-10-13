"""Estimated positions STOP immediately; only live, bounded recovery can resume."""
import copy
import pytest
from test_xr_frame_gate import frame
from quest_tracking import TrackingGate


def tracked(sequence, held=True):
    value = frame(sequence, float(held), held)
    value.update(rightInputId=2, rightInputConnected=True)
    return value


def estimated(sequence):
    return dict(sequence=sequence, valid=False, reason='right_pose_missing_or_emulated',
                rightInputId=2, rightInputConnected=True,
                rightState=dict(trigger=True, triggerValue=1),
                positionPauseProof=dict(pose_present=True, position_emulated=True,
                                        viewer_tracked=True, visibility='visible',
                                        viewer_matrix=frame(1)['camera']['matrix']))


def active_gate():
    gate = TrackingGate()
    gate.update(tracked(1, False), 0)
    assert gate.update(tracked(2), .03)['rightState']['trigger']
    return gate


@pytest.mark.parametrize('duration', [.241, 1.118])
def test_real_trial_position_gaps_stop_then_resume_without_release(duration):
    gate = active_gate()
    sequence, at = 3, .06
    while at < .06 + duration:
        assert gate.update(estimated(sequence), at) is None
        assert gate.reason == 'right_position_paused'
        assert not gate.release_required
        sequence, at = sequence + 1, at + .03
    for step in range(4):
        assert gate.update(tracked(sequence), at + step * .03) is None
        assert gate.reason == 'right_position_recovering'
        sequence += 1
    result = gate.update(tracked(sequence), at + .12)
    assert result['rightState']['trigger'] and not gate.release_required
    assert gate.position_pause_started is None


@pytest.mark.parametrize('change', [
    {'rightInputId': 3}, {'rightInputConnected': False},
    {'rightState': {'trigger': False, 'triggerValue': 0}},
    {'rightState': {'trigger': True, 'triggerValue': .49}},
    {'rightState': None}, {'sequence': 2}, {'valid': True},
    {'reason': 'right_input_source_missing'},
    {'positionPauseProof': None},
])
def test_missing_button_identity_or_proof_requires_manual_release(change):
    gate = active_gate()
    value = estimated(3)
    value.update(copy.deepcopy(change))
    assert gate.update(value, .06) is None
    assert gate.release_required
    assert not gate.update(tracked(4), .09)['rightState']['trigger']
    gate.update(tracked(5, False), .12)
    assert gate.update(tracked(6), .15)['rightState']['trigger']


@pytest.mark.parametrize('field,value', [('pose_present', False), ('position_emulated', False),
                                      ('viewer_tracked', False), ('visibility', 'hidden'),
                                      ('viewer_matrix', [0] * 16)])
def test_invalid_viewer_or_unproven_position_estimate_cannot_resume(field, value):
    gate = active_gate()
    packet = estimated(3)
    packet['positionPauseProof'][field] = value
    assert gate.update(packet, .06) is None and gate.release_required


def test_pause_does_not_extend_network_lease_or_allow_replayed_packets():
    for packet, at in [(estimated(4), .57), (estimated(3), .09)]:
        gate = active_gate()
        assert gate.update(estimated(3), .06) is None
        assert gate.update(packet, at) is None and gate.release_required
        assert not gate.update(tracked(5), at + .03)['rightState']['trigger']


def test_pause_timeout_and_expiry_during_recovery_require_release():
    for recover in (False, True):
        gate = active_gate()
        sequence = 3
        for step in range(67):
            assert gate.update(estimated(sequence), .06 + step * .03) is None
            sequence += 1
        result = gate.update(tracked(sequence) if recover else estimated(sequence), 2.10)
        assert gate.release_required
        assert result is None or not result['rightState']['trigger']


@pytest.mark.parametrize('change', [ {'rightInputId': 3}, {'rightInputConnected': False},
    {'rightState': {'trigger': False, 'triggerValue': 0}},
    {'rightState': {'trigger': True, 'triggerValue': .49}} ])
def test_recovery_requires_same_controller_still_holding_trigger(change):
    gate = active_gate()
    gate.update(estimated(3), .06)
    packet = tracked(4)
    packet.update(change)
    result = gate.update(packet, .09)
    assert not result['rightState']['trigger']
    # A fully tracked physical release can clear the manual-release gate.
    assert gate.release_required == bool(packet['rightState']['triggerValue'])


def test_second_estimate_restarts_stability_timer_without_extending_budget():
    gate = active_gate()
    gate.update(estimated(3), .06)
    assert gate.update(tracked(4), .09) is None
    assert gate.update(estimated(5), .15) is None
    assert gate.position_pause_started == .06
    assert gate.update(tracked(6), .18) is None
    assert gate.update(tracked(7), .25) is None
    assert gate.update(tracked(8), .30)['rightState']['trigger']


def test_legacy_client_without_identity_keeps_manual_rearm_behavior():
    gate = TrackingGate()
    gate.update(frame(1), 0)
    gate.update(frame(2, 1, True), .03)
    assert gate.update(estimated(3), .06) is None and gate.release_required
