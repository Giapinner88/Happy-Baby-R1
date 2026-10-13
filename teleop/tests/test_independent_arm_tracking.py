"""Explicit inactive-arm holds never depend on stale Quest poses or IK output."""
import json
import sys
from types import SimpleNamespace
import numpy as np
from teleop.hardware.high_level_sidecar import H4SourceSession, H4Target, JOINT_NAMES, parse_h4_target
from test_head_video_regressions import ik


def target(sequence, positions, left=True):
    return H4Target(sequence, sequence, True, positions, (left, True))


def test_right_only_holds_left_encoder_position_for_two_minutes():
    session = H4SourceSession(None, .3)
    observed = [.4] * 12
    for sequence in range(3601):
        q = [2.0] * 5 + [.1 + sequence / 36010] * 5 + [0, 0]
        value = session.consume(target(sequence, q, left=False), observed, sequence / 30)
        assert value[:5] == [.4] * 5  # Solver's unrelated left targets are excluded.
        assert value[5:10] == q[5:10]
        observed = [.8] * 12  # Hold is latched once, never follows encoder drift.


def test_left_loss_freezes_encoders_and_reappearance_waits_for_release():
    session = H4SourceSession(None, .3)
    assert session.consume(target(1, [.7] * 12), [.2] * 12, 0)[:5] == [.7] * 5
    assert session.consume(target(2, [1.3] * 12, False), [.3] * 12, .03)[:5] == [.3] * 5
    assert session.consume(target(3, [1.4] * 12, True), [.4] * 12, .06)[:5] == [.3] * 5
    assert session.consume(H4Target(4, 4, False, None), [.4] * 12, .09) is None
    assert session.consume(target(5, [.6] * 12), [.4] * 12, .12)[:5] == [.6] * 5


def test_h4_mask_is_explicit_and_malformed_or_missing_deadman_hand_rejected():
    payload = dict(sequence_id=1, upstream_sequence_id=1, enabled=True,
                   joint_names=JOINT_NAMES, positions_rad=[0] * 12,
                   arm_tracking={'left': False, 'right': True})
    assert parse_h4_target(json.dumps(payload), 0).arm_tracking == (False, True)
    for mask in ({'left': False, 'right': False}, {'left': 0, 'right': True}, {}, [False, True]):
        assert parse_h4_target(json.dumps({**payload, 'arm_tracking': mask}), 0) is None


def test_inactive_ik_effector_uses_robot_fk_and_active_arm_is_solved_every_sample(monkeypatch):
    fk = np.eye(4); fk[:3, 3] = [.3, .2, .1]
    data = SimpleNamespace(oMf=[SimpleNamespace(homogeneous=fk)])
    model = SimpleNamespace(createData=lambda: data)
    calls = []
    def solve(left, right):
        calls.append((left.copy(), right.copy()))
        return np.arange(10) / 10, np.zeros(10)
    solver = SimpleNamespace(reduced_robot=SimpleNamespace(model=model), init_data=np.zeros(10),
                             L_hand_id=0, solve_ik=solve)
    monkeypatch.setitem(sys.modules, 'pinocchio', SimpleNamespace(framesForwardKinematics=lambda *args: None))
    for x in (.4, .5):
        right = np.eye(4); right[0, 3] = x
        solution, _, left, result_right = ik.solve_tracked_arms(
            solver, np.full((4, 4), 123), right, {'left': False, 'right': True})
        assert np.array_equal(left, fk) and np.array_equal(result_right, right)
        assert np.all(np.isfinite(solution))
    assert len(calls) == 2 and calls[0][1][0, 3] != calls[1][1][0, 3]
