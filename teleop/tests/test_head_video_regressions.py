"""Head reference, adaptive video and raw deadman regression checks."""
import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts/teleop"
sys.path.insert(0, str(SCRIPTS))
from video_adaptation import AdaptiveVideoRate
from quest_camera_webrtc import raw_trigger

spec = importlib.util.spec_from_file_location("head_regression", SCRIPTS / "run_r1_vendor_ik_stream.py")
ik = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ik)


def rot(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == "z":
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    if axis == "y":
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])



@pytest.mark.parametrize("heading", [0, 80, 120, -150, 179])
@pytest.mark.parametrize("pitch,yaw", [(0, .25), (0, -.25), (.3, 0), (-.3, .2), (.25, -.3)])
def test_neck_angles_independent_of_room_heading(heading, pitch, yaw, monkeypatch):
    neutral = np.eye(4)
    neutral[:3,:3] = rot("z", np.deg2rad(heading)) @ rot("y", .12) @ rot("x", -.08)
    matrix = neutral.copy()
    matrix[:3,:3] = neutral[:3,:3] @ rot("y", pitch) @ rot("z", yaw)
    monkeypatch.setattr(ik, "pose_to_matrix", lambda _: matrix)
    assert ik.relative_head_angles({}, neutral, ((-.7,.7),(-.65,.65))) == pytest.approx((pitch,yaw))


def feedback(policy, **overrides):
    values = dict(capture_s=.01, cycle_s=.01, rtt_s=.01, loss=0, sent_bps=100000, budget_bps=1000000)
    values.update(overrides)
    return policy.update(**values)


def test_auto_fps_grows_and_respects_measured_processing_capacity():
    policy = AdaptiveVideoRate()
    for _ in range(30):
        feedback(policy)
    assert policy.fps > 30
    assert feedback(policy, capture_s=.1) == "processing_capacity"
    assert policy.fps == pytest.approx(8.5)


def test_network_reduces_resolution_then_fps_and_recovers():
    policy = AdaptiveVideoRate()
    assert feedback(policy, loss=.1) == "network_reduce_resolution"
    assert policy.fps == 12 and policy.scale < 1
    for _ in range(8):
        feedback(policy, loss=.1)
    assert policy.scale == .5 and policy.fps < 12
    for _ in range(60):
        feedback(policy)
    assert policy.scale == 1 and policy.fps > 12


def test_explicit_operator_fps_cap_applies_from_start():
    policy = AdaptiveVideoRate(6)
    assert policy.fps == 6
    for _ in range(30):
        feedback(policy)
    assert policy.fps <= 6


@pytest.mark.parametrize("state", [{}, None, {"triggerValue": None}, {"triggerValue": float("nan")},
                                    {"triggerValue": 1.1}, {"triggerValue": True}, {"trigger": "true"}])
def test_missing_or_invalid_raw_trigger_is_not_a_hold(state):
    result = raw_trigger(state)
    assert not result["digital_pressed"] and not result["analog_valid"]


def test_raw_diagnostics_distinguish_release_and_missing():
    missing = raw_trigger({})
    release = raw_trigger({"trigger": False, "triggerValue": 0})
    held = raw_trigger({"trigger": True, "triggerValue": 1})
    assert not missing["analog_present"]
    assert release["analog_valid"] and release["analog_value"] == 0
    assert held["analog_value"] == 1 and held["digital_pressed"]


def test_remb_startup_estimate_does_not_reduce_on_idle_network():
    policy = AdaptiveVideoRate()
    for _ in range(12):
        feedback(policy, sent_bps=20000, budget_bps=18000)
    assert policy.scale == 1 and policy.fps > 12


def test_sustained_budget_overrun_reduces_after_two_windows():
    policy = AdaptiveVideoRate()
    assert feedback(policy, sent_bps=700000, budget_bps=500000) == "steady"
    assert feedback(policy, sent_bps=700000, budget_bps=500000) == "network_reduce_resolution"
