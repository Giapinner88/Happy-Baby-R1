"""Regression checks for operator translation and reset without motor transport."""
import importlib.util
import json
from pathlib import Path
import sys
import subprocess
import select
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "vendor_reference_test", ROOT / "scripts/teleop/run_r1_vendor_ik_stream.py")
ik = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ik)


def test_body_translation_does_not_push_hand_forward():
    anchor = np.eye(4)
    anchor[:3, 3] = [-0.32, -0.07, 1.10]
    wrist = np.eye(4)
    wrist[:3, 3] = [0.08, -0.2, -0.3]
    head = anchor.copy()
    head[:3, 3] += [0.78, 0.17, 0.35]
    np.testing.assert_allclose(ik.body_relative_wrist_matrix(wrist, head, anchor), wrist)
    old = ik.reanchor_wrist_matrix(wrist, head, anchor)
    np.testing.assert_allclose(old[:3, 3] - wrist[:3, 3], [0.78, 0.17, 0.35])


def test_slow_solver_keeps_reset_and_release_before_latest_pose():
    commands = [dict(sequence_id=i, deadman_enabled=i != 4, reset_requested=i == 2)
                for i in range(8)]
    batch = []
    for command in commands:
        batch = ik.compact_live_commands(batch + [json.dumps(command)])
    assert [json.loads(c)["sequence_id"] for c in batch] == [2, 4, 7]


def test_gaze_yaw_does_not_rotate_stationary_hand():
    anchor = np.eye(4)
    head = np.eye(4)
    angle = 0.7
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    head[:3, :3] = rotation
    desired = np.eye(4)
    desired[:3, 3] = [0.1, -0.25, -0.2]
    reported = desired.copy()
    reported[:3, :3] = rotation.T
    offset = ik.VENDOR_HEAD_TO_WAIST_OFFSET_M
    reported[:3, 3] = rotation.T @ (desired[:3, 3] - offset) + offset
    np.testing.assert_allclose(ik.body_relative_wrist_matrix(reported, head, anchor),
                               desired, atol=1e-12)


def test_reset_emits_stop_and_requires_release_before_new_solve(tmp_path, monkeypatch):
    class Solver:
        def solve_ik(self, left, right):
            return np.zeros(10), np.zeros(10)
    monkeypatch.setattr(ik, "load_solver", lambda: Solver())
    pose = {"position": {"x": 0, "y": 0, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
    commands = []
    # Calibrate, solve, reset while held, continue holding, release, then resume.
    for seq, (enabled, reset) in enumerate([
        (False, False), (True, False), (True, False), (True, False),
        (True, True), (True, False), (False, False),
        (True, False), (True, False), (True, False)]):
        commands.append(dict(sequence_id=seq, timestamp_monotonic_s=float(seq),
                             deadman_enabled=enabled, reset_requested=reset,
                             head_pose=pose, left_wrist_pose=pose, right_wrist_pose=pose))
    source, output = tmp_path / "input.jsonl", tmp_path / "output.jsonl"
    source.write_text("".join(json.dumps(c) + "\n" for c in commands))
    monkeypatch.setattr(sys, "argv", ["ik", "--input", str(source), "--output", str(output),
                                     "--passthrough", "--wrist-reference", "body_relative"])
    assert ik.main() == 0
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [r["sequence_id"] for r in records] == list(range(10))
    assert [r["sequence_id"] for r in records if r["deadman_enabled"]] == [3, 9]
    assert all("upstream_joint_position_rad" not in r for r in records if not r["deadman_enabled"])


def test_h4_mapper_survives_reset_and_accepts_next_fresh_target():
    proc = subprocess.Popen([sys.executable, str(ROOT / "scripts/teleop/run_r1_vendor_targets.py"),
                             "--duration-s", "0", "--control-hz", "100",
                             "--source-timeout-s", "1", "--stream-contract", "h4",
                             "--joint-limits-only"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True)
    names = ik.ARM_JOINT_NAMES + ik.HEAD_JOINT_NAMES
    try:
        for seq, (enabled, reset) in enumerate([(True, False), (False, True),
                                                (False, False), (True, False)]):
            frame = dict(sequence_id=seq, timestamp_monotonic_s=time.monotonic(),
                         deadman_enabled=enabled, reset_requested=reset,
                         upstream_solver="upstream_xr_teleoperate_R1_A5_ArmIK",
                         upstream_joint_names=names, upstream_joint_position_rad=[0.0]*12,
                         upstream_solved_this_sample=enabled)
            proc.stdin.write(json.dumps(frame) + "\n")
            proc.stdin.flush()
            assert select.select([proc.stdout], [], [], 3)[0], "mapper stopped producing frames"
            line = proc.stdout.readline()
            assert line, "mapper exited on reset"
            assert json.loads(line)["enabled"] is enabled
            assert proc.poll() is None
    finally:
        proc.terminate()
        proc.communicate(timeout=3)
