from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from scripts.teleop.record_robot_camera_zmq import validate_endpoint
from scripts.teleop.compare_sim_hardware_run import compare


ROOT = Path(__file__).resolve().parents[2]


def test_fanout_preserves_primary_and_mirror_streams(tmp_path: Path) -> None:
    mirror = tmp_path / "mirror.jsonl"
    stats = tmp_path / "stats.json"
    source = "".join(json.dumps({"sequence_id": index}) + "\n" for index in range(5))
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/teleop/fanout_command_stream.py"),
            "--mirror-path",
            str(mirror),
            "--stats-path",
            str(stats),
        ],
        input=source,
        text=True,
        capture_output=True,
        check=True,
    )
    assert result.stdout == source
    assert mirror.read_text(encoding="utf-8") == source
    report = json.loads(stats.read_text(encoding="utf-8"))
    assert report["primary_line_count"] == 5
    assert report["mirror_line_count"] == 5
    assert report["mirror_drop_count"] == 0


def test_camera_zmq_endpoint_contract() -> None:
    assert validate_endpoint("tcp://robot.local:55555") == "tcp://robot.local:55555"
    for invalid in (
        "https://robot.local:60001/offer",
        "tcp://robot.local",
        "tcp://robot.local:55555/camera",
    ):
        with pytest.raises(ValueError):
            validate_endpoint(invalid)


def test_comparison_uses_shared_sequence_and_named_joint_reordering(tmp_path: Path) -> None:
    names = ["joint_a", "joint_b"]
    wire_names = ["joint_b", "joint_a"]
    producer = {
        "sequence_id": 7,
        "operator_enabled": True,
        "upstream_joint_names": names,
        "upstream_joint_position_rad": [0.1, 0.2],
    }
    (tmp_path / "hardware_targets.jsonl").write_text(json.dumps(producer) + "\n")
    simulation = tmp_path / "simulation"
    simulation.mkdir()
    (simulation / "targets.json").write_text(
        json.dumps([
            {
                "sequence_id": 7,
                "enabled": True,
                "whole_upper_body": {
                    "accepted": True,
                    "controlled_joint_names": names,
                    "joint_target_rad": [0.1, 0.2],
                },
                "post_physics_whole_upper_body_position_rad": [0.11, 0.19],
            }
        ])
    )
    robot = tmp_path / "robot"
    robot.mkdir()
    (robot / "samples.jsonl").write_text(
        json.dumps(
            {
                "upstream_sequence_id": 7,
                "joint_names": wire_names,
                "target_q": [0.2, 0.1],
                "observed_q": [0.18, 0.12],
            }
        )
        + "\n"
    )
    result = compare(tmp_path)
    assert result["producer_to_sim_command_max_abs_rad"]["max"] == 0.0
    assert result["producer_to_robot_target_max_abs_rad"]["max"] == 0.0
    assert result["sim_to_robot_observed_max_abs_rad"]["max"] == pytest.approx(0.01)
