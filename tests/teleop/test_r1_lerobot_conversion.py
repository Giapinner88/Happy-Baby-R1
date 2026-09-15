"""Tests for the project-owned 12-DoF R1 LeRobot contract."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from teleop.r1.lerobot_conversion import (
    HEAD_FEATURE_NAMES,
    HEAD_RAW_NAMES,
    feature_name,
    joint_vector,
    validate_head_contract,
)


def _content(offset: float = 0.0) -> dict:
    return {
        "left_arm": {"qpos": [offset + value for value in range(5)]},
        "right_arm": {"qpos": [offset + value for value in range(5, 10)]},
        "body": {"qpos": [offset + 10.0, offset + 11.0]},
    }


class R1LeRobotConversionTests(unittest.TestCase):
    def test_joint_vector_appends_head_pitch_then_head_yaw(self) -> None:
        vector = joint_vector(_content(), episode=Path("data.json"), index=0)
        np.testing.assert_array_equal(vector, np.arange(12, dtype=np.float32))

    def test_head_feature_names_follow_existing_q_convention(self) -> None:
        self.assertEqual(
            tuple(feature_name(name) for name in HEAD_RAW_NAMES),
            HEAD_FEATURE_NAMES,
        )

    def test_head_contract_requires_names_and_two_values_in_every_section(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            episode_dir = Path(raw) / "episode_0000"
            episode_dir.mkdir()
            payload = {
                "info": {"joint_names": {"body": list(HEAD_RAW_NAMES)}},
                "data": [{"states": _content(), "actions": _content(0.5)}],
            }
            (episode_dir / "data.json").write_text(json.dumps(payload), encoding="utf-8")
            episode = SimpleNamespace(path=episode_dir, frames=payload["data"])
            self.assertEqual(validate_head_contract([episode]), HEAD_FEATURE_NAMES)

            payload["data"][0]["actions"]["body"]["qpos"] = [0.0]
            episode.frames = payload["data"]
            with self.assertRaisesRegex(ValueError, "expected 2"):
                validate_head_contract([episode])

    def test_head_contract_rejects_ambiguous_joint_order(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            episode_dir = Path(raw) / "episode_0000"
            episode_dir.mkdir()
            payload = {
                "info": {"joint_names": {"body": list(reversed(HEAD_RAW_NAMES))}},
                "data": [{"states": _content(), "actions": _content()}],
            }
            (episode_dir / "data.json").write_text(json.dumps(payload), encoding="utf-8")
            episode = SimpleNamespace(path=episode_dir, frames=payload["data"])
            with self.assertRaisesRegex(ValueError, "info.joint_names.body"):
                validate_head_contract([episode])


if __name__ == "__main__":
    unittest.main()
