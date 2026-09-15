"""Kiểm tra gate trước khi gắn nhãn một raw dataset là true-30-FPS."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from scripts.teleop.check_r1_dataset_fps import main


def _write_episode(root: Path, source_frames: list[int | None]) -> None:
    episode = root / "episode_0000"
    colors = episode / "colors"
    colors.mkdir(parents=True)
    data = []
    for index, source_frame in enumerate(source_frames):
        relative = f"colors/{index:06d}_color_0.jpg"
        (episode / relative).touch()
        sim_state = {"elapsed_s": index / 30.0}
        if source_frame is not None:
            sim_state["source_camera_frame"] = source_frame
        data.append(
            {
                "idx": index,
                "colors": {"color_0": relative},
                "sim_state": sim_state,
            }
        )
    payload = {
        "info": {
            "image": {"fps": 30.0, "fps_requested": 30.0},
            "accepted": True,
            "writer_dropped_capture_count": 0,
        },
        "data": data,
    }
    (episode / "data.json").write_text(json.dumps(payload), encoding="utf-8")


class DatasetFpsGateTests(unittest.TestCase):
    def _run(self, raw_dir: Path) -> int:
        argv = [
            "check_r1_dataset_fps.py",
            "--raw-dir",
            str(raw_dir),
            "--expected-fps",
            "30",
            "--min-measured-fps",
            "27",
            "--max-measured-fps",
            "33",
        ]
        with mock.patch.object(sys, "argv", argv):
            return main()

    def test_strictly_increasing_source_frames_pass(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            _write_episode(root, [10, 11, 12])
            self.assertEqual(self._run(root), 0)

    def test_duplicate_source_frame_fails(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            _write_episode(root, [10, 10, 11])
            self.assertEqual(self._run(root), 2)

    def test_missing_source_frame_id_fails(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            _write_episode(root, [10, None, 12])
            self.assertEqual(self._run(root), 2)


if __name__ == "__main__":
    unittest.main()
