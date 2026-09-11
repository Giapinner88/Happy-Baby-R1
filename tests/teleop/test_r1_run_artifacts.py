from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.teleop.finalize_r1_run import HARDWARE_DATA, SIM_DATA, evaluate_artifacts


class RunArtifactGateTests(unittest.TestCase):
    def test_simulation_requires_data_figure_and_video(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            for name in SIM_DATA:
                (run / name).write_bytes(b"x")
            (run / "figures").mkdir()
            (run / "figures" / "tracking.png").write_bytes(b"png")
            (run / "simulator_view.mp4").write_bytes(b"mp4")
            result = evaluate_artifacts(run, "simulation")
            self.assertTrue(result["data"])
            self.assertTrue(result["figures"])
            self.assertTrue(result["video"])

    def test_hardware_rejects_missing_robot_samples(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            for name in HARDWARE_DATA:
                if name == "robot/samples.jsonl":
                    continue
                path = run / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"x")
            (run / "metrics.json").write_bytes(b"x")
            (run / "figures").mkdir()
            (run / "figures" / "tracking.png").write_bytes(b"png")
            (run / "telemetry_tracking.mp4").write_bytes(b"mp4")
            result = evaluate_artifacts(run, "hardware")
            self.assertFalse(result["data"])
            self.assertIn("robot/samples.jsonl", result["missing_data"])


if __name__ == "__main__":
    unittest.main()
