import json
import tempfile
import unittest
from pathlib import Path

from teleop.r1.dataset_reclassification import reclassify_run


def _write_episode(root: Path, number: int, frames: int, *, accepted: bool, reasons: list[str]) -> None:
    branch = "episodes" if accepted else "episodes_rejected"
    episode = root / branch / f"episode_{number:04d}"
    colors = episode / "colors"
    colors.mkdir(parents=True, exist_ok=True)
    rows = []
    for index in range(frames):
        relative = f"colors/{index:06d}_color_0.jpg"
        (episode / relative).write_bytes(b"jpeg")
        rows.append(
            {
                "idx": index,
                "colors": {"color_0": relative},
                "sim_state": {"control_step": index, "elapsed_s": index / 7},
            }
        )
    payload = {
        "info": {
            "accepted": accepted,
            "rejection_reasons": reasons,
            "projected_item_count": 0,
            "writer_dropped_capture_count": 0,
            "image": {"fps": 7.0, "fps_requested": 30},
            "task": {"target_digit": number + 1, "target_slot": 0},
        },
        "data": rows,
    }
    (episode / "data.json").write_text(json.dumps(payload), encoding="utf-8")


class DatasetReclassificationTests(unittest.TestCase):
    def test_promotes_only_too_short_without_mutating_source(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            source = root / "source"
            (source / "episodes").mkdir(parents=True)
            (source / "episodes_rejected").mkdir()
            _write_episode(source, 0, 3, accepted=False, reasons=["too_short: 3 item < 5"])
            _write_episode(source, 1, 3, accepted=False, reasons=["writer_overflow: 1 capture"])
            (source / "episode_index.json").write_text(
                json.dumps({"rejection_policy": {"min_items": 5, "max_projected_fraction": 1}}),
                encoding="utf-8",
            )

            output = root / "derived"
            manifest = reclassify_run(source, output, min_items=2)

            self.assertEqual(manifest["episodes_promoted"], 1)
            self.assertFalse(manifest["certified_true_30_fps"])
            promoted = json.loads((output / "episodes/episode_0000/data.json").read_text())
            retained = json.loads(
                (output / "episodes_rejected/episode_0001/data.json").read_text()
            )
            original = json.loads(
                (source / "episodes_rejected/episode_0000/data.json").read_text()
            )
            self.assertTrue(promoted["info"]["accepted"])
            self.assertEqual(promoted["info"]["rejection_reasons"], [])
            self.assertFalse(retained["info"]["accepted"])
            self.assertFalse(original["info"]["accepted"])
            self.assertEqual(json.loads((output / "episode_index.json").read_text())["episode_count_accepted"], 1)

    def test_refuses_to_overwrite_output(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            source = root / "source"
            (source / "episodes").mkdir(parents=True)
            (source / "episodes_rejected").mkdir()
            _write_episode(source, 0, 3, accepted=False, reasons=["too_short: 3 item < 5"])
            (source / "episode_index.json").write_text(
                json.dumps({"rejection_policy": {"min_items": 5}}), encoding="utf-8"
            )
            output = root / "derived"
            output.mkdir()
            with self.assertRaises(FileExistsError):
                reclassify_run(source, output, min_items=2)

    def test_reads_recovery_layout_and_records_incomplete_directory(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            recovery = root / "recovery"
            (recovery / "episodes_accepted").mkdir(parents=True)
            (recovery / "episodes_rejected").mkdir()
            _write_episode(recovery, 0, 3, accepted=False, reasons=["too_short: 3 item < 5"])
            source_episode = recovery / "episodes_rejected/episode_0000"
            source_episode.rename(recovery / "episodes_rejected/episode_0001")
            accepted_episode = recovery / "episodes_accepted/episode_0000"
            accepted_episode.mkdir()
            payload = json.loads(
                (recovery / "episodes_rejected/episode_0001/data.json").read_text()
            )
            payload["info"]["accepted"] = True
            payload["info"]["rejection_reasons"] = []
            for frame in payload["data"]:
                source_image = recovery / "episodes_rejected/episode_0001" / frame["colors"]["color_0"]
                destination_image = accepted_episode / frame["colors"]["color_0"]
                destination_image.parent.mkdir(exist_ok=True)
                destination_image.write_bytes(source_image.read_bytes())
            (accepted_episode / "data.json").write_text(json.dumps(payload), encoding="utf-8")
            (recovery / "episodes_accepted/episode_0002").mkdir()

            output = root / "derived"
            manifest = reclassify_run(recovery, output, min_items=2)

            self.assertEqual(manifest["episodes_accepted"], 2)
            self.assertEqual(manifest["episodes_promoted"], 1)
            self.assertFalse(manifest["source_episode_index_present"])
            self.assertEqual(len(manifest["incomplete_episode_directories"]), 1)


if __name__ == "__main__":
    unittest.main()
