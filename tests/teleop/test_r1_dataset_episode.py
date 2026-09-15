"""Kiểm tra bộ ghi episode và độ trung thành với lược đồ của Unitree.

Không cần Isaac Sim. Camera được thay bằng một vật giả trả ảnh cố định, nên
phần được kiểm tra là bố cục thư mục, lược đồ JSON và cách tách state/action.
"""

import json
import re
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from teleop.r1.dataset_episode import (
    EpisodeConfig,
    EpisodeRecorder,
    RejectionPolicy,
    load_episode_config,
)


ROOT = Path(__file__).resolve().parents[2]
VENDOR_WRITER = ROOT / "third_party/xr_teleoperate/teleop/utils/episode_writer.py"
D001_CONFIG = (
    ROOT / "experiments/r1_dataset/quest3_sim_v1/D001/config/r1_d001_reach_point_dataset.json"
)
D002_CONFIG = (
    ROOT / "experiments/r1_dataset/quest3_sim_v1/D002/config/r1_d002_number_pointing.json"
)
JOINTS = [f"j{i}" for i in range(12)]


class _FakeCameraData:
    def __init__(self) -> None:
        import torch

        # Tensor thật, không phải mảng numpy: bộ ghi gọi .detach().cpu().numpy()
        # đúng như với `Camera` của Isaac, nên test phải đi qua cùng đường đó.
        self.output = {"rgb": torch.zeros((1, 4, 4, 4), dtype=torch.uint8)}


class _FakeCamera:
    def __init__(self) -> None:
        self.data = _FakeCameraData()

    def update(self, dt: float) -> None:  # noqa: D401 - khớp giao diện của Camera
        self.dt = dt


def _config(
    min_items: int = 1,
    max_projected: float = 0.20,
    queue_size: int = 64,
    min_measured_fps: float | None = None,
    max_measured_fps: float | None = None,
    require_unique_camera_frames: bool = False,
) -> EpisodeConfig:
    return EpisodeConfig(
        fps=30.0,
        image_width=4,
        image_height=4,
        joint_names={
            "left_arm": JOINTS[0:5],
            "right_arm": JOINTS[5:10],
            "body": JOINTS[10:12],
            "left_ee": [],
            "right_ee": [],
        },
        task_text={"goal": "g", "desc": "d", "steps": "s"},
        rejection=RejectionPolicy(
            max_projected_fraction=max_projected,
            min_items=min_items,
            min_measured_fps=min_measured_fps,
            max_measured_fps=max_measured_fps,
            require_unique_camera_frames=require_unique_camera_frames,
        ),
        writer_queue_size=queue_size,
    )


def _recorder(tmp: Path, **kwargs) -> EpisodeRecorder:
    return EpisodeRecorder(
        staging_dir=tmp, camera=_FakeCamera(), physics_dt_s=0.005,
        config=_config(**kwargs), controlled_joint_names=list(JOINTS),
    )


class EpisodeRecorderTests(unittest.TestCase):
    def test_slow_jpeg_write_does_not_block_add_item(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            writer_started = threading.Event()
            allow_writer = threading.Event()

            def slow_write(*_args, **_kwargs):
                writer_started.set()
                allow_writer.wait(timeout=2.0)
                return True

            with mock.patch("cv2.imwrite", side_effect=slow_write):
                started = time.monotonic()
                self.assertTrue(rec.add_item(0, 0.0, list(range(12)), list(range(12))))
                add_elapsed = time.monotonic() - started
                self.assertTrue(writer_started.wait(timeout=1.0))
                self.assertLess(add_elapsed, 0.1, "JPEG/disk IO không được chặn control thread")
                rec.end_episode(wait=False)
                allow_writer.set()
                out = tmp / "run"
                out.mkdir()
                stats = rec.finalize(out)

            self.assertEqual(stats["dataset_writer_written_frame_count"], 1)
            self.assertEqual(stats["dataset_writer_dropped_frame_count"], 0)
            self.assertTrue((tmp / "episode_0000/data.json").is_file())

    def test_bounded_queue_overflow_is_counted_and_rejects_the_episode(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp / "staging", queue_size=1)
            writer_started = threading.Event()
            allow_writer = threading.Event()

            def slow_write(*_args, **_kwargs):
                writer_started.set()
                allow_writer.wait(timeout=2.0)
                return True

            with mock.patch("cv2.imwrite", side_effect=slow_write):
                self.assertTrue(rec.add_item(0, 0.0, list(range(12)), list(range(12))))
                self.assertTrue(writer_started.wait(timeout=1.0))
                self.assertFalse(rec.add_item(1, 0.1, list(range(12)), list(range(12))))
                rec.end_episode(wait=False)
                allow_writer.set()
                out = tmp / "run"
                out.mkdir()
                stats = rec.finalize(out)

            self.assertEqual(stats["dataset_writer_dropped_frame_count"], 1)
            rejected = tmp / "staging.rejected/episode_0000/data.json"
            payload = json.loads(rejected.read_text(encoding="utf-8"))
            self.assertEqual(len(payload["data"]), 1)
            self.assertTrue(any("writer_overflow" in reason for reason in payload["info"]["rejection_reasons"]))

    def test_each_reset_starts_a_new_episode_directory(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            for episode in range(3):
                rec.begin_episode()
                for step in range(2):
                    rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)))
                rec.end_episode()
            names = sorted(p.name for p in tmp.iterdir())
            self.assertEqual(names, ["episode_0000", "episode_0001", "episode_0002"])

    def test_an_episode_with_no_items_leaves_no_directory_behind(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            rec.end_episode()
            self.assertEqual(list(tmp.iterdir()), [])

    def test_image_count_matches_item_count(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            for step in range(5):
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)))
            rec.end_episode()
            episode = tmp / "episode_0000"
            payload = json.loads((episode / "data.json").read_text(encoding="utf-8"))
            self.assertEqual(len(payload["data"]), 5)
            self.assertEqual(len(list((episode / "colors").glob("*.jpg"))), 5)

    def test_states_and_actions_are_split_by_the_declared_joint_groups(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            state = [float(i) for i in range(12)]
            action = [float(i) + 0.5 for i in range(12)]
            rec.add_item(0, 0.0, state, action)
            rec.end_episode()
            item = json.loads((tmp / "episode_0000/data.json").read_text(encoding="utf-8"))["data"][0]
            self.assertEqual(item["states"]["left_arm"]["qpos"], state[0:5])
            self.assertEqual(item["states"]["right_arm"]["qpos"], state[5:10])
            self.assertEqual(item["states"]["body"]["qpos"], state[10:12])
            self.assertEqual(item["actions"]["right_arm"]["qpos"], action[5:10])

    def test_an_absent_end_effector_is_an_empty_list_not_a_row_of_zeros(self) -> None:
        # `[]` nghĩa là 'không đo'; `[0.0]` nghĩa là 'đo được và bằng không'.
        # R1 không có bàn tay, nên nhầm hai thứ này là bịa ra dữ liệu.
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            rec.add_item(0, 0.0, list(range(12)), list(range(12)))
            rec.end_episode()
            item = json.loads((tmp / "episode_0000/data.json").read_text(encoding="utf-8"))["data"][0]
            for side in ("left_ee", "right_ee"):
                self.assertEqual(item["states"][side]["qpos"], [])
                self.assertEqual(item["actions"][side]["qpos"], [])

    def test_run_level_index_records_every_frame_across_episodes(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            out = tmp / "run"
            out.mkdir()
            rec = _recorder(tmp / "staging")
            for _ in range(2):
                rec.begin_episode()
                for step in range(3):
                    rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)))
                rec.end_episode()
            summary = rec.finalize(out)
            self.assertEqual(summary["episode_count_accepted"], 2)
            self.assertEqual(summary["episode_count_rejected"], 0)
            self.assertEqual(summary["episode_item_counts"], [3, 3])
            index = json.loads((out / "episode_index.json").read_text(encoding="utf-8"))
            self.assertEqual(len(index["frames"]), 6)


class StagedEpisodeStateTests(unittest.TestCase):
    """Trạng thái mà vòng điều khiển dùng để quyết định có bốc số lại hay không.

    Nhả cò phải rồi bóp cò trái là HAI sự kiện nhưng chỉ mở MỘT episode. Nếu cả
    hai đều bốc số thì người vận hành vừa đọc xong mục tiêu đã thấy nó đổi — lỗi
    quan sát được trong phiên d002_20260908T020005Z, nơi cặp lời nhắc cách nhau
    0.17 giây.
    """

    def test_a_freshly_opened_episode_is_neither_between_nor_populated(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            rec = _recorder(Path(raw))
            rec.begin_episode({"target_digit": 3})
            self.assertFalse(rec.is_between_episodes, "episode đã mở")
            self.assertFalse(rec.has_items, "nhưng chưa có dữ liệu nào")

    def test_state_flips_only_after_the_first_item(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            rec = _recorder(Path(raw))
            rec.begin_episode({"target_digit": 3})
            rec.add_item(0, 0.0, list(range(12)), list(range(12)))
            self.assertTrue(rec.has_items)

    def test_closing_an_untouched_episode_returns_to_between(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            rec = _recorder(Path(raw))
            rec.begin_episode({"target_digit": 3})
            rec.end_episode()
            self.assertTrue(rec.is_between_episodes)


class RejectionPolicyTests(unittest.TestCase):
    def test_episode_below_the_measured_fps_gate_is_preserved_as_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp / "staging", min_measured_fps=27.0)
            rec.begin_episode()
            for step in range(10):
                rec.add_item(
                    step,
                    step / 8.0,
                    list(range(12)),
                    list(range(12)),
                    source_camera_frame=step + 1,
                )
            rec.end_episode()
            summary = rec.episode_summaries[0]
            self.assertFalse(summary["accepted"])
            self.assertAlmostEqual(summary["measured_fps"], 8.0)
            self.assertTrue(
                any("dataset_fps_below_gate" in reason for reason in summary["rejection_reasons"])
            )

    def test_duplicate_source_camera_frame_rejects_a_strict_episode(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp / "staging", require_unique_camera_frames=True)
            rec.begin_episode()
            for step, source_frame in enumerate((1, 1, 2)):
                rec.add_item(
                    step,
                    step / 30.0,
                    list(range(12)),
                    list(range(12)),
                    source_camera_frame=source_frame,
                )
            rec.end_episode()
            summary = rec.episode_summaries[0]
            self.assertEqual(summary["duplicate_camera_frame_count"], 1)
            self.assertFalse(summary["accepted"])
            self.assertTrue(
                any("duplicate_camera_frames" in reason for reason in summary["rejection_reasons"])
            )

    def test_a_short_episode_is_moved_aside_with_its_reason_not_deleted(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            staging = tmp / "staging"
            rec = _recorder(staging, min_items=5)
            rec.begin_episode()
            for step in range(2):
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)))
            rec.end_episode()
            rejected = tmp / "staging.rejected/episode_0000"
            self.assertTrue(rejected.is_dir(), "episode bị loại phải được giữ lại, không xoá")
            payload = json.loads((rejected / "data.json").read_text(encoding="utf-8"))
            self.assertFalse(payload["info"]["accepted"])
            self.assertTrue(any("too_short" in r for r in payload["info"]["rejection_reasons"]))
            self.assertEqual(len(payload["data"]), 2, "dữ liệu của episode bị loại vẫn còn nguyên")

    def test_an_episode_mostly_solved_by_projection_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp / "staging", min_items=1, max_projected=0.20)
            rec.begin_episode()
            for step in range(10):
                kind = "projected" if step < 5 else "exact"
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)), kind)
            rec.end_episode()
            summary = rec.episode_summaries[0]
            self.assertFalse(summary["accepted"])
            self.assertEqual(summary["projected_item_count"], 5)
            self.assertTrue(any("mostly_projected" in r for r in summary["rejection_reasons"]))

    def test_an_episode_just_under_the_projection_threshold_is_kept(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            staging = tmp / "staging"
            rec = _recorder(staging, min_items=1, max_projected=0.20)
            rec.begin_episode()
            for step in range(10):
                kind = "projected" if step < 2 else "exact"
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)), kind)
            rec.end_episode()
            self.assertTrue(rec.episode_summaries[0]["accepted"])
            self.assertTrue((staging / "episode_0000").is_dir())

    def test_projection_can_be_diagnostic_without_rejecting_virtual_joint_data(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp / "staging", min_items=1, max_projected=1.0)
            rec.begin_episode()
            for step in range(4):
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)), "projected")
            rec.end_episode()
            self.assertTrue(rec.episode_summaries[0]["accepted"])
            self.assertEqual(rec.episode_summaries[0]["projected_item_count"], 4)

    def test_dynamic_prompt_is_the_vla_goal_for_the_same_episode(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp, max_projected=1.0)
            rec.begin_episode({"target_digit": 7, "prompt": "Touch number 7."})
            rec.add_item(0, 0.0, list(range(12)), list(range(12)), "projected")
            rec.end_episode()
            payload = json.loads((tmp / "episode_0000/data.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["text"]["goal"], "Touch number 7.")
            self.assertEqual(payload["info"]["task"]["target_digit"], 7)

    def test_the_run_index_counts_rejections_by_reason(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            out = tmp / "run"
            out.mkdir()
            rec = _recorder(tmp / "staging", min_items=5)
            rec.begin_episode()
            for step in range(6):
                rec.add_item(step, 0.1 * step, list(range(12)), list(range(12)))
            rec.end_episode()
            rec.begin_episode()
            rec.add_item(0, 0.0, list(range(12)), list(range(12)))
            rec.end_episode()
            result = rec.finalize(out)
            self.assertEqual(result["episode_count_accepted"], 1)
            self.assertEqual(result["episode_count_rejected"], 1)
            self.assertEqual(result["episode_rejection_reason_counts"], {"too_short": 1})
            index = json.loads((out / "episode_index.json").read_text(encoding="utf-8"))
            self.assertEqual(len(index["frames"]), 6, "chỉ mục chỉ trỏ tới ảnh của episode được giữ")

    def test_the_policy_is_read_from_the_d001_profile(self) -> None:
        payload = json.loads(D001_CONFIG.read_text(encoding="utf-8"))
        config = load_episode_config(payload)
        declared = payload["episode_rejection"]
        self.assertEqual(config.rejection.max_projected_fraction, declared["max_projected_fraction"])
        self.assertEqual(config.rejection.min_items, declared["min_items"])


class VendorSchemaCompatibilityTests(unittest.TestCase):
    """Định dạng là thứ được tái sử dụng, nên nó phải không trôi khỏi vendor."""

    def test_info_block_carries_the_same_keys_the_vendor_writer_declares(self) -> None:
        source = VENDOR_WRITER.read_text(encoding="utf-8")
        declared = set(re.findall(r'"(version|date|author|image|depth|audio|joint_names|tactile_names|sim_state)"', source))
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            rec.add_item(0, 0.0, list(range(12)), list(range(12)))
            rec.end_episode()
            info = json.loads((tmp / "episode_0000/data.json").read_text(encoding="utf-8"))["info"]
        self.assertTrue(declared.issubset(set(info)), f"thiếu: {declared - set(info)}")

    def test_item_keys_match_the_vendor_add_item_signature(self) -> None:
        source = VENDOR_WRITER.read_text(encoding="utf-8")
        self.assertIn("'idx': self.item_id", source)
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            rec = _recorder(tmp)
            rec.begin_episode()
            rec.add_item(0, 0.0, list(range(12)), list(range(12)))
            rec.end_episode()
            item = json.loads((tmp / "episode_0000/data.json").read_text(encoding="utf-8"))["data"][0]
        for key in ("idx", "colors", "depths", "states", "actions", "tactiles", "audios", "sim_state"):
            self.assertIn(key, item)


class EpisodeConfigTests(unittest.TestCase):
    def test_d001_profile_yields_a_usable_episode_config(self) -> None:
        payload = json.loads(D001_CONFIG.read_text(encoding="utf-8"))
        config = load_episode_config(payload)
        self.assertIsNotNone(config)
        self.assertEqual(len(config.joint_names["right_arm"]), 5)
        self.assertEqual(config.joint_names["left_ee"], [])
        self.assertTrue(config.task_text["goal"])

    def test_d002_keeps_projected_virtual_joint_episodes(self) -> None:
        payload = json.loads(D002_CONFIG.read_text(encoding="utf-8"))
        config = load_episode_config(payload)
        self.assertEqual(config.rejection.max_projected_fraction, 1.0)
        self.assertEqual(config.rejection.min_measured_fps, 27.0)
        self.assertEqual(config.rejection.max_measured_fps, 33.0)
        self.assertTrue(config.rejection.require_unique_camera_frames)
        self.assertEqual(config.rejection.fps_gate_warmup_frames, 60)

    def test_a_profile_without_a_record_section_yields_no_episode_config(self) -> None:
        self.assertIsNone(load_episode_config({"schema_version": 3}))


if __name__ == "__main__":
    unittest.main()
