"""Kiểm tra phần đọc profile cảnh dataset.

Không cần Isaac Sim: chỉ kiểm tra việc đọc và kiểm chứng cấu hình, tức phần
chạy trước khi simulator khởi động và là nơi một profile sai phải hỏng ngay.
"""

import json
import tempfile
import unittest
from pathlib import Path

from teleop.r1.dataset_scene import YCB_WITH_PHYSICS, load_dataset_scene_config


ROOT = Path(__file__).resolve().parents[2]
D001_CONFIG = (
    ROOT / "experiments/r1_dataset/quest3_sim_v1/D001/config/r1_d001_reach_point_dataset.json"
)
D002_CONFIG = (
    ROOT / "experiments/r1_dataset/quest3_sim_v1/D002/config/r1_d002_number_pointing.json"
)


def _write(payload: dict) -> Path:
    handle = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8")
    json.dump(payload, handle)
    handle.close()
    return Path(handle.name)


class DatasetSceneConfigTests(unittest.TestCase):
    def test_d001_profile_declares_a_table_two_objects_and_a_head_camera(self) -> None:
        config = load_dataset_scene_config(D001_CONFIG)
        self.assertIsNotNone(config.table)
        self.assertIsNotNone(config.head_camera)
        self.assertEqual(len(config.objects), 2)

    def test_table_surface_is_derived_from_centre_and_height_not_declared_twice(self) -> None:
        config = load_dataset_scene_config(D001_CONFIG)
        self.assertAlmostEqual(
            config.table.surface_z_m, config.table.center_m[2] + config.table.size_m[2] / 2.0
        )

    def test_declared_surface_matches_the_derived_one(self) -> None:
        # Profile ghi surface_z_m cho người đọc; nếu nó lệch khỏi hình học thì
        # một trong hai chỗ đã bị sửa mà quên chỗ kia.
        payload = json.loads(D001_CONFIG.read_text(encoding="utf-8"))
        config = load_dataset_scene_config(D001_CONFIG)
        self.assertAlmostEqual(
            float(payload["scene"]["table"]["surface_z_m"]), config.table.surface_z_m
        )

    def test_every_rigid_ycb_object_names_an_asset_that_has_a_physics_variant(self) -> None:
        config = load_dataset_scene_config(D001_CONFIG)
        for spec in config.objects:
            if spec["kind"] == "ycb_rigid":
                self.assertIn(spec["asset"], YCB_WITH_PHYSICS)

    def test_objects_are_declared_above_the_table_surface(self) -> None:
        config = load_dataset_scene_config(D001_CONFIG)
        for spec in config.objects:
            self.assertGreater(spec["position_m"][2], config.table.surface_z_m)

    def test_an_nvidia_table_asset_is_refused_with_the_reason(self) -> None:
        path = _write({"scene": {"table": {"kind": "usd_asset", "center_m": [0, 0, 0], "size_m": [1, 1, 1]}}})
        with self.assertRaises(ValueError) as ctx:
            load_dataset_scene_config(path)
        self.assertIn("collision", str(ctx.exception))

    def test_an_unknown_object_kind_is_refused(self) -> None:
        path = _write({"scene": {"objects": [{"name": "x", "kind": "teapot", "position_m": [0, 0, 1]}]}})
        with self.assertRaises(ValueError):
            load_dataset_scene_config(path)

    def test_a_profile_with_no_scene_section_loads_as_empty_rather_than_failing(self) -> None:
        # Profile T007 không có phần scene; đường teleop cũ phải không bị ảnh hưởng.
        path = _write({"schema_version": 3, "mode": "simulation_only"})
        config = load_dataset_scene_config(path)
        self.assertIsNone(config.table)
        self.assertIsNone(config.head_camera)
        self.assertEqual(config.objects, ())

    def test_d002_declares_an_operator_only_cue(self) -> None:
        config = load_dataset_scene_config(D002_CONFIG)
        self.assertEqual(config.experiment_id, "d002")
        self.assertIsNotNone(config.operator_cue)
        self.assertTrue(config.operator_cue.desktop_hud)
        self.assertTrue(config.operator_cue.head_view_overlay)
        self.assertIn("{n}", config.operator_cue.text_template)


if __name__ == "__main__":
    unittest.main()
