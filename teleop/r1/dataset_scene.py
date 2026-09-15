"""Vật thể trong cảnh và camera đầu robot cho việc thu dataset.

Đường teleop T007 chạy trong một thế giới trống: mặt đất, một cái đèn, robot.
Đủ để nghiên cứu bộ giải, nhưng một dataset học bắt chước cần hai thứ nữa —
cái gì đó để tương tác, và một con mắt gắn trên robot để quan sát nó.

Module này giữ cả hai, tách khỏi `run_r1_quest3_live.py`, vì chúng là mối quan
tâm của việc thu dữ liệu chứ không phải của vòng điều khiển. Không có gì ở đây
chạy trừ khi người gọi truyền vào một profile cảnh; đường T007 mặc định không
đổi một chút nào.

Mọi import của Isaac Lab đều nằm trong thân hàm: module này bị import trước khi
`AppLauncher` khởi động, lúc đó `isaaclab.sim` chưa dùng được.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# Trong kho Isaac 5.1 chỉ bốn vật YCB có biến thể đã gắn sẵn physics. Số còn lại
# chỉ có hình học, và gắn RigidBodyAPI vào lúc spawn không đáng tin cho mọi mesh.
YCB_WITH_PHYSICS = frozenset(
    {"003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "006_mustard_bottle"}
)

REPO_ROOT = Path(__file__).resolve().parents[2]

OBJECT_KINDS = frozenset(
    {"visual_sphere", "ycb_rigid", "ycb_static", "usd_rigid", "usd_static"}
)

# Môi trường chuẩn NVIDIA phát hành kèm Isaac 5.1. Danh sách lấy bằng cách duyệt
# bucket asset rồi kiểm từng đường dẫn bằng một yêu cầu HEAD — tất cả đều trả
# 200. `Modular_Warehouse` có trong danh mục nhưng file gốc trả 404, nên không
# được liệt ở đây.
ENVIRONMENTS = {
    "simple_room": "Environments/Simple_Room/simple_room.usd",
    "office": "Environments/Office/office.usd",
    "hospital": "Environments/Hospital/hospital.usd",
    "warehouse": "Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_full": "Environments/Simple_Warehouse/full_warehouse.usd",
    "warehouse_shelves": "Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "warehouse_forklifts": "Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "grid": "Environments/Grid/default_environment.usd",
}


@dataclass(frozen=True)
class HeadCameraConfig:
    """Camera gắn trên link đầu robot, tức là 'mắt' của nó."""

    prim_path: str
    width: int
    height: int
    focal_length_mm: float
    horizontal_aperture_mm: float
    focus_distance: float
    clipping_range_m: tuple[float, float]
    offset_pos_m: tuple[float, float, float]
    offset_rot_wxyz: tuple[float, float, float, float]
    offset_convention: str
    update_period_s: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Kích thước ảnh camera đầu phải dương.")
        if self.focal_length_mm <= 0.0 or self.horizontal_aperture_mm <= 0.0:
            raise ValueError("Tiêu cự và khẩu độ ngang phải dương.")
        if len(self.offset_rot_wxyz) != 4:
            raise ValueError("offset_rot_wxyz phải là quaternion 4 phần tử (w, x, y, z).")
        if self.offset_convention not in ("ros", "world", "opengl"):
            raise ValueError(f"Quy ước camera không hợp lệ: {self.offset_convention}")


@dataclass(frozen=True)
class TableConfig:
    """Khối hộp kinematic đóng vai cái bàn."""

    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]

    @property
    def surface_z_m(self) -> float:
        return self.center_m[2] + self.size_m[2] / 2.0

    def validate(self) -> None:
        if any(v <= 0.0 for v in self.size_m):
            raise ValueError("Kích thước bàn phải dương.")


@dataclass(frozen=True)
class EnvironmentConfig:
    """Khung cảnh nền. Môi trường mang sẵn sàn và đèn của nó."""

    name: str
    usd_relpath: str
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    fill_light_intensity: float = 1000.0


@dataclass(frozen=True)
class MarkerPoolConfig:
    """Kho tấm số để bốc ra mỗi episode.

    Toàn bộ tấm trong kho được dựng một lần lúc khởi động; mỗi episode chỉ đổi
    vị trí và độ hiển thị của chúng. Dựng lại prim giữa chừng vừa chậm vừa dễ
    làm hỏng trạng thái physics đang chạy.
    """

    digits: tuple[int, ...]
    usd_path_template: str
    slot_x_m: float
    slot_y_m: tuple[float, ...]
    slot_z_m: float
    slot_y_jitter_m: float
    parked_position_m: tuple[float, float, float]
    prompt_templates: tuple[str, ...]
    seed: int | None = None

    def validate(self) -> None:
        if len(self.digits) < len(self.slot_y_m):
            raise ValueError(
                f"Kho có {len(self.digits)} chữ số nhưng cần {len(self.slot_y_m)} ô."
            )
        if not self.prompt_templates:
            raise ValueError("marker_pool phải khai ít nhất một mẫu câu lệnh.")
        for template in self.prompt_templates:
            if "{n}" not in template:
                raise ValueError(f"Mẫu câu lệnh thiếu chỗ điền {{n}}: {template!r}")

    def prim_path(self, digit: int) -> str:
        return f"/World/MarkerPool/marker_{digit:02d}"

    def usd_path(self, digit: int) -> Path:
        relative = Path(self.usd_path_template.format(digit=digit))
        return relative if relative.is_absolute() else REPO_ROOT / relative


@dataclass(frozen=True)
class OperatorCueConfig:
    """Lời nhắc riêng cho người thu, nằm ngoài sensor image của dataset."""

    enabled: bool = True
    desktop_hud: bool = True
    head_view_overlay: bool = True
    text_template: str = "MỤC TIÊU: ĐẶT TAY LÊN SỐ {n}"

    def validate(self) -> None:
        if "{n}" not in self.text_template:
            raise ValueError("operator_cue.text_template phải chứa {n}.")


@dataclass(frozen=True)
class DatasetSceneConfig:
    """Phần cảnh và camera đọc ra từ một profile dataset."""

    source_path: Path
    table: TableConfig | None
    experiment_id: str | None = None
    objects: tuple[dict[str, Any], ...] = ()
    head_camera: HeadCameraConfig | None = None
    episode: Any = None
    """Phần bản ghi episode, `None` nếu profile không khai `record`."""

    environment: EnvironmentConfig | None = None
    """Khung cảnh nền. `None` giữ nguyên mặt đất phẳng và đèn dome như T007."""

    marker_pool: MarkerPoolConfig | None = None
    """Kho tấm số cho task VLA. `None` giữ cảnh tĩnh như D001."""

    operator_cue: OperatorCueConfig | None = None
    """HUD cho người vận hành; không phải một prim và không đi vào ảnh sensor."""

    def validate(self) -> None:
        if self.table is not None:
            self.table.validate()
        if self.head_camera is not None:
            self.head_camera.validate()
        if self.marker_pool is not None:
            self.marker_pool.validate()
        if self.operator_cue is not None:
            self.operator_cue.validate()
        for spec in self.objects:
            kind = str(spec.get("kind", ""))
            if kind not in OBJECT_KINDS:
                raise ValueError(f"Loại vật thể không hỗ trợ: {kind!r}; nhận {sorted(OBJECT_KINDS)}")
            if len(spec.get("position_m") or ()) != 3:
                raise ValueError(f"Vật {spec.get('name')!r} thiếu position_m gồm 3 số.")


def _tuple3(values: Any, field_name: str) -> tuple[float, float, float]:
    items = list(values or ())
    if len(items) != 3:
        raise ValueError(f"{field_name} phải có đúng 3 số.")
    return (float(items[0]), float(items[1]), float(items[2]))


def load_dataset_scene_config(path: Path) -> DatasetSceneConfig:
    """Đọc profile dataset và lấy ra phần cảnh + camera.

    Profile còn chứa nhãn task, chính sách loại episode, tiêu chí hợp lệ… Những
    thứ đó không thuộc về simulator nên bị bỏ qua ở đây một cách có chủ ý.
    """

    resolved = path.expanduser().resolve()
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Không đọc được profile cảnh {resolved}: {exc}") from exc

    scene = payload.get("scene") or {}
    table_payload = scene.get("table")
    table = None
    if table_payload is not None:
        if str(table_payload.get("kind", "primitive_box")) != "primitive_box":
            raise ValueError(
                "Chỉ hỗ trợ bàn kiểu 'primitive_box'. Hai asset bàn của NVIDIA đã bị loại: "
                "SeattleLabTable không có collision, PackingTable rộng 2.474 m nên vượt tầm với của R1."
            )
        table = TableConfig(
            center_m=_tuple3(table_payload.get("center_m"), "table.center_m"),
            size_m=_tuple3(table_payload.get("size_m"), "table.size_m"),
        )

    environment_payload = scene.get("environment")
    environment = None
    if environment_payload:
        name = str(environment_payload.get("name") or "")
        relpath = environment_payload.get("usd_relpath") or ENVIRONMENTS.get(name)
        if not relpath:
            raise ValueError(
                f"Môi trường {name!r} không có trong danh sách; chọn một trong "
                f"{sorted(ENVIRONMENTS)} hoặc khai usd_relpath."
            )
        environment = EnvironmentConfig(
            name=name or "custom",
            usd_relpath=str(relpath),
            position_m=_tuple3(environment_payload.get("position_m") or (0.0, 0.0, 0.0), "environment.position_m"),
            fill_light_intensity=float(environment_payload.get("fill_light_intensity", 1000.0)),
        )

    pool_payload = scene.get("marker_pool")
    marker_pool = None
    if pool_payload:
        randomization = payload.get("randomization") or {}
        marker_pool = MarkerPoolConfig(
            digits=tuple(int(v) for v in (pool_payload.get("digits") or ())),
            usd_path_template=str(pool_payload.get("usd_path_template") or ""),
            slot_x_m=float(pool_payload.get("slot_x_m", 0.37)),
            slot_y_m=tuple(float(v) for v in (pool_payload.get("slot_y_m") or ())),
            slot_z_m=float(pool_payload.get("slot_z_m", 0.885)),
            slot_y_jitter_m=float(randomization.get("slot_y_jitter_m", 0.0)),
            parked_position_m=_tuple3(
                pool_payload.get("parked_position_m") or (0.0, 0.0, -5.0), "parked_position_m"
            ),
            prompt_templates=tuple(randomization.get("prompt_templates") or ()),
            seed=randomization.get("seed"),
        )

    cue_payload = payload.get("operator_cue")
    operator_cue = None
    if cue_payload is not None:
        operator_cue = OperatorCueConfig(
            enabled=bool(cue_payload.get("enabled", True)),
            desktop_hud=bool(cue_payload.get("desktop_hud", True)),
            head_view_overlay=bool(cue_payload.get("head_view_overlay", True)),
            text_template=str(
                cue_payload.get("text_template") or "MỤC TIÊU: ĐẶT TAY LÊN SỐ {n}"
            ),
        )

    camera_payload = (payload.get("cameras") or {}).get("head_camera")
    head_camera = None
    if camera_payload is not None:
        clipping = list(camera_payload.get("clipping_range_m") or (0.1, 1.0e5))
        rotation = list(camera_payload.get("offset_rot_wxyz") or (0.5, -0.5, 0.5, -0.5))
        head_camera = HeadCameraConfig(
            prim_path=str(camera_payload.get("prim_path") or "/World/Robot/head_yaw_link/head_camera"),
            width=int(camera_payload.get("width", 640)),
            height=int(camera_payload.get("height", 480)),
            focal_length_mm=float(camera_payload.get("focal_length_mm", 7.6)),
            horizontal_aperture_mm=float(camera_payload.get("horizontal_aperture_mm", 20.0)),
            focus_distance=float(camera_payload.get("focus_distance", 400.0)),
            clipping_range_m=(float(clipping[0]), float(clipping[1])),
            offset_pos_m=_tuple3(camera_payload.get("offset_pos_m") or (0.06, 0.0, 0.03), "offset_pos_m"),
            offset_rot_wxyz=(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])),
            offset_convention=str(camera_payload.get("offset_convention", "ros")),
            update_period_s=float(camera_payload.get("update_period_s", 0.02)),
        )

    from teleop.r1.dataset_episode import load_episode_config

    config = DatasetSceneConfig(
        source_path=resolved,
        table=table,
        experiment_id=(str(payload.get("experiment_id")) if payload.get("experiment_id") else None),
        objects=tuple(scene.get("objects") or ()),
        head_camera=head_camera,
        episode=load_episode_config(payload),
        environment=environment,
        marker_pool=marker_pool,
        operator_cue=operator_cue,
    )
    config.validate()
    return config


def spawn_scene_props(config: DatasetSceneConfig) -> dict[str, Any]:
    """Dựng bàn và vật thể. Trả về mô tả những gì đã dựng, để ghi vào bằng chứng.

    Vật có physics được trả về dưới khoá `rigid_objects` vì vòng lặp điều khiển
    phải gọi `write_data_to_sim()` / `update()` cho chúng; thiếu bước đó thì vị
    trí đọc ra là giá trị khởi tạo chứ không phải vị trí thật.
    """

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

    spawned: list[dict[str, Any]] = []
    rigid_objects: list[Any] = []

    if config.environment is not None:
        env_cfg = sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/{config.environment.usd_relpath}"
        )
        env_cfg.func("/World/Environment", env_cfg, translation=config.environment.position_m)
        # Môi trường chuẩn có đèn riêng nhưng thường tối với camera mặc định.
        fill = sim_utils.DomeLightCfg(
            intensity=config.environment.fill_light_intensity, color=(0.9, 0.9, 0.9)
        )
        fill.func("/World/EnvironmentFillLight", fill)
        spawned.append(
            {
                "name": config.environment.name,
                "kind": "environment",
                "prim_path": "/World/Environment",
                "usd_relpath": config.environment.usd_relpath,
                "position_m": list(config.environment.position_m),
            }
        )

    if config.table is not None:
        table_cfg = sim_utils.CuboidCfg(
            size=config.table.size_m,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.35, 0.25)),
        )
        table_cfg.func("/World/DatasetTable", table_cfg, translation=config.table.center_m)
        spawned.append(
            {
                "name": "table",
                "kind": "primitive_box",
                "prim_path": "/World/DatasetTable",
                "center_m": list(config.table.center_m),
                "size_m": list(config.table.size_m),
                "surface_z_m": config.table.surface_z_m,
                "has_collision": True,
            }
        )

    if config.marker_pool is not None:
        pool = config.marker_pool
        for digit in pool.digits:
            usd = pool.usd_path(digit)
            if not usd.is_file():
                raise ValueError(
                    f"Tấm số {digit} không có file: {usd}. Sinh bằng "
                    "experiments/r1_dataset/quest3_sim_v1/tools/make_number_markers.py"
                )
            marker_cfg = sim_utils.UsdFileCfg(usd_path=str(usd))
            marker_cfg.func(pool.prim_path(digit), marker_cfg, translation=pool.parked_position_m)
        spawned.append(
            {
                "name": "marker_pool",
                "kind": "marker_pool",
                "digits": list(pool.digits),
                "slot_count": len(pool.slot_y_m),
                "prim_paths": {str(d): pool.prim_path(d) for d in pool.digits},
            }
        )

    for index, spec in enumerate(config.objects):
        kind = str(spec["kind"])
        name = str(spec.get("name") or f"object_{index}")
        position = _tuple3(spec.get("position_m"), f"objects[{index}].position_m")
        prim_path = str(spec.get("prim_path") or f"/World/DatasetObject_{index}")
        record: dict[str, Any] = {
            "name": name,
            "kind": kind,
            "prim_path": prim_path,
            "position_m": list(position),
        }

        if kind == "visual_sphere":
            sphere = sim_utils.SphereCfg(
                radius=float(spec.get("radius_m", 0.05)),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(spec.get("diffuse_color") or (0.9, 0.1, 0.1))
                ),
            )
            sphere.func(prim_path, sphere, translation=position)
            record["has_collision"] = False
            record["radius_m"] = float(spec.get("radius_m", 0.05))
        elif kind in ("usd_rigid", "usd_static"):
            # `usd_relpath` trỏ vào kho asset NVIDIA; `usd_path` trỏ vào file
            # trong repo này, ví dụ các tấm số sinh bằng make_number_markers.py.
            relpath = str(spec.get("usd_relpath") or "")
            local = str(spec.get("usd_path") or "")
            if not relpath and not local:
                raise ValueError(
                    f"Vật {name!r} kiểu {kind} phải khai usd_relpath (kho NVIDIA) "
                    "hoặc usd_path (file trong repo)."
                )
            if relpath and local:
                raise ValueError(f"Vật {name!r} khai cả usd_relpath lẫn usd_path; chọn một.")
            if local:
                resolved_local = Path(local)
                if not resolved_local.is_absolute():
                    resolved_local = REPO_ROOT / resolved_local
                if not resolved_local.is_file():
                    raise ValueError(f"Vật {name!r} trỏ tới file không tồn tại: {resolved_local}")
                usd_path = str(resolved_local)
            else:
                usd_path = f"{ISAAC_NUCLEUS_DIR}/{relpath}"
            if kind == "usd_rigid":
                rigid_objects.append(
                    RigidObject(
                        RigidObjectCfg(
                            prim_path=prim_path,
                            spawn=sim_utils.UsdFileCfg(
                                usd_path=usd_path,
                                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                                mass_props=sim_utils.MassPropertiesCfg(
                                    mass=float(spec.get("mass_kg", 0.2))
                                ),
                                collision_props=sim_utils.CollisionPropertiesCfg(),
                            ),
                            init_state=RigidObjectCfg.InitialStateCfg(pos=position),
                        )
                    )
                )
                record["has_collision"] = True
            else:
                static_cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
                static_cfg.func(prim_path, static_cfg, translation=position)
                record["has_collision"] = False
            record["usd_relpath"] = relpath or None
            record["usd_path"] = local or None
        else:
            asset = str(spec.get("asset") or name)
            if kind == "ycb_rigid":
                if asset not in YCB_WITH_PHYSICS:
                    raise ValueError(
                        f"Vật YCB {asset!r} không có biến thể physics trong kho Isaac; "
                        f"dùng kind='ycb_static' hoặc chọn một trong {sorted(YCB_WITH_PHYSICS)}."
                    )
                rigid_objects.append(
                    RigidObject(
                        RigidObjectCfg(
                            prim_path=prim_path,
                            spawn=sim_utils.UsdFileCfg(
                                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/{asset}.usd"
                            ),
                            init_state=RigidObjectCfg.InitialStateCfg(pos=position),
                        )
                    )
                )
                record["has_collision"] = True
            else:
                usd_cfg = sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned/{asset}.usd"
                )
                usd_cfg.func(prim_path, usd_cfg, translation=position)
                record["has_collision"] = False
            record["asset"] = asset
        spawned.append(record)

    return {"spawned": spawned, "rigid_objects": rigid_objects}


def build_head_camera(config: HeadCameraConfig) -> Any:
    """Tạo camera gắn dưới prim của robot, nên nó tự đi theo khớp đầu."""

    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    return Camera(
        CameraCfg(
            prim_path=config.prim_path,
            update_period=config.update_period_s,
            height=config.height,
            width=config.width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=config.focal_length_mm,
                focus_distance=config.focus_distance,
                horizontal_aperture=config.horizontal_aperture_mm,
                clipping_range=config.clipping_range_m,
            ),
            offset=CameraCfg.OffsetCfg(
                pos=config.offset_pos_m,
                rot=config.offset_rot_wxyz,
                convention=config.offset_convention,
            ),
        )
    )


class HeadCameraRecorder:
    """Ghi ảnh camera đầu ra đĩa và giữ một manifest khớp ảnh với bước điều khiển.

    Bố cục thư mục theo `EpisodeWriter` của Unitree (`colors/<idx>_color_0.jpg`)
    để Giai đoạn 4 không phải đổi định dạng lần nữa.
    """

    def __init__(self, output_dir: Path, camera: Any, physics_dt_s: float) -> None:
        self.colors_dir = output_dir / "colors"
        self.colors_dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        self.physics_dt_s = physics_dt_s
        self.frames: list[dict[str, Any]] = []

    def capture(self, control_step: int, elapsed_s: float) -> None:
        """Lưu một khung. Người gọi phải đã gọi `sim.render()` ở bước này."""

        import imageio.v2 as imageio

        self.camera.update(self.physics_dt_s)
        rgb = self.camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
        index = len(self.frames)
        relative = f"colors/{index:06d}_color_0.jpg"
        imageio.imwrite(self.colors_dir.parent / relative, rgb.astype("uint8"))
        self.frames.append(
            {"idx": index, "control_step": control_step, "elapsed_s": elapsed_s, "color_0": relative}
        )

    def write_manifest(self, output_dir: Path) -> dict[str, Any]:
        summary = {
            "frame_count": len(self.frames),
            "colors_dir": "colors",
            "frames": self.frames,
        }
        (output_dir / "head_camera_frames.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        return {"head_camera_frame_count": len(self.frames)}


__all__ = [
    "ENVIRONMENTS",
    "MarkerPoolConfig",
    "DatasetSceneConfig",
    "EnvironmentConfig",
    "HeadCameraConfig",
    "HeadCameraRecorder",
    "TableConfig",
    "YCB_WITH_PHYSICS",
    "build_head_camera",
    "load_dataset_scene_config",
    "OperatorCueConfig",
    "spawn_scene_props",
]
