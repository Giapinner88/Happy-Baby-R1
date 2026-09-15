#!/usr/bin/env python3
"""Dựng thử một khung cảnh quanh R1 và xem nó trong Isaac Sim.

Công cụ để làm quen và để thử bố cục trước khi chốt scene cho D001. Nó KHÔNG
teleop, KHÔNG ghi dữ liệu, KHÔNG chạm vào bất cứ thứ gì trong `runs/`. Chỉ dựng
scene, chạy physics và vẽ ra màn hình.

    conda run --no-capture-output -n unitree_sim_env python \
        experiments/r1_dataset/quest3_sim_v1/tools/preview_scene.py \
        --environment simple_room --table --ycb 006_mustard_bottle,025_mug

Asset lấy từ kho chuẩn của NVIDIA (Isaac 5.1) qua mạng ở lần chạy đầu, sau đó
Isaac tự cache lại. Dùng --environment none để chạy hoàn toàn offline.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
# `training/isaaclab/robot.py` phân giải đường dẫn USD của R1 từ biến này; nếu
# không đặt, nó đoán sai gốc workspace khi script nằm sâu như thế này.
os.environ.setdefault("HAPPY_BABY_R1_ROOT", str(ROOT))

# Tên ngắn -> đường dẫn dưới {ISAAC_NUCLEUS_DIR}. Danh sách này là các môi trường
# chuẩn NVIDIA phát hành kèm Isaac Sim; liệt kê bằng cách duyệt bucket S3.
ENVIRONMENTS = {
    "none": None,
    "simple_room": "Environments/Simple_Room/simple_room.usd",
    "office": "Environments/Office/office.usd",
    "hospital": "Environments/Hospital/hospital.usd",
    "warehouse": "Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_full": "Environments/Simple_Warehouse/full_warehouse.usd",
    "grid": "Environments/Grid/default_environment.usd",
}

# Trong kho Isaac 5.1 chỉ 4 vật YCB có sẵn biến thể đã gắn physics. Số còn lại
# nằm ở Axis_Aligned (chỉ hình học), nên script tự gắn rigid body + collision.
YCB_WITH_PHYSICS = {
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
}

# Đo ngày 2026-09-04 bằng cách thả bi thử và lấy bounding box:
#   SeattleLabTable  KHÔNG có collision (bi rơi xuyên qua), hình khối trải
#                    x[-0.865,0.554] y[-0.460,1.200] z[-1.040,0.715] quanh gốc
#                    của nó, tức hơn nửa bàn nằm dưới sàn và mặt bằng của nó
#                    chồng lên chỗ robot đứng.
#   PackingTable     CÓ collision, mặt bàn ở z=0.994 so với gốc, nhưng rộng
#                    2.474 m nên muốn tránh robot thì phải đẩy ra xa 1.59 m —
#                    vượt tầm với của R1.
# Vì vậy mặc định là một khối hộp tự dựng: đúng kích thước, chắc chắn có
# collision, không phải tải gì.
TABLES = {
    "lab": "Props/Mounts/SeattleLabTable/table_instanceable.usd",
    "packing": "Props/PackingTable/packing_table.usd",
}

TABLES_WITHOUT_COLLISION = {"lab"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--dataset-scene-config",
        type=Path,
        help=(
            "Dựng đúng cảnh mà một profile dataset khai, thay vì các cờ nhanh bên dưới. "
            "Dùng cái này để thứ bạn xem trước GIỐNG HỆT thứ sẽ được thu."
        ),
    )
    parser.add_argument("--environment", choices=sorted(ENVIRONMENTS), default="none")
    parser.add_argument("--environment-pos", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--table", action="store_true", help="Thêm một cái bàn trước mặt robot.")
    parser.add_argument("--table-kind", choices=["box"] + sorted(TABLES), default="box")
    parser.add_argument(
        "--table-pos", type=float, nargs=3, default=(0.70, 0.0, 0.40),
        help="Tâm bàn. Với 'box' đây là tâm khối hộp, nên mặt bàn = z + table-size[2]/2.",
    )
    parser.add_argument(
        "--table-size", type=float, nargs=3, default=(0.70, 1.20, 0.80),
        help="Chỉ dùng cho --table-kind box: dài x rộng x cao, mét.",
    )
    parser.add_argument(
        "--ycb",
        default="",
        help="Danh sách vật YCB ngăn cách bởi dấu phẩy, ví dụ 006_mustard_bottle,025_mug. "
             "Đây là bộ vật chuẩn benchmark; bản Axis_Aligned_Physics đã có rigid body và collision.",
    )
    parser.add_argument("--ycb-origin", type=float, nargs=3, default=(0.55, -0.15, 1.05))
    parser.add_argument("--ycb-spacing-m", type=float, default=0.22)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument(
        "--viewport-camera",
        choices=("perspective", "head"),
        default="perspective",
        help="'head' gán camera trên đầu robot vào cửa sổ, tức góc nhìn thứ nhất.",
    )
    parser.add_argument(
        "--camera-eye", type=float, nargs=3, default=(2.2, 1.6, 1.5),
        help="Vị trí camera perspective. Bỏ qua khi --viewport-camera head.",
    )
    parser.add_argument(
        "--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.9),
        help="Điểm camera perspective nhìn vào.",
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--device", default="cuda:0", help="USDRT chỉ hỗ trợ cuda:0.")
    parser.add_argument("--headless", action="store_true", help="Không mở cửa sổ; dùng để kiểm tra scene dựng được.")
    return parser


def _use_head_camera(args, head_camera_config) -> None:
    """Gán camera đầu robot vào viewport, nếu người dùng yêu cầu và có cái để gán."""

    if args.viewport_camera != "head" or args.headless:
        return
    if head_camera_config is None:
        print(
            "[viewport] profile này không khai head_camera; giữ camera perspective.",
            flush=True,
        )
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            print("[viewport] không tìm thấy viewport nào.", flush=True)
            return
        viewport.camera_path = head_camera_config.prim_path
        print(f"[viewport] cửa sổ dùng {head_camera_config.prim_path}", flush=True)
    except Exception as exc:  # noqa: BLE001 - đổi góc nhìn là tiện ích, không phải bằng chứng
        print(f"[viewport] không gán được camera đầu: {exc}", flush=True)


def main() -> int:
    args = build_parser().parse_args()
    if args.duration_s <= 0.0 or args.physics_hz <= 0.0:
        raise SystemExit("--duration-s và --physics-hz phải dương.")

    sys.path.insert(0, str(ROOT))
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=args.headless, device=args.device)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils  # noqa: E402
    from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

    from training.isaaclab.robot import UNITREE_R1_CFG  # noqa: E402

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / args.physics_hz, device=args.device)
    )
    sim.set_camera_view(list(args.camera_eye), list(args.camera_target))

    if args.dataset_scene_config is not None:
        # Một nguồn sự thật: cảnh xem trước và cảnh lúc thu đọc cùng một file.
        from teleop.r1.dataset_scene import load_dataset_scene_config, spawn_scene_props

        scene_config = load_dataset_scene_config(args.dataset_scene_config)
        if scene_config.environment is None:
            sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
            light = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9))
            light.func("/World/Light", light)
        props = spawn_scene_props(scene_config)
        robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
        robot_cfg.spawn.articulation_props.enabled_self_collisions = False
        robot_cfg.spawn.articulation_props.fix_root_link = True
        robot = Articulation(robot_cfg)
        sim.reset()
        print("", flush=True)
        print(f"Profile    : {scene_config.source_path.name}", flush=True)
        for item in props["spawned"]:
            print(f"  {item['kind']:16s} {item['name']}", flush=True)
        _use_head_camera(args, scene_config.head_camera)
        print(f"Gốc robot  : {robot.data.root_pos_w[0].tolist()}", flush=True)
        print("Ctrl-C để thoát.\n", flush=True)
        steps = int(args.duration_s * args.physics_hz)
        try:
            for _ in range(steps):
                if not simulation_app.is_running():
                    break
                for obj in props["rigid_objects"]:
                    obj.write_data_to_sim()
                sim.step()
                for obj in props["rigid_objects"]:
                    obj.update(1.0 / args.physics_hz)
                sim.render()
        except KeyboardInterrupt:
            print("Đã dừng theo yêu cầu.", flush=True)
        import threading as _threading

        sys.stdout.flush()
        closer = _threading.Thread(target=simulation_app.close, daemon=True)
        closer.start()
        closer.join(timeout=15.0)
        os._exit(0)

    environment_usd = ENVIRONMENTS[args.environment]
    if environment_usd is None:
        # Không có môi trường thì phải tự trải mặt đất, nếu không robot lơ lửng
        # trên hư không và không nhìn ra tỉ lệ.
        sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
        light = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9))
        light.func("/World/Light", light)
    else:
        cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/{environment_usd}")
        cfg.func("/World/Environment", cfg, translation=tuple(args.environment_pos))
        # Môi trường chuẩn đã có đèn riêng, nhưng thường tối với camera mặc định.
        light = sim_utils.DomeLightCfg(intensity=1000.0, color=(0.9, 0.9, 0.9))
        light.func("/World/FillLight", light)

    table_surface_z: float | None = None
    if args.table:
        if args.table_kind == "box":
            table_cfg = sim_utils.CuboidCfg(
                size=tuple(args.table_size),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.35, 0.25)),
            )
            table_surface_z = args.table_pos[2] + args.table_size[2] / 2.0
        else:
            table_cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/{TABLES[args.table_kind]}")
        table_cfg.func("/World/Table", table_cfg, translation=tuple(args.table_pos))

    objects: list[object] = []
    static_props: list[str] = []
    names = [n.strip() for n in args.ycb.split(",") if n.strip()]
    for index, name in enumerate(names):
        origin = list(args.ycb_origin)
        origin[1] += index * args.ycb_spacing_m
        if name in YCB_WITH_PHYSICS:
            objects.append(
                RigidObject(
                    RigidObjectCfg(
                        prim_path=f"/World/Ycb_{index}",
                        spawn=sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/{name}.usd"
                        ),
                        init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(origin)),
                    )
                )
            )
        else:
            # Bản Axis_Aligned không mang RigidBodyAPI, và gắn thêm vào lúc spawn
            # không đáng tin cho mọi mesh. Dựng nó như vật trang trí tĩnh: đủ để
            # xem bố cục và đúng nhu cầu "có cái để chỉ tay vào" của D001.
            cfg = sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned/{name}.usd"
            )
            cfg.func(f"/World/YcbStatic_{index}", cfg, translation=tuple(origin))
            static_props.append(name)

    robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
    # Cùng lựa chọn với đường teleop: self-collision bật thì khớp cổ R1 bị kẹt cơ
    # học và đầu đứng im; root cố định vì đây là nghiên cứu thân trên.
    robot_cfg.spawn.articulation_props.enabled_self_collisions = False
    robot_cfg.spawn.articulation_props.fix_root_link = True
    robot = Articulation(robot_cfg)

    sim.reset()
    _use_head_camera(args, None)

    print("", flush=True)
    print(f"Môi trường : {args.environment}", flush=True)
    if args.table:
        print(f"Bàn        : {args.table_kind} tâm {tuple(args.table_pos)}", flush=True)
        if table_surface_z is not None:
            print(f"  mặt bàn ở z = {table_surface_z:.3f} m (vai robot ở z ≈ 1.006)", flush=True)
        if args.table_kind in TABLES_WITHOUT_COLLISION:
            print("  CẢNH BÁO: asset này không có collision — vật sẽ rơi xuyên qua bàn.", flush=True)
    else:
        print("Bàn        : không", flush=True)
    print(f"Vật YCB    : {names if names else 'không'}", flush=True)
    if static_props:
        print(f"  (tĩnh, không physics: {static_props} — kho Isaac 5.1 chưa có bản physics cho chúng)", flush=True)
    print(f"Gốc robot  : {robot.data.root_pos_w[0].tolist()}", flush=True)
    print("Ctrl-C để thoát.", flush=True)
    print("", flush=True)

    steps = int(args.duration_s * args.physics_hz)
    try:
        for _ in range(steps):
            if not simulation_app.is_running():
                break
            for obj in objects:
                obj.write_data_to_sim()
            sim.step()
            for obj in objects:
                obj.update(1.0 / args.physics_hz)
            # Viewport chỉ cập nhật khi render() được gọi.
            sim.render()
    except KeyboardInterrupt:
        print("Đã dừng theo yêu cầu.", flush=True)

    for index, obj in enumerate(objects):
        # `data.root_pos_w` chỉ được làm mới bởi `update()`; thiếu nó thì đọc ra
        # đúng giá trị khởi tạo và tưởng vật đã nằm yên.
        obj.update(1.0 / args.physics_hz)
        print(f"Ycb_{index} vị trí cuối: {[round(v, 3) for v in obj.data.root_pos_w[0].tolist()]}", flush=True)

    # `SimulationApp.close()` treo vô hạn trên máy trạm này — cùng lý do
    # `scripts/teleop/run_r1_quest3_live.py:586` phải bọc nó lại. Không có gì để
    # ghi ra đĩa nên cứ cho nó 15 s rồi thoát hẳn.
    sys.stdout.flush()
    closer = threading.Thread(target=simulation_app.close, daemon=True)
    closer.start()
    closer.join(timeout=15.0)
    if closer.is_alive():
        print("Isaac Sim không đóng trong 15 s; thoát cưỡng bức.", file=sys.stderr, flush=True)
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
