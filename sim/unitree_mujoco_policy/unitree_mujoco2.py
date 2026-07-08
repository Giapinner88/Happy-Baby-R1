import argparse
import csv as csvlib
import os
import sys
import time
from pathlib import Path
from threading import Thread
import threading

import mujoco
import importlib

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config

_JOINT_NAMES = config.MOTOR_JOINT_NAMES
DEFAULT_Q = config.DEFAULT_Q.tolist()


def _xml_joint_name(motor_name: str) -> str:
    return motor_name if motor_name.endswith("_joint") else f"{motor_name}_joint"


def init_state_from_csv(model, data, csv_path: str, row_index: int = 0):
    """
    Khởi tạo trạng thái MuJoCo từ một hàng trong file CSV log.
    - qpos[7:36] ← q_<joint>   (29 góc khớp)
    - qvel[6:35] ← dq_<joint>  (29 vận tốc khớp)
    - qpos[3:7]  ← imu_quat    (hướng base: w,x,y,z)
    - qpos[0:3]  giữ nguyên    (vị trí XYZ từ spawn mặc định)
    """
    path = Path(csv_path)
    if not path.exists():
        print(f"[init-csv] Không tìm thấy: {csv_path}", file=sys.stderr)
        return

    with open(path, newline="") as f:
        reader = csvlib.DictReader(f)
        target_row = None
        for i, row in enumerate(reader):
            if i == row_index:
                target_row = row
                break

    if target_row is None:
        print(f"[init-csv] Không tìm thấy row {row_index} trong CSV.", file=sys.stderr)
        return

    # 1. Set góc khớp (qpos[7..35])
    missing_q = []
    for jname in _JOINT_NAMES:
        col = f"q_{jname}"
        if col not in target_row:
            missing_q.append(col)
            continue
        joint_id  = model.joint(_xml_joint_name(jname)).id
        qpos_adr  = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = float(target_row[col])

    # 2. Set vận tốc khớp (qvel[6..34])
    for jname in _JOINT_NAMES:
        col = f"dq_{jname}"
        if col not in target_row:
            continue
        joint_id = model.joint(_xml_joint_name(jname)).id
        dof_adr  = model.jnt_dofadr[joint_id]
        data.qvel[dof_adr] = float(target_row[col])

    # 3. Set hướng base từ IMU quaternion (w,x,y,z)
    try:
        qw = float(target_row.get("imu_quat_w", 1.0))
        qx = float(target_row.get("imu_quat_x", 0.0))
        qy = float(target_row.get("imu_quat_y", 0.0))
        qz = float(target_row.get("imu_quat_z", 0.0))
        base_jid = model.joint("floating_base_joint").id
        base_adr = model.jnt_qposadr[base_jid]  # = 0
        # qpos[base_adr+0:3] = position — giữ nguyên (spawn height)
        data.qpos[base_adr + 3] = qw
        data.qpos[base_adr + 4] = qx
        data.qpos[base_adr + 5] = qy
        data.qpos[base_adr + 6] = qz
    except Exception as e:
        print(f"[init-csv] Cảnh báo khi set base quat: {e}")

    mujoco.mj_forward(model, data)

    print(f"[init-csv] State đã set từ row {row_index} của '{path.name}'")
    if missing_q:
        print(f"[init-csv] Thiếu cột: {missing_q}")


def init_default_q(model, data):
    """Match the simulator spawn pose to the policy's initial hold command."""
    for joint_name, q in zip(_JOINT_NAMES, DEFAULT_Q):
        joint_id = model.joint(_xml_joint_name(joint_name)).id
        qpos_adr = model.jnt_qposadr[joint_id]
        dof_adr = model.jnt_dofadr[joint_id]
        data.qpos[qpos_adr] = q
        data.qvel[dof_adr] = 0.0

    mujoco.mj_forward(model, data)
    print("[init-default-q] State đã set theo DEFAULT_Q của policy.")


def set_hanging_equality(model, data, active: bool):
    """Toggle the optional scene equality used by hanging/debug scenes."""
    equality_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "r1_hanging_connect"
    )
    if equality_id < 0:
        return
    data.eq_active[equality_id] = 1 if active else 0
    state = "active" if active else "disabled"
    print(f"[sim-config] r1_hanging_connect {state}.")


# ─── Parse arguments ─────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser(description="Unitree MuJoCo Simulator")
_parser.add_argument(
    "--init-csv", default=None, metavar="CSV",
    help="Đường dẫn CSV log để khởi tạo trạng thái ban đầu của sim."
)
_parser.add_argument(
    "--init-row", type=int, default=0, metavar="N",
    help="Hàng trong CSV để đọc trạng thái (mặc định: 0 = hàng đầu tiên)."
)
_parser.add_argument("--robot", default=None, help="Must be r1 for this local policy runtime.")
_parser.add_argument("--scene", default=None, help="Override scene file, e.g. scene.xml or scene_hanging.xml.")
_parser.add_argument("--domain-id", type=int, default=None, help="Override DDS domain id.")
_parser.add_argument("--interface", default=None, help="Override DDS network interface.")
_args, _unknown = _parser.parse_known_args()

if _args.robot:
    if _args.robot != "r1":
        raise ValueError("sim/unitree_mujoco_policy is R1-only; do not run it with G1/other robots.")
    config.ROBOT = _args.robot
    config.USE_HG_IDL = True
if _args.scene:
    os.environ["ROBOT_SCENE_NAME"] = _args.scene
    config.ROBOT_SCENE_NAME = _args.scene
    config.ROBOT_SCENE = str(config._resolve_robot_scene())
if _args.domain_id is not None:
    config.DOMAIN_ID = _args.domain_id
if _args.interface:
    config.INTERFACE = _args.interface

from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand


locker = threading.Lock()

print(
    f"[sim-config] robot={config.ROBOT} scene={config.ROBOT_SCENE} "
    f"domain={config.DOMAIN_ID} interface={config.INTERFACE} hg_idl={config.USE_HG_IDL}"
)
mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

disable_hanging_equality = os.environ.get("DISABLE_HANGING_EQUALITY", "0").lower()
if disable_hanging_equality in {"1", "true", "yes"}:
    set_hanging_equality(mj_model, mj_data, active=False)

if os.environ.get("INIT_DEFAULT_Q", "0").lower() in {"1", "true", "yes"}:
    init_default_q(mj_model, mj_data)


class _HeadlessViewer:
    """Minimal viewer replacement for headless operation.

    Provides the methods used by the rest of the simulator (`is_running`,
    `sync`) so the simulation loop can run without creating an OpenGL window.
    """
    def __init__(self):
        self._running = True

    def is_running(self):
        return self._running

    def sync(self):
        # no-op for headless
        return


try:
    try:
        # Import mujoco.viewer lazily — on some Wayland setups importing the
        # viewer module can trigger a native crash. Import inside try so we
        # can fall back to headless if it fails.
        mujoco_viewer = importlib.import_module('mujoco.viewer')
        if config.ENABLE_ELASTIC_BAND:
            elastic_band = ElasticBand()
            if config.ROBOT in config.HUMANOID_HG_ROBOTS:
                band_attached_link = mj_model.body("torso_link").id
            else:
                band_attached_link = mj_model.body("base_link").id
            viewer = mujoco_viewer.launch_passive(
                mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
            )
        else:
            viewer = mujoco_viewer.launch_passive(mj_model, mj_data)
    except Exception:
        # Fallback to headless viewer if the platform/OpenGL driver fails
        print("[sim] Viewer initialization failed — falling back to headless mode.")
        viewer = _HeadlessViewer()
except Exception:
    # Fallback to headless viewer if the platform/OpenGL driver fails
    print("[sim] Viewer initialization failed — falling back to headless mode.")
    viewer = _HeadlessViewer()

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data, data_lock=locker)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=config.JOYSTICK_DEVICE, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    # ── Khởi tạo state từ CSV nếu có --init-csv ──────────────────────────────
    if _args.init_csv:
        # Tìm CSV mới nhất nếu dùng wildcard hoặc tên thư mục
        csv_path = Path(_args.init_csv)
        if not csv_path.exists():
            # Thử tìm trong sim_state_logs/
            # Prefer repo-level data directory for logs
            log_dir = Path(__file__).resolve().parents[2] / "data" / "sim_state_logs"
            matches = sorted(log_dir.glob("*.csv"),
                             key=lambda p: p.stat().st_mtime, reverse=True)
            csv_path = matches[0] if matches else csv_path
        init_state_from_csv(mj_model, mj_data, str(csv_path), _args.init_row)

    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
