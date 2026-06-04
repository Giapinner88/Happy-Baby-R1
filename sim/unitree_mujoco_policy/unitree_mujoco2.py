import argparse
import csv as csvlib
import sys
import time
from pathlib import Path
from threading import Thread
import threading

import mujoco
import importlib

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config

# ─── Thứ tự 29 khớp (khớp với motor_state và cột CSV) ───────────────────────
_JOINT_NAMES = [
    "left_hip_pitch",     "left_hip_roll",     "left_hip_yaw",
    "left_knee",          "left_ankle_pitch",  "left_ankle_roll",
    "right_hip_pitch",    "right_hip_roll",    "right_hip_yaw",
    "right_knee",         "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw",          "waist_roll",        "waist_pitch",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
    "left_elbow",         "left_wrist_roll",   "left_wrist_pitch",  "left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    "right_elbow",        "right_wrist_roll",  "right_wrist_pitch", "right_wrist_yaw",
]


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
        joint_id  = model.joint(jname + "_joint").id
        qpos_adr  = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = float(target_row[col])

    # 2. Set vận tốc khớp (qvel[6..34])
    for jname in _JOINT_NAMES:
        col = f"dq_{jname}"
        if col not in target_row:
            continue
        joint_id = model.joint(jname + "_joint").id
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
_args, _unknown = _parser.parse_known_args()


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)


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
            if config.ROBOT == "h1" or config.ROBOT == "g1":
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
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
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
