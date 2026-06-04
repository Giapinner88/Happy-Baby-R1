import os
import time
import threading
from pathlib import Path
import numpy as np
import onnxruntime as ort

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from state_logger import SimStateLogger

robot_state = None
cmd = unitree_hg_msg_dds__LowCmd_()
SCRIPT_DIR = Path(__file__).resolve().parent

# --- BẢN ĐỒ DÂY THẦN KINH (CHUYỂN ĐỔI RL -> URDF) ---
JOINT_MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11,
             17, 24, 18, 25, 19, 26, 20, 27, 21, 28]

# --- HẰNG SỐ NÀY ĐANG Ở HỆ URDF (VẬT LÝ) ---
KP_ARRAY = np.array([40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
                     40.2, 28.5, 28.5, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3,
                     14.3, 16.8, 16.8], dtype=np.float32)

KD_ARRAY = np.array([2.56, 6.31, 2.56, 6.31, 1.81, 1.81, 2.56, 6.31, 2.56, 6.31, 1.81, 1.81,
                     2.56, 1.81, 1.81, 0.907, 0.907, 0.907, 0.907, 0.907, 1.07, 1.07, 0.907, 0.907, 0.907,
                     0.907, 0.907, 1.07, 1.07], dtype=np.float32)

DEFAULT_Q = np.array([-0.302, -0.319, 0.00124, 0.000442, 0.00489, 0.00191, 0.00929,
                      0.00796, 0.00546, 0.672, 0.67, 0.2, 0.202, -0.368, -0.355, 0.194, -0.196, -0.00644,
                      0.00976, 0.00258, -0.00029, 0.605, 0.596, 0.00818, 0.00322, 0.00293, -0.00339, -0.00955,
                      -0.00715], dtype=np.float32)

ACTION_SCALE = np.array([0.548, 0.548, 0.548, 0.351, 0.351, 0.439, 0.548, 0.548, 0.439, 0.351,
                         0.351, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439,
                         0.439, 0.439, 0.439, 0.439, 0.0745, 0.0745, 0.0745, 0.0745], dtype=np.float32)

def state_handler(msg: LowState_):
    global robot_state
    robot_state = msg

def dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        cmd.crc = crc_calc.Crc(cmd)
        pub.Write(cmd)
        time.sleep(0.002)

def resolve_asset_path(env_name, default_name):
    env_value = os.environ.get(env_name, "").strip()
    if env_value:
        path = Path(env_value).expanduser()
        if not path.is_absolute():
            path = SCRIPT_DIR / path
        if path.exists():
            return path
        raise FileNotFoundError(f"Không tìm thấy {env_name}: {path}")

    path = SCRIPT_DIR / default_name
    if path.exists():
        return path
    raise FileNotFoundError(f"Không tìm thấy file bắt buộc: {path}")


def compute_projected_gravity(quat):
    w, x, y, z = quat
    return np.array([
        2 * (w * y - x * z),
        -2 * (y * z + w * x),
        2 * (x**2 + y**2) - 1,
    ], dtype=np.float32)


def load_reference_motion(filename):
    try:
        data = np.loadtxt(filename, delimiter=',')
        return data
    except Exception as exc:
        print(f"LỖI: Không đọc được file CSV: {filename} ({exc})")
        return None

def main():
    global robot_state, cmd
    
    domain_id = int(os.environ.get("DOMAIN_ID", "1"))
    interface = os.environ.get("INTERFACE", "lo")
    ChannelFactoryInitialize(domain_id, interface) 
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)

    # KHỞI TẠO ĐỘNG CƠ (Sử dụng hệ URDF trực tiếp)
    for i in range(29):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = DEFAULT_Q[i]
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
        cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    policy_path = resolve_asset_path("POLICY_ONNX", "policy_dance.onnx")
    print(f"Dùng policy: {policy_path}")
    session = ort.InferenceSession(str(policy_path), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    
    motion_path = resolve_asset_path("MOTION_CSV", "G1_Take_102.bvh_60hz.csv")
    print(f"Dùng motion: {motion_path}")
    ref_motion = load_reference_motion(motion_path)
    if ref_motion is None: return
    num_frames = ref_motion.shape[0]

    last_action = np.zeros(29, dtype=np.float32)
    t_start = time.perf_counter()
    state_logger = SimStateLogger(__file__)
    step = 0

    try:
        while True:
            if robot_state is None:
                time.sleep(0.002)
                continue
                
            step_start = time.perf_counter()
            t_current = step_start - t_start

            # --- NỘI SUY THỜI GIAN MOCAP ---
            time_in_motion = t_current % (num_frames / 60.0)
            frame_exact = time_in_motion * 60.0
            idx_low = int(frame_exact)
            idx_high = (idx_low + 1) % num_frames
            alpha = frame_exact - idx_low

            current_row = (1.0 - alpha) * ref_motion[idx_low] + alpha * ref_motion[idx_high]

            # 1. TRÍCH XUẤT LỆNH THAM CHIẾU VÀ ÉP SANG HỆ RL (RL MAP)
            ref_q_rl = np.zeros(29, dtype=np.float32)
            ref_dq_rl = np.zeros(29, dtype=np.float32)
            
            row_q_urdf = current_row[7:36]
            row_q_next_urdf = ref_motion[idx_high][7:36]
            row_dq_urdf = (row_q_next_urdf - row_q_urdf) * 60.0
            
            for i in range(29): # i là index trong hệ RL
                urdf_idx = JOINT_MAP[i]
                ref_q_rl[i] = row_q_urdf[urdf_idx]
                ref_dq_rl[i] = row_dq_urdf[urdf_idx]
                
            motion_command = np.concatenate([ref_q_rl, ref_dq_rl]).astype(np.float32)

            # 2. TÍNH TOÁN 6D ROTATION
            ref_x, ref_y, ref_z, ref_w = current_row[3:7]
            R00 = 1.0 - 2.0*(ref_y**2 + ref_z**2)
            R10 = 2.0*(ref_x*ref_y + ref_w*ref_z)
            R20 = 2.0*(ref_x*ref_z - ref_w*ref_y)
            R01 = 2.0*(ref_x*ref_y - ref_w*ref_z)
            R11 = 1.0 - 2.0*(ref_x**2 + ref_z**2)
            R21 = 2.0*(ref_y*ref_z + ref_w*ref_x)
            motion_anchor_ori_b = np.array([R00, R10, R20, R01, R11, R21], dtype=np.float32)

            # 3. TRÍCH XUẤT TRẠNG THÁI ROBOT VÀ ÉP SANG HỆ RL
            q_state = np.zeros(29, dtype=np.float32)
            dq_state = np.zeros(29, dtype=np.float32)
            q_rel_rl = np.zeros(29, dtype=np.float32)
            dq_current_rl = np.zeros(29, dtype=np.float32)
            for i in range(29):
                urdf_idx = JOINT_MAP[i]
                q_real = robot_state.motor_state[urdf_idx].q
                dq_real = robot_state.motor_state[urdf_idx].dq
                q_state[urdf_idx] = q_real
                dq_state[urdf_idx] = dq_real
                dq_current_rl[i] = dq_real
                # Phép trừ cùng hệ quy chiếu URDF, sau đó gán vào mảng RL
                q_rel_rl[i] = q_real - DEFAULT_Q[urdf_idx]
                
            gyro = np.array(robot_state.imu_state.gyroscope, dtype=np.float32)
            quat = np.array(robot_state.imu_state.quaternion, dtype=np.float32)
            projected_gravity = compute_projected_gravity(quat)

            # 4. GÓI VECTOR QUAN SÁT (Tất cả đã ở hệ RL)
            obs = np.concatenate([
                motion_command,       
                motion_anchor_ori_b,  
                gyro,                 
                q_rel_rl,                
                dq_current_rl,           
                last_action           
            ]).astype(np.float32)

            obs_tensor = np.expand_dims(obs, axis=0)
            action = session.run(None, {input_name: obs_tensor})[0][0]
            last_action = action.copy()

            # 5. GỬI LỆNH XUỐNG ĐỘNG CƠ (Chuyển RL -> URDF)
            target_q_arr = DEFAULT_Q.copy()
            action_urdf = np.zeros(29, dtype=np.float32)
            for i in range(29):
                urdf_idx = JOINT_MAP[i]
                # Nhân action RL với scale tương ứng của khớp đó ở hệ URDF
                target_q = DEFAULT_Q[urdf_idx] + (action[i] * ACTION_SCALE[urdf_idx])
                action_urdf[urdf_idx] = action[i]
                target_q_arr[urdf_idx] = target_q
                cmd.motor_cmd[urdf_idx].q = float(target_q)

            state_logger.log(
                step=step,
                t=t_current,
                target_q=target_q_arr,
                q=q_state,
                dq=dq_state,
                action=action_urdf,
                quat=quat,
                gyro=gyro,
                proj_grav=projected_gravity,
                commands=np.zeros(3, dtype=np.float32),
                gait_phase=np.zeros(2, dtype=np.float32),
                gait_scale=1.0,
                gait_time=time_in_motion,
            )
            step += 1

            time_until_next = 0.02 - (time.perf_counter() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
    except KeyboardInterrupt:
        pass
    finally:
        state_logger.close()

if __name__ == '__main__':
    main()
