"""
run_mimic.py — Deploy mimic tracking policy cho G1 29-DOF.
Logic khớp hoàn toàn với mjlab/deploy/robots/g1/src/State_Mimic.cpp
"""
import time
import argparse
import threading
import numpy as np
import onnxruntime as ort

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

robot_state = None
cmd = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()
state_lock = threading.Lock()

# --- THÔNG SỐ TRAINING (khớp với deploy.yaml) ---
JOINT_MAP = list(range(29))  # Identity: RL index == URDF motor index

KP_ARRAY = np.array([40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 28.5, 28.5,
                     14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8], dtype=np.float32)

KD_ARRAY = np.array([2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 1.8, 1.8,
                     0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1], dtype=np.float32)

DEFAULT_Q = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0,
                       0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0], dtype=np.float32)

ACTION_SCALE = np.array([0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44,
                          0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07], dtype=np.float32)


# ===========================================================
# QUATERNION HELPERS (wxyz convention)
# ===========================================================
def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float32)

def quat_to_rot_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float32)

def yaw_quat(q):
    """Lấy chỉ thành phần yaw từ quaternion (wxyz). Trả về quaternion wxyz."""
    w, x, y, z = q
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)
    return np.array([cy, 0.0, 0.0, sy], dtype=np.float32)

def compute_torso_quat(pelvis_quat_wxyz, waist_yaw, waist_roll, waist_pitch):
    """
    Tính quaternion của torso_link từ pelvis + khớp eo.
    Khớp với logic C++:
      torso_quat = root_quat
                 * AngleAxis(waist_yaw,   Z)
                 * AngleAxis(waist_roll,  X)
                 * AngleAxis(waist_pitch, Y)
    """
    def angle_axis(angle, axis):
        """Tạo quaternion [w,x,y,z] từ angle-axis."""
        c = np.cos(angle / 2)
        s = np.sin(angle / 2)
        return np.array([c, s*axis[0], s*axis[1], s*axis[2]], dtype=np.float32)

    q_yaw   = angle_axis(waist_yaw,   [0, 0, 1])
    q_roll  = angle_axis(waist_roll,  [1, 0, 0])
    q_pitch = angle_axis(waist_pitch, [0, 1, 0])

    torso_q = quat_mul(pelvis_quat_wxyz, q_yaw)
    torso_q = quat_mul(torso_q, q_roll)
    torso_q = quat_mul(torso_q, q_pitch)
    return torso_q


# ===========================================================
# ĐỌC FILE NPZ (định dạng mjlab)
# ===========================================================
class MotionData:
    def __init__(self, filename, target_dt=0.02):
        import os
        # Tìm trong thư mục motions/ nếu chưa có đường dẫn đầy đủ
        if not os.path.exists(filename) and not os.path.isabs(filename):
            possible_path = os.path.join("motions", filename)
            if os.path.exists(possible_path):
                filename = possible_path

        print(f"[LOAD] Đang đọc file: {filename}")
        if not filename.endswith(".npz"):
            raise ValueError("Chỉ hỗ trợ file NPZ. Dùng file được tạo bởi csv_to_npz.py của mjlab.")

        data = np.load(filename)
        print(f"[LOAD] NPZ keys: {list(data.files)}")

        self.joint_pos  = data["joint_pos"].astype(np.float32)   # (T, 29)
        self.joint_vel  = data["joint_vel"].astype(np.float32)   # (T, 29)
        body_pos_w      = data["body_pos_w"].astype(np.float32)  # (T, N_bodies, 3)
        body_quat_w     = data["body_quat_w"].astype(np.float32) # (T, N_bodies, 4) - wxyz

        self.num_frames = self.joint_pos.shape[0]
        self.dt = target_dt

        # body index 0 = pelvis (theo MotionLoader C++ line 69-78)
        self.root_quat = body_quat_w[:, 0, :]  # (T, 4) wxyz

        print(f"[LOAD] Frames: {self.num_frames}, dt: {self.dt}s, joints: {self.joint_pos.shape[1]}")
        print(f"[LOAD] N_bodies trong NPZ: {body_quat_w.shape[1]}")


def state_handler(msg: LowState_):
    global robot_state
    with state_lock:
        robot_state = msg

def dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        time.sleep(0.002)


def main():
    global robot_state, cmd

    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=str, default="policy_motion_data.onnx")
    parser.add_argument("--npz",    type=str, default="motion_data.npz", help="File NPZ (từ csv_to_npz.py)")
    parser.add_argument("--network",type=str, default="lo")
    args = parser.parse_args()

    print("=" * 60)
    print(f"  Policy : {args.policy}")
    print(f"  Motion : {args.npz}")
    print(f"  Network: {args.network}")
    print("=" * 60)

    ChannelFactoryInitialize(1, args.network)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)

    # Khởi tạo motor
    for i in range(29):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q    = float(DEFAULT_Q[i])
        cmd.motor_cmd[i].dq   = 0.0
        cmd.motor_cmd[i].tau  = 0.0
        cmd.motor_cmd[i].kp   = float(KP_ARRAY[i])
        cmd.motor_cmd[i].kd   = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    session = ort.InferenceSession(args.policy, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    print(f"[ONNX] Input: {input_name}, shape: {session.get_inputs()[0].shape}")

    motion = MotionData(args.npz)

    # --- Chờ kết nối với simulator ---
    print("[WAIT] Chờ dữ liệu từ simulator...")
    while True:
        with state_lock:
            rs = robot_state
        if rs is not None:
            print("[OK] Kết nối thành công!")
            break
        time.sleep(0.01)

    # --- Khởi tạo init_quat (bù lệch yaw giữa robot và motion) ---
    # Logic C++ (State_Mimic.cpp lines 163-165):
    #   ref_yaw   = yawQuaternion(motion->root_quaternion())
    #   robot_yaw = yawQuaternion(robot_quat_w(env))
    #   init_quat = robot_yaw * ref_yaw.transpose()  (conjugate vì rotation matrix)
    with state_lock:
        rs_init = robot_state
    pelvis_q0 = np.array(rs_init.imu_state.quaternion, dtype=np.float32)  # wxyz
    waist_yaw0   = rs_init.motor_state[12].q
    waist_roll0  = rs_init.motor_state[13].q
    waist_pitch0 = rs_init.motor_state[14].q
    robot_torso_q0 = compute_torso_quat(pelvis_q0, waist_yaw0, waist_roll0, waist_pitch0)

    ref_root_q0 = motion.root_quat[0]  # pelvis reference quat
    ref_waist_yaw0   = motion.joint_pos[0, 12]
    ref_waist_roll0  = motion.joint_pos[0, 13]
    ref_waist_pitch0 = motion.joint_pos[0, 14]
    ref_torso_q0 = compute_torso_quat(ref_root_q0, ref_waist_yaw0, ref_waist_roll0, ref_waist_pitch0)

    robot_yaw_q = yaw_quat(robot_torso_q0)
    ref_yaw_q   = yaw_quat(ref_torso_q0)
    # init_quat = robot_yaw * ref_yaw^{-1}
    init_quat = quat_mul(robot_yaw_q, quat_inv(ref_yaw_q))
    print(f"[INIT] init_quat (wxyz) = {init_quat}")

    last_action = np.zeros(29, dtype=np.float32)
    t_start = time.perf_counter()

    try:
        while True:
            # --- Đọc trạng thái robot (thread-safe) ---
            with state_lock:
                rs = robot_state
                q_real  = [rs.motor_state[i].q  for i in range(29)]
                dq_real = [rs.motor_state[i].dq for i in range(29)]
                pelvis_q = np.array(rs.imu_state.quaternion, dtype=np.float32)  # wxyz
                gyro     = np.array(rs.imu_state.gyroscope,  dtype=np.float32)

            step_start = time.perf_counter()
            t_current  = step_start - t_start
            frame_idx  = int(t_current / motion.dt) % motion.num_frames

            # === 1. motion_command (58 dims) ===
            ref_q  = motion.joint_pos[frame_idx]   # (29,) absolute joint pos từ NPZ
            ref_dq = motion.joint_vel[frame_idx]   # (29,) joint vel từ NPZ
            motion_command = np.concatenate([ref_q, ref_dq]).astype(np.float32)

            # === 2. motion_anchor_ori_b (6 dims) ===
            # Robot torso quat = pelvis * rotate(waist joints)
            waist_yaw_r   = q_real[12]
            waist_roll_r  = q_real[13]
            waist_pitch_r = q_real[14]
            real_torso_q = compute_torso_quat(pelvis_q, waist_yaw_r, waist_roll_r, waist_pitch_r)

            # Reference torso quat = ref_root_quat * rotate(ref waist joints)
            ref_root_q     = motion.root_quat[frame_idx]
            ref_waist_yaw   = ref_q[12]
            ref_waist_roll  = ref_q[13]
            ref_waist_pitch = ref_q[14]
            ref_torso_q = compute_torso_quat(ref_root_q, ref_waist_yaw, ref_waist_roll, ref_waist_pitch)

            # rot_ = (init_quat * ref_torso_q)^{-1} * real_torso_q
            # Đây là relative rotation: "thực tế so với tham chiếu đã offset"
            target_torso_q = quat_mul(init_quat, ref_torso_q)
            q_rel_rot = quat_mul(quat_inv(target_torso_q), real_torso_q)

            # Rotation matrix, lấy 2 cột đầu theo row-major (khớp C++ line 75)
            R = quat_to_rot_mat(q_rel_rot).T  # .T vì C++ dùng rot.transpose()
            motion_anchor_ori_b = np.array([
                R[0, 0], R[0, 1],
                R[1, 0], R[1, 1],
                R[2, 0], R[2, 1],
            ], dtype=np.float32)

            # === 3. base_ang_vel / gyro (3 dims) ===
            # gyro đã đọc bên trên

            # === 4. joint_pos_rel (29 dims) ===
            q_rel_rl = np.array(q_real, dtype=np.float32) - DEFAULT_Q

            # === 5. joint_vel_rel (29 dims) ===
            dq_rl = np.array(dq_real, dtype=np.float32)

            # === 6. Gộp observation (154 dims) ===
            obs = np.concatenate([
                motion_command,         # 58
                motion_anchor_ori_b,    # 6
                gyro,                   # 3
                q_rel_rl,               # 29
                dq_rl,                  # 29
                last_action,            # 29
            ]).astype(np.float32)       # Total: 154

            # === 7. Chạy policy ===
            obs_tensor = np.expand_dims(obs, axis=0)
            action = session.run(None, {input_name: obs_tensor})[0][0]
            last_action = action.copy()

            # === 8. Gửi lệnh xuống motor ===
            with cmd_lock:
                for i in range(29):
                    target_q = DEFAULT_Q[i] + action[i] * ACTION_SCALE[i]
                    cmd.motor_cmd[i].q = float(target_q)

            elapsed = time.perf_counter() - step_start
            time_until_next = motion.dt - elapsed
            if time_until_next > 0:
                time.sleep(time_until_next)

    except KeyboardInterrupt:
        print("\n>>> Dừng chương trình.")


if __name__ == '__main__':
    main()
