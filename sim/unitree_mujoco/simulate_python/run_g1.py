import time
import threading
import math
import numpy as np
import onnxruntime as ort
import pygame

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from fall_detector.detector import G1FallDetector
from arm_csv_player import ArmCSVPlayer
try:
    from fall_detector.logger import IMULogger
    _HAS_IMU_LOGGER = True
except ImportError:
    _HAS_IMU_LOGGER = False

# ===== THÔNG SỐ MOTOR =====
KP = np.array([40.2,99.1,40.2,99.1,28.5,28.5, 40.2,99.1,40.2,99.1,28.5,28.5,
               40.2,28.5,28.5, 14.3,14.3,14.3,14.3,14.3,16.8,16.8,
               14.3,14.3,14.3,14.3,14.3,16.8,16.8], dtype=np.float32)
KD = np.array([2.6,6.3,2.6,6.3,1.8,1.8, 2.6,6.3,2.6,6.3,1.8,1.8,
               2.6,1.8,1.8, 0.9,0.9,0.9,0.9,0.9,1.1,1.1,
               0.9,0.9,0.9,0.9,0.9,1.1,1.1], dtype=np.float32)
DEFAULT_Q = np.array([-0.1,0,0,0.3,-0.2,0, -0.1,0,0,0.3,-0.2,0, 0,0,0,
                       0.35,0.18,0,0.87,0,0,0, 0.35,-0.18,0,0.87,0,0,0], dtype=np.float32)
ACTION_SCALE = np.array([0.55,0.35,0.55,0.35,0.44,0.44, 0.55,0.35,0.55,0.35,0.44,0.44,
                          0.55,0.44,0.44, 0.44,0.44,0.44,0.44,0.44,0.07,0.07,
                          0.44,0.44,0.44,0.44,0.44,0.07,0.07], dtype=np.float32)

# ===== MODE CONFIG =====
MODES = [
    {
        "name":   "🚶 Locomotion",
        "policy": "policy/policy98.onnx",
        "npz":    None,
    },
    {
        "name":   "💃 Mimic Dance",
        "policy": "policy/policy_motion_data.onnx",
        "npz":    "motions/motion_data.npz",
    },
    {
        "name":   "🎾 Mimic Tennis",
        "policy": "policy/policy_tennis.onnx",
        "npz":    "motions/unitree_g1_tennis.npz",
    },
]
NUM_MODES = len(MODES)

# ===== GLOBAL STATE =====
robot_state = None
state_lock  = threading.Lock()
cmd         = unitree_hg_msg_dds__LowCmd_()
cmd_lock    = threading.Lock()


# ===== HELPERS =====
def state_handler(msg: LowState_):
    global robot_state
    with state_lock:
        robot_state = msg

def dds_publisher_loop(pub):
    crc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
        time.sleep(0.002)

def init_motor_cmd():
    with cmd_lock:
        for i in range(29):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q    = float(DEFAULT_Q[i])
            cmd.motor_cmd[i].dq   = 0.0
            cmd.motor_cmd[i].tau  = 0.0
            cmd.motor_cmd[i].kp   = float(KP[i])
            cmd.motor_cmd[i].kd   = float(KD[i])

def compute_projected_gravity(q):
    w, x, y, z = q
    return np.array([2*(w*y - x*z), -2*(y*z + w*x), 2*(x*x + y*y) - 1], dtype=np.float32)

# ===== QUATERNION (wxyz) =====
def quat_inv(q):   return np.array([q[0],-q[1],-q[2],-q[3]], dtype=np.float32)
def quat_mul(a,b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], dtype=np.float32)
def quat_to_rot(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]], dtype=np.float32)
def yaw_quat(q):
    w,x,y,z=q; yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    c,s=math.cos(yaw/2),math.sin(yaw/2)
    return np.array([c,0,0,s], dtype=np.float32)
def angle_axis(angle, axis):
    c,s=math.cos(angle/2),math.sin(angle/2)
    return np.array([c,s*axis[0],s*axis[1],s*axis[2]], dtype=np.float32)
def torso_quat(pelvis_q, j12, j13, j14):
    q = quat_mul(pelvis_q, angle_axis(j12,[0,0,1]))
    q = quat_mul(q, angle_axis(j13,[1,0,0]))
    q = quat_mul(q, angle_axis(j14,[0,1,0]))
    return q

# ===== NPZ LOADER =====
class MotionData:
    def __init__(self, path, target_dt=0.02):
        import os
        if not os.path.exists(path) and not os.path.isabs(path):
            alt = os.path.join("motions", os.path.basename(path))
            if os.path.exists(alt): path = alt
        d = np.load(path)
        self.joint_pos  = d["joint_pos"].astype(np.float32)
        self.joint_vel  = d["joint_vel"].astype(np.float32)
        self.root_quat  = d["body_quat_w"][:,0,:].astype(np.float32)
        self.num_frames = self.joint_pos.shape[0]
        self.dt         = target_dt
        print(f"[NPZ] {path}: {self.num_frames} frames, {self.joint_pos.shape[1]} joints")


# ===== POLICY RUNNERS =====

def build_loco_obs(rs, smoothed_cmds, gait_phase, arm_player, last_action):
    """Xây dựng observation 69 dims cho locomotion policy."""
    q = np.array([rs.motor_state[i].q  for i in range(29)], dtype=np.float32)
    dq= np.array([rs.motor_state[i].dq for i in range(29)], dtype=np.float32)
    gyro = np.array(rs.imu_state.gyroscope, dtype=np.float32)
    quat = np.array(rs.imu_state.quaternion, dtype=np.float32)
    proj_g = compute_projected_gravity(quat)
    q_rel = q - DEFAULT_Q
    la = last_action.copy()
    
    smoothed_cmds_obs = smoothed_cmds.copy()
    proj_g_obs = proj_g.copy()
    if getattr(arm_player, 'blend_weight', 0.0) > 0.0:
        q_rel[15:29] = 0.0; dq[15:29] = 0.0; la[15:29] = 0.0
        
        # Bù trừ thăng bằng kết hợp để giữ robot thăng bằng tại chỗ không bị trôi forward
        if arm_player.active_motion == "vaytay":
            gx_bias = 0.08
            vx_bias = 0.08
        elif arm_player.active_motion == "battay":
            gx_bias = 0.05
            vx_bias = 0.05
        elif arm_player.active_motion == "traitim":
            gx_bias = 0.0
            vx_bias = 0.0
        else:
            gx_bias = 0.08
            vx_bias = 0.08
        smoothed_cmds_obs[0] -= vx_bias * arm_player.blend_weight
        proj_g_obs[0] += gx_bias * arm_player.blend_weight
        
    return np.concatenate([gyro, proj_g_obs, smoothed_cmds_obs, gait_phase, q_rel, dq, la]).astype(np.float32)

def build_mimic_obs(rs, motion, frame_idx, init_q, last_action):
    """Xây dựng observation 154 dims cho mimic policy."""
    q_real  = np.array([rs.motor_state[i].q  for i in range(29)], dtype=np.float32)
    dq_real = np.array([rs.motor_state[i].dq for i in range(29)], dtype=np.float32)
    pelvis_q = np.array(rs.imu_state.quaternion, dtype=np.float32)
    gyro     = np.array(rs.imu_state.gyroscope,  dtype=np.float32)

    ref_q  = motion.joint_pos[frame_idx]
    ref_dq = motion.joint_vel[frame_idx]
    motion_command = np.concatenate([ref_q, ref_dq]).astype(np.float32)

    real_t = torso_quat(pelvis_q,  q_real[12], q_real[13], q_real[14])
    ref_t  = torso_quat(motion.root_quat[frame_idx], ref_q[12], ref_q[13], ref_q[14])
    tgt_t  = quat_mul(init_q, ref_t)
    q_rel_rot = quat_mul(quat_inv(tgt_t), real_t)
    R = quat_to_rot(q_rel_rot).T
    anchor_ori = np.array([R[0,0],R[0,1],R[1,0],R[1,1],R[2,0],R[2,1]], dtype=np.float32)

    q_rel_rl = q_real - DEFAULT_Q
    return np.concatenate([motion_command, anchor_ori, gyro, q_rel_rl, dq_real, last_action]).astype(np.float32)


# ===== MAIN =====
def main():
    global robot_state

    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(state_handler, 10)

    init_motor_cmd()
    threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True).start()

    # ===== GAMEPAD / PYGAME =====
    pygame.init()
    screen = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("G1 Humanoid HUD")
    font_title = pygame.font.SysFont("monospace", 18, bold=True)
    font_bold  = pygame.font.SysFont("monospace", 14, bold=True)
    font_small = pygame.font.SysFont("monospace", 12)
    font       = pygame.font.SysFont("monospace", 14)
    joystick = None
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0); joystick.init()
        print(f"[JOY] {joystick.get_name()}")
    else:
        print("[JOY] Không có gamepad — dùng bàn phím (WASD/QE)")

    # ===== NẠP TẤT CẢ POLICY =====
    print("[LOAD] Đang nạp các policy...")
    sessions = {}
    motions  = {}
    
    # Cấu hình ONNX Runtime tối ưu cho thời gian thực (tránh sinh quá nhiều luồng tranh chấp CPU)
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    opts.inter_op_num_threads = 1

    for i, m in enumerate(MODES):
        import os
        policy_path = m["policy"]
        if not os.path.exists(policy_path):
            alt_path = os.path.join(os.path.dirname(__file__), policy_path)
            if os.path.exists(alt_path):
                policy_path = alt_path
            else:
                base_name = os.path.basename(policy_path)
                if os.path.exists(base_name):
                    policy_path = base_name
                else:
                    print(f"  [!] Bỏ qua mode {i+1} ({m['name']}): không tìm thấy {m['policy']}")
                    sessions[i] = None; motions[i] = None; continue
        sessions[i] = ort.InferenceSession(policy_path, sess_options=opts, providers=['CPUExecutionProvider'])
        if m["npz"]:
            try:
                motions[i] = MotionData(m["npz"])
            except Exception as e:
                print(f"  [!] NPZ lỗi: {e}"); motions[i] = None; sessions[i] = None; continue
        else:
            motions[i] = None
        print(f"  [OK] Mode {i+1}: {m['name']}")

    # ===== CHỜ SIMULATOR =====
    print("[WAIT] Chờ dữ liệu từ simulator...")
    while True:
        with state_lock:
            if robot_state is not None: break
        time.sleep(0.01)
    print("[OK] Kết nối thành công!")

    # ===== STATE =====
    current_mode  = 0
    last_action   = np.zeros(29, dtype=np.float32)
    smoothed_cmds = np.zeros(3, dtype=np.float32)
    gait_time     = 0.0
    gait_scale    = 1.0
    CTRL_DT       = 0.02
    ALPHA         = 0.1

    # Mimic state
    mimic_t_start = time.perf_counter()
    mimic_init_q  = np.array([1,0,0,0], dtype=np.float32)

    fall_detector = G1FallDetector()
    arm_player    = ArmCSVPlayer()
    imu_logger    = IMULogger() if _HAS_IMU_LOGGER else None

    btn_cooldown = {}  # button → last press time

    def btn(idx, cooldown=0.4):
        if joystick is None: return False
        try:
            if joystick.get_button(idx):
                now = time.perf_counter()
                if now - btn_cooldown.get(idx, 0) > cooldown:
                    btn_cooldown[idx] = now; return True
        except: pass
        return False

    def switch_mode(new_mode):
        nonlocal current_mode, last_action, gait_time, gait_scale
        nonlocal mimic_t_start, mimic_init_q, smoothed_cmds
        if sessions[new_mode] is None:
            print(f"[SKIP] Mode {new_mode+1} chưa sẵn sàng (thiếu policy/npz).")
            return
        current_mode = new_mode
        last_action[:]   = 0.0
        smoothed_cmds[:] = 0.0
        gait_time  = 0.0
        gait_scale = 1.0
        print(f"\n>>> SWITCH → [{current_mode+1}] {MODES[current_mode]['name']} <<<\n")

        if motions[current_mode] is not None:
            # Tính init_quat cho mimic
            with state_lock:
                rs = robot_state
            if rs is not None:
                pelvis_q = np.array(rs.imu_state.quaternion, dtype=np.float32)
                q_real = np.array([rs.motor_state[i].q for i in range(29)], dtype=np.float32)
                mot = motions[current_mode]
                robot_t0 = torso_quat(pelvis_q, q_real[12], q_real[13], q_real[14])
                ref_t0   = torso_quat(mot.root_quat[0], mot.joint_pos[0,12],
                                      mot.joint_pos[0,13], mot.joint_pos[0,14])
                mimic_init_q = quat_mul(yaw_quat(robot_t0), quat_inv(yaw_quat(ref_t0)))
            mimic_t_start = time.perf_counter()

    switch_mode(0)  # Bắt đầu ở locomotion

    def do_sim_reset():
        """Teleport robot về spawn point trong simulator, rồi khôi phục trạng thái."""
        nonlocal last_action, smoothed_cmds, gait_time, gait_scale
        # 1. Gửi mode=0xFF → simulator teleport robot về vị trí ban đầu
        with cmd_lock:
            cmd.motor_cmd[0].mode = 0xFF
        time.sleep(0.05)  # chờ publisher gửi đi (chu kỳ 2ms)
        # 2. Khôi phục motor về mặc định
        with cmd_lock:
            for i in range(29):
                cmd.motor_cmd[i].mode = 0x01
                cmd.motor_cmd[i].q    = float(DEFAULT_Q[i])
                cmd.motor_cmd[i].dq   = 0.0
                cmd.motor_cmd[i].tau  = 0.0
                cmd.motor_cmd[i].kp   = float(KP[i])
                cmd.motor_cmd[i].kd   = float(KD[i])
        # 3. Xóa trạng thái nội bộ
        last_action[:]   = 0.0
        smoothed_cmds[:] = 0.0
        gait_time  = 0.0
        gait_scale = 1.0
        fall_detector.reset()
        time.sleep(0.5)

    last_step_time = time.perf_counter()
    step_counter = 0

    try:
        while True:
            step_counter += 1
            # ===== SỰ KIỆN PYGAME (chỉ trigger 1 lần per nhấn) =====
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    k = event.key
                    if k == pygame.K_ESCAPE:
                        return
                    # --- Phím 0: Teleport về spawn + Locomotion ---
                    elif k == pygame.K_0:
                        print(">>> [0] TELEPORT về spawn + Locomotion")
                        do_sim_reset(); switch_mode(0)
                    # --- Chuyển mode ---
                    elif k == pygame.K_1:
                        switch_mode(0)
                    elif k == pygame.K_2:
                        switch_mode(1)
                    elif k == pygame.K_3:
                        switch_mode(2)
                    # --- Reset (R): Teleport, giữ nguyên mode ---
                    elif k == pygame.K_r:
                        print(">>> [R] TELEPORT về spawn")
                        do_sim_reset()
                    # --- Khớp tay CSV (chỉ chạy ở Locomotion mode) ---
                    elif k == pygame.K_v and current_mode == 0:
                        arm_player.trigger("vaytay")
                    elif k == pygame.K_h and current_mode == 0:
                        arm_player.trigger("traitim")
                    elif k == pygame.K_b and current_mode == 0:
                        arm_player.trigger("battay")

            # ===== NÚT GAMEPAD =====
            if btn(7):  # START → next mode
                switch_mode((current_mode + 1) % NUM_MODES)
            if btn(6):  # BACK → prev mode
                switch_mode((current_mode - 1) % NUM_MODES)
            if btn(2):  # X → thoát
                print(">>> Thoát."); break
            if btn(1):  # B → reset
                print(">>> [B] TELEPORT về spawn (gamepad)")
                do_sim_reset()
            if btn(3):  # Y → vẫy tay
                if current_mode == 0:
                    arm_player.trigger("vaytay")
            if btn(0):  # A → trái tim
                if current_mode == 0:
                    arm_player.trigger("traitim")
            if btn(4):  # LB → bắt tay
                if current_mode == 0:
                    arm_player.trigger("battay")



            # ===== ROBOT STATE =====
            with state_lock:
                rs = robot_state
                if rs is None: time.sleep(0.002); continue

            step_start = time.perf_counter()

            # ===== AXES (liên tục) =====
            raw_vx = raw_vy = raw_yaw = 0.0
            keys = pygame.key.get_pressed()
            if joystick:
                def dz(v): return 0.0 if abs(v) < 0.15 else v
                raw_vx  = -dz(joystick.get_axis(1)) * 1.0
                raw_vy  = -dz(joystick.get_axis(0)) * 0.5
                raw_yaw = -dz(joystick.get_axis(3)) * 1.0
            else:
                raw_vx  = 1.0 if keys[pygame.K_w] else (-0.5 if keys[pygame.K_s] else 0.0)
                raw_vy  = 0.5 if keys[pygame.K_a] else (-0.5 if keys[pygame.K_d] else 0.0)
                raw_yaw = 1.0 if keys[pygame.K_q] else (-1.0 if keys[pygame.K_e] else 0.0)

            smoothed_cmds = ALPHA * np.array([raw_vx, raw_vy, raw_yaw], dtype=np.float32) \
                          + (1.0 - ALPHA) * smoothed_cmds

            # ===== FALL DETECTION =====

            gyro_v  = np.array(rs.imu_state.gyroscope,  dtype=np.float32)
            accel_v = np.array(rs.imu_state.accelerometer, dtype=np.float32)
            quat_v  = np.array(rs.imu_state.quaternion, dtype=np.float32)
            proj_g  = compute_projected_gravity(quat_v)
            if imu_logger: imu_logger.log_step(step_start, proj_g, gyro_v, accel_v)
            is_fallen, is_lay_down, reasons = fall_detector.check(proj_g, gyro_v, accel_v)
            if fall_detector.is_fallen:
                with cmd_lock:
                    for i in range(29):
                        cmd.motor_cmd[i].kp  = 0.0
                        cmd.motor_cmd[i].kd  = 0.0 if is_lay_down else float(KD[i])
                        cmd.motor_cmd[i].tau = 0.0
                time.sleep(0.02); continue

            sess = sessions[current_mode]
            if sess is None:
                time.sleep(0.02); continue

            input_name = sess.get_inputs()[0].name

            # ===== BUILD OBS & INFER =====
            if MODES[current_mode]["npz"] is None:
                # --- LOCOMOTION ---
                dt = min(max(step_start - last_step_time, 0.0), 0.05)
                if abs(raw_vx) > 0.05 or abs(raw_vy) > 0.05 or abs(raw_yaw) > 0.05:
                    gait_time += CTRL_DT
                    gait_scale = min(1.0, gait_scale + CTRL_DT / 0.3)
                else:
                    remainder = gait_time % 0.6
                    if 0.02 < remainder < 0.58:
                        gait_time += CTRL_DT
                        gait_scale = min(1.0, gait_scale + CTRL_DT / 0.3)
                    else:
                        gait_time = round(gait_time / 0.6) * 0.6
                        gait_scale = max(0.0, gait_scale - CTRL_DT / 0.3)
                phase_ratio = (gait_time % 0.6) / 0.6
                gait_phase  = np.array([math.sin(2*math.pi*phase_ratio),
                                        math.cos(2*math.pi*phase_ratio)], dtype=np.float32) * gait_scale
                obs = build_loco_obs(rs, smoothed_cmds, gait_phase, arm_player, last_action)
            else:
                # --- MIMIC ---
                t_cur     = step_start - mimic_t_start
                frame_idx = int(t_cur / motions[current_mode].dt) % motions[current_mode].num_frames
                obs = build_mimic_obs(rs, motions[current_mode], frame_idx, mimic_init_q, last_action)

            action = sess.run(None, {input_name: np.expand_dims(obs, 0)})[0][0]
            last_action = action.copy()

            target_q = DEFAULT_Q + action * ACTION_SCALE

            if MODES[current_mode]["npz"] is None:
                target_q = arm_player.update_and_blend(CTRL_DT, target_q)

            with cmd_lock:
                for i in range(29):
                    cmd.motor_cmd[i].q = float(target_q[i])

            # ===== HUD PYGAME =====
            if step_counter % 5 == 0:
                BG_COLOR = (15, 15, 25)
                BANNER_COLOR = (25, 25, 38)
                BORDER_COLOR = (40, 40, 60)
                TEXT_WHITE = (240, 240, 240)
                TEXT_GRAY = (150, 150, 160)

                # Colors based on current mode
                if current_mode == 0:
                    ACCENT_COLOR = (0, 195, 255)     # Cyan: Locomotion
                elif current_mode == 1:
                    ACCENT_COLOR = (255, 80, 180)    # Magenta: Mimic Dance
                else:
                    ACCENT_COLOR = (150, 255, 0)     # Lime: Mimic Tennis

                # Clear screen
                screen.fill(BG_COLOR)

                # Header Banner
                pygame.draw.rect(screen, BANNER_COLOR, (0, 0, 400, 75))
                pygame.draw.line(screen, ACCENT_COLOR, (0, 75), (400, 75), 2)
                screen.blit(font_title.render("UNITREE G1 HUMAN_STATE", True, TEXT_WHITE), (15, 12))
                
                mode_lbl = f"MODE {current_mode+1}/{NUM_MODES}: {MODES[current_mode]['name']}"
                screen.blit(font_bold.render(mode_lbl, True, ACCENT_COLOR), (15, 42))

                # Left Widget: Input Stick (W-S-A-D / Gamepad Axis)
                is_locomotion = MODES[current_mode]["npz"] is None
                joy_center_x = 200 if is_locomotion else 110

                pygame.draw.circle(screen, (30, 30, 45), (joy_center_x, 185), 55)
                pygame.draw.circle(screen, BORDER_COLOR, (joy_center_x, 185), 55, 1)
                pygame.draw.circle(screen, BORDER_COLOR, (joy_center_x, 185), 28, 1)
                pygame.draw.line(screen, BORDER_COLOR, (joy_center_x - 55, 185), (joy_center_x + 55, 185), 1)
                pygame.draw.line(screen, BORDER_COLOR, (joy_center_x, 185 - 55), (joy_center_x, 185 + 55), 1)
                
                # Map vx (smoothed_cmds[0]) to vertical axis, vy (smoothed_cmds[1]) to horizontal axis
                # Max expected magnitude: vx in [-0.5, 1.0], vy in [-0.5, 0.5]
                # Normalize to [-1, 1] relative to scale factor
                stick_dx = -(smoothed_cmds[1] / 0.5) * 50
                stick_dy = -(smoothed_cmds[0] / 1.0) * 50
                
                dot_x = int(joy_center_x + stick_dx)
                dot_y = int(185 + stick_dy)
                
                # Clamp dot within visual stick radius (50 pixels)
                dist = math.hypot(dot_x - joy_center_x, dot_y - 185)
                if dist > 50:
                    angle = math.atan2(dot_y - 185, dot_x - joy_center_x)
                    dot_x = int(joy_center_x + 50 * math.cos(angle))
                    dot_y = int(joy_center_x + 50 * math.sin(angle))
                    
                pygame.draw.circle(screen, ACCENT_COLOR, (dot_x, dot_y), 7)
                pygame.draw.circle(screen, (255, 255, 255), (dot_x, dot_y), 7, 1)
                
                # Center the text label relative to the joystick
                lbl_surface = font_small.render("VIRTUAL JOYSTICK", True, TEXT_GRAY)
                screen.blit(lbl_surface, (joy_center_x - lbl_surface.get_width() // 2, 252))

                # Right Widget: Motion progress (only drawn in Mimic modes)
                if not is_locomotion:
                    pygame.draw.circle(screen, (30, 30, 45), (290, 185), 55)
                    pygame.draw.circle(screen, BORDER_COLOR, (290, 185), 55, 1)

                    # Mimic Motion: draw rotating progress indicator
                    progress_angle = (frame_idx / motions[current_mode].num_frames) * 2 * math.pi
                    hand_x = int(290 + 48 * math.sin(progress_angle))
                    hand_y = int(185 - 48 * math.cos(progress_angle))
                    pygame.draw.line(screen, BORDER_COLOR, (290, 185), (hand_x, hand_y), 1)
                    pygame.draw.line(screen, ACCENT_COLOR, (290, 185), (hand_x, hand_y), 3)
                    pygame.draw.circle(screen, ACCENT_COLOR, (290, 185), 5)
                    
                    f_txt = f"Frame: {frame_idx+1}/{motions[current_mode].num_frames}"
                    f_surface = font_small.render(f_txt, True, TEXT_WHITE)
                    screen.blit(f_surface, (290 - f_surface.get_width() // 2, 252))

                # Bottom Info Panel
                pygame.draw.rect(screen, BANNER_COLOR, (0, 275, 400, 125))
                pygame.draw.line(screen, BORDER_COLOR, (0, 275), (400, 275), 1)

                # Robot stability status
                if fall_detector.is_fallen:
                    blink = int(time.time() * 5) % 2
                    status_color = (255, 40, 40) if blink else (150, 20, 20)
                    status_text = "FALL DETECTED! (Press 0/R to Reset)"
                else:
                    status_color = (0, 220, 100)
                    status_text = "STABLE"
                    
                screen.blit(font_small.render("ROBOT STATE:", True, TEXT_GRAY), (20, 288))
                screen.blit(font_bold.render(status_text, True, status_color), (120, 288))

                # Arm Action status
                active_act = arm_player.active_motion
                act_text = active_act.upper() if active_act else "NONE"
                act_color = (0, 220, 255) if active_act else TEXT_GRAY
                screen.blit(font_small.render("ARM ACTION:", True, TEXT_GRAY), (20, 308))
                screen.blit(font_bold.render(act_text, True, act_color), (120, 308))

                # Help legend
                screen.blit(font_small.render("Controls: [0/R] Reset  [1-3] Mode  [X] Quit", True, TEXT_GRAY), (20, 338))
                screen.blit(font_small.render("          Arm CSV Actions: [V] Wave  [H] Heart  [B] Shake", True, TEXT_GRAY), (20, 358))

                pygame.display.flip()

            last_step_time = step_start
            elapsed = time.perf_counter() - step_start
            if CTRL_DT - elapsed > 0:
                time.sleep(CTRL_DT - elapsed)
            else:
                # Log loop delay!
                delay = elapsed - CTRL_DT
                print(f"[WARN] Control loop delay: {delay*1000:.2f} ms (Total step time: {elapsed*1000:.2f} ms)")

    except KeyboardInterrupt:
        print("\n>>> Ctrl+C — thoát.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
