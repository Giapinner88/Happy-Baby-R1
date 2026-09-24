import time
import threading
import numpy as np
import onnxruntime as ort
import pygame
import sys
import yaml
import os
import mujoco

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.utils.crc import CRC

from state_logger import SimStateLogger
from fall_detector.detector import G1FallDetector
from fall_detector.logger import IMULogger

robot_state = None
_got_first_state = False
state_lock = threading.Lock()

sport_state = None
sport_state_lock = threading.Lock()

def sport_state_handler(msg: SportModeState_):
    global sport_state
    with sport_state_lock:
        sport_state = msg

cmd = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()

# --- KHAI BÁO CÁC HẰNG SỐ ĐỘNG LỰC HỌC CỦA R1 (24 KHỚP) CHUẨN TỪ TRAINING ---
KP_ARRAY = np.array([
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,   # Chân trái (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,   # Chân phải
    100.0, 100.0,                              # Eo (waist_roll, waist_yaw)
    40.0, 40.0, 20.0, 20.0, 20.0,             # Tay trái (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
    40.0, 40.0, 20.0, 20.0, 20.0              # Tay phải
], dtype=np.float32)

KD_ARRAY = np.array([
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,             # Chân trái
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,             # Chân phải
    2.0, 2.0,                                  # Eo
    2.0, 2.0, 1.0, 1.0, 1.0,                  # Tay trái
    2.0, 2.0, 1.0, 1.0, 1.0                   # Tay phải
], dtype=np.float32)

DEFAULT_Q = np.array([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # Chân trái (6)
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # Chân phải (6)
    0.0, 0.0,                                # Eo (2)
    0.35, 0.18, 0.0, 0.87, 0.0,              # Tay trái (5)
    0.35, -0.18, 0.0, 0.87, 0.0              # Tay phải (5)
], dtype=np.float32)

NUM_JOINTS = 24

# ACTION_SCALE trích xuất từ training: 0.25 * effort_limit / stiffness
ACTION_SCALE = np.array([
    0.22, 0.22, 0.22, 0.3475, 0.3125, 0.3125, # Chân trái (6)
    0.22, 0.22, 0.22, 0.3475, 0.3125, 0.3125, # Chân phải (6)
    0.22, 0.125,                              # Eo (2)
    0.15625, 0.15625, 0.15625, 0.15625, 0.15625, # Tay trái (5)
    0.15625, 0.15625, 0.15625, 0.15625, 0.15625  # Tay phải (5)
], dtype=np.float32)

def state_handler(msg: LowState_):
    global robot_state, _got_first_state
    with state_lock:
        robot_state = msg
    if not _got_first_state:
        _got_first_state = True
        print("Đã nhận LowState từ simulator (DDS OK).")

def dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        time.sleep(0.002) 

def compute_projected_gravity(quat):
    """
    Biến đổi vector trọng lực thế giới [0, 0, -1] về hệ tọa độ cục bộ của thân robot.
    Quy ước Unitree SDK: quat = [w, x, y, z]
    Công thức: R^T @ [0,0,-1] = [2(wy-xz), -2(yz+wx), 2(x^2+y^2)-1]
    """
    w, x, y, z = quat

    gx = 2 * (w * y - x * z)
    gy = -2 * (y * z + w * x)
    gz = 2 * (x**2 + y**2) - 1

    return np.array([gx, gy, gz], dtype=np.float32)



def main():
    global robot_state, cmd
    
    ChannelFactoryInitialize(1, "lo") 
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)
    sub_sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub_sport.Init(sport_state_handler, 10)

    # R1 policy/XML/SDK cùng thứ tự eo: 12=roll, 13=yaw.
    JOINT_IDS_MAP = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26]

    # Khởi tạo tư thế mặc định của R1 (24 khớp)
    with cmd_lock:
        for i in range(NUM_JOINTS):
            sdk_idx = JOINT_IDS_MAP[i]
            cmd.motor_cmd[sdk_idx].mode = 0x01
            cmd.motor_cmd[sdk_idx].q = float(DEFAULT_Q[i])
            cmd.motor_cmd[sdk_idx].dq = 0.0
            cmd.motor_cmd[sdk_idx].tau = 0.0
            cmd.motor_cmd[sdk_idx].kp = float(KP_ARRAY[i])
            cmd.motor_cmd[sdk_idx].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    # --- KHỞI TẠO MÔ HÌNH ĐỊA HÌNH CHO RAY CASTING CỤC BỘ ---
    print("Đang nạp mô hình địa hình MuJoCo cho Ray Casting...")
    mj_model = None
    mj_data = None
    try:
        mj_model = mujoco.MjModel.from_xml_path("../unitree_robots/r1/scene.xml")
        mj_data = mujoco.MjData(mj_model)
        print("Đã nạp thành công mô hình địa hình!")
    except Exception as e:
        print(f"CẢNH BÁO: Không thể nạp mô hình địa hình: {e}")

    # Lưới quét 1.6m x 1.0m, độ phân giải 0.1m -> 17 x 11 = 187 điểm
    xs = np.arange(-0.8, 0.8 + 1e-5, 0.1)
    ys = np.arange(-0.5, 0.5 + 1e-5, 0.1)
    xv, yv = np.meshgrid(xs, ys, indexing='ij')
    local_grid = np.stack([xv.flatten(), yv.flatten()], axis=-1)

    def is_robot_body(body_id):
        if mj_model is None:
            return False
        pelvis_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        curr_id = body_id
        while curr_id > 0:
            if curr_id == pelvis_id:
                return True
            curr_id = mj_model.body_parentid[curr_id]
        return False

    def cast_ray_to_ground(ray_start, ray_dir):
        if mj_model is None or mj_data is None:
            return -1.0
        current_start = ray_start.copy()
        geomid = np.zeros(1, dtype=np.int32)
        total_dist = 0.0
        for _ in range(5):  # Tối đa 5 bước để xuyên qua các bộ phận của robot nếu va chạm
            dist = mujoco.mj_ray(mj_model, mj_data, current_start, ray_dir, None, 1, -1, geomid)
            if dist < 0:
                return -1.0
            hit_geom = geomid[0]
            body_id = mj_model.geom_bodyid[hit_geom]
            if not is_robot_body(body_id):
                return total_dist + dist
            else:
                # Tiến điểm xuất phát tia qua bề mặt vừa chạm để quét tiếp xuống dưới
                step_dist = dist + 1e-3
                current_start += step_dist * ray_dir
                total_dist += step_dist
        return -1.0

    print("Đang nạp model ONNX...")
    policy_path = os.path.join(os.path.dirname(__file__), "policy", "policy_r1_270.onnx")
    if not os.path.exists(policy_path) and os.path.exists("policy_r1_270.onnx"):
        policy_path = "policy_r1_270.onnx"
    session = ort.InferenceSession(policy_path, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    expected_dim = session.get_inputs()[0].shape[1]
    print(f"Model nạp thành công! Input shape dimension: {expected_dim}")
    
    pygame.init()
    pygame.display.set_mode((300, 200))
    pygame.display.set_caption('GAMEPAD CONTROL R1')

    # KHỞI TẠO GAMEPAD
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Hệ thống điều khiển: Gamepad ({joystick.get_name()})")
    else:
        print("CẢNH BÁO: Không tìm thấy Gamepad. Chuyển về chế độ bàn phím.")

    last_action_policy = np.zeros(NUM_JOINTS, dtype=np.float32)
    smoothed_commands = np.zeros(3, dtype=np.float32)
    alpha = 0.1 # Hệ số làm mượt vận tốc

    R1_JOINT_NAMES = [
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
        "waist_roll", "waist_yaw",
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll"
    ]

    gait_time = 0.0
    last_step_time = time.perf_counter()
    last_print_time = 0.0
    gait_scale = 1.0
    
    logger = SimStateLogger(__file__, joint_names=R1_JOINT_NAMES)
    step = 0
    t0 = time.perf_counter()

    fall_detector = G1FallDetector()
    imu_logger = IMULogger()

    print(f"--- R1 SẴN SÀNG ({NUM_JOINTS} KHỚP - ROUGH TERRAIN RUNNER) ---")
    print("Nhấn phím W/A/S/D/Q/E để điều khiển.")
    print("Nhấn nút X trên Gamepad (hoặc phím ESC) để thoát.")
    print("Nhấn nút R trên Bàn phím (hoặc nút B/1 trên Gamepad) để Reset mô phỏng.")

    try:
        while True:
            with state_lock:
                have_state = robot_state is not None
                if have_state:
                    # Chờ cho đến khi nhận được trạng thái khớp thực tế (không phải toàn 0 khi khởi động)
                    q_sum = sum(abs(robot_state.motor_state[JOINT_IDS_MAP[i]].q) for i in range(NUM_JOINTS))
                    if q_sum < 0.01:
                        have_state = False
            if not have_state:
                time.sleep(0.002)
                last_step_time = time.perf_counter()
                continue
                
            step_start = time.perf_counter()
            dt = step_start - last_step_time
            last_step_time = step_start
            
            if dt < 0.0:
                dt = 0.0
            elif dt > 0.05:
                dt = 0.05
            
            CTRL_DT = 0.02  # Tần số 50 Hz
            
            # --- XỬ LÝ SỰ KIỆN THOÁT ---
            exit_pressed = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_pressed = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_x:
                        exit_pressed = True

            if joystick is not None:
                try:
                    if joystick.get_button(2): # Nút X Xbox
                        exit_pressed = True
                except Exception:
                    pass

            if exit_pressed:
                print("\n>>> NHẬN LỆNH THOÁT. Đang đóng chương trình và lưu log...")
                break
                
            # --- ĐỌC TÍN HIỆU ĐIỀU KHIỂN ---
            raw_vx = 0.0
            raw_vy = 0.0
            raw_yaw = 0.0
    
            if joystick is not None:
                def apply_deadzone(value, threshold=0.15):
                    return 0.0 if abs(value) < threshold else value
    
                axis_left_x = apply_deadzone(joystick.get_axis(0)) 
                axis_left_y = apply_deadzone(joystick.get_axis(1))  
                axis_right_x = apply_deadzone(joystick.get_axis(3)) 
    
                raw_vx = -axis_left_y * 1.0  
                raw_vy = -axis_left_x * 0.5  
                raw_yaw = -axis_right_x * 1.0 
            else:
                keys = pygame.key.get_pressed()
                raw_vx = 1.0 if keys[pygame.K_w] else (-0.5 if keys[pygame.K_s] else 0.0)
                raw_vy = 0.5 if keys[pygame.K_a] else (-0.5 if keys[pygame.K_d] else 0.0)
                raw_yaw = 1.0 if keys[pygame.K_q] else (-1.0 if keys[pygame.K_e] else 0.0)
    
            target_commands = np.array([raw_vx, raw_vy, raw_yaw], dtype=np.float32)
            smoothed_commands = alpha * target_commands + (1.0 - alpha) * smoothed_commands
            
            # --- XỬ LÝ SỰ KIỆN RESET ---
            keys = pygame.key.get_pressed()
            reset_pressed = keys[pygame.K_r]
            if joystick is not None:
                try:
                    reset_pressed = reset_pressed or joystick.get_button(1)
                except Exception:
                    pass

            if reset_pressed:
                print(">>> RESET TOÀN BỘ TRẠNG THÁI & KHỞI ĐỘNG LẠI SIMULATOR...")
                
                # 1. Gửi lệnh reset đặc biệt sang simulator (DDS)
                with cmd_lock:
                    cmd.motor_cmd[0].mode = 0xFF
                
                # Chờ publisher thread gửi đi
                time.sleep(0.05)
                
                # 2. Khôi phục lại trạng thái của client
                fall_detector.reset()
                imu_logger.reset()
                last_action_policy = np.zeros(NUM_JOINTS, dtype=np.float32)
                smoothed_commands = np.zeros(3, dtype=np.float32)
                gait_time = 0.0
                gait_scale = 1.0
                step = 0
                t0 = time.perf_counter()
                last_print_time = 0.0
                
                # Khôi phục KP, KD, Q và MODE mặc định gửi xuống motor (24 khớp)
                with cmd_lock:
                    for i in range(NUM_JOINTS):
                        sdk_idx = JOINT_IDS_MAP[i]
                        cmd.motor_cmd[sdk_idx].mode = 0x01
                        cmd.motor_cmd[sdk_idx].q = float(DEFAULT_Q[i])
                        cmd.motor_cmd[sdk_idx].dq = 0.0
                        cmd.motor_cmd[sdk_idx].tau = 0.0
                        cmd.motor_cmd[sdk_idx].kp = float(KP_ARRAY[i])
                        cmd.motor_cmd[sdk_idx].kd = float(KD_ARRAY[i])
                        
                time.sleep(0.5) # Chống dội phím
                last_step_time = time.perf_counter()
                continue
            
            # --- LOGIC ĐIỀU KHIỂN CHU KỲ BƯỚC ---
            if abs(smoothed_commands[0]) > 0.01 or abs(smoothed_commands[1]) > 0.01 or abs(smoothed_commands[2]) > 0.01:
                now = time.perf_counter()
                if now - last_print_time > 0.5:
                    print(f"Đang gửi lệnh: Vx={smoothed_commands[0]:.2f}, Vy={smoothed_commands[1]:.2f}, Yaw={smoothed_commands[2]:.2f}")
                    last_print_time = now
                gait_time += dt
                gait_scale = min(1.0, gait_scale + dt / 0.3)
            else:
                remainder = gait_time % 0.6
                if 0.02 < remainder < 0.58:
                    gait_time += dt
                    gait_scale = min(1.0, gait_scale + dt / 0.3)
                else:
                    gait_time = round(gait_time / 0.6) * 0.6
                    gait_scale = max(0.0, gait_scale - dt / 0.3)
    
            # Đọc LowState từ simulator
            q_current = np.zeros(NUM_JOINTS, dtype=np.float32)
            dq_current = np.zeros(NUM_JOINTS, dtype=np.float32)
            gyro = np.zeros(3, dtype=np.float32)
            quat = np.zeros(4, dtype=np.float32)
            accel = np.zeros(3, dtype=np.float32)
            with state_lock:
                rs = robot_state
                if rs is None:
                    continue
                for i in range(NUM_JOINTS):
                    sdk_idx = JOINT_IDS_MAP[i]
                    q_current[i] = rs.motor_state[sdk_idx].q
                    dq_current[i] = rs.motor_state[sdk_idx].dq
                gyro[:] = np.array(rs.imu_state.gyroscope, dtype=np.float32)
                quat[:] = np.array(rs.imu_state.quaternion, dtype=np.float32)
                accel[:] = np.array(rs.imu_state.accelerometer, dtype=np.float32)
            
            projected_gravity = compute_projected_gravity(quat)
     
            # Ghi log vòng lặp tròn
            imu_logger.log_step(time.perf_counter(), projected_gravity, gyro, accel)
            
            # --- KIỂM TRA NGÃ (FALL DETECTION) ---
            is_fallen, is_lay_down, reasons = fall_detector.check(projected_gravity, gyro, accel)
            
            if is_fallen and len(reasons) > 0:
                print(f"\n!!! PHÁT HIỆN: {' | '.join(reasons)} !!!")
                imu_logger.trigger_fall_event(time.perf_counter())
                
                with cmd_lock:
                    for i in range(NUM_JOINTS):
                        sdk_idx = JOINT_IDS_MAP[i]
                        cmd.motor_cmd[sdk_idx].kp = 0.0 # Bỏ độ cứng
                        cmd.motor_cmd[sdk_idx].kd = 0.0 # Ngắt toàn bộ momen
                        cmd.motor_cmd[sdk_idx].tau = 0.0
                smoothed_commands[:] = 0.0 # Xóa lệnh chạy
                
            if fall_detector.is_fallen:
                step += 1
                time_until_next = 0.02 - (time.perf_counter() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)
                continue
    
            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)
            gait_phase *= gait_scale
            
            # Đồng bộ với deploy.yaml: khi không di chuyển thì gait_phase = [0, 0]
            cmd_norm = np.linalg.norm(smoothed_commands)
            if cmd_norm < 0.1:
                gait_phase = np.zeros(2, dtype=np.float32)
    
            q_rel_sdk = q_current - DEFAULT_Q
            
            # Kế thừa trực tiếp vì q_current đã được đọc theo thứ tự của policy
            q_rel_policy = q_rel_sdk
            dq_policy = dq_current
    
            # Tính toán đầu vào obs cho ONNX model
            obs_base_list = [
                gyro,                 # 3
                projected_gravity,    # 3
                smoothed_commands,    # 3
                gait_phase,           # 2
                q_rel_policy,         # NUM_JOINTS
                dq_policy,            # NUM_JOINTS
                last_action_policy    # NUM_JOINTS
            ]
            obs_base = np.concatenate(obs_base_list).astype(np.float32)
            
            # --- TÍNH TOÁN RAY CASTING THỰC TẾ ---
            height_scan = np.zeros(187, dtype=np.float32)
            p_pelvis = np.array([0.0, 0.0, 0.76], dtype=np.float64)
            with sport_state_lock:
                if sport_state is not None:
                    p_pelvis[:] = np.array(sport_state.position, dtype=np.float64)
            
            # Lấy góc yaw từ quaternion của IMU
            w, x, y, z = quat
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
            
            if mj_model is not None and mj_data is not None:
                # Đồng bộ vị trí pelvis của robot vào mô hình ngầm
                mj_data.qpos[0:3] = p_pelvis
                mj_data.qpos[3:7] = quat
                
                # Đồng bộ góc quay khớp vào mj_data.qpos
                for i in range(NUM_JOINTS):
                    joint_name = R1_JOINT_NAMES[i] + "_joint"
                    try:
                        qpos_adr = mj_model.jnt_qposadr[mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)]
                        mj_data.qpos[qpos_adr] = q_current[i]
                    except Exception:
                        pass
                
                # Cập nhật hình học hệ thống
                mujoco.mj_forward(mj_model, mj_data)
                
                ray_dir = np.array([0.0, 0.0, -1.0], dtype=np.float64)
                for idx in range(187):
                    dx, dy = local_grid[idx]
                    # Xoay lưới theo yaw của robot
                    dx_world = dx * np.cos(yaw) - dy * np.sin(yaw)
                    dy_world = dx * np.sin(yaw) + dy * np.cos(yaw)
                    
                    # Điểm bắt đầu bắn tia (nhấc lên 0.5m để chắc chắn nằm trên robot)
                    ray_start = np.array([p_pelvis[0] + dx_world, p_pelvis[1] + dy_world, p_pelvis[2] + 0.5], dtype=np.float64)
                    
                    dist = cast_ray_to_ground(ray_start, ray_dir)
                    if dist >= 0:
                        ground_z = ray_start[2] - dist
                        relative_height = p_pelvis[2] - ground_z
                    else:
                        relative_height = 5.0
                        
                    # Giới hạn & chuẩn hoá
                    relative_height = np.clip(relative_height, -5.0, 5.0)
                    height_scan[idx] = relative_height * 0.2 # scale = 1/5.0
            else:
                # Fallback nếu lỗi load scene
                height_scan[:] = p_pelvis[2] * 0.2
                
            # Ghép phần height_scan vào obs
            remaining_dim = expected_dim - len(obs_base)
            if remaining_dim > 0:
                obs = np.concatenate([obs_base, height_scan])
            else:
                obs = obs_base
                
            obs_tensor = np.expand_dims(obs, axis=0)
            
            try:
                action_policy = session.run(None, {input_name: obs_tensor})[0][0]
                action_policy = action_policy[:NUM_JOINTS]
                last_action_policy = action_policy.copy()
                
                target_q_arr = DEFAULT_Q + action_policy * ACTION_SCALE
                
                with cmd_lock:
                    for i in range(NUM_JOINTS):
                        sdk_idx = JOINT_IDS_MAP[i]
                        cmd.motor_cmd[sdk_idx].q = float(target_q_arr[i])
                
                # DEBUG: In thông tin policy mỗi 50 bước
                if step % 50 == 0:
                    hip_L = action_policy[0]
                    hip_R = action_policy[6]
                    tq_L  = target_q_arr[0]
                    tq_R  = target_q_arr[6]
                    gz    = projected_gravity[2]
                    gx    = projected_gravity[0]
                    mode0 = cmd.motor_cmd[JOINT_IDS_MAP[0]].mode
                    # Tính trung bình chiều cao quét thực tế
                    avg_h = np.mean(height_scan) / 0.2
                    print(f"[DBG s={step:4d}] gz={gz:.3f} gx={gx:.3f} | avg_h={avg_h:.3f}m | "
                          f"act_hipL={hip_L:.3f} act_hipR={hip_R:.3f} | "
                          f"tq_hipL={tq_L:.3f} tq_hipR={tq_R:.3f} | mode={mode0:#04x}")
            except Exception as e:
                print(f"\n[LỖI NGHIÊM TRỌNG] Lỗi khi tính toán Action: {e}")
                print(f"Kích thước obs_tensor: {obs_tensor.shape}")
                import sys
                sys.exit(1)
    
            # Ghi log
            logger.log(
                step   = step,
                t      = step_start - t0,
                target_q   = target_q_arr,
                q          = q_current,
                dq         = dq_current,
                action     = action_policy,
                quat       = quat,
                gyro       = gyro,
                proj_grav  = projected_gravity,
                commands   = smoothed_commands,
                gait_phase = gait_phase,
                gait_scale = gait_scale,
                gait_time  = gait_time,
            )
            step += 1
    
            time_until_next = 0.02 - (time.perf_counter() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        print("Đã tắt script điều khiển.")

if __name__ == '__main__':
    main()
