import time
import threading
import numpy as np
import onnxruntime as ort
import pygame
import sys
import yaml
import os

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
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,  # Chân trái (6)
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,  # Chân phải (6)
    100.0, 100.0,                            # Eo (2: waist_roll, waist_yaw)
    40.0, 40.0, 20.0, 20.0, 20.0,            # Tay trái (5)
    40.0, 40.0, 20.0, 20.0, 20.0             # Tay phải (5)
], dtype=np.float32)

KD_ARRAY = np.array([
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,            # Chân trái (6)
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,            # Chân phải (6)
    2.0, 2.0,                                # Eo (2)
    2.0, 2.0, 1.0, 1.0, 1.0,                 # Tay trái (5)
    2.0, 2.0, 1.0, 1.0, 1.0                  # Tay phải (5)
], dtype=np.float32)

DEFAULT_Q = np.array([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # Chân trái (6)
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,          # Chân phải (6)
    0.0, 0.0,                                # Eo (2)
    0.35, 0.18, 0.0, 0.87, 0.0,              # Tay trái (5)
    0.35, -0.18, 0.0, 0.87, 0.0              # Tay phải (5)
], dtype=np.float32)

NUM_JOINTS = 24

# ACTION_SCALE theo r1_constants.py: 0.25 * effort_limit / stiffness
ACTION_SCALE = np.array([
    0.150, 0.150, 0.150, 0.150, 0.3125, 0.3125,  # Chân trái: hip(60/100)×4, ankle(50/40)×2
    0.150, 0.150, 0.150, 0.150, 0.3125, 0.3125,  # Chân phải
    0.150, 0.150,                                  # Eo: waist(60/100)
    0.375, 0.375, 0.4125, 0.4125, 0.4125,         # Tay trái: shoulder_p/r(60/40), yaw/elbow/wrist(33/20)
    0.375, 0.375, 0.4125, 0.4125, 0.4125,         # Tay phải
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
    Chiếu vector trọng lực thế giới [0, 0, -1] vào hệ tọa độ thân robot.
    Quy ước Unitree SDK: quat = [w, x, y, z] (body→world rotation).
    
    Công thức: g_body = R(q)^T @ [0, 0, -1]
    gx = -2*(w*y + x*z)
    gy =  2*(w*x - y*z)
    gz =  2*(x**2 + y**2) - 1
    """
    w, x, y, z = quat
    gx = -2.0 * (w * y + x * z)
    gy =  2.0 * (w * x - y * z)
    gz =  2.0 * (x**2 + y**2) - 1.0
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

    # Ánh xạ khớp cho Simulator (24 khớp):
    # Simulator: 12=waist_roll, 13=waist_yaw
    # Policy:    12=waist_yaw,  13=waist_roll
    # Chỉ cần tráo đổi index 12 <-> 13, các khớp khác map trực tiếp.
    JOINT_IDS_MAP = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                     13, 12,
                     14, 15, 16, 17, 18,
                     19, 20, 21, 22, 23]

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

    print("Đang nạp model ONNX...")
    session = ort.InferenceSession("policy_r1_270.onnx", providers=['CPUExecutionProvider'])
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
        "waist_yaw", "waist_roll",
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

    print(f"--- R1 SẴN SÀNG ({NUM_JOINTS} KHỚP) ---")
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
    
            # Ghi log vòng lặp tròn (O(1) memory)
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
            
            # Đồng bộ với deploy.yaml / observations.h: khi không di chuyển (cmd_norm < 0.1) thì gait_phase = [0, 0]
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
            
            # Lấy pelvis_z động từ SportModeState để tính height_scan
            pelvis_z = 0.76  # fallback mặc định
            with sport_state_lock:
                if sport_state is not None:
                    pelvis_z = sport_state.position[2]
            
            height_scan_obs = pelvis_z * 0.2  # scale = 1/max_distance = 1/5.0, ground_z=0

            # Tự động padding phần còn lại với height_scan_obs
            remaining_dim = expected_dim - len(obs_base)
            if remaining_dim > 0:
                height_scan = height_scan_obs * np.ones(remaining_dim, dtype=np.float32)
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
                    hip_L = action_policy[0]   # left_hip_pitch action
                    hip_R = action_policy[6]   # right_hip_pitch action
                    tq_L  = target_q_arr[0]    # left_hip_pitch target
                    tq_R  = target_q_arr[6]    # right_hip_pitch target
                    gz    = projected_gravity[2]
                    gx    = projected_gravity[0]
                    mode0 = cmd.motor_cmd[JOINT_IDS_MAP[0]].mode
                    print(f"[DBG s={step:4d}] gz={gz:.3f} gx={gx:.3f} | "
                          f"act_hipL={hip_L:.3f} act_hipR={hip_R:.3f} | "
                          f"tq_hipL={tq_L:.3f} tq_hipR={tq_R:.3f} | mode={mode0:#04x}")
            except Exception as e:
                print(f"\n[LỖI NGHIÊM TRỌNG] Lỗi khi tính toán Action: {e}")
                print(f"Kích thước obs_tensor: {obs_tensor.shape}")
                print(f"Kích thước action trả về: {getattr(action_policy, 'shape', 'N/A') if 'action_policy' in locals() else 'N/A'}")
                import sys
                sys.exit(1)
    
            # Ghi log (non-blocking: chỉ put vào queue ~100ns)
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
