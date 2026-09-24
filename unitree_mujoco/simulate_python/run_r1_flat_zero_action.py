import time
import threading
import numpy as np
import onnxruntime as ort
import pygame
import math
import sys
import os

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from state_logger import SimStateLogger
from fall_detector.detector import G1FallDetector
from fall_detector.logger import IMULogger

robot_state = None
_got_first_state = False
state_lock = threading.Lock()

cmd = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()

# --- KHAI BÁO CÁC HẰNG SỐ ĐỘNG LỰC HỌC CỦA R1 (24 KHỚP) ---
NUM_JOINTS = 24

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

    print(">>> Đang gửi lệnh reset simulator lúc khởi động...")
    with cmd_lock:
        cmd.motor_cmd[0].mode = 0xFF
    time.sleep(0.2)
    with cmd_lock:
        cmd.motor_cmd[0].mode = 0x01

    print("Đang nạp model ONNX...")
    policy_path = os.path.join(os.path.dirname(__file__), "policy", "policy_r1_flat_2.onnx")
    if not os.path.exists(policy_path) and os.path.exists("policy_r1_flat_2.onnx"):
        policy_path = "policy_r1_flat_2.onnx"
        
    session = ort.InferenceSession(policy_path, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    print(f"Model nạp thành công! Tên input: {input_name}")
    
    pygame.init()
    pygame.display.set_mode((300, 200))
    pygame.display.set_caption('R1 FLAT ROAD CONTROL')

    # KHỞI TẠO GAMEPAD
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Hệ thống điều khiển: Gamepad ({joystick.get_name()})")
    else:
        print("CẢNH BÁO: Không tìm thấy Gamepad. Chuyển về chế độ bàn phím.")

    last_action = np.zeros(NUM_JOINTS, dtype=np.float32)
    smoothed_commands = np.zeros(3, dtype=np.float32)
    alpha = 0.1 # Hệ số làm mượt vận tốc

    gait_time = 0.0
    last_step_time = time.perf_counter()
    last_print_time = 0.0
    gait_scale = 1.0

    R1_JOINT_NAMES = [
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
        "waist_roll", "waist_yaw",
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll"
    ]
    
    logger = SimStateLogger(__file__, joint_names=R1_JOINT_NAMES)
    step = 0
    fall_steps = 0
    t0 = time.perf_counter()

    fall_detector = G1FallDetector()
    imu_logger = IMULogger()

    print("--- R1 FLAT ROAD SẴN SÀNG ---")
    print("Nhấn phím W/A/S/D/Q/E để điều khiển di chuyển.")
    print("Nhấn phím R trên bàn phím (hoặc nút B/1 trên Gamepad) để RESET mô phỏng.")
    print("Nhấn nút X trên Gamepad (hoặc phím ESC / phím X) để thoát.")

    try:
        while True:
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
                    if joystick.get_button(2): # Nút X Xbox (thường là button 2)
                        exit_pressed = True
                except Exception:
                    pass

            if exit_pressed:
                print("\n>>> NHẬN LỆNH THOÁT. Đang đóng và lưu log...")
                break

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
            
            CTRL_DT = 0.02  # 50 Hz
            
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
            
            # --- XỬ LÝ RESET ---
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
                last_action = np.zeros(NUM_JOINTS, dtype=np.float32)
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
                logger.log(
                    step       = step,
                    t          = step_start - t0,
                    target_q   = np.zeros(NUM_JOINTS, dtype=np.float32),
                    q          = q_current,
                    dq         = dq_current,
                    action     = np.zeros(NUM_JOINTS, dtype=np.float32),
                    quat       = quat,
                    gyro       = gyro,
                    proj_grav  = projected_gravity,
                    commands   = smoothed_commands,
                    gait_phase = np.zeros(2, dtype=np.float32),
                    gait_scale = 0.0,
                    gait_time  = 0.0,
                )
                step += 1
                fall_steps += 1
                if fall_steps > 150:
                    print(">>> Đã ghi nhận đủ dữ liệu sau khi ngã (150 steps). Thoát chương trình...")
                    break
                time_until_next = 0.02 - (time.perf_counter() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)
                continue
    
            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)
            gait_phase *= gait_scale
            
            # Đồng bộ với deploy.yaml / observations: khi đứng yên thì gait_phase = [0, 0]
            cmd_norm = np.linalg.norm(smoothed_commands)
            if cmd_norm < 0.1:
                gait_phase = np.zeros(2, dtype=np.float32)
    
            q_rel = q_current - DEFAULT_Q
            
            # Đầu vào 83 chiều
            obs = np.concatenate([
                gyro,                 # 3
                projected_gravity,    # 3
                smoothed_commands,    # 3
                gait_phase,           # 2
                q_rel,                # 24
                dq_current,           # 24
                last_action           # 24
            ]).astype(np.float32)
            
            obs_tensor = np.expand_dims(obs, axis=0)
            
            try:
                action = np.zeros(NUM_JOINTS, dtype=np.float32)
                last_action = action.copy()
                
                target_q_arr = DEFAULT_Q.copy()
                
                with cmd_lock:
                    for i in range(NUM_JOINTS):
                        sdk_idx = JOINT_IDS_MAP[i]
                        cmd.motor_cmd[sdk_idx].q = float(target_q_arr[i])
            except Exception as e:
                print(f"\n[LỖI NGHIÊM TRỌNG] Lỗi khi tính toán Action: {e}")
                print(f"Kích thước obs_tensor: {obs_tensor.shape}")
                print(f"Kích thước action trả về: {getattr(action, 'shape', 'N/A') if 'action' in locals() else 'N/A'}")
                sys.exit(1)
    
            # Ghi log state
            logger.log(
                step       = step,
                t          = step_start - t0,
                target_q   = target_q_arr,
                q          = q_current,
                dq         = dq_current,
                action     = action,
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
