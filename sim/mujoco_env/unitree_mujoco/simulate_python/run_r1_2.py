import time
import threading
import numpy as np
import onnxruntime as ort
import pygame
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

robot_state = None
_got_first_state = False
state_lock = threading.Lock()

cmd = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()

# --- KHAI BÁO CÁC HẰNG SỐ ĐỘNG LỰC HỌC CỦA R1 (24 KHỚP) ---
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

# ACTION_SCALE trích xuất từ metadata chính xác của policy_r1.onnx
ACTION_SCALE = np.array([
    0.150, 0.150, 0.150, 0.150, 0.312, 0.312, # Chân trái (6)
    0.150, 0.150, 0.150, 0.150, 0.312, 0.312, # Chân phải (6)
    0.150, 0.150,                            # Eo (2)
    0.375, 0.375, 0.412, 0.412, 0.412,        # Tay trái (5)
    0.375, 0.375, 0.412, 0.412, 0.412         # Tay phải (5)
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

    # Khởi tạo tư thế mặc định của R1 (24 khớp)
    with cmd_lock:
        for i in range(24):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = float(DEFAULT_Q[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    print("Đang nạp model ONNX...")
    session = ort.InferenceSession("policy_r1_2.onnx", providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    print("Model nạp thành công!")
    
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

    last_action = np.zeros(24, dtype=np.float32)
    smoothed_commands = np.zeros(3, dtype=np.float32)
    alpha = 0.1 # Hệ số làm mượt vận tốc

    gait_time = 0.0
    last_step_time = time.perf_counter()
    last_print_time = 0.0
    gait_scale = 1.0

    print("--- R1 SẴN SÀNG ---")
    print("Nhấn phím W/A/S/D/Q/E để điều khiển.")
    print("Nhấn nút X trên Gamepad (hoặc phím ESC / phím X) để thoát.")

    try:
        while True:
            # --- XỬ LÝ SỰ KIỆN THOÁT (Phải đặt lên đầu để chống treo cửa sổ) ---
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
                print("\n>>> NHẬN LỆNH THOÁT. Đang đóng chương trình...")
                break

            with state_lock:
                have_state = robot_state is not None
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
            
            # --- LOGIC ĐIỀU KHIỂN CHU KỲ BƯỚC (GAIT LOOP) ---
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
            q_current = np.zeros(24, dtype=np.float32)
            dq_current = np.zeros(24, dtype=np.float32)
            gyro = np.zeros(3, dtype=np.float32)
            quat = np.zeros(4, dtype=np.float32)
            with state_lock:
                rs = robot_state
                if rs is None:
                    continue
                for i in range(24):
                    q_current[i] = rs.motor_state[i].q
                    dq_current[i] = rs.motor_state[i].dq
                gyro[:] = np.array(rs.imu_state.gyroscope, dtype=np.float32)
                quat[:] = np.array(rs.imu_state.quaternion, dtype=np.float32)
            
            projected_gravity = compute_projected_gravity(quat)
    
            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)
            gait_phase *= gait_scale
    
            q_rel = q_current - DEFAULT_Q
    
            # Đầu vào 83 chiều của policy_r1_2.onnx (Không có height_scan)
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
                action = session.run(None, {input_name: obs_tensor})[0][0]
                last_action = action.copy()
                target_q_arr = DEFAULT_Q + action * ACTION_SCALE
                
                with cmd_lock:
                    for i in range(24):
                        cmd.motor_cmd[i].q = float(target_q_arr[i])
            except Exception as e:
                print(f"\n[LỖI NGHIÊM TRỌNG] Lỗi khi tính toán Action: {e}")
                print(f"Kích thước obs_tensor: {obs_tensor.shape}")
                print(f"Kích thước action trả về: {getattr(action, 'shape', 'N/A') if 'action' in locals() else 'N/A'}")
                import sys
                sys.exit(1)
    
            time_until_next = 0.02 - (time.perf_counter() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

    except KeyboardInterrupt:
        pass
    finally:
        print("Đã tắt script điều khiển.")

if __name__ == '__main__':
    main()
