import os
import time
import threading
from pathlib import Path
import numpy as np
import onnxruntime as ort
import pygame

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from state_logging import SimStateLogger

robot_state = None
cmd = unitree_hg_msg_dds__LowCmd_()
SCRIPT_DIR = Path(__file__).resolve().parent

# --- KHAI BÁO HẰNG SỐ ĐỘNG LỰC HỌC TỪ TASK.YAML ---
KP_ARRAY = np.array([40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 28.5, 28.5,
                     14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8], dtype=np.float32)

KD_ARRAY = np.array([2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 1.8, 1.8,
                     0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1], dtype=np.float32)

DEFAULT_Q = np.array([-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0, 0, 0, 0, 
                      0.35, 0.18, 0, 0.87, 0, 0, 0, 0.35, -0.18, 0, 0.87, 0, 0, 0], dtype=np.float32)

ACTION_SCALE = np.array([0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44,
                         0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07], dtype=np.float32)


def state_handler(msg: LowState_):
    global robot_state
    robot_state = msg

def dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
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

def resolve_policy_path(default_name="policy.onnx"):
    env_value = os.environ.get("POLICY_ONNX", "").strip()
    if env_value:
        path = Path(env_value).expanduser()
        if not path.is_absolute():
            path = SCRIPT_DIR / path
        if path.exists():
            return path
        raise FileNotFoundError(f"Không tìm thấy POLICY_ONNX: {path}")

    path = SCRIPT_DIR / default_name
    if path.exists():
        return path
    raise FileNotFoundError(f"Không tìm thấy policy ONNX: {path}")

def main():
    global robot_state, cmd
    
    domain_id = int(os.environ.get("DOMAIN_ID", "1"))
    interface = os.environ.get("INTERFACE", "lo")
    ChannelFactoryInitialize(domain_id, interface) 
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)

    for i in range(29):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = DEFAULT_Q[i]
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
        cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    policy_path = resolve_policy_path()
    print(f"Dùng policy: {policy_path}")
    session = ort.InferenceSession(str(policy_path), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    
    pygame.init()
    pygame.display.set_mode((300, 200))
    pygame.display.set_caption('GAMEPAD CONTROL')

    # KHỞI TẠO GAMEPAD
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Hệ thống điều khiển: Gamepad ({joystick.get_name()})")
    else:
        print("CẢNH BÁO: Không tìm thấy Gamepad. Chuyển về chế độ bàn phím.")

    last_action = np.zeros(29, dtype=np.float32)
    
    # Biến lưu trữ lệnh vận tốc để lọc EMA (Tránh giật cục)
    smoothed_commands = np.zeros(3, dtype=np.float32)
    alpha = 0.1 # Hệ số làm mượt

    t_start = time.perf_counter()
    state_logger = SimStateLogger(__file__)

    while True:
        if robot_state is None:
            time.sleep(0.002)
            continue
            
        step_start = time.perf_counter()
        t_current = step_start - t_start

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                print("Đóng cửa sổ điều khiển. Dừng policy.")
                pygame.quit()
                return
        
        # --- ĐỌC TÍN HIỆU ĐIỀU KHIỂN ---
        raw_vx = 0.0
        raw_vy = 0.0
        raw_yaw = 0.0

        if joystick is not None:
            def apply_deadzone(value, threshold=0.15):
                return 0.0 if abs(value) < threshold else value

            # Đọc trục (Axis mapping có thể khác nhau, test thực tế để chỉnh số 0, 1, 3, 4)
            axis_left_x = apply_deadzone(joystick.get_axis(0)) 
            axis_left_y = apply_deadzone(joystick.get_axis(1))  
            axis_right_x = apply_deadzone(joystick.get_axis(3)) 

            # Map sang hệ tọa độ robot (Tối đa 1.0 m/s tới, 0.5 m/s ngang, 1.0 rad/s xoay)
            raw_vx = -axis_left_y * 1.0  
            raw_vy = -axis_left_x * 0.5  
            raw_yaw = -axis_right_x * 1.0 
        else:
            keys = pygame.key.get_pressed()
            raw_vx = 1.0 if keys[pygame.K_w] else (-0.5 if keys[pygame.K_s] else 0.0)
            raw_vy = 0.5 if keys[pygame.K_a] else (-0.5 if keys[pygame.K_d] else 0.0)
            raw_yaw = 1.0 if keys[pygame.K_q] else (-1.0 if keys[pygame.K_e] else 0.0)

        # ÁP DỤNG BỘ LỌC TÍN HIỆU TOÁN HỌC (EMA)
        target_commands = np.array([raw_vx, raw_vy, raw_yaw], dtype=np.float32)
        smoothed_commands = alpha * target_commands + (1.0 - alpha) * smoothed_commands
# In ra màn hình terminal số 2 để xác nhận có tín hiệu
        if abs(smoothed_commands[0]) > 0.01 or abs(smoothed_commands[1]) > 0.01 or abs(smoothed_commands[2]) > 0.01:
            print(f"Đang gửi lệnh: Vx={smoothed_commands[0]:.2f}, Vy={smoothed_commands[1]:.2f}, Yaw={smoothed_commands[2]:.2f}")
        q_current = np.zeros(29, dtype=np.float32)
        dq_current = np.zeros(29, dtype=np.float32)
        for i in range(29):
            q_current[i] = robot_state.motor_state[i].q
            dq_current[i] = robot_state.motor_state[i].dq
            
        gyro = np.array(robot_state.imu_state.gyroscope, dtype=np.float32)
        quat = robot_state.imu_state.quaternion
        state_logger.log_low_state(
            robot_state,
            q=q_current,
            dq=dq_current,
            imu_quat=quat,
            imu_gyro=gyro,
            timestamp_s=t_current,
        )
        
        projected_gravity = compute_projected_gravity(quat)

        phase_ratio = (t_current % 0.6) / 0.6
        gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)

        q_rel = q_current - DEFAULT_Q

        obs = np.concatenate([
            gyro,                 
            projected_gravity,    
            smoothed_commands,    # Sử dụng lệnh đã được lọc mượt
            gait_phase,           
            q_rel,                
            dq_current,           
            last_action           
        ]).astype(np.float32)

        obs_tensor = np.expand_dims(obs, axis=0)
        action = session.run(None, {input_name: obs_tensor})[0][0]

        last_action = action.copy()

        for i in range(29):
            target_q = DEFAULT_Q[i] + (action[i] * ACTION_SCALE[i])
            cmd.motor_cmd[i].q = float(target_q)

        time_until_next = 0.02 - (time.perf_counter() - step_start)
        if time_until_next > 0:
            time.sleep(time_until_next)

if __name__ == '__main__':
    main()
