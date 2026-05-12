import time
import threading
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pygame

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from state_logging import SimStateLogger

robot_state = None
cmd = unitree_hg_msg_dds__LowCmd_()

# === CẤU HÌNH THEO CHUẨN URDF GỐC CỦA POLICY98 ===
# Bắt buộc phải dùng Mảng chứ không được dùng Scalar để tránh làm gãy cổ chân
KP_ARRAY = np.array([40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 28.5, 28.5,
                     14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8], dtype=np.float32)

KD_ARRAY = np.array([2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 1.8, 1.8,
                     0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1], dtype=np.float32)

DEFAULT_Q = np.array([-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0, 0, 0, 0, 
                      0.35, 0.18, 0, 0.87, 0, 0, 0, 0.35, -0.18, 0, 0.87, 0, 0, 0], dtype=np.float32)

ACTION_SCALE = np.array([0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44,
                         0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07], dtype=np.float32)

OBS_SIZE = 96
HISTORY_LEN = 5

obs_history = np.zeros((HISTORY_LEN, OBS_SIZE), dtype=np.float32)

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

    for i in range(29):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = float(DEFAULT_Q[i])
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
        cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    policy_path = Path(__file__).with_name("policy480.onnx")
    session = ort.InferenceSession(str(policy_path), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    
    pygame.init()
    pygame.display.set_mode((300, 200))
    pygame.display.set_caption('GAMEPAD CONTROL')

    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Hệ thống điều khiển: Gamepad ({joystick.get_name()})")
    else:
        print("CẢNH BÁO: Không tìm thấy Gamepad. Chuyển về chế độ bàn phím.")

    last_action = np.zeros(29, dtype=np.float32)
    smoothed_commands = np.zeros(3, dtype=np.float32)

    print("--- ĐANG KHỞI TẠO BỘ NHỚ LỊCH SỬ ---")
    while robot_state is None:
        time.sleep(0.01)
        
    time.sleep(0.5) 
    
    # Fill history buffer with static pose
    q_raw = np.array([robot_state.motor_state[i].q for i in range(29)])
    dq_raw = np.array([robot_state.motor_state[i].dq for i in range(29)])
    
    initial_frame = np.concatenate([
        np.array(robot_state.imu_state.gyroscope) * 0.2,
        compute_projected_gravity(robot_state.imu_state.quaternion),
        np.zeros(3, dtype=np.float32),
        (q_raw - DEFAULT_Q),
        dq_raw * 0.05,
        last_action
    ]).astype(np.float32)
    
    for i in range(HISTORY_LEN):
        obs_history[i] = initial_frame

    print("--- BẮT ĐẦU ĐIỀU KHIỂN ---")
    state_logger = SimStateLogger(__file__)

    while True:
        if robot_state is None:
            time.sleep(0.002)
            continue
            
        step_start = time.perf_counter()
        pygame.event.pump()

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
        smoothed_commands = 0.1 * target_commands + 0.9 * smoothed_commands

        q_raw = np.array([robot_state.motor_state[i].q for i in range(29)])
        dq_raw = np.array([robot_state.motor_state[i].dq for i in range(29)])
        gyro = np.array(robot_state.imu_state.gyroscope, dtype=np.float32)
        quat = robot_state.imu_state.quaternion
        state_logger.log_low_state(
            robot_state,
            q=q_raw,
            dq=dq_raw,
            imu_quat=quat,
            imu_gyro=gyro,
        )
        
        current_frame = np.concatenate([
            gyro * 0.2,
            compute_projected_gravity(quat),
            smoothed_commands, # ĐỂ MỘC, KHÔNG NHÂN SCALE (Giống y hệt policy98)
            (q_raw - DEFAULT_Q),
            dq_raw * 0.05,
            last_action
        ]).astype(np.float32)

        for i in range(HISTORY_LEN - 1):
            obs_history[i] = obs_history[i+1]
        obs_history[-1] = current_frame

        obs_tensor = np.expand_dims(obs_history.flatten(), axis=0)
        action = session.run(None, {input_name: obs_tensor})[0][0]
        
        last_action = action.copy()

        for i in range(29):
            target_q = DEFAULT_Q[i] + (action[i] * ACTION_SCALE[i])
            cmd.motor_cmd[i].q = float(target_q)
            
            cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

        time_until_next = 0.02 - (time.perf_counter() - step_start)
        if time_until_next > 0:
            time.sleep(time_until_next)

if __name__ == '__main__':
    main()
