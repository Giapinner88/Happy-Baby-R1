import time
import threading
import numpy as np
import onnxruntime as ort
import pygame
import math

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from state_logger import SimStateLogger
from fall_detector.detector import G1FallDetector
from fall_detector.logger import IMULogger
from wave_animation.controller import WaveController
from arm_csv_player import ArmCSVPlayer

robot_state = None
_got_first_state = False
state_lock = threading.Lock()

cmd = unitree_hg_msg_dds__LowCmd_()
cmd_lock = threading.Lock()

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

    with cmd_lock:
        for i in range(29):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = DEFAULT_Q[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd = float(KD_ARRAY[i])

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    session = ort.InferenceSession("policy98.onnx", providers=['CPUExecutionProvider'])
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

    # Tích lũy thời gian chu kỳ bước (gait_time) thay vì dùng t_current liên tục
    gait_time = 0.0
    last_step_time = time.perf_counter()
    last_print_time = 0.0
    
    # Biến để triệt tiêu dần tín hiệu nhịp bước (gait_phase) khi đứng im
    gait_scale = 1.0

    logger = SimStateLogger(__file__)
    step = 0
    t0 = time.perf_counter()

    fall_detector = G1FallDetector()
    imu_logger = IMULogger()
    arm_player = ArmCSVPlayer()

    try:
        while True:
            with state_lock:
                have_state = robot_state is not None
            if not have_state:
                time.sleep(0.002)
                last_step_time = time.perf_counter()
                continue
                
            step_start = time.perf_counter()
            dt = step_start - last_step_time
            last_step_time = step_start
            # Clamp dt to avoid large phase jumps when the loop stalls (OS scheduling/print/etc.)
            if dt < 0.0:
                dt = 0.0
            elif dt > 0.05:
                dt = 0.05
            # Policy được train với dt cố định 20ms — dùng giá trị cố định cho gait_time
            # để tránh phụ thuộc vào jitter CPU. dt thực tế chỉ dùng để enforce timing.
            CTRL_DT = 0.02  # 50 Hz — khớp với training
    
            # --- XỬ LÝ SỰ KIỆN THOÁT ---
            exit_pressed = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_pressed = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        exit_pressed = True

            if joystick is not None:
                try:
                    # Nút X trên tay Xbox thường là button 2
                    if joystick.get_button(2):
                        exit_pressed = True
                except Exception:
                    pass

            if exit_pressed:
                print("\n>>> NHẬN LỆNH THOÁT (Phím ESC hoặc nút X trên Gamepad). Đang đóng và lưu log...")
                break
            
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
            
            # Khôi phục trạng thái ngã nếu người dùng yêu cầu
            keys = pygame.key.get_pressed()
            reset_pressed = keys[pygame.K_r]
            if joystick is not None:
                try:
                    reset_pressed = reset_pressed or joystick.get_button(1)
                except Exception:
                    pass

            # --- Khớp tay CSV ---
            wave_pressed = keys[pygame.K_v]
            heart_pressed = keys[pygame.K_h]
            shake_pressed = keys[pygame.K_b]
            if joystick is not None:
                try:
                    wave_pressed = wave_pressed or joystick.get_button(3)   # Nút Y -> vẫy tay
                    heart_pressed = heart_pressed or joystick.get_button(0) # Nút A -> trái tim
                    shake_pressed = shake_pressed or joystick.get_button(4) # Nút LB -> bắt tay
                except Exception:
                    pass
                    
            if wave_pressed:
                arm_player.trigger("vaytay")
                time.sleep(0.3) # Chống dội phím
            elif heart_pressed:
                arm_player.trigger("traitim")
                time.sleep(0.3) # Chống dội phím
            elif shake_pressed:
                arm_player.trigger("battay")
                time.sleep(0.3) # Chống dội phím

            if reset_pressed:
                print(">>> RESET TOÀN BỘ TRẠNG THÁI & KHỞI ĐỘNG LẠI SIMULATOR...")
                
                # 1. Gửi lệnh reset đặc biệt sang simulator (DDS)
                with cmd_lock:
                    cmd.motor_cmd[0].mode = 0xFF
                
                # Chờ publisher thread gửi đi (chu kỳ publisher là 2ms)
                time.sleep(0.05)
                
                # 2. Khôi phục lại trạng thái của client
                fall_detector.reset()
                imu_logger.reset()
                last_action = np.zeros(29, dtype=np.float32)
                smoothed_commands = np.zeros(3, dtype=np.float32)
                gait_time = 0.0
                gait_scale = 1.0
                step = 0
                t0 = time.perf_counter()
                last_print_time = 0.0
                
                # Khôi phục KP, KD, Q và MODE mặc định gửi xuống motor
                with cmd_lock:
                    for i in range(29):
                        cmd.motor_cmd[i].mode = 0x01
                        cmd.motor_cmd[i].q = DEFAULT_Q[i]
                        cmd.motor_cmd[i].dq = 0.0
                        cmd.motor_cmd[i].tau = 0.0
                        cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
                        cmd.motor_cmd[i].kd = float(KD_ARRAY[i])
                        
                time.sleep(0.5) # Chống dội phím
                last_step_time = time.perf_counter()
                continue
    
            # --- LOGIC ĐIỀU KHIỂN CHU KỲ BƯỚC ---
            # Chỉ tiến hành tăng pha bước khi có tín hiệu vận tốc
            if abs(smoothed_commands[0]) > 0.01 or abs(smoothed_commands[1]) > 0.01 or abs(smoothed_commands[2]) > 0.01:
                now = time.perf_counter()
                if now - last_print_time > 0.5:
                    print(f"Đang gửi lệnh: Vx={smoothed_commands[0]:.2f}, Vy={smoothed_commands[1]:.2f}, Yaw={smoothed_commands[2]:.2f}")
                    last_print_time = now
                gait_time += CTRL_DT
                gait_scale = min(1.0, gait_scale + CTRL_DT / 0.3)
            else:
                # Nếu không có lệnh di chuyển, cho phép chu kỳ bước tiếp tục đến khi hoàn thành (chu kỳ 0.6s)
                # để robot trở về tư thế đứng cân bằng trên 2 chân (phase_ratio = 0)
                remainder = gait_time % 0.6
                if 0.02 < remainder < 0.58:
                    gait_time += CTRL_DT
                    gait_scale = min(1.0, gait_scale + CTRL_DT / 0.3)
                else:
                    gait_time = round(gait_time / 0.6) * 0.6
                    gait_scale = max(0.0, gait_scale - CTRL_DT / 0.3) # Giảm dần tín hiệu pha về 0
    
            # Snapshot state under lock so a single control step uses a consistent LowState
            q_current = np.zeros(29, dtype=np.float32)
            dq_current = np.zeros(29, dtype=np.float32)
            gyro = np.zeros(3, dtype=np.float32)
            quat = np.zeros(4, dtype=np.float32)
            accel = np.zeros(3, dtype=np.float32)
            with state_lock:
                rs = robot_state
                if rs is None:
                    continue
                for i in range(29):
                    q_current[i] = rs.motor_state[i].q
                    dq_current[i] = rs.motor_state[i].dq
                gyro[:] = np.array(rs.imu_state.gyroscope, dtype=np.float32)
                quat[:] = np.array(rs.imu_state.quaternion, dtype=np.float32)
                accel[:] = np.array(rs.imu_state.accelerometer, dtype=np.float32)
            
            projected_gravity = compute_projected_gravity(quat)
            
            # Ghi log vòng lặp tròn (O(1) memory, zero overhead)
            imu_logger.log_step(time.perf_counter(), projected_gravity, gyro, accel)
            
            # --- KIỂM TRA NGÃ (FALL DETECTION) ---
            is_fallen, is_lay_down, reasons = fall_detector.check(projected_gravity, gyro, accel)
            
            if is_fallen and len(reasons) > 0:
                print(f"\n!!! PHÁT HIỆN: {' | '.join(reasons)} !!!")
                imu_logger.trigger_fall_event(time.perf_counter())
                
                with cmd_lock:
                    for i in range(29):
                        cmd.motor_cmd[i].kp = 0.0 # Bỏ độ cứng
                        if is_lay_down:
                            cmd.motor_cmd[i].kd = 0.0 # Ngắt toàn bộ momen
                        else:
                            # Dùng hệ số cản gốc của từng khớp thay vì 5.0 (gây rung bần bật cánh tay)
                            cmd.motor_cmd[i].kd = float(KD_ARRAY[i]) 
                        cmd.motor_cmd[i].tau = 0.0
                smoothed_commands[:] = 0.0 # Xóa lệnh chạy
                
            if fall_detector.is_fallen:
                step += 1
                time_until_next = 0.02 - (time.perf_counter() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)
                continue
    
            # Tính toán phase_ratio từ biến thời gian tích lũy
            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)
            
            # Triệt tiêu dần tín hiệu nhịp bước (gait_phase) về [0, 0] khi muốn đứng yên
            gait_phase *= gait_scale
    
            q_rel = q_current - DEFAULT_Q
    
            # --- MASKING OBSERVATION CHO CÁNH TAY ĐANG VẪY ---
            # Che giấu trạng thái thực của tay phải, giả vờ như nó đang ở vị trí mặc định
            # để mạng Neural không hoảng loạn (do khác biệt quá lớn với dữ liệu huấn luyện) và phá vỡ dáng đi.
            if getattr(arm_player, 'blend_weight', 0.0) > 0.0:
                q_rel_obs = q_rel.copy()
                dq_obs = dq_current.copy()
                last_action_obs = last_action.copy()
                
                # Che giấu trạng thái thực của 2 tay (khớp 15->28)
                q_rel_obs[15:29] = 0.0
                dq_obs[15:29] = 0.0
                last_action_obs[15:29] = 0.0
            else:
                q_rel_obs = q_rel
                dq_obs = dq_current
                last_action_obs = last_action

            # --- BÙ ĐẮP THĂNG BẰNG (BALANCE COMPENSATION) KHI CHƠI CỬ CHỈ ---
            # Kết hợp bù trừ vận tốc tiến (vx_bias) và bù trừ góc nghiêng trọng lực (gx_bias)
            # để "đánh lừa" robot ngửa thân về sau nhằm triệt tiêu mô-men kéo do vươn tay trước.
            smoothed_commands_obs = smoothed_commands.copy()
            projected_gravity_obs = projected_gravity.copy()
            if getattr(arm_player, 'blend_weight', 0.0) > 0.0:
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
                smoothed_commands_obs[0] -= vx_bias * arm_player.blend_weight
                projected_gravity_obs[0] += gx_bias * arm_player.blend_weight

            obs = np.concatenate([
                gyro,                 
                projected_gravity_obs,  # Sử dụng trọng lực đã bù trừ
                smoothed_commands_obs,  # Sử dụng vận tốc đã bù trừ
                gait_phase,           
                q_rel_obs,                
                dq_obs,           
                last_action_obs           
            ]).astype(np.float32)
    
            obs_tensor = np.expand_dims(obs, axis=0)
            action = session.run(None, {input_name: obs_tensor})[0][0]
    
            last_action = action.copy()
    
            # Tính target_q array (dùng để log replay VÀ gửi xuống motor)
            target_q_arr = DEFAULT_Q + action * ACTION_SCALE
            
            # --- GHI ĐÈ CHUYỂN ĐỘNG KHỚP TAY TỪ CSV ---
            target_q_arr = arm_player.update_and_blend(0.02, target_q_arr)
            with cmd_lock:
                for i in range(29):
                    cmd.motor_cmd[i].q = float(target_q_arr[i])
    
            # Ghi log (non-blocking: chỉ put vào queue ~100ns)
            logger.log(
                step   = step,
                t      = step_start - t0,
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

if __name__ == '__main__':
    main()
