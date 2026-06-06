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
from state_logger import SimStateLogger

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


def get_int_env(name: str, default: int) -> int:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return int(value)
    except ValueError:
        print(f"CẢNH BÁO: {name}={value!r} không hợp lệ. Dùng {default}.")
        return default


def sleep_until(target_time: float, spin_threshold_s: float = 0.0005):
    """Sleep until `target_time` (perf_counter timebase) with reduced jitter.

    - Sleeps coarsely using `time.sleep`.
    - Optionally busy-spins for the final `spin_threshold_s` seconds.

    This pattern tends to reduce drift/oversleep jitter on Linux.
    """
    while True:
        now = time.perf_counter()
        remaining = target_time - now
        if remaining <= 0.0:
            return
        if remaining > spin_threshold_s:
            # Leave some margin for scheduler oversleep.
            time.sleep(max(0.0, remaining - spin_threshold_s))
        else:
            # Busy wait for very short remainder.
            while time.perf_counter() < target_time:
                pass
            return


def get_float_env(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"CẢNH BÁO: {name}={value!r} không hợp lệ. Dùng {default}.")
        return default


def smoothstep01(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def state_handler(msg: LowState_):
    global robot_state, _got_first_state
    with state_lock:
        robot_state = msg
    if not _got_first_state:
        _got_first_state = True
        print("Đã nhận LowState từ simulator (DDS OK).")

def dds_publisher_loop(pub):
    crc_calc = CRC()
    publish_hz = max(1, get_int_env("DDS_PUBLISH_HZ", 500))
    publish_dt = 1.0 / float(publish_hz)
    spin_threshold = max(0.0, get_float_env("SLEEP_SPIN_THRESHOLD", 0.0005))
    next_pub_time = time.perf_counter()
    while True:
        # Deadline-based scheduling helps reduce jitter/drift.
        next_pub_time += publish_dt
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        sleep_until(next_pub_time, spin_threshold_s=spin_threshold)


def set_motor_position_targets(target_q):
    with cmd_lock:
        for i in range(29):
            cmd.motor_cmd[i].q = float(target_q[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(KP_ARRAY[i])
            cmd.motor_cmd[i].kd = float(KD_ARRAY[i])


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


def resolve_policy_path() -> Path:
    """Resolve the policy file used by the controller.

    Resolution order:
    1. `POLICY_ONNX` environment variable, if set.
    2. `policy.onnx` in the current directory.
    3. The newest `policy*.onnx` file in the current directory.

    This lets the user replace the policy by renaming the file only.
    """
    env_value = os.environ.get("POLICY_ONNX", "").strip()
    if env_value:
        env_path = Path(env_value).expanduser()
        if not env_path.is_absolute():
            env_path = Path(__file__).parent / env_path
        if env_path.exists():
            return env_path

    default_path = Path(__file__).parent / "policy.onnx"
    if default_path.exists():
        return default_path

    policy_candidates = sorted(
        Path(__file__).parent.glob("policy*.onnx"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if policy_candidates:
        return policy_candidates[0]

    raise FileNotFoundError(
        "Không tìm thấy policy ONNX. Hãy đặt file policy.onnx hoặc "
        "truyền POLICY_ONNX trỏ tới file .onnx hợp lệ."
    )

def main():
    global robot_state, cmd
    
    domain_id = int(os.environ.get("DOMAIN_ID", "1"))
    interface = os.environ.get("INTERFACE", "lo")
    ChannelFactoryInitialize(domain_id, interface) 
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

    policy_path = resolve_policy_path()
    print(f"Dùng policy: {policy_path.name}")
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

    # Tích lũy thời gian chu kỳ bước (gait_time) thay vì dùng t_current liên tục
    gait_time = 0.0
    last_print_time = 0.0
    
    # Biến để triệt tiêu dần tín hiệu nhịp bước (gait_phase) khi đứng im
    gait_scale = 0.0
    warmup_seconds = max(0.0, get_float_env("POLICY_WARMUP_SECONDS", 2.0))
    warmup_done = warmup_seconds == 0.0
    warmup_start = None
    warmup_q0 = None
    policy_start = None
    policy_fade_seconds = max(0.0, get_float_env("POLICY_FADE_SECONDS", 2.0))
    action_clip = max(0.0, get_float_env("POLICY_ACTION_CLIP", 0.6))
    target_rate_limit = max(0.0, get_float_env("POLICY_TARGET_RATE_LIMIT", 4.0))
    fall_guard_gravity_z = get_float_env("POLICY_FALL_GUARD_GRAVITY_Z", -0.55)
    previous_target_q = DEFAULT_Q.copy()
    last_guard_print = 0.0

    logger = SimStateLogger(__file__)
    step = 0
    t0 = time.perf_counter()
    CTRL_DT = 0.02  # 50 Hz — khớp với training
    spin_threshold = max(0.0, get_float_env("SLEEP_SPIN_THRESHOLD", 0.0005))
    next_step_time = None

    try:
        while True:
            with state_lock:
                have_state = robot_state is not None
            if not have_state:
                time.sleep(0.002)
                next_step_time = None
                continue

            if next_step_time is None:
                next_step_time = time.perf_counter()

            # Sleep until the next control deadline for stable 50Hz pacing.
            sleep_until(next_step_time, spin_threshold_s=spin_threshold)
            step_start = time.perf_counter()

            # If we're far behind (e.g. OS hiccup), resync to avoid accumulating lag.
            if step_start - next_step_time > 2.0 * CTRL_DT:
                next_step_time = step_start

            dt = CTRL_DT
    
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
            with state_lock:
                rs = robot_state
                if rs is None:
                    continue
                for i in range(29):
                    q_current[i] = rs.motor_state[i].q
                    dq_current[i] = rs.motor_state[i].dq
                gyro[:] = np.array(rs.imu_state.gyroscope, dtype=np.float32)
                quat[:] = np.array(rs.imu_state.quaternion, dtype=np.float32)

            if not warmup_done:
                if warmup_start is None:
                    warmup_start = step_start
                    warmup_q0 = q_current.copy()
                    print(f"FixStand warmup {warmup_seconds:.1f}s trước khi bật ONNX policy.")

                alpha_warmup = smoothstep01((step_start - warmup_start) / warmup_seconds)
                target_q_arr = (1.0 - alpha_warmup) * warmup_q0 + alpha_warmup * DEFAULT_Q
                set_motor_position_targets(target_q_arr)

                if alpha_warmup >= 1.0:
                    warmup_done = True
                    last_action.fill(0.0)
                    smoothed_commands.fill(0.0)
                    gait_time = 0.0
                    gait_scale = 0.0
                    previous_target_q = DEFAULT_Q.copy()
                    policy_start = time.perf_counter()
                    next_step_time = time.perf_counter()
                    print("FixStand warmup xong. Bật ONNX policy.")
                else:
                    next_step_time += CTRL_DT
                    continue
            
            projected_gravity = compute_projected_gravity(quat)

            if projected_gravity[2] > fall_guard_gravity_z:
                now = time.perf_counter()
                if now - last_guard_print > 0.5:
                    print(
                        "Fall guard: thân robot nghiêng quá mức, "
                        "đưa target về DEFAULT_Q và reset policy state."
                    )
                    last_guard_print = now
                last_action.fill(0.0)
                smoothed_commands.fill(0.0)
                gait_time = 0.0
                gait_scale = 0.0
                previous_target_q = DEFAULT_Q.copy()
                set_motor_position_targets(DEFAULT_Q)
                next_step_time += CTRL_DT
                continue
    
            # Tính toán phase_ratio từ biến thời gian tích lũy
            phase_ratio = (gait_time % 0.6) / 0.6
            gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)], dtype=np.float32)
            
            # Triệt tiêu dần tín hiệu nhịp bước (gait_phase) về [0, 0] khi muốn đứng yên
            gait_phase *= gait_scale
    
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
            raw_action = session.run(None, {input_name: obs_tensor})[0][0]
            action = raw_action.astype(np.float32)
            if action_clip > 0.0:
                action = np.clip(action, -action_clip, action_clip)
            if policy_fade_seconds > 0.0:
                if policy_start is None:
                    policy_start = step_start
                action *= smoothstep01((step_start - policy_start) / policy_fade_seconds)
    
            last_action = action.copy()
    
            # Tính target_q array (dùng để log replay VÀ gửi xuống motor)
            target_q_arr = DEFAULT_Q + action * ACTION_SCALE
            if target_rate_limit > 0.0:
                max_delta = target_rate_limit * CTRL_DT
                target_q_arr = previous_target_q + np.clip(
                    target_q_arr - previous_target_q,
                    -max_delta,
                    max_delta,
                )
            previous_target_q = target_q_arr.copy()
            set_motor_position_targets(target_q_arr)
    
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

            next_step_time += CTRL_DT

    except KeyboardInterrupt:
        pass
    finally:
        logger.close()

if __name__ == '__main__':
    main()
