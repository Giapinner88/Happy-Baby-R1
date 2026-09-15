"""Single R1 policy runner for Unitree MuJoCo.

Policy selection, robot asset, joint order, gains, and ONNX path are configured
in config.py. This runner intentionally supports only the local R1 policy path.
"""

from __future__ import annotations

import os
import signal
import threading
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pygame

import config
from state_logger import SimStateLogger
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.utils.crc import CRC


robot_state: LowState_ | None = None
sport_state: SportModeState_ | None = None
state_lock = threading.Lock()
sport_lock = threading.Lock()
cmd_lock = threading.Lock()
cmd = unitree_hg_msg_dds__LowCmd_()
stop_event = threading.Event()


def get_int_env(name: str, default: int) -> int:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return int(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.")
        return default


def get_float_env(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.")
        return default


def sleep_until(target_time: float, spin_threshold_s: float = 0.0005) -> None:
    while True:
        now = time.perf_counter()
        remaining = target_time - now
        if remaining <= 0.0:
            return
        if remaining > spin_threshold_s:
            time.sleep(max(0.0, remaining - spin_threshold_s))
        else:
            while time.perf_counter() < target_time:
                pass
            return


def smoothstep01(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def command_limit_array() -> np.ndarray:
    return np.array(
        [
            max(0.0, get_float_env("POLICY_CMD_LIMIT_VX", 0.0)),
            max(0.0, get_float_env("POLICY_CMD_LIMIT_VY", 0.0)),
            max(0.0, get_float_env("POLICY_CMD_LIMIT_YAW", 0.0)),
        ],
        dtype=np.float32,
    )


def command_slew_array() -> np.ndarray:
    return np.array(
        [
            max(0.0, get_float_env("POLICY_CMD_SLEW_VX", 0.0)),
            max(0.0, get_float_env("POLICY_CMD_SLEW_VY", 0.0)),
            max(0.0, get_float_env("POLICY_CMD_SLEW_YAW", 0.0)),
        ],
        dtype=np.float32,
    )


def clamp_command(command: np.ndarray, limits: np.ndarray) -> np.ndarray:
    clamped = command.astype(np.float32, copy=True)
    active = limits > 0.0
    clamped[active] = np.clip(clamped[active], -limits[active], limits[active])
    return clamped


def filter_command(
    raw_command: np.ndarray,
    previous_command: np.ndarray,
    *,
    alpha: float,
    dt: float,
    limits: np.ndarray,
    slew_limits: np.ndarray,
) -> np.ndarray:
    desired = clamp_command(raw_command, limits)
    filtered = alpha * desired + (1.0 - alpha) * previous_command
    active_slew = slew_limits > 0.0
    if np.any(active_slew):
        max_delta = slew_limits * dt
        delta = filtered - previous_command
        delta[active_slew] = np.clip(
            delta[active_slew],
            -max_delta[active_slew],
            max_delta[active_slew],
        )
        filtered = previous_command + delta
    return clamp_command(filtered, limits)


def state_handler(msg: LowState_) -> None:
    global robot_state
    first_state = False
    with state_lock:
        if robot_state is None:
            first_state = True
        robot_state = msg
    if first_state:
        print("[policy] received first LowState from simulator.")


def sport_state_handler(msg: SportModeState_) -> None:
    global sport_state
    first_state = False
    with sport_lock:
        if sport_state is None:
            first_state = True
        sport_state = msg
    if first_state:
        print("[policy] received first SportModeState from simulator.")


def request_stop(signum: int, _frame) -> None:
    print(f"[policy] received signal {signum}; stopping.")
    stop_event.set()


def dds_publisher_loop(pub: ChannelPublisher) -> None:
    crc_calc = CRC()
    publish_hz = max(1, get_int_env("DDS_PUBLISH_HZ", 500))
    publish_dt = 1.0 / float(publish_hz)
    spin_threshold = max(0.0, get_float_env("SLEEP_SPIN_THRESHOLD", 0.0005))
    next_pub_time = time.perf_counter()

    while not stop_event.is_set():
        next_pub_time += publish_dt
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
        sleep_until(next_pub_time, spin_threshold_s=spin_threshold)


def set_motor_position_targets(target_q: np.ndarray) -> None:
    with cmd_lock:
        for i in range(len(config.MOTOR_JOINT_NAMES)):
            cmd.motor_cmd[i].q = float(target_q[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(config.KP_ARRAY[i])
            cmd.motor_cmd[i].kd = float(config.KD_ARRAY[i])


def compute_projected_gravity(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            2 * (w * y - x * z),
            -2 * (y * z + w * x),
            2 * (x**2 + y**2) - 1,
        ],
        dtype=np.float32,
    )


def read_state_snapshot() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    q_current = np.zeros(len(config.MOTOR_JOINT_NAMES), dtype=np.float32)
    dq_current = np.zeros(len(config.MOTOR_JOINT_NAMES), dtype=np.float32)
    gyro = np.zeros(3, dtype=np.float32)
    quat = np.zeros(4, dtype=np.float32)
    base_pos = np.full(3, np.nan, dtype=np.float32)
    base_vel = np.full(3, np.nan, dtype=np.float32)

    with state_lock:
        rs = robot_state
        if rs is None:
            return None
        for i in range(len(config.MOTOR_JOINT_NAMES)):
            q_current[i] = rs.motor_state[i].q
            dq_current[i] = rs.motor_state[i].dq
        gyro[:] = np.array(rs.imu_state.gyroscope, dtype=np.float32)
        quat[:] = np.array(rs.imu_state.quaternion, dtype=np.float32)

    with sport_lock:
        ss = sport_state
        if ss is not None:
            base_pos[:] = np.array(ss.position, dtype=np.float32)
            base_vel[:] = np.array(ss.velocity, dtype=np.float32)

    return q_current, dq_current, gyro, quat, base_pos, base_vel


def resolve_policy_session() -> tuple[ort.InferenceSession, str]:
    policy_path = Path(config.POLICY_ONNX)
    if not policy_path.exists():
        raise FileNotFoundError(
            f"Missing R1 ONNX policy: {policy_path}\n"
            "Export/copy the selected R1 policy to data/models/unitree_mujoco_policy/ "
            "or set POLICY_ONNX to an R1 policy file."
        )

    providers = ["CPUExecutionProvider"]
    provider_env = os.environ.get("ONNXRUNTIME_PROVIDERS", "").strip()
    if provider_env:
        providers = [item.strip() for item in provider_env.split(",") if item.strip()]

    session = ort.InferenceSession(str(policy_path), providers=providers)
    input_info = session.get_inputs()[0]
    output_info = session.get_outputs()[0]

    input_shape = list(input_info.shape)
    output_shape = list(output_info.shape)
    if input_shape[-1] not in (None, "obs", config.OBS_DIM):
        raise ValueError(
            f"Policy input dim mismatch: expected {config.OBS_DIM}, got {input_shape}."
        )
    if output_shape[-1] not in (None, "actions", config.ACTION_DIM):
        raise ValueError(
            f"Policy output dim mismatch: expected {config.ACTION_DIM}, got {output_shape}."
        )

    print(f"[policy] name={config.POLICY_NAME}")
    print(f"[policy] onnx={policy_path}")
    print(f"[policy] obs_dim={config.OBS_DIM} action_dim={config.ACTION_DIM}")
    print(f"[policy] providers={session.get_providers()}")
    return session, input_info.name


def init_pygame() -> pygame.joystick.Joystick | None:
    pygame.init()
    pygame.display.set_mode((300, 200))
    pygame.display.set_caption("R1 POLICY CONTROL")
    pygame.joystick.init()

    if pygame.joystick.get_count() <= 0:
        print("[policy] no gamepad found; using keyboard commands.")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[policy] gamepad={joystick.get_name()}")
    return joystick


def scripted_command(elapsed_s: float) -> np.ndarray | None:
    vx = get_float_env("POLICY_CMD_VX", 0.0)
    vy = get_float_env("POLICY_CMD_VY", 0.0)
    yaw = get_float_env("POLICY_CMD_YAW", 0.0)
    if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(yaw) < 1e-6:
        return None

    start_s = max(0.0, get_float_env("POLICY_CMD_START_TIME", 0.0))
    stop_s = get_float_env("POLICY_CMD_STOP_TIME", 0.0)
    ramp_s = max(0.0, get_float_env("POLICY_CMD_RAMP_TIME", 0.0))
    if elapsed_s < start_s or (stop_s > 0.0 and elapsed_s >= stop_s):
        scale = 0.0
    elif ramp_s > 0.0:
        scale = smoothstep01((elapsed_s - start_s) / ramp_s)
    else:
        scale = 1.0
    return np.array([vx, vy, yaw], dtype=np.float32) * scale


def read_command(joystick: pygame.joystick.Joystick | None, elapsed_s: float) -> np.ndarray:
    scripted = scripted_command(elapsed_s)
    if scripted is not None:
        return scripted

    vx_scale = get_float_env("MANUAL_CMD_VX_SCALE", 0.1)
    vy_scale = get_float_env("MANUAL_CMD_VY_SCALE", 0.05)
    yaw_scale = get_float_env("MANUAL_CMD_YAW_SCALE", 0.4)
    raw_vx = 0.0
    raw_vy = 0.0
    raw_yaw = 0.0

    if joystick is not None:
        def apply_deadzone(value: float, threshold: float = 0.15) -> float:
            return 0.0 if abs(value) < threshold else value

        axis_left_x = apply_deadzone(joystick.get_axis(0))
        axis_left_y = apply_deadzone(joystick.get_axis(1))
        axis_right_x = apply_deadzone(joystick.get_axis(3))
        raw_vx = -axis_left_y * vx_scale
        raw_vy = -axis_left_x * vy_scale
        raw_yaw = -axis_right_x * yaw_scale
    else:
        keys = pygame.key.get_pressed()
        raw_vx = vx_scale if keys[pygame.K_w] else (-vx_scale if keys[pygame.K_s] else 0.0)
        raw_vy = vy_scale if keys[pygame.K_a] else (-vy_scale if keys[pygame.K_d] else 0.0)
        raw_yaw = yaw_scale if keys[pygame.K_q] else (-yaw_scale if keys[pygame.K_e] else 0.0)

    return np.array([raw_vx, raw_vy, raw_yaw], dtype=np.float32)


def main() -> None:
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    session, input_name = resolve_policy_session()

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)
    sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sport_sub.Init(sport_state_handler, 10)

    for i in range(len(config.MOTOR_JOINT_NAMES)):
        cmd.motor_cmd[i].mode = 0x01
    set_motor_position_targets(config.DEFAULT_Q)

    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()

    joystick = init_pygame()
    logger = SimStateLogger(__file__, joint_names=config.MOTOR_JOINT_NAMES)

    controlled = config.CONTROLLED_INDICES
    default_controlled_q = config.DEFAULT_Q[controlled]
    action_scale = config.ACTION_SCALE[controlled]
    last_action = np.zeros(config.ACTION_DIM, dtype=np.float32)
    previous_target_q = config.DEFAULT_Q.copy()
    smoothed_commands = np.zeros(3, dtype=np.float32)

    alpha = get_float_env("COMMAND_SMOOTHING_ALPHA", 0.1)
    warmup_seconds = max(0.0, get_float_env("POLICY_WARMUP_SECONDS", 0.0))
    policy_fade_seconds = max(0.0, get_float_env("POLICY_FADE_SECONDS", 0.0))
    action_clip = max(0.0, get_float_env("POLICY_ACTION_CLIP", 0.0))
    target_rate_limit = max(0.0, get_float_env("POLICY_TARGET_RATE_LIMIT", 4.0))
    fall_guard_gravity_z = get_float_env("POLICY_FALL_GUARD_GRAVITY_Z", -0.55)
    spin_threshold = max(0.0, get_float_env("SLEEP_SPIN_THRESHOLD", 0.0005))
    command_limits = command_limit_array()
    command_slew_limits = command_slew_array()
    print(
        "[policy] safety "
        f"cmd_limit={command_limits.tolist()} "
        f"cmd_slew={command_slew_limits.tolist()} "
        f"target_rate_limit={target_rate_limit:g} "
        f"fall_guard_gravity_z={fall_guard_gravity_z:g}"
    )
    # Gait clock must match the training `phase` observation: period 0.6 s and a
    # stand gate at ||command|| < 0.1 (see mjlab velocity task observations.phase).
    gait_period = max(1e-3, get_float_env("POLICY_GAIT_PERIOD", 0.6))
    gait_stand_threshold = max(0.0, get_float_env("POLICY_GAIT_STAND_THRESHOLD", 0.1))

    gait_time = 0.0
    warmup_done = warmup_seconds == 0.0
    warmup_start: float | None = None
    warmup_q0: np.ndarray | None = None
    policy_start: float | None = None
    last_guard_print = 0.0
    last_command_print = 0.0
    step = 0
    t0 = time.perf_counter()
    next_step_time: float | None = None

    try:
        while not stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                ):
                    print("[policy] stop requested.")
                    stop_event.set()
                    break

            snapshot = read_state_snapshot()
            if snapshot is None:
                time.sleep(0.002)
                next_step_time = None
                continue

            if next_step_time is None:
                next_step_time = time.perf_counter()

            sleep_until(next_step_time, spin_threshold_s=spin_threshold)
            step_start = time.perf_counter()
            if step_start - next_step_time > 2.0 * config.CONTROL_DT:
                next_step_time = step_start

            q_current, dq_current, gyro, quat, base_pos, base_vel = snapshot

            if not warmup_done:
                if warmup_start is None:
                    warmup_start = step_start
                    warmup_q0 = q_current.copy()
                    print(f"[policy] FixStand warmup {warmup_seconds:.1f}s.")
                assert warmup_q0 is not None
                warmup_alpha = smoothstep01((step_start - warmup_start) / warmup_seconds)
                target_q = (1.0 - warmup_alpha) * warmup_q0 + warmup_alpha * config.DEFAULT_Q
                set_motor_position_targets(target_q)
                if warmup_alpha >= 1.0:
                    warmup_done = True
                    last_action.fill(0.0)
                    smoothed_commands.fill(0.0)
                    previous_target_q = config.DEFAULT_Q.copy()
                    gait_time = 0.0
                    policy_start = time.perf_counter()
                    next_step_time = time.perf_counter()
                    print("[policy] warmup complete; ONNX policy enabled.")
                else:
                    next_step_time += config.CONTROL_DT
                    continue

            projected_gravity = compute_projected_gravity(quat)
            if projected_gravity[2] > fall_guard_gravity_z:
                now = time.perf_counter()
                if now - last_guard_print > 0.5:
                    print("[policy] fall guard active; holding DEFAULT_Q.")
                    last_guard_print = now
                last_action.fill(0.0)
                smoothed_commands.fill(0.0)
                previous_target_q = config.DEFAULT_Q.copy()
                gait_time = 0.0
                set_motor_position_targets(config.DEFAULT_Q)
                next_step_time += config.CONTROL_DT
                continue

            raw_command = read_command(joystick, step_start - t0)
            smoothed_commands = filter_command(
                raw_command,
                smoothed_commands,
                alpha=alpha,
                dt=config.CONTROL_DT,
                limits=command_limits,
                slew_limits=command_slew_limits,
            )
            if np.linalg.norm(smoothed_commands) >= gait_stand_threshold:
                now = time.perf_counter()
                if now - last_command_print > 0.5:
                    print(
                        f"[policy] cmd vx={smoothed_commands[0]:.2f} "
                        f"vy={smoothed_commands[1]:.2f} yaw={smoothed_commands[2]:.2f}"
                    )
                    last_command_print = now

            # Match the training `phase` observation exactly
            # (unitree_rl_mjlab src/tasks/velocity/mdp/observations.py):
            #   * free-running clock that always advances by the control dt,
            #   * hard on/off gate by command norm (no gait_scale amplitude ramp),
            #   * unit-amplitude sin/cos when walking, zero when standing.
            # The previous ramped/snapped clock fed out-of-distribution phase
            # amplitudes right as a command turned on, which destabilised the gait.
            gait_time += config.CONTROL_DT
            phase_ratio = (gait_time % gait_period) / gait_period
            if np.linalg.norm(smoothed_commands) < gait_stand_threshold:
                gait_phase = np.zeros(2, dtype=np.float32)
            else:
                gait_phase = np.array(
                    [np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)],
                    dtype=np.float32,
                )

            q_rel = q_current[controlled] - default_controlled_q
            dq_rel = dq_current[controlled]
            obs = np.concatenate(
                [
                    gyro,
                    projected_gravity,
                    smoothed_commands,
                    gait_phase,
                    q_rel,
                    dq_rel,
                    last_action,
                ]
            ).astype(np.float32)
            if obs.shape[0] != config.OBS_DIM:
                raise RuntimeError(f"Observation shape mismatch: {obs.shape[0]} != {config.OBS_DIM}")

            raw_action = session.run(None, {input_name: obs[None, :]})[0][0].astype(np.float32)
            if raw_action.shape[0] != config.ACTION_DIM:
                raise RuntimeError(
                    f"Action shape mismatch: {raw_action.shape[0]} != {config.ACTION_DIM}"
                )

            action = raw_action
            if action_clip > 0.0:
                action = np.clip(action, -action_clip, action_clip)
            if policy_fade_seconds > 0.0:
                if policy_start is None:
                    policy_start = step_start
                action *= smoothstep01((step_start - policy_start) / policy_fade_seconds)

            target_q = config.DEFAULT_Q.copy()
            target_q[controlled] = default_controlled_q + action * action_scale
            if target_rate_limit > 0.0:
                max_delta = target_rate_limit * config.CONTROL_DT
                target_q = previous_target_q + np.clip(
                    target_q - previous_target_q,
                    -max_delta,
                    max_delta,
                )
            previous_target_q = target_q.copy()
            set_motor_position_targets(target_q)
            # Feed the applied action back as the `last_action` observation next
            # step. Without this it stayed zero, corrupting the final 24 obs values
            # and destabilising the gait (direct-eval sets this; the bridge did not).
            last_action = action.copy()

            action_full = np.zeros(len(config.MOTOR_JOINT_NAMES), dtype=np.float32)
            action_full[controlled] = action
            logger.log(
                step=step,
                t=step_start - t0,
                target_q=target_q,
                q=q_current,
                dq=dq_current,
                action=action_full,
                quat=quat,
                gyro=gyro,
                proj_grav=projected_gravity,
                base_pos=base_pos,
                base_vel=base_vel,
                commands=smoothed_commands,
                gait_phase=gait_phase,
                gait_scale=float(np.linalg.norm(gait_phase) > 0.0),
                gait_time=gait_time,
            )

            step += 1
            next_step_time += config.CONTROL_DT
    finally:
        logger.close()
        pygame.quit()


if __name__ == "__main__":
    main()
