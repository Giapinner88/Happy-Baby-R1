import argparse
import csv as csvlib
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from threading import Thread
import threading

import mujoco
import importlib
import numpy as np
import onnxruntime as ort

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config
from state_logger import SimStateLogger

_JOINT_NAMES = config.MOTOR_JOINT_NAMES
DEFAULT_Q = config.DEFAULT_Q.tolist()


def _xml_joint_name(motor_name: str) -> str:
    return motor_name if motor_name.endswith("_joint") else f"{motor_name}_joint"


def init_state_from_csv(model, data, csv_path: str, row_index: int = 0):
    """
    Khởi tạo trạng thái MuJoCo từ một hàng trong file CSV log.
    - qpos[7:36] ← q_<joint>   (29 góc khớp)
    - qvel[6:35] ← dq_<joint>  (29 vận tốc khớp)
    - qpos[3:7]  ← imu_quat    (hướng base: w,x,y,z)
    - qpos[0:3]  giữ nguyên    (vị trí XYZ từ spawn mặc định)
    """
    path = Path(csv_path)
    if not path.exists():
        print(f"[init-csv] Không tìm thấy: {csv_path}", file=sys.stderr)
        return

    with open(path, newline="") as f:
        reader = csvlib.DictReader(f)
        target_row = None
        for i, row in enumerate(reader):
            if i == row_index:
                target_row = row
                break

    if target_row is None:
        print(f"[init-csv] Không tìm thấy row {row_index} trong CSV.", file=sys.stderr)
        return

    # 1. Set góc khớp (qpos[7..35])
    missing_q = []
    for jname in _JOINT_NAMES:
        col = f"q_{jname}"
        if col not in target_row:
            missing_q.append(col)
            continue
        joint_id  = model.joint(_xml_joint_name(jname)).id
        qpos_adr  = model.jnt_qposadr[joint_id]
        data.qpos[qpos_adr] = float(target_row[col])

    # 2. Set vận tốc khớp (qvel[6..34])
    for jname in _JOINT_NAMES:
        col = f"dq_{jname}"
        if col not in target_row:
            continue
        joint_id = model.joint(_xml_joint_name(jname)).id
        dof_adr  = model.jnt_dofadr[joint_id]
        data.qvel[dof_adr] = float(target_row[col])

    # 3. Set hướng base từ IMU quaternion (w,x,y,z)
    try:
        qw = float(target_row.get("imu_quat_w", 1.0))
        qx = float(target_row.get("imu_quat_x", 0.0))
        qy = float(target_row.get("imu_quat_y", 0.0))
        qz = float(target_row.get("imu_quat_z", 0.0))
        base_jid = model.joint("floating_base_joint").id
        base_adr = model.jnt_qposadr[base_jid]  # = 0
        # qpos[base_adr+0:3] = position — giữ nguyên (spawn height)
        data.qpos[base_adr + 3] = qw
        data.qpos[base_adr + 4] = qx
        data.qpos[base_adr + 5] = qy
        data.qpos[base_adr + 6] = qz
    except Exception as e:
        print(f"[init-csv] Cảnh báo khi set base quat: {e}")

    mujoco.mj_forward(model, data)

    print(f"[init-csv] State đã set từ row {row_index} của '{path.name}'")
    if missing_q:
        print(f"[init-csv] Thiếu cột: {missing_q}")


def init_default_q(model, data):
    """Match the simulator spawn pose to the policy's initial hold command."""
    for joint_name, q in zip(_JOINT_NAMES, DEFAULT_Q):
        joint_id = model.joint(_xml_joint_name(joint_name)).id
        qpos_adr = model.jnt_qposadr[joint_id]
        dof_adr = model.jnt_dofadr[joint_id]
        data.qpos[qpos_adr] = q
        data.qvel[dof_adr] = 0.0

    mujoco.mj_forward(model, data)
    print("[init-default-q] State đã set theo DEFAULT_Q của policy.")


def set_hanging_equality(model, data, active: bool):
    """Toggle the optional scene equality used by hanging/debug scenes."""
    equality_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "r1_hanging_connect"
    )
    if equality_id < 0:
        return
    data.eq_active[equality_id] = 1 if active else 0
    state = "active" if active else "disabled"
    print(f"[sim-config] r1_hanging_connect {state}.")


def get_float_env(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.")
        return default


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


def quat_apply_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the body frame for MuJoCo wxyz quats."""
    qvec = np.array(vec, dtype=np.float64)
    result = np.empty(3, dtype=np.float64)
    mujoco.mju_rotVecQuat(result, qvec, np.array([quat[0], -quat[1], -quat[2], -quat[3]]))
    return result.astype(np.float32)


def compute_projected_gravity(quat: np.ndarray) -> np.ndarray:
    return quat_apply_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))


def scripted_command(elapsed_s: float) -> np.ndarray:
    vx = get_float_env("POLICY_CMD_VX", 0.0)
    vy = get_float_env("POLICY_CMD_VY", 0.0)
    yaw = get_float_env("POLICY_CMD_YAW", 0.0)
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


def _resolve_policy_session() -> tuple[ort.InferenceSession, str]:
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

    print(f"[direct-policy] name={config.POLICY_NAME}")
    print(f"[direct-policy] onnx={policy_path}")
    print(f"[direct-policy] obs_dim={config.OBS_DIM} action_dim={config.ACTION_DIM}")
    print(f"[direct-policy] providers={session.get_providers()}")
    return session, input_info.name


def patch_mjlab_position_actuators(model: mujoco.MjModel) -> None:
    """Convert the XML motor actuators to MJLab-style built-in position actuators."""
    patched = 0
    for joint_name, kp, kd in zip(
        config.MOTOR_JOINT_NAMES,
        config.KP_ARRAY,
        config.KD_ARRAY,
    ):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
        )
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, _xml_joint_name(joint_name)
        )
        if actuator_id < 0 or joint_id < 0:
            raise RuntimeError(f"Missing actuator/joint for {joint_name!r}.")

        effort_limit = float(max(abs(model.actuator_ctrlrange[actuator_id])))
        if effort_limit <= 0.0:
            effort_limit = 60.0
        joint_range = model.jnt_range[joint_id]
        delta = effort_limit / float(kp)

        model.actuator_dyntype[actuator_id] = int(mujoco.mjtDyn.mjDYN_NONE)
        model.actuator_gaintype[actuator_id] = int(mujoco.mjtGain.mjGAIN_FIXED)
        model.actuator_biastype[actuator_id] = int(mujoco.mjtBias.mjBIAS_AFFINE)
        model.actuator_gainprm[actuator_id, :] = 0.0
        model.actuator_biasprm[actuator_id, :] = 0.0
        model.actuator_gainprm[actuator_id, 0] = float(kp)
        model.actuator_biasprm[actuator_id, 1] = -float(kp)
        model.actuator_biasprm[actuator_id, 2] = -float(kd)
        model.actuator_ctrllimited[actuator_id] = 0
        model.actuator_forcelimited[actuator_id] = 1
        model.actuator_ctrlrange[actuator_id] = np.array(
            [joint_range[0] - delta, joint_range[1] + delta], dtype=np.float64
        )
        model.actuator_forcerange[actuator_id] = np.array(
            [-effort_limit, effort_limit], dtype=np.float64
        )
        model.dof_armature[model.jnt_dofadr[joint_id]] = 0.01
        model.dof_frictionloss[model.jnt_dofadr[joint_id]] = 0.0
        patched += 1

    print(f"[direct-policy] patched {patched} actuators to MJLab-style position control.")


class DirectMjlabPolicyController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self.session, self.input_name = _resolve_policy_session()
        self.logger = SimStateLogger(__file__, joint_names=config.MOTOR_JOINT_NAMES)
        self.controlled = config.CONTROLLED_INDICES
        self.default_controlled_q = config.DEFAULT_Q[self.controlled]
        self.action_scale = config.ACTION_SCALE[self.controlled]
        self.last_action = np.zeros(config.ACTION_DIM, dtype=np.float32)
        self.previous_target_q = config.DEFAULT_Q.copy()
        self.smoothed_commands = np.zeros(3, dtype=np.float32)
        self.qpos_addr = np.array(
            [
                model.jnt_qposadr[model.joint(_xml_joint_name(name)).id]
                for name in config.MOTOR_JOINT_NAMES
            ],
            dtype=np.int64,
        )
        self.qvel_addr = np.array(
            [
                model.jnt_dofadr[model.joint(_xml_joint_name(name)).id]
                for name in config.MOTOR_JOINT_NAMES
            ],
            dtype=np.int64,
        )
        self.actuator_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in config.MOTOR_JOINT_NAMES
            ],
            dtype=np.int64,
        )
        self.base_joint_addr = model.jnt_qposadr[model.joint("floating_base_joint").id]
        self.pelvis_body_id = model.body("pelvis").id
        self.alpha = get_float_env("COMMAND_SMOOTHING_ALPHA", 0.1)
        self.policy_fade_seconds = max(0.0, get_float_env("POLICY_FADE_SECONDS", 0.0))
        self.action_clip = max(0.0, get_float_env("POLICY_ACTION_CLIP", 0.0))
        self.target_rate_limit = max(0.0, get_float_env("POLICY_TARGET_RATE_LIMIT", 0.0))
        self.fall_guard_gravity_z = get_float_env("POLICY_FALL_GUARD_GRAVITY_Z", -0.55)
        self.command_limits = command_limit_array()
        self.command_slew_limits = command_slew_array()
        self.gait_period = max(1e-3, get_float_env("POLICY_GAIT_PERIOD", 0.6))
        self.gait_stand_threshold = max(0.0, get_float_env("POLICY_GAIT_STAND_THRESHOLD", 0.1))
        self.gait_time = 0.0
        self.obs_delay_steps = max(0, int(get_float_env("POLICY_OBS_DELAY_STEPS", 0)))
        self.obs_buffer: list[np.ndarray] = []
        self.policy_start: float | None = None
        self.last_command_print = 0.0
        self.last_guard_print = 0.0
        self.step_count = 0
        print(
            "[direct-policy] safety "
            f"cmd_limit={self.command_limits.tolist()} "
            f"cmd_slew={self.command_slew_limits.tolist()} "
            f"target_rate_limit={self.target_rate_limit:g} "
            f"fall_guard_gravity_z={self.fall_guard_gravity_z:g}"
        )

    def close(self) -> None:
        self.logger.close()

    def _read_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        q = np.asarray(self.data.qpos[self.qpos_addr], dtype=np.float32)
        dq = np.asarray(self.data.qvel[self.qvel_addr], dtype=np.float32)
        quat = np.asarray(
            self.data.qpos[self.base_joint_addr + 3 : self.base_joint_addr + 7],
            dtype=np.float32,
        )
        ang_vel_w = np.asarray(self.data.cvel[self.pelvis_body_id, 0:3], dtype=np.float32)
        ang_vel_b = quat_apply_inverse(quat, ang_vel_w)
        base_pos = np.asarray(
            self.data.qpos[self.base_joint_addr : self.base_joint_addr + 3],
            dtype=np.float32,
        )
        base_vel = np.asarray(self.data.qvel[0:3], dtype=np.float32)
        return q, dq, quat, ang_vel_b, base_pos, base_vel

    def _write_targets(self, target_q: np.ndarray) -> None:
        self.data.ctrl[self.actuator_ids] = target_q.astype(np.float64)

    def step(self, sim_time: float) -> None:
        q_current, dq_current, quat, ang_vel_b, base_pos, base_vel = self._read_state()
        projected_gravity = compute_projected_gravity(quat)
        if projected_gravity[2] > self.fall_guard_gravity_z:
            now = time.perf_counter()
            if now - self.last_guard_print > 0.5:
                print("[direct-policy] fall guard active; holding DEFAULT_Q.")
                self.last_guard_print = now
            self.last_action.fill(0.0)
            self.smoothed_commands.fill(0.0)
            self.previous_target_q = config.DEFAULT_Q.copy()
            self.gait_time = 0.0
            self._write_targets(config.DEFAULT_Q)
            return

        raw_command = scripted_command(sim_time)
        self.smoothed_commands = filter_command(
            raw_command,
            self.smoothed_commands,
            alpha=self.alpha,
            dt=config.CONTROL_DT,
            limits=self.command_limits,
            slew_limits=self.command_slew_limits,
        )
        if np.linalg.norm(self.smoothed_commands) >= self.gait_stand_threshold:
            now = time.perf_counter()
            if now - self.last_command_print > 0.5:
                print(
                    f"[direct-policy] cmd vx={self.smoothed_commands[0]:.2f} "
                    f"vy={self.smoothed_commands[1]:.2f} yaw={self.smoothed_commands[2]:.2f}"
                )
                self.last_command_print = now

        self.gait_time += config.CONTROL_DT
        phase_ratio = (self.gait_time % self.gait_period) / self.gait_period
        if np.linalg.norm(self.smoothed_commands) < self.gait_stand_threshold:
            gait_phase = np.zeros(2, dtype=np.float32)
        else:
            gait_phase = np.array(
                [math.sin(2 * math.pi * phase_ratio), math.cos(2 * math.pi * phase_ratio)],
                dtype=np.float32,
            )

        q_rel = q_current[self.controlled] - self.default_controlled_q
        dq_rel = dq_current[self.controlled]
        obs = np.concatenate(
            [
                ang_vel_b,
                projected_gravity,
                self.smoothed_commands,
                gait_phase,
                q_rel,
                dq_rel,
                self.last_action,
            ]
        ).astype(np.float32)
        if obs.shape[0] != config.OBS_DIM:
            raise RuntimeError(f"Observation shape mismatch: {obs.shape[0]} != {config.OBS_DIM}")

        # Diagnostic-only: emulate DDS-bridge control latency by feeding the policy
        # an observation delayed by POLICY_OBS_DELAY_STEPS control steps (default 0,
        # i.e. no change). Used to prove that loop latency — not actuator semantics —
        # is what destabilises the split-process bridge path.
        if self.obs_delay_steps > 0:
            self.obs_buffer.append(obs)
            obs = self.obs_buffer.pop(0) if len(self.obs_buffer) > self.obs_delay_steps else self.obs_buffer[0]

        raw_action = self.session.run(None, {self.input_name: obs[None, :]})[0][0].astype(np.float32)
        if raw_action.shape[0] != config.ACTION_DIM:
            raise RuntimeError(
                f"Action shape mismatch: {raw_action.shape[0]} != {config.ACTION_DIM}"
            )

        action = raw_action
        if self.action_clip > 0.0:
            action = np.clip(action, -self.action_clip, self.action_clip)
        if self.policy_fade_seconds > 0.0:
            if self.policy_start is None:
                self.policy_start = sim_time
            action = action * smoothstep01((sim_time - self.policy_start) / self.policy_fade_seconds)

        target_q = config.DEFAULT_Q.copy()
        target_q[self.controlled] = self.default_controlled_q + action * self.action_scale
        if self.target_rate_limit > 0.0:
            max_delta = self.target_rate_limit * config.CONTROL_DT
            target_q = self.previous_target_q + np.clip(
                target_q - self.previous_target_q,
                -max_delta,
                max_delta,
            )
        self.previous_target_q = target_q.copy()
        self._write_targets(target_q)
        self.last_action = action.copy()

        action_full = np.zeros(len(config.MOTOR_JOINT_NAMES), dtype=np.float32)
        action_full[self.controlled] = action
        self.logger.log(
            step=self.step_count,
            t=sim_time,
            target_q=target_q,
            q=q_current,
            dq=dq_current,
            action=action_full,
            quat=quat,
            gyro=ang_vel_b,
            proj_grav=projected_gravity,
            base_pos=base_pos,
            base_vel=base_vel,
            commands=self.smoothed_commands,
            gait_phase=gait_phase,
            gait_scale=float(np.linalg.norm(gait_phase) > 0.0),
            gait_time=self.gait_time,
        )
        self.step_count += 1


# ─── Parse arguments ─────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser(description="Unitree MuJoCo Simulator")
_parser.add_argument(
    "--init-csv", default=None, metavar="CSV",
    help="Đường dẫn CSV log để khởi tạo trạng thái ban đầu của sim."
)
_parser.add_argument(
    "--init-row", type=int, default=0, metavar="N",
    help="Hàng trong CSV để đọc trạng thái (mặc định: 0 = hàng đầu tiên)."
)
_parser.add_argument("--robot", default=None, help="Must be r1 for this local policy runtime.")
_parser.add_argument("--scene", default=None, help="Override scene file, e.g. scene.xml or scene_hanging.xml.")
_parser.add_argument("--domain-id", type=int, default=None, help="Override DDS domain id.")
_parser.add_argument("--interface", default=None, help="Override DDS network interface.")
_args, _unknown = _parser.parse_known_args()

if _args.robot:
    if _args.robot != "r1":
        raise ValueError("sim/unitree_mujoco_policy is R1-only; do not run it with G1/other robots.")
    config.ROBOT = _args.robot
    config.USE_HG_IDL = True
if _args.scene:
    os.environ["ROBOT_SCENE_NAME"] = _args.scene
    config.ROBOT_SCENE_NAME = _args.scene
    config.ROBOT_SCENE = str(config._resolve_robot_scene())
if _args.domain_id is not None:
    config.DOMAIN_ID = _args.domain_id
if _args.interface:
    config.INTERFACE = _args.interface

from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand


locker = threading.Lock()
stop_event = threading.Event()

print(
    f"[sim-config] robot={config.ROBOT} scene={config.ROBOT_SCENE} "
    f"domain={config.DOMAIN_ID} interface={config.INTERFACE} hg_idl={config.USE_HG_IDL}"
)
mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

direct_eval_mode = os.environ.get("POLICY_DIRECT_EVAL", "0").lower() in {"1", "true", "yes"}
bridge_actuator_mode = os.environ.get("UNITREE_BRIDGE_ACTUATOR_MODE", "torque").strip().lower()
bridge_position_mode = not direct_eval_mode and bridge_actuator_mode == "position"
if direct_eval_mode or bridge_position_mode:
    patch_mjlab_position_actuators(mj_model)
if bridge_position_mode:
    print("[sim-config] bridge will use MJLab-style position actuator targets.")

disable_hanging_equality = os.environ.get("DISABLE_HANGING_EQUALITY", "0").lower()
if disable_hanging_equality in {"1", "true", "yes"}:
    set_hanging_equality(mj_model, mj_data, active=False)

if os.environ.get("INIT_DEFAULT_Q", "0").lower() in {"1", "true", "yes"}:
    init_default_q(mj_model, mj_data)


class _HeadlessViewer:
    """Minimal viewer replacement for headless operation.

    Provides the methods used by the rest of the simulator (`is_running`,
    `sync`) so the simulation loop can run without creating an OpenGL window.
    """
    def __init__(self):
        self._running = True

    def is_running(self):
        return self._running

    def sync(self):
        # no-op for headless
        return

    def close(self):
        self._running = False


def request_stop(signum: int, _frame) -> None:
    print(f"[sim] received signal {signum}; stopping.")
    stop_event.set()
    try:
        viewer.close()
    except Exception:
        if hasattr(viewer, "_running"):
            viewer._running = False


headless_mode = os.environ.get("MUJOCO_HEADLESS", "0").lower() in {"1", "true", "yes"}
if headless_mode:
    viewer = _HeadlessViewer()
    print("[sim] Headless mode enabled.")
else:
    try:
        try:
            # Import mujoco.viewer lazily — on some Wayland setups importing the
            # viewer module can trigger a native crash. Import inside try so we
            # can fall back to headless if it fails.
            mujoco_viewer = importlib.import_module('mujoco.viewer')
            if config.ENABLE_ELASTIC_BAND:
                elastic_band = ElasticBand()
                if config.ROBOT in config.HUMANOID_HG_ROBOTS:
                    band_attached_link = mj_model.body("torso_link").id
                else:
                    band_attached_link = mj_model.body("base_link").id
                viewer = mujoco_viewer.launch_passive(
                    mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
                )
            else:
                viewer = mujoco_viewer.launch_passive(mj_model, mj_data)
        except Exception:
            # Fallback to headless viewer if the platform/OpenGL driver fails
            print("[sim] Viewer initialization failed — falling back to headless mode.")
            viewer = _HeadlessViewer()
    except Exception:
        # Fallback to headless viewer if the platform/OpenGL driver fails
        print("[sim] Viewer initialization failed — falling back to headless mode.")
        viewer = _HeadlessViewer()

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    direct_controller = None
    next_direct_control_time = 0.0
    if direct_eval_mode:
        direct_controller = DirectMjlabPolicyController(mj_model, mj_data)
        print("[direct-policy] DDS bridge disabled for direct MJLab-style eval.")
    else:
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        unitree = UnitreeSdk2Bridge(mj_model, mj_data, data_lock=locker)

        if config.USE_JOYSTICK:
            unitree.SetupJoystick(device_id=config.JOYSTICK_DEVICE, js_type=config.JOYSTICK_TYPE)
        if config.PRINT_SCENE_INFORMATION:
            unitree.PrintSceneInformation()

    sim_duration = float(os.environ.get("MUJOCO_SIM_DURATION_SECONDS", "0") or 0.0)
    sim_start = time.perf_counter()
    try:
        while not stop_event.is_set() and viewer.is_running():
            if sim_duration > 0.0 and time.perf_counter() - sim_start >= sim_duration:
                print(f"[sim] duration reached: {sim_duration:g}s; stopping.")
                break

            step_start = time.perf_counter()

            locker.acquire()

            if direct_controller is not None and mj_data.time + 1e-9 >= next_direct_control_time:
                direct_controller.step(float(mj_data.time))
                next_direct_control_time += config.CONTROL_DT

            if config.ENABLE_ELASTIC_BAND:
                if elastic_band.enable:
                    mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                        mj_data.qpos[:3], mj_data.qvel[:3]
                    )
            mujoco.mj_step(mj_model, mj_data)

            locker.release()

            time_until_next_step = mj_model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    finally:
        if direct_controller is not None:
            direct_controller.close()


def PhysicsViewerThread():
    while not stop_event.is_set() and viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


def _even_video_dimension(value: str, default: int) -> int:
    dimension = max(16, int(value or default))
    return dimension if dimension % 2 == 0 else dimension - 1


def _resolve_ffmpeg_exe() -> str | None:
    ffmpeg_exe = os.environ.get("FFMPEG_BINARY")
    if ffmpeg_exe:
        return ffmpeg_exe

    ffmpeg_exe = shutil.which("ffmpeg")
    if ffmpeg_exe:
        return ffmpeg_exe

    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def VideoRecorderThread():
    if os.environ.get("MUJOCO_RECORD_VIDEO", "0").lower() not in {"1", "true", "yes"}:
        return

    import cv2
    import numpy as np

    video_path = Path(
        os.environ.get("MUJOCO_RECORD_VIDEO_PATH", "mujoco_policy.mp4")
    ).expanduser()
    video_path.parent.mkdir(parents=True, exist_ok=True)
    width = _even_video_dimension(os.environ.get("MUJOCO_RECORD_VIDEO_WIDTH", ""), 1280)
    height = _even_video_dimension(os.environ.get("MUJOCO_RECORD_VIDEO_HEIGHT", ""), 720)
    fps = float(os.environ.get("MUJOCO_RECORD_VIDEO_FPS", "50"))
    fps = max(1.0, fps)
    codec = os.environ.get("MUJOCO_RECORD_VIDEO_CODEC", "h264").lower()

    mj_model.vis.global_.offwidth = max(mj_model.vis.global_.offwidth, width)
    mj_model.vis.global_.offheight = max(mj_model.vis.global_.offheight, height)
    try:
        renderer = mujoco.Renderer(mj_model, height=height, width=width)
    except Exception as exc:
        print(f"[video] failed to create MuJoCo renderer: {exc}", file=sys.stderr)
        return

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    body_name = "torso_link" if mj_model.body("torso_link").id >= 0 else "pelvis"
    camera.trackbodyid = mj_model.body(body_name).id
    camera.distance = float(os.environ.get("MUJOCO_RECORD_CAMERA_DISTANCE", "3.0"))
    camera.azimuth = float(os.environ.get("MUJOCO_RECORD_CAMERA_AZIMUTH", "90.0"))
    camera.elevation = float(os.environ.get("MUJOCO_RECORD_CAMERA_ELEVATION", "-15.0"))

    writer = None
    ffmpeg_proc = None
    ffmpeg_stderr = None
    if codec == "h264":
        ffmpeg_exe = _resolve_ffmpeg_exe()
        if ffmpeg_exe:
            ffmpeg_cmd = [
                ffmpeg_exe,
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{width}x{height}",
                "-r",
                f"{fps:g}",
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                os.environ.get("MUJOCO_RECORD_H264_PRESET", "veryfast"),
                "-crf",
                os.environ.get("MUJOCO_RECORD_H264_CRF", "18"),
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(video_path),
            ]
            ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        else:
            print("[video] ffmpeg not found; falling back to mp4v.", file=sys.stderr)

    if ffmpeg_proc is None:
        codec = "mp4v"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(video_path), fourcc, fps, (width, height))
        if not writer.isOpened():
            print(f"[video] failed to open writer: {video_path}", file=sys.stderr)
            renderer.close()
            return

    frame_dt = 1.0 / fps
    next_frame_time = time.perf_counter()
    frames = 0
    print(f"[video] recording {width}x{height}@{fps:g}fps codec={codec} -> {video_path}")
    try:
        while not stop_event.is_set() and viewer.is_running():
            now = time.perf_counter()
            if now < next_frame_time:
                time.sleep(min(frame_dt, next_frame_time - now))
                continue

            locker.acquire()
            try:
                renderer.update_scene(mj_data, camera=camera)
                rgb = renderer.render()
            finally:
                locker.release()

            rgb_frame = np.ascontiguousarray(rgb, dtype=np.uint8)
            if ffmpeg_proc is not None:
                try:
                    ffmpeg_proc.stdin.write(rgb_frame.tobytes())
                except (BrokenPipeError, AttributeError):
                    print("[video] ffmpeg pipe closed before recording finished.", file=sys.stderr)
                    break
            else:
                bgr = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                writer.write(bgr)
            frames += 1
            next_frame_time += frame_dt
    finally:
        if ffmpeg_proc is not None:
            if ffmpeg_proc.stdin is not None:
                ffmpeg_proc.stdin.close()
            try:
                ffmpeg_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                ffmpeg_proc.kill()
                ffmpeg_proc.wait(timeout=3)
            if ffmpeg_proc.stderr is not None:
                ffmpeg_stderr = ffmpeg_proc.stderr.read().decode("utf-8", errors="replace").strip()
            if ffmpeg_proc.returncode != 0 and ffmpeg_stderr:
                print(f"[video] ffmpeg failed: {ffmpeg_stderr}", file=sys.stderr)
        else:
            writer.release()
        renderer.close()
        print(f"[video] saved {frames} frames codec={codec} -> {video_path}")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    # ── Khởi tạo state từ CSV nếu có --init-csv ──────────────────────────────
    if _args.init_csv:
        # Tìm CSV mới nhất nếu dùng wildcard hoặc tên thư mục
        csv_path = Path(_args.init_csv)
        if not csv_path.exists():
            # Thử tìm trong sim_state_logs/
            # Prefer repo-level data directory for logs
            log_dir = Path(__file__).resolve().parents[2] / "data" / "sim_state_logs"
            matches = sorted(log_dir.glob("*.csv"),
                             key=lambda p: p.stat().st_mtime, reverse=True)
            csv_path = matches[0] if matches else csv_path
        init_state_from_csv(mj_model, mj_data, str(csv_path), _args.init_row)

    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)
    video_thread = Thread(target=VideoRecorderThread)

    viewer_thread.start()
    sim_thread.start()
    video_thread.start()

    try:
        sim_thread.join()
    finally:
        stop_event.set()
        try:
            viewer.close()
        except Exception:
            pass
        viewer_thread.join(timeout=2)
        video_thread.join(timeout=5)
