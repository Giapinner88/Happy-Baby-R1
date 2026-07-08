#!/usr/bin/env python3
"""Load the local R1 MuJoCo model and run a free or hanging movement test."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[1]
R1_MUJOCO_ASSET_ROOT = REPO_ROOT / "asset" / "mujoco" / "unitree_robots" / "r1"
DEFAULT_SCENE = R1_MUJOCO_ASSET_ROOT / "scene.xml"
DEFAULT_HANGING_SCENE = R1_MUJOCO_ASSET_ROOT / "scene_hanging.xml"
ENV_SCENES = {
    "flat": DEFAULT_HANGING_SCENE,
    "stairs": R1_MUJOCO_ASSET_ROOT / "scene_stairs.xml",
    "slope": R1_MUJOCO_ASSET_ROOT / "scene_slope.xml",
    "obstacles": R1_MUJOCO_ASSET_ROOT / "scene_obstacles.xml",
}

DEFAULT_Q = {
    "left_hip_pitch_joint": -0.1,
    "left_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.1,
    "right_knee_joint": 0.3,
    "right_ankle_pitch_joint": -0.2,
    "left_shoulder_pitch_joint": 0.35,
    "left_shoulder_roll_joint": 0.18,
    "left_elbow_joint": 0.87,
    "right_shoulder_pitch_joint": 0.35,
    "right_shoulder_roll_joint": -0.18,
    "right_elbow_joint": 0.87,
}

WAVE_JOINTS = (
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=None,
        help="Override the MuJoCo XML. Otherwise --env selects the scene.",
    )
    parser.add_argument(
        "--env",
        choices=tuple(ENV_SCENES),
        default="flat",
        help="Environment scene to load.",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=0.25)
    parser.add_argument("--frequency", type=float, default=0.5)
    mode_arg = parser.add_argument(
        "--motion-mode",
        "--mode",
        dest="mode",
        choices=("kinematic", "dynamic"),
        default="dynamic",
        help="kinematic writes joint qpos directly; dynamic sends sine torque commands.",
    )
    mode_arg.metavar = "MOTION_MODE"
    parser.add_argument(
        "--support-mode",
        choices=("free", "hanging"),
        default="hanging",
        help="free disables the hang equality; hanging connects the world and pelvis sites.",
    )
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--base-height", type=float, default=0.74)
    return parser.parse_args()


def resolve_model_path(args: argparse.Namespace) -> Path:
    if args.model is not None:
        return args.model.expanduser().resolve()
    if args.env in ENV_SCENES:
        return ENV_SCENES[args.env].resolve()
    if args.support_mode == "free":
        return DEFAULT_SCENE.resolve()
    return DEFAULT_HANGING_SCENE.resolve()


def pin_base(model: mujoco.MjModel, data: mujoco.MjData, base_height: float) -> None:
    free_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
    if free_joint_id >= 0:
        qadr = model.jnt_qposadr[free_joint_id]
        data.qpos[qadr : qadr + 7] = [0.0, 0.0, base_height, 1.0, 0.0, 0.0, 0.0]
        vadr = model.jnt_dofadr[free_joint_id]
        data.qvel[vadr : vadr + 6] = 0.0


def reset_base(model: mujoco.MjModel, data: mujoco.MjData, base_height: float) -> None:
    pin_base(model, data, base_height)
    mujoco.mj_forward(model, data)


def hanging_equality_id(model: mujoco.MjModel) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "r1_hanging_connect")


def set_hanging_constraint(model: mujoco.MjModel, data: mujoco.MjData, active: bool) -> None:
    equality_id = hanging_equality_id(model)
    if equality_id >= 0:
        data.eq_active[equality_id] = active


def set_initial_pose(model: mujoco.MjModel, data: mujoco.MjData, base_height: float) -> None:
    pin_base(model, data, base_height)

    for joint_name, qpos in DEFAULT_Q.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            continue
        data.qpos[model.jnt_qposadr[joint_id]] = qpos
        data.qvel[model.jnt_dofadr[joint_id]] = 0.0

    mujoco.mj_forward(model, data)


def set_kinematic_wave(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    elapsed: float,
    amplitude: float,
    frequency: float,
) -> None:
    phase = 2.0 * math.pi * frequency * elapsed
    for index, joint_name in enumerate(WAVE_JOINTS):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            continue
        sign = -1.0 if index % 2 else 1.0
        base = DEFAULT_Q.get(joint_name, 0.0)
        data.qpos[model.jnt_qposadr[joint_id]] = base + sign * amplitude * math.sin(phase)
        data.qvel[model.jnt_dofadr[joint_id]] = sign * amplitude * 2.0 * math.pi * frequency * math.cos(phase)

    mujoco.mj_forward(model, data)


def set_dynamic_wave(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    elapsed: float,
    amplitude: float,
    frequency: float,
) -> None:
    phase = 2.0 * math.pi * frequency * elapsed
    data.ctrl[:] = 0.0
    for actuator_id in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or ""
        if not any(key in actuator_name for key in ("hip_pitch", "knee", "shoulder_pitch", "elbow")):
            continue
        sign = -1.0 if actuator_id % 2 else 1.0
        data.ctrl[actuator_id] = sign * amplitude * math.sin(phase)


def run_headless(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    set_hanging_constraint(model, data, args.support_mode == "hanging")
    start = time.perf_counter()
    while True:
        elapsed = time.perf_counter() - start
        if elapsed >= args.duration:
            break
        if args.mode == "kinematic":
            set_kinematic_wave(model, data, elapsed, args.amplitude, args.frequency)
            continue
        else:
            set_dynamic_wave(model, data, elapsed, args.amplitude, args.frequency)
        mujoco.mj_step(model, data)


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> None:
    import mujoco.viewer

    glfw = mujoco.glfw.glfw
    support_mode = args.support_mode
    set_hanging_constraint(model, data, support_mode == "hanging")

    def key_callback(key: int) -> None:
        nonlocal support_mode
        if key == glfw.KEY_F:
            support_mode = "free"
            set_hanging_constraint(model, data, False)
            print("[viewer] support_mode=free")
        elif key == glfw.KEY_H:
            support_mode = "hanging"
            reset_base(model, data, args.base_height)
            set_hanging_constraint(model, data, True)
            print("[viewer] support_mode=hanging")

    print("[viewer] Click the MuJoCo window, then press H for hanging or F for free.")
    start = time.perf_counter()
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()
            elapsed = step_start - start
            if elapsed >= args.duration:
                break
            if args.mode == "kinematic":
                set_kinematic_wave(model, data, elapsed, args.amplitude, args.frequency)
            else:
                set_dynamic_wave(model, data, elapsed, args.amplitude, args.frequency)
                mujoco.mj_step(model, data)
            viewer.sync()
            sleep_time = max(0.0, model.opt.timestep - (time.perf_counter() - step_start))
            time.sleep(sleep_time)


def main() -> int:
    args = parse_args()
    model_path = resolve_model_path(args)
    if not model_path.exists():
        print(f"Missing MuJoCo model: {model_path}")
        return 2

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    set_initial_pose(model, data, args.base_height)

    print(f"Loaded: {model_path}")
    print(f"nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} nu={model.nu}")
    print(
        f"env={args.env} motion_mode={args.mode} support_mode={args.support_mode} "
        f"duration={args.duration}s viewer={args.viewer}"
    )

    if args.viewer:
        run_viewer(model, data, args)
    else:
        run_headless(model, data, args)

    print("Movement test finished.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
