#!/usr/bin/env python3
"""Validate the editable I002 config and print its exact launcher overrides."""

from __future__ import annotations

import json
from pathlib import Path


EXPERIMENT_DIR = Path(__file__).resolve().parent


def _range(value: object, name: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must be a two-value JSON list.")
    lower, upper = float(value[0]), float(value[1])
    if lower > upper:
        raise ValueError(f"{name} lower bound must not exceed upper bound.")
    return lower, upper


def load_config() -> dict[str, object]:
    value = json.loads((EXPERIMENT_DIR / "config.json").read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("config.json must contain a JSON object.")
    return value


def hydra_overrides(config: dict[str, object]) -> list[str]:
    if config.get("framework") != "rl_lab" or config.get("task") != "Unitree-R1-Stand":
        raise ValueError("I002 is defined only for rl_lab / Unitree-R1-Stand.")
    training = config.get("training")
    execution = config.get("execution")
    commands = config.get("command_distribution")
    nominal = config.get("nominal_conditions")
    capture = config.get("capture")
    artifacts = config.get("artifacts")
    if not all(isinstance(value, dict) for value in (training, execution, commands, nominal, capture, artifacts)):
        raise ValueError("training, execution, command_distribution, nominal_conditions, capture, and artifacts must be objects.")
    assert isinstance(training, dict) and isinstance(execution, dict)
    assert isinstance(commands, dict) and isinstance(nominal, dict)
    assert isinstance(capture, dict) and isinstance(artifacts, dict)

    num_envs = int(training["num_envs"])
    max_iterations = int(training["max_iterations"])
    save_interval = int(training["save_interval"])
    seed = int(training["seed"])
    if min(num_envs, max_iterations, save_interval) <= 0:
        raise ValueError("training.num_envs, max_iterations, and save_interval must be positive.")
    if save_interval > max_iterations:
        raise ValueError("training.save_interval must not exceed training.max_iterations.")
    if not isinstance(execution.get("cuda_visible_devices"), str) or not str(execution["cuda_visible_devices"]).isdecimal():
        raise ValueError("execution.cuda_visible_devices must be one non-negative physical GPU index as a string.")
    for name in ("lin_vel_x_mps", "lin_vel_y_mps", "yaw_rate_radps"):
        if _range(commands.get(name), name) != (0.0, 0.0):
            raise ValueError(f"I002 is standing-only; {name} must be [0.0, 0.0].")
    if float(commands.get("standing_probability", -1.0)) != 1.0:
        raise ValueError("I002 requires standing_probability=1.0.")
    required_nominal = (
        "disable_terrain_curriculum",
        "disable_velocity_curriculum",
        "disable_interval_pushes",
        "disable_startup_material_randomization",
        "disable_startup_base_mass_randomization",
        "disable_policy_observation_corruption",
    )
    if nominal.get("terrain") != "plane" or not all(nominal.get(key) is True for key in required_nominal):
        raise ValueError("I002 requires the declared clean nominal plane conditions.")
    if _range(nominal.get("reset_joint_velocity_radps"), "reset_joint_velocity_radps") != (0.0, 0.0):
        raise ValueError("I002 requires zero reset joint velocity.")
    if _range(nominal.get("reset_base_xy_yaw"), "reset_base_xy_yaw") != (0.0, 0.0):
        raise ValueError("I002 requires a fixed zero x/y/yaw reset.")
    if capture.get("record_training_video") is not True or capture.get("convert_tensorboard_to_csv_and_plots") is not True:
        raise ValueError("I002 requires training video plus TensorBoard CSV/plot conversion.")
    if capture.get("raw_evaluation_trace_required") is not True or capture.get("evaluation_video_required") is not True:
        raise ValueError("I002 requires raw evaluation traces and evaluation video.")
    if artifacts.get("export_policy_after_training") is not True:
        raise ValueError("I002 requires ONNX/JIT policy export after completed training.")
    video_length = int(capture["training_video_length_steps"])
    video_interval = int(capture["training_video_interval_steps"])
    if min(video_length, video_interval) <= 0:
        raise ValueError("training video length and interval must be positive.")

    return [
        "env.scene.terrain.terrain_type=plane",
        "env.scene.terrain.terrain_generator=null",
        "env.scene.terrain.max_init_terrain_level=null",
        "env.curriculum.terrain_levels=null",
        "env.curriculum.lin_vel_cmd_levels=null",
        "env.events.push_robot=null",
        "env.events.physics_material=null",
        "env.events.add_base_mass=null",
        "env.events.reset_robot_joints.params.position_range=[1.0,1.0]",
        "env.events.reset_robot_joints.params.velocity_range=[0.0,0.0]",
        "env.events.reset_base.params.pose_range.x=[0.0,0.0]",
        "env.events.reset_base.params.pose_range.y=[0.0,0.0]",
        "env.events.reset_base.params.pose_range.yaw=[0.0,0.0]",
        "env.observations.policy.enable_corruption=False",
        "env.commands.base_velocity.rel_standing_envs=1.0",
        "env.commands.base_velocity.ranges.lin_vel_x=[0.0,0.0]",
        "env.commands.base_velocity.ranges.lin_vel_y=[0.0,0.0]",
        "env.commands.base_velocity.ranges.ang_vel_z=[0.0,0.0]",
        "env.commands.base_velocity.limit_ranges.lin_vel_x=[0.0,0.0]",
        "env.commands.base_velocity.limit_ranges.lin_vel_y=[0.0,0.0]",
        "env.commands.base_velocity.limit_ranges.ang_vel_z=[0.0,0.0]",
        "env.rewards.gait=null",
        "env.rewards.feet_clearance=null",
        f"agent.save_interval={save_interval}",
        f"agent.run_name={str(training['run_name'])}",
        "--video",
        f"--video-length={video_length}",
        f"--video-interval={video_interval}",
        f"--num-envs={num_envs}",
        f"--max-iterations={max_iterations}",
        f"--seed={seed}",
    ]


def main() -> int:
    config = load_config()
    print("I002 config is valid. Hydra and CLI overrides:")
    print("\n".join(hydra_overrides(config)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
