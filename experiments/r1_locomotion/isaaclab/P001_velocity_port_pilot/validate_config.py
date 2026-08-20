#!/usr/bin/env python3
"""Validate the archived P001 config and print its historic Hydra overrides."""

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
    path = EXPERIMENT_DIR / "config.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("config.json must contain a JSON object.")
    return value


def hydra_overrides(config: dict[str, object]) -> list[str]:
    if config.get("framework") != "rl_lab" or config.get("task") != "Unitree-R1-Velocity":
        raise ValueError("P001 is defined only for rl_lab / Unitree-R1-Velocity.")
    training = config.get("training")
    distribution = config.get("command_distribution")
    protocol = config.get("environment_protocol")
    capture = config.get("capture")
    artifacts = config.get("artifacts")
    execution = config.get("execution")
    if not isinstance(training, dict) or not isinstance(execution, dict) or not isinstance(distribution, dict) or not isinstance(protocol, dict) or not isinstance(capture, dict) or not isinstance(artifacts, dict):
        raise ValueError("training, execution, command_distribution, environment_protocol, capture, and artifacts must be objects.")

    num_envs = int(training["num_envs"])
    max_iterations = int(training["max_iterations"])
    save_interval = int(training["save_interval"])
    seed = int(training["seed"])
    run_name = str(training["run_name"])
    standing_probability = float(distribution["standing_probability"])
    lin_vel_x = _range(distribution["lin_vel_x_mps"], "lin_vel_x_mps")
    lin_vel_y = _range(distribution["lin_vel_y_mps"], "lin_vel_y_mps")
    yaw_rate = _range(distribution["yaw_rate_radps"], "yaw_rate_radps")
    if num_envs <= 0 or max_iterations <= 0 or save_interval <= 0:
        raise ValueError("num_envs, max_iterations, and save_interval must be positive.")
    visible_devices = execution.get("cuda_visible_devices")
    if not isinstance(visible_devices, str) or not visible_devices.isdecimal():
        raise ValueError("execution.cuda_visible_devices must be one non-negative physical GPU index as a string.")
    if not 0.0 <= standing_probability <= 1.0:
        raise ValueError("standing_probability must be between 0 and 1.")
    if protocol.get("terrain") != "plane":
        raise ValueError("P001 is a plane-only protocol; terrain must be 'plane'.")
    if not all(protocol.get(key) is True for key in (
        "disable_terrain_curriculum", "disable_velocity_curriculum", "disable_interval_pushes"
    )):
        raise ValueError("P001 requires all curricula and interval pushes disabled.")
    if capture.get("record_training_video") is not True or capture.get("convert_tensorboard_to_csv_and_plots") is not True:
        raise ValueError("P001 requires training video plus TensorBoard CSV/plot conversion.")
    if capture.get("raw_evaluation_trace_required") is not True or capture.get("evaluation_video_required") is not True:
        raise ValueError("P001 requires raw evaluation traces and evaluation video before a positive result.")
    if artifacts.get("export_policy_after_training") is not True:
        raise ValueError("P001 requires ONNX/JIT policy export after a completed training run.")
    video_length = int(capture["training_video_length_steps"])
    video_interval = int(capture["training_video_interval_steps"])
    if video_length <= 0 or video_interval <= 0:
        raise ValueError("training video length and interval must be positive.")

    def hydra_list(values: tuple[float, float]) -> str:
        return f"[{values[0]},{values[1]}]"

    return [
        "env.scene.terrain.terrain_type=plane",
        "env.scene.terrain.terrain_generator=null",
        "env.scene.terrain.max_init_terrain_level=null",
        "env.curriculum.terrain_levels=null",
        "env.curriculum.lin_vel_cmd_levels=null",
        "env.events.push_robot=null",
        f"env.commands.base_velocity.rel_standing_envs={standing_probability}",
        f"env.commands.base_velocity.ranges.lin_vel_x={hydra_list(lin_vel_x)}",
        f"env.commands.base_velocity.ranges.lin_vel_y={hydra_list(lin_vel_y)}",
        f"env.commands.base_velocity.ranges.ang_vel_z={hydra_list(yaw_rate)}",
        f"env.commands.base_velocity.limit_ranges.lin_vel_x={hydra_list(lin_vel_x)}",
        f"env.commands.base_velocity.limit_ranges.lin_vel_y={hydra_list(lin_vel_y)}",
        f"env.commands.base_velocity.limit_ranges.ang_vel_z={hydra_list(yaw_rate)}",
        f"agent.save_interval={save_interval}",
        f"agent.run_name={run_name}",
        "--video",
        f"--video-length={video_length}",
        f"--video-interval={video_interval}",
        f"--num-envs={num_envs}",
        f"--max-iterations={max_iterations}",
        f"--seed={seed}",
    ]


def main() -> int:
    config = load_config()
    overrides = hydra_overrides(config)
    print("P001 config is valid. Hydra and CLI overrides:")
    print("\n".join(overrides))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
