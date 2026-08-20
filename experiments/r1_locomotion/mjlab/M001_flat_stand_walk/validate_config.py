#!/usr/bin/env python3
"""Validate the editable M001 config and print its exact MJLab CLI overrides."""

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


def _tuple(values: tuple[float, float]) -> str:
    return f"({values[0]},{values[1]})"


def load_config() -> dict[str, object]:
    value = json.loads((EXPERIMENT_DIR / "config.json").read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("config.json must contain a JSON object.")
    return value


def mjlab_overrides(config: dict[str, object]) -> list[str]:
    if config.get("framework") != "mjlab" or config.get("task") != "Unitree-R1-Flat":
        raise ValueError("M001 is defined only for mjlab / Unitree-R1-Flat.")
    training = config.get("training")
    distribution = config.get("command_distribution")
    protocol = config.get("environment_protocol")
    capture = config.get("capture")
    execution = config.get("execution")
    if not all(isinstance(value, dict) for value in (training, distribution, protocol, capture, execution)):
        raise ValueError("training, execution, command_distribution, environment_protocol, and capture must be objects.")

    assert isinstance(training, dict)
    assert isinstance(distribution, dict)
    assert isinstance(protocol, dict)
    assert isinstance(capture, dict)
    assert isinstance(execution, dict)
    num_envs = int(training["num_envs"])
    max_iterations = int(training["max_iterations"])
    save_interval = int(training["save_interval"])
    seed = int(training["seed"])
    standing_probability = float(distribution["standing_probability"])
    lin_vel_x = _range(distribution["lin_vel_x_mps"], "lin_vel_x_mps")
    lin_vel_y = _range(distribution["lin_vel_y_mps"], "lin_vel_y_mps")
    yaw_rate = _range(distribution["yaw_rate_radps"], "yaw_rate_radps")
    minimum_norm = float(distribution["minimum_nonzero_command_norm_mps"])
    if num_envs <= 0 or max_iterations <= 0 or save_interval <= 0:
        raise ValueError("num_envs, max_iterations, and save_interval must be positive.")
    visible_devices = execution.get("cuda_visible_devices")
    if not isinstance(visible_devices, str) or not visible_devices.isdecimal():
        raise ValueError("execution.cuda_visible_devices must be one non-negative physical GPU index as a string.")
    if not 0.0 <= standing_probability <= 1.0:
        raise ValueError("standing_probability must be between 0 and 1.")
    if minimum_norm <= 0.0:
        raise ValueError("minimum_nonzero_command_norm_mps must be positive.")
    if max(abs(lin_vel_x[0]), abs(lin_vel_x[1]), abs(lin_vel_y[0]), abs(lin_vel_y[1]), abs(yaw_rate[0]), abs(yaw_rate[1])) <= minimum_norm:
        raise ValueError("The command range cannot produce a nonzero MJLab command above its configured mask threshold.")
    if protocol.get("terrain") != "plane":
        raise ValueError("M001 is a plane-only protocol.")
    if not all(protocol.get(key) is True for key in (
        "disable_terrain_curriculum", "disable_velocity_curriculum", "disable_interval_pushes"
    )):
        raise ValueError("M001 requires all curricula and interval pushes disabled.")
    if capture.get("record_training_video") is not True or capture.get("convert_tensorboard_to_csv_and_plots") is not True:
        raise ValueError("M001 requires training video plus TensorBoard CSV/plot conversion.")
    if capture.get("raw_evaluation_trace_required") is not True or capture.get("evaluation_video_required") is not True:
        raise ValueError("M001 requires raw evaluation traces and evaluation video before a positive result.")
    video_length = int(capture["training_video_length_steps"])
    video_interval = int(capture["training_video_interval_steps"])
    if video_length <= 0 or video_interval <= 0:
        raise ValueError("training video length and interval must be positive.")

    return [
        "--env.curriculum.command-vel", "None",
        "--env.events.push-robot", "None",
        "--env.commands.twist.rel-standing-envs", str(standing_probability),
        "--env.commands.twist.ranges.lin-vel-x", _tuple(lin_vel_x),
        "--env.commands.twist.ranges.lin-vel-y", _tuple(lin_vel_y),
        "--env.commands.twist.ranges.ang-vel-z", _tuple(yaw_rate),
        "--agent.seed", str(seed),
    ]


def main() -> int:
    config = load_config()
    print("M001 config is valid. MJLab CLI overrides:")
    print(" ".join(mjlab_overrides(config)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
