#!/usr/bin/env python3
"""Run the fixed, simulation-only I002 standing evaluation for one checkpoint."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parent
ROOT = EXPERIMENT_DIR.parents[3]
RL_LAB_ROOT = ROOT / "third_party" / "unitree_rl_lab"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _configure_environment(env_cfg: Any, duration_s: float) -> None:
    """Repeat I002's fixed nominal conditions for an independent replay."""
    velocity = env_cfg.commands.base_velocity
    velocity.ranges.lin_vel_x = (0.0, 0.0)
    velocity.ranges.lin_vel_y = (0.0, 0.0)
    velocity.ranges.ang_vel_z = (0.0, 0.0)
    velocity.limit_ranges.lin_vel_x = (0.0, 0.0)
    velocity.limit_ranges.lin_vel_y = (0.0, 0.0)
    velocity.limit_ranges.ang_vel_z = (0.0, 0.0)
    velocity.rel_standing_envs = 1.0
    velocity.resampling_time_range = (duration_s + 1.0, duration_s + 1.0)
    velocity.debug_vis = False
    env_cfg.scene.terrain.terrain_type = "plane"
    env_cfg.scene.terrain.terrain_generator = None
    env_cfg.scene.terrain.max_init_terrain_level = None
    env_cfg.curriculum.terrain_levels = None
    env_cfg.curriculum.lin_vel_cmd_levels = None
    env_cfg.events.push_robot = None
    env_cfg.events.physics_material = None
    env_cfg.events.add_base_mass = None
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    env_cfg.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)
    env_cfg.events.reset_base.params["pose_range"] = {
        "x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)
    }
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.viewer.origin_type = "env"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = (2.6, -2.6, 1.5)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.72)
    env_cfg.viewer.resolution = (1280, 720)


def _trace_row(env: Any, action: Any, reward: Any, done: Any, step: int, time_s: float) -> dict[str, float | int]:
    robot = env.unwrapped.scene["robot"]
    pos = robot.data.root_pos_w[0].detach().cpu().numpy()
    vel = robot.data.root_lin_vel_b[0].detach().cpu().numpy()
    angular = robot.data.root_ang_vel_b[0].detach().cpu().numpy()
    gravity = robot.data.projected_gravity_b[0].detach().cpu().numpy()
    action_np = action[0].detach().cpu().numpy()
    tilt_rad = float(np.arccos(np.clip(-gravity[2], -1.0, 1.0)))
    row: dict[str, float | int] = {
        "step": step,
        "time_s": time_s,
        "base_x_m": float(pos[0]),
        "base_y_m": float(pos[1]),
        "base_z_m": float(pos[2]),
        "base_vx_body_mps": float(vel[0]),
        "base_vy_body_mps": float(vel[1]),
        "base_vz_body_mps": float(vel[2]),
        "base_wx_body_radps": float(angular[0]),
        "base_wy_body_radps": float(angular[1]),
        "base_wz_body_radps": float(angular[2]),
        "tilt_rad": tilt_rad,
        "reward": float(reward[0].detach().cpu().item()),
        "done": int(done[0].detach().cpu().item()),
        "action_l2": float(np.linalg.norm(action_np)),
    }
    row.update({f"action_{index}": float(value) for index, value in enumerate(action_np)})
    return row


def _write_trace(output_dir: Path, rows: list[dict[str, float | int]]) -> None:
    with (output_dir / "trace.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(output_dir / "trace.npz", **{field: np.array([row[field] for row in rows]) for field in rows[0]})


def _write_plots(output_dir: Path, rows: list[dict[str, float | int]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time_s = np.asarray([row["time_s"] for row in rows])
    displacement = np.hypot(
        np.asarray([row["base_x_m"] for row in rows]) - rows[0]["base_x_m"],
        np.asarray([row["base_y_m"] for row in rows]) - rows[0]["base_y_m"],
    )
    figure, axis = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axis[0].plot(time_s, [row["tilt_rad"] for row in rows])
    axis[0].set_ylabel("tilt [rad]")
    axis[1].plot(time_s, [row["base_z_m"] for row in rows])
    axis[1].set_xlabel("time [s]")
    axis[1].set_ylabel("base height [m]")
    figure.tight_layout()
    figure.savefig(output_dir / "orientation_and_height.png", dpi=160)
    plt.close(figure)
    figure, axis = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axis[0].plot(time_s, [row["base_vx_body_mps"] for row in rows], label="vx")
    axis[0].plot(time_s, [row["base_vy_body_mps"] for row in rows], label="vy")
    axis[0].set_ylabel("body velocity [m/s]")
    axis[0].legend(loc="best")
    axis[1].plot(time_s, displacement)
    axis[1].set_xlabel("time [s]")
    axis[1].set_ylabel("xy displacement [m]")
    figure.tight_layout()
    figure.savefig(output_dir / "velocity_and_displacement.png", dpi=160)
    plt.close(figure)


def _evaluate_metrics(rows: list[dict[str, float | int]], protocol: dict[str, Any]) -> dict[str, Any]:
    settling_window_s = float(protocol["settling_window_s"])
    criteria = protocol["criteria"]
    post_settle = [row for row in rows if float(row["time_s"]) >= settling_window_s] or rows
    xy_speed = np.asarray([
        np.hypot(float(row["base_vx_body_mps"]), float(row["base_vy_body_mps"])) for row in post_settle
    ])
    displacement = np.asarray([
        np.hypot(float(row["base_x_m"]) - float(rows[0]["base_x_m"]), float(row["base_y_m"]) - float(rows[0]["base_y_m"]))
        for row in post_settle
    ])
    values = {
        "terminated": any(bool(row["done"]) for row in rows),
        "max_tilt_rad": float(max(float(row["tilt_rad"]) for row in rows)),
        "minimum_base_height_m": float(min(float(row["base_z_m"]) for row in rows)),
        "xy_speed_rms_mps_after_settling": float(np.sqrt(np.mean(np.square(xy_speed)))),
        "max_xy_displacement_m_after_settling": float(max(displacement)),
        "duration_s_observed": float(rows[-1]["time_s"]),
    }
    passed = (
        (not values["terminated"] if criteria["no_termination"] else True)
        and values["max_tilt_rad"] <= float(criteria["max_tilt_rad"])
        and values["minimum_base_height_m"] >= float(criteria["minimum_base_height_m"])
        and values["xy_speed_rms_mps_after_settling"] <= float(criteria["maximum_xy_speed_rms_mps"])
        and values["max_xy_displacement_m_after_settling"] <= float(criteria["maximum_xy_displacement_m"])
    )
    return values | {"scientific_status": "passed" if passed else "failed"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, help="Completed I002 model_*.pt checkpoint.")
    parser.add_argument("--output-dir", type=Path, help="New, empty evaluation evidence directory.")
    parser.add_argument("--protocol", type=Path, default=EXPERIMENT_DIR / "evaluation.json")
    parser.add_argument("--task", default="Unitree-R1-Stand")
    parser.add_argument("--disable_fabric", action="store_true", default=False)
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.enable_cameras = True
    if args.checkpoint is None or args.output_dir is None:
        parser.error("--checkpoint and --output-dir are required")
    checkpoint = args.checkpoint.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    protocol_path = args.protocol.expanduser().resolve()
    if not checkpoint.is_file():
        raise SystemExit(f"checkpoint does not exist: {checkpoint}")
    if output_dir.exists():
        raise SystemExit(f"refusing to overwrite evaluation evidence: {output_dir}")
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("task") != args.task:
        raise SystemExit(f"protocol task {protocol.get('task')!r} does not match {args.task!r}")

    output_dir.mkdir(parents=True)
    shutil.copy2(protocol_path, output_dir / "protocol_snapshot.json")
    started_at = datetime.now(timezone.utc).isoformat()
    _write_json(output_dir / "status.json", {"execution_status": "running", "started_at": started_at})
    _write_json(output_dir / "manifest.json", {
        "schema_version": 1,
        "protocol_id": protocol["protocol_id"],
        "task": args.task,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": _sha256(checkpoint),
        "r1_usd": "assets/R1/R1.usd",
        "r1_usd_sha256": _sha256(ROOT / "assets" / "R1" / "R1.usd"),
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        "started_at": started_at,
        "environment_contract": "nominal plane; zero command; zero reset velocity; no startup randomization; no policy observation corruption",
    })

    simulation_app = AppLauncher(args).app
    env = None
    try:
        sys.path.insert(0, str(ROOT))
        sys.path.insert(0, str(RL_LAB_ROOT / "source" / "unitree_rl_lab"))
        sys.path.insert(0, str(RL_LAB_ROOT / "scripts" / "rsl_rl"))
        import gymnasium as gym
        import torch
        from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from rsl_rl.runners import OnPolicyRunner
        from unitree_rl_lab.utils.parser_cfg import parse_env_cfg
        import cli_args
        import training.isaaclab  # noqa: F401

        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1, use_fabric=not args.disable_fabric, entry_point_key="play_env_cfg_entry_point")
        _configure_environment(env_cfg, float(protocol["duration_s"]))
        raw_env = gym.make(args.task, cfg=env_cfg, render_mode="rgb_array")
        if isinstance(raw_env.unwrapped, DirectMARLEnv):
            raw_env = multi_agent_to_single_agent(raw_env)
        raw_env = gym.wrappers.RecordVideo(
            raw_env,
            video_folder=str(output_dir / "video"),
            step_trigger=lambda step: step == 0,
            video_length=1_000_000,
            disable_logger=True,
        )
        env = RslRlVecEnvWrapper(raw_env, clip_actions=None)
        env.seed(int(protocol["seed"]))
        agent_args = SimpleNamespace(seed=None, resume=False, load_run=None, checkpoint=str(checkpoint), run_name=None, logger=None, log_project_name=None, task=args.task)
        agent_cfg = cli_args.parse_rsl_rl_cfg(args.task, agent_args)
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(str(checkpoint))
        policy = runner.get_inference_policy(device=env.unwrapped.device)
        obs = env.get_observations()
        observation_dim = int(obs["policy"].shape[-1])
        dt = float(env.unwrapped.step_dt)
        rows: list[dict[str, float | int]] = []
        action_dim: int | None = None
        for step in range(int(np.ceil(float(protocol["duration_s"]) / dt))):
            with torch.inference_mode():
                action = policy(obs)
                action_dim = int(action.shape[-1])
                row = _trace_row(env, action, torch.zeros(1, device=action.device), torch.zeros(1, device=action.device, dtype=torch.long), step, step * dt)
                obs, reward, done, _ = env.step(action)
            row["reward"] = float(reward[0].detach().cpu().item())
            row["done"] = int(done[0].detach().cpu().item())
            rows.append(row)
            if row["done"]:
                break
        _write_trace(output_dir, rows)
        _write_plots(output_dir, rows)
        metrics = _evaluate_metrics(rows, protocol)
        with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(metrics))
            writer.writeheader()
            writer.writerow(metrics)
        _write_json(output_dir / "status.json", {
            "execution_status": "completed",
            "scientific_outcome": metrics["scientific_status"],
            "completed_at": datetime.now(timezone.utc).isoformat(),
            "metrics": metrics,
        })
        _write_json(output_dir / "evaluation_manifest.json", {
            "schema_version": 1,
            "framework": "rl_lab",
            "task": args.task,
            "status": metrics["scientific_status"],
            "source_checkpoint": str(checkpoint),
            "source_checkpoint_sha256": _sha256(checkpoint),
            "r1_usd": "assets/R1/R1.usd",
            "r1_usd_sha256": _sha256(ROOT / "assets" / "R1" / "R1.usd"),
            "observation_action_signature": {
                "observation_group": "policy",
                "observation_dim": observation_dim,
                "action_dim": action_dim,
                "action_type": "normalized joint position offsets",
            },
            "protocol_snapshot": "protocol_snapshot.json",
            "metrics": metrics,
            "completed_at": datetime.now(timezone.utc).isoformat(),
        })
    except BaseException as error:
        _write_json(output_dir / "status.json", {
            "execution_status": "failed", "scientific_outcome": "unassessed", "reason": repr(error),
            "traceback": traceback.format_exc(), "updated_at": datetime.now(timezone.utc).isoformat(),
        })
        raise
    finally:
        if env is not None:
            env.close()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
