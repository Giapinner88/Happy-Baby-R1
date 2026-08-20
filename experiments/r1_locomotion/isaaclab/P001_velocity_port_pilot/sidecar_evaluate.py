#!/usr/bin/env python3
"""Evaluate one archived P001 checkpoint under fixed, single-environment commands."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import sys
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


def _write_json(path: Path, content: dict[str, Any]) -> None:
    path.write_text(json.dumps(content, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _configure_fixed_command(env_cfg: Any, command: dict[str, float], duration_s: float, no_corruption: bool) -> None:
    velocity = env_cfg.commands.base_velocity
    velocity.ranges.lin_vel_x = (command["vx_mps"], command["vx_mps"])
    velocity.ranges.lin_vel_y = (command["vy_mps"], command["vy_mps"])
    velocity.ranges.ang_vel_z = (command["yaw_rate_radps"], command["yaw_rate_radps"])
    velocity.limit_ranges.lin_vel_x = velocity.ranges.lin_vel_x
    velocity.limit_ranges.lin_vel_y = velocity.ranges.lin_vel_y
    velocity.limit_ranges.ang_vel_z = velocity.ranges.ang_vel_z
    velocity.rel_standing_envs = 0.0
    velocity.resampling_time_range = (duration_s + 1.0, duration_s + 1.0)
    velocity.debug_vis = True
    if no_corruption and hasattr(env_cfg.observations.policy, "enable_corruption"):
        env_cfg.observations.policy.enable_corruption = False


def _configure_p001_environment(env_cfg: Any) -> None:
    """Apply the archived fixed flat/no-push environment contract of P001."""
    env_cfg.scene.terrain.terrain_type = "plane"
    env_cfg.scene.terrain.terrain_generator = None
    env_cfg.scene.terrain.max_init_terrain_level = None
    env_cfg.curriculum.terrain_levels = None
    env_cfg.curriculum.lin_vel_cmd_levels = None
    env_cfg.events.push_robot = None
    # The recorded viewport is relative to the sole environment rather than
    # to the world origin of a tiled terrain.
    env_cfg.viewer.origin_type = "env"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = (3.0, -3.0, 1.8)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.70)
    env_cfg.viewer.resolution = (1280, 720)


def _trace_row(env: Any, action: Any, reward: Any, done: Any, step: int, time_s: float) -> dict[str, float | int]:
    robot = env.unwrapped.scene["robot"]
    command = env.unwrapped.command_manager.get_command("base_velocity")[0].detach().cpu().numpy()
    pos = robot.data.root_pos_w[0].detach().cpu().numpy()
    quat = robot.data.root_quat_w[0].detach().cpu().numpy()
    lin_vel = robot.data.root_lin_vel_b[0].detach().cpu().numpy()
    ang_vel = robot.data.root_ang_vel_b[0].detach().cpu().numpy()
    gravity = robot.data.projected_gravity_b[0].detach().cpu().numpy()
    action_np = action[0].detach().cpu().numpy()
    row: dict[str, float | int] = {
        "step": step,
        "time_s": time_s,
        "command_vx_mps": float(command[0]),
        "command_vy_mps": float(command[1]),
        "command_yaw_rate_radps": float(command[2]),
        "base_x_m": float(pos[0]),
        "base_y_m": float(pos[1]),
        "base_z_m": float(pos[2]),
        "base_quat_w": float(quat[0]),
        "base_quat_x": float(quat[1]),
        "base_quat_y": float(quat[2]),
        "base_quat_z": float(quat[3]),
        "base_vx_body_mps": float(lin_vel[0]),
        "base_vy_body_mps": float(lin_vel[1]),
        "base_vz_body_mps": float(lin_vel[2]),
        "base_wx_body_radps": float(ang_vel[0]),
        "base_wy_body_radps": float(ang_vel[1]),
        "base_wz_body_radps": float(ang_vel[2]),
        "projected_gravity_x": float(gravity[0]),
        "projected_gravity_y": float(gravity[1]),
        "projected_gravity_z": float(gravity[2]),
        "reward": float(reward[0].detach().cpu().item()),
        "done": int(done[0].detach().cpu().item()),
        "action_l2": float(np.linalg.norm(action_np)),
    }
    row.update({f"action_{index}": float(value) for index, value in enumerate(action_np)})
    return row


def _write_trace(case_dir: Path, rows: list[dict[str, float | int]]) -> None:
    fields = list(rows[0])
    with (case_dir / "trace.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(case_dir / "trace.npz", **{field: np.array([row[field] for row in rows]) for field in fields})


def _plots(case_dir: Path, rows: list[dict[str, float | int]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time_s = np.asarray([row["time_s"] for row in rows])
    figure, axis = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axis[0].plot(time_s, [row["command_vx_mps"] for row in rows], label="command vx")
    axis[0].plot(time_s, [row["base_vx_body_mps"] for row in rows], label="measured vx")
    axis[0].plot(time_s, [row["command_vy_mps"] for row in rows], label="command vy")
    axis[0].plot(time_s, [row["base_vy_body_mps"] for row in rows], label="measured vy")
    axis[0].set_ylabel("velocity [m/s]")
    axis[0].legend(loc="best")
    axis[1].plot(time_s, [row["command_yaw_rate_radps"] for row in rows], label="command yaw rate")
    axis[1].plot(time_s, [row["base_wz_body_radps"] for row in rows], label="measured yaw rate")
    axis[1].set_xlabel("time [s]")
    axis[1].set_ylabel("yaw rate [rad/s]")
    axis[1].legend(loc="best")
    figure.tight_layout()
    figure.savefig(case_dir / "tracking.png", dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axis[0].plot(time_s, [row["base_z_m"] for row in rows], label="base height")
    axis[0].set_ylabel("height [m]")
    axis[0].legend(loc="best")
    axis[1].plot(time_s, [row["projected_gravity_x"] for row in rows], label="projected gravity x")
    axis[1].plot(time_s, [row["projected_gravity_y"] for row in rows], label="projected gravity y")
    axis[1].plot(time_s, [row["action_l2"] for row in rows], label="action L2")
    axis[1].set_xlabel("time [s]")
    axis[1].legend(loc="best")
    figure.tight_layout()
    figure.savefig(case_dir / "orientation.png", dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    # AppLauncher internally calls ``parse_known_args`` while extending this
    # parser; required arguments would make ``--help`` fail at that stage.
    parser.add_argument("--checkpoint", type=Path, help="Stable model_*.pt checkpoint to read.")
    parser.add_argument("--output-dir", type=Path, help="New, empty diagnostic output directory.")
    parser.add_argument("--protocol", type=Path, default=EXPERIMENT_DIR / "sidecar_evaluation.json")
    parser.add_argument("--task", default="Unitree-R1-Velocity")
    parser.add_argument("--case", help="Evaluate exactly one protocol case; used by the isolating shell wrapper.")
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
        raise SystemExit(f"output directory already exists: {output_dir}")
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("task") != args.task:
        raise SystemExit(f"protocol task {protocol.get('task')!r} does not match {args.task!r}")
    cases_by_id = {str(case["case_id"]): case for case in protocol["cases"]}
    if args.case is not None and args.case not in cases_by_id:
        raise SystemExit(f"unknown case {args.case!r}; choices: {', '.join(cases_by_id)}")
    selected_cases = [cases_by_id[args.case]] if args.case else list(cases_by_id.values())

    output_dir.mkdir(parents=True)
    shutil.copy2(protocol_path, output_dir / "protocol_snapshot.json")
    started_at = datetime.now(timezone.utc).isoformat()
    _write_json(output_dir / "status.json", {"execution_status": "running", "started_at": started_at})
    _write_json(output_dir / "manifest.json", {
        "schema_version": 1,
        "protocol_id": protocol["protocol_id"],
        "started_at": started_at,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": _sha256(checkpoint),
        "task": args.task,
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        "policy_observation_corruption": protocol["execution"]["policy_observation_corruption"],
        "selected_cases": [case["case_id"] for case in selected_cases],
        "environment_contract": "plane; terrain curriculum disabled; velocity curriculum disabled; pushes disabled",
        "source_mutation": "none",
    })

    # Isaac Sim must launch before importing Torch, Gym, or task modules.
    simulation_app = AppLauncher(args).app

    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(RL_LAB_ROOT / "source" / "unitree_rl_lab"))
    sys.path.insert(0, str(RL_LAB_ROOT / "scripts" / "rsl_rl"))
    import gymnasium as gym
    import torch
    from rsl_rl.runners import OnPolicyRunner
    from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from unitree_rl_lab.utils.parser_cfg import parse_env_cfg
    import training.isaaclab  # noqa: F401
    import cli_args

    summaries: list[dict[str, Any]] = []
    try:
        for case in selected_cases:
            case_id = str(case["case_id"])
            command = dict(case["command"])
            case_dir = output_dir / "case" / case_id
            (case_dir / "video").mkdir(parents=True)
            env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1, use_fabric=not args.disable_fabric, entry_point_key="play_env_cfg_entry_point")
            _configure_p001_environment(env_cfg)
            _configure_fixed_command(env_cfg, command, float(protocol["duration_s"]), bool(protocol["execution"]["policy_observation_corruption"]))
            raw_env = gym.make(args.task, cfg=env_cfg, render_mode="rgb_array")
            if isinstance(raw_env.unwrapped, DirectMARLEnv):
                raw_env = multi_agent_to_single_agent(raw_env)
            raw_env = gym.wrappers.RecordVideo(raw_env, video_folder=str(case_dir / "video"), step_trigger=lambda step: step == 0, video_length=1_000_000, disable_logger=True)
            env = RslRlVecEnvWrapper(raw_env, clip_actions=None)
            env.seed(int(protocol["execution"]["seed"]))
            # ``cli_args`` expects the complete upstream play parser.  The
            # sidecar intentionally exposes only its relevant options, so
            # provide its small update contract explicitly.
            agent_args = SimpleNamespace(
                seed=None,
                resume=False,
                load_run=None,
                checkpoint=str(checkpoint),
                run_name=None,
                logger=None,
                log_project_name=None,
                task=args.task,
            )
            agent_cfg = cli_args.parse_rsl_rl_cfg(args.task, agent_args)
            runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            runner.load(str(checkpoint))
            policy = runner.get_inference_policy(device=env.unwrapped.device)
            obs = env.get_observations()
            dt = float(env.unwrapped.step_dt)
            steps = int(np.ceil(float(protocol["duration_s"]) / dt))
            rows: list[dict[str, float | int]] = []
            first_done_step: int | None = None
            termination_terms = ""
            for step in range(steps):
                with torch.inference_mode():
                    action = policy(obs)
                    # Record the state before stepping.  ManagerBasedRLEnv
                    # resets a terminated environment before returning from
                    # ``step``, so sampling afterwards would silently insert
                    # a post-reset state into a terminal trace.
                    row = _trace_row(
                        env,
                        action,
                        torch.zeros(1, device=action.device),
                        torch.zeros(1, device=action.device, dtype=torch.long),
                        step,
                        step * dt,
                    )
                    obs, reward, done, _extras = env.step(action)
                row["reward"] = float(reward[0].detach().cpu().item())
                row["done"] = int(done[0].detach().cpu().item())
                row["transition_end_s"] = (step + 1) * dt
                rows.append(row)
                if row["done"]:
                    first_done_step = step + 1
                    termination_terms = ",".join(
                        name for name, value in env.unwrapped.termination_manager.get_active_iterable_terms(0)
                        if value[0] > 0.5
                    )
                    break
            env.close()
            _write_trace(case_dir, rows)
            _plots(case_dir, rows)
            velocity_error = np.asarray([row["base_vx_body_mps"] - row["command_vx_mps"] for row in rows])
            summaries.append({
                "case_id": case_id,
                "steps": len(rows),
                "duration_s": rows[-1]["time_s"],
                "terminated": first_done_step is not None,
                "first_done_step": first_done_step or "",
                "termination_terms": termination_terms,
                "mean_abs_vx_error_mps": float(np.mean(np.abs(velocity_error))),
                "mean_base_height_m": float(np.mean([row["base_z_m"] for row in rows])),
            })
        with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        _write_json(output_dir / "status.json", {"execution_status": "completed", "completed_at": datetime.now(timezone.utc).isoformat()})
    except BaseException as error:
        _write_json(output_dir / "status.json", {"execution_status": "failed", "reason": repr(error), "updated_at": datetime.now(timezone.utc).isoformat()})
        raise
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
