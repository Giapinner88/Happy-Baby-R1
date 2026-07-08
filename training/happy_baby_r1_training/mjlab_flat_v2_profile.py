"""Local MJLab task profiles for R1 velocity training.

This module registers workspace-only task variants without changing the
vendored Unitree RL MJLab sources.
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any


FLAT_V2_TASK_ID = "Unitree-R1-FlatV2"


def _apply_flat_v2_env(cfg: Any) -> Any:
    """Bias flat training toward faster, stable forward walking."""
    twist_cmd = cfg.commands["twist"]
    twist_cmd.rel_standing_envs = 0.02
    twist_cmd.ranges.lin_vel_x = (0.0, 1.0)
    twist_cmd.ranges.lin_vel_y = (-0.35, 0.35)
    twist_cmd.ranges.ang_vel_z = (-0.8, 0.8)

    command_curriculum = cfg.curriculum.get("command_vel")
    if command_curriculum is not None:
        command_curriculum.params["velocity_stages"] = [
            {
                "step": 0,
                "lin_vel_x": (0.0, 0.45),
                "lin_vel_y": (-0.12, 0.12),
                "ang_vel_z": (-0.35, 0.35),
            },
            {
                "step": 80_000,
                "lin_vel_x": (0.05, 0.75),
                "lin_vel_y": (-0.20, 0.20),
                "ang_vel_z": (-0.50, 0.50),
            },
            {
                "step": 180_000,
                "lin_vel_x": (-0.10, 1.00),
                "lin_vel_y": (-0.35, 0.35),
                "ang_vel_z": (-0.80, 0.80),
            },
        ]

    cfg.rewards["track_linear_velocity"].weight = 1.8
    cfg.rewards["track_linear_velocity"].params["std"] = 0.35
    cfg.rewards["track_angular_velocity"].weight = 0.5
    cfg.rewards["track_angular_velocity"].params["std"] = 0.6

    cfg.rewards["pose"].weight = 0.55
    cfg.rewards["pose"].params["walking_threshold"] = 0.05
    cfg.rewards["pose"].params["running_threshold"] = 1.0
    cfg.rewards["pose"].params["std_walking"].update(
        {
            r".*hip_pitch.*": 0.65,
            r".*hip_roll.*": 0.22,
            r".*hip_yaw.*": 0.22,
            r".*knee.*": 0.65,
            r".*ankle_pitch.*": 0.22,
            r".*ankle_roll.*": 0.12,
            r".*waist_yaw.*": 0.22,
            r".*shoulder_pitch.*": 0.22,
        }
    )
    cfg.rewards["pose"].params["std_running"].update(
        {
            r".*hip_pitch.*": 0.75,
            r".*hip_roll.*": 0.30,
            r".*hip_yaw.*": 0.30,
            r".*knee.*": 0.75,
            r".*ankle_pitch.*": 0.30,
            r".*ankle_roll.*": 0.14,
            r".*waist_yaw.*": 0.30,
            r".*shoulder_pitch.*": 0.30,
        }
    )

    cfg.rewards["body_orientation_l2"].weight = -1.25
    cfg.rewards["body_ang_vel"].weight = -0.10
    cfg.rewards["angular_momentum"].weight = -0.045
    cfg.rewards["is_terminated"].weight = -250.0
    cfg.rewards["joint_acc_l2"].weight = -7.5e-7
    cfg.rewards["action_rate_l2"].weight = -0.09
    cfg.rewards["foot_gait"].weight = 0.65
    cfg.rewards["foot_gait"].params["command_threshold"] = 0.05
    cfg.rewards["foot_clearance"].weight = -0.45
    cfg.rewards["foot_clearance"].params["target_height"] = 0.08
    cfg.rewards["foot_clearance"].params["command_threshold"] = 0.05
    cfg.rewards["foot_slip"].weight = -0.45
    cfg.rewards["foot_slip"].params["command_threshold"] = 0.05
    cfg.rewards["stand_still"].weight = -0.25

    return cfg


def _apply_flat_v2_agent(cfg: Any) -> Any:
    if not getattr(cfg, "run_name", ""):
        cfg.run_name = "r1_flat_walk_v2"
    cfg.algorithm.entropy_coef = 0.005
    return cfg


def register_flat_v2_task(
    register_fn: Any,
    *,
    env_cfg: Any,
    play_env_cfg: Any,
    rl_cfg: Any,
    runner_cls: type | None,
) -> None:
    """Register the local flat v2 task beside the upstream flat task."""
    register_fn(
        task_id=FLAT_V2_TASK_ID,
        env_cfg=_apply_flat_v2_env(deepcopy(env_cfg)),
        play_env_cfg=_apply_flat_v2_env(deepcopy(play_env_cfg)),
        rl_cfg=_apply_flat_v2_agent(deepcopy(rl_cfg)),
        runner_cls=runner_cls,
    )
