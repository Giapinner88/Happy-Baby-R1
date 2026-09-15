"""Local MJLab task profiles for R1 velocity training.

This module registers workspace-only task variants without changing the
vendored Unitree RL MJLab sources.
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any


FLAT_V2_TASK_ID = "Unitree-R1-FlatV2"
FLAT_V3_TASK_ID = "Unitree-R1-FlatV3"
FLAT_V4_TASK_ID = "Unitree-R1-FlatV4"

# Feet site names used by the vendored R1 velocity cfg.
_FEET_SITES = ("left_foot", "right_foot")


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


def _apply_flat_v3_env(cfg: Any) -> Any:
    """Bias flat training toward smoother body-stable walking."""
    cfg = _apply_flat_v2_env(cfg)

    twist_cmd = cfg.commands["twist"]
    twist_cmd.rel_standing_envs = 0.04
    twist_cmd.ranges.lin_vel_x = (0.0, 0.85)
    twist_cmd.ranges.lin_vel_y = (-0.25, 0.25)
    twist_cmd.ranges.ang_vel_z = (-0.55, 0.55)

    command_curriculum = cfg.curriculum.get("command_vel")
    if command_curriculum is not None:
        command_curriculum.params["velocity_stages"] = [
            {
                "step": 0,
                "lin_vel_x": (0.0, 0.35),
                "lin_vel_y": (-0.08, 0.08),
                "ang_vel_z": (-0.20, 0.20),
            },
            {
                "step": 90_000,
                "lin_vel_x": (0.05, 0.55),
                "lin_vel_y": (-0.14, 0.14),
                "ang_vel_z": (-0.35, 0.35),
            },
            {
                "step": 220_000,
                "lin_vel_x": (0.05, 0.85),
                "lin_vel_y": (-0.25, 0.25),
                "ang_vel_z": (-0.55, 0.55),
            },
        ]

    cfg.rewards["track_linear_velocity"].weight = 1.55
    cfg.rewards["track_linear_velocity"].params["std"] = 0.32
    cfg.rewards["track_angular_velocity"].weight = 0.35
    cfg.rewards["track_angular_velocity"].params["std"] = 0.55

    cfg.rewards["pose"].weight = 0.75
    cfg.rewards["body_orientation_l2"].weight = -1.85
    cfg.rewards["body_ang_vel"].weight = -0.20
    cfg.rewards["angular_momentum"].weight = -0.085
    cfg.rewards["joint_acc_l2"].weight = -1.8e-6
    cfg.rewards["action_rate_l2"].weight = -0.18
    cfg.rewards["foot_gait"].weight = 0.50
    cfg.rewards["foot_clearance"].weight = -0.55
    cfg.rewards["foot_clearance"].params["target_height"] = 0.065
    cfg.rewards["foot_slip"].weight = -0.65
    cfg.rewards["stand_still"].weight = -0.35

    return cfg


def _apply_flat_v3_agent(cfg: Any) -> Any:
    if not getattr(cfg, "run_name", ""):
        cfg.run_name = "r1_flat_walk_v3"
    cfg.algorithm.entropy_coef = 0.003
    return cfg


def _apply_flat_v4_env(cfg: Any) -> Any:
    """Reproduce the tuned "G1-equivalent" R1 flat recipe (~/train_mujoco).

    This is the faithful port of the checkout that produced the stable-walking
    policy. Unlike v2/v3 (which detuned *away* from that recipe), v4 mirrors it:
    higher yaw-tracking, direct yaw commands (rel_heading_envs), longer gait
    period, and the three extra shaping rewards. Kept as a clean overlay — the
    three new reward callables live in :mod:`training.mjlab.rewards`.

    Deviation from the source: the ``payload_mass`` domain-randomization event is
    intentionally dropped because the workspace deploy XML has no ``dev_pc`` body.
    """
    import math

    from mjlab.managers.reward_manager import RewardTermCfg
    from mjlab.managers.scene_entity_config import SceneEntityCfg
    # Use the vendored velocity mdp (src.tasks.velocity.mdp) — NOT the installed
    # mjlab.tasks.velocity.mdp. They ship different feet_air_time signatures
    # (vendored uses `threshold`; installed uses `threshold_min/max`), and the
    # base cfg is built against the vendored one.
    import src.tasks.velocity.mdp as mdp

    from training.mjlab import rewards as ws_rewards

    # ── Observation: gait clock period 0.6 -> 0.8 (must match runtime gait) ──
    for group in ("actor", "critic"):
        obs_group = cfg.observations.get(group)
        if obs_group is not None and "phase" in obs_group.terms:
            obs_group.terms["phase"].params["period"] = 0.8

    # ── Command: learn direct yaw (half the envs) + snappier heading ──
    twist = cfg.commands["twist"]
    twist.rel_standing_envs = 0.05
    twist.rel_heading_envs = 0.5
    twist.heading_control_stiffness = 0.8
    twist.ranges.lin_vel_x = (-1.0, 2.0)
    twist.ranges.lin_vel_y = (-1.0, 1.0)
    twist.ranges.ang_vel_z = (-1.0, 1.0)

    # ── Reward overrides (match tuned base) ──
    cfg.rewards["track_linear_velocity"].weight = 1.5
    cfg.rewards["track_linear_velocity"].params["std"] = math.sqrt(0.25)
    cfg.rewards["track_angular_velocity"].weight = 2.0
    cfg.rewards["track_angular_velocity"].params["std"] = math.sqrt(0.35)
    cfg.rewards["body_orientation_l2"].weight = -1.5
    cfg.rewards["body_ang_vel"].weight = -0.1
    cfg.rewards["angular_momentum"].weight = -0.05
    cfg.rewards["is_terminated"].weight = -200.0
    cfg.rewards["joint_acc_l2"].weight = -2.5e-7
    cfg.rewards["action_rate_l2"].weight = -0.1

    cfg.rewards["pose"].weight = 1.0
    cfg.rewards["pose"].params["walking_threshold"] = 0.1
    cfg.rewards["pose"].params["running_threshold"] = 1.5
    cfg.rewards["pose"].params["std_standing"] = {r".*": 0.05}
    cfg.rewards["pose"].params["std_walking"] = {
        r".*hip_pitch.*": 0.5,
        r".*hip_roll.*": 0.08,
        r".*hip_yaw.*": 0.35,
        r".*knee.*": 0.5,
        r".*ankle_pitch.*": 0.15,
        r".*ankle_roll.*": 0.1,
        r".*waist_yaw.*": 0.20,
        r".*waist_roll.*": 0.1,
        r".*shoulder_pitch.*": 0.6,
        r".*shoulder_roll.*": 0.1,
        r".*shoulder_yaw.*": 0.15,
        r".*elbow.*": 0.25,
        r".*wrist.*": 0.1,
    }
    cfg.rewards["pose"].params["std_running"] = {
        r".*hip_pitch.*": 0.5,
        r".*hip_roll.*": 0.10,
        r".*hip_yaw.*": 0.40,
        r".*knee.*": 0.5,
        r".*ankle_pitch.*": 0.25,
        r".*ankle_roll.*": 0.1,
        r".*waist_yaw.*": 0.30,
        r".*waist_roll.*": 0.1,
        r".*shoulder_pitch.*": 0.65,
        r".*shoulder_roll.*": 0.1,
        r".*shoulder_yaw.*": 0.15,
        r".*elbow.*": 0.25,
        r".*wrist.*": 0.1,
    }

    cfg.rewards["foot_gait"].weight = 0.5
    cfg.rewards["foot_gait"].params["period"] = 0.8
    cfg.rewards["foot_gait"].params["offset"] = [0.0, 0.5]
    cfg.rewards["foot_gait"].params["threshold"] = 0.56
    cfg.rewards["foot_gait"].params["command_threshold"] = 0.05

    cfg.rewards["foot_clearance"].weight = -1.0
    cfg.rewards["foot_clearance"].params["target_height"] = 0.05
    cfg.rewards["foot_clearance"].params["command_threshold"] = 0.1

    cfg.rewards["foot_slip"].weight = -0.25
    cfg.rewards["foot_slip"].params["command_threshold"] = 0.1

    cfg.rewards["soft_landing"].weight = -5e-3
    cfg.rewards["soft_landing"].params["command_threshold"] = 0.1

    cfg.rewards["stand_still"].weight = -1.5
    cfg.rewards["stand_still"].params["command_threshold"] = 0.05

    if "self_collisions" in cfg.rewards:
        cfg.rewards["self_collisions"].weight = -3.0
        cfg.rewards["self_collisions"].params["force_threshold"] = 5.0

    # ── New shaping rewards (added by the tuned recipe) ──
    cfg.rewards["feet_air_time"] = RewardTermCfg(
        func=mdp.feet_air_time,
        weight=0.25,
        params={
            "sensor_name": "feet_ground_contact",
            "threshold": 0.5,
            "command_name": "twist",
            "command_threshold": 0.05,
        },
    )
    cfg.rewards["feet_distance"] = RewardTermCfg(
        func=ws_rewards.feet_distance_penalty,
        weight=-3.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", site_names=_FEET_SITES),
            "min_distance": 0.15,
        },
    )
    cfg.rewards["straight_tracking"] = RewardTermCfg(
        func=ws_rewards.penalize_y_when_straight,
        weight=-2.0,
        params={"command_name": "twist"},
    )
    cfg.rewards["turning_tracking"] = RewardTermCfg(
        func=ws_rewards.penalize_xy_when_turning,
        weight=-1.0,
        params={"command_name": "twist"},
    )

    # ── Events: wider base-CoM randomization (payload_mass dropped: no dev_pc) ──
    base_com = cfg.events.get("base_com")
    if base_com is not None:
        base_com.params["ranges"] = {0: (-0.1, 0.1), 1: (-0.1, 0.1), 2: (-0.1, 0.1)}

    # ── Curriculum: tuned flat velocity stages ──
    command_curriculum = cfg.curriculum.get("command_vel")
    if command_curriculum is not None:
        command_curriculum.params["velocity_stages"] = [
            {
                "step": 0,
                "lin_vel_x": (-0.5, 1.0),
                "lin_vel_y": (-0.5, 0.5),
                "ang_vel_z": (-1.0, 1.0),
            },
            {
                "step": 2000 * 24,
                "lin_vel_x": (-1.0, 2.0),
                "lin_vel_y": (-0.5, 0.5),
                "ang_vel_z": (-1.0, 1.0),
            },
        ]

    return cfg


def _apply_flat_v4_agent(cfg: Any) -> Any:
    if not getattr(cfg, "run_name", ""):
        cfg.run_name = "r1_flat_walk_v4"
    # Tuned PPO: keep exploration alive (entropy 0.02) — v2/v3 crushed it to
    # 0.003-0.005, which is part of why they under-tracked and drifted.
    algo = cfg.algorithm
    algo.entropy_coef = 0.02
    algo.value_loss_coef = 1.0
    algo.clip_param = 0.2
    algo.num_learning_epochs = 5
    algo.num_mini_batches = 4
    algo.learning_rate = 1.0e-3
    algo.schedule = "adaptive"
    algo.gamma = 0.99
    algo.lam = 0.95
    algo.desired_kl = 0.01
    algo.max_grad_norm = 1.0
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


def register_flat_v3_task(
    register_fn: Any,
    *,
    env_cfg: Any,
    play_env_cfg: Any,
    rl_cfg: Any,
    runner_cls: type | None,
) -> None:
    """Register the local flat v3 task beside the upstream flat task."""
    register_fn(
        task_id=FLAT_V3_TASK_ID,
        env_cfg=_apply_flat_v3_env(deepcopy(env_cfg)),
        play_env_cfg=_apply_flat_v3_env(deepcopy(play_env_cfg)),
        rl_cfg=_apply_flat_v3_agent(deepcopy(rl_cfg)),
        runner_cls=runner_cls,
    )


def register_flat_v4_task(
    register_fn: Any,
    *,
    env_cfg: Any,
    play_env_cfg: Any,
    rl_cfg: Any,
    runner_cls: type | None,
) -> None:
    """Register the tuned flat v4 task (faithful G1-equivalent recipe)."""
    register_fn(
        task_id=FLAT_V4_TASK_ID,
        env_cfg=_apply_flat_v4_env(deepcopy(env_cfg)),
        play_env_cfg=_apply_flat_v4_env(deepcopy(play_env_cfg)),
        rl_cfg=_apply_flat_v4_agent(deepcopy(rl_cfg)),
        runner_cls=runner_cls,
    )
