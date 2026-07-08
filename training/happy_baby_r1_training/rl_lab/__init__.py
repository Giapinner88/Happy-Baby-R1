"""Register the workspace-local R1 task for Unitree RL Lab."""

import gymnasium as gym


gym.register(
    id="Unitree-R1-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.r1_velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.r1_velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:R1PPORunnerCfg",
    },
)

