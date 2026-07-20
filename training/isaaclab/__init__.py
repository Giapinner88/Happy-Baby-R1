"""Register the workspace-local R1 task for Unitree RL Lab."""

import gymnasium as gym


gym.register(
    id="Unitree-R1-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.velocity_env:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.ppo:R1PPORunnerCfg",
    },
)
