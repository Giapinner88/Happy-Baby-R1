"""RSL-RL config for the workspace R1 IsaacLab task."""

from isaaclab.utils import configclass
from unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg import BasePPORunnerCfg


@configclass
class R1PPORunnerCfg(BasePPORunnerCfg):
    experiment_name = "r1_velocity"
    run_name = "rl_lab"
    max_iterations = 10001
    save_interval = 100

