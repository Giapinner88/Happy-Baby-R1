"""R1 velocity task overlay for Unitree RL Lab.

This reuses the upstream G1 29DOF locomotion task structure without modifying
the vendored Unitree RL Lab repository.
"""

import importlib

from isaaclab.utils import configclass

from .robot import UNITREE_R1_CFG


_g1_cfg = importlib.import_module("unitree_rl_lab.tasks.locomotion.robots.g1.29dof.velocity_env_cfg")


@configclass
class RobotSceneCfg(_g1_cfg.RobotSceneCfg):
    robot = UNITREE_R1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class RewardsCfg(_g1_cfg.RewardsCfg):
    base_height = _g1_cfg.RewTerm(
        func=_g1_cfg.mdp.base_height_l2,
        weight=-10,
        params={"target_height": 0.76},
    )


@configclass
class RobotEnvCfg(_g1_cfg.RobotEnvCfg):
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=4096, env_spacing=2.5)
    rewards: RewardsCfg = RewardsCfg()

    def __post_init__(self):
        super().__post_init__()

        # The R1 USD has its root body at pelvis_link, so inherited base-body
        # references are redirected to that name.
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/pelvis_link"
        self.events.add_base_mass.params["asset_cfg"].body_names = "pelvis_link"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "pelvis_link"


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 2
            self.scene.terrain.terrain_generator.num_cols = 10
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
