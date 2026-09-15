"""RSL-RL config for the workspace R1 IsaacLab task."""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg
from unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg import BasePPORunnerCfg


@configclass
class R1PPORunnerCfg(BasePPORunnerCfg):
    # RSL-RL 5 consumes separate actor/critic model configurations.  The
    # Unitree base config still supplies the deprecated combined `policy`
    # config, but its launcher does not run Isaac Lab's compatibility adapter.
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
    )
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    experiment_name = "r1_velocity"
    run_name = "rl_lab"
    max_iterations = 10001
    save_interval = 100

    def __post_init__(self):
        # The Unitree launcher bypasses Isaac Lab's deprecated-config adapter.
        # Remove the RSL-RL <5 fields ourselves; RSL-RL 5 reads the output
        # distribution exclusively from ``distribution_cfg``.
        for model_cfg in (self.actor, self.critic):
            for field in ("stochastic", "init_noise_std", "noise_std_type", "state_dependent_std"):
                if hasattr(model_cfg, field):
                    delattr(model_cfg, field)
