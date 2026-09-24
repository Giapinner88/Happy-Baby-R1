"""RL configuration for Unitree R1 velocity task."""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def _base_ppo_algorithm_cfg() -> RslRlPpoAlgorithmCfg:
  """Shared PPO algorithm hyperparameters for all R1 tasks."""
  return RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0,
    use_clipped_value_loss=True,
    clip_param=0.2,
    entropy_coef=0.01,
    num_learning_epochs=5,
    num_mini_batches=4,
    learning_rate=1.0e-3,
    schedule="adaptive",
    gamma=0.99,
    lam=0.95,
    desired_kl=0.01,
    max_grad_norm=1.0,
  )


def _base_model_cfg() -> RslRlModelCfg:
  """Shared actor/critic architecture for all R1 tasks."""
  return RslRlModelCfg(
    hidden_dims=(512, 256, 128),
    activation="elu",
    obs_normalization=True,
  )


def unitree_r1_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """RL runner config for Unitree R1 Flat terrain (20000 iterations)."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 1.0,
        "std_type": "scalar",
      },
    ),
    critic=_base_model_cfg(),
    algorithm=_base_ppo_algorithm_cfg(),
    experiment_name="r1_velocity",
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=10000,
  )


def unitree_r1_rough_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """RL runner config for Unitree R1 Rough terrain (30000 iterations).

  Rough terrain requires more iterations than flat because:
  - Curriculum learning needs time to ramp up terrain difficulty.
  - The policy must learn diverse foot placement strategies.
  - Raycast observations add complexity to the observation space (270 dims).
  """
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 1.0,
        "std_type": "scalar",
      },
    ),
    critic=_base_model_cfg(),
    algorithm=_base_ppo_algorithm_cfg(),
    experiment_name="r1_velocity_rough",
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=30000,
  )

