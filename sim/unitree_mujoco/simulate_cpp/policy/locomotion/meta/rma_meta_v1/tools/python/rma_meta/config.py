"""Versioned configuration for the RMA-inspired expert meta-policy."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


RMA_META_CONFIG_VERSION = 1
RMA_META_CHECKPOINT_VERSION = 1
HOLD_NAME = "HOLD"


@dataclass(frozen=True)
class RmaMetaModelCfg:
  observation_dim: int = 83
  factor_dim: int = 12
  latent_dim: int = 8
  history_steps: int = 50
  adaptation_hidden_dim: int = 128
  adaptation_num_layers: int = 1
  factor_hidden_dims: tuple[int, ...] = (64, 32)
  meta_hidden_dims: tuple[int, ...] = (256, 128)

  def __post_init__(self) -> None:
    scalar_values = (
      self.observation_dim,
      self.factor_dim,
      self.latent_dim,
      self.history_steps,
      self.adaptation_hidden_dim,
      self.adaptation_num_layers,
    )
    if any(value <= 0 for value in scalar_values):
      raise ValueError("all RMA meta model dimensions must be positive")
    if not self.factor_hidden_dims or not self.meta_hidden_dims:
      raise ValueError("RMA meta MLP hidden dimensions cannot be empty")
    if any(value <= 0 for value in (*self.factor_hidden_dims, *self.meta_hidden_dims)):
      raise ValueError("RMA meta MLP hidden dimensions must be positive")


@dataclass(frozen=True)
class RmaMetaPpoCfg:
  meta_decimation: int = 5
  rollout_steps: int = 32
  learning_epochs: int = 5
  mini_batches: int = 4
  learning_rate: float = 3.0e-4
  finetune_learning_rate: float = 1.0e-4
  clip_param: float = 0.2
  value_loss_coef: float = 1.0
  entropy_coef: float = 0.01
  gamma: float = 0.99
  gae_lambda: float = 0.95
  max_grad_norm: float = 1.0
  target_kl: float = 0.02
  switch_penalty: float = 0.02
  max_iterations: int = 3000
  finetune_iterations: int = 1000
  save_interval: int = 100

  def __post_init__(self) -> None:
    integer_values = (
      self.meta_decimation,
      self.rollout_steps,
      self.learning_epochs,
      self.mini_batches,
      self.max_iterations,
      self.finetune_iterations,
      self.save_interval,
    )
    if any(value <= 0 for value in integer_values):
      raise ValueError("RMA meta PPO integer settings must be positive")
    if self.rollout_steps % self.mini_batches != 0:
      raise ValueError("rollout_steps must be divisible by mini_batches")
    if min(self.learning_rate, self.finetune_learning_rate) <= 0.0:
      raise ValueError("PPO learning rates must be positive")
    if self.max_grad_norm <= 0.0:
      raise ValueError("learning rate and max_grad_norm must be positive")
    if not 0.0 < self.clip_param < 1.0:
      raise ValueError("clip_param must be in (0, 1)")
    if not 0.0 < self.gamma <= 1.0 or not 0.0 < self.gae_lambda <= 1.0:
      raise ValueError("gamma and gae_lambda must be in (0, 1]")
    if min(self.value_loss_coef, self.entropy_coef, self.target_kl) < 0.0:
      raise ValueError("PPO coefficients cannot be negative")
    if self.switch_penalty < 0.0:
      raise ValueError("switch_penalty cannot be negative")


@dataclass(frozen=True)
class RmaMetaAdaptationCfg:
  epochs: int = 100
  batch_size: int = 512
  learning_rate: float = 3.0e-4
  weight_decay: float = 1.0e-4
  validation_fraction: float = 0.1
  observation_noise_std: float = 0.01

  def __post_init__(self) -> None:
    if self.epochs <= 0 or self.batch_size <= 0:
      raise ValueError("adaptation epochs and batch_size must be positive")
    if self.learning_rate <= 0.0 or self.weight_decay < 0.0:
      raise ValueError("invalid adaptation optimizer settings")
    if not 0.0 < self.validation_fraction < 1.0:
      raise ValueError("validation_fraction must be in (0, 1)")
    if self.observation_noise_std < 0.0:
      raise ValueError("observation_noise_std cannot be negative")


@dataclass(frozen=True)
class RmaMetaRuntimeCfg:
  low_level_hz: float = 50.0
  meta_hz: float = 10.0
  confidence_threshold: float = 0.55
  crossfade_s: float = 0.2

  def __post_init__(self) -> None:
    if self.low_level_hz <= 0.0 or self.meta_hz <= 0.0:
      raise ValueError("runtime frequencies must be positive")
    ratio = self.low_level_hz / self.meta_hz
    if abs(ratio - round(ratio)) > 1.0e-6:
      raise ValueError("low_level_hz must be an integer multiple of meta_hz")
    if not 0.0 <= self.confidence_threshold <= 1.0:
      raise ValueError("confidence_threshold must be in [0, 1]")
    if self.crossfade_s < 0.0:
      raise ValueError("crossfade_s cannot be negative")

  @property
  def meta_decimation(self) -> int:
    return round(self.low_level_hz / self.meta_hz)

  @property
  def crossfade_steps(self) -> int:
    return max(1, round(self.crossfade_s * self.low_level_hz))


@dataclass(frozen=True)
class RmaMetaCfg:
  config_version: int = RMA_META_CONFIG_VERSION
  task_id: str = "Unitree-R1-RMA-Meta"
  expert_registry: str = (
    "configs/rma_meta/r1_expert_registry_v1.yaml"
  )
  deploy_template: str = (
    "deploy/robots/r1/config/policy/velocity/v0/params/"
    "deploy_with_flat.template.yaml"
  )
  expert_names: tuple[str, ...] = ("slope_up", "slope_down", "flat")
  initial_expert: str = "flat"
  factor_names: tuple[str, ...] = (
    "ground_normal_b_x",
    "ground_normal_b_y",
    "ground_normal_b_z",
    "uphill_direction_b_x",
    "uphill_direction_b_y",
    "uphill_direction_b_z",
    "foot_friction_delta",
    "body_mass_ratio_delta",
    "torso_com_delta_x_norm",
    "torso_com_delta_y_norm",
    "torso_com_delta_z_norm",
    "motor_strength_ratio_delta",
  )
  model: RmaMetaModelCfg = field(default_factory=RmaMetaModelCfg)
  ppo: RmaMetaPpoCfg = field(default_factory=RmaMetaPpoCfg)
  adaptation: RmaMetaAdaptationCfg = field(default_factory=RmaMetaAdaptationCfg)
  runtime: RmaMetaRuntimeCfg = field(default_factory=RmaMetaRuntimeCfg)

  def __post_init__(self) -> None:
    if self.config_version != RMA_META_CONFIG_VERSION:
      raise ValueError(
        f"RMA meta config version must be {RMA_META_CONFIG_VERSION}, "
        f"got {self.config_version}"
      )
    if not self.expert_names or len(set(self.expert_names)) != len(self.expert_names):
      raise ValueError("expert_names must be non-empty and unique")
    if self.initial_expert not in self.expert_names:
      raise ValueError("initial_expert must be present in expert_names")
    if HOLD_NAME in self.expert_names:
      raise ValueError(f"{HOLD_NAME} is reserved and cannot be an expert name")
    if len(self.factor_names) != self.model.factor_dim:
      raise ValueError("factor_names length must equal model.factor_dim")
    if self.ppo.meta_decimation != self.runtime.meta_decimation:
      raise ValueError("PPO and runtime meta decimation must match")

  @property
  def initial_expert_index(self) -> int:
    return self.expert_names.index(self.initial_expert)

  @property
  def hold_index(self) -> int:
    return len(self.expert_names)

  @property
  def class_names(self) -> tuple[str, ...]:
    return (*self.expert_names, HOLD_NAME)

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)


def _tuple_fields(payload: dict[str, Any], fields: tuple[str, ...]) -> None:
  for name in fields:
    if name in payload:
      payload[name] = tuple(payload[name])


def load_rma_meta_cfg(path: str | Path) -> RmaMetaCfg:
  """Load a fail-closed YAML config into immutable dataclasses."""
  payload = yaml.safe_load(Path(path).read_text())
  if not isinstance(payload, dict):
    raise ValueError("RMA meta config must contain a YAML mapping")
  return rma_meta_cfg_from_dict(payload)


def rma_meta_cfg_from_dict(payload: dict[str, Any]) -> RmaMetaCfg:
  """Reconstruct a config from YAML or checkpoint dictionary data."""
  payload = dict(payload)
  model_payload = dict(payload.pop("model", {}))
  _tuple_fields(model_payload, ("factor_hidden_dims", "meta_hidden_dims"))
  ppo_payload = dict(payload.pop("ppo", {}))
  adaptation_payload = dict(payload.pop("adaptation", {}))
  runtime_payload = dict(payload.pop("runtime", {}))
  _tuple_fields(payload, ("expert_names", "factor_names"))
  return RmaMetaCfg(
    **payload,
    model=RmaMetaModelCfg(**model_payload),
    ppo=RmaMetaPpoCfg(**ppo_payload),
    adaptation=RmaMetaAdaptationCfg(**adaptation_payload),
    runtime=RmaMetaRuntimeCfg(**runtime_payload),
  )
