"""Fail-closed checkpoint schemas for all RMA-meta training stages."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import torch

from .config import (
  RMA_META_CHECKPOINT_VERSION,
  RmaMetaCfg,
  rma_meta_cfg_from_dict,
)
from .models import AdaptationModule, EnvironmentFactorEncoder, MetaActorCritic


def sha256_file(path: str | Path) -> str:
  digest = hashlib.sha256()
  with Path(path).open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def save_checkpoint_atomic(payload: dict[str, Any], path: str | Path) -> Path:
  """Write a checkpoint atomically so interrupted jobs keep the last good file."""
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + ".tmp")
  torch.save(payload, temporary)
  temporary.replace(path)
  return path


def make_checkpoint(
  *,
  stage: str,
  cfg: RmaMetaCfg,
  actor_critic: MetaActorCritic,
  factor_encoder: EnvironmentFactorEncoder,
  adaptation: AdaptationModule | None,
  iteration: int,
  registry_sha256: str,
  optimizer_state_dict: dict[str, Any] | None = None,
  metrics: dict[str, Any] | None = None,
  provenance: dict[str, Any] | None = None,
) -> dict[str, Any]:
  if stage not in ("teacher", "adaptation", "deployment_finetune"):
    raise ValueError("invalid RMA meta checkpoint stage")
  if len(registry_sha256) != 64:
    raise ValueError("registry_sha256 must be a SHA-256 hex digest")
  if stage != "teacher" and adaptation is None:
    raise ValueError(f"stage {stage!r} requires an adaptation module")
  return {
    "checkpoint_version": RMA_META_CHECKPOINT_VERSION,
    "stage": stage,
    "config": cfg.to_dict(),
    "expert_names": cfg.expert_names,
    "class_names": cfg.class_names,
    "actor_critic_state_dict": actor_critic.state_dict(),
    "factor_encoder_state_dict": factor_encoder.state_dict(),
    "adaptation_state_dict": (
      adaptation.state_dict() if adaptation is not None else None
    ),
    "iteration": int(iteration),
    "registry_sha256": registry_sha256,
    "optimizer_state_dict": optimizer_state_dict,
    "metrics": metrics or {},
    "provenance": provenance or {},
  }


def load_checkpoint_models(
  path: str | Path,
  *,
  map_location: str | torch.device = "cpu",
  expected_stage: str | None = None,
  expected_expert_names: tuple[str, ...] | None = None,
) -> tuple[
  dict[str, Any],
  RmaMetaCfg,
  MetaActorCritic,
  EnvironmentFactorEncoder,
  AdaptationModule | None,
]:
  payload = torch.load(path, map_location=map_location, weights_only=False)
  if int(payload.get("checkpoint_version", 0)) != RMA_META_CHECKPOINT_VERSION:
    raise ValueError("unsupported RMA meta checkpoint version")
  stage = str(payload.get("stage", ""))
  if expected_stage is not None and stage != expected_stage:
    raise ValueError(f"checkpoint stage is {stage!r}, expected {expected_stage!r}")
  cfg = rma_meta_cfg_from_dict(dict(payload["config"]))
  if tuple(payload.get("expert_names", ())) != cfg.expert_names:
    raise ValueError("checkpoint expert_names do not match embedded config")
  if tuple(payload.get("class_names", ())) != cfg.class_names:
    raise ValueError("checkpoint class_names do not match embedded config")
  if expected_expert_names is not None and cfg.expert_names != expected_expert_names:
    raise ValueError("checkpoint experts do not match requested registry")
  actor_critic = MetaActorCritic(cfg).to(map_location)
  factor_encoder = EnvironmentFactorEncoder(cfg.model).to(map_location)
  actor_critic.load_state_dict(payload["actor_critic_state_dict"], strict=True)
  factor_encoder.load_state_dict(
    payload["factor_encoder_state_dict"], strict=True
  )
  adaptation_state = payload.get("adaptation_state_dict")
  adaptation = None
  if adaptation_state is not None:
    adaptation = AdaptationModule(cfg.model).to(map_location)
    adaptation.load_state_dict(adaptation_state, strict=True)
  elif stage != "teacher":
    raise ValueError(f"checkpoint stage {stage!r} is missing adaptation weights")
  return payload, cfg, actor_critic, factor_encoder, adaptation
