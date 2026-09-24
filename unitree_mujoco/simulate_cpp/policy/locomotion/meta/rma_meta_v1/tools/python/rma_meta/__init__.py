"""RMA-inspired latent adaptation for the hierarchical R1 expert selector.

This package is intentionally separate from :mod:`src.tasks.velocity.meta_policy`.
The legacy supervised GRU gate remains a supported, independently testable path.
"""

from .config import (
  RMA_META_CHECKPOINT_VERSION,
  RMA_META_CONFIG_VERSION,
  RmaMetaCfg,
  load_rma_meta_cfg,
)
from .models import (
  AdaptationModule,
  DeployAdapter,
  DeployMetaSelector,
  EnvironmentFactorEncoder,
  MetaActorCritic,
)
from .runtime import ExpertSelectionState, TargetCrossfader

__all__ = [
  "RMA_META_CHECKPOINT_VERSION",
  "RMA_META_CONFIG_VERSION",
  "AdaptationModule",
  "DeployAdapter",
  "DeployMetaSelector",
  "EnvironmentFactorEncoder",
  "ExpertSelectionState",
  "MetaActorCritic",
  "RmaMetaCfg",
  "TargetCrossfader",
  "load_rma_meta_cfg",
]
