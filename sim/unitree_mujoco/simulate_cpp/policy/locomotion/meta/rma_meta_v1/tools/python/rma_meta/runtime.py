"""Tensorized high-level expert selection and target crossfade state."""

from __future__ import annotations

import torch


class ExpertSelectionState:
  """Resolve categorical expert/HOLD proposals for batched environments."""

  def __init__(
    self,
    num_envs: int,
    num_experts: int,
    initial_expert_index: int,
    *,
    device: torch.device | str,
  ) -> None:
    if num_envs <= 0 or num_experts <= 0:
      raise ValueError("selection state dimensions must be positive")
    if not 0 <= initial_expert_index < num_experts:
      raise ValueError("initial expert index is outside the registry")
    self.num_envs = num_envs
    self.num_experts = num_experts
    self.hold_index = num_experts
    self.initial_expert_index = initial_expert_index
    self.device = torch.device(device)
    self.selected = torch.full(
      (num_envs,), initial_expert_index, device=self.device, dtype=torch.long
    )

  @property
  def one_hot(self) -> torch.Tensor:
    return torch.nn.functional.one_hot(
      self.selected, num_classes=self.num_experts
    ).float()

  def resolve(self, proposed: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    if proposed.shape != (self.num_envs,):
      raise ValueError("meta action must have shape [num_envs]")
    proposed = proposed.to(device=self.device, dtype=torch.long)
    if torch.any(proposed < 0) or torch.any(proposed > self.hold_index):
      raise ValueError("meta action contains an invalid class index")
    previous = self.selected.clone()
    choose = proposed < self.num_experts
    self.selected = torch.where(choose, proposed, self.selected)
    return self.selected.clone(), self.selected != previous

  def reset(self, env_ids: torch.Tensor | None = None) -> None:
    if env_ids is None:
      self.selected.fill_(self.initial_expert_index)
      return
    env_ids = env_ids.to(self.device, dtype=torch.long)
    self.selected[env_ids] = self.initial_expert_index


class TargetCrossfader:
  """Crossfade normalized common-contract actions after expert changes."""

  def __init__(
    self,
    num_envs: int,
    action_dim: int,
    crossfade_steps: int,
    *,
    device: torch.device | str,
  ) -> None:
    if min(num_envs, action_dim, crossfade_steps) <= 0:
      raise ValueError("crossfader dimensions and steps must be positive")
    self.num_envs = num_envs
    self.action_dim = action_dim
    self.crossfade_steps = crossfade_steps
    self.device = torch.device(device)
    self.last_action = torch.zeros(num_envs, action_dim, device=self.device)
    self.blend_from = self.last_action.clone()
    self.blend_step = torch.full(
      (num_envs,), crossfade_steps, device=self.device, dtype=torch.long
    )

  def begin_switch(self, changed: torch.Tensor) -> None:
    if changed.shape != (self.num_envs,):
      raise ValueError("changed mask must have shape [num_envs]")
    changed = changed.to(self.device, dtype=torch.bool)
    self.blend_from[changed] = self.last_action[changed]
    self.blend_step[changed] = 0

  def apply(self, target_action: torch.Tensor) -> torch.Tensor:
    if target_action.shape != (self.num_envs, self.action_dim):
      raise ValueError("target action has the wrong shape")
    if not torch.isfinite(target_action).all():
      raise ValueError("target action contains non-finite values")
    active = self.blend_step < self.crossfade_steps
    self.blend_step[active] += 1
    alpha = torch.clamp(
      self.blend_step.float() / self.crossfade_steps, 0.0, 1.0
    )[:, None]
    output = torch.where(
      active[:, None],
      (1.0 - alpha) * self.blend_from + alpha * target_action,
      target_action,
    )
    self.last_action.copy_(output)
    return output

  def reset(self, env_ids: torch.Tensor | None = None) -> None:
    if env_ids is None:
      self.last_action.zero_()
      self.blend_from.zero_()
      self.blend_step.fill_(self.crossfade_steps)
      return
    env_ids = env_ids.to(self.device, dtype=torch.long)
    self.last_action[env_ids] = 0.0
    self.blend_from[env_ids] = 0.0
    self.blend_step[env_ids] = self.crossfade_steps
