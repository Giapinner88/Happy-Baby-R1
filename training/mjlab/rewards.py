"""Workspace-local velocity reward terms for the tuned R1 flat_v4 recipe.

These three functions are ported verbatim from the tuned Unitree RL MJLab
checkout (~/train_mujoco) because the vendored ``third_party`` copy does not
ship them. ``flat_v4`` wires them in via :mod:`training.mjlab.profiles`; nothing
in ``third_party`` is modified.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def feet_distance_penalty(
    env: "ManagerBasedRlEnv",
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    min_distance: float = 0.15,
) -> torch.Tensor:
    """Penalize when the two feet get closer than ``min_distance`` in the world plane.

    Uses the full xy distance so the penalty is heading-invariant (using only the
    world-y component would misfire whenever the robot faces away from +x).
    """
    asset: Entity = env.scene[asset_cfg.name]
    foot_pos_xy = asset.data.site_pos_w[:, asset_cfg.site_ids, :2]  # [B, 2, 2]
    dist = torch.norm(foot_pos_xy[:, 0, :] - foot_pos_xy[:, 1, :], dim=-1)  # [B]
    return torch.clamp(min_distance - dist, min=0.0)


def penalize_y_when_straight(
    env: "ManagerBasedRlEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize lateral velocity when the command is to walk straight (kills drift)."""
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    assert command is not None
    is_straight = (torch.abs(command[:, 2]) < 0.1) & (torch.abs(command[:, 0]) > 0.1)
    actual_vel_y = asset.data.root_link_lin_vel_b[:, 1]
    return torch.square(actual_vel_y) * is_straight.float()


def penalize_xy_when_turning(
    env: "ManagerBasedRlEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize translation when the command is a pure turn-in-place."""
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    assert command is not None
    is_turning = (torch.abs(command[:, 2]) > 0.1) & (torch.norm(command[:, :2], dim=1) < 0.1)
    actual_vel_xy = asset.data.root_link_lin_vel_b[:, :2]
    return torch.sum(torch.square(actual_vel_xy), dim=1) * is_turning.float()
