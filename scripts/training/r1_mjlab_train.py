#!/usr/bin/env python3
"""Workspace-safe launcher for Unitree RL MJLab train.py."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MJLAB_ROOT = ROOT / "third_party" / "unitree_rl_mjlab"
UPSTREAM_TRAIN = ROOT / "third_party" / "unitree_rl_mjlab" / "scripts" / "train.py"


def _patch_warp_context() -> None:
    """Expose the old wp.context API expected by mjlab 1.2.0 on Warp 1.14."""
    try:
        import warp as wp
    except Exception:
        return
    if hasattr(wp, "context"):
        return
    try:
        import warp._src.context as context
    except Exception:
        return
    wp.context = context


def _patch_r1_robot_asset() -> None:
    """Make upstream Unitree-R1 tasks use the workspace MuJoCo R1 asset."""
    for path in (ROOT, MJLAB_ROOT):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)

    from training.mjlab import robot as robot_cfg
    import src.assets.robots as robots
    import src.assets.robots.unitree_r1.r1_constants as r1_constants

    robots.get_r1_robot_cfg = robot_cfg.get_r1_robot_cfg
    robots.R1_ACTION_SCALE = robot_cfg.R1_ACTION_SCALE
    r1_constants.get_r1_robot_cfg = robot_cfg.get_r1_robot_cfg
    r1_constants.R1_ACTION_SCALE = robot_cfg.R1_ACTION_SCALE


def _patch_flat_v2_task_registration() -> None:
    """Register local R1 task variants when upstream imports task configs."""
    from training.mjlab.profiles import (
        FLAT_V2_TASK_ID,
        FLAT_V3_TASK_ID,
        FLAT_V4_TASK_ID,
        register_flat_v2_task,
        register_flat_v3_task,
        register_flat_v4_task,
    )
    import mjlab.tasks.registry as registry

    original_register = registry.register_mjlab_task
    if getattr(original_register, "_happy_baby_r1_flat_v2", False):
        return

    def register_with_flat_v2(task_id, env_cfg, play_env_cfg, rl_cfg, runner_cls=None):
        original_register(task_id, env_cfg, play_env_cfg, rl_cfg, runner_cls)
        if task_id == "Unitree-R1-Flat" and FLAT_V2_TASK_ID not in registry.list_tasks():
            register_flat_v2_task(
                original_register,
                env_cfg=env_cfg,
                play_env_cfg=play_env_cfg,
                rl_cfg=rl_cfg,
                runner_cls=runner_cls,
            )
        if task_id == "Unitree-R1-Flat" and FLAT_V3_TASK_ID not in registry.list_tasks():
            register_flat_v3_task(
                original_register,
                env_cfg=env_cfg,
                play_env_cfg=play_env_cfg,
                rl_cfg=rl_cfg,
                runner_cls=runner_cls,
            )
        if task_id == "Unitree-R1-Flat" and FLAT_V4_TASK_ID not in registry.list_tasks():
            register_flat_v4_task(
                original_register,
                env_cfg=env_cfg,
                play_env_cfg=play_env_cfg,
                rl_cfg=rl_cfg,
                runner_cls=runner_cls,
            )

    register_with_flat_v2._happy_baby_r1_flat_v2 = True
    registry.register_mjlab_task = register_with_flat_v2


def main() -> None:
    _patch_warp_context()
    _patch_r1_robot_asset()
    _patch_flat_v2_task_registration()
    sys.argv[0] = str(UPSTREAM_TRAIN)
    runpy.run_path(str(UPSTREAM_TRAIN), run_name="__main__")


if __name__ == "__main__":
    main()
