"""Opt-in R1 roll training with a validated randomized stance at frame zero."""
from dataclasses import dataclass, fields
import hashlib
from pathlib import Path
import numpy as np
import torch
from src.tasks.tracking.mdp.commands import MotionCommand
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from .env_cfgs import unitree_r1_flat_tracking_env_cfg


class RollStanceMotionCommand(MotionCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if cfg.sampling_mode != 'start' or not cfg.stance_bank_file:
            raise ValueError('Roll stance reset requires sampling_mode=start and a stance bank')
        with np.load(cfg.stance_bank_file, allow_pickle=False) as bank:
            q = bank['joint_pos'].copy()
            idx = bank['leg_joint_indices'].copy()
            width = bank['width_m'].copy()
            reference = bank['reference_joint_pos'].copy()
            if str(bank['motion_sha256'][0]) != hashlib.sha256(Path(cfg.motion_file).read_bytes()).hexdigest():
                raise ValueError('Stance bank does not match motion')
        if q.ndim != 2 or q.shape[1] != self.robot.num_joints or q.shape[0] < 2:
            raise ValueError('Invalid stance bank dimensions')
        if idx.shape != (12,) or len(np.unique(idx)) != 12 or np.any(idx < 0) or np.any(idx >= q.shape[1]):
            raise ValueError('Invalid stance bank leg indexes')
        if width.shape != (len(q),) or not np.isfinite(q).all() or not np.isfinite(width).all():
            raise ValueError('Invalid stance bank values')
        if not np.allclose(reference, self.motion.joint_pos[0].cpu().numpy(), atol=1e-6):
            raise ValueError('Stance bank frame-zero reference differs from motion')
        self.stance_bank = torch.tensor(q, dtype=torch.float32, device=self.device)
        self.stance_leg_indices = torch.tensor(idx, dtype=torch.long, device=self.device)
        self.stance_widths = torch.tensor(width, dtype=torch.float32, device=self.device)
        limits = self.robot.data.soft_joint_pos_limits[0]
        if not bool(((self.stance_bank >= limits[:, 0] - 1e-5) &
                     (self.stance_bank <= limits[:, 1] + 1e-5)).all()):
            raise ValueError('Stance bank exceeds runtime soft joint limits')
        self.metrics['initial_stance_width'] = torch.zeros(self.num_envs, device=self.device)

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        if len(env_ids) == 0:
            return
        sample = torch.randint(len(self.stance_bank), (len(env_ids),), device=self.device)
        q = self.robot_joint_pos[env_ids].clone()
        v = self.robot_joint_vel[env_ids].clone()
        q[:, self.stance_leg_indices] = self.stance_bank[sample][:, self.stance_leg_indices]
        v[:, self.stance_leg_indices] = 0
        self.robot.write_joint_state_to_sim(q, v, env_ids=env_ids)
        self.robot.clear_state(env_ids=env_ids)
        self.metrics['initial_stance_width'][env_ids] = self.stance_widths[sample]


@dataclass(kw_only=True)
class RollStanceMotionCommandCfg(MotionCommandCfg):
    stance_bank_file: str = ''

    def build(self, env):
        return RollStanceMotionCommand(self, env)


def roll_stance_env_cfg(play=False):
    cfg = unitree_r1_flat_tracking_env_cfg(has_state_estimation=False, play=play)
    original = cfg.commands['motion']
    original.sampling_mode = 'start'
    original.pose_range = {}
    original.velocity_range = {}
    original.joint_position_range = (0.0, 0.0)
    if not play:
        cfg.commands['motion'] = RollStanceMotionCommandCfg(
            **{f.name: getattr(original, f.name) for f in fields(original)})
    return cfg
