"""Sequence-indexed Isaac Lab replay of an offline upper-body trajectory."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Protocol, Sequence

import numpy as np

from .mapping import R1TeleopTargets
from .upper_body_kinematics import load_r1_a5_upper_body_model


class OfflineReplayHandle(Protocol):
    def write_joint_targets(
        self, joint_names: Sequence[str], positions_rad: Sequence[float]
    ) -> None: ...

    def joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]: ...


@dataclass(frozen=True)
class OfflineTrajectoryReplayConfig:
    trajectory_path: Path
    urdf_path: Path
    body_mode: str = "arms_head"
    fixed_waist_yaw_rad: float = 0.0
    hold_uncontrolled_waist_joints: bool = False
    """Command the waist joints the solver does not own to their fixed values.

    ``arms_head`` leaves ``waist_yaw_joint`` and ``waist_roll_joint`` out of the
    controlled set, so nothing writes them and PhysX lets them deflect under the
    reaction torque of the swinging arms. Run
    ``t007_mirror_equivariant_natural_isaac_20260822_v21`` measured 22.4 deg of
    yaw and 16.7 deg of roll that way, while the offline solver assumed the
    waist stayed at ``fixed_waist_yaw_rad``. That mismatch moved the hand by up
    to 153 mm and made the offline reference and the observed state disagree for
    a reason that had nothing to do with the arm solution.

    Enabling this holds those joints at the pose the solver assumed, so the
    simulation matches the model it is being compared against. It does not give
    the waist to the IK: the controlled joint set is unchanged.
    """
    waist_roll_hold_rad: float = 0.0
    held_joint_stiffness: float = 10000.0
    """Actuator stiffness applied to held joints, N m/rad.

    The asset's waist actuator is 100 N m/rad, which the arms' reaction torque
    bends by up to 22.4 deg. A hold has to be stiff enough that the residual
    deflection is negligible against the quantity being measured.
    """
    held_joint_damping: float = 200.0


@dataclass
class OfflineTrajectoryIsaacLabSink:
    """Dispatch precomputed joints by the original Quest command sequence id."""

    handle: OfflineReplayHandle
    config: OfflineTrajectoryReplayConfig
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    last_application: dict[str, object] | None = None
    arm_targets_withheld: int = 0

    def __post_init__(self) -> None:
        if self.config.body_mode != "arms_head":
            raise ValueError("Offline continuation v1 supports body_mode='arms_head' only.")
        self.model = load_r1_a5_upper_body_model(
            self.config.urdf_path,
            control_waist_yaw=False,
            fixed_waist_yaw_rad=self.config.fixed_waist_yaw_rad,
        )
        # Joints the solver does not own but the simulation must still hold, so
        # PhysX matches the pose the offline model assumed. Kept separate from
        # `model.joint_names` because the controlled set must not change.
        self._held_joint_names: tuple[str, ...] = ()
        self._held_joint_values: tuple[float, ...] = ()
        if self.config.hold_uncontrolled_waist_joints:
            self._held_joint_names = ("waist_yaw_joint", "waist_roll_joint")
            self._held_joint_values = (
                float(self.config.fixed_waist_yaw_rad),
                float(self.config.waist_roll_hold_rad),
            )
        with np.load(self.config.trajectory_path) as data:
            self.time_s = np.asarray(data["time_s"], dtype=float)
            self.sequence_id = np.asarray(data["sequence_id"], dtype=int)
            self.q_rad = np.asarray(data["joint_position_reference_rad"], dtype=float)
            joint_names = tuple(str(value) for value in data["joint_names"].tolist())
            self.left_target_position_m = np.asarray(
                data["left_target_position_pelvis_m"], dtype=float
            )
            self.right_target_position_m = np.asarray(
                data["right_target_position_pelvis_m"], dtype=float
            )
        if joint_names != tuple(self.model.joint_names):
            raise ValueError("Offline trajectory joint names do not match the selected R1 model.")
        if self.q_rad.shape != (len(self.sequence_id), self.model.dof):
            raise ValueError("Offline trajectory joint array has an invalid shape.")
        if len(self.sequence_id) == 0 or len(np.unique(self.sequence_id)) != len(self.sequence_id):
            raise ValueError("Offline trajectory sequence ids must be non-empty and unique.")
        if not np.all(np.isfinite(self.q_rad)):
            raise ValueError("Offline trajectory contains non-finite joint targets.")
        if np.any(self.q_rad < self.model.lower_limits) or np.any(
            self.q_rad > self.model.upper_limits
        ):
            raise ValueError("Offline trajectory exceeds the selected R1 joint limits.")
        self._index_by_sequence = {
            int(sequence): index for index, sequence in enumerate(self.sequence_id)
        }
        self.nominal_joint_position_rad = tuple(float(value) for value in self.q_rad[0])
        self.last_target = self.q_rad[0].copy()

    @property
    def held_joint_names(self) -> tuple[str, ...]:
        return self._held_joint_names

    def _write(self, positions_rad: object) -> None:
        """Write the controlled vector plus any declared held joints."""

        names = tuple(self.model.joint_names) + self._held_joint_names
        values = tuple(float(v) for v in positions_rad) + self._held_joint_values
        self.handle.write_joint_targets(names, values)

    def reset_session(self) -> None:
        self.last_target = self.q_rad[0].copy()
        self._write(self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": "session_reset_to_offline_trajectory_start",
            "joint_target_rad": self.last_target.tolist(),
        }
        self.events.append({"event": "session_reset_to_offline_trajectory_start"})

    def hold(self, reason: str) -> None:
        self._write(self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": reason,
            "joint_target_rad": self.last_target.tolist(),
        }
        self.events.append({"event": "hold", **self.last_application})

    def apply_upper_body(
        self, targets: R1TeleopTargets, joints: Sequence[str]
    ) -> None:
        compute_start = time.perf_counter()
        missing = set(self.model.joint_names) - set(joints)
        if missing:
            raise ValueError(f"Offline replay has incomplete ownership: {sorted(missing)}")
        index = self._index_by_sequence.get(int(targets.sequence_id))
        if index is None:
            self.hold("sequence_not_in_offline_trajectory")
            return
        self.last_target = self.q_rad[index].copy()
        self._write(self.last_target)
        head = self.model.head_slice
        pitch_index, yaw_index = head.start, head.start + 1
        self.last_application = {
            "accepted": True,
            "controller_type": "offline_trajectory_continuation",
            "trajectory_index": int(index),
            "trajectory_time_s": float(self.time_s[index]),
            "controller_compute_ms": float(
                1000.0 * (time.perf_counter() - compute_start)
            ),
            "controlled_joint_names": list(self.model.joint_names),
            "held_joint_names": list(self._held_joint_names),
            "held_joint_position_rad": list(self._held_joint_values),
            "joint_target_rad": self.last_target.tolist(),
            "limited_joint_target_rad": self.last_target.tolist(),
            "left_arm_target_rad": self.last_target[self.model.left_arm_slice].tolist(),
            "right_arm_target_rad": self.last_target[self.model.right_arm_slice].tolist(),
            "head_pitch_yaw_target_rad": self.last_target[head].tolist(),
            "head_target_rad": [
                float(self.last_target[yaw_index]),
                float(self.last_target[pitch_index]),
            ],
            "left_target_position_pelvis_m": self.left_target_position_m[index].tolist(),
            "right_target_position_pelvis_m": self.right_target_position_m[index].tolist(),
        }
        self.events.append(
            {
                "event": "whole_upper_body",
                "sequence_id": int(targets.sequence_id),
                **self.last_application,
            }
        )
        self.acknowledgements.append(
            {
                "sequence_id": int(targets.sequence_id),
                "accepted_joints": list(self.model.joint_names),
                "withheld_joints": [],
            }
        )

    def apply_base_velocity(
        self, targets: R1TeleopTargets, joints: Sequence[str]
    ) -> None:
        raise RuntimeError("Base velocity is prohibited in offline trajectory replay.")


__all__ = ["OfflineTrajectoryIsaacLabSink", "OfflineTrajectoryReplayConfig"]
