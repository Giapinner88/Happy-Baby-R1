"""Head-only IsaacLab R1 teleop sink for the T001 connectivity pilot.

Scope is deliberately narrow. `R1TeleopMapper` already produces head yaw and
pitch as scalars, so driving the two head joints needs no inverse kinematics.
Arm and wrist targets are carried through the command path and recorded, but are
**not** written to the simulator: the arm/wrist IK method gate ("Gate M" in
`experiments/r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md`)
requires a reviewed method record before any IK runs, and T001 asks only whether
the connection is established, observed, and fails closed.

The sink is split from the simulator so it can be unit tested without Isaac Sim.
`R1SimulatorHandle` is the whole surface the sink needs; `IsaacLabArticulationHandle`
implements it against an IsaacLab articulation and imports nothing at module
scope, so this file stays importable in environments without IsaacLab.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence

from .mapping import R1TeleopTargets


HEAD_JOINT_NAMES: tuple[str, str] = ("head_yaw_joint", "head_pitch_joint")
LEFT_ARM_JOINT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
)
RIGHT_ARM_JOINT_NAMES: tuple[str, ...] = (
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
)


class VelocityDispatchError(RuntimeError):
    """Raised if a base-velocity dispatch reaches the T001 sink.

    T001 runs with velocity disabled, so this is an invariant breach in the
    mapper or adapter rather than a recoverable runtime condition.
    """


class R1SimulatorHandle(Protocol):
    """Minimal simulator surface required by the head-only sink."""

    def write_head_targets(self, yaw_rad: float, pitch_rad: float) -> None: ...

    def head_joint_positions(self) -> tuple[float, float]: ...


@dataclass
class HeadOnlyIsaacLabSink:
    """Applies head targets to a simulator and holds them on any unsafe input.

    Hold semantics are freeze-in-place: the last commanded head target is
    re-issued rather than snapping to a nominal pose, because a snap would be an
    uncommanded fast motion at exactly the moment the input became untrusted.
    The first hold before any enabled command leaves the head at its spawn pose.
    """

    handle: R1SimulatorHandle
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    last_head_target: tuple[float, float] | None = None
    arm_targets_withheld: int = 0

    def hold(self, reason: str) -> None:
        if self.last_head_target is None:
            observed = self.handle.head_joint_positions()
        else:
            self.handle.write_head_targets(*self.last_head_target)
            observed = self.handle.head_joint_positions()
        self.events.append(
            {
                "event": "hold",
                "reason": reason,
                "held_head_target_rad": list(self.last_head_target) if self.last_head_target else None,
                "observed_head_position_rad": list(observed),
            }
        )

    def apply_upper_body(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        missing = [name for name in HEAD_JOINT_NAMES if name not in tuple(joints)]
        if missing:
            raise ValueError(f"Upper-body ownership is missing head joints: {missing}")
        self.handle.write_head_targets(targets.head_yaw_rad, targets.head_pitch_rad)
        self.last_head_target = (targets.head_yaw_rad, targets.head_pitch_rad)
        if targets.left_wrist_target is not None or targets.right_wrist_target is not None:
            self.arm_targets_withheld += 1
        self.events.append(
            {
                "event": "upper_body",
                "sequence_id": targets.sequence_id,
                "commanded_head_target_rad": [targets.head_yaw_rad, targets.head_pitch_rad],
                "pre_physics_head_position_rad": list(self.handle.head_joint_positions()),
                "arm_target_written": False,
                "arm_target_withheld_reason": "arm_wrist_ik_method_gate",
            }
        )
        self.acknowledgements.append(
            {
                "sequence_id": targets.sequence_id,
                "accepted_joints": list(HEAD_JOINT_NAMES),
                "withheld_joints": [name for name in joints if name not in HEAD_JOINT_NAMES],
            }
        )

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        raise VelocityDispatchError(
            "T001 runs with base velocity disabled; a lower-body dispatch invalidates the run."
        )


class IsaacLabArticulationHandle:
    """Adapts an IsaacLab R1 articulation to `R1SimulatorHandle`.

    Joint indices are resolved once by name against the articulation, so a USD
    whose joint ordering differs from the SDK ordering cannot silently shift the
    head targets onto other joints.
    """

    def __init__(self, articulation: object, env_index: int = 0) -> None:
        self.articulation = articulation
        self.env_index = env_index
        names = list(articulation.data.joint_names)
        self._joint_name_to_id = {name: index for index, name in enumerate(names)}
        missing = [name for name in HEAD_JOINT_NAMES if name not in names]
        if missing:
            raise ValueError(
                f"R1 articulation is missing head joints {missing}; the asset cannot serve the T001 sink."
            )
        self.joint_ids = [names.index(name) for name in HEAD_JOINT_NAMES]
        missing_arms = [name for name in (*LEFT_ARM_JOINT_NAMES, *RIGHT_ARM_JOINT_NAMES) if name not in names]
        if missing_arms:
            raise ValueError(f"R1 articulation is missing arm joints {missing_arms}.")
        self.left_arm_joint_ids = [names.index(name) for name in LEFT_ARM_JOINT_NAMES]
        self.right_arm_joint_ids = [names.index(name) for name in RIGHT_ARM_JOINT_NAMES]

    def write_head_targets(self, yaw_rad: float, pitch_rad: float) -> None:
        import torch

        target = torch.tensor(
            [[float(yaw_rad), float(pitch_rad)]],
            device=self.articulation.device,
            dtype=torch.float32,
        )
        self.articulation.set_joint_position_target(target, joint_ids=self.joint_ids)

    def head_joint_positions(self) -> tuple[float, float]:
        positions = self.articulation.data.joint_pos[self.env_index]
        return (float(positions[self.joint_ids[0]]), float(positions[self.joint_ids[1]]))

    def write_arm_targets(self, left_rad: Sequence[float], right_rad: Sequence[float]) -> None:
        import torch
        if len(left_rad) != 5 or len(right_rad) != 5:
            raise ValueError("Each R1 arm target must contain exactly five joints.")
        self.articulation.set_joint_position_target(
            torch.tensor([[float(value) for value in left_rad]], device=self.articulation.device, dtype=torch.float32), joint_ids=self.left_arm_joint_ids,
        )
        self.articulation.set_joint_position_target(
            torch.tensor([[float(value) for value in right_rad]], device=self.articulation.device, dtype=torch.float32), joint_ids=self.right_arm_joint_ids,
        )

    def arm_joint_positions(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        positions = self.articulation.data.joint_pos[self.env_index]
        return (tuple(float(positions[index]) for index in self.left_arm_joint_ids), tuple(float(positions[index]) for index in self.right_arm_joint_ids))

    def write_joint_targets(self, joint_names: Sequence[str], positions_rad: Sequence[float]) -> None:
        """Write one name-resolved target vector without assuming Isaac/SDK order."""

        import torch

        names = tuple(joint_names)
        values = tuple(float(value) for value in positions_rad)
        if len(names) != len(values):
            raise ValueError("Joint target names and values have different lengths.")
        missing = [name for name in names if name not in self._joint_name_to_id]
        if missing:
            raise ValueError(f"R1 articulation is missing target joints {missing}.")
        ids = [self._joint_name_to_id[name] for name in names]
        self.articulation.set_joint_position_target(
            torch.tensor([values], device=self.articulation.device, dtype=torch.float32), joint_ids=ids
        )

    def joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]:
        """Read one name-resolved joint vector from the current simulation state."""

        names = tuple(joint_names)
        missing = [name for name in names if name not in self._joint_name_to_id]
        if missing:
            raise ValueError(f"R1 articulation is missing observed joints {missing}.")
        positions = self.articulation.data.joint_pos[self.env_index]
        return tuple(float(positions[self._joint_name_to_id[name]]) for name in names)


__all__ = [
    "HEAD_JOINT_NAMES",
    "LEFT_ARM_JOINT_NAMES",
    "RIGHT_ARM_JOINT_NAMES",
    "HeadOnlyIsaacLabSink",
    "IsaacLabArticulationHandle",
    "R1SimulatorHandle",
    "VelocityDispatchError",
]
