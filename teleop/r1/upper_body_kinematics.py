"""Coupled R1-A5 upper-body forward kinematics.

The default controlled model is the hardware-common 13-DoF subset:

``waist_yaw + left_arm[5] + right_arm[5] + head_pitch + head_yaw``.

The workstation simulation URDF also exposes ``waist_roll_joint``. Unitree's
R1-A5 controller marks that IDL slot as not used, so by default this model
fixes waist roll at zero instead of silently creating a hardware-incompatible
degree of freedom. The URDF path is explicit at load time because the full
simulation asset and the vendor R1-A5 upper-body asset have different
pelvis-to-waist transforms.

Which torso joints are controlled is selected by a declared body mode; see
`BODY_MODES` and `body_mode_flags`. ``arms_head`` freezes the whole torso and
drives both arms plus the head only, which is what stops the waist being
recruited to chase a hand target the arms alone cannot reach.
``full_upper_body`` adds ``waist_roll_joint`` as a controlled joint instead of
fixing it: a deliberate deviation from the real R1-A5 motor interface, not a
hardware-comparable mode (see `docs/teleop/r1_upper_body_ik.md`), and it
requires a URDF that actually has that joint — the vendor R1-A5 reference asset
does not.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from .kinematics import (
    DEFAULT_URDF,
    ArmChain,
    JointGeometry,
    KinematicsError,
    joint_transform,
    load_arm_chain,
    load_joint_geometry,
)


PELVIS_LINK = "pelvis_link"
UPPER_BODY_JOINT_NAMES = (
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
)
# Declared simulation-only 14-DoF variant: see the module docstring and
# `control_waist_roll` on `load_r1_a5_upper_body_model`.
UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL = ("waist_roll_joint",) + UPPER_BODY_JOINT_NAMES
# Arms and head only: the torso is held at its declared fixed pose. Selecting
# this removes the waist from the task, so it cannot be recruited to chase a
# target the arms alone cannot reach.
ARMS_HEAD_JOINT_NAMES = UPPER_BODY_JOINT_NAMES[1:]  # 12 joints: both arms + head

BODY_MODES = ("arms_head", "waist_yaw", "full_upper_body")
"""Which torso joints the coupled solver owns.

``arms_head``        both arms + head; waist yaw and roll held fixed (12 DoF).
``waist_yaw``        adds waist yaw, the hardware-common R1-A5 set (13 DoF).
``full_upper_body``  adds waist roll as a declared simulation-only deviation
                     (14 DoF); see the module docstring.
"""


def body_mode_flags(body_mode: str) -> tuple[bool, bool]:
    """Return ``(control_waist_yaw, control_waist_roll)`` for a declared mode."""

    if body_mode not in BODY_MODES:
        raise KinematicsError(f"body_mode must be one of {BODY_MODES}, got {body_mode!r}")
    return body_mode != "arms_head", body_mode == "full_upper_body"


def _split_nominal(
    nominal_rad: np.ndarray, body_mode: str, fixed_waist_yaw_rad: float
) -> tuple[float, float, np.ndarray]:
    """Split a mode-specific nominal into ``(waist_roll, waist_yaw, arms_head)``."""

    values = np.asarray(nominal_rad, dtype=float)
    expected = {"arms_head": 12, "waist_yaw": 13, "full_upper_body": 14}[body_mode]
    if values.shape != (expected,):
        raise KinematicsError(
            f"body_mode={body_mode!r} needs a {expected}-vector nominal, got {values.shape}"
        )
    if body_mode == "full_upper_body":
        return float(values[0]), float(values[1]), values[2:]
    if body_mode == "waist_yaw":
        return 0.0, float(values[0]), values[1:]
    # arms_head carries no waist entry, so the frozen yaw is supplied separately.
    return 0.0, float(fixed_waist_yaw_rad), values


def retarget_nominal(
    nominal_rad: np.ndarray,
    from_mode: str,
    to_mode: str,
    fixed_waist_yaw_rad: float = 0.0,
) -> tuple[tuple[float, ...], float]:
    """Convert a declared nominal pose between body modes.

    Returns the nominal for ``to_mode`` and the waist-yaw value to hold when the
    torso is frozen, so selecting a mode at the command line does not require
    editing the experiment profile's nominal vector. The torso's commanded pose
    is preserved across the conversion: freezing the torso holds it at the yaw
    the profile declared rather than snapping it to zero. Waist roll is
    introduced at 0.0 because the hardware-common set carries no roll value.
    """

    for name, mode in (("from_mode", from_mode), ("to_mode", to_mode)):
        if mode not in BODY_MODES:
            raise KinematicsError(f"{name} must be one of {BODY_MODES}, got {mode!r}")
    roll, yaw, arms_head = _split_nominal(nominal_rad, from_mode, fixed_waist_yaw_rad)
    if to_mode == "arms_head":
        return tuple(float(v) for v in arms_head), yaw
    if to_mode == "waist_yaw":
        return (yaw,) + tuple(float(v) for v in arms_head), 0.0
    return (roll, yaw) + tuple(float(v) for v in arms_head), 0.0


def _joint_parent_child(text: str, name: str) -> tuple[str, str]:
    match = re.search(
        r'<joint\s+name="' + re.escape(name) + r'"[^>]*>(.*?)</joint>', text, re.S
    )
    if match is None:
        raise KinematicsError(f"Joint not found in the asset: {name}")
    parent = re.search(r'<parent link="([^"]+)"', match.group(1))
    child = re.search(r'<child link="([^"]+)"', match.group(1))
    if parent is None or child is None:
        raise KinematicsError(f"Joint has no parent/child link declaration: {name}")
    return parent.group(1), child.group(1)


@dataclass(frozen=True)
class UpperBodyFK:
    """Task frames expressed in ``pelvis_link``."""

    pelvis_to_waist_yaw: np.ndarray
    left_end_effector: np.ndarray
    right_end_effector: np.ndarray
    head: np.ndarray


@dataclass(frozen=True)
class R1A5UpperBodyModel:
    """R1-A5 upper-body subset loaded from one URDF.

    Defaults to the hardware-common 13-DoF subset. ``control_waist_roll=True``
    (only settable via `load_r1_a5_upper_body_model`) adds ``waist_roll_joint``
    as a 14th controlled joint, a declared simulation-only deviation; see the
    module docstring.
    """

    urdf_path: Path
    waist_yaw: JointGeometry
    left_arm: ArmChain
    right_arm: ArmChain
    head_pitch: JointGeometry
    head_yaw: JointGeometry
    fixed_waist_roll: JointGeometry | None = None
    control_waist_roll: bool = False
    control_waist_yaw: bool = True
    fixed_waist_yaw_rad: float = 0.0
    """Waist yaw held at this value when ``control_waist_yaw`` is false."""

    def __post_init__(self) -> None:
        if self.control_waist_roll and self.fixed_waist_roll is None:
            raise KinematicsError(
                "control_waist_roll=True requires a URDF with waist_roll_joint; "
                "the loaded asset has none (e.g. the vendor R1-A5 reference)."
            )
        if self.control_waist_roll and not self.control_waist_yaw:
            raise KinematicsError(
                "control_waist_roll=True requires control_waist_yaw=True; "
                "rolling a torso whose yaw is frozen is not a declared mode."
            )
        if not np.isfinite(self.fixed_waist_yaw_rad):
            raise KinematicsError("fixed_waist_yaw_rad must be finite.")

    @property
    def joint_names(self) -> tuple[str, ...]:
        if self.control_waist_roll:
            return UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL
        if self.control_waist_yaw:
            return UPPER_BODY_JOINT_NAMES
        return ARMS_HEAD_JOINT_NAMES

    @property
    def body_mode(self) -> str:
        if self.control_waist_roll:
            return "full_upper_body"
        return "waist_yaw" if self.control_waist_yaw else "arms_head"

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def waist_roll_index(self) -> int | None:
        return 0 if self.control_waist_roll else None

    @property
    def waist_yaw_index(self) -> int | None:
        if not self.control_waist_yaw:
            return None
        return 1 if self.control_waist_roll else 0

    @property
    def left_arm_slice(self) -> slice:
        start = int(self.control_waist_roll) + int(self.control_waist_yaw)
        return slice(start, start + 5)

    @property
    def right_arm_slice(self) -> slice:
        start = self.left_arm_slice.stop
        return slice(start, start + 5)

    @property
    def head_slice(self) -> slice:
        start = self.right_arm_slice.stop
        return slice(start, start + 2)

    def _torso_attribute(self, attribute: str) -> list[float]:
        values: list[float] = []
        if self.control_waist_roll:
            assert self.fixed_waist_roll is not None
            values.append(getattr(self.fixed_waist_roll, attribute))
        if self.control_waist_yaw:
            values.append(getattr(self.waist_yaw, attribute))
        return values

    @property
    def lower_limits(self) -> np.ndarray:
        return np.concatenate(
            (
                self._torso_attribute("lower_rad"),
                self.left_arm.lower_limits,
                self.right_arm.lower_limits,
                [self.head_pitch.lower_rad, self.head_yaw.lower_rad],
            )
        )

    @property
    def upper_limits(self) -> np.ndarray:
        return np.concatenate(
            (
                self._torso_attribute("upper_rad"),
                self.left_arm.upper_limits,
                self.right_arm.upper_limits,
                [self.head_pitch.upper_rad, self.head_yaw.upper_rad],
            )
        )

    @property
    def velocity_limits(self) -> np.ndarray:
        return np.concatenate(
            (
                self._torso_attribute("velocity_radps"),
                self.left_arm.velocity_limits,
                self.right_arm.velocity_limits,
                [self.head_pitch.velocity_radps, self.head_yaw.velocity_radps],
            )
        )

    def clamp(self, q: np.ndarray) -> np.ndarray:
        values = np.asarray(q, dtype=float)
        if values.shape != (self.dof,):
            raise KinematicsError(f"R1-A5 upper body expects {self.dof} joint values, got {values.shape}")
        return np.clip(values, self.lower_limits, self.upper_limits)

    def pelvis_to_waist(self, waist_yaw_rad: float, waist_roll_rad: float = 0.0) -> np.ndarray:
        transform = np.eye(4)
        if self.fixed_waist_roll is not None:
            transform = transform @ joint_transform(self.fixed_waist_roll, waist_roll_rad)
        return transform @ joint_transform(self.waist_yaw, waist_yaw_rad)

    def head_rotation(self, pitch_rad: float, yaw_rad: float) -> np.ndarray:
        """Waist-relative head rotation for a commanded pitch/yaw pair.

        This is the head chain's own forward kinematics, so a target built from
        it is exactly realizable by the two head joints. It must not be replaced
        by a hand-written ``Rz(yaw) @ Ry(pitch)``: the R1 chain applies
        ``head_pitch`` before ``head_yaw``, and the two orders differ whenever
        both angles are non-zero. Deriving the target here also keeps it correct
        if the asset's head axes or joint order ever change.
        """

        transform = joint_transform(self.head_pitch, float(pitch_rad)) @ joint_transform(
            self.head_yaw, float(yaw_rad)
        )
        return transform[:3, :3]

    def waist_transform_from_q(self, q: np.ndarray) -> np.ndarray:
        """`pelvis_to_waist` reading yaw (and roll, when controlled) out of a full-dof vector."""

        values = np.asarray(q, dtype=float)
        if values.shape != (self.dof,):
            raise KinematicsError(f"R1-A5 upper body expects {self.dof} joint values, got {values.shape}")
        yaw_index = self.waist_yaw_index
        yaw = self.fixed_waist_yaw_rad if yaw_index is None else float(values[yaw_index])
        roll_index = self.waist_roll_index
        roll = 0.0 if roll_index is None else float(values[roll_index])
        return self.pelvis_to_waist(yaw, roll)

    def forward_kinematics(self, q: np.ndarray) -> UpperBodyFK:
        values = np.asarray(q, dtype=float)
        if values.shape != (self.dof,):
            raise KinematicsError(f"R1-A5 upper body expects {self.dof} joint values, got {values.shape}")
        waist = self.waist_transform_from_q(values)
        left = waist @ self.left_arm.forward_kinematics(values[self.left_arm_slice])
        right = waist @ self.right_arm.forward_kinematics(values[self.right_arm_slice])
        head_pitch_rad, head_yaw_rad = values[self.head_slice]
        head = (
            waist
            @ joint_transform(self.head_pitch, float(head_pitch_rad))
            @ joint_transform(self.head_yaw, float(head_yaw_rad))
        )
        return UpperBodyFK(waist, left, right, head)


@lru_cache(maxsize=16)
def load_r1_a5_upper_body_model(
    urdf_path: Path | None = None,
    control_waist_roll: bool = False,
    control_waist_yaw: bool = True,
    fixed_waist_yaw_rad: float = 0.0,
) -> R1A5UpperBodyModel:
    """Load the upper-body subset from a full-simulation or vendor R1-A5 URDF.

    ``control_waist_yaw=False`` gives the 11-DoF arms-and-head set, holding the
    torso at ``fixed_waist_yaw_rad``. ``control_waist_roll=True`` loads the
    declared 14-DoF simulation-only variant (see the module docstring) and
    requires an asset that has ``waist_roll_joint``; the vendor R1-A5 reference
    URDF does not, and raises `KinematicsError` if requested there. Prefer
    `body_mode_flags` to derive both booleans from a declared mode name.
    """

    path = (urdf_path or DEFAULT_URDF).resolve()
    if not path.is_file():
        raise KinematicsError(f"R1 URDF not found: {path}")
    text = path.read_text(encoding="utf-8")

    waist_parent, waist_child = _joint_parent_child(text, "waist_yaw_joint")
    if waist_child != "waist_yaw_link":
        raise KinematicsError("waist_yaw_joint does not produce waist_yaw_link.")
    fixed_roll: JointGeometry | None = None
    if waist_parent == "waist_roll_link":
        roll_parent, roll_child = _joint_parent_child(text, "waist_roll_joint")
        if (roll_parent, roll_child) != (PELVIS_LINK, "waist_roll_link"):
            raise KinematicsError("The simulation waist-roll chain is not pelvis_link -> waist_roll_link.")
        fixed_roll = load_joint_geometry("waist_roll_joint", path)
    elif waist_parent != PELVIS_LINK:
        raise KinematicsError(
            f"waist_yaw_joint is rooted at {waist_parent!r}; expected pelvis_link or waist_roll_link."
        )

    pitch_parent, pitch_child = _joint_parent_child(text, "head_pitch_joint")
    yaw_parent, _ = _joint_parent_child(text, "head_yaw_joint")
    if pitch_parent != "waist_yaw_link" or yaw_parent != pitch_child:
        raise KinematicsError("Head chain must be waist_yaw_link -> head_pitch -> head_yaw.")

    return R1A5UpperBodyModel(
        urdf_path=path,
        waist_yaw=load_joint_geometry("waist_yaw_joint", path),
        left_arm=load_arm_chain("left", path),
        right_arm=load_arm_chain("right", path),
        head_pitch=load_joint_geometry("head_pitch_joint", path),
        head_yaw=load_joint_geometry("head_yaw_joint", path),
        fixed_waist_roll=fixed_roll,
        control_waist_roll=control_waist_roll,
        control_waist_yaw=control_waist_yaw,
        fixed_waist_yaw_rad=fixed_waist_yaw_rad,
    )


__all__ = [
    "ARMS_HEAD_JOINT_NAMES",
    "BODY_MODES",
    "PELVIS_LINK",
    "UPPER_BODY_JOINT_NAMES",
    "UPPER_BODY_JOINT_NAMES_WITH_WAIST_ROLL",
    "R1A5UpperBodyModel",
    "UpperBodyFK",
    "body_mode_flags",
    "load_r1_a5_upper_body_model",
    "retarget_nominal",
]
