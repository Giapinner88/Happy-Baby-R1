"""Forward kinematics for the R1 arm chains, read from the URDF.

The chain geometry is parsed from `assets/R1.urdf` rather than transcribed into
code, so an asset change cannot silently invalidate a workspace measured against
it. Two of the five joint origins carry a non-zero `rpy` (±0.26187 rad at the
shoulder), which a hand-copied translation-only chain would drop.

Conventions follow `docs/teleop/r1_arm_wrist_ik.md`: metres, radians, and the
URDF joint axes. The endpoint is `*_wrist_roll_link` and the chain root is
`waist_yaw_link`, so every pose here is expressed in the waist frame.  For the
R1-A5, the controlled end-effector is the vendor-declared virtual frame 0.20 m
along the wrist-roll link's local x axis, not the wrist joint origin.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPO_ROOT / "assets" / "R1.urdf"

ARM_JOINT_ORDER = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll")
CHAIN_ROOT_LINK = "waist_yaw_link"
R1_A5_END_EFFECTOR_OFFSET_M = np.array([0.20, 0.0, 0.0], dtype=float)


class KinematicsError(RuntimeError):
    """Raised when the asset cannot supply a usable arm chain."""


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis XYZ convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""

    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def joint_transform(joint: "JointGeometry", angle_rad: float) -> np.ndarray:
    """Homogeneous parent-to-child transform for one revolute URDF joint."""

    transform = np.eye(4)
    transform[:3, :3] = joint.origin_rotation @ axis_angle_to_matrix(joint.axis, float(angle_rad))
    transform[:3, 3] = joint.origin_translation
    return transform


@dataclass(frozen=True)
class JointGeometry:
    """One revolute joint: its fixed origin and its rotation axis."""

    name: str
    origin_translation: np.ndarray  # (3,)
    origin_rotation: np.ndarray  # (3, 3)
    axis: np.ndarray  # (3,) unit
    lower_rad: float
    upper_rad: float
    effort_nm: float
    velocity_radps: float


@dataclass(frozen=True)
class ArmChain:
    """A five-joint R1 arm from `waist_yaw_link` to `*_wrist_roll_link`."""

    side: str
    joints: tuple[JointGeometry, ...]
    end_effector_offset_m: tuple[float, float, float] = (0.20, 0.0, 0.0)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def lower_limits(self) -> np.ndarray:
        return np.array([joint.lower_rad for joint in self.joints])

    @property
    def upper_limits(self) -> np.ndarray:
        return np.array([joint.upper_rad for joint in self.joints])

    @property
    def velocity_limits(self) -> np.ndarray:
        return np.array([joint.velocity_radps for joint in self.joints])

    @property
    def max_reach_from_shoulder_m(self) -> float:
        """Upper bound on the endpoint's distance from the shoulder origin.

        Triangle inequality over the link translations below the shoulder plus
        the tool offset, so it is conservative by construction and exact to
        compute: no posture can place the endpoint farther than this. It is
        derived from the asset rather than declared, because it is a property of
        the geometry and not a tuning choice. A target beyond it is definitely
        unreachable, which lets a caller tell "the solver is stuck" apart from
        "the operator reached past the arm".
        """

        links = sum(float(np.linalg.norm(joint.origin_translation)) for joint in self.joints[1:])
        return links + float(np.linalg.norm(self.end_effector_offset_m))

    def shoulder_origin(self) -> np.ndarray:
        """Shoulder position in the chain's root frame."""

        return np.asarray(self.joints[0].origin_translation, dtype=float)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.lower_limits, self.upper_limits)

    def link_transforms(self, q: np.ndarray) -> list[np.ndarray]:
        """Cumulative 4x4 transforms after each joint, in the waist frame."""

        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise KinematicsError(f"{self.side} arm expects {self.dof} joint values, got {q.shape}")
        transform = np.eye(4)
        transforms: list[np.ndarray] = []
        for joint, angle in zip(self.joints, q):
            transform = transform @ joint_transform(joint, float(angle))
            transforms.append(transform.copy())
        return transforms

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Vendor R1-A5 virtual end-effector pose in the waist frame."""

        wrist = self.link_transforms(q)[-1].copy()
        wrist[:3, 3] += wrist[:3, :3] @ np.asarray(self.end_effector_offset_m, dtype=float)
        return wrist

    def endpoint_position(self, q: np.ndarray) -> np.ndarray:
        return self.forward_kinematics(q)[:3, 3]

    def wrist_roll(self, q: np.ndarray) -> float:
        """Commanded wrist-roll angle.

        The wrist-roll joint is the last in the chain and its own coordinate is
        the roll, so this is the joint value itself. It is exposed as a method so
        callers state the intent rather than indexing, and so a future asset with
        a different wrist can change it in one place.
        """

        return float(np.asarray(q, dtype=float)[-1])

    def position_jacobian(self, q: np.ndarray) -> np.ndarray:
        """(3, dof) geometric Jacobian of the endpoint position.

        For a revolute joint, d(endpoint)/d(theta) = axis_world x (endpoint - origin_world).
        """

        transforms = self.link_transforms(q)
        # The controlled point is the vendor virtual EE, not the wrist joint
        # origin.  Using the wrist origin here while FK uses the tool offset
        # gives a self-inconsistent Jacobian and makes reachable extension
        # targets appear to stall centimetres away from the goal.
        endpoint = self.endpoint_position(q)
        jacobian = np.zeros((3, self.dof))
        parent = np.eye(4)
        for index, (joint, transform) in enumerate(zip(self.joints, transforms)):
            fixed = np.eye(4)
            fixed[:3, :3] = joint.origin_rotation
            fixed[:3, 3] = joint.origin_translation
            joint_frame = parent @ fixed
            axis_world = joint_frame[:3, :3] @ joint.axis
            origin_world = joint_frame[:3, 3]
            jacobian[:, index] = np.cross(axis_world, endpoint - origin_world)
            parent = transform
        return jacobian


def _parse_joint(text: str, name: str) -> JointGeometry:
    match = re.search(r'<joint\s+name="' + re.escape(name) + r'"[^>]*type="revolute">(.*?)</joint>', text, re.S)
    if match is None:
        raise KinematicsError(f"Revolute joint not found in the asset: {name}")
    body = match.group(1)

    origin = re.search(r"<origin([^/>]*)/?>", body)
    attributes = dict(re.findall(r'(\w+)="([^"]+)"', origin.group(1))) if origin else {}
    translation = np.array([float(value) for value in attributes.get("xyz", "0 0 0").split()])
    roll, pitch, yaw = (float(value) for value in attributes.get("rpy", "0 0 0").split())

    axis_match = re.search(r'<axis xyz="([^"]+)"', body)
    if axis_match is None:
        raise KinematicsError(f"Joint has no axis: {name}")
    axis = np.array([float(value) for value in axis_match.group(1).split()])
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        raise KinematicsError(f"Joint axis has zero length: {name}")

    limit = re.search(r"<limit([^/]*)/>", body)
    limits = dict(re.findall(r'(\w+)="([^"]+)"', limit.group(1))) if limit else {}
    if "lower" not in limits or "upper" not in limits:
        raise KinematicsError(f"Joint has no position limits: {name}")

    return JointGeometry(
        name=name,
        origin_translation=translation,
        origin_rotation=_rpy_to_matrix(roll, pitch, yaw),
        axis=axis / norm,
        lower_rad=float(limits["lower"]),
        upper_rad=float(limits["upper"]),
        effort_nm=float(limits.get("effort", 0.0)),
        velocity_radps=float(limits.get("velocity", 0.0)),
    )


def load_joint_geometry(name: str, urdf_path: Path | None = None) -> JointGeometry:
    """Load one revolute joint from an R1 URDF.

    Upper-body kinematics needs the waist and head joints in addition to the
    five-joint arm chains.  Keeping their parsing here gives every solver the
    same URDF origin/axis/limit semantics instead of transcribing another model.
    """

    path = urdf_path or DEFAULT_URDF
    if not path.is_file():
        raise KinematicsError(f"R1 URDF not found: {path}")
    return _parse_joint(path.read_text(encoding="utf-8"), name)


@lru_cache(maxsize=8)
def load_arm_chain(
    side: str,
    urdf_path: Path | None = None,
    end_effector_offset_m: tuple[float, float, float] = (0.20, 0.0, 0.0),
) -> ArmChain:
    """Parse one arm chain from the asset.

    The current R1-A5 method uses the vendor's 0.20 m virtual tool frame.  The
    explicit argument exists only so schema-1 T002--T006 records, which used the
    wrist-joint origin, remain reproducible without changing the active T007
    model.  It is a tuple (rather than an array) so cached calls are hashable.
    """

    if side not in ("left", "right"):
        raise KinematicsError(f"side must be 'left' or 'right', got {side!r}")
    if len(end_effector_offset_m) != 3 or not np.all(np.isfinite(end_effector_offset_m)):
        raise KinematicsError("end_effector_offset_m must be a finite 3-vector.")
    path = urdf_path or DEFAULT_URDF
    if not path.is_file():
        raise KinematicsError(f"R1 URDF not found: {path}")
    text = path.read_text(encoding="utf-8")

    first = f"{side}_shoulder_pitch_joint"
    parent = re.search(
        r'<joint\s+name="' + re.escape(first) + r'"[^>]*>.*?<parent link="([^"]+)"', text, re.S
    )
    if parent is None or parent.group(1) != CHAIN_ROOT_LINK:
        found = parent.group(1) if parent else "none"
        raise KinematicsError(
            f"{first} is rooted at {found!r}, not {CHAIN_ROOT_LINK!r}; the method's frame no longer holds."
        )

    return ArmChain(
        side=side,
        joints=tuple(_parse_joint(text, f"{side}_{joint}_joint") for joint in ARM_JOINT_ORDER),
        end_effector_offset_m=tuple(float(value) for value in end_effector_offset_m),
    )


__all__ = [
    "ARM_JOINT_ORDER",
    "CHAIN_ROOT_LINK",
    "R1_A5_END_EFFECTOR_OFFSET_M",
    "ArmChain",
    "JointGeometry",
    "KinematicsError",
    "axis_angle_to_matrix",
    "joint_transform",
    "load_arm_chain",
    "load_joint_geometry",
]
