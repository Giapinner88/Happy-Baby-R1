"""Versioned, transport-neutral R1 Quest teleoperation command schema.

All positions are metres, angles are radians, and timestamps use a monotonic
clock in seconds. The schema intentionally contains no DDS types.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Any


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(float(value)) for value in (self.x, self.y, self.z)):
            raise ValueError("Vector3 values must be finite.")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Vector3":
        return cls(float(value["x"]), float(value["y"]), float(value["z"]))

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        if not all(isfinite(float(value)) for value in (self.x, self.y, self.z, self.w)):
            raise ValueError("Quaternion values must be finite.")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Quaternion":
        return cls(float(value["x"]), float(value["y"]), float(value["z"]), float(value["w"]))

    def normalized(self) -> "Quaternion":
        norm = sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
        if not isfinite(norm) or norm <= 1e-12:
            raise ValueError("Quaternion norm must be non-zero.")
        return Quaternion(self.x / norm, self.y / norm, self.z / norm, self.w / norm)

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}


@dataclass(frozen=True)
class Pose:
    position: Vector3
    orientation: Quaternion

    def __post_init__(self) -> None:
        if not isinstance(self.position, Vector3) or not isinstance(self.orientation, Quaternion):
            raise TypeError("Pose requires Vector3 position and Quaternion orientation.")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Pose":
        return cls(Vector3.from_dict(value["position"]), Quaternion.from_dict(value["orientation"]).normalized())

    def as_dict(self) -> dict[str, dict[str, float]]:
        return {"position": self.position.as_dict(), "orientation": self.orientation.as_dict()}


@dataclass(frozen=True)
class BaseVelocity:
    vx_mps: float
    vy_mps: float
    yaw_rate_radps: float

    def __post_init__(self) -> None:
        if not all(
            isfinite(float(value))
            for value in (self.vx_mps, self.vy_mps, self.yaw_rate_radps)
        ):
            raise ValueError("Base velocity values must be finite.")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BaseVelocity":
        return cls(float(value["vx_mps"]), float(value["vy_mps"]), float(value["yaw_rate_radps"]))

    def as_dict(self) -> dict[str, float]:
        return {
            "vx_mps": self.vx_mps,
            "vy_mps": self.vy_mps,
            "yaw_rate_radps": self.yaw_rate_radps,
        }

    @classmethod
    def zero(cls) -> "BaseVelocity":
        return cls(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class R1TeleopCommand:
    """A normalized Quest command in the headset source frame."""

    sequence_id: int
    timestamp_monotonic_s: float
    deadman_enabled: bool
    head_pose: Pose
    left_wrist_pose: Pose
    right_wrist_pose: Pose
    base_velocity: BaseVelocity
    source_frame: str = "quest_headset"
    schema_version: int = SCHEMA_VERSION
    reset_requested: bool = False

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError("Unsupported R1 teleop command schema version.")
        if self.sequence_id < 0:
            raise ValueError("sequence_id must be non-negative.")
        if not isfinite(float(self.timestamp_monotonic_s)) or self.timestamp_monotonic_s < 0.0:
            raise ValueError("timestamp_monotonic_s must be finite and non-negative.")
        if not self.source_frame.strip():
            raise ValueError("source_frame must be non-empty.")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "R1TeleopCommand":
        if int(value.get("schema_version", SCHEMA_VERSION)) != SCHEMA_VERSION:
            raise ValueError("Unsupported R1 teleop command schema version.")
        command = cls(
            sequence_id=int(value["sequence_id"]),
            timestamp_monotonic_s=float(value["timestamp_monotonic_s"]),
            deadman_enabled=bool(value["deadman_enabled"]),
            head_pose=Pose.from_dict(value["head_pose"]),
            left_wrist_pose=Pose.from_dict(value["left_wrist_pose"]),
            right_wrist_pose=Pose.from_dict(value["right_wrist_pose"]),
            base_velocity=BaseVelocity.from_dict(value["base_velocity"]),
            source_frame=str(value.get("source_frame", "quest_headset")),
            reset_requested=bool(value.get("reset_requested", False)),
        )
        return command

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "sequence_id": self.sequence_id,
            "timestamp_monotonic_s": self.timestamp_monotonic_s,
            "deadman_enabled": self.deadman_enabled,
            "source_frame": self.source_frame,
            "head_pose": self.head_pose.as_dict(),
            "left_wrist_pose": self.left_wrist_pose.as_dict(),
            "right_wrist_pose": self.right_wrist_pose.as_dict(),
            "base_velocity": self.base_velocity.as_dict(),
            "reset_requested": self.reset_requested,
        }
