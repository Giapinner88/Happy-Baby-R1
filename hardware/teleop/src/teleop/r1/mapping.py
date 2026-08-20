"""Frame mapping and ownership rules for simulation-only R1 teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, sin

from .schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3


@dataclass(frozen=True)
class TeleopCalibration:
    translation_m: Vector3 = Vector3(0.0, 0.0, 0.0)
    yaw_rad: float = 0.0
    source_frame: str = "quest_headset"
    robot_frame: str = "r1_base"


@dataclass(frozen=True)
class TeleopLimits:
    command_timeout_s: float
    allow_velocity: bool = False
    max_vx_mps: float = 0.0
    max_vy_mps: float = 0.0
    max_yaw_rate_radps: float = 0.0
    head_yaw_range_rad: tuple[float, float] = (-3.141592653589793, 3.141592653589793)
    head_pitch_range_rad: tuple[float, float] = (-1.5707963267948966, 1.5707963267948966)


@dataclass(frozen=True)
class R1JointOwnership:
    """Joint ownership prevents a locomotion policy and IK from writing the same joint."""

    lower_body: tuple[str, ...] = (
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint",
        "right_ankle_roll_joint", "waist_roll_joint", "waist_yaw_joint",
    )
    upper_body: tuple[str, ...] = (
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
        "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
    )

    def validate(self) -> None:
        overlap = set(self.lower_body).intersection(self.upper_body)
        if overlap:
            raise ValueError(f"R1 teleop joint ownership overlaps: {sorted(overlap)}")


_LEG_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)
_ARMS_HEAD_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)


@dataclass(frozen=True)
class R1A5WholeUpperBodyOwnership:
    """Simulation ownership for the coupled R1-A5 subset, per declared body mode.

    ``arms_head`` leaves the whole torso with the lower-body owner.
    ``waist_yaw`` moves ``waist_yaw_joint`` to coupled IK, the hardware-common
    set. ``full_upper_body`` additionally transfers ``waist_roll_joint`` as a
    declared simulation-only deviation, because the R1-A5 hardware controller
    marks that motor slot as not used.
    """

    body_mode: str = "waist_yaw"

    def __post_init__(self) -> None:
        if self.body_mode not in ("arms_head", "waist_yaw", "full_upper_body"):
            raise ValueError(f"Unknown coupled upper-body body_mode: {self.body_mode!r}")

    @property
    def _torso(self) -> tuple[str, ...]:
        if self.body_mode == "full_upper_body":
            return ("waist_roll_joint", "waist_yaw_joint")
        return ("waist_yaw_joint",) if self.body_mode == "waist_yaw" else ()

    @property
    def lower_body(self) -> tuple[str, ...]:
        held = tuple(j for j in ("waist_roll_joint", "waist_yaw_joint") if j not in self._torso)
        return _LEG_JOINTS + held

    @property
    def upper_body(self) -> tuple[str, ...]:
        return self._torso + _ARMS_HEAD_JOINTS

    def validate(self) -> None:
        overlap = set(self.lower_body).intersection(self.upper_body)
        if overlap:
            raise ValueError(f"R1-A5 coupled upper-body ownership overlaps: {sorted(overlap)}")


@dataclass(frozen=True)
class R1TeleopTargets:
    sequence_id: int
    enabled: bool
    reason: str | None
    left_wrist_target: Pose | None
    right_wrist_target: Pose | None
    head_yaw_rad: float
    head_pitch_rad: float
    base_velocity: BaseVelocity
    base_velocity_enabled: bool
    robot_frame: str


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    return Quaternion(
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    ).normalized()


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    return Quaternion(0.0, 0.0, sin(yaw_rad / 2.0), cos(yaw_rad / 2.0))


def _transform_pose(pose: Pose, calibration: TeleopCalibration) -> Pose:
    c, s = cos(calibration.yaw_rad), sin(calibration.yaw_rad)
    position = pose.position
    rotated = Vector3(c * position.x - s * position.y, s * position.x + c * position.y, position.z)
    translated = Vector3(
        rotated.x + calibration.translation_m.x,
        rotated.y + calibration.translation_m.y,
        rotated.z + calibration.translation_m.z,
    )
    return Pose(translated, _quaternion_multiply(_yaw_quaternion(calibration.yaw_rad), pose.orientation))


def _yaw_pitch(pose: Pose) -> tuple[float, float]:
    q = pose.orientation.normalized()
    yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = asin(_clamp(sin_pitch, -1.0, 1.0))
    return yaw, pitch


class R1TeleopMapper:
    def __init__(self, calibration: TeleopCalibration, limits: TeleopLimits, ownership: R1JointOwnership | None = None):
        if limits.command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive.")
        self.calibration = calibration
        self.limits = limits
        self.ownership = ownership or R1JointOwnership()
        self.ownership.validate()

    def map(self, command: R1TeleopCommand, received_monotonic_s: float) -> R1TeleopTargets:
        if command.source_frame != self.calibration.source_frame:
            return self._disabled(command.sequence_id, "source_frame_mismatch")
        if received_monotonic_s - command.timestamp_monotonic_s > self.limits.command_timeout_s:
            return self._disabled(command.sequence_id, "command_timeout")
        if not command.deadman_enabled:
            return self._disabled(command.sequence_id, "deadman_released")

        head_pose = _transform_pose(command.head_pose, self.calibration)
        head_yaw, head_pitch = _yaw_pitch(head_pose)
        velocity = command.base_velocity if self.limits.allow_velocity else BaseVelocity.zero()
        velocity = BaseVelocity(
            _clamp(velocity.vx_mps, -self.limits.max_vx_mps, self.limits.max_vx_mps),
            _clamp(velocity.vy_mps, -self.limits.max_vy_mps, self.limits.max_vy_mps),
            _clamp(velocity.yaw_rate_radps, -self.limits.max_yaw_rate_radps, self.limits.max_yaw_rate_radps),
        )
        return R1TeleopTargets(
            sequence_id=command.sequence_id,
            enabled=True,
            reason=None,
            left_wrist_target=_transform_pose(command.left_wrist_pose, self.calibration),
            right_wrist_target=_transform_pose(command.right_wrist_pose, self.calibration),
            head_yaw_rad=_clamp(head_yaw, *self.limits.head_yaw_range_rad),
            head_pitch_rad=_clamp(head_pitch, *self.limits.head_pitch_range_rad),
            base_velocity=velocity,
            base_velocity_enabled=self.limits.allow_velocity,
            robot_frame=self.calibration.robot_frame,
        )

    def _disabled(self, sequence_id: int, reason: str) -> R1TeleopTargets:
        return R1TeleopTargets(
            sequence_id=sequence_id,
            enabled=False,
            reason=reason,
            left_wrist_target=None,
            right_wrist_target=None,
            head_yaw_rad=0.0,
            head_pitch_rad=0.0,
            base_velocity=BaseVelocity.zero(),
            base_velocity_enabled=False,
            robot_frame=self.calibration.robot_frame,
        )
yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))