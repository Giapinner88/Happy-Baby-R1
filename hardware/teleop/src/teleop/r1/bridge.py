"""Transport-neutral conversion from Quest XR telemetry to R1 teleop commands.

This module is the reusable half of the T001 live bridge. It deliberately does
not import the Quest vendor wrapper, a simulator, DDS, ROS, or hardware code, so
it can be unit tested in any environment. The vendor-specific process feeds it
plain sample values and receives normalized `R1TeleopCommand` objects.

Conventions follow `schema.py`: metres, radians, and a monotonic clock in
seconds. Pose matrices are 4x4 row-major sequences in the vendor wrapper's
already-converted robot basis; this module changes no basis of its own.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt
from typing import Sequence

from .schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3


Matrix4x4 = Sequence[Sequence[float]]


class BridgeError(ValueError):
    """Raised when a Quest sample cannot be normalized into a command."""


@dataclass(frozen=True)
class QuestTransportSample:
    """One vendor telemetry sample, decoupled from the vendor wrapper type.

    `motion_data_ready` mirrors the vendor flag that only becomes true once an
    immersive XR session has delivered a controller or hand event. A sample with
    the flag false carries wrapper defaults, not observed motion, so the bridge
    treats it as "not connected" rather than as a neutral pose.
    """

    motion_data_ready: bool
    head_pose_matrix: Matrix4x4
    left_wrist_pose_matrix: Matrix4x4
    right_wrist_pose_matrix: Matrix4x4
    deadman_pressed: bool
    reset_requested: bool = False


def _validate_matrix(matrix: Matrix4x4, name: str) -> list[list[float]]:
    rows = [list(row) for row in matrix]
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise BridgeError(f"{name} must be a 4x4 matrix.")
    values = [float(value) for row in rows for value in row]
    if not all(isfinite(value) for value in values):
        raise BridgeError(f"{name} contains a non-finite value.")
    return [values[0:4], values[4:8], values[8:12], values[12:16]]


def rotation_matrix_to_quaternion(matrix: Matrix4x4) -> Quaternion:
    """Convert the rotation block of a 4x4 pose matrix to a unit quaternion.

    Uses Shepperd's method: pivot on the largest of the four candidate
    denominators so the division never approaches zero.
    """

    rows = _validate_matrix(matrix, "pose matrix")
    m00, m01, m02 = rows[0][0], rows[0][1], rows[0][2]
    m10, m11, m12 = rows[1][0], rows[1][1], rows[1][2]
    m20, m21, m22 = rows[2][0], rows[2][1], rows[2][2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        quaternion = Quaternion(
            (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale
        )
    elif m00 > m11 and m00 > m22:
        scale = sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = Quaternion(
            0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale
        )
    elif m11 > m22:
        scale = sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = Quaternion(
            (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale
        )
    else:
        scale = sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = Quaternion(
            (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale
        )
    return quaternion.normalized()


def pose_from_matrix(matrix: Matrix4x4) -> Pose:
    """Read the translation column and rotation block of a 4x4 pose matrix."""

    rows = _validate_matrix(matrix, "pose matrix")
    position = Vector3(rows[0][3], rows[1][3], rows[2][3])
    return Pose(position, rotation_matrix_to_quaternion(rows))


def _orthonormality_defect(matrix: Matrix4x4) -> float:
    """Largest deviation of the rotation block from an orthonormal basis."""

    rows = _validate_matrix(matrix, "pose matrix")
    basis = [row[0:3] for row in rows[0:3]]
    defect = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(basis[i][k] * basis[j][k] for k in range(3))
            defect = max(defect, abs(dot - (1.0 if i == j else 0.0)))
    return defect


def _rotation_determinant(matrix: Matrix4x4) -> float:
    """Return det(R) for the 3x3 rotation block of a validated pose matrix."""

    rows = _validate_matrix(matrix, "pose matrix")
    m00, m01, m02 = rows[0][0], rows[0][1], rows[0][2]
    m10, m11, m12 = rows[1][0], rows[1][1], rows[1][2]
    m20, m21, m22 = rows[2][0], rows[2][1], rows[2][2]
    return m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20)


@dataclass
class BridgeConnectionState:
    """Connection bookkeeping the runner records as T001 connection evidence."""

    connected: bool = False
    connect_count: int = 0
    disconnect_count: int = 0
    dropped_sample_count: int = 0
    rejected_sample_count: int = 0
    transitions: list[dict[str, object]] = field(default_factory=list)

    def _record(self, event: str, timestamp_monotonic_s: float, sequence_id: int, detail: str | None) -> None:
        self.transitions.append(
            {
                "event": event,
                "timestamp_monotonic_s": timestamp_monotonic_s,
                "sequence_id": sequence_id,
                "detail": detail,
            }
        )


@dataclass(frozen=True)
class BridgeConfig:
    """Declared bridge behaviour; every field is recorded in run provenance.

    `max_orthonormality_defect` and `max_rotation_determinant_error` reject a
    pose whose rotation block is not a proper rotation. `base_velocity_source` is fixed to a constant zero for v1 because
    no reviewed configuration supplies R1 velocity limits, and inventing one
    would put an unaudited number into the command stream.

    `max_pose_stale_s` is what actually detects a lost headset. The vendor's
    `motion_data_ready` flag latches: it is set true on the first controller or
    hand event and is never cleared, so it reports "has ever delivered motion",
    not "is delivering motion now". A removed headset therefore keeps producing
    samples that look ready while carrying the last observed pose. Treating
    bit-identical repeated poses as a disconnect is the only observable that
    distinguishes the two.

    The 0.5 s default is bounded by measurement rather than taste. In run
    `t001_b_20260802T111348Z` the longest frozen stretch during genuine
    operation was 0.2 s, because real head tracking never repeats a pose
    bit-for-bit, while removing the headset froze the pose for 5.9 s. Any
    threshold between those is safe; 0.5 s also matches the mapper's
    `command_timeout_s`, so the two fail-closed layers agree on what "stale"
    means.
    """

    source_frame: str = "quest_headset"
    max_orthonormality_defect: float = 1e-3
    max_rotation_determinant_error: float = 1e-3
    base_velocity_source: str = "constant_zero"
    max_pose_stale_s: float = 0.5


class QuestCommandBridge:
    """Turns Quest transport samples into a strictly increasing command stream.

    The bridge is fail-closed at its own boundary: a sample that is not
    motion-ready, whose pose matrices are unusable, or whose pose has stopped
    changing produces no command at all. Downstream `R1TeleopMapper` then
    observes a receipt gap and applies its own `command_timeout` hold, so a
    dropped Quest session cannot leave a stale command driving the simulator.

    Emitting a command for a frozen pose is the failure this guards against: it
    refreshes the timestamp, so the mapper's timeout never fires either and the
    simulator keeps tracking a pose the operator is no longer producing.
    """

    def __init__(self, config: BridgeConfig | None = None) -> None:
        self.config = config or BridgeConfig()
        if self.config.max_orthonormality_defect <= 0.0 or self.config.max_rotation_determinant_error <= 0.0:
            raise ValueError("Rotation validation tolerances must be positive.")
        if self.config.base_velocity_source != "constant_zero":
            raise ValueError("R1 teleop v1 only permits a constant-zero base velocity source.")
        if self.config.max_pose_stale_s <= 0.0:
            raise ValueError("max_pose_stale_s must be positive.")
        self.state = BridgeConnectionState()
        self._next_sequence_id = 0
        self._last_pose_key: tuple[float, ...] | None = None
        self._last_pose_change_s: float | None = None

    @staticmethod
    def _pose_key(sample: QuestTransportSample) -> tuple[float, ...]:
        return tuple(
            float(value)
            for matrix in (sample.head_pose_matrix, sample.left_wrist_pose_matrix, sample.right_wrist_pose_matrix)
            for row in matrix
            for value in row
        )

    def _pose_is_stale(self, sample: QuestTransportSample, timestamp_monotonic_s: float) -> bool:
        """True when the pose has not changed for longer than the declared limit."""

        key = self._pose_key(sample)
        if key != self._last_pose_key:
            self._last_pose_key = key
            self._last_pose_change_s = timestamp_monotonic_s
            return False
        if self._last_pose_change_s is None:
            self._last_pose_change_s = timestamp_monotonic_s
            return False
        return timestamp_monotonic_s - self._last_pose_change_s > self.config.max_pose_stale_s

    def build(self, sample: QuestTransportSample, timestamp_monotonic_s: float) -> R1TeleopCommand | None:
        """Return the next command, or None when the sample carries no motion.

        A None result is a connection observation, not an error: the caller
        records it and lets the mapper's timeout decide when to hold.
        """

        if not isfinite(timestamp_monotonic_s) or timestamp_monotonic_s < 0.0:
            raise BridgeError("timestamp_monotonic_s must be finite and non-negative.")

        if not sample.motion_data_ready:
            self.state.dropped_sample_count += 1
            if self.state.connected:
                self.state.connected = False
                self.state.disconnect_count += 1
                self.state._record(
                    "disconnected", timestamp_monotonic_s, self._next_sequence_id, "motion_data_ready cleared"
                )
            return None

        matrices = (
            ("head_pose", sample.head_pose_matrix),
            ("left_wrist_pose", sample.left_wrist_pose_matrix),
            ("right_wrist_pose", sample.right_wrist_pose_matrix),
        )
        for name, matrix in matrices:
            try:
                defect = _orthonormality_defect(matrix)
                determinant = _rotation_determinant(matrix)
            except BridgeError as exc:
                self.state.rejected_sample_count += 1
                self.state._record("rejected", timestamp_monotonic_s, self._next_sequence_id, f"{name}: {exc}")
                return None
            if defect > self.config.max_orthonormality_defect:
                self.state.rejected_sample_count += 1
                self.state._record(
                    "rejected",
                    timestamp_monotonic_s,
                    self._next_sequence_id,
                    f"{name} rotation block defect {defect:.3e} exceeds {self.config.max_orthonormality_defect:.3e}",
                )
                return None
            if abs(determinant - 1.0) > self.config.max_rotation_determinant_error:
                self.state.rejected_sample_count += 1
                self.state._record(
                    "rejected",
                    timestamp_monotonic_s,
                    self._next_sequence_id,
                    (
                        f"{name} rotation determinant {determinant:.3e} differs from +1 by more than "
                        f"{self.config.max_rotation_determinant_error:.3e}"
                    ),
                )
                return None

        # Staleness is checked after the rotation checks so a malformed sample is
        # reported as rejected rather than being mistaken for a frozen pose.
        if not sample.reset_requested and self._pose_is_stale(sample, timestamp_monotonic_s):
            self.state.dropped_sample_count += 1
            if self.state.connected:
                self.state.connected = False
                self.state.disconnect_count += 1
                self.state._record(
                    "disconnected",
                    timestamp_monotonic_s,
                    self._next_sequence_id,
                    f"pose unchanged for more than {self.config.max_pose_stale_s:g} s",
                )
            return None

        if not self.state.connected:
            self.state.connected = True
            self.state.connect_count += 1
            detail = "first motion-ready sample" if self.state.connect_count == 1 else "pose changed again"
            self.state._record("connected", timestamp_monotonic_s, self._next_sequence_id, detail)

        command = R1TeleopCommand(
            sequence_id=self._next_sequence_id,
            timestamp_monotonic_s=timestamp_monotonic_s,
            deadman_enabled=bool(sample.deadman_pressed),
            head_pose=pose_from_matrix(sample.head_pose_matrix),
            left_wrist_pose=pose_from_matrix(sample.left_wrist_pose_matrix),
            right_wrist_pose=pose_from_matrix(sample.right_wrist_pose_matrix),
            base_velocity=BaseVelocity.zero(),
            source_frame=self.config.source_frame,
            reset_requested=bool(sample.reset_requested),
        )
        self._next_sequence_id += 1
        return command


__all__ = [
    "BridgeConfig",
    "BridgeConnectionState",
    "BridgeError",
    "QuestCommandBridge",
    "QuestTransportSample",
    "pose_from_matrix",
    "rotation_matrix_to_quaternion",
]
