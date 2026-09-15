"""Live differential Cartesian controller for the simulation-only R1 path."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter_ns
from typing import Protocol, Sequence

import numpy as np

from .differential_tracking import DifferentialTrackingConfig, DifferentialUpperBodyTracker
from .mapping import R1TeleopTargets
from .upper_body_ik import UpperBodyIKTarget, quaternion_xyzw_to_matrix, so3_log
from .upper_body_kinematics import BODY_MODES, body_mode_flags, load_r1_a5_upper_body_model
from .workspace_projection import project_upper_body_target


class DifferentialSimulatorHandle(Protocol):
    def write_joint_targets(
        self, joint_names: Sequence[str], positions_rad: Sequence[float]
    ) -> None: ...

    def joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]: ...


def _pose_transform(pose: object) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    transform[:3, :3] = quaternion_xyzw_to_matrix(
        np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    )
    return transform


@dataclass(frozen=True)
class DifferentialWholeUpperBodyLiveConfig:
    urdf_path: Path
    nominal_joint_position_rad: tuple[float, ...]
    tracker: DifferentialTrackingConfig
    source_target_frame: str = "neutral_waist_yaw_link"
    body_mode: str = "arms_head"
    fixed_waist_yaw_rad: float = 0.0
    workspace_projection_margin_m: float = 0.01
    mapping_mode: str = "relative_session"
    position_scale: float = 0.75
    velocity_feedforward_task: str = "head_only"
    source_rate_hz: float = 30.0
    velocity_filter_alpha: float = 0.35

    def validate(self) -> None:
        self.tracker.validate()
        if self.source_target_frame != "neutral_waist_yaw_link":
            raise ValueError(
                "Differential live control only supports neutral_waist_yaw_link targets."
            )
        if self.body_mode not in BODY_MODES:
            raise ValueError(f"Unknown body_mode: {self.body_mode}")
        expected = {"arms_head": 12, "waist_yaw": 13, "full_upper_body": 14}[
            self.body_mode
        ]
        nominal = np.asarray(self.nominal_joint_position_rad, dtype=float)
        if nominal.shape != (expected,) or not np.all(np.isfinite(nominal)):
            raise ValueError(
                f"body_mode={self.body_mode!r} requires a finite {expected}-vector nominal."
            )
        if not np.isfinite(self.workspace_projection_margin_m) or not (
            0.0 <= self.workspace_projection_margin_m < 0.2
        ):
            raise ValueError("workspace_projection_margin_m must be in [0, 0.2) m.")
        if self.mapping_mode not in ("absolute", "relative_session"):
            raise ValueError("mapping_mode must be absolute or relative_session.")
        if not np.isfinite(self.position_scale) or self.position_scale <= 0.0:
            raise ValueError("position_scale must be finite and positive.")
        if self.velocity_feedforward_task not in (
            "none",
            "head_only",
            "wrist_position_head",
            "all",
        ):
            raise ValueError(
                "velocity_feedforward_task must be none, head_only, "
                "wrist_position_head, or all."
            )
        if not np.isfinite(self.source_rate_hz) or self.source_rate_hz <= 0.0:
            raise ValueError("source_rate_hz must be finite and positive.")
        if not np.isfinite(self.velocity_filter_alpha) or not (
            0.0 < self.velocity_filter_alpha <= 1.0
        ):
            raise ValueError("velocity_filter_alpha must be in (0, 1].")


@dataclass
class DifferentialWholeUpperBodyIsaacLabSink:
    """One-step weighted-DLS tracking with pre-solve wrist projection."""

    handle: DifferentialSimulatorHandle
    config: DifferentialWholeUpperBodyLiveConfig
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    last_application: dict[str, object] | None = None
    last_target: np.ndarray | None = None
    session_started: bool = False
    arm_targets_withheld: int = 0

    def __post_init__(self) -> None:
        self.config.validate()
        control_yaw, control_roll = body_mode_flags(self.config.body_mode)
        self.model = load_r1_a5_upper_body_model(
            self.config.urdf_path,
            control_waist_roll=control_roll,
            control_waist_yaw=control_yaw,
            fixed_waist_yaw_rad=self.config.fixed_waist_yaw_rad,
        )
        if self.config.tracker.max_joint_velocity_rad_s > float(
            np.min(self.model.velocity_limits)
        ):
            raise ValueError(
                "Differential velocity limit exceeds the slowest selected URDF joint."
            )
        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        measured = np.asarray(
            self.handle.joint_positions(self.model.joint_names), dtype=float
        )
        self.tracker = DifferentialUpperBodyTracker(
            self.model, nominal, measured, self.config.tracker
        )
        self.last_target = self.tracker.q_reference.copy()
        self._previous_task_target: UpperBodyIKTarget | None = None
        self._previous_sequence: int | None = None
        self._filtered_task_velocity = np.zeros(15, dtype=float)
        self._source_origin: UpperBodyIKTarget | None = None
        self._robot_origin: UpperBodyIKTarget | None = None

    def _build_target(self, targets: R1TeleopTargets) -> UpperBodyIKTarget:
        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        neutral_waist = self.model.waist_transform_from_q(nominal)
        left = neutral_waist @ _pose_transform(targets.left_wrist_target)
        right = neutral_waist @ _pose_transform(targets.right_wrist_target)
        head = neutral_waist[:3, :3] @ self.model.head_rotation(
            targets.head_pitch_rad, targets.head_yaw_rad
        )
        return UpperBodyIKTarget(
            left[:3, 3], left[:3, :3], right[:3, 3], right[:3, :3], head
        )

    def _task_velocity(
        self, sequence_id: int, target: UpperBodyIKTarget
    ) -> np.ndarray:
        raw = np.zeros(15, dtype=float)
        if self._previous_task_target is not None and self._previous_sequence is not None:
            sequence_delta = max(1, sequence_id - self._previous_sequence)
            dt = sequence_delta / self.config.source_rate_hz
            if self.config.velocity_feedforward_task in (
                "wrist_position_head",
                "all",
            ):
                raw[0:3] = (
                    target.left_position_m - self._previous_task_target.left_position_m
                ) / dt
                raw[3:6] = (
                    target.right_position_m - self._previous_task_target.right_position_m
                ) / dt
                if self.config.velocity_feedforward_task == "all":
                    raw[6:9] = so3_log(
                        target.left_orientation
                        @ self._previous_task_target.left_orientation.T
                    ) / dt
                    raw[9:12] = so3_log(
                        target.right_orientation
                        @ self._previous_task_target.right_orientation.T
                    ) / dt
            if self.config.velocity_feedforward_task in (
                "head_only",
                "wrist_position_head",
                "all",
            ):
                raw[12:15] = so3_log(
                    target.head_orientation
                    @ self._previous_task_target.head_orientation.T
                ) / dt
        alpha = self.config.velocity_filter_alpha
        self._filtered_task_velocity = (
            alpha * raw + (1.0 - alpha) * self._filtered_task_velocity
        )
        self._previous_task_target = target
        self._previous_sequence = sequence_id
        return self._filtered_task_velocity.copy()

    def _calibrated_target(
        self, source: UpperBodyIKTarget, measured: np.ndarray
    ) -> UpperBodyIKTarget:
        if self.config.mapping_mode == "absolute":
            return source
        if self._source_origin is None or self._robot_origin is None:
            state = self.model.forward_kinematics(measured)
            self._source_origin = source
            self._robot_origin = UpperBodyIKTarget(
                state.left_end_effector[:3, 3].copy(),
                state.left_end_effector[:3, :3].copy(),
                state.right_end_effector[:3, 3].copy(),
                state.right_end_effector[:3, :3].copy(),
                state.head[:3, :3].copy(),
            )
        source_origin = self._source_origin
        robot_origin = self._robot_origin
        scale = self.config.position_scale
        return UpperBodyIKTarget(
            robot_origin.left_position_m
            + scale * (source.left_position_m - source_origin.left_position_m),
            source.left_orientation
            @ source_origin.left_orientation.T
            @ robot_origin.left_orientation,
            robot_origin.right_position_m
            + scale * (source.right_position_m - source_origin.right_position_m),
            source.right_orientation
            @ source_origin.right_orientation.T
            @ robot_origin.right_orientation,
            source.head_orientation
            @ source_origin.head_orientation.T
            @ robot_origin.head_orientation,
        )

    def _clear_feedforward(self) -> None:
        self._previous_task_target = None
        self._previous_sequence = None
        self._filtered_task_velocity.fill(0.0)

    def reset_session(self) -> None:
        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        self.tracker.reset(nominal)
        self.last_target = nominal.copy()
        self.handle.write_joint_targets(self.model.joint_names, nominal)
        self._clear_feedforward()
        self._source_origin = None
        self._robot_origin = None
        self.session_started = False
        self.last_application = {
            "accepted": False,
            "reason": "session_reset_to_declared_nominal",
            "joint_target_rad": nominal.tolist(),
            "controller_type": "differential_dls",
        }
        self.events.append(
            {"event": "session_reset_to_declared_nominal", **self.last_application}
        )

    def hold(self, reason: str) -> None:
        self.tracker.hold()
        self._clear_feedforward()
        if self.last_target is not None:
            self.handle.write_joint_targets(self.model.joint_names, self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": reason,
            "joint_target_rad": (
                self.last_target.tolist() if self.last_target is not None else None
            ),
            "controller_type": "differential_dls",
        }
        self.events.append({"event": "hold", **self.last_application})

    def apply_upper_body(
        self, targets: R1TeleopTargets, joints: Sequence[str]
    ) -> None:
        missing = set(self.model.joint_names) - set(joints)
        if missing:
            raise ValueError(
                f"Differential sink received incomplete ownership: {sorted(missing)}"
            )
        if targets.robot_frame != self.config.source_target_frame:
            self.hold("target_frame_mismatch")
            return
        if targets.left_wrist_target is None or targets.right_wrist_target is None:
            self.hold("missing_wrist_target")
            return

        started_ns = perf_counter_ns()
        measured = np.asarray(
            self.handle.joint_positions(self.model.joint_names), dtype=float
        )
        source_target = self._build_target(targets)
        raw_target = self._calibrated_target(source_target, measured)
        projection = project_upper_body_target(
            self.model,
            measured,
            raw_target,
            self.config.workspace_projection_margin_m,
        )
        task_velocity = self._task_velocity(targets.sequence_id, projection.target)
        step = self.tracker.step(
            measured,
            projection.target,
            task_velocity,
            velocity_feedforward=self.config.velocity_feedforward_task != "none",
        )
        self.last_target = step.joint_position_reference_rad.copy()
        self.handle.write_joint_targets(self.model.joint_names, self.last_target)
        compute_ms = (perf_counter_ns() - started_ns) / 1.0e6
        head = self.model.head_slice
        pitch_index, yaw_index = head.start, head.start + 1
        self.session_started = True
        self.last_application = {
            "accepted": True,
            "controller_type": "differential_dls",
            "task_priority": self.config.tracker.task_priority,
            "jacobian_backend": "numpy_central_difference",
            "controller_compute_ms": compute_ms,
            "body_mode": self.model.body_mode,
            "controlled_joint_names": list(self.model.joint_names),
            "solver_solution_kind": (
                "workspace_projected" if projection.any_projected else "inside_reach_bound"
            ),
            "joint_position_reference_rad": self.last_target.tolist(),
            "joint_velocity_reference_rad_s": step.joint_velocity_reference_rad_s.tolist(),
            "limited_joint_target_rad": self.last_target.tolist(),
            "task_error": step.task_error.tolist(),
            "desired_task_velocity": task_velocity.tolist(),
            "minimum_weighted_jacobian_singular_value": step.minimum_singular_value,
            "wide_elbow_pole_activation": step.wide_elbow_pole_activation,
            "wide_elbow_pole_error_m": step.wide_elbow_pole_error_m.tolist(),
            "branch_recovery_active": step.branch_recovery_active.tolist(),
            "branch_recovery_velocity_rad_s": (
                step.branch_recovery_velocity_rad_s.tolist()
            ),
            "velocity_saturated": step.velocity_saturated.tolist(),
            "acceleration_saturated": step.acceleration_saturated.tolist(),
            "reference_lead_clamped": step.reference_lead_clamped.tolist(),
            "joint_limit_active": step.joint_limit_active.tolist(),
            "raw_left_target_position_pelvis_m": raw_target.left_position_m.tolist(),
            "raw_right_target_position_pelvis_m": raw_target.right_position_m.tolist(),
            "source_left_target_position_pelvis_m": source_target.left_position_m.tolist(),
            "source_right_target_position_pelvis_m": source_target.right_position_m.tolist(),
            "mapping_mode": self.config.mapping_mode,
            "position_scale": self.config.position_scale,
            "left_target_position_pelvis_m": projection.target.left_position_m.tolist(),
            "right_target_position_pelvis_m": projection.target.right_position_m.tolist(),
            "workspace_projection": {
                "left_projected": projection.left_projected,
                "right_projected": projection.right_projected,
                "left_projection_distance_m": projection.left_projection_distance_m,
                "right_projection_distance_m": projection.right_projection_distance_m,
                "left_reach_limit_m": projection.left_reach_limit_m,
                "right_reach_limit_m": projection.right_reach_limit_m,
            },
            "head_pitch_yaw_target_rad": self.last_target[head].tolist(),
            "head_target_rad": [
                float(self.last_target[yaw_index]),
                float(self.last_target[pitch_index]),
            ],
        }
        self.events.append(
            {
                "event": "whole_upper_body",
                "sequence_id": targets.sequence_id,
                **self.last_application,
            }
        )
        self.acknowledgements.append(
            {
                "sequence_id": targets.sequence_id,
                "accepted_joints": list(self.model.joint_names),
                "withheld_joints": [],
            }
        )

    def apply_base_velocity(
        self, targets: R1TeleopTargets, joints: Sequence[str]
    ) -> None:
        raise RuntimeError("Base velocity is prohibited in differential upper-body simulation.")


__all__ = [
    "DifferentialWholeUpperBodyIsaacLabSink",
    "DifferentialWholeUpperBodyLiveConfig",
]
