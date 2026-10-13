"""Simulation-only live sink for coupled R1-A5 waist/arms/head IK."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

from .mapping import R1TeleopTargets
from .rate_limit import OnlineJointLimiter
from .upper_body_ik import (
    UpperBodyIKConfig,
    UpperBodyIKTarget,
    quaternion_xyzw_to_matrix,
    solve_upper_body_ik,
)
from .upper_body_kinematics import BODY_MODES, body_mode_flags, load_r1_a5_upper_body_model


class WholeUpperBodySimulatorHandle(Protocol):
    def write_joint_targets(self, joint_names: Sequence[str], positions_rad: Sequence[float]) -> None: ...
    def joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]: ...


def _pose_transform(pose: object) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    transform[:3, :3] = quaternion_xyzw_to_matrix(
        np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    )
    return transform


_PROJECTED_STATUSES = frozenset({"projected_to_reachable_boundary", "iteration_budget_exhausted"})


@dataclass(frozen=True)
class WholeUpperBodyLiveConfig:
    urdf_path: Path
    nominal_joint_position_rad: tuple[float, ...]
    max_joint_velocity_rad_s: float
    max_joint_acceleration_rad_s2: float
    control_dt_s: float
    ik: UpperBodyIKConfig
    source_target_frame: str = "neutral_waist_yaw_link"
    allow_nonconverged_solution: bool = False
    body_mode: str = "waist_yaw"
    """Which torso joints the solver owns: ``arms_head`` (12 DoF, torso frozen),
    ``waist_yaw`` (13 DoF, hardware-common), or ``full_upper_body`` (14 DoF,
    adds waist roll as a declared simulation-only deviation). Freezing the torso
    also removes it from the task, so it cannot be recruited to chase a target
    the arms alone cannot reach."""
    fixed_waist_yaw_rad: float = 0.0
    """Waist yaw held while ``body_mode`` is ``arms_head``. `retarget_nominal`
    sets this from the profile's declared waist yaw so freezing the torso holds
    the declared pose rather than snapping it to zero."""
    seed_restart_residual_m: float | None = None
    """Re-solve from the declared nominal when a continuation solve stalls with a
    position residual above this value and the target is inside the arms' reach.

    The continuation seed is what makes successive solves smooth, but it also
    traps the solver: bringing the hands in towards the body reaches a folded
    posture whose basin the previous, extended solution cannot cross, and the
    damped-least-squares step cannot climb out. Iterations, step size, damping
    and task weights do not change that -- only the seed does. One extra solve
    from the nominal pose recovers it, and the reach check keeps that cost off
    targets that are simply beyond the arm, where a large residual is honest
    rather than a solver failure. ``None`` disables the restart."""
    allow_projected_position_solution: bool = False
    """Dispatch the closest reachable iterate for a target the solver could
    not fully converge on (`projected_to_reachable_boundary` /
    `iteration_budget_exhausted`), instead of freezing all controlled joints
    at the last accepted target. Mirrors the legacy per-arm sink's
    `allow_projected_position_solution`. `singular_system` and other genuine
    solver failures are never accepted through this flag."""

    def validate(self) -> None:
        if self.source_target_frame != "neutral_waist_yaw_link":
            raise ValueError("Coupled upper-body v1 only supports neutral_waist_yaw_link source targets.")
        if self.body_mode not in BODY_MODES:
            raise ValueError(f"body_mode must be one of {BODY_MODES}, got {self.body_mode!r}")
        nominal = np.asarray(self.nominal_joint_position_rad, dtype=float)
        expected_dof = {"arms_head": 12, "waist_yaw": 13, "full_upper_body": 14}[self.body_mode]
        if nominal.shape != (expected_dof,) or not np.all(np.isfinite(nominal)):
            raise ValueError(
                f"nominal_joint_position_rad must be a finite {expected_dof}-vector for body_mode={self.body_mode!r}."
            )
        if self.seed_restart_residual_m is not None and not (
            np.isfinite(self.seed_restart_residual_m) and self.seed_restart_residual_m > 0.0
        ):
            raise ValueError("seed_restart_residual_m must be finite and positive when set.")
        if min(
            self.max_joint_velocity_rad_s,
            self.max_joint_acceleration_rad_s2,
            self.control_dt_s,
        ) <= 0.0:
            raise ValueError("Upper-body rate limits and timestep must be positive.")
        self.ik.validate()


@dataclass
class WholeUpperBodyIsaacLabSink:
    """Atomically dispatch the coupled waist/arms/head target or hold all 13 joints."""

    handle: WholeUpperBodySimulatorHandle
    config: WholeUpperBodyLiveConfig
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    last_application: dict[str, object] | None = None
    last_target: np.ndarray | None = None
    seed: np.ndarray = field(init=False)
    limiter: OnlineJointLimiter = field(init=False)
    model: object = field(init=False)
    session_started: bool = False
    arm_targets_withheld: int = 0

    def __post_init__(self) -> None:
        self.config.validate()
        control_waist_yaw, control_waist_roll = body_mode_flags(self.config.body_mode)
        self.model = load_r1_a5_upper_body_model(
            self.config.urdf_path,
            control_waist_roll=control_waist_roll,
            control_waist_yaw=control_waist_yaw,
            fixed_waist_yaw_rad=self.config.fixed_waist_yaw_rad,
        )
        if self.config.max_joint_velocity_rad_s > float(np.min(self.model.velocity_limits)):
            raise ValueError(
                "Declared common upper-body velocity exceeds at least one selected URDF joint limit."
            )
        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        if np.any(nominal < self.model.lower_limits) or np.any(nominal > self.model.upper_limits):
            raise ValueError("Declared upper-body nominal pose exceeds the selected URDF limits.")
        measured = np.asarray(self.handle.joint_positions(self.model.joint_names), dtype=float)
        if measured.shape != (self.model.dof,) or not np.all(np.isfinite(measured)):
            raise ValueError("Simulator returned a malformed upper-body state.")
        self.seed = self.model.clamp(measured)
        self.last_target = self.seed.copy()
        self.limiter = OnlineJointLimiter(
            self.config.max_joint_velocity_rad_s,
            self.config.max_joint_acceleration_rad_s2,
            self.config.control_dt_s,
            lower_limits=self.model.lower_limits,
            upper_limits=self.model.upper_limits,
        )
        self.limiter.reset(self.seed)

    def reset_session(self) -> None:
        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        self.seed = nominal.copy()
        self.last_target = nominal.copy()
        self.limiter.reset(nominal)
        self.handle.write_joint_targets(self.model.joint_names, nominal)
        self.session_started = False
        self.last_application = {
            "accepted": False,
            "reason": "session_reset_to_declared_nominal",
            "joint_target_rad": nominal.tolist(),
        }
        self.events.append({"event": "session_reset_to_declared_nominal", **self.last_application})

    def _hold(self, reason: str) -> None:
        self.limiter.hold()
        if self.last_target is not None:
            self.handle.write_joint_targets(self.model.joint_names, self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": reason,
            "joint_target_rad": self.last_target.tolist() if self.last_target is not None else None,
        }
        self.events.append({"event": "hold", **self.last_application})

    def hold(self, reason: str) -> None:
        self._hold(reason)

    def apply_upper_body(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        missing = set(self.model.joint_names) - set(joints)
        if missing:
            raise ValueError(f"Coupled upper-body sink was given incomplete ownership: {sorted(missing)}")
        if targets.left_wrist_target is None or targets.right_wrist_target is None:
            self._hold("missing_wrist_target")
            return

        nominal = np.asarray(self.config.nominal_joint_position_rad, dtype=float)
        neutral_waist = self.model.waist_transform_from_q(nominal)
        left_target = neutral_waist @ _pose_transform(targets.left_wrist_target)
        right_target = neutral_waist @ _pose_transform(targets.right_wrist_target)
        # Built from the head chain's own FK so the target is exactly
        # realizable. A hand-written Rz(yaw) @ Ry(pitch) is a different rotation
        # from this chain's pitch-then-yaw order whenever both angles are
        # non-zero, which made the head task unsatisfiable and drove the solver
        # to twist waist_yaw chasing it.
        head_target = neutral_waist[:3, :3] @ self.model.head_rotation(
            targets.head_pitch_rad, targets.head_yaw_rad
        )
        target = UpperBodyIKTarget(
            left_target[:3, 3],
            left_target[:3, :3],
            right_target[:3, 3],
            right_target[:3, :3],
            head_target,
        )
        result = solve_upper_body_ik(self.model, target, self.seed, nominal, self.config.ik)
        restarted = False
        if self._should_restart(result, target):
            alternative = solve_upper_body_ik(self.model, target, nominal, nominal, self.config.ik)
            if self._position_error(alternative) < self._position_error(result):
                result, restarted = alternative, True
        projected = (
            self.config.allow_projected_position_solution and result.status in _PROJECTED_STATUSES
        )
        usable = result.converged or self.config.allow_nonconverged_solution or projected
        if not usable:
            self._hold(f"upper_body_ik_refused:{result.status}")
            assert self.last_application is not None
            self.last_application["ik"] = self._diagnostics(result)
            self.events[-1]["ik"] = self._diagnostics(result)
            return

        head = self.model.head_slice
        head_pitch_index, head_yaw_index = head.start, head.start + 1
        self.seed = result.joint_positions.copy()
        self.last_target = self.limiter.step(result.joint_positions)
        self.handle.write_joint_targets(self.model.joint_names, self.last_target)
        self.session_started = True
        self.last_application = {
            "accepted": True,
            "solver_solution_kind": "exact" if result.converged else "projected",
            "body_mode": self.model.body_mode,
            "seed_restarted_from_nominal": restarted,
            "controlled_joint_names": list(self.model.joint_names),
            "ik_joint_target_rad": result.joint_positions.tolist(),
            "limited_joint_target_rad": self.last_target.tolist(),
            "rate_limiter_velocity_rad_s": self.limiter.velocity.tolist(),
            "waist_yaw_target_rad": (
                None
                if self.model.waist_yaw_index is None
                else float(self.last_target[self.model.waist_yaw_index])
            ),
            "left_arm_target_rad": self.last_target[self.model.left_arm_slice].tolist(),
            "right_arm_target_rad": self.last_target[self.model.right_arm_slice].tolist(),
            "head_pitch_yaw_target_rad": self.last_target[head].tolist(),
            # Evidence compatibility: existing tracking traces use
            # [head_yaw, head_pitch], while the coupled model joint order is
            # [head_pitch, head_yaw]. Keep both forms explicit.
            "head_target_rad": [
                float(self.last_target[head_yaw_index]),
                float(self.last_target[head_pitch_index]),
            ],
            "left_target_position_pelvis_m": left_target[:3, 3].tolist(),
            "right_target_position_pelvis_m": right_target[:3, 3].tolist(),
            "ik": self._diagnostics(result),
        }
        if self.model.waist_roll_index is not None:
            self.last_application["waist_roll_target_rad"] = float(
                self.last_target[self.model.waist_roll_index]
            )
        self.events.append(
            {"event": "whole_upper_body", "sequence_id": targets.sequence_id, **self.last_application}
        )
        self.acknowledgements.append(
            {
                "sequence_id": targets.sequence_id,
                "accepted_joints": list(self.model.joint_names),
                "withheld_joints": [],
            }
        )

    @staticmethod
    def _position_error(result: object) -> float:
        return max(float(result.left_position_residual_m), float(result.right_position_residual_m))

    def _within_reach(self, target: UpperBodyIKTarget, q: np.ndarray) -> bool:
        """Are both endpoints inside the arms' conservative reach bound?

        Measured from the shoulders at the solved torso pose, so freeing or
        freezing the waist does not change the meaning of the check.
        """

        waist = self.model.waist_transform_from_q(q)
        for side, position in (
            ("left", target.left_position_m),
            ("right", target.right_position_m),
        ):
            chain = getattr(self.model, f"{side}_arm")
            shoulder = (waist @ np.append(chain.shoulder_origin(), 1.0))[:3]
            if float(np.linalg.norm(np.asarray(position) - shoulder)) > chain.max_reach_from_shoulder_m:
                return False
        return True

    def _should_restart(self, result: object, target: UpperBodyIKTarget) -> bool:
        threshold = self.config.seed_restart_residual_m
        if threshold is None or result.converged:
            return False
        if self._position_error(result) <= threshold:
            return False
        return self._within_reach(target, result.joint_positions)

    @staticmethod
    def _diagnostics(result: object) -> dict[str, object]:
        return {
            "converged": bool(result.converged),
            "status": str(result.status),
            "iterations": int(result.iterations),
            "left_position_residual_m": float(result.left_position_residual_m),
            "right_position_residual_m": float(result.right_position_residual_m),
            "left_orientation_residual_rad": float(result.left_orientation_residual_rad),
            "right_orientation_residual_rad": float(result.right_orientation_residual_rad),
            "head_orientation_residual_rad": float(result.head_orientation_residual_rad),
            "limit_margin_rad": float(result.limit_margin_rad),
            "clamped_joints": list(result.clamped_joints),
        }

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        raise RuntimeError("Base velocity is prohibited in coupled upper-body simulation.")


__all__ = [
    "WholeUpperBodyIsaacLabSink",
    "WholeUpperBodyLiveConfig",
    "WholeUpperBodySimulatorHandle",
]
