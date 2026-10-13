"""Pure-Python arm/head session mapper and fail-closed Isaac sink for T007."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, pi
from typing import Protocol, Sequence

import numpy as np

from .ik import ArmIKConfig, solve_arm_ik
from .kinematics import load_arm_chain
from .mapping import R1TeleopTargets
from .rate_limit import OnlineJointLimiter

ARM_HEAD_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint",
)


class ArmHeadSimulatorHandle(Protocol):
    def write_head_targets(self, yaw_rad: float, pitch_rad: float) -> None: ...
    def head_joint_positions(self) -> tuple[float, float]: ...
    def write_arm_targets(self, left_rad: Sequence[float], right_rad: Sequence[float]) -> None: ...
    def arm_joint_positions(self) -> tuple[tuple[float, ...], tuple[float, ...]]: ...


def _wrap(value: float) -> float:
    return (value + pi) % (2.0 * pi) - pi


def pose_roll_rad(pose: object) -> float:
    q = pose.orientation.normalized()
    return atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))


@dataclass(frozen=True)
class ArmHeadLiveConfig:
    left_neutral_position_m: np.ndarray
    right_neutral_position_m: np.ndarray
    left_lower_m: np.ndarray
    left_upper_m: np.ndarray
    right_lower_m: np.ndarray
    right_upper_m: np.ndarray
    position_scale: float
    max_joint_velocity_rad_s: float
    max_joint_acceleration_rad_s2: float
    control_dt_s: float
    ik: ArmIKConfig
    enforce_workspace: bool = True
    allow_converged_joint_limit_solution: bool = False
    mapping_mode: str = "relative_session"
    allow_projected_position_solution: bool = False
    allow_clamped_roll_solution: bool = False
    neutral_joint_position_rad: tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def validate(self) -> None:
        for value in (self.left_neutral_position_m, self.right_neutral_position_m, self.left_lower_m, self.left_upper_m, self.right_lower_m, self.right_upper_m):
            if np.asarray(value).shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError("Arm-head positions and workspace bounds must be finite 3-vectors.")
        if np.any(self.left_lower_m > self.left_upper_m) or np.any(self.right_lower_m > self.right_upper_m):
            raise ValueError("An arm workspace lower bound exceeds its upper bound.")
        if min(self.position_scale, self.max_joint_velocity_rad_s, self.max_joint_acceleration_rad_s2, self.control_dt_s) <= 0.0:
            raise ValueError("Arm-head scale, rate limits and timestep must be positive.")
        if self.mapping_mode not in {"relative_session", "absolute_vendor_pose"}:
            raise ValueError("mapping_mode must be relative_session or absolute_vendor_pose.")
        neutral_q = np.asarray(self.neutral_joint_position_rad, dtype=float)
        if neutral_q.shape != (5,) or not np.all(np.isfinite(neutral_q)):
            raise ValueError("neutral_joint_position_rad must be a finite 5-vector.")
        self.ik.validate()


@dataclass
class ArmHeadIsaacLabSink:
    """Maps relative Quest motion to arms/head; any invalid arm result holds all."""
    handle: ArmHeadSimulatorHandle
    config: ArmHeadLiveConfig
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    calibrated: bool = False
    source_left: np.ndarray | None = None
    source_right: np.ndarray | None = None
    source_left_roll: float = 0.0
    source_right_roll: float = 0.0
    source_head: tuple[float, float] | None = None
    left_seed: np.ndarray | None = None
    right_seed: np.ndarray | None = None
    left_limiter: OnlineJointLimiter = field(init=False)
    right_limiter: OnlineJointLimiter = field(init=False)
    last_head: tuple[float, float] | None = None
    last_left: np.ndarray | None = None
    last_right: np.ndarray | None = None
    last_application: dict[str, object] | None = None
    session_left_neutral: np.ndarray | None = None
    session_right_neutral: np.ndarray | None = None
    session_head_neutral: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        self.config.validate()
        left, right = self.handle.arm_joint_positions()
        self.left_seed, self.right_seed = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
        self.left_limiter = OnlineJointLimiter(self.config.max_joint_velocity_rad_s, self.config.max_joint_acceleration_rad_s2, self.config.control_dt_s)
        self.right_limiter = OnlineJointLimiter(self.config.max_joint_velocity_rad_s, self.config.max_joint_acceleration_rad_s2, self.config.control_dt_s)
        self.left_limiter.reset(left); self.right_limiter.reset(right)
        self.left_chain, self.right_chain = load_arm_chain("left"), load_arm_chain("right")
        neutral_q = np.asarray(self.config.neutral_joint_position_rad, dtype=float)
        for chain, declared_position in (
            (self.left_chain, self.config.left_neutral_position_m),
            (self.right_chain, self.config.right_neutral_position_m),
        ):
            if np.any(neutral_q < chain.lower_limits) or np.any(neutral_q > chain.upper_limits):
                raise ValueError("Declared reset joint pose exceeds an R1 arm joint limit.")
            if not np.allclose(chain.endpoint_position(neutral_q), declared_position, atol=1e-9):
                raise ValueError("Declared reset joint pose and neutral endpoint position disagree.")
        self.session_left_neutral = self.config.left_neutral_position_m.copy()
        self.session_right_neutral = self.config.right_neutral_position_m.copy()

    def reset_session(self) -> None:
        """Return to declared neutral, then require a new right-trigger session."""
        # Reset is a declared simulator pose, not another IK request.  Solving
        # the neutral Cartesian point from the current branch can return a bent
        # redundant posture—or fail to settle—so a left-trigger reset would not
        # reliably restore the visible initial pose.
        neutral_q = np.asarray(self.config.neutral_joint_position_rad, dtype=float)
        self.left_seed, self.right_seed = neutral_q.copy(), neutral_q.copy()
        self.left_limiter.reset(self.left_seed); self.right_limiter.reset(self.right_seed)
        self.last_left, self.last_right = self.left_seed.copy(), self.right_seed.copy()
        self.last_head = (0.0, 0.0)
        self.handle.write_head_targets(*self.last_head); self.handle.write_arm_targets(self.last_left, self.last_right)
        self.session_left_neutral = self.config.left_neutral_position_m.copy()
        self.session_right_neutral = self.config.right_neutral_position_m.copy()
        self.session_head_neutral = self.last_head
        self.calibrated = False; self.source_left = None; self.source_right = None; self.source_head = None
        self.events.append({"event": "session_reset_to_declared_neutral", "reset_kind": "declared_joint_pose", "robot_left_neutral_position_m": self.session_left_neutral.tolist(), "robot_right_neutral_position_m": self.session_right_neutral.tolist(), "robot_head_neutral_rad": list(self.session_head_neutral), "left_joint_target_rad": self.last_left.tolist(), "right_joint_target_rad": self.last_right.tolist()})

    def _hold(self, reason: str) -> None:
        self.left_limiter.hold(); self.right_limiter.hold()
        if self.last_head is not None: self.handle.write_head_targets(*self.last_head)
        if self.last_left is not None and self.last_right is not None: self.handle.write_arm_targets(self.last_left, self.last_right)
        self.events.append({"event": "hold", "reason": reason, "calibrated": self.calibrated})
        self.last_application = {"accepted": False, "reason": reason, "calibrated": self.calibrated}

    def hold(self, reason: str) -> None:
        self._hold(reason)

    @staticmethod
    def _ik_diagnostics(result: object) -> dict[str, object]:
        """Lossless solver evidence for a rejected live target."""
        return {
            "converged": bool(result.converged),
            "status": str(result.status),
            "iterations": int(result.iterations),
            "position_residual_m": float(result.position_residual_m),
            "roll_residual_rad": float(result.roll_residual_rad),
            "limit_margin_rad": float(result.limit_margin_rad),
            "clamped_joints": list(result.clamped_joints),
        }

    def apply_upper_body(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        if set(ARM_HEAD_JOINT_NAMES) - set(joints):
            raise ValueError("Arm/head sink was given incomplete upper-body ownership.")
        if targets.left_wrist_target is None or targets.right_wrist_target is None:
            self._hold("missing_wrist_target"); return
        left_pose, right_pose = targets.left_wrist_target, targets.right_wrist_target
        left_source = np.asarray([left_pose.position.x, left_pose.position.y, left_pose.position.z], dtype=float)
        right_source = np.asarray([right_pose.position.x, right_pose.position.y, right_pose.position.z], dtype=float)
        left_roll, right_roll = pose_roll_rad(left_pose), pose_roll_rad(right_pose)
        if not self.calibrated:
            self.calibrated = True; self.source_left, self.source_right = left_source.copy(), right_source.copy()
            self.source_left_roll, self.source_right_roll = left_roll, right_roll
            self.source_head = (targets.head_yaw_rad, targets.head_pitch_rad)
            self.events.append({"event": "session_neutral_calibrated", "sequence_id": targets.sequence_id,
                                "left_source_position_m": left_source.tolist(), "right_source_position_m": right_source.tolist(),
                                "left_source_roll_rad": left_roll, "right_source_roll_rad": right_roll,
                                "head_source_yaw_pitch_rad": list(self.source_head)})
        assert self.source_left is not None and self.source_right is not None and self.source_head is not None
        assert self.session_left_neutral is not None and self.session_right_neutral is not None
        if self.config.mapping_mode == "absolute_vendor_pose":
            left_target, right_target = left_source, right_source
            left_roll_target, right_roll_target = _wrap(left_roll), _wrap(right_roll)
        else:
            left_target = self.session_left_neutral + self.config.position_scale * (left_source - self.source_left)
            right_target = self.session_right_neutral + self.config.position_scale * (right_source - self.source_right)
            left_roll_target = _wrap(left_roll - self.source_left_roll)
            right_roll_target = _wrap(right_roll - self.source_right_roll)
        outside_declared_box = not (
            np.all(left_target >= self.config.left_lower_m)
            and np.all(left_target <= self.config.left_upper_m)
            and np.all(right_target >= self.config.right_lower_m)
            and np.all(right_target <= self.config.right_upper_m)
        )
        if self.config.enforce_workspace and outside_declared_box:
            self._hold("workspace_outside_selected_envelope"); return
        assert self.left_seed is not None and self.right_seed is not None
        left = solve_arm_ik(self.left_chain, left_target, left_roll_target, self.left_seed, np.zeros(5), self.config.ik)
        right = solve_arm_ik(self.right_chain, right_target, right_roll_target, self.right_seed, np.zeros(5), self.config.ik)
        reject_limit_solution = (
            (left.hit_a_limit or right.hit_a_limit)
            and not self.config.allow_converged_joint_limit_solution
        )
        def usable(result: object) -> bool:
            if bool(result.converged):
                return True
            if result.status == "roll_target_clamped_to_limit":
                return self.config.allow_clamped_roll_solution
            return self.config.allow_projected_position_solution and result.status in {
                "iteration_budget_exhausted", "tolerance_not_met", "projected_to_reachable_boundary"
            }

        if not usable(left) or not usable(right) or reject_limit_solution:
            self._hold(f"ik_refused:left={left.status};right={right.status}")
            rejection = {
                "left_target_position_m": left_target.tolist(),
                "right_target_position_m": right_target.tolist(),
                "left_ik": self._ik_diagnostics(left),
                "right_ik": self._ik_diagnostics(right),
            }
            assert self.last_application is not None
            self.last_application.update(rejection)
            self.events[-1].update(rejection)
            return
        self.left_seed, self.right_seed = left.joint_positions.copy(), right.joint_positions.copy()
        self.last_left, self.last_right = self.left_limiter.step(left.joint_positions), self.right_limiter.step(right.joint_positions)
        self.last_head = (self.session_head_neutral[0] + _wrap(targets.head_yaw_rad - self.source_head[0]), self.session_head_neutral[1] + _wrap(targets.head_pitch_rad - self.source_head[1]))
        self.handle.write_head_targets(*self.last_head); self.handle.write_arm_targets(self.last_left, self.last_right)
        self.last_application = {
            "accepted": True,
            "calibrated": True,
            "mapping_mode": self.config.mapping_mode,
            "left_target_position_m": left_target.tolist(),
            "right_target_position_m": right_target.tolist(),
            # Preserve the pre-limiter IK result as well as the dispatched
            # command.  Without both, a run cannot separate geometric IK
            # residual from deliberate rate-limit lag.
            "left_ik_joint_target_rad": left.joint_positions.tolist(),
            "right_ik_joint_target_rad": right.joint_positions.tolist(),
            "left_limited_joint_target_rad": self.last_left.tolist(),
            "right_limited_joint_target_rad": self.last_right.tolist(),
            "left_rate_limiter_velocity_rad_s": self.left_limiter.velocity.tolist(),
            "right_rate_limiter_velocity_rad_s": self.right_limiter.velocity.tolist(),
            "head_target_rad": list(self.last_head),
            "left_position_residual_m": left.position_residual_m,
            "right_position_residual_m": right.position_residual_m,
            "left_solution_kind": "exact" if left.converged else "projected",
            "right_solution_kind": "exact" if right.converged else "projected",
            "left_solver_status": left.status,
            "right_solver_status": right.status,
            "left_limit_margin_rad": left.limit_margin_rad,
            "right_limit_margin_rad": right.limit_margin_rad,
            "left_clamped_joints": list(left.clamped_joints),
            "right_clamped_joints": list(right.clamped_joints),
        }
        self.events.append({"event": "upper_body", "sequence_id": targets.sequence_id, **self.last_application})
        self.acknowledgements.append({"sequence_id": targets.sequence_id, "accepted_joints": list(ARM_HEAD_JOINT_NAMES), "withheld_joints": []})

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        raise RuntimeError("Base velocity is prohibited in arm/head simulation.")


__all__ = ["ARM_HEAD_JOINT_NAMES", "ArmHeadIsaacLabSink", "ArmHeadLiveConfig", "ArmHeadSimulatorHandle", "OnlineJointLimiter", "pose_roll_rad"]
