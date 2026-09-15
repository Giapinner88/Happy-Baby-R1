"""Isaac Lab sink for joint targets solved upstream of this process.

The vendor `xr_teleoperate` solver cannot run in the simulator's interpreter:
it needs CasADi and the Pinocchio 3 CasADi bindings, the Isaac environment has
Pinocchio 2.7 with neither, and upstream ships a package whose name collides
with this one. So the solve happens in a separate process beside the Quest
bridge -- which is upstream's own arrangement, and the one the hardware sidecar
wants, since the robot side then needs no kinematic model at all.

What arrives here is therefore a command stream whose lines carry the solved
joint vector alongside the original wrist poses. This sink applies that vector
and does not solve anything. It is deliberately thin: the point of running the
vendor solver unmodified is lost if this end quietly reshapes its output.

Two things it does do, because no one else can:

* Clamp to the asset's joint limits. Upstream constrains against its own
  `r1_a5.urdf`, which is not the asset the simulator loads, so agreement
  between the two is checked here rather than assumed.
* Hold the last applied vector whenever a sequence id arrives without a solved
  vector, or the deadman is released. Holding is the only safe response: there
  is no local solver to fall back on, and writing anything else would be
  inventing a command the operator did not give.

Rate limiting is intentionally absent. The vendor already smooths with its
`WeightedMovingFilter`, and adding a second limiter here would mean this path
is no longer the vendor's behaviour. Joint velocity is measured and recorded
instead, so the cost of that choice is visible in evidence rather than hidden.
"""

from __future__ import annotations

import time
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

from .mapping import R1TeleopTargets
from .upper_body_kinematics import load_r1_a5_upper_body_model


class UpstreamStreamHandle(Protocol):
    def write_joint_targets(
        self, joint_names: Sequence[str], positions_rad: Sequence[float]
    ) -> None: ...

    def joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]: ...


@dataclass(frozen=True)
class UpstreamJointStreamConfig:
    urdf_path: Path
    body_mode: str = "arms_head"
    fixed_waist_yaw_rad: float = 0.0
    hold_uncontrolled_waist_joints: bool = True
    """Command the waist joints nobody solves for to their assumed values.

    `arms_head` leaves `waist_yaw_joint` and `waist_roll_joint` outside the
    controlled set, so without this nothing writes them and PhysX lets them
    deflect under the arms' reaction torque -- 22.4 deg of yaw in a measured
    run, while the solver upstream assumed they had not moved at all. The
    vendor solver locks `waist_yaw_joint` in its own model, so holding it here
    is what makes the simulator match the model that produced the targets.
    """
    waist_roll_hold_rad: float = 0.0
    held_joint_stiffness: float = 10000.0
    held_joint_damping: float = 200.0
    joint_limit_tolerance_rad: float = 1e-6
    """How far past an asset limit a vendor solution may sit before it is clamped.

    The two assets are not the same file, so exact agreement is not guaranteed
    and a bit-level mismatch is not worth a hold. A clamp beyond this is counted
    and surfaced, because a systematic one means the assets have diverged.
    """
    pending_capacity: int = 512
    """Solved vectors kept while waiting to be applied.

    The stream is consumed at control rate and produced at headset rate, so a
    handful are ever in flight. The bound exists so an unconsumed stream cannot
    grow without limit over a long session.
    """


@dataclass
class UpstreamJointStreamSink:
    """Apply vendor-solved joint vectors, indexed by Quest command sequence id."""

    handle: UpstreamStreamHandle
    config: UpstreamJointStreamConfig
    events: list[dict[str, object]] = field(default_factory=list)
    acknowledgements: list[dict[str, object]] = field(default_factory=list)
    last_application: dict[str, object] | None = None
    arm_targets_withheld: int = 0
    clamped_sample_count: int = 0
    max_clamp_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.config.body_mode != "arms_head":
            raise ValueError(
                "The vendor solver locks waist yaw and both head joints, so this sink "
                "supports body_mode='arms_head' only."
            )
        self.model = load_r1_a5_upper_body_model(
            self.config.urdf_path,
            control_waist_yaw=False,
            fixed_waist_yaw_rad=self.config.fixed_waist_yaw_rad,
        )
        self._held_joint_names: tuple[str, ...] = ()
        self._held_joint_values: tuple[float, ...] = ()
        if self.config.hold_uncontrolled_waist_joints:
            self._held_joint_names = ("waist_yaw_joint", "waist_roll_joint")
            self._held_joint_values = (
                float(self.config.fixed_waist_yaw_rad),
                float(self.config.waist_roll_hold_rad),
            )
        self._pending: "OrderedDict[int, np.ndarray]" = OrderedDict()
        self.last_target = np.zeros(self.model.dof, dtype=float)
        self._last_applied_monotonic_s: float | None = None
        self._velocity_samples: list[float] = []

    @property
    def held_joint_names(self) -> tuple[str, ...]:
        return self._held_joint_names

    @property
    def nominal_joint_position_rad(self) -> tuple[float, ...]:
        """Startup pose, which is also the pose a session reset returns to.

        Zero for every controlled joint: that is the R1 arms-head neutral this
        project starts from elsewhere, and it is the configuration the vendor
        solver itself warm-starts from, so beginning anywhere else would put the
        simulator and the solver in different places on the first sample.
        """

        return tuple(0.0 for _ in range(self.model.dof))

    def ingest(
        self, sequence_id: int, joint_names: Sequence[str], positions_rad: Sequence[float]
    ) -> None:
        """Accept one solved vector from the stream.

        Rejects rather than reorders a vector whose joint names disagree with
        the asset: a silent mismatch would apply the right numbers to the wrong
        joints, which is the one failure this path must never produce.
        """

        names = tuple(str(name) for name in joint_names)
        if names != tuple(self.model.joint_names):
            self.events.append(
                {
                    "event": "upstream_joint_names_rejected",
                    "sequence_id": int(sequence_id),
                    "received": list(names),
                }
            )
            return
        values = np.asarray(positions_rad, dtype=float)
        if values.shape != (self.model.dof,) or not np.all(np.isfinite(values)):
            self.events.append(
                {
                    "event": "upstream_joint_vector_rejected",
                    "sequence_id": int(sequence_id),
                    "reason": "wrong shape or non-finite",
                }
            )
            return
        self._pending[int(sequence_id)] = values
        while len(self._pending) > self.config.pending_capacity:
            self._pending.popitem(last=False)

    def _write(self, positions_rad: Sequence[float]) -> None:
        names = tuple(self.model.joint_names) + self._held_joint_names
        values = tuple(float(v) for v in positions_rad) + self._held_joint_values
        self.handle.write_joint_targets(names, values)

    def reset_session(self) -> None:
        self.last_target = np.zeros(self.model.dof, dtype=float)
        self._last_applied_monotonic_s = None
        self._write(self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": "session_reset_to_nominal",
            "joint_target_rad": self.last_target.tolist(),
        }
        self.events.append({"event": "session_reset_to_nominal"})

    def hold(self, reason: str) -> None:
        self._write(self.last_target)
        self.last_application = {
            "accepted": False,
            "reason": reason,
            "joint_target_rad": self.last_target.tolist(),
        }
        self.events.append({"event": "hold", **self.last_application})

    def apply_upper_body(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        compute_start = time.perf_counter()
        missing = set(self.model.joint_names) - set(joints)
        if missing:
            raise ValueError(f"Upstream stream has incomplete ownership: {sorted(missing)}")
        # Looked up, not consumed. When no fresh command arrives the control
        # loop re-applies the last one, and a consuming lookup turned the second
        # application of the same sequence into a spurious hold.
        solved = self._pending.get(int(targets.sequence_id))
        if solved is None:
            self.hold("no_upstream_solution_for_sequence")
            return

        clamped = np.clip(solved, self.model.lower_limits, self.model.upper_limits)
        clamp = float(np.max(np.abs(clamped - solved)))
        if clamp > self.config.joint_limit_tolerance_rad:
            self.clamped_sample_count += 1
            self.max_clamp_rad = max(self.max_clamp_rad, clamp)

        now = time.perf_counter()
        velocity: list[float] | None = None
        if self._last_applied_monotonic_s is not None:
            span = now - self._last_applied_monotonic_s
            if span > 0.0:
                velocity = ((clamped - self.last_target) / span).tolist()
                self._velocity_samples.append(float(np.max(np.abs(velocity))))
        self._last_applied_monotonic_s = now
        self.last_target = clamped
        self._write(self.last_target)

        head = self.model.head_slice
        pitch_index, yaw_index = head.start, head.start + 1
        self.last_application = {
            "accepted": True,
            "controller_type": "upstream_xr_teleoperate_R1_A5_ArmIK",
            "solved_upstream": True,
            "controller_compute_ms": float(1000.0 * (time.perf_counter() - compute_start)),
            "joint_limit_clamp_rad": clamp,
            "controlled_joint_names": list(self.model.joint_names),
            "held_joint_names": list(self._held_joint_names),
            "held_joint_position_rad": list(self._held_joint_values),
            "joint_target_rad": self.last_target.tolist(),
            "limited_joint_target_rad": self.last_target.tolist(),
            "joint_velocity_rad_s": velocity,
            "left_arm_target_rad": self.last_target[self.model.left_arm_slice].tolist(),
            "right_arm_target_rad": self.last_target[self.model.right_arm_slice].tolist(),
            "head_pitch_yaw_target_rad": self.last_target[head].tolist(),
            "head_target_rad": [
                float(self.last_target[yaw_index]),
                float(self.last_target[pitch_index]),
            ],
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

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: Sequence[str]) -> None:
        raise RuntimeError("Base velocity is prohibited on the upstream joint stream path.")

    def summary(self) -> dict[str, object]:
        velocities = np.asarray(self._velocity_samples, dtype=float)
        return {
            "controller_type": "upstream_xr_teleoperate_R1_A5_ArmIK",
            "solver_process": "external (tv environment)",
            "rate_limiter": "none; the vendor WeightedMovingFilter is the only smoothing",
            "joint_limit_clamped_sample_count": int(self.clamped_sample_count),
            "joint_limit_max_clamp_rad": float(self.max_clamp_rad),
            "pending_unapplied_count": int(len(self._pending)),
            "applied_joint_velocity_rad_s": (
                {
                    "count": int(len(velocities)),
                    "mean": float(np.mean(velocities)),
                    "median": float(np.median(velocities)),
                    "p95": float(np.quantile(velocities, 0.95)),
                    "max": float(np.max(velocities)),
                }
                if velocities.size
                else None
            ),
        }


__all__ = ["UpstreamJointStreamSink", "UpstreamJointStreamConfig"]
