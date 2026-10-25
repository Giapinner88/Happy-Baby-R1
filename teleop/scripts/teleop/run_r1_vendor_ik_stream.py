#!/usr/bin/env python3
"""Stream joint targets by solving Quest wrist poses with the upstream IK.

This is the online counterpart to `solve_r1_baseline_upstream_ik.py`. It reads
`R1TeleopCommand` JSON lines on stdin, solves each one with `xr_teleoperate`'s
`R1_A5_ArmIK` (augmented in memory with 5-DOF elbow-swivel retargeting and dual
hand+elbow tracking), and writes one joint-target JSON line per command on
stdout.

Splitting IK away from the robot side is upstream's own arrangement, not an
invention here: the solver runs beside the headset reader and whatever drives
the robot receives joint angles it only has to apply. That is also what the
hardware sidecar wants, since the robot side then needs neither CasADi nor a
kinematic model. It is required in practice too -- CasADi and the Pinocchio 3
CasADi bindings live in the `tv` environment, while the Isaac environment has
Pinocchio 2.7 without them, and upgrading a working simulator environment to
suit the solver is a worse trade than passing joint angles between processes.

Commands are parsed with plain `json` rather than this repository's schema
class on purpose. Upstream ships a package also named `teleop`; importing both
in one interpreter means one of them stops resolving. Reading the wire format
directly keeps this process free of that collision, and the format is stable
enough to parse in a dozen lines.

The vendor wrapper emits wrists relative to the *current* head position and
yaw. Before solving, this process reverses that transform and expresses both
wrists relative to the session's initial head anchor. Moving the head therefore
does not move an otherwise stationary controller target. The resulting poses
remain in `neutral_waist_yaw_link`: the vendor's `r1_a5.urdf` gives
`waist_yaw_joint` a zero origin, so its root frame and the waist frame coincide.

Head pitch and yaw are taken from the recorded head pose because upstream's
reduced model locks both head joints along with `waist_yaw_joint`. The start of
the first sustained deadman press supplies both the wrist anchor and the head
angular neutral. Requiring a few consecutive samples avoids calibrating against
a one-frame startup pulse.

The robot-local launcher selects body_relative: head translation follows the
operator while the calibrated yaw defines forward. Its responsive filter uses
two samples. The default replay route retains the initial fixed head anchor
and vendor filter. Neither route solves while the deadman is released; the H4
passthrough emits explicit disabled frames, not an active hold target.

Emits joint targets only. Whether those reach a simulator or hardware is the
caller's decision, and this process opens no robot transport itself.
"""

from __future__ import annotations

import argparse
import atexit
import signal
import hashlib
import json
import os
import queue
import re
import sys
import threading
import time
import types
from collections import deque
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
UPSTREAM_ROOT = REPO_ROOT / "third_party" / "xr_teleoperate_r1_a5_845b25b"
CONTRACT_PATH = REPO_ROOT / "config" / "vendor_r1_a5_845b25b.json"

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
)
HEAD_JOINT_NAMES = ("head_pitch_joint", "head_yaw_joint")
DEFAULT_URDF = REPO_ROOT / "src" / "assets" / "R1.urdf"
HEAD_NEUTRAL_CONFIRM_SAMPLES = 3
VENDOR_HEAD_TO_WAIST_OFFSET_M = np.array([0.15, 0.0, 0.45])

# Exact R1-A5 kinematic geometry from r1_a5.urdf / R1.urdf:
# - shoulder_roll_joint in waist_yaw_link at q=0: [0.03445, ±0.13790, 0.18490]
# - shoulder_roll -> elbow_joint local vector at q=0: [-0.018259, ±0.030761, -0.191208]
# - elbow_joint -> L_ee/R_ee (at +0.20m from wrist_roll_joint): [0.312180, ∓0.030020, -0.011702]
R1_A5_UPPER_ARM_LEN_M = 0.1945205
R1_A5_FOREARM_EE_LEN_M = 0.3138396
R1_A5_NEUTRAL_SWIVEL_RAD = 0.181682
R1_A5_NEUTRAL_REACH_AZIMUTH_RAD = 0.002480
# Swivel mode keeps the pinned vendor weights (robot_arm_ik.py R1_A5_ArmIK:
# 50 / 0.5 / 0.02 / 0.1) and only adds the elbow position term.
SWIVEL_IK_WEIGHTS = {
    "translational": 50.0,
    "elbow": 30.0,
    "rotation": 0.5,
    "regularization": 0.02,
    "smooth": 0.1,
}


class SolveTimingStats:
    """Exact lifetime count/mean/max with bounded recent quantile samples."""
    def __init__(self):
        self.samples = deque(maxlen=4096)
        self.count = 0
        self.total_ms = 0.0
        self.max_ms = 0.0

    def append(self, value):
        self.samples.append(value)
        self.count += 1
        self.total_ms += value
        self.max_ms = max(self.max_ms, value)


def head_limits_rad(urdf_path: Path) -> tuple[tuple[float, float], ...]:
    """Head joint bounds read from the asset rather than transcribed.

    The vendor reduced model locks both head joints, so this process is the only
    place they are bounded and a stale hand-copied pair would silently command
    past the asset. Read with a regex because importing this repository's
    kinematics would drag in the `teleop` package this process must avoid.
    """

    text = urdf_path.read_text(encoding="utf-8")
    limits: list[tuple[float, float]] = []
    for name in HEAD_JOINT_NAMES:
        block = re.search(
            rf'<joint name="{name}".*?<limit[^>]*lower="([-0-9.eE+]+)"[^>]*upper="([-0-9.eE+]+)"',
            text,
            re.DOTALL,
        )
        if block is None:
            raise SystemExit(f"{urdf_path} declares no limits for {name}")
        limits.append((float(block.group(1)), float(block.group(2))))
    return tuple(limits)


def pose_to_matrix(pose: dict) -> np.ndarray:
    orientation = pose["orientation"]
    x, y, z, w = (
        float(orientation["x"]), float(orientation["y"]),
        float(orientation["z"]), float(orientation["w"]),
    )
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm <= 0.0:
        raise ValueError("pose carries a zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    position = pose["position"]
    transform[:3, 3] = [float(position["x"]), float(position["y"]), float(position["z"])]
    return transform


def neck_angles_from_rotation(rotation: np.ndarray, previous_pitch: float | None = None) -> tuple[float, float]:
    """Aim the forward axis using R1's Ry(pitch) @ Rz(yaw) joint order.

    Pitch stays within +/-90 degrees; the sign of forward X selects the yaw
    branch beyond 90 degrees. At the singular side-facing pose use the full
    rotation for pitch. Head roll does not change the forward direction.
    """
    forward = np.asarray(rotation, dtype=float)[:, 0]
    forward = forward / np.linalg.norm(forward)
    horizontal = float(np.hypot(forward[0], forward[2]))
    if horizontal < 1e-7:
        pitch = (float(previous_pitch) if previous_pitch is not None else
                 float(np.arctan2(rotation[0, 2], rotation[2, 2])))
        yaw = float(np.copysign(np.pi / 2, forward[1]))
    else:
        branch = -1.0 if forward[0] < 0 else 1.0
        yaw = float(np.arctan2(forward[1], branch * horizontal))
        pitch = float(np.arctan2(-branch * forward[2], branch * forward[0]))
    return pitch, yaw


def _head_angles_unbounded(pose: dict) -> tuple[float, float]:
    return neck_angles_from_rotation(pose_to_matrix(pose)[:3, :3])


def head_angles(pose: dict, limits: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    pitch, yaw = _head_angles_unbounded(pose)
    return float(np.clip(pitch, *limits[0])), float(np.clip(yaw, *limits[1]))


def relative_head_angles(pose: dict, neutral_matrix: np.ndarray,
                         limits: tuple[tuple[float, float], ...],
                         previous_pitch: float | None = None) -> tuple[float, float]:
    """Remove the neutral rotation before extracting both neck joint angles."""
    rotation = np.asarray(neutral_matrix)[:3, :3].T @ pose_to_matrix(pose)[:3, :3]
    pitch, yaw = neck_angles_from_rotation(rotation, previous_pitch)
    return float(np.clip(pitch, *limits[0])), float(np.clip(yaw, *limits[1]))


def head_yaw_rotation(head_pose_matrix: np.ndarray) -> np.ndarray:
    """Mirror the vendor's horizontal-head-axis yaw extraction exactly."""

    forward = np.asarray(head_pose_matrix[:3, 0], dtype=float).copy()
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if not np.isfinite(norm) or norm <= 1e-6:
        return np.eye(3)
    forward /= norm
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, up))


def reanchor_wrist_matrix(
    wrist_pose_matrix: np.ndarray,
    current_head_pose_matrix: np.ndarray,
    anchor_head_pose_matrix: np.ndarray,
) -> np.ndarray:
    """Move a vendor current-head-relative wrist pose to the initial head anchor.

    TeleVuer emits ``A_t = yaw(H_t)^-1 * W`` plus its fixed waist offset.
    Recovering ``W`` with the current head and applying ``yaw(H_0)^-1`` removes
    head motion without changing the vendor wrist convention or workspace
    offset.
    """

    wrist = np.asarray(wrist_pose_matrix, dtype=float)
    current_head = np.asarray(current_head_pose_matrix, dtype=float)
    anchor_head = np.asarray(anchor_head_pose_matrix, dtype=float)
    current_yaw = head_yaw_rotation(current_head)
    anchor_yaw = head_yaw_rotation(anchor_head)

    world_wrist_rotation = current_yaw @ wrist[:3, :3]
    world_wrist_position = (
        current_yaw @ (wrist[:3, 3] - VENDOR_HEAD_TO_WAIST_OFFSET_M)
        + current_head[:3, 3]
    )

    anchored = np.eye(4)
    anchored[:3, :3] = anchor_yaw.T @ world_wrist_rotation
    anchored[:3, 3] = (
        anchor_yaw.T @ (world_wrist_position - anchor_head[:3, 3])
        + VENDOR_HEAD_TO_WAIST_OFFSET_M
    )
    return anchored


def body_relative_wrist_matrix(wrist: np.ndarray, head: np.ndarray,
                               anchor: np.ndarray) -> np.ndarray:
    """Follow head translation while retaining the calibrated forward direction.

    Head yaw is removed by reanchor_wrist_matrix; moving the operator through
    the room must not become an extra robot hand displacement.
    """
    moving_anchor = anchor.copy()
    moving_anchor[:3, 3] = head[:3, 3]
    return reanchor_wrist_matrix(wrist, head, moving_anchor)


def retarget_wrist_for_elbow_swivel(
    wrist_pose_matrix: np.ndarray,
    side: str,
    *,
    lever_arm_compensation: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Project a 6-DOF Quest wrist pose onto the R1-A5 5-DOF kinematic manifold.

    Each R1-A5 arm has 5 degrees of freedom (3 shoulder + 1 elbow + 1 forearm
    roll) and no wrist pitch/yaw joints. Consequently, once the 3D end-effector
    position ``p_ee`` (3 DOF) and the elbow swivel angle ``psi`` around the
    shoulder-to-EE axis (1 DOF) are chosen, the 3D elbow position ``p_elbow``
    and the forearm pointing direction ``x_ee`` are geometrically fixed, leaving
    ``wrist_roll_joint`` as the 5th DOF.

    Upstream's copy-pasted 6-DOF cost (50 * pos_cost + 0.5 * rot_cost, with no
    elbow constraint) allowed ``wrist_roll_joint`` and ``shoulder_yaw_joint`` to
    absorb controller roll/tilt while keeping the robot's elbow stationary when
    the operator held the controller in place and swung their elbow. Even worse,
    WebXR's ``targetRaySpace`` is pitched downward around the local +Y axis by
    ~35-40 deg relative to the forearm, so forearm roll rotated the ray's X-axis
    in the opposite yaw direction.

    Because the second column ``y_h = R_hand[:, 1]`` (the elbow hinge axis) is
    mathematically invariant to any local Y-axis pitch ``Ry(theta)`` (both
    ``targetRaySpace`` tilt and human wrist flexion/extension), extracting the
    arm-plane swivel angle from ``y_h`` in the reach-aligned horizontal frame
    reconstructs the operator's true elbow swivel ``psi`` across:
      1. Pure forearm/controller roll (operator bends wrist to keep controller
         aimed forward while flapping the elbow),
      2. Rigid-wrist 3D swivel around the shoulder-to-hand axis, and
      3. Horizontal forearm yaw around a stationary hand,
    while keeping ``delta_psi == 0`` when the operator keeps their elbow fixed
    and swings their hand left/right around the elbow.
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    sign = 1.0 if side == "left" else -1.0
    p_shoulder = np.array([0.03445, sign * 0.13790, 0.18490], dtype=float)
    v_sh_to_el_local = np.array([-0.018259, sign * 0.030761, -0.191208], dtype=float)
    v_el_to_ee_local = np.array([0.312180, -sign * 0.030020, -0.011702], dtype=float)
    gamma_0 = sign * R1_A5_NEUTRAL_REACH_AZIMUTH_RAD

    T_in = np.asarray(wrist_pose_matrix, dtype=float)
    if T_in.shape != (4, 4) or not np.all(np.isfinite(T_in)):
        raise ValueError(f"invalid {side} wrist pose matrix")
    p_raw = T_in[:3, 3].copy()
    R_hand = T_in[:3, :3].copy()
    y_h = R_hand[:, 1]

    if lever_arm_compensation:
        palm_to_ray_z = 0.025
        p_ee = p_raw - np.array(
            [0.0, -palm_to_ray_z * y_h[2], palm_to_ray_z * (1.0 - y_h[1])],
            dtype=float,
        )
    else:
        p_ee = p_raw

    v = p_ee - p_shoulder
    d = float(np.linalg.norm(v))
    max_reach = 0.96 * (R1_A5_UPPER_ARM_LEN_M + R1_A5_FOREARM_EE_LEN_M)
    min_reach = abs(R1_A5_UPPER_ARM_LEN_M - R1_A5_FOREARM_EE_LEN_M) + 0.02
    if d < 1e-6:
        # Degenerate zero-vector input (e.g. synthetic origin fixture): return
        # unmodified target pose and neutral elbow position.
        p_el_neutral = p_shoulder + v_sh_to_el_local
        return T_in.copy(), p_el_neutral
    if d > max_reach:
        p_ee = p_shoulder + (max_reach / d) * v
        v = p_ee - p_shoulder
        d = max_reach
    elif d < min_reach:
        p_ee = p_shoulder + (min_reach / d) * v
        v = p_ee - p_shoulder
        d = min_reach
    u = v / max(d, 1e-6)

    a = (R1_A5_UPPER_ARM_LEN_M**2 - R1_A5_FOREARM_EE_LEN_M**2 + d**2) / (2.0 * d)
    h = float(np.sqrt(max(0.0, R1_A5_UPPER_ARM_LEN_M**2 - a**2)))

    down = np.array([0.0, 0.0, -1.0], dtype=float)
    e_down = down - np.dot(down, u) * u
    norm_ed = float(np.linalg.norm(e_down))
    if norm_ed < 1e-5:
        e_down = np.array([-1.0, 0.0, 0.0], dtype=float) - u[0] * u
        norm_ed = float(np.linalg.norm(e_down))
    e_down /= max(norm_ed, 1e-6)
    e_lat = np.cross(u, e_down)

    horiz_norm = float(np.hypot(v[0], v[1]))
    if horiz_norm > 0.05:
        gamma_reach = float(np.arctan2(v[1], v[0])) - gamma_0
    else:
        gamma_reach = 0.0
    c_g, s_g = float(np.cos(gamma_reach)), float(np.sin(gamma_reach))
    f_horiz = np.array([c_g, s_g, 0.0], dtype=float)
    l_horiz = np.array([-s_g, c_g, 0.0], dtype=float)

    y_lat = float(y_h[1])
    y_up = float(y_h[2])
    psi_roll = float(np.arctan2(y_up, max(y_lat, 1e-4)))
    alpha_y = float(np.arctan2(float(y_h[0]), max(y_lat, 1e-4)))
    if alpha_y * gamma_reach < 0.0:
        psi_yaw = float(np.copysign(max(0.0, abs(alpha_y) - abs(gamma_reach)), alpha_y) + 0.19 * gamma_reach)
    else:
        psi_yaw = alpha_y
    if np.sign(psi_roll) == np.sign(psi_yaw):
        delta_psi_raw = float(
            np.copysign(
                np.hypot(psi_roll, psi_yaw),
                psi_roll if abs(psi_roll) >= abs(psi_yaw) else psi_yaw,
            )
        )
    else:
        delta_psi_raw = psi_roll + psi_yaw

    if side == "right":
        delta_psi = float(np.clip(delta_psi_raw, -np.deg2rad(75.0), np.deg2rad(12.0)))
    else:
        delta_psi = float(np.clip(delta_psi_raw, -np.deg2rad(12.0), np.deg2rad(75.0)))

    psi_neutral = sign * R1_A5_NEUTRAL_SWIVEL_RAD
    psi = psi_neutral + delta_psi

    perp = np.cos(psi) * e_down + np.sin(psi) * e_lat
    p_elbow = p_shoulder + a * u + h * perp

    vec_el_to_ee = p_ee - p_elbow
    vec_sh_to_el = p_elbow - p_shoulder
    x_ee = vec_el_to_ee / max(float(np.linalg.norm(vec_el_to_ee)), 1e-6)
    y_elbow = l_horiz.copy()
    for _ in range(4):
        z_elbow = np.cross(x_ee, y_elbow)
        z_elbow /= max(float(np.linalg.norm(z_elbow)), 1e-6)
        upper_z_axis = vec_sh_to_el - v_sh_to_el_local[1] * y_elbow - v_sh_to_el_local[0] * x_ee
        upper_z_axis /= max(float(np.linalg.norm(upper_z_axis)), 1e-6)
        y_elbow = np.cross(x_ee, upper_z_axis)
        norm_ye = float(np.linalg.norm(y_elbow))
        if norm_ye < 1e-5:
            y_elbow = l_horiz.copy()
        else:
            y_elbow /= norm_ye
        z_elbow = np.cross(x_ee, y_elbow)
        x_vec = vec_el_to_ee - v_el_to_ee_local[1] * y_elbow - v_el_to_ee_local[2] * z_elbow
        x_ee = x_vec / max(float(np.linalg.norm(x_vec)), 1e-6)

    excess_roll = delta_psi_raw - delta_psi
    y_proj = y_h - np.dot(y_h, x_ee) * x_ee
    norm_yp = float(np.linalg.norm(y_proj))
    if norm_yp > 1e-5:
        y_proj /= norm_yp
        residual_roll = float(np.arctan2(np.dot(y_proj, z_elbow), np.dot(y_proj, y_elbow)))
        applied_roll = float(np.clip(excess_roll + 0.20 * (residual_roll - excess_roll), -1.8, 1.8))
        y_ee = np.cos(applied_roll) * y_elbow + np.sin(applied_roll) * z_elbow
    else:
        y_ee = y_elbow
    z_ee = np.cross(x_ee, y_ee)
    z_ee /= max(float(np.linalg.norm(z_ee)), 1e-6)

    T_out = np.eye(4, dtype=float)
    T_out[:3, :3] = np.column_stack([x_ee, y_ee, z_ee])
    T_out[:3, 3] = p_ee
    return T_out, p_elbow


class HeadNeutralCalibrator:
    """Capture one stable session neutral and hold the head with deadman off."""

    def __init__(
        self,
        limits: tuple[tuple[float, float], ...],
        confirm_samples: int = HEAD_NEUTRAL_CONFIRM_SAMPLES,
    ) -> None:
        if confirm_samples < 1:
            raise ValueError("confirm_samples must be positive")
        self.limits = limits
        self.confirm_samples = confirm_samples
        self.neutral: tuple[float, float] | None = None
        self.anchor_pose_matrix: np.ndarray | None = None
        self.candidate_count = 0
        self.last = (0.0, 0.0)

    @property
    def ready(self) -> bool:
        return self.anchor_pose_matrix is not None

    def update(self, pose: dict, enabled: bool) -> tuple[float, float]:
        if not enabled:
            if self.neutral is None:
                self.candidate_count = 0
            return self.last

        if self.neutral is None:
            self.candidate_count += 1
            self.last = (0.0, 0.0)
            if self.candidate_count >= self.confirm_samples:
                self.neutral = _head_angles_unbounded(pose)
                self.anchor_pose_matrix = pose_to_matrix(pose)
            return self.last

        self.last = relative_head_angles(pose, self.anchor_pose_matrix, self.limits, self.last[0])
        return self.last


def verify_vendor_artifacts() -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    for artifact in contract["artifacts"].values():
        path = UPSTREAM_ROOT / artifact["path"]
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != artifact["sha256"]:
            raise SystemExit(f"vendor artifact absent or hash mismatch: {path}")
    meshes = sorted((UPSTREAM_ROOT / "assets" / "r1" / "meshes").iterdir())
    manifest = "".join(
        f"{path.name} {hashlib.sha256(path.read_bytes()).hexdigest()}\n"
        for path in meshes if path.is_file()
    )
    if (len(meshes) != contract["mesh_count"] or
            hashlib.sha256(manifest.encode()).hexdigest() != contract["mesh_tree_sha256"]):
        raise SystemExit("vendor R1-A5 mesh tree is missing or changed")


def configure_elbow_aware_solver(solver, elbow_mode: str = "swivel"):
    """Augment an in-memory R1_A5_ArmIK instance with 5-DOF elbow swivel tracking.

    The pinned vendor files on disk remain completely unmodified. When
    ``elbow_mode == "swivel"``, this attaches symbolic elbow position tracking
    for ``left_elbow_joint`` and ``right_elbow_joint`` to the existing CasADi
    ``Opti`` problem and wraps ``solver.solve_ik`` so active arms are retargeted
    onto the 5-DOF R1-A5 manifold before solving. ``vendor`` leaves the solver
    untouched. The Opti objective can only be patched once, so a second call
    is rejected instead of silently keeping the first mode.
    """
    if elbow_mode not in ("swivel", "vendor"):
        raise ValueError(f"elbow_mode must be 'swivel' or 'vendor', got {elbow_mode!r}")
    if getattr(solver, "elbow_mode", None) is not None:
        raise RuntimeError("solver elbow_mode is already configured; it cannot change after load")
    solver.elbow_mode = elbow_mode
    solver.last_elbow_targets = None
    if elbow_mode != "swivel":
        return solver

    if not all(
        hasattr(solver, attr)
        for attr in (
            "opti", "reduced_robot", "cdata", "cq", "var_q",
            "translational_cost", "rotation_cost",
            "regularization_cost", "smooth_cost",
        )
    ):
        raise SystemExit("vendor R1_A5_ArmIK no longer exposes the Opti cost terms swivel mode patches")
    import casadi

    model = solver.reduced_robot.model
    solver.L_elbow_id = model.getFrameId("left_elbow_joint")
    solver.R_elbow_id = model.getFrameId("right_elbow_joint")
    solver.param_elbow_l = solver.opti.parameter(3)
    solver.param_elbow_r = solver.opti.parameter(3)
    elbow_error = casadi.Function(
        "elbow_error",
        [solver.cq, solver.param_elbow_l, solver.param_elbow_r],
        [
            casadi.vertcat(
                solver.cdata.oMf[solver.L_elbow_id].translation - solver.param_elbow_l,
                solver.cdata.oMf[solver.R_elbow_id].translation - solver.param_elbow_r,
            )
        ],
    )
    solver.elbow_cost = casadi.sumsqr(
        elbow_error(solver.var_q, solver.param_elbow_l, solver.param_elbow_r)
    )
    w = SWIVEL_IK_WEIGHTS
    solver.opti.minimize(
        w["translational"] * solver.translational_cost
        + w["elbow"] * solver.elbow_cost
        + w["rotation"] * solver.rotation_cost
        + w["regularization"] * solver.regularization_cost
        + w["smooth"] * solver.smooth_cost
    )

    original_solve_ik = solver.solve_ik
    solver._inactive_left_elbow = None

    def _set_inactive_left_elbow(data) -> None:
        solver._inactive_left_elbow = np.asarray(
            data.oMf[solver.L_elbow_id].translation, dtype=float
        ).reshape(3).copy()

    def _elbow_aware_solve_ik(
        left_wrist,
        right_wrist,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
    ):
        inactive_left_el = solver._inactive_left_elbow
        solver._inactive_left_elbow = None
        if inactive_left_el is not None:
            left_target = np.asarray(left_wrist, dtype=float)
            left_elbow = inactive_left_el
        else:
            left_target, left_elbow = retarget_wrist_for_elbow_swivel(left_wrist, "left")
        right_target, right_elbow = retarget_wrist_for_elbow_swivel(right_wrist, "right")
        solver.last_elbow_targets = (left_elbow, right_elbow)
        solver.opti.set_value(solver.param_elbow_l, left_elbow)
        solver.opti.set_value(solver.param_elbow_r, right_elbow)
        if current_lr_arm_motor_q is None and current_lr_arm_motor_dq is None:
            return original_solve_ik(left_target, right_target)
        return original_solve_ik(
            left_target,
            right_target,
            current_lr_arm_motor_q,
            current_lr_arm_motor_dq,
        )

    solver._set_inactive_left_elbow = _set_inactive_left_elbow
    solver.solve_ik = _elbow_aware_solve_ik
    return solver


def load_solver(elbow_mode: str):
    verify_vendor_artifacts()
    # The vendor cache has no source/URDF signature. Rebuild the model from the
    # checked artifacts for every session so an old pickle cannot bypass them.
    (UPSTREAM_ROOT / "teleop" / "robot_control" / "r1_a5_model_cache.pkl").unlink(missing_ok=True)
    # Upstream resolves its URDF, meshes and model cache relative to the working
    # directory, so the process moves there rather than editing vendor paths.
    os.chdir(UPSTREAM_ROOT / "teleop" / "robot_control")
    sys.path.insert(0, str(UPSTREAM_ROOT))
    sys.path.insert(0, str(UPSTREAM_ROOT / "teleop"))
    # The HB source tree also has a regular ``teleop`` package.  The pinned
    # vendor tree intentionally has no package marker, so Python would pick HB
    # first and fail at ``teleop.utils``.  Install a short-lived namespace
    # package for this process only; no vendor file is modified and the vendor
    # solver remains the only IK implementation loaded here.
    vendor_pkg = types.ModuleType("teleop")
    vendor_pkg.__path__ = [str(UPSTREAM_ROOT / "teleop")]
    vendor_pkg.__package__ = "teleop"
    sys.modules["teleop"] = vendor_pkg
    from robot_control.robot_arm_ik import R1_A5_ArmIK

    solver = R1_A5_ArmIK(Unit_Test=True, Visualization=False)
    return configure_elbow_aware_solver(solver, elbow_mode=elbow_mode)


def compact_live_commands(lines):
    """Keep the latest pose plus the latest reset and release, in source order."""
    keep = {len(lines) - 1}
    if lines and lines[-1] is None and len(lines) > 1:
        keep.add(len(lines) - 2)
    last_reset = last_release = None
    first_invalid = None
    for index, line in enumerate(lines):
        if line is None:
            continue
        try:
            command = json.loads(line)
            if not isinstance(command, dict):
                raise ValueError('command must be an object')
        except (ValueError, TypeError):
            # Keep the first fault for the consumer, without accumulating
            # every malformed line while the solver is busy.
            if first_invalid is None:
                first_invalid = index
            continue
        if command.get("reset_requested"):
            last_reset = index
        if command.get("deadman_enabled") is False:
            last_release = index
    keep.update(i for i in (last_reset, last_release, first_invalid) if i is not None)
    return [lines[i] for i in sorted(keep)]


def latest_live_lines(source):
    """Yield only the newest live command after each IK solve.

    The vendor solver is intentionally run synchronously, and on ARM64 it is
    slower than the 30 Hz Quest sample stream.  Reading stdin directly would
    build an ever older backlog; by the time a line reached the solver it
    already violated ``max-command-age-s``.  A one-slot mailbox keeps the
    deadman state and pose current without replaying stale poses.  File replay
    remains on the ordinary iterator path so evidence tools stay deterministic.
    """

    mailbox = queue.Queue(maxsize=1)

    def offer(value: str | None) -> None:
        batch = [value]
        while True:
            try:
                mailbox.put_nowait(batch)
                return
            except queue.Full:
                try:
                    batch = compact_live_commands(mailbox.get_nowait() + batch)
                except queue.Empty:
                    pass

    def reader() -> None:
        for line in source:
            if line.strip():
                offer(line)
        offer(None)

    threading.Thread(target=reader, name="ik-live-reader", daemon=True).start()
    while True:
        for line in mailbox.get():
            if line is None:
                return
            yield line


def solve_tracked_arms(solver, left_wrist, right_wrist, tracking):
    """Fresh vendor solve for the active hand; inactive arm uses a FK hold target.

    The sidecar independently holds inactive motors at fresh encoder positions.
    The FK target here only keeps the two-effector vendor optimization well posed;
    it is not an observation from a missing Quest controller.
    """
    if (not isinstance(tracking, dict) or
            any(type(tracking.get(side)) is not bool for side in ('left', 'right')) or
            not tracking['right']):
        raise ValueError('enabled command requires fresh right-arm tracking')
    if not tracking['left']:
        import pinocchio as pin
        model = solver.reduced_robot.model
        data = model.createData()
        seed = np.asarray(solver.init_data, dtype=float).copy()
        if seed.shape != (10,) or not np.all(np.isfinite(seed)):
            raise ValueError('invalid vendor seed for inactive arm')
        pin.framesForwardKinematics(model, data, seed)
        left_wrist = np.asarray(data.oMf[solver.L_hand_id].homogeneous).copy()
        if hasattr(solver, "_set_inactive_left_elbow"):
            solver._set_inactive_left_elbow(data)
    solution, torque = solver.solve_ik(left_wrist, right_wrist)
    return solution, torque, left_wrist, right_wrist


def shutdown_logging_listener():
    """The pinned logging-mp listener ignores SIGTERM: close its queue first."""
    module = sys.modules.get("logging_mp")
    manager = getattr(module, "_internal_manager", None)
    child = getattr(manager, "_listener_process", None)
    if child is None:
        return
    try:
        manager._log_queue.put_nowait(None)
        child.join(timeout=2)
    finally:
        if child.is_alive():
            child.kill()
            child.join(timeout=1)
        manager._listener_process = None
        manager._is_started = False


def terminate_ik(signum, frame):
    # SystemExit reaches finally and atexit, including while stdin is idle.
    raise SystemExit(0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--input", type=Path, default=None,
        help="Read commands from this file instead of stdin; useful for replaying a trace.",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Write joint targets here instead of stdout.",
    )
    parser.add_argument(
        "--stats-path", type=Path, default=None,
        help="Write per-run solve timing here on exit.",
    )
    parser.add_argument(
        "--passthrough", action="store_true",
        help=(
            "Emit each original command with the solved joints attached, instead of "
            "joint targets alone. The simulator runner needs the wrist poses to keep "
            "writing its usual evidence, and `R1TeleopCommand` ignores the extra keys, "
            "so the augmented line stays a valid command stream."
        ),
    )
    parser.add_argument("--urdf-path", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--preflight", action="store_true", help="Validate pinned local artifacts and exit.")
    parser.add_argument("--max-command-age-s", type=float, default=0.3)
    parser.add_argument("--wrist-reference", choices=("initial_head", "body_relative"),
                        default="initial_head")
    parser.add_argument("--arm-filter", choices=("vendor", "responsive"), default="vendor",
                        help="responsive uses two samples (0.7/0.3) instead of four.")
    parser.add_argument("--elbow-mode", choices=("swivel", "vendor"), default="vendor",
                        help="swivel reconstructs 5-DOF elbow swivel targets so elbow motion tracks naturally.")
    args = parser.parse_args()
    if not np.isfinite(args.max_command_age_s) or args.max_command_age_s <= 0:
        raise SystemExit("--max-command-age-s must be finite and positive")
    if args.preflight:
        verify_vendor_artifacts()
        print("[VENDOR IK PREFLIGHT] pinned artifacts present", file=sys.stderr, flush=True)
        return 0

    # Resolved before the solver loads, because loading it moves the working
    # directory into the vendor tree and would strand any relative path here.
    for name in ("input", "output", "stats_path", "urdf_path"):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.expanduser().resolve())

    head_bounds = head_limits_rad(args.urdf_path)
    signal.signal(signal.SIGTERM, terminate_ik)
    atexit.register(shutdown_logging_listener)
    solver = load_solver(args.elbow_mode)
    if args.arm_filter == "responsive":
        solver.smooth_filter = type(solver.smooth_filter)(np.array([0.7, 0.3]), 10)
    print(f"[VENDOR IK READY] pinned R1_A5_ArmIK loaded (elbow_mode={args.elbow_mode})", file=sys.stderr, flush=True)
    joint_names = list(ARM_JOINT_NAMES) + list(HEAD_JOINT_NAMES)

    source = args.input.open("r", encoding="utf-8") if args.input else sys.stdin
    sink = args.output.open("w", encoding="utf-8") if args.output else sys.stdout
    solve_ms = SolveTimingStats()
    held = 0
    calibration_samples_skipped = 0
    input_line_count = 0
    stale_samples_skipped = 0
    last_sequence_id: int | None = None
    last_arms: np.ndarray | None = None
    last_wrist_targets: tuple[np.ndarray, np.ndarray] | None = None
    last_elbow_targets: tuple[np.ndarray, np.ndarray] | None = None
    head_calibrator = HeadNeutralCalibrator(head_bounds)
    reset_requires_release = True

    downstream_closed = False
    failure: str | None = None
    try:
        line_iter = source if args.input else latest_live_lines(source)
        for line in line_iter:
            line = line.strip()
            if not line:
                continue
            command = json.loads(line)
            input_line_count += 1
            source_at = float(command["timestamp_monotonic_s"])
            if not np.isfinite(source_at):
                raise ValueError("Quest source timestamp is not finite")
            if (args.input is None and time.monotonic() - source_at > args.max_command_age_s
                    and command.get("deadman_enabled") and not command.get("reset_requested")):
                stale_samples_skipped += 1
                continue
            sequence_value = command.get("sequence_id")
            last_sequence_id = int(sequence_value) if sequence_value is not None else None
            enabled = bool(command.get("deadman_enabled", False))
            if command.get("reset_requested"):
                head_calibrator = HeadNeutralCalibrator(head_bounds)
                reset_requires_release = True
            elif not enabled:
                reset_requires_release = False
            enabled = enabled and not reset_requires_release
            pitch, yaw = head_calibrator.update(command["head_pose"], enabled)
            if enabled and not head_calibrator.ready:
                calibration_samples_skipped += 1
                enabled = False
            # Carry STOP even before the first solve and throughout reset and
            # calibration. No fabricated zero joint target is needed.
            if not enabled and args.passthrough:
                record = dict(command)
                record["deadman_enabled"] = False
                record["upstream_solved_this_sample"] = False
                sink.write(json.dumps(record) + "\n")
                sink.flush()
                held += 1
                continue
            if enabled:
                assert head_calibrator.anchor_pose_matrix is not None
                current_head = pose_to_matrix(command["head_pose"])
                transform_wrist = (body_relative_wrist_matrix
                                   if args.wrist_reference == "body_relative"
                                   else reanchor_wrist_matrix)
                left_wrist = transform_wrist(
                    pose_to_matrix(command["left_wrist_pose"]),
                    current_head,
                    head_calibrator.anchor_pose_matrix,
                )
                right_wrist = transform_wrist(
                    pose_to_matrix(command["right_wrist_pose"]),
                    current_head,
                    head_calibrator.anchor_pose_matrix,
                )
                started = time.perf_counter()
                arm_tracking = command.get("arm_tracking", {"left": True, "right": True})
                solution, _torque, left_wrist, right_wrist = solve_tracked_arms(
                    solver, left_wrist, right_wrist, arm_tracking)
                solve_ms.append(1000.0 * (time.perf_counter() - started))
                last_arms = np.asarray(solution, dtype=float)
                if last_arms.shape != (10,) or not np.all(np.isfinite(last_arms)):
                    raise ValueError("vendor IK returned an invalid 10-joint solution")
                last_wrist_targets = (left_wrist, right_wrist)
                elbow_pair = getattr(solver, "last_elbow_targets", None)
                if elbow_pair is None and args.elbow_mode == "swivel":
                    _, left_el = retarget_wrist_for_elbow_swivel(left_wrist, "left")
                    _, right_el = retarget_wrist_for_elbow_swivel(right_wrist, "right")
                    elbow_pair = (left_el, right_el)
                last_elbow_targets = elbow_pair
            elif last_arms is None:
                # Nothing solved yet, so there is no pose to hold and emitting a
                # zero vector would be a command, not a hold.
                continue
            else:
                held += 1
            solved = [*(float(v) for v in last_arms), pitch, yaw]
            if args.passthrough:
                # The original command is preserved verbatim so the consumer's
                # own evidence stays exactly what it would have been, and the
                # solved vector rides along under its own keys.
                record = dict(command)
                record["upstream_arm_tracking"] = command.get("arm_tracking", {"left": True, "right": True})
                record["upstream_solver"] = "upstream_xr_teleoperate_R1_A5_ArmIK"
                record["upstream_joint_names"] = joint_names
                record["upstream_joint_position_rad"] = solved
                record["upstream_solved_this_sample"] = enabled
                if enabled:
                    # The Quest timestamp measures capture time, while the
                    # downstream watchdog needs the time the vendor solve
                    # actually became available.  On ARM64 a valid solve can
                    # take close to the 300 ms source lease even after stale
                    # input has already been dropped above.
                    record["upstream_solved_timestamp_monotonic_s"] = time.monotonic()
                record["upstream_wrist_reference_mode"] = args.wrist_reference
                record["upstream_elbow_mode"] = args.elbow_mode
                assert last_wrist_targets is not None
                record["upstream_left_wrist_target_matrix"] = last_wrist_targets[0].tolist()
                record["upstream_right_wrist_target_matrix"] = last_wrist_targets[1].tolist()
                if last_elbow_targets is not None:
                    record["upstream_left_elbow_target_m"] = last_elbow_targets[0].tolist()
                    record["upstream_right_elbow_target_m"] = last_elbow_targets[1].tolist()
            else:
                record = {
                    "schema_version": 1,
                    "sequence_id": command.get("sequence_id"),
                    "timestamp_monotonic_s": command.get("timestamp_monotonic_s"),
                    "deadman_enabled": enabled,
                    "solver": "upstream_xr_teleoperate_R1_A5_ArmIK",
                    "joint_names": joint_names,
                    "joint_position_rad": solved,
                }
            try:
                sink.write(json.dumps(record) + "\n")
                sink.flush()
            except BrokenPipeError:
                # Isaac đóng ống trước khi solver kịp nhận ra phiên đã kết thúc.
                # Đó là trình tự tắt máy bình thường của pipeline ba tiến trình,
                # không phải solver hỏng. `quest_bridge.py:313` xử lý y hệt. Đổi
                # stdout sang /dev/null để lượt flush lúc Python thoát không ném
                # thêm một lỗi thứ hai, rồi vẫn ghi thống kê ra file.
                sys.stdout = open(os.devnull, "w", encoding="utf-8")
                downstream_closed = True
                break
    except SystemExit:
        raise
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        shutdown_logging_listener()
        if args.input:
            source.close()
        if args.output:
            sink.close()
        if args.stats_path:
            values = np.asarray(solve_ms.samples, dtype=float)
            solve_summary = None
            implied_rate = None
            if len(values):
                solve_summary = {
                    "mean": solve_ms.total_ms / solve_ms.count,
                    "median": float(np.median(values)),
                    "p95": float(np.quantile(values, 0.95)),
                    "max": solve_ms.max_ms,
                    "quantile_window_sample_count": len(values),
                    "quantile_scope": "most_recent_4096_solves",
                }
                implied_rate = float(1000.0 / solve_summary['mean'])
            args.stats_path.write_text(
                json.dumps(
                    {
                        "status": "failed" if failure else "completed",
                        "failure": failure,
                        "input_line_count": input_line_count,
                        "last_sequence_id": last_sequence_id,
                        "solved_sample_count": solve_ms.count,
                        "held_sample_count": int(held),
                        "calibration_samples_skipped": int(calibration_samples_skipped),
                        "stale_samples_skipped": int(stale_samples_skipped),
                        "wrist_reference_mode": args.wrist_reference,
                        "arm_filter": args.arm_filter,
                        "elbow_mode": args.elbow_mode,
                        "solve_ms": solve_summary,
                        "implied_rate_ceiling_hz": implied_rate,
                        "head_neutral_pitch_yaw_rad": (
                            list(head_calibrator.neutral) if head_calibrator.neutral else None
                        ),
                        "head_anchor_pose_matrix": (
                            head_calibrator.anchor_pose_matrix.tolist()
                            if head_calibrator.anchor_pose_matrix is not None
                            else None
                        ),
                        "stop_reason": (
                            "exception" if failure
                            else "downstream_closed" if downstream_closed
                            else "input_exhausted"
                        ),
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
