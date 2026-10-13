#!/usr/bin/env python3
"""Verify a quasi-static R1 crouch trajectory in MuJoCo before robot deployment.

This tool is deliberately offline: it does not publish DDS messages or use the
robot network.  It creates the same 24-joint target ordering used by
``controller`` and checks the full-body center of mass (CoM) against the
convex hull of the actual MuJoCo foot geometry at every 500 Hz trajectory
sample.  The R1 XML has no actuators or learned standing policy, so this is a
quasi-static safety verifier, not a substitute for a closed-loop robot test.

Run from the repository root with its simulation environment:

    ./.venv/bin/python HB/controller/tools/verify_crouch_com.py

The candidate is a squat, not a chair-sit: there is no final transfer of
weight behind the feet.  A non-zero exit code means the candidate is not an
acceptable deployment trajectory.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np


# This is spec::kDefaultJointPos in src/config/RobotSpec.hpp.  The XML joint
# order after the floating base is exactly the 24-joint policy order.
DEFAULT_Q = np.array(
    [
        -0.10, 0.00, 0.00, 0.30, -0.20, 0.00,
        -0.10, 0.00, 0.00, 0.30, -0.20, 0.00,
        0.00, 0.00,
        0.35, 0.18, 0.00, 0.87, 0.00,
        0.35, -0.18, 0.00, 0.87, 0.00,
    ],
    dtype=np.float64,
)

@dataclass(frozen=True)
class CrouchProfile:
    """One smooth, symmetric squat candidate in controller joint convention."""

    prepare_s: float = 1.20
    descend_s: float = 4.80
    hold_s: float = 1.50
    hip_deg: float = 150.0
    knee_deg: float = 100.0
    lean_deg: float = 58.0
    arm_forward_rad: float = -1.50
    arm_elbow_rad: float = 0.30
    spread_rad: float = 0.08

    @property
    def duration_s(self) -> float:
        return self.prepare_s + self.descend_s + self.hold_s

    def target(self, t: float) -> np.ndarray:
        """Return the 24-joint reference at time t using zero-velocity blends."""
        target = DEFAULT_Q.copy()
        prep = quintic_progress(t / self.prepare_s)
        descent = quintic_progress((t - self.prepare_s) / self.descend_s)

        # Phase 1: widen the base and reach both hands before flexing the legs.
        target[1] = -self.spread_rad * prep
        target[7] = self.spread_rad * prep
        target[14] = target[19] = lerp(DEFAULT_Q[14], self.arm_forward_rad, prep)
        target[17] = target[22] = lerp(DEFAULT_Q[17], self.arm_elbow_rad, prep)

        # Phase 2: symmetric squat.  The ankle reference follows the same
        # feet-flat/forward-lean convention as RunSafeShutdown(), but never
        # transitions to a chair-only "rest" pose.
        hip = lerp(DEFAULT_Q[0], -math.radians(self.hip_deg), descent)
        knee = lerp(DEFAULT_Q[3], math.radians(self.knee_deg), descent)
        lean = math.radians(self.lean_deg) * descent
        ankle = -(hip + knee) - lean

        target[0] = target[6] = hip
        target[3] = target[9] = knee
        target[4] = target[10] = ankle
        return target


@dataclass
class Result:
    min_margin_m: float = math.inf
    max_foot_axis_tilt_deg: float = 0.0
    max_joint_speed: float = 0.0
    samples: int = 0
    first_failure: str | None = None


def quintic_progress(x: float) -> float:
    """Minimum-jerk scalar blend, clamped to [0, 1]."""
    x = min(1.0, max(0.0, x))
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


def lerp(a: float, b: float, s: float) -> float:
    return a + (b - a) * s


def load_model(model_path: Path) -> mujoco.MjModel:
    """Load the standalone robot XML and add a ground plane in memory."""
    xml = model_path.read_text(encoding="utf-8")
    meshdir = model_path.parent / "assets"
    if not meshdir.is_dir():
        raise RuntimeError(f"missing mesh directory: {meshdir}")
    xml = xml.replace('meshdir="assets"', f'meshdir="{meshdir}"', 1)
    ground = (
        '<geom name="crouch_ground" type="plane" size="10 10 0.1" '
        'friction="1.5 0.005 0.0001" condim="6" rgba="0.16 0.16 0.16 1"/>'
    )
    xml = xml.replace("<worldbody>", f"<worldbody>{ground}", 1)
    return mujoco.MjModel.from_xml_string(xml)


def convex_hull(points: Iterable[np.ndarray]) -> list[np.ndarray]:
    """Monotonic-chain hull in the ground XY plane."""
    unique = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(unique) <= 1:
        return [np.array(p) for p in unique]

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return [np.array(p) for p in lower[:-1] + upper[:-1]]


def signed_hull_margin(point: np.ndarray, hull: list[np.ndarray]) -> float:
    """Distance to the support-hull boundary; positive only when point is inside."""
    if len(hull) < 3:
        return -math.inf
    area2 = sum(
        hull[i][0] * hull[(i + 1) % len(hull)][1]
        - hull[(i + 1) % len(hull)][0] * hull[i][1]
        for i in range(len(hull))
    )
    sign = 1.0 if area2 > 0.0 else -1.0
    margins = []
    for i, start in enumerate(hull):
        end = hull[(i + 1) % len(hull)]
        edge = end - start
        length = float(np.linalg.norm(edge))
        if length <= 1e-9:
            continue
        cross = edge[0] * (point[1] - start[1]) - edge[1] * (point[0] - start[0])
        margins.append(sign * cross / length)
    return min(margins, default=-math.inf)


def footprint_points(model: mujoco.MjModel, data: mujoco.MjData, foot_geoms: set[int]) -> tuple[list[np.ndarray], float]:
    """Project the collision-capsule footprints to XY and return their tilt."""
    points: list[np.ndarray] = []
    max_axis_tilt_deg = 0.0
    for geom_id in foot_geoms:
        radius, half_length = model.geom_size[geom_id, :2]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        center = data.geom_xpos[geom_id]
        axis = rotation[:, 2]  # MuJoCo aligns capsule fromto along local Z.
        max_axis_tilt_deg = max(max_axis_tilt_deg, math.degrees(math.asin(min(1.0, abs(axis[2])))))
        for endpoint in (center - axis * half_length, center + axis * half_length):
            # The capsule's projected disc is part of its real footprint.
            for angle in np.linspace(0.0, 2.0 * math.pi, num=12, endpoint=False):
                point = endpoint + radius * (math.cos(angle) * rotation[:, 0] + math.sin(angle) * rotation[:, 1])
                points.append(point[:2].copy())
    return points, max_axis_tilt_deg


def set_quasistatic_pose(model: mujoco.MjModel, data: mujoco.MjData,
                         target: np.ndarray, lean_rad: float,
                         foot_geoms: set[int]) -> float:
    """Set a feet-flat nominal base pose and return maximum foot-axis tilt.

    RunSafeShutdown() encodes torso forward lean through the ankle target.  For
    the free-base model, apply that same nominal pelvis pitch, then translate
    the whole robot vertically until its lowest foot-capsule point is on the
    floor.  XY CoM and support margins are invariant to this vertical shift.
    """
    data.qpos[:] = 0.0
    data.qpos[3] = math.cos(lean_rad * 0.5)
    data.qpos[5] = math.sin(lean_rad * 0.5)
    data.qpos[7:31] = target
    mujoco.mj_forward(model, data)
    footprint, _ = footprint_points(model, data, foot_geoms)
    # Reconstruct full 3-D capsule samples to find the ground height.
    lowest_z = math.inf
    for geom_id in foot_geoms:
        radius, half_length = model.geom_size[geom_id, :2]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        center = data.geom_xpos[geom_id]
        axis = rotation[:, 2]
        for endpoint in (center - axis * half_length, center + axis * half_length):
            lowest_z = min(lowest_z, float(endpoint[2] - radius))
    if not math.isfinite(lowest_z):
        raise RuntimeError("could not determine foot support geometry")
    data.qpos[2] -= lowest_z
    mujoco.mj_forward(model, data)
    _, max_axis_tilt_deg = footprint_points(model, data, foot_geoms)
    return max_axis_tilt_deg


def joint_limits_ok(model: mujoco.MjModel, target: np.ndarray) -> bool:
    for index, value in enumerate(target):
        joint_id = index + 1  # joint 0 is the floating base; remaining order matches policy order.
        low, high = model.jnt_range[joint_id]
        if value < low - 1e-6 or value > high + 1e-6:
            return False
    return True


def analyze(model: mujoco.MjModel, profile: CrouchProfile, minimum_margin_m: float,
            max_joint_speed: float) -> Result:
    data = mujoco.MjData(model)
    pelvis_id = model.body("pelvis").id
    foot_geoms = {
        index
        for index in range(model.ngeom)
        if (name := model.geom(index).name) is not None and "foot" in name and "collision" in name
    }
    if not foot_geoms:
        raise RuntimeError("no foot collision geoms found in model")

    result = Result()
    dt = 0.002  # spec::kLoopDt
    previous_target: np.ndarray | None = None
    for sample in range(int(math.ceil(profile.duration_s / dt)) + 1):
        t = min(profile.duration_s, sample * dt)
        target = profile.target(t)
        descent = quintic_progress((t - profile.prepare_s) / profile.descend_s)
        lean = math.radians(profile.lean_deg) * descent
        foot_axis_tilt_deg = set_quasistatic_pose(model, data, target, lean, foot_geoms)
        footprint, _ = footprint_points(model, data, foot_geoms)
        hull = convex_hull(footprint)
        com_xy = data.subtree_com[pelvis_id][:2].copy()
        margin = signed_hull_margin(com_xy, hull)
        speed = 0.0 if previous_target is None else float(np.max(np.abs(target - previous_target)) / dt)
        previous_target = target

        result.min_margin_m = min(result.min_margin_m, margin)
        result.max_foot_axis_tilt_deg = max(result.max_foot_axis_tilt_deg, foot_axis_tilt_deg)
        result.max_joint_speed = max(result.max_joint_speed, speed)
        result.samples += 1

        if result.first_failure is None:
            if not joint_limits_ok(model, target):
                result.first_failure = f"joint limit violation at t={t:.3f}s"
            elif len(hull) < 3:
                result.first_failure = f"support polygon unavailable at t={data.time:.3f}s"
            elif margin < minimum_margin_m:
                result.first_failure = (
                    f"CoM margin {margin * 1000.0:.1f} mm < {minimum_margin_m * 1000.0:.1f} mm "
                    f"at t={t:.3f}s"
                )
            elif speed > max_joint_speed:
                result.first_failure = (
                    f"joint speed {speed:.2f} rad/s > {max_joint_speed:.2f} rad/s at t={t:.3f}s"
                )

    return result


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    default_model = root / "unitree_rl_mjlab/src/assets/robots/unitree_r1/xmls/r1.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=default_model)
    parser.add_argument("--min-margin-mm", type=float, default=30.0,
                        help="required CoM-to-support-boundary margin (default: 30 mm)")
    parser.add_argument("--max-joint-speed", type=float, default=3.0,
                        help="deployed sender rate limit in rad/s (default: 3.0)")
    parser.add_argument("--hip-deg", type=float, default=CrouchProfile.hip_deg)
    parser.add_argument("--knee-deg", type=float, default=CrouchProfile.knee_deg)
    parser.add_argument("--lean-deg", type=float, default=CrouchProfile.lean_deg)
    parser.add_argument("--descend-s", type=float, default=CrouchProfile.descend_s)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.min_margin_mm <= 0.0 or args.descend_s <= 0.0 or args.max_joint_speed <= 0.0:
        print("min margin, descent time, and max joint speed must be positive", file=sys.stderr)
        return 2
    profile = CrouchProfile(
        hip_deg=args.hip_deg,
        knee_deg=args.knee_deg,
        lean_deg=args.lean_deg,
        descend_s=args.descend_s,
    )
    model_path = args.model.resolve()
    if not model_path.is_file():
        print(f"model not found: {model_path}", file=sys.stderr)
        return 2
    result = analyze(load_model(model_path), profile, args.min_margin_mm / 1000.0, args.max_joint_speed)

    print("CROUCH TRAJECTORY")
    print(
        f"  prepare={profile.prepare_s:.2f}s descend={profile.descend_s:.2f}s hold={profile.hold_s:.2f}s"
    )
    print(
        f"  hip={profile.hip_deg:.1f}deg knee={profile.knee_deg:.1f}deg "
        f"lean={profile.lean_deg:.1f}deg spread={profile.spread_rad:.3f}rad"
    )
    print("MUJOCO QUASI-STATIC CHECK")
    print(f"  samples={result.samples}")
    print(f"  CoM support margin min={result.min_margin_m * 1000.0:.1f} mm")
    print(f"  foot longitudinal-axis tilt max={result.max_foot_axis_tilt_deg:.1f}deg")
    print(f"  joint speed max={result.max_joint_speed:.2f}rad/s")
    if result.first_failure:
        print(f"RESULT: FAIL — {result.first_failure}")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
