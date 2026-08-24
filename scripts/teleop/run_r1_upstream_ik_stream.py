#!/usr/bin/env python3
"""Stream joint targets by solving Quest wrist poses with the upstream IK.

This is the online counterpart to `solve_r1_t007_upstream_ik.py`. It reads
`R1TeleopCommand` JSON lines on stdin, solves each one with `xr_teleoperate`'s
`R1_A5_ArmIK` exactly as the vendor ships it, and writes one joint-target JSON
line per command on stdout.

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

Wrist targets are consumed in the `neutral_waist_yaw_link` frame and passed to
the solver unconverted: the vendor's `r1_a5.urdf` gives `waist_yaw_joint` a
zero origin, so its root frame and the waist frame coincide. Head pitch and yaw
are taken from the recorded head pose because upstream's reduced model locks
both head joints along with `waist_yaw_joint`.

While the deadman is released no solve is performed and the last solved target
is repeated, so a released trigger holds position instead of drifting toward
whatever the loose controllers report.

Emits joint targets only. Whether those reach a simulator or hardware is the
caller's decision, and this process opens no robot transport itself.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
UPSTREAM_ROOT = REPO_ROOT / "third_party" / "xr_teleoperate_v1_6"

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
)
HEAD_JOINT_NAMES = ("head_pitch_joint", "head_yaw_joint")
DEFAULT_URDF = REPO_ROOT / "assets" / "R1.urdf"


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


def head_angles(pose: dict, limits: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    """Head joint angles that aim the R1 head down the headset's forward axis.

    R1 mounts pitch outside yaw -- `head_pitch_joint` hangs off `waist_yaw_link`
    and `head_yaw_joint` off `head_pitch_link` -- so the head rotation is
    Ry(pitch) @ Rz(yaw) and its forward axis is

        (cos pitch cos yaw, sin yaw, -sin pitch cos yaw).

    Inverting that is not the textbook ZYX extraction, which assumes yaw is the
    outer joint. Reading the angles the ZYX way costs nothing on a pure yaw or a
    pure pitch and goes wrong exactly when the two combine, which is why it
    looks like an off-axis head rather than a bad number.

    `arcsin` bounds yaw at +-90 deg while the joint reaches +-115 deg. The last
    25 deg are unreachable through this inverse; giving them up is preferable to
    a branch that flips the head around when the operator turns far enough.
    """

    forward = pose_to_matrix(pose)[:3, 0]
    norm = float(np.linalg.norm(forward))
    if norm <= 0.0:
        raise ValueError("head pose carries a degenerate forward axis")
    forward = forward / norm
    yaw = float(np.arcsin(float(np.clip(forward[1], -1.0, 1.0))))
    pitch = float(np.arctan2(-forward[2], forward[0]))
    return float(np.clip(pitch, *limits[0])), float(np.clip(yaw, *limits[1]))


def load_solver():
    # Upstream resolves its URDF, meshes and model cache relative to the working
    # directory, so the process moves there rather than editing vendor paths.
    os.chdir(UPSTREAM_ROOT / "teleop" / "robot_control")
    sys.path.insert(0, str(UPSTREAM_ROOT))
    sys.path.insert(0, str(UPSTREAM_ROOT / "teleop"))
    from robot_control.robot_arm_ik import R1_A5_ArmIK

    return R1_A5_ArmIK(Unit_Test=True, Visualization=False)


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
    args = parser.parse_args()

    # Resolved before the solver loads, because loading it moves the working
    # directory into the vendor tree and would strand any relative path here.
    for name in ("input", "output", "stats_path", "urdf_path"):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.expanduser().resolve())

    head_bounds = head_limits_rad(args.urdf_path)
    solver = load_solver()
    joint_names = list(ARM_JOINT_NAMES) + list(HEAD_JOINT_NAMES)

    source = args.input.open("r", encoding="utf-8") if args.input else sys.stdin
    sink = args.output.open("w", encoding="utf-8") if args.output else sys.stdout
    solve_ms: list[float] = []
    held = 0
    last_arms: np.ndarray | None = None

    try:
        for line in source:
            line = line.strip()
            if not line:
                continue
            command = json.loads(line)
            enabled = bool(command.get("deadman_enabled", False))
            if enabled:
                started = time.perf_counter()
                solution, _torque = solver.solve_ik(
                    pose_to_matrix(command["left_wrist_pose"]),
                    pose_to_matrix(command["right_wrist_pose"]),
                )
                solve_ms.append(1000.0 * (time.perf_counter() - started))
                last_arms = np.asarray(solution, dtype=float)
            elif last_arms is None:
                # Nothing solved yet, so there is no pose to hold and emitting a
                # zero vector would be a command, not a hold.
                continue
            else:
                held += 1
            pitch, yaw = head_angles(command["head_pose"], head_bounds)
            solved = [*(float(v) for v in last_arms), pitch, yaw]
            if args.passthrough:
                # The original command is preserved verbatim so the consumer's
                # own evidence stays exactly what it would have been, and the
                # solved vector rides along under its own keys.
                record = dict(command)
                record["upstream_solver"] = "upstream_xr_teleoperate_R1_A5_ArmIK"
                record["upstream_joint_names"] = joint_names
                record["upstream_joint_position_rad"] = solved
                record["upstream_solved_this_sample"] = enabled
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
            sink.write(json.dumps(record) + "\n")
            sink.flush()
    finally:
        if args.input:
            source.close()
        if args.output:
            sink.close()

    if args.stats_path and solve_ms:
        values = np.asarray(solve_ms, dtype=float)
        args.stats_path.write_text(
            json.dumps(
                {
                    "solved_sample_count": int(len(values)),
                    "held_sample_count": int(held),
                    "solve_ms": {
                        "mean": float(np.mean(values)),
                        "median": float(np.median(values)),
                        "p95": float(np.quantile(values, 0.95)),
                        "max": float(np.max(values)),
                    },
                    "implied_rate_ceiling_hz": float(1000.0 / float(np.mean(values))),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
