#!/usr/bin/env python3
"""Solve a recorded T007 trace with the upstream `R1_A5_ArmIK`, unmodified.

This project has three solvers of its own and each cost a day of chasing its
own failure mode. This script sets them aside and runs the vendor's solver as
`xr_teleoperate` ships it: one CasADi/IPOPT problem over both arms, cost
`50*translation + 1*rotation + 0.02*regularisation + 0.1*smoothness`, joint
limits as constraints, and `WeightedMovingFilter([0.4, 0.3, 0.2, 0.1])` applied
to every result whether or not IPOPT converged. Nothing here reimplements or
retunes that, and `third_party` is not edited.

The solve itself runs in `_upstream_ik_worker.py` under a separate interpreter,
because upstream ships a package also named `teleop` that collides with this
repository's. See that file for the arrangement.

Output is the `offline_joint_trajectory.npz` schema the Isaac replay path
already consumes, so the vendor solver can be measured against this project's
own on identical evidence with no new plumbing.

Two frame details decide whether the comparison means anything, and both are
handled here rather than in the worker:

* Recorded wrist targets are expressed in `neutral_waist_yaw_link`, and they are
  handed to the vendor solver in that frame unconverted. Upstream's reduced
  model is nominally rooted at `pelvis_link`, but the vendor's `r1_a5.urdf`
  gives `waist_yaw_joint` a zero origin, so its pelvis and waist frames
  coincide. This project's `R1.urdf` puts a real `waist_roll_link` between the
  two and carries a [0.0325, 0, 0.049] m offset there. Applying that offset to
  the vendor's targets would displace them 59 mm and then score the vendor for
  missing; the offset is used only to write pelvis-frame targets for the Isaac
  replay, where this project's asset is the one in play.
* Upstream drives `L_ee`/`R_ee`, virtual frames 0.20 m beyond the wrist roll
  joint. This project's arm chain already carries that same vendor offset, so
  both are scored at the same physical point.

The head is outside upstream's reduced model, which locks `waist_yaw_joint`,
`head_pitch_joint` and `head_yaw_joint`. Head angles therefore come from the
recorded head pose, as this project's live path already maps them, and only the
ten arm joints come from the vendor solve.

Simulation-only. Grants no hardware authority.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evidence.writer import (  # noqa: E402
    write_experiment_config,
    write_json,
    write_metadata,
    write_runner_command,
    write_status,
)
from teleop.r1.schema import R1TeleopCommand  # noqa: E402
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model  # noqa: E402

WORKER = Path(__file__).resolve().parent / "_upstream_ik_worker.py"


def pose_to_matrix(pose: object) -> np.ndarray:
    """Homogeneous transform from a schema `Pose` with an xyzw quaternion."""

    quaternion = pose.orientation
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm <= 0.0:
        raise ValueError("wrist pose carries a zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    transform[:3, 3] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    return transform


def deadman_segments(commands: list[R1TeleopCommand]) -> list[tuple[int, int]]:
    spans: list[tuple[int, int]] = []
    start: int | None = None
    for index in range(len(commands) + 1):
        enabled = index < len(commands) and commands[index].deadman_enabled
        if enabled and start is None:
            start = index
        elif not enabled and start is not None:
            spans.append((start, index))
            start = None
    return spans


def summarise(values: np.ndarray) -> dict[str, float]:
    return {
        "count": int(len(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)),
        "max": float(np.max(values)),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--deadman-segment-index",
        type=int,
        default=None,
        help="Which contiguous deadman segment to solve; default is the longest.",
    )
    parser.add_argument("--urdf-path", type=Path, default=ROOT / "assets" / "R1.urdf")
    parser.add_argument(
        "--worker-python",
        default="conda run --no-capture-output -n tv python",
        help="Command that runs the worker; must reach an interpreter with CasADi and Pinocchio.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output = args.output_dir.expanduser().resolve()
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing run directory: {output}")
    source = args.source_run.expanduser().resolve()

    commands = [
        R1TeleopCommand.from_dict(json.loads(line))
        for line in (source / "raw_commands.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    spans = deadman_segments(commands)
    if not spans:
        raise SystemExit("source trace has no deadman-enabled segment")
    if args.deadman_segment_index is None:
        begin, end = max(
            spans,
            key=lambda span: commands[span[1] - 1].timestamp_monotonic_s
            - commands[span[0]].timestamp_monotonic_s,
        )
        segment_index = spans.index((begin, end))
    else:
        segment_index = int(args.deadman_segment_index)
        begin, end = spans[segment_index]
    selected = commands[begin:end]
    if len(selected) < 3:
        raise SystemExit("selected segment has fewer than three samples")

    model = load_r1_a5_upper_body_model(
        args.urdf_path.expanduser().resolve(), control_waist_yaw=False
    )
    pelvis_from_waist = model.pelvis_to_waist(0.0)

    left_waist = np.stack([pose_to_matrix(c.left_wrist_pose) for c in selected])
    right_waist = np.stack([pose_to_matrix(c.right_wrist_pose) for c in selected])

    with tempfile.TemporaryDirectory() as scratch:
        request = Path(scratch) / "targets.npz"
        response = Path(scratch) / "solution.npz"
        np.savez(request, left_target_pelvis=left_waist, right_target_pelvis=right_waist)
        started = time.perf_counter()
        subprocess.run(
            [*args.worker_python.split(), str(WORKER), "--input", str(request),
             "--output", str(response)],
            check=True,
        )
        wall = time.perf_counter() - started
        solved = dict(np.load(response, allow_pickle=False))

    arm_q = np.asarray(solved["joint_position_rad"], dtype=float)
    vendor_endpoint = np.asarray(solved["ee_position_pelvis"], dtype=float)
    solve_ms = np.asarray(solved["solve_ms"], dtype=float)
    if arm_q.shape != (len(selected), 10):
        raise SystemExit(f"worker returned {arm_q.shape}, expected {(len(selected), 10)}")

    head_slice = model.head_slice
    head_angles = np.zeros((len(selected), 2), dtype=float)
    for index, command in enumerate(selected):
        rotation = pose_to_matrix(command.head_pose)[:3, :3]
        head_angles[index] = (
            float(np.arctan2(-rotation[2, 0], float(np.hypot(rotation[0, 0], rotation[1, 0])))),
            float(np.arctan2(rotation[1, 0], rotation[0, 0])),
        )
    head_angles = np.clip(
        head_angles, model.lower_limits[head_slice], model.upper_limits[head_slice]
    )

    joints = np.zeros((len(selected), model.dof), dtype=float)
    joints[:, model.left_arm_slice] = arm_q[:, :5]
    joints[:, model.right_arm_slice] = arm_q[:, 5:]
    joints[:, head_slice] = head_angles
    clipped = np.clip(joints, model.lower_limits, model.upper_limits)
    limit_clip_m = float(np.max(np.abs(clipped - joints)))
    joints = clipped

    origin = selected[0].timestamp_monotonic_s
    times = np.array([c.timestamp_monotonic_s - origin for c in selected], dtype=float)
    duration = float(times[-1] - times[0])

    # Vendor scored on the vendor's own frames, then cross-checked against this
    # project's chain so a disagreement between the two models is visible rather
    # than silently folded into the tracking number.
    vendor_error = {
        "left": summarise(np.linalg.norm(vendor_endpoint[:, 0] - left_waist[:, :3, 3], axis=1)),
        "right": summarise(np.linalg.norm(vendor_endpoint[:, 1] - right_waist[:, :3, 3], axis=1)),
    }
    project_error = {}
    model_disagreement = {}
    for order, side in enumerate(("left", "right")):
        chain = getattr(model, f"{side}_arm")
        arm_slice = getattr(model, f"{side}_arm_slice")
        waist_targets = (left_waist if side == "left" else right_waist)[:, :3, 3]
        achieved = np.stack([chain.endpoint_position(row[arm_slice]) for row in joints])
        project_error[side] = summarise(np.linalg.norm(achieved - waist_targets, axis=1))
        model_disagreement[side] = summarise(
            np.linalg.norm(achieved - vendor_endpoint[:, order], axis=1)
        )

    step = np.max(np.abs(np.diff(joints, axis=0)), axis=1)
    intervals = np.diff(times)
    velocity = np.max(np.abs(np.diff(joints, axis=0) / intervals[:, None]), axis=1)

    output.mkdir(parents=True)
    np.savez(
        output / "offline_joint_trajectory.npz",
        time_s=times,
        sequence_id=np.array([c.sequence_id for c in selected]),
        joint_names=np.asarray(model.joint_names),
        joint_position_reference_rad=joints,
        left_target_position_pelvis_m=(
            pelvis_from_waist @ np.c_[left_waist[:, :3, 3], np.ones(len(selected))].T
        ).T[:, :3],
        right_target_position_pelvis_m=(
            pelvis_from_waist @ np.c_[right_waist[:, :3, 3], np.ones(len(selected))].T
        ).T[:, :3],
        left_target_position_waist_m=left_waist[:, :3, 3],
        right_target_position_waist_m=right_waist[:, :3, 3],
    )
    (output / "source_segment_raw_commands.jsonl").write_text(
        "\n".join(json.dumps(c.as_dict()) for c in selected) + "\n", encoding="utf-8"
    )
    write_json(
        output / "metrics.json",
        {
            "schema_version": 1,
            "solver": "upstream_xr_teleoperate_R1_A5_ArmIK",
            "solver_modified": False,
            "sample_count": int(len(selected)),
            "deadman_segment_index": segment_index,
            "source_duration_s": duration,
            "source_mean_rate_hz": float((len(selected) - 1) / duration) if duration > 0 else 0.0,
            "vendor_frame_position_error_m": vendor_error,
            "project_chain_position_error_m": project_error,
            "model_disagreement_m": model_disagreement,
            "joint_limit_clip_rad": limit_clip_m,
            "joint_step_rad": summarise(step),
            "joint_velocity_rad_s": summarise(velocity),
            "solver_timing_ms": {
                **summarise(solve_ms),
                "worker_total_wall_s": float(wall),
                "realtime_factor_against_source": float(duration / max(wall, 1e-12)),
                "implied_rate_ceiling_hz": float(1000.0 / max(float(np.mean(solve_ms)), 1e-12)),
            },
        },
    )
    write_experiment_config(
        output,
        {
            "schema_version": 1,
            "experiment_id": "t007",
            "mode": "simulation_only",
            "model": {
                "urdf_path": str(args.urdf_path),
                "body_mode": "arms_head",
                "fixed_waist_yaw_rad": 0.0,
                "hold_uncontrolled_waist_joints": True,
                "waist_roll_hold_rad": 0.0,
                "held_joint_stiffness": 10000.0,
                "held_joint_damping": 200.0,
                "target_frame": "neutral_waist_yaw_link",
                "solver_target_frame": "neutral_waist_yaw_link (unconverted)",
                "pelvis_from_waist_translation_m": pelvis_from_waist[:3, 3].tolist(),
                "pelvis_offset_note": (
                    "Applied only when writing pelvis-frame targets for the Isaac replay. "
                    "The vendor r1_a5.urdf gives waist_yaw_joint a zero origin, so its pelvis "
                    "and waist frames coincide and its targets are passed unconverted."
                ),
            },
            "solver": {
                "source": "third_party/xr_teleoperate_v1_6 R1_A5_ArmIK",
                "modified": False,
                "cost": "50*translation + 1*rotation + 0.02*regularisation + 0.1*smoothness",
                "output_filter": [0.4, 0.3, 0.2, 0.1],
                "locked_joints": ["waist_yaw_joint", "head_pitch_joint", "head_yaw_joint"],
                "controlled_frames": ["L_ee", "R_ee"],
                "head_note": (
                    "Head is outside the vendor reduced model; pitch and yaw come from the "
                    "recorded head pose and are clipped to this project's asset limits."
                ),
                "warm_start": "vendor-internal (self.init_data); no seed is supplied per sample",
            },
            "source_run": str(source),
        },
    )
    write_runner_command(output, [sys.executable, *sys.argv])
    write_metadata(output, ROOT, {"source_run": str(source)})
    write_status(
        output,
        execution_status="completed",
        scientific_outcome="unassessed",
        reason="Vendor-solver reference pass; not gated against this project's acceptance criteria.",
        extra={"hardware_claim": "none", "dds_or_hardware_called": False},
    )
    print(str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
