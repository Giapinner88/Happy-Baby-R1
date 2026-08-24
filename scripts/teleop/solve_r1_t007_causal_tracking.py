#!/usr/bin/env python3
"""Causal single-pass wrist tracking, the online-shaped counterpart to the offline solver.

`solve_r1_t007_offline_continuation.py` answers "what is the best joint path for
this recorded gesture", and to do that it looks at the whole trace: it picks a
global anchor by scanning every sample, continues outward in both directions,
sweeps backward during refinement, reprojects the manifold over the full path,
and interpolates between solver nodes. None of that exists online, where the
next Quest sample is the only new information and the previous command has
already been sent.

This script answers the different question the project actually needs on the
way to a live controller: **what does a strictly causal controller achieve on
the same data?** It therefore refuses every non-causal shortcut:

- one pass, strictly increasing time, no lookahead of any kind;
- warm start is the previous *dispatched* joint vector, never a future node;
- the velocity smoothness term sees only the previous sample, and the
  acceleration term is dropped entirely because it needs the following one;
- no anchor search, no multistart, no backward sweep, no reprojection;
- every sample is solved, at the source rate, with a fixed iteration budget.

Output smoothing is the upstream `xr_teleoperate` mechanism: the
`WeightedMovingFilter([0.4, 0.3, 0.2, 0.1])` that vendor applies to every IK
result, converged or not, before it becomes both the command and the next warm
start. The project solver has no equivalent, which is why a non-convergent
region there produces 38-40 deg joint steps.

The metrics are computed exactly as the offline solver computes them so the two
paths are directly comparable. The gap between them is the cost of causality,
which is the number this experiment exists to measure. It is simulation-only
and grants no hardware authority.
"""

from __future__ import annotations

import argparse
import json
import sys
import time as wallclock
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
from scripts.teleop.run_r1_t007_mujoco_replay import map_packets  # noqa: E402
from scripts.teleop.solve_r1_t007_offline_continuation import (  # noqa: E402
    absolute_vendor_wrist_packets,
    canonicalize_right_arm_problem,
    rate_limit_trajectory,
    R1_A5_MIRROR_JOINT_SIGNS,
)
from teleop.r1.offline_continuation import (  # noqa: E402
    OfflineContinuationConfig,
    _solve_local,
    arm_straightness_deg,
    elbow_outward_pole_m,
    elbow_pole_vector_m,
)
from teleop.r1.schema import R1TeleopCommand  # noqa: E402
from teleop.r1.workspace_projection import project_arm_position_to_reach_sphere  # noqa: E402
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model  # noqa: E402


UPSTREAM_FIR_WEIGHTS = (0.4, 0.3, 0.2, 0.1)


class CausalOutputFilter:
    """The upstream weighted moving filter, newest sample weighted highest.

    Reimplemented rather than imported because the vendor module lives in the
    `tv` environment. `tests/teleop/test_r1_causal_tracking.py` checks this
    against the vendored implementation so the two cannot drift apart.
    """

    def __init__(self, weights: tuple[float, ...], size: int) -> None:
        total = float(sum(weights))
        if not np.isclose(total, 1.0):
            raise ValueError(f"filter weights must sum to 1.0, got {total}")
        self._weights = np.asarray(weights, dtype=float)
        self._queue: list[np.ndarray] = []
        self._size = int(size)

    def push(self, sample: np.ndarray) -> np.ndarray:
        values = np.asarray(sample, dtype=float)
        if values.shape != (self._size,):
            raise ValueError(f"filter expects {self._size} values, got {values.shape}")
        if len(self._queue) >= len(self._weights):
            self._queue.pop(0)
        self._queue.append(values.copy())
        if len(self._queue) < len(self._weights):
            return self._queue[-1].copy()
        stacked = np.asarray(self._queue)
        return np.sum(stacked * self._weights[::-1, None], axis=0)


def solve_causal_arm(
    chain: object,
    time_s: np.ndarray,
    positions_m: np.ndarray,
    orientations: np.ndarray,
    nominal_q: np.ndarray,
    config: OfflineContinuationConfig,
    filter_weights: tuple[float, ...] | None,
    restart_residual_m: float | None = None,
    restart_max_travel_rad: float | None = None,
) -> tuple[np.ndarray, np.ndarray, float, int]:
    """One strictly causal forward pass. Returns (raw, filtered, wall seconds, restarts).

    `restart_residual_m` re-solves a sample from the nominal pose when
    continuing from the previous command leaves the endpoint further away than
    that. Warm-starting is what keeps the path smooth, but it also traps the
    arm: on segment 2 of `t007_whole_upper_body_20260823T072635Z` the left arm
    settled against three joint limits at once (shoulder pitch -180 deg, roll
    142 deg, elbow 125 deg) 497 mm from a target a fresh multistart reaches to
    0.6 mm. Escaping needs about 247 deg of joint travel, which `maximum_step_rad`
    spreads over at least twelve consecutive samples, so the local pull of each
    new target never lets it out. One extra solve from a known-good posture is
    the causal way out; it costs a solve only when the trap is entered.
    """

    count = len(time_s)
    raw = np.empty((count, chain.dof), dtype=float)
    filtered = np.empty_like(raw)
    smoother = (
        CausalOutputFilter(filter_weights, chain.dof) if filter_weights else None
    )
    # The first command has no history, so the declared nominal is the only
    # admissible warm start: online there is nothing else to start from.
    previous = np.asarray(nominal_q, dtype=float).copy()
    restarts = 0
    started = wallclock.perf_counter()
    for index in range(count):
        neighbours: tuple[tuple[np.ndarray, float], ...] = ()
        if index > 0:
            dt = float(time_s[index] - time_s[index - 1])
            if dt > 0.0:
                neighbours = ((previous, dt),)
        solution = _solve_local(
            chain,
            previous,
            positions_m[index],
            orientations[index],
            np.asarray(nominal_q, dtype=float),
            neighbours,
            None,  # acceleration needs the following sample; unavailable online
            config,
        )
        if restart_residual_m is not None:
            error = float(
                np.linalg.norm(chain.endpoint_position(solution) - positions_m[index])
            )
            if error > restart_residual_m:
                fresh = _solve_local(
                    chain, np.asarray(nominal_q, dtype=float), positions_m[index],
                    orientations[index], np.asarray(nominal_q, dtype=float),
                    (), None, config,
                )
                fresh_error = float(
                    np.linalg.norm(chain.endpoint_position(fresh) - positions_m[index])
                )
                # A fresh solution is only usable if the arm can actually get
                # there: the rate limiter moves at most `max_joint_step` per
                # sample, so accepting a far-away restart replaces a stuck arm
                # with a lagging one and the peak error gets worse, not better.
                travel = float(np.linalg.norm(fresh - previous))
                affordable = restart_max_travel_rad is None or travel <= restart_max_travel_rad
                if fresh_error < error and affordable:
                    solution = fresh
                    restarts += 1
        raw[index] = solution
        # The filtered value is what a live controller would send, so it is also
        # what the next solve must continue from. Warm-starting from the raw
        # solution instead would let an unfiltered jump seed the next sample.
        filtered[index] = smoother.push(solution) if smoother else solution
        previous = filtered[index].copy()
    return raw, filtered, wallclock.perf_counter() - started, restarts


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--no-output-filter",
        action="store_true",
        help="Disable the upstream FIR so the filter's own contribution is measurable.",
    )
    parser.add_argument(
        "--no-seed-restart",
        action="store_true",
        help="Never re-solve from nominal, so the continuation trap is measurable.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = json.loads(args.config.expanduser().resolve().read_text(encoding="utf-8"))
    if config.get("experiment_id") != "t007" or config.get("mode") != "simulation_only":
        raise SystemExit("--config must be a T007 simulation-only configuration.")
    output = args.output_dir.expanduser().resolve()
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing run directory: {output}")
    output.mkdir(parents=True)

    source = (ROOT / config["source_run"]).resolve()
    commands = [
        R1TeleopCommand.from_dict(json.loads(line))
        for line in (source / "raw_commands.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    segment_index = config["source_selection"].get("deadman_segment_index")
    if segment_index is not None:
        spans: list[tuple[int, int]] = []
        start_index: int | None = None
        for position in range(len(commands) + 1):
            enabled = position < len(commands) and commands[position].deadman_enabled
            if enabled and start_index is None:
                start_index = position
            elif not enabled and start_index is not None:
                spans.append((start_index, position))
                start_index = None
        begin, end = spans[int(segment_index)]
        commands = commands[begin:end]

    model_config = config["model"]
    model = load_r1_a5_upper_body_model(
        (ROOT / model_config["urdf_path"]).resolve(),
        control_waist_yaw=False,
        fixed_waist_yaw_rad=float(model_config["fixed_waist_yaw_rad"]),
    )
    calibration = config["calibration"]
    packets = map_packets(commands, model, float(calibration.get("position_scale", 1.0)), 1.0, None)
    packets = absolute_vendor_wrist_packets(commands, packets)

    continuation_values = dict(config["continuation"])
    continuation_values.pop("anchor_policy", None)
    for key in [name for name in continuation_values if name.endswith("_note")]:
        continuation_values.pop(key)
    solver_config = OfflineContinuationConfig(**continuation_values)
    nominal = np.asarray(model_config["nominal_joint_position_rad"], dtype=float)
    margin = float(calibration.get("workspace_projection_margin_m", 0.01))

    def project_trace(side: str, trace: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        chain = getattr(model, f"{side}_arm")
        projected: list[np.ndarray] = []
        distances: list[float] = []
        for position in trace:
            value, _changed, distance, _limit = project_arm_position_to_reach_sphere(
                chain, position, margin
            )
            projected.append(value)
            distances.append(distance)
        return np.asarray(projected), np.asarray(distances)

    left_target, left_projection = project_trace("left", packets.lp)
    right_target, right_projection = project_trace("right", packets.rp)

    filter_weights = None if args.no_output_filter else UPSTREAM_FIR_WEIGHTS
    restart_residual = (
        None if args.no_seed_restart
        else float(model_config.get("causal_seed_restart_residual_m", 0.05))
    )
    restart_travel = model_config.get("causal_seed_restart_max_travel_rad")
    restart_travel = None if restart_travel is None else float(restart_travel)
    left_raw, left_q, left_wall, left_restarts = solve_causal_arm(
        model.left_arm, packets.t, left_target, packets.lR,
        nominal[model.left_arm_slice], solver_config, filter_weights, restart_residual, restart_travel,
    )

    right_canonical = bool(model_config.get("right_arm_canonical_mirror", False))
    if right_canonical:
        mirrored_target, mirrored_orientation, _ = canonicalize_right_arm_problem(
            right_target, packets.rR, None
        )
        right_raw, right_q, right_wall, right_restarts = solve_causal_arm(
            model.left_arm, packets.t, mirrored_target, mirrored_orientation,
            nominal[model.right_arm_slice] * R1_A5_MIRROR_JOINT_SIGNS,
            solver_config, filter_weights, restart_residual, restart_travel,
        )
        right_raw = right_raw * R1_A5_MIRROR_JOINT_SIGNS
        right_q = right_q * R1_A5_MIRROR_JOINT_SIGNS
    else:
        right_raw, right_q, right_wall, right_restarts = solve_causal_arm(
            model.right_arm, packets.t, right_target, packets.rR,
            nominal[model.right_arm_slice], solver_config, filter_weights, restart_residual, restart_travel,
        )

    unlimited = np.tile(nominal, (len(packets.t), 1))
    unlimited[:, model.left_arm_slice] = left_q
    unlimited[:, model.right_arm_slice] = right_q
    unlimited[:, model.head_slice] = packets.ha

    limits = config["trajectory_limits"]
    q, velocity, acceleration = rate_limit_trajectory(
        unlimited, packets.t, model.lower_limits, model.upper_limits,
        float(limits["max_joint_velocity_rad_s"]),
        float(limits["max_joint_acceleration_rad_s2"]),
        mode=str(limits.get("rate_limit_mode", "component_wise")),
    )

    def endpoint_error(side: str, joints: np.ndarray, target: np.ndarray) -> np.ndarray:
        chain = getattr(model, f"{side}_arm")
        slice_ = getattr(model, f"{side}_arm_slice")
        return np.asarray(
            [
                float(np.linalg.norm(chain.endpoint_position(row[slice_]) - point))
                for row, point in zip(joints, target)
            ]
        )

    def summary(values: np.ndarray) -> dict[str, float]:
        finite = np.asarray(values, dtype=float)
        return {
            "count": int(len(finite)),
            "mean": float(np.mean(finite)),
            "median": float(np.median(finite)),
            "p95": float(np.quantile(finite, 0.95)),
            "max": float(np.max(finite)),
        }

    metrics = {
        "schema_version": 1,
        "causality": "strictly_causal_single_pass",
        "output_filter": list(filter_weights) if filter_weights else None,
        "sample_count": int(len(packets.t)),
        "source_duration_s": float(packets.t[-1] - packets.t[0]),
        "wrist_position_error_m": {
            "left": summary(endpoint_error("left", q, left_target)),
            "right": summary(endpoint_error("right", q, right_target)),
        },
        "wrist_position_error_before_rate_limit_m": {
            "left": summary(endpoint_error("left", unlimited, left_target)),
            "right": summary(endpoint_error("right", unlimited, right_target)),
        },
        "workspace_projection": {
            "left_projected_fraction": float(np.mean(left_projection > 1.0e-9)),
            "right_projected_fraction": float(np.mean(right_projection > 1.0e-9)),
        },
        "joint_step_rad": {
            "raw_solution": summary(np.max(np.abs(np.diff(np.c_[left_raw, right_raw], axis=0)), axis=1)),
            "after_filter": summary(np.max(np.abs(np.diff(np.c_[left_q, right_q], axis=0)), axis=1)),
            "after_rate_limit": summary(np.max(np.abs(np.diff(q, axis=0)), axis=1)),
        },
        "seed_restart_residual_m": restart_residual,
        "seed_restart_max_travel_rad": restart_travel,
        "seed_restart_count": {"left": int(left_restarts), "right": int(right_restarts)},
        "causal_solver_timing": {
            "left_wall_s": float(left_wall),
            "right_wall_s": float(right_wall),
            "total_wall_s": float(left_wall + right_wall),
            "milliseconds_per_bilateral_sample": float(
                1000.0 * (left_wall + right_wall) / max(len(packets.t), 1)
            ),
            "realtime_factor_against_source_duration": float(
                (packets.t[-1] - packets.t[0]) / max(left_wall + right_wall, 1e-12)
            ),
        },
        "elbow_pole_direction": {
            side: {
                "forward_fraction": float(
                    np.mean(
                        [elbow_pole_vector_m(getattr(model, f"{side}_arm"), row)[0] > 0.005
                         for row in q[:, getattr(model, f"{side}_arm_slice")]]
                    )
                ),
                "inward_fraction": float(
                    np.mean(
                        [elbow_outward_pole_m(getattr(model, f"{side}_arm"), row) < -0.005
                         for row in q[:, getattr(model, f"{side}_arm_slice")]]
                    )
                ),
                "straightness_deg": summary(
                    np.asarray(
                        [arm_straightness_deg(getattr(model, f"{side}_arm"), row)
                         for row in q[:, getattr(model, f"{side}_arm_slice")]]
                    )
                ),
            }
            for side in ("left", "right")
        },
    }

    # The Isaac replay sink keys off sequence id and pelvis-frame targets, so
    # the causal trajectory is written in exactly the schema it already reads.
    waist = model.pelvis_to_waist(float(model_config["fixed_waist_yaw_rad"]))
    def to_pelvis(trace: np.ndarray) -> np.ndarray:
        return (waist @ np.c_[trace, np.ones(len(trace))].T).T[:, :3]

    np.savez(
        output / "causal_joint_trajectory.npz",
        time_s=packets.t,
        sequence_id=packets.seq,
        joint_names=np.asarray(model.joint_names),
        left_target_position_pelvis_m=to_pelvis(left_target),
        right_target_position_pelvis_m=to_pelvis(right_target),
        joint_position_reference_rad=q,
        unfiltered_joint_solution_rad=np.c_[left_raw, right_raw],
        joint_velocity_reference_rad_s=velocity,
        joint_acceleration_reference_rad_s2=acceleration,
        left_target_position_waist_m=left_target,
        right_target_position_waist_m=right_target,
        left_workspace_projection_distance_m=left_projection,
        right_workspace_projection_distance_m=right_projection,
    )
    write_json(output / "metrics.json", metrics)
    write_experiment_config(output, config)
    write_runner_command(output, [sys.executable, *sys.argv])
    write_metadata(output, ROOT, {"source_run": str(source)})
    write_status(
        output,
        execution_status="completed",
        scientific_outcome="unassessed",
        reason="Causal reference pass; compared against the offline path, not gated on its criteria.",
        extra={"hardware_claim": "none", "dds_or_hardware_called": False},
    )
    print(str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
