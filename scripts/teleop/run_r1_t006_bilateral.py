#!/usr/bin/env python3
"""Execute one declared T006 bilateral kinematic/ownership screening case.

This command is simulation-only and CPU-only. It does not open Quest, Isaac
Lab, DDS, ROS, Unitree SDK, a renderer, or any hardware command channel.
Its endpoint-separation diagnostic is not collision evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evidence.run_id import allocate_run_id  # noqa: E402
from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_json,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)
from teleop.r1.bilateral import arm_ownership_audit, evaluate_bilateral_case  # noqa: E402
from teleop.r1.calibration_study import load_t003_waypoints  # noqa: E402
from teleop.r1.ik import ArmIKConfig  # noqa: E402
from teleop.r1.kinematics import load_arm_chain  # noqa: E402
from teleop.r1.mapping import R1JointOwnership  # noqa: E402


T006_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T006"
RUN_ROOT = T006_ROOT / "runs"
DEFAULT_CONFIG = T006_ROOT / "config" / "r1_t006_bilateral_kinematic_cases.json"


def load_config(path: Path) -> dict[str, object]:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load T006 config {path}: {exc}") from exc
    if config.get("experiment_id") != "t006" or config.get("mode") != "simulation_only":
        raise SystemExit("T006 requires experiment_id='t006' and mode='simulation_only'.")
    cases = config.get("cases")
    if not isinstance(cases, list) or not cases:
        raise SystemExit("T006 config must contain at least one declared case.")
    ids = [case.get("case_id") for case in cases if isinstance(case, dict)]
    if len(ids) != len(cases) or any(not isinstance(case_id, str) for case_id in ids) or len(set(ids)) != len(ids):
        raise SystemExit("Every T006 case must have a unique string case_id.")
    return config


def select_cases(config: dict[str, object], case_id: str) -> list[dict[str, object]]:
    cases = [dict(case) for case in config["cases"]]  # type: ignore[index]
    if case_id == "all":
        return cases
    selected = [case for case in cases if case["case_id"] == case_id]
    if len(selected) != 1:
        known = [str(case["case_id"]) for case in cases]
        raise SystemExit(f"Unknown T006 case {case_id!r}; choose one of {known} or 'all'.")
    return selected


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _t003_trace(case: dict[str, object], waypoints_path: Path) -> list[dict[str, object]]:
    right = {str(item["name"]): item for item in load_t003_waypoints(waypoints_path, "right")}
    left = {str(item["name"]): item for item in load_t003_waypoints(waypoints_path, "left")}
    pairs = case.get("waypoint_pairs")
    if not isinstance(pairs, list) or not pairs:
        raise ValueError("A T003 bilateral case requires non-empty waypoint_pairs.")
    trace: list[dict[str, object]] = []
    for index, pair in enumerate(pairs):
        if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(name, str) for name in pair):
            raise ValueError(f"Malformed T003 waypoint pair at index {index}: {pair!r}")
        left_wp, right_wp = left.get(pair[0]), right.get(pair[1])
        if left_wp is None or right_wp is None:
            raise ValueError(f"T003 waypoint pair {pair!r} names a missing source waypoint.")
        trace.append({
            "name": f"{pair[0]}__{pair[1]}", "source": "T003-A recorded right trace + declared left Y-mirror",
            "left_target_position_m": np.asarray(left_wp["source_position_m"], dtype=float).tolist(),
            "right_target_position_m": np.asarray(right_wp["source_position_m"], dtype=float).tolist(),
            "left_wrist_roll_rad": float(left_wp["wrist_roll_rad"]),
            "right_wrist_roll_rad": float(right_wp["wrist_roll_rad"]),
        })
    return trace


def _grid_values(spec: dict[str, object]) -> tuple[set[float], set[float], set[float]]:
    ranges = (spec["x_range_m"], spec["y_range_m"], spec["z_range_m"])
    counts = spec["counts"]
    if not isinstance(ranges, tuple) or not isinstance(counts, list) or len(counts) != 3:
        raise ValueError("Malformed T002 workspace-grid specification.")
    return tuple(set(np.linspace(float(axis[0]), float(axis[1]), int(count)).tolist()) for axis, count in zip(ranges, counts))  # type: ignore[return-value]


def _is_declared_grid_target(target: object, grid: tuple[set[float], set[float], set[float]]) -> bool:
    if not isinstance(target, list) or len(target) != 3:
        return False
    return all(any(abs(float(value) - declared) <= 1e-12 for declared in values) for value, values in zip(target, grid))


def _t002_trace(case: dict[str, object], workspace_path: Path) -> list[dict[str, object]]:
    source = json.loads(workspace_path.read_text(encoding="utf-8"))
    grid = source.get("workspace_grid")
    if not isinstance(grid, dict) or not isinstance(grid.get("left"), dict) or not isinstance(grid.get("right"), dict):
        raise ValueError("T002 workspace source does not contain left/right workspace grids.")
    left_grid, right_grid = _grid_values(dict(grid["left"])), _grid_values(dict(grid["right"]))
    steps = case.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("A T002 bilateral case requires non-empty explicit steps.")
    trace: list[dict[str, object]] = []
    for step in steps:
        if not isinstance(step, dict):
            raise ValueError("T002 bilateral step must be an object.")
        left_target, right_target = step.get("left_target_position_m"), step.get("right_target_position_m")
        if not _is_declared_grid_target(left_target, left_grid) or not _is_declared_grid_target(right_target, right_grid):
            raise ValueError(f"T006 T002-grid target is outside the declared source grid: {step!r}")
        trace.append({**step, "source": "T002 declared workspace grid"})
    return trace


def build_trace(case: dict[str, object], config: dict[str, object]) -> tuple[list[dict[str, object]], dict[str, Path]]:
    sources = dict(config["sources"])
    t003 = ROOT / str(sources["t003_waypoints"])
    t002 = ROOT / str(sources["t002_workspace_config"])
    if not t003.is_file() or not t002.is_file():
        raise ValueError("One or more declared T002/T003 source records are missing.")
    source_kind = case.get("source_kind")
    if source_kind == "t003_mirrored_waypoints":
        return _t003_trace(case, t003), {"t003_waypoints": t003, "t002_workspace_config": t002}
    if source_kind == "t002_workspace_grid":
        return _t002_trace(case, t002), {"t003_waypoints": t003, "t002_workspace_config": t002}
    raise ValueError(f"Unknown T006 source_kind {source_kind!r}.")


def _write_trace(path: Path, rows: list[dict[str, object]]) -> None:
    fields = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            encoded = dict(row)
            for key, value in encoded.items():
                if isinstance(value, list):
                    encoded[key] = json.dumps(value)
            writer.writerow(encoded)


def _repeatability_delta(first: list[dict[str, object]], second: list[dict[str, object]]) -> float:
    return max((
        float(max(
            np.max(np.abs(np.asarray(a["left_joint_positions_rad"]) - np.asarray(b["left_joint_positions_rad"]))),
            np.max(np.abs(np.asarray(a["right_joint_positions_rad"]) - np.asarray(b["right_joint_positions_rad"]))),
        )) for a, b in zip(first, second)
    ), default=0.0)


def run_case(config: dict[str, object], config_path: Path, case: dict[str, object], output_dir: Path) -> None:
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T006 evidence: {output_dir}")
    trace, source_paths = build_trace(case, config)
    raw = dict(config["ik"])
    ik = ArmIKConfig(**{key: raw[key] for key in (
        "position_tolerance_m", "roll_tolerance_rad", "max_iterations", "damping",
        "posture_weight", "max_joint_step_rad", "posture_tolerance_rad",
    )})
    offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
    left = load_arm_chain("left", end_effector_offset_m=offset)
    right = load_arm_chain("right", end_effector_offset_m=offset)
    ownership = R1JointOwnership()
    common = dict(
        trace=trace, left_chain=left, right_chain=right, ik_config=ik,
        left_seed_q=np.asarray(raw["left_seed_q_rad"]), right_seed_q=np.asarray(raw["right_seed_q_rad"]),
        left_nominal_q=np.asarray(raw["left_nominal_q_rad"]), right_nominal_q=np.asarray(raw["right_nominal_q_rad"]),
        ownership=ownership,
    )
    first, second = evaluate_bilateral_case(**common), evaluate_bilateral_case(**common)
    audit = arm_ownership_audit(ownership, left, right)
    repeatability = _repeatability_delta(first.rows, second.rows)
    validity = dict(config["validity"])
    verification = {
        "all_declared_sources_present": True,
        "ownership_disjoint": first.ownership_disjoint,
        "ownership_complete_for_arms": first.ownership_complete_for_arms,
        "base_velocity_dispatch_count": 0,
        "repeatability_max_joint_delta_rad": repeatability,
        "repeatability_ok": repeatability <= float(validity["repeatability_tolerance_rad"]),
        "all_trace_values_finite": all(np.isfinite(float(row["endpoint_separation_m"])) for row in first.rows),
        "kinematic_screen_pass": first.all_converged and not first.any_joint_clamped and first.ownership_disjoint and first.ownership_complete_for_arms,
        "collision_status": "inconclusive",
    }
    metrics = {
        "schema_version": 1, "case_id": case["case_id"], "step_count": len(first.rows),
        "all_steps_converged": first.all_converged, "any_joint_clamped": first.any_joint_clamped,
        "minimum_limit_margin_rad": first.minimum_limit_margin_rad,
        "minimum_endpoint_separation_m": first.minimum_endpoint_separation_m,
        "endpoint_separation_definition": "Euclidean distance between solved wrist-roll-link endpoints; kinematic diagnostic only, not collision clearance.",
        "scope": config["scope"], "collision_status": "inconclusive",
    }
    output_dir.mkdir(parents=True)
    _write_trace(output_dir / "bilateral_trace.csv", first.rows)
    write_json(output_dir / "ownership_audit.json", audit)
    write_json(output_dir / "case_metrics.json", metrics)
    write_json(output_dir / "verification.json", verification)
    write_experiment_config(output_dir, config)
    write_resolved_config(output_dir, {"config_path": str(config_path.relative_to(ROOT)), "case": case, "ik": raw, "source_sha256": {key: _sha256(path) for key, path in source_paths.items()}})
    write_runner_command(output_dir)
    write_metadata(output_dir, ROOT, {
        "protocol_id": "t006", "case_id": case["case_id"], "execution_backend": "deterministic_bilateral_ik_and_ownership_audit",
        "hardware_command_channel": "not_opened", "collision_status": "inconclusive",
    })
    write_evidence_completeness(output_dir, {
        "bilateral_trace_csv": True, "ownership_audit": True, "case_metrics": True, "verification": True,
        "source_t003_waypoints": True, "source_t002_workspace_config": True,
        "collision_contact_events": {"present": False, "reason": str(dict(config["limitations"])["collision_reason"])},
        "video": {"present": False, "reason": str(dict(config["limitations"])["video"])},
    })
    completed = all(value is not False for value in verification.values() if isinstance(value, bool))
    write_status(output_dir, "completed" if completed else "failed", "unassessed", "kinematic/ownership evidence recorded; collision status remains inconclusive" if completed else "declared T006 execution/verification check failed")
    print(f"{case['case_id']}: {output_dir.relative_to(ROOT)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--case-id", default="all", help="One declared case ID, or 'all' (default).")
    parser.add_argument("--output-dir", type=Path, help="A new output directory; allowed only for one --case-id.")
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    config = load_config(config_path)
    cases = select_cases(config, args.case_id)
    if args.output_dir is not None and len(cases) != 1:
        raise SystemExit("--output-dir is only valid with one --case-id.")
    RUN_ROOT.mkdir(parents=True, exist_ok=True)
    for case in cases:
        output = args.output_dir.expanduser().resolve() if args.output_dir else RUN_ROOT / allocate_run_id(RUN_ROOT, f"t006_{case['case_id']}")
        run_case(config, config_path, case, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
