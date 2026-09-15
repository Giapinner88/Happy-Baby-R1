#!/usr/bin/env python3
"""Execute one or all declared T004 calibration-sensitivity cases.

This is a deterministic, simulation-only mapper + IK study. It does not start
Isaac Lab, contact simulation, Quest transport, DDS, ROS, Unitree SDK, or any
hardware command channel. Each case gets its own immutable evidence directory.
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
from teleop.r1.calibration_study import evaluate_calibration_case, load_t003_waypoints  # noqa: E402
from teleop.r1.ik import ArmIKConfig  # noqa: E402
from teleop.r1.kinematics import load_arm_chain  # noqa: E402


T004_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T004"
RUN_ROOT = T004_ROOT / "runs"
DEFAULT_CONFIG = T004_ROOT / "config" / "r1_t004_calibration_study.json"
PROTOCOL = "t004"


def load_config(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load T004 study config {path}: {exc}") from exc
    if payload.get("study_id") != "t004" or payload.get("mode") != "simulation_only":
        raise SystemExit("T004 requires study_id='t004' and mode='simulation_only'.")
    cases = payload.get("cases")
    if not isinstance(cases, list) or not cases:
        raise SystemExit("T004 config must contain a non-empty case list.")
    case_ids = [item.get("case_id") for item in cases if isinstance(item, dict)]
    if len(case_ids) != len(cases) or any(not isinstance(value, str) for value in case_ids):
        raise SystemExit("Every T004 case must have a string case_id.")
    if len(set(case_ids)) != len(case_ids):
        raise SystemExit("T004 case IDs must be unique.")
    return payload


def select_cases(config: dict[str, object], case_id: str) -> list[dict[str, object]]:
    cases = [dict(item) for item in config["cases"]]  # type: ignore[index]
    if case_id == "all":
        return cases
    selected = [item for item in cases if item["case_id"] == case_id]
    if len(selected) != 1:
        known = [str(item["case_id"]) for item in cases]
        raise SystemExit(f"Unknown T004 case {case_id!r}; choose one of {known} or 'all'.")
    return selected


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_trace(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "waypoint_index", "waypoint", "source_x_m", "source_y_m", "source_z_m",
        "mapped_x_m", "mapped_y_m", "mapped_z_m", "target_displacement_m",
        "wrist_roll_rad", "converged", "solver_status", "iterations",
        "position_residual_m", "roll_residual_rad", "limit_margin_rad",
        "clamped_joints", "joint_positions_rad", "base_velocity_zero",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            encoded = dict(row)
            for key in ("clamped_joints", "joint_positions_rad"):
                encoded[key] = json.dumps(encoded[key])
            writer.writerow(encoded)


def _run_case(config: dict[str, object], config_path: Path, case: dict[str, object], output_dir: Path) -> None:
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T004 evidence: {output_dir}")
    source_run = ROOT / str(config["source_t003_run"])
    source_waypoints = source_run / str(config["source_waypoints_file"])
    if not source_waypoints.is_file():
        raise SystemExit(f"Declared T003 source waypoint evidence is missing: {source_waypoints}")
    side = str(case["side"])
    calibration = dict(case["calibration"])
    trace = load_t003_waypoints(source_waypoints, side)
    ik_raw = dict(config["ik"])
    ik_config = ArmIKConfig(
        position_tolerance_m=float(ik_raw["position_tolerance_m"]),
        roll_tolerance_rad=float(ik_raw["roll_tolerance_rad"]),
        max_iterations=int(ik_raw["max_iterations"]),
        damping=float(ik_raw["damping"]),
        posture_weight=float(ik_raw["posture_weight"]),
        max_joint_step_rad=float(ik_raw["max_joint_step_rad"]),
        posture_tolerance_rad=float(ik_raw["posture_tolerance_rad"]),
    )
    offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
    chain = load_arm_chain(side, end_effector_offset_m=offset)
    common = dict(
        source_trace=trace,
        side=side,
        calibration_translation_m=np.asarray(calibration["translation_m"], dtype=float),
        calibration_yaw_rad=float(calibration["yaw_rad"]),
        chain=chain,
        ik_config=ik_config,
        seed_q=np.asarray(ik_raw["seed_q_rad"], dtype=float),
        nominal_q=np.asarray(ik_raw["nominal_q_rad"], dtype=float),
    )
    first = evaluate_calibration_case(**common)
    second = evaluate_calibration_case(**common)
    repeatability = max(
        (
            float(np.max(np.abs(np.asarray(left["joint_positions_rad"]) - np.asarray(right["joint_positions_rad"]))))
            for left, right in zip(first.rows, second.rows)
        ),
        default=0.0,
    )
    validity = dict(config["case_validity"])
    identity_expected = bool(
        np.allclose(np.asarray(calibration["translation_m"], dtype=float), 0.0)
        and float(calibration["yaw_rad"]) == 0.0
    )
    verification = {
        "source_waypoints_present": True,
        "source_waypoint_sha256": _sha256(source_waypoints),
        "all_waypoints_converged": first.all_converged,
        "no_joint_clamp": not first.any_joint_clamped,
        "base_velocity_nonzero_count": first.base_velocity_nonzero_count,
        "identity_mapping_checked": identity_expected,
        "identity_mapping_ok": (
            first.identity_mapping_max_error_m <= float(validity["identity_mapping_tolerance_m"])
            if identity_expected else None
        ),
        "repeatability_max_joint_delta_rad": repeatability,
        "repeatability_ok": repeatability <= float(validity["repeatability_tolerance_rad"]),
        "usable_trace_by_declared_ik_rule": first.all_converged and not first.any_joint_clamped,
    }
    metrics = {
        "schema_version": 1,
        "case_id": case["case_id"],
        "side": side,
        "calibration": calibration,
        "waypoint_count": len(first.rows),
        "all_waypoints_converged": first.all_converged,
        "any_joint_clamped": first.any_joint_clamped,
        "max_position_residual_m": first.max_position_residual_m,
        "max_target_displacement_m": first.max_target_displacement_m,
        "minimum_limit_margin_rad": first.minimum_limit_margin_rad,
        "identity_mapping_max_error_m": first.identity_mapping_max_error_m,
        "usable_trace_by_declared_ik_rule": verification["usable_trace_by_declared_ik_rule"],
        "scope": "deterministic mapper + IK only; no Isaac dynamics, collision/contact, transport, or hardware",
    }
    output_dir.mkdir(parents=True)
    _write_trace(output_dir / "trace.csv", first.rows)
    write_json(output_dir / "case_metrics.json", metrics)
    write_json(output_dir / "verification.json", verification)
    write_experiment_config(output_dir, config)
    write_resolved_config(output_dir, {"config_path": str(config_path.relative_to(ROOT)), "case": case, "ik": ik_raw})
    write_runner_command(output_dir)
    write_metadata(output_dir, ROOT, {
        "protocol_id": PROTOCOL,
        "case_id": case["case_id"],
        "source_t003_waypoints": str(source_waypoints.relative_to(ROOT)),
        "source_t003_waypoints_sha256": _sha256(source_waypoints),
        "execution_backend": "deterministic_mapper_and_ik",
        "hardware_command_channel": "not_opened",
    })
    write_evidence_completeness(output_dir, {
        "trace_csv": True,
        "case_metrics": True,
        "verification": True,
        "source_t003_waypoints": True,
        "video": {"present": False, "reason": "not applicable: no physics or rendering backend in this geometric study"},
    })
    reason = "all declared mapper/IK verification checks passed" if all(
        value is not False for value in verification.values() if isinstance(value, bool) or value is None
    ) else "one or more declared mapper/IK verification checks failed; inspect verification.json"
    write_status(output_dir, "completed", "unassessed", reason)
    print(f"{case['case_id']}: {output_dir.relative_to(ROOT)}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--case-id", default="all", help="One declared case ID, or 'all' (default).")
    parser.add_argument("--output-dir", type=Path, help="A new output directory; permitted only for one case.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config_path = args.config.expanduser().resolve()
    config = load_config(config_path)
    cases = select_cases(config, args.case_id)
    if args.output_dir is not None and len(cases) != 1:
        raise SystemExit("--output-dir is only valid with one --case-id.")
    RUN_ROOT.mkdir(parents=True, exist_ok=True)
    for case in cases:
        if args.output_dir is not None:
            output_dir = args.output_dir.expanduser().resolve()
        else:
            prefix = f"{PROTOCOL}_{case['case_id']}"
            output_dir = RUN_ROOT / allocate_run_id(RUN_ROOT, prefix)
        _run_case(config, config_path, case, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
