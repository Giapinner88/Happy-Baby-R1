#!/usr/bin/env python3
"""Aggregate one complete T004 calibration-study execution without judging it.

The aggregator refuses missing or duplicate selected cases. It writes a complete
case table and descriptive summary only; scientific interpretation belongs in a
separate T004 analysis record.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


T004_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T004"
RUN_ROOT = T004_ROOT / "runs"
DEFAULT_CONFIG = T004_ROOT / "config" / "r1_t004_calibration_study.json"


def load_json(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"Expected an object in {path}")
    return payload


def collect_cases(config: dict[str, object], run_root: Path) -> list[dict[str, object]]:
    expected = {str(case["case_id"]): dict(case) for case in config["cases"]}  # type: ignore[index]
    found: dict[str, list[tuple[Path, dict[str, object], dict[str, object]]]] = {case_id: [] for case_id in expected}
    for run_dir in sorted(run_root.glob("t004_*")):
        metrics_path = run_dir / "case_metrics.json"
        status_path = run_dir / "status.json"
        if not metrics_path.is_file() or not status_path.is_file():
            continue
        metrics = load_json(metrics_path)
        case_id = str(metrics.get("case_id", ""))
        if case_id in found:
            found[case_id].append((run_dir, metrics, load_json(status_path)))
    missing = [case_id for case_id, entries in found.items() if not entries]
    duplicates = {case_id: entries for case_id, entries in found.items() if len(entries) > 1}
    if missing or duplicates:
        parts = []
        if missing:
            parts.append(f"missing={missing}")
        if duplicates:
            parts.append(f"duplicate={sorted(duplicates)}")
        raise SystemExit("T004 aggregate selection is incomplete: " + "; ".join(parts))
    rows: list[dict[str, object]] = []
    for case_id, case in expected.items():
        run_dir, metrics, status = found[case_id][0]
        verification = load_json(run_dir / "verification.json")
        rows.append({
            "case_id": case_id,
            "run_id": run_dir.name,
            "run_path": str(run_dir.relative_to(ROOT)),
            "side": metrics["side"],
            "yaw_rad": metrics["calibration"]["yaw_rad"],  # type: ignore[index]
            "translation_m": metrics["calibration"]["translation_m"],  # type: ignore[index]
            "execution_status": status["execution_status"],
            "scientific_outcome": status["scientific_outcome"],
            "all_waypoints_converged": metrics["all_waypoints_converged"],
            "any_joint_clamped": metrics["any_joint_clamped"],
            "usable_trace_by_declared_ik_rule": metrics["usable_trace_by_declared_ik_rule"],
            "max_target_displacement_m": metrics["max_target_displacement_m"],
            "max_position_residual_m": metrics["max_position_residual_m"],
            "minimum_limit_margin_rad": metrics["minimum_limit_margin_rad"],
            "repeatability_max_joint_delta_rad": verification["repeatability_max_joint_delta_rad"],
            "repeatability_ok": verification["repeatability_ok"],
            "identity_mapping_checked": verification["identity_mapping_checked"],
            "identity_mapping_ok": verification["identity_mapping_ok"],
        })
    return rows


def write_aggregate(rows: list[dict[str, object]], output_dir: Path, config_path: Path) -> None:
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T004 aggregate: {output_dir}")
    output_dir.mkdir(parents=True)
    fields = list(rows[0]) if rows else []
    with (output_dir / "case_table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            encoded = dict(row)
            encoded["translation_m"] = json.dumps(encoded["translation_m"])
            writer.writerow(encoded)
    summary = {
        "schema_version": 1,
        "study_id": "t004",
        "config": str(config_path.relative_to(ROOT)),
        "case_count_expected_and_selected": len(rows),
        "execution_status_counts": {
            status: sum(row["execution_status"] == status for row in rows)
            for status in sorted({str(row["execution_status"]) for row in rows})
        },
        "scientific_outcome_counts": {
            outcome: sum(row["scientific_outcome"] == outcome for row in rows)
            for outcome in sorted({str(row["scientific_outcome"]) for row in rows})
        },
        "usable_by_declared_ik_rule_count": sum(bool(row["usable_trace_by_declared_ik_rule"]) for row in rows),
        "nonconverged_case_count": sum(not bool(row["all_waypoints_converged"]) for row in rows),
        "clamped_case_count": sum(bool(row["any_joint_clamped"]) for row in rows),
        "repeatability_failure_count": sum(not bool(row["repeatability_ok"]) for row in rows),
        "aggregation_rule": "Every declared case is included exactly once; no case is filtered by success or scientific outcome.",
        "interpretation": "not performed by this aggregation",
    }
    (output_dir / "aggregate_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-root", type=Path, default=RUN_ROOT)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    rows = collect_cases(load_json(config_path), args.run_root.expanduser().resolve())
    write_aggregate(rows, args.output_dir.expanduser().resolve(), config_path)
    print(f"Wrote {len(rows)} selected T004 cases to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
