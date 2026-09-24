#!/usr/bin/env python3
"""Aggregate meta-policy/expert acceptance CSV files without extra dependencies."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


CASE_KEYS = ("scene", "angle_deg", "category", "direction", "vx", "vy", "yaw")


def resolve_artifact(path_text: str, run_directory: Path) -> Path:
    """Resolve absolute and launcher-relative artifact paths reproducibly."""
    path = Path(path_text)
    if path.is_absolute():
        return path

    candidates = [Path.cwd() / path]
    candidates.extend(parent / path for parent in (run_directory, *run_directory.parents))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def classify_failure(row: dict[str, str], run_directory: Path) -> str:
    if row.get("control_result") == "PASS":
        return "pass"
    log_path = resolve_artifact(row.get("log", ""), run_directory)
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        return "missing_log"
    if "fall detector" in text:
        return "fall"
    if "RESULT=FAIL" in text:
        return "metric_violation"
    return "missing_result"


def classify_process(row: dict[str, str], run_directory: Path) -> str:
    if row.get("process_result") != "ERROR":
        return "ok"
    log_path = resolve_artifact(row.get("log", ""), run_directory)
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        return "missing_log"
    if "EntityDelegate::prevent_callbacks(): Assertion" in text:
        return "cyclonedds_teardown_assertion"
    if "RESULT=PASS" not in text and "RESULT=FAIL" not in text:
        return "missing_control_result"
    return "other_process_error"


def load_run(directory: Path) -> dict:
    summary_path = directory / "summary.csv"
    with summary_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    reasons = Counter(classify_failure(row, directory) for row in rows)
    process = Counter(row.get("process_result", "UNKNOWN") for row in rows)
    process_reasons = Counter(classify_process(row, directory) for row in rows)
    by_angle: dict[str, Counter] = defaultdict(Counter)
    by_category: dict[str, Counter] = defaultdict(Counter)
    for row in rows:
        outcome = row.get("control_result", "UNKNOWN").lower()
        by_angle[row.get("angle_deg", "unknown")][outcome] += 1
        by_category[row.get("category", "unknown")][outcome] += 1
    return {
        "directory": str(directory.resolve()),
        "cases": len(rows),
        "control": dict(Counter(row.get("control_result", "UNKNOWN") for row in rows)),
        "failure_reason": dict(reasons),
        "process": dict(process),
        "process_reason": dict(process_reasons),
        "by_angle": {key: dict(value) for key, value in by_angle.items()},
        "by_category": {key: dict(value) for key, value in by_category.items()},
    }


def load_case_outcomes(directory: Path) -> dict[tuple[str, ...], bool]:
    with (directory / "summary.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {
        tuple(row.get(key, "") for key in CASE_KEYS):
        row.get("control_result") == "PASS"
        for row in rows
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    runs = [load_run(directory) for directory in args.directories]
    aggregate_control = Counter()
    aggregate_reason = Counter()
    aggregate_process = Counter()
    aggregate_process_reason = Counter()
    for run in runs:
        aggregate_control.update(run["control"])
        aggregate_reason.update(run["failure_reason"])
        aggregate_process.update(run["process"])
        aggregate_process_reason.update(run["process_reason"])
    case_maps = [load_case_outcomes(directory) for directory in args.directories]
    case_consistency = None
    if case_maps and all(set(case_map) == set(case_maps[0]) for case_map in case_maps[1:]):
        pass_histogram = Counter(
            sum(case_map[key] for case_map in case_maps)
            for key in case_maps[0]
        )
        run_count = len(case_maps)
        case_consistency = {
            "cases": len(case_maps[0]),
            "reset_repetitions": run_count,
            "pass_count_histogram": {
                f"{pass_count}_of_{run_count}": pass_histogram.get(pass_count, 0)
                for pass_count in range(run_count + 1)
            },
            "note": "Reset consistency only; the runs do not expose independent random seeds or injected noise.",
        }
    report = {
        "runs": runs,
        "aggregate": {
            "runs": len(runs),
            "cases": sum(run["cases"] for run in runs),
            "control": dict(aggregate_control),
            "failure_reason": dict(aggregate_reason),
            "process": dict(aggregate_process),
            "process_reason": dict(aggregate_process_reason),
        },
        "case_consistency": case_consistency,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)


if __name__ == "__main__":
    main()
