#!/usr/bin/env python3
"""Summarize the compact closed-loop meta-policy ablation screen."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path


def failure_reason(row: dict[str, str]) -> str:
    if row["control_result"] == "PASS":
        return "pass"
    try:
        log = Path(row["log"]).read_text(errors="replace")
    except OSError:
        return "missing_log"
    if "fall detector" in log:
        return "fall"
    if "RESULT=FAIL" in log:
        return "metric_violation"
    return "missing_result"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()

    variants = []
    with (args.root / "runs.csv").open(newline="") as stream:
        run_rows = list(csv.DictReader(stream))
    for run_row in run_rows:
        summary = Path(run_row["summary"])
        with summary.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        outcomes = {row["case"]: row["control_result"] for row in rows}
        reasons = Counter(failure_reason(row) for row in rows)
        process = Counter(row["process_result"] for row in rows)
        variants.append(
            {
                "variant": run_row["variant"],
                "suite_exit_code": int(run_row["exit_code"]),
                "control_pass": sum(value == "PASS" for value in outcomes.values()),
                "cases": len(rows),
                "outcomes": outcomes,
                "failure_reason": dict(reasons),
                "process": dict(process),
            }
        )

    print(
        json.dumps(
            {
                "protocol": "Five-case transition-focused closed-loop screen; not a replacement for the 64-case matrix.",
                "variants": variants,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
