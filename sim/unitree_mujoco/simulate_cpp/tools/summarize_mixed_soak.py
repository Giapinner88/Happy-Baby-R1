#!/usr/bin/env python3
"""Summarize recorded mixed-shuttle meta-policy soak traces."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def phase_reached(time_s: float) -> str:
    if time_s < 48.0:
        return "OUTBOUND"
    if time_s < 54.0:
        return "TURNAROUND_STOP"
    if time_s < 102.0:
        return "INBOUND"
    return "FINAL_STOP"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()

    with (args.directory / "summary.csv").open(newline="") as stream:
        summary = list(csv.DictReader(stream))

    runs = []
    for item in summary:
        trace_path = Path(item["csv"])
        with trace_path.open(newline="") as stream:
            trace = list(csv.DictReader(stream))
        transitions = []
        previous = trace[0]["meta_selected"]
        for row in trace[1:]:
            selected = row["meta_selected"]
            if selected != previous:
                transitions.append(
                    {
                        "time_s": float(row["t"]),
                        "position_x_m": float(row["drift_x"]),
                        "from": previous,
                        "to": selected,
                    }
                )
                previous = selected
        end = trace[-1]
        runs.append(
            {
                "run": int(item["run"]),
                "control_result": item["control_result"],
                "process_result": item["process_result"],
                "samples": len(trace),
                "duration_s": float(end["t"]),
                "phase_reached": phase_reached(float(end["t"])),
                "end_position_x_m": float(end["drift_x"]),
                "end_position_y_m": float(end["drift_y"]),
                "end_tilt_deg": float(end["tilt_rad"]) * 180.0 / 3.141592653589793,
                "selected_at_end": end["meta_selected"],
                "safety_hold_samples": sum(
                    int(row["meta_safety_hold"]) for row in trace
                ),
                "class_switches": len(transitions),
                "transitions": transitions,
            }
        )

    print(
        json.dumps(
            {
                "protocol": "108 s OUTBOUND (48 s), STOP (6 s), INBOUND (48 s), FINAL_STOP (6 s)",
                "control_pass": sum(run["control_result"] == "PASS" for run in runs),
                "runs": runs,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
