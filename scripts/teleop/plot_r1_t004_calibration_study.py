#!/usr/bin/env python3
"""Render the unfiltered initial T004 calibration-study case table."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-table", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = args.case_table.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"T004 case table does not exist: {source}")
    output_dir.mkdir(parents=True, exist_ok=True)

    with source.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit("T004 case table is empty.")
    if any(row["all_waypoints_converged"] != "True" or row["any_joint_clamped"] != "False" for row in rows):
        failure_note = "A cross marker denotes non-convergence or a joint clamp."
    else:
        failure_note = "All displayed cases converged without a joint clamp."

    import matplotlib.pyplot as plt

    labels = [row["case_id"].replace("right_", "R ").replace("left_", "L ") for row in rows]
    positions = list(range(len(rows)))
    colors = ["#1f77b4" if row["side"] == "right" else "#ff7f0e" for row in rows]
    markers = ["o" if row["all_waypoints_converged"] == "True" and row["any_joint_clamped"] == "False" else "x" for row in rows]
    margin = [float(row["minimum_limit_margin_rad"]) for row in rows]
    displacement_mm = [1000.0 * float(row["max_target_displacement_m"]) for row in rows]

    figure, axes = plt.subplots(2, 1, figsize=(15, 9), sharex=True, constrained_layout=True)
    for axis, values, ylabel, title in (
        (axes[0], margin, "minimum reported joint-limit margin (rad)", "IK limit-margin screening"),
        (axes[1], displacement_mm, "maximum mapped-target displacement (mm)", "Calibration-induced target displacement"),
    ):
        for x, value, color, marker in zip(positions, values, colors, markers):
            axis.scatter(x, value, color=color, marker=marker, s=42, zorder=3)
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_ylabel(ylabel)
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.25)
    axes[1].set_xticks(positions, labels, rotation=60, ha="right", fontsize=8)
    axes[1].set_xlabel("predeclared case (R = recorded right trace; L = mirrored synthetic left trace)")
    figure.suptitle("T004 initial calibration sensitivity — 22 unfiltered mapper + IK cases", fontsize=14)
    axes[0].text(
        0.99,
        0.04,
        failure_note,
        transform=axes[0].transAxes,
        ha="right",
        va="bottom",
        fontsize=8,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8},
    )
    output = output_dir / "calibration_sensitivity.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    provenance = {
        "schema_version": 1,
        "source_case_table": str(source),
        "output": str(output),
        "case_count": len(rows),
        "quantities": {
            "panel_a": "minimum_limit_margin_rad from case_metrics.json",
            "panel_b": "1000 * max_target_displacement_m from case_metrics.json",
        },
        "transformations": "unit conversion m to mm only for panel B; no filtering, smoothing, interpolation, normalization, or case exclusion",
        "failure_encoding": "x marker for non-convergence or a joint clamp; circle otherwise",
        "command": " ".join(sys.argv),
    }
    (output_dir / "calibration_sensitivity_plot_provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
