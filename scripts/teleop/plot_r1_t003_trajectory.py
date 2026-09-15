#!/usr/bin/env python3
"""Plot unfiltered target and observed T003 arm joint kinematics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True, help="Completed T003 evidence run.")
    parser.add_argument("--output-dir", type=Path, help="Derived output directory (defaults to the run directory).")
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    source = run_dir / "trajectory.csv"
    if not source.is_file():
        raise SystemExit(f"T003 trajectory CSV does not exist: {source}")
    output_dir = (args.output_dir or run_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    import matplotlib.pyplot as plt

    with source.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("T003 trajectory CSV contains no samples.")
    joint_names = [field[: -len("_target_rad")] for field in rows[0] if field.endswith("_target_rad")]
    time_s = [float(row["sim_time_s"]) for row in rows]
    quantities = (
        ("position", "target_rad", "observed_rad", "rad"),
        ("velocity", "target_vel_rad_s", "observed_vel_rad_s", "rad/s"),
        ("acceleration", "target_acc_rad_s2", "observed_acc_rad_s2", "rad/s²"),
    )
    figure, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    for axis, (title, target_suffix, observed_suffix, unit) in zip(axes, quantities):
        for name in joint_names:
            axis.plot(time_s, [float(row[f"{name}_{observed_suffix}"]) for row in rows], label=name.removesuffix("_joint"))
            axis.plot(time_s, [float(row[f"{name}_{target_suffix}"]) for row in rows], linestyle="--", alpha=0.65)
        axis.set_title(f"Joint {title}: solid observed, dashed commanded")
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel("simulated time (s)")
    axes[0].legend(ncol=3, fontsize=8)
    output = output_dir / "trajectory_joint_kinematics.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    (output_dir / "trajectory_plot_provenance.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "source_csv": str(source),
                "plot": str(output),
                "transformations": "none; direct per-control-step values, no smoothing, resampling, filtering, or interpolation",
                "command": " ".join(__import__("sys").argv),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
