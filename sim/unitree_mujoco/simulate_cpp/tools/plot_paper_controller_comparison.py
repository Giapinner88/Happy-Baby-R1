#!/usr/bin/env python3
"""Plot reset-repetition controller outcomes for the meta-policy paper."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
  "meta": "#6C5CE7",
  "flat": "#377BE8",
  "up": "#E58A1F",
  "down": "#2E8B57",
}


def load(path: Path) -> dict:
  with path.open() as stream:
    return json.load(stream)


def run_pass_rate(run: dict) -> float:
  return 100.0 * run["control"].get("PASS", 0) / run["cases"]


def angle_pass_rate(report: dict, angle: str) -> float:
  passed = 0
  total = 0
  for run in report["runs"]:
    counts = run["by_angle"].get(angle, {})
    passed += counts.get("pass", 0)
    total += counts.get("pass", 0) + counts.get("fail", 0)
  return 100.0 * passed / total if total else float("nan")


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--meta", type=Path, required=True)
  parser.add_argument("--flat", type=Path, required=True)
  parser.add_argument("--up", type=Path, required=True)
  parser.add_argument("--down", type=Path, required=True)
  parser.add_argument("--output-svg", type=Path, required=True)
  parser.add_argument("--output-pdf", type=Path, required=True)
  parser.add_argument("--language", choices=("en", "vi"), default="en")
  args = parser.parse_args()

  reports = {
    "meta": load(args.meta),
    "flat": load(args.flat),
    "up": load(args.up),
    "down": load(args.down),
  }
  labels = {
    "en": {
      "names": ("Meta-policy", "Always-Flat", "Always-Up", "Always-Down"),
      "overall": "Overall pass rate across reset repetitions",
      "angle": "Pass rate by terrain angle",
      "ylabel": "Pass rate (%)",
      "xlabel": "Terrain angle (deg)",
      "note": "Bars are means; dots are reset repetitions; no confidence intervals are implied.",
    },
    "vi": {
      "names": ("Meta-policy", "Luôn-Flat", "Luôn-Up", "Luôn-Down"),
      "overall": "Pass rate tổng qua các lần reset",
      "angle": "Pass rate theo góc địa hình",
      "ylabel": "Pass rate (%)",
      "xlabel": "Góc địa hình (độ)",
      "note": "Cột là mean; mỗi chấm là một lần reset; không ngụ ý confidence interval.",
    },
  }[args.language]

  keys = ("meta", "flat", "up", "down")
  figure, axes = plt.subplots(1, 2, figsize=(10.6, 4.1), constrained_layout=True)

  axis = axes[0]
  x = np.arange(len(keys))
  for index, key in enumerate(keys):
    values = np.asarray([run_pass_rate(run) for run in reports[key]["runs"]])
    mean = float(values.mean())
    axis.bar(index, mean, width=0.66, color=COLORS[key], alpha=0.82)
    offsets = np.linspace(-0.10, 0.10, len(values)) if len(values) > 1 else np.zeros(1)
    axis.plot(index + offsets, values, "o", color="#1E2430", ms=4)
    axis.text(index, mean + 2.0, f"{mean:.1f}", ha="center", va="bottom", fontsize=9)
  axis.set_xticks(x, labels["names"], rotation=15, ha="right")
  axis.set_ylim(0, 110)
  axis.set_ylabel(labels["ylabel"])
  axis.set_title(labels["overall"], weight="bold")
  axis.grid(axis="y", alpha=0.25)

  axis = axes[1]
  angles = ("0", "5", "10", "15", "20", "25", "30")
  angle_values = np.asarray([int(angle) for angle in angles])
  for key, name in zip(keys, labels["names"], strict=True):
    values = [angle_pass_rate(reports[key], angle) for angle in angles]
    axis.plot(
      angle_values,
      values,
      marker="o",
      linewidth=2,
      markersize=4,
      color=COLORS[key],
      label=name,
    )
  axis.set_xlim(-1, 31)
  axis.set_ylim(0, 105)
  axis.set_xticks(angle_values)
  axis.set_xlabel(labels["xlabel"])
  axis.set_ylabel(labels["ylabel"])
  axis.set_title(labels["angle"], weight="bold")
  axis.grid(alpha=0.25)
  axis.legend(frameon=False, fontsize=8, loc="lower left")

  figure.text(0.5, -0.02, labels["note"], ha="center", fontsize=8, color="#4C5566")
  args.output_svg.parent.mkdir(parents=True, exist_ok=True)
  args.output_pdf.parent.mkdir(parents=True, exist_ok=True)
  figure.savefig(args.output_svg, bbox_inches="tight")
  figure.savefig(args.output_pdf, bbox_inches="tight")
  plt.close(figure)


if __name__ == "__main__":
  main()
