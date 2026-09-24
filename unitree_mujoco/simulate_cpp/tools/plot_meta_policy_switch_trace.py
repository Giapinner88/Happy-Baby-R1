#!/usr/bin/env python3
"""Plot terrain-conditioned meta-policy transitions from an autotest CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "slope_up": "#D97706",
    "slope_down": "#15803D",
    "flat": "#2563EB",
    "NEUTRAL": "#6B7280",
    "SAFETY_HOLD": "#DC2626",
}

LABELS = {
    "en": {
        "title": "Policy switching on a Flat–15° Up–Flat–15° Down course",
        "terrain": "Terrain grade (deg)",
        "position": "Forward position (m)",
        "probability": "Gate probability",
        "weight": "Expert blend weight",
        "selected": "Selected",
        "relative_time": "Time from switch (s)",
        "tilt": "Relative tilt (deg)",
        "height": "Base height (m)",
        "time": "Time after command onset (s)",
        "crossfade": "nominal 0.2 s crossfade",
        "source": "Measured simulation trace; terrain grade is evaluator-only and is not an actor input.",
    },
    "vi": {
        "title": "Chuyển policy trên tuyến Phẳng–Lên 15°–Phẳng–Xuống 15°",
        "terrain": "Độ dốc địa hình (độ)",
        "position": "Vị trí tiến (m)",
        "probability": "Xác suất của gate",
        "weight": "Trọng số trộn expert",
        "selected": "Đã chọn",
        "relative_time": "Thời gian từ lúc switch (s)",
        "tilt": "Độ nghiêng tương đối (độ)",
        "height": "Độ cao base (m)",
        "time": "Thời gian sau khi phát lệnh (s)",
        "crossfade": "crossfade danh định 0,2 s",
        "source": "Trace mô phỏng đo được; độ dốc chỉ dành cho evaluator, không đi vào actor.",
    },
}


def read_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty trace: {path}")

    numeric = {}
    for name in rows[0]:
        if name == "meta_selected":
            numeric[name] = np.asarray([row[name] for row in rows], dtype=object)
        else:
            numeric[name] = np.asarray(
                [float(row[name]) if row[name] else np.nan for row in rows],
                dtype=float,
            )
    return numeric


def terrain_grade(forward_position: np.ndarray) -> np.ndarray:
    """Evaluator-only grade for the central 15-degree lane in scene_mixed_terrain."""
    grade = np.zeros_like(forward_position)
    grade[(forward_position >= 3.0) & (forward_position < 5.898)] = 15.0
    grade[(forward_position >= 6.898) & (forward_position < 9.796)] = -15.0
    return grade


def transition_indices(classes: np.ndarray) -> list[int]:
    return [index for index in range(1, len(classes)) if classes[index] != classes[index - 1]]


def class_spans(time: np.ndarray, classes: np.ndarray):
    start = 0
    for end in transition_indices(classes) + [len(classes)]:
        yield time[start], time[end - 1], str(classes[start])
        start = end


def plot(trace: dict[str, np.ndarray], output: Path, language: str) -> None:
    text = LABELS[language]
    time = trace["t"]
    classes = trace["meta_selected"]
    forward = trace["drift_x"]
    grade = terrain_grade(forward)

    plt.rcParams.update({
        "font.size": 9.5,
        "axes.titlesize": 10,
        "axes.labelsize": 9.5,
        "legend.fontsize": 8.5,
        "svg.fonttype": "none",
    })
    fig = plt.figure(figsize=(8.2, 8.9))
    grid = fig.add_gridspec(
        4,
        3,
        height_ratios=(1.0, 1.45, 1.0, 1.0),
        hspace=0.62,
        wspace=0.26,
    )
    terrain_axis = fig.add_subplot(grid[0, :])
    probability_axis = fig.add_subplot(grid[1, :], sharex=terrain_axis)
    blend_axes = [fig.add_subplot(grid[2, column]) for column in range(3)]
    stability_axis = fig.add_subplot(grid[3, :], sharex=terrain_axis)
    main_axes = [terrain_axis, probability_axis, stability_axis]
    fig.subplots_adjust(left=0.115, right=0.885, top=0.92, bottom=0.085)
    fig.suptitle(text["title"], fontsize=13, fontweight="bold")

    terrain_axis.step(time, grade, where="post", color="#111827", linewidth=1.8)
    terrain_axis.axhline(0.0, color="#9CA3AF", linewidth=0.7)
    terrain_axis.set_ylabel(text["terrain"])
    terrain_axis.set_ylim(-18, 18)
    pos_axis = terrain_axis.twinx()
    pos_axis.plot(time, forward, color="#7C3AED", linewidth=1.1, alpha=0.8)
    pos_axis.set_ylabel(text["position"], color="#7C3AED")
    pos_axis.tick_params(axis="y", colors="#7C3AED")

    probability_names = [
        ("prob_flat", "Flat", COLORS["flat"]),
        ("prob_slope_up", "Up", COLORS["slope_up"]),
        ("prob_slope_down", "Down", COLORS["slope_down"]),
        ("prob_NEUTRAL", "Neutral", COLORS["NEUTRAL"]),
    ]
    lane_bases = np.asarray([3.0, 2.0, 1.0, 0.0])
    lane_height = 0.72
    for base, (column, _, color) in zip(lane_bases, probability_names):
        values = trace[column]
        probability_axis.axhspan(base, base + lane_height, color="#F9FAFB", zorder=0)
        probability_axis.fill_between(
            time,
            base,
            base + lane_height * values,
            step="post",
            color=color,
            alpha=0.72,
            linewidth=0.0,
            zorder=2,
        )
        probability_axis.step(
            time,
            base + lane_height * values,
            where="post",
            color=color,
            linewidth=0.65,
            zorder=3,
        )
        probability_axis.axhline(base, color="#D1D5DB", linewidth=0.45, zorder=1)

    transitions = transition_indices(classes)
    median_dt = float(np.median(np.diff(time)))
    for span_start, span_end, selected_class in class_spans(time, classes):
        color = COLORS.get(selected_class, COLORS["NEUTRAL"])
        probability_axis.fill_between(
            [span_start, span_end + median_dt],
            -0.48,
            -0.25,
            color=color,
            alpha=0.9,
            linewidth=0.0,
        )

    probability_axis.set_ylabel(text["probability"])
    probability_axis.set_ylim(-0.55, 3.82)
    probability_axis.set_yticks(
        [base + lane_height / 2 for base in lane_bases] + [-0.365],
        [label for _, label, _ in probability_names] + [text["selected"]],
    )
    for tick, (_, _, color) in zip(probability_axis.get_yticklabels()[:4], probability_names):
        tick.set_color(color)
        tick.set_fontweight("bold")

    weight_names = [
        ("weight_flat", "Flat", COLORS["flat"]),
        ("weight_slope_up", "Up", COLORS["slope_up"]),
        ("weight_slope_down", "Down", COLORS["slope_down"]),
    ]
    pretty_class = {
        "flat": "Flat",
        "slope_up": "Up",
        "slope_down": "Down",
        "NEUTRAL": "Neutral",
    }
    for transition_axis, index in zip(blend_axes, transitions):
        transition_time = time[index]
        window = (time >= transition_time - 0.10) & (time <= transition_time + 0.30)
        relative_time = time[window] - transition_time
        for column, label, color in weight_names:
            transition_axis.step(
                relative_time,
                trace[column][window],
                where="post",
                label=label,
                color=color,
                linewidth=1.45,
            )
        old_class = pretty_class.get(str(classes[index - 1]), str(classes[index - 1]))
        new_class = pretty_class.get(str(classes[index]), str(classes[index]))
        transition_axis.set_title(
            f"{old_class} → {new_class}  |  {transition_time:.2f} s",
            fontsize=8.2,
            pad=4,
        )
        transition_axis.axvline(0.0, color="#374151", linewidth=0.7, linestyle="--")
        transition_axis.axvspan(0.0, 0.2, color="#F59E0B", alpha=0.08)
        transition_axis.set_xlim(-0.10, 0.30)
        transition_axis.set_ylim(-0.03, 1.03)
        transition_axis.set_xticks([-0.1, 0.0, 0.1, 0.2, 0.3])
        transition_axis.grid(True, color="#E5E7EB", linewidth=0.55)
        transition_axis.set_axisbelow(True)
    blend_axes[0].set_ylabel(text["weight"])
    blend_axes[1].set_xlabel(text["relative_time"])
    blend_axes[1].legend(
        ncol=3,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.26),
        frameon=False,
        borderaxespad=0.0,
        handlelength=1.2,
        columnspacing=1.0,
        fontsize=7.5,
    )
    for transition_axis in blend_axes[1:]:
        transition_axis.tick_params(labelleft=False)

    tilt_deg = np.rad2deg(trace["tilt_rad"])
    stability_axis.plot(time, tilt_deg, color="#BE123C", linewidth=1.2)
    stability_axis.set_ylabel(text["tilt"], color="#BE123C")
    stability_axis.tick_params(axis="y", colors="#BE123C")
    height_axis = stability_axis.twinx()
    height_axis.plot(time, trace["base_z"], color="#0369A1", linewidth=1.0, alpha=0.8)
    height_axis.set_ylabel(text["height"], color="#0369A1")
    height_axis.tick_params(axis="y", colors="#0369A1")
    stability_axis.set_xlabel(text["time"])

    for index in transitions:
        transition_time = time[index]
        for axis in main_axes:
            axis.axvline(transition_time, color="#374151", linewidth=0.7, linestyle="--")
            axis.axvspan(transition_time, transition_time + 0.2, color="#F59E0B", alpha=0.07)

    for axis in main_axes:
        axis.grid(True, color="#E5E7EB", linewidth=0.6)
        axis.set_axisbelow(True)
    plt.setp(terrain_axis.get_xticklabels(), visible=False)
    plt.setp(probability_axis.get_xticklabels(), visible=False)
    fig.text(0.115, 0.018, text["source"], fontsize=7.5, color="#4B5563")

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    if output.suffix.lower() != ".svg":
        fig.savefig(output.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--language", choices=("en", "vi"), default="en")
    args = parser.parse_args()
    plot(read_trace(args.trace), args.output, args.language)


if __name__ == "__main__":
    main()
