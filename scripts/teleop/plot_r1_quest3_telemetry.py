#!/usr/bin/env python3
"""Plot Quest head/wrist position, velocity, and acceleration from T001 JSONL."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1 import R1TeleopCommand  # noqa: E402
from teleop.r1.bridge import pose_from_matrix  # noqa: E402


SIGNALS = ("head", "left_wrist", "right_wrist")
AXES = ("x", "y", "z")


@dataclass(frozen=True)
class TelemetryTrace:
    timestamp_s: np.ndarray
    sequence_id: np.ndarray
    deadman_enabled: np.ndarray
    position_m: dict[str, np.ndarray]


def load_trace(path: Path) -> TelemetryTrace:
    timestamps: list[float] = []
    sequence_ids: list[int] = []
    deadman: list[bool] = []
    positions = {name: [] for name in SIGNALS}
    previous_timestamp = -1.0
    previous_sequence = -1
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            command = R1TeleopCommand.from_dict(json.loads(line))
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            raise ValueError(f"Invalid R1TeleopCommand at line {line_number}: {exc}") from exc
        if command.timestamp_monotonic_s <= previous_timestamp:
            raise ValueError("Command timestamps must increase strictly for kinematic differentiation.")
        if command.sequence_id <= previous_sequence:
            raise ValueError("Command sequence IDs must increase strictly for kinematic differentiation.")
        previous_timestamp = command.timestamp_monotonic_s
        previous_sequence = command.sequence_id
        timestamps.append(command.timestamp_monotonic_s)
        sequence_ids.append(command.sequence_id)
        deadman.append(command.deadman_enabled)
        positions["head"].append(command.head_pose.position)
        positions["left_wrist"].append(command.left_wrist_pose.position)
        positions["right_wrist"].append(command.right_wrist_pose.position)
    if len(timestamps) < 3:
        raise ValueError("At least three strictly time-ordered commands are required for velocity and acceleration.")
    return TelemetryTrace(
        timestamp_s=np.asarray(timestamps, dtype=float),
        sequence_id=np.asarray(sequence_ids, dtype=int),
        deadman_enabled=np.asarray(deadman, dtype=bool),
        position_m={
            name: np.asarray([[value.x, value.y, value.z] for value in values], dtype=float)
            for name, values in positions.items()
        },
    )


def load_transport_capture(path: Path) -> TelemetryTrace:
    """Read a T001-A `transport_samples.jsonl` capture.

    Two differences from a command trace. Poses arrive as 4x4 matrices rather
    than as a normalized `R1TeleopCommand`, and samples recorded before the
    first `motion_data_ready` carry vendor defaults rather than observed motion,
    so they are dropped instead of being differentiated into fictitious velocity.

    A capture has no deadman field and no sequence id; the deadman column is
    filled from the controller trigger when present, and sequence ids are the
    retained sample index so the CSV stays traceable to the source line.
    """

    timestamps: list[float] = []
    sequence_ids: list[int] = []
    deadman: list[bool] = []
    positions: dict[str, list] = {name: [] for name in SIGNALS}
    matrix_field = {
        "head": "head_pose_matrix",
        "left_wrist": "left_wrist_pose_matrix",
        "right_wrist": "right_wrist_pose_matrix",
    }
    previous_timestamp = -1.0
    seen_motion = False
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid transport sample at line {line_number}: {exc}") from exc
        if not sample.get("motion_data_ready"):
            continue
        seen_motion = True
        timestamp = float(sample["timestamp_monotonic_s"])
        if timestamp <= previous_timestamp:
            raise ValueError("Transport sample timestamps must increase strictly for differentiation.")
        previous_timestamp = timestamp
        timestamps.append(timestamp)
        sequence_ids.append(line_number)
        controller = sample.get("controller") or {}
        right = controller.get("right") or {}
        deadman.append(bool(right.get("trigger", False)))
        for signal, field in matrix_field.items():
            positions[signal].append(pose_from_matrix(sample[field]).position)

    if not seen_motion:
        raise ValueError(
            "This capture contains no motion-ready sample, so it holds no observed Quest motion to plot."
        )
    if len(timestamps) < 3:
        raise ValueError("At least three motion-ready samples are required for velocity and acceleration.")
    return TelemetryTrace(
        timestamp_s=np.asarray(timestamps, dtype=float),
        sequence_id=np.asarray(sequence_ids, dtype=int),
        deadman_enabled=np.asarray(deadman, dtype=bool),
        position_m={
            name: np.asarray([[value.x, value.y, value.z] for value in values], dtype=float)
            for name, values in positions.items()
        },
    )


def detect_format(path: Path) -> str:
    """Pick a loader from the first JSON object's fields."""

    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path} does not start with a JSON object: {exc}") from exc
        if "motion_data_ready" in sample or "head_pose_matrix" in sample:
            return "transport"
        if "sequence_id" in sample or "head_pose" in sample:
            return "command"
        raise ValueError(f"Cannot recognize {path} as a transport capture or a command trace.")
    raise ValueError(f"{path} is empty.")


def segment_ids(timestamp_s: np.ndarray, max_gap_s: float) -> np.ndarray:
    """Label contiguous samples; a derivative is never taken across a gap."""

    if max_gap_s <= 0.0:
        raise ValueError("max_gap_s must be positive.")
    boundaries = np.diff(timestamp_s) > max_gap_s
    return np.cumsum(np.concatenate(([0], boundaries))).astype(int)


def differentiate_position(
    timestamp_s: np.ndarray, position_m: np.ndarray, segment_id: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Central finite-difference velocity and acceleration for each valid segment."""

    velocity = np.full_like(position_m, np.nan, dtype=float)
    acceleration = np.full_like(position_m, np.nan, dtype=float)
    for current_segment in np.unique(segment_id):
        indices = np.flatnonzero(segment_id == current_segment)
        if len(indices) < 3:
            continue
        segment_time = timestamp_s[indices]
        segment_velocity = np.gradient(position_m[indices], segment_time, axis=0, edge_order=2)
        velocity[indices] = segment_velocity
        acceleration[indices] = np.gradient(segment_velocity, segment_time, axis=0, edge_order=2)
    return velocity, acceleration


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_csv(path: Path, trace: TelemetryTrace, segments: np.ndarray, velocity_mps: dict[str, np.ndarray], acceleration_mps2: dict[str, np.ndarray]) -> None:
    fields = ["elapsed_s", "timestamp_monotonic_s", "sequence_id", "deadman_enabled", "segment_id"]
    for signal in SIGNALS:
        for prefix in ("position_m", "velocity_mps", "acceleration_mps2"):
            fields.extend(f"{signal}_{prefix}_{axis}" for axis in AXES)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, timestamp in enumerate(trace.timestamp_s):
            row: dict[str, object] = {"elapsed_s": timestamp - trace.timestamp_s[0], "timestamp_monotonic_s": timestamp, "sequence_id": int(trace.sequence_id[index]), "deadman_enabled": bool(trace.deadman_enabled[index]), "segment_id": int(segments[index])}
            for signal in SIGNALS:
                for axis_index, axis in enumerate(AXES):
                    row[f"{signal}_position_m_{axis}"] = trace.position_m[signal][index, axis_index]
                    row[f"{signal}_velocity_mps_{axis}"] = velocity_mps[signal][index, axis_index]
                    row[f"{signal}_acceleration_mps2_{axis}"] = acceleration_mps2[signal][index, axis_index]
            writer.writerow(row)


def _plot(path: Path, trace: TelemetryTrace, velocity_mps: dict[str, np.ndarray], acceleration_mps2: dict[str, np.ndarray], title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed_s = trace.timestamp_s - trace.timestamp_s[0]
    quantities = (("Position (m)", trace.position_m), ("Velocity (m/s)", velocity_mps), ("Acceleration (m/s²)", acceleration_mps2))
    figure, axes = plt.subplots(3, 3, figsize=(18, 11), sharex="col", constrained_layout=True)
    for row, (label, values) in enumerate(quantities):
        for column, signal in enumerate(SIGNALS):
            axis = axes[row, column]
            for axis_index, axis_name in enumerate(AXES):
                axis.plot(elapsed_s, values[signal][:, axis_index], label=axis_name)
            axis.set_title(signal.replace("_", " ").title())
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            if row == 0:
                axis.legend(loc="best")
            if row == 2:
                axis.set_xlabel("Elapsed time (s)")
    figure.suptitle(title)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="A T001-A transport_samples.jsonl capture or a T001-B raw_commands.jsonl trace.",
    )
    parser.add_argument(
        "--input-format",
        choices=("auto", "transport", "command"),
        default="auto",
        help="Source format; 'auto' detects it from the first record.",
    )
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory for derived outputs; never overwritten.")
    parser.add_argument("--max-gap-s", type=float, default=0.2, help="Split a derivative segment when adjacent command timestamps exceed this interval.")
    parser.add_argument("--title", default="R1 Quest transport kinematics", help="Title for the output PNG.")
    args = parser.parse_args()
    input_path = args.input.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not input_path.is_file():
        raise SystemExit(f"Input command stream does not exist: {input_path}")
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite analysis output: {output_dir}")

    source_format = detect_format(input_path) if args.input_format == "auto" else args.input_format
    try:
        trace = load_transport_capture(input_path) if source_format == "transport" else load_trace(input_path)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    segments = segment_ids(trace.timestamp_s, args.max_gap_s)
    velocity_mps: dict[str, np.ndarray] = {}
    acceleration_mps2: dict[str, np.ndarray] = {}
    for signal in SIGNALS:
        velocity_mps[signal], acceleration_mps2[signal] = differentiate_position(trace.timestamp_s, trace.position_m[signal], segments)

    output_dir.mkdir(parents=True)
    _write_csv(output_dir / "telemetry_kinematics.csv", trace, segments, velocity_mps, acceleration_mps2)
    _plot(output_dir / "telemetry_kinematics.png", trace, velocity_mps, acceleration_mps2, args.title)
    summary = {
        "source": str(input_path),
        "source_format": source_format,
        "source_sha256": _sha256(input_path),
        "sample_count": len(trace.timestamp_s),
        "elapsed_s": float(trace.timestamp_s[-1] - trace.timestamp_s[0]),
        "deadman_enabled_sample_count": int(np.count_nonzero(trace.deadman_enabled)),
        "segment_count": int(np.max(segments) + 1),
        "max_gap_s": args.max_gap_s,
        "derivative_method": "numpy.gradient central finite differences; segments with fewer than three samples are NaN",
        "signals": list(SIGNALS),
        "units": {"position": "m", "velocity": "m/s", "acceleration": "m/s^2"},
    }
    if source_format == "transport":
        summary["transport_note"] = (
            "Samples before the first motion_data_ready carry vendor defaults and were dropped; "
            "sequence_id is the source line number and deadman_enabled is the right controller trigger."
        )
    (output_dir / "analysis_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Telemetry analysis written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
