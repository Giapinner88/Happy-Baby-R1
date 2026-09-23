#!/usr/bin/env python3
"""Compare one mirrored Isaac/hardware run by shared Quest sequence id."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def read_jsonl(path: Path) -> list[dict[str, object]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def summary(values: list[float]) -> dict[str, object]:
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {"count": 0, "mean": None, "p95": None, "max": None}
    return {
        "count": int(len(array)),
        "mean": float(np.mean(array)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def reorder(values: object, source_names: object, target_names: object) -> np.ndarray:
    source = [str(name) for name in source_names]
    target = [str(name) for name in target_names]
    vector = np.asarray(values, dtype=float)
    if vector.shape != (len(source),) or set(source) != set(target):
        raise ValueError("joint vectors cannot be reordered across different name sets")
    by_name = dict(zip(source, vector))
    return np.asarray([by_name[name] for name in target], dtype=float)


def compare(run_dir: Path) -> dict[str, object]:
    producer_rows = {
        int(row["sequence_id"]): row
        for row in read_jsonl(run_dir / "hardware_targets.jsonl")
        if bool(row.get("operator_enabled", True))
    }
    sim_rows = {}
    for row in json.loads((run_dir / "simulation/targets.json").read_text(encoding="utf-8")):
        application = row.get("whole_upper_body") or {}
        if bool(row.get("enabled")) and bool(application.get("accepted")):
            sim_rows[int(row["sequence_id"])] = row
    robot_rows = {
        int(row["upstream_sequence_id"]): row
        for row in read_jsonl(run_dir / "robot/samples.jsonl")
    }

    producer_sim_error: list[float] = []
    producer_robot_target_error: list[float] = []
    sim_robot_observed_error: list[float] = []
    for sequence in sorted(set(producer_rows) & set(sim_rows)):
        producer = producer_rows[sequence]
        sim = sim_rows[sequence]
        application = dict(sim["whole_upper_body"])
        producer_model = np.asarray(producer["upstream_joint_position_rad"], dtype=float)
        sim_target = reorder(
            application["joint_target_rad"],
            application["controlled_joint_names"],
            producer["upstream_joint_names"],
        )
        producer_sim_error.append(float(np.max(np.abs(producer_model - sim_target))))
        if sequence not in robot_rows:
            continue
        robot = robot_rows[sequence]
        producer_wire = reorder(
            producer_model, producer["upstream_joint_names"], robot["joint_names"]
        )
        robot_target = np.asarray(robot["target_q"], dtype=float)
        producer_robot_target_error.append(
            float(np.max(np.abs(producer_wire - robot_target)))
        )
        sim_observed = reorder(
            sim["post_physics_whole_upper_body_position_rad"],
            application["controlled_joint_names"],
            robot["joint_names"],
        )
        robot_observed = np.asarray(robot["observed_q"], dtype=float)
        sim_robot_observed_error.append(
            float(np.max(np.abs(sim_observed - robot_observed)))
        )

    if not producer_sim_error:
        raise ValueError("no enabled producer/simulator sequence ids matched")
    if not producer_robot_target_error:
        raise ValueError("no producer/robot sequence ids matched")
    command_parity_ok = max(producer_sim_error) <= 1e-9

    return {
        "schema_version": 1,
        "status": "valid" if command_parity_ok else "invalid_command_parity",
        "alignment_key": "Quest sequence_id / robot upstream_sequence_id",
        "command_contract": (
            "Isaac and hardware receive the same workstation-rate-limited joint vector; "
            "robot-side envelope/head gates may still alter the final hardware target."
        ),
        "producer_to_sim_command_max_abs_rad": summary(producer_sim_error),
        "producer_to_robot_target_max_abs_rad": summary(producer_robot_target_error),
        "sim_to_robot_observed_max_abs_rad": summary(sim_robot_observed_error),
        "interpretation": {
            "producer_to_sim": "wiring parity; expected numerical zero",
            "producer_to_robot_target": "robot-side mapping/envelope/head-gate deviation",
            "sim_to_robot_observed": "combined physical tracking/model discrepancy, not pure controller error",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    try:
        result = compare(run_dir)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot compare mirrored run: {exc}") from exc
    output = run_dir / "sim_hardware_comparison.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    return 0 if result["status"] == "valid" else 1


if __name__ == "__main__":
    raise SystemExit(main())
