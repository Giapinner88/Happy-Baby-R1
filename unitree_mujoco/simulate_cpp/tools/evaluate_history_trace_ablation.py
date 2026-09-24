#!/usr/bin/env python3
"""Counterfactual history truncation on a recorded 50-step mixed trace."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import torch

from replay_selector_ablations import Config, replay


CLASS_NAMES = np.asarray(("slope_up", "slope_down", "flat", "NEUTRAL"))


def load_meta_module(path: Path):
    spec = importlib.util.spec_from_file_location("history_meta_policy_module", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def oracle_role(position: np.ndarray) -> np.ndarray:
    result = np.full(len(position), "flat", dtype="<U16")
    result[(position >= 3.0) & (position < 5.898)] = "slope_up"
    result[(position >= 6.898) & (position < 9.796)] = "slope_down"
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("model_source", type=Path)
    parser.add_argument("--batch-size", type=int, default=256)
    args = parser.parse_args()

    with args.trace.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    observations = np.asarray(
        [[float(row[f"obs_{index}"]) for index in range(83)] for row in rows],
        dtype=np.float32,
    )
    time = np.asarray([float(row["t"]) for row in rows])
    position = np.asarray([float(row["drift_x"]) for row in rows])
    actual_class = np.asarray([row["meta_selected"] for row in rows])
    actual_weights = np.asarray([
        [float(row[f"weight_{name}"]) for name in CLASS_NAMES[:3]] for row in rows
    ])

    module = load_meta_module(args.model_source)
    model = module.load_slope_meta_checkpoint(
        str(args.checkpoint),
        map_location="cpu",
        expected_expert_names=tuple(CLASS_NAMES[:3]),
    ).eval()
    full_history = 50
    start = full_history - 1
    indices = np.arange(start, len(observations))
    windows = np.stack([
        observations[index - full_history + 1 : index + 1] for index in indices
    ])
    oracle = oracle_role(position[start:])

    outputs = {}
    with torch.inference_mode():
        for retained in (1, 10, 25, 50):
            probabilities = []
            for batch_start in range(0, len(windows), args.batch_size):
                sequence = windows[batch_start : batch_start + args.batch_size].copy()
                sequence[:, : full_history - retained] = 0.0
                logits, _ = model(torch.from_numpy(sequence))
                probabilities.append(torch.softmax(logits[:, -1, :], dim=-1).numpy())
            probability = np.concatenate(probabilities)
            gate_class = CLASS_NAMES[np.argmax(probability, axis=1)]
            selector_class, selector_weights = replay(
                Config(),
                probability,
                actual_weights[start:],
                actual_class[start:],
            )
            transitions = np.flatnonzero(selector_class[1:] != selector_class[:-1]) + 1
            entropy = -np.sum(probability * np.log(np.clip(probability, 1.0e-12, 1.0)), axis=1)
            outputs[str(retained)] = {
                "gate_oracle_accuracy": float(np.mean(gate_class == oracle)),
                "selector_oracle_accuracy": float(np.mean(selector_class == oracle)),
                "selector_agreement_with_50_step_cpp": float(
                    np.mean(selector_class == actual_class[start:])
                ),
                "mean_entropy": float(np.mean(entropy)),
                "switches": int(len(transitions)),
                "weight_total_variation": float(
                    np.abs(np.diff(selector_weights, axis=0)).sum()
                ),
                "transitions": [
                    {
                        "time_s": float(time[start + index]),
                        "position_m": float(position[start + index]),
                        "from": str(selector_class[index - 1]),
                        "to": str(selector_class[index]),
                    }
                    for index in transitions
                ],
            }
    report = {
        "trace": str(args.trace.resolve()),
        "checkpoint": str(args.checkpoint.resolve()),
        "evaluated_samples": len(indices),
        "controlled_by": "50-step C++ baseline; shorter histories are counterfactual replay",
        "history_ablation": outputs,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
