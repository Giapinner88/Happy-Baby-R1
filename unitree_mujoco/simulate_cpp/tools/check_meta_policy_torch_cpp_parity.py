#!/usr/bin/env python3
"""Compare the training-side Torch gate with probabilities logged by C++."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import torch


CLASS_NAMES = ("slope_up", "slope_down", "flat", "NEUTRAL")


def load_meta_module(path: Path):
    spec = importlib.util.spec_from_file_location("paper_meta_policy_module", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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
    cpp_probability = np.asarray(
        [[float(row[f"prob_{name}"]) for name in CLASS_NAMES] for row in rows],
        dtype=np.float32,
    )

    module = load_meta_module(args.model_source)
    model = module.load_slope_meta_checkpoint(
        str(args.checkpoint),
        map_location="cpu",
        expected_expert_names=CLASS_NAMES[:3],
    ).eval()
    history_steps = 50
    outputs = []
    with torch.inference_mode():
        for batch_start in range(history_steps - 1, len(observations), args.batch_size):
            indices = range(batch_start, min(batch_start + args.batch_size, len(observations)))
            sequence = np.stack([
                observations[index - history_steps + 1 : index + 1]
                for index in indices
            ])
            logits, _ = model(torch.from_numpy(sequence))
            outputs.append(torch.softmax(logits[:, -1, :], dim=-1).numpy())
    torch_probability = np.concatenate(outputs, axis=0)
    reference = cpp_probability[history_steps - 1 :]
    error = np.abs(torch_probability - reference)
    report = {
        "trace": str(args.trace.resolve()),
        "checkpoint": str(args.checkpoint.resolve()),
        "evaluated_samples": len(reference),
        "history_steps": history_steps,
        "torch_vs_cpp_probabilities": {
            "max_abs_error": float(error.max()),
            "mean_abs_error": float(error.mean()),
            "argmax_agreement": float(np.mean(
                np.argmax(torch_probability, axis=1) == np.argmax(reference, axis=1)
            )),
        },
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
