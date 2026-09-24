#!/usr/bin/env python3
"""Replay selector ablations on probabilities captured from MuJoCo C++."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


CLASSES = ("slope_up", "slope_down", "flat", "NEUTRAL")


@dataclass(frozen=True)
class Config:
    policy_hz: float = 50.0
    enter: float = 0.75
    exit: float = 0.55
    agreement: int = 5
    dwell_s: float = 0.5
    crossfade_s: float = 0.2
    allow_neutral: bool = True


def steps(seconds: float, hz: float) -> int:
    return max(1, int(math.floor(seconds * hz + 0.5)))


class Selector:
    def __init__(self, cfg: Config, initial_class: int, initial_weights: np.ndarray):
        self.cfg = cfg
        self.neutral = 3
        self.selected = initial_class
        self.last_expert = initial_class if initial_class < self.neutral else 2
        self.weights = initial_weights.astype(float).copy()
        self.blend_from = self.weights.copy()
        self.blend_to = self.weights.copy()
        self.blend_step = steps(cfg.crossfade_s, cfg.policy_hz)
        self.crossfade_steps = steps(cfg.crossfade_s, cfg.policy_hz)
        self.dwell_steps = steps(cfg.dwell_s, cfg.policy_hz)
        self.minimum_dwell_steps = steps(cfg.dwell_s, cfg.policy_hz)
        self.candidate = initial_class
        self.candidate_frames = cfg.agreement
        self.safety_hold = False

    def target_weights(self, target: int) -> np.ndarray:
        result = np.zeros(3, dtype=float)
        if target < self.neutral:
            result[target] = 1.0
        else:
            result[self.last_expert] = 1.0
        return result

    def request_switch(self, target: int) -> None:
        self.blend_from = self.weights.copy()
        self.blend_to = self.target_weights(target)
        self.blend_step = 0
        self.selected = target
        self.dwell_steps = 0
        if target < self.neutral:
            self.last_expert = target

    def update(self, probabilities: np.ndarray) -> None:
        confidence = float(np.max(probabilities))
        proposed = int(np.argmax(probabilities))
        self.dwell_steps += 1
        self.safety_hold = False

        if not self.cfg.allow_neutral:
            proposed = int(np.argmax(probabilities[:3]))
        elif self.selected < self.neutral:
            if probabilities[self.selected] >= self.cfg.exit:
                proposed = self.selected
            elif confidence < self.cfg.enter:
                proposed = self.neutral
        elif confidence < self.cfg.enter:
            proposed = self.neutral

        if proposed == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = proposed
            self.candidate_frames = 1

        dwell_ok = self.selected == self.neutral or self.dwell_steps >= self.minimum_dwell_steps
        if (
            self.candidate_frames >= self.cfg.agreement
            and dwell_ok
            and self.candidate != self.selected
        ):
            self.request_switch(self.candidate)

        if self.blend_step < self.crossfade_steps:
            self.blend_step += 1
            alpha = self.blend_step / self.crossfade_steps
            self.weights = (1.0 - alpha) * self.blend_from + alpha * self.blend_to
        else:
            self.weights = self.blend_to.copy()


def load(path: Path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    time = np.asarray([float(row["t"]) for row in rows])
    probability = np.asarray([
        [float(row[f"prob_{name}"]) for name in CLASSES] for row in rows
    ])
    actual_weights = np.asarray([
        [float(row[f"weight_{name}"]) for name in CLASSES[:3]] for row in rows
    ])
    actual_class = np.asarray([row["meta_selected"] for row in rows])
    position = np.asarray([float(row["drift_x"]) for row in rows])
    return time, probability, actual_weights, actual_class, position


def replay(cfg: Config, probability: np.ndarray, actual_weights: np.ndarray, actual_class: np.ndarray):
    initial_class = CLASSES.index(str(actual_class[0]))
    selector = Selector(cfg, initial_class, actual_weights[0])
    classes = [CLASSES[selector.selected]]
    weights = [selector.weights.copy()]
    for values in probability[1:]:
        selector.update(values)
        classes.append(CLASSES[selector.selected])
        weights.append(selector.weights.copy())
    return np.asarray(classes), np.asarray(weights)


def summarize(name, time, classes, weights, actual_class, actual_weights, position):
    changes = np.flatnonzero(classes[1:] != classes[:-1]) + 1
    expert_changes = [
        int(index) for index in changes
        if classes[index] != "NEUTRAL" and classes[index - 1] != "NEUTRAL"
    ]
    return {
        "name": name,
        "switches": int(len(changes)),
        "direct_expert_switches": len(expert_changes),
        "neutral_fraction": float(np.mean(classes == "NEUTRAL")),
        "weight_total_variation": float(np.abs(np.diff(weights, axis=0)).sum()),
        "class_agreement_with_cpp": float(np.mean(classes == actual_class)),
        "max_abs_weight_error_vs_cpp": float(np.max(np.abs(weights - actual_weights))),
        "transitions": [
            {
                "time_s": float(time[index]),
                "position_m": float(position[index]),
                "from": str(classes[index - 1]),
                "to": str(classes[index]),
            }
            for index in changes
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    time, probability, actual_weights, actual_class, position = load(args.trace)
    configs = {
        "baseline": Config(),
        "no_hysteresis": Config(exit=0.75),
        "no_dwell": Config(dwell_s=0.0),
        "no_neutral": Config(enter=0.0, exit=0.0, allow_neutral=False),
        "hard_switch": Config(crossfade_s=0.0),
    }
    report = {"trace": str(args.trace.resolve()), "samples": len(time), "duration_s": float(time[-1])}
    report["ablations"] = []
    for name, cfg in configs.items():
        classes, weights = replay(cfg, probability, actual_weights, actual_class)
        report["ablations"].append(
            summarize(name, time, classes, weights, actual_class, actual_weights, position)
        )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
