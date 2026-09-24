#!/usr/bin/env python3
"""Compare Python ONNX replay with a fully instrumented MuJoCo C++ trace."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml


EXPERT_FILES = ("slope_up.onnx", "slope_down.onnx", "flat_common_pd.onnx")
CLASS_NAMES = ("slope_up", "slope_down", "flat", "NEUTRAL")


def columns(rows, prefix: str, count: int) -> np.ndarray:
    return np.asarray(
        [[float(row[f"{prefix}{index}"]) for index in range(count)] for row in rows],
        dtype=np.float32,
    )


def metadata_contract(session: ort.InferenceSession):
    metadata = session.get_modelmeta().custom_metadata_map
    scale = np.asarray([float(x) for x in metadata["action_scale"].split(",")], dtype=np.float32)
    offset = np.asarray([float(x) for x in metadata["default_joint_pos"].split(",")], dtype=np.float32)
    return scale, offset


def gate_replay(session, observations, history_steps=50, batch_size=128):
    outputs = []
    for batch_start in range(history_steps - 1, len(observations), batch_size):
        indices = np.arange(batch_start, min(batch_start + batch_size, len(observations)))
        sequence = np.stack(
            [observations[index - history_steps + 1 : index + 1] for index in indices]
        )
        hidden = np.zeros((1, len(indices), 128), dtype=np.float32)
        probabilities = None
        for step in range(history_steps):
            probabilities, hidden = session.run(
                None,
                {"observation": sequence[:, step, :], "hidden": hidden},
            )
        outputs.append(probabilities)
    return np.concatenate(outputs, axis=0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("package", type=Path)
    args = parser.parse_args()

    with args.trace.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    observations = columns(rows, "obs_", 83)
    cpp_action = columns(rows, "action_", 24)
    cpp_target = columns(rows, "target_q_", 24)
    cpp_probability = np.asarray(
        [[float(row[f"prob_{name}"]) for name in CLASS_NAMES] for row in rows],
        dtype=np.float32,
    )
    weights = np.asarray(
        [[float(row[f"weight_{name}"]) for name in CLASS_NAMES[:3]] for row in rows],
        dtype=np.float32,
    )
    selected = np.asarray([row["meta_selected"] for row in rows])

    deploy = yaml.safe_load((args.package / "params/deploy.yaml").read_text())
    action_cfg = deploy["actions"]["JointPositionAction"]
    common_scale = np.asarray(action_cfg["scale"], dtype=np.float32)
    common_offset = np.asarray(action_cfg["offset"], dtype=np.float32)
    meta_cfg = deploy["meta_policy"]
    history_steps = int(meta_cfg["history_steps"])

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    providers = ["CPUExecutionProvider"]
    exported = args.package / "exported"
    gate = ort.InferenceSession(
        str(exported / "slope_meta_with_flat.onnx"),
        sess_options=options,
        providers=providers,
    )
    expert_sessions = [
        ort.InferenceSession(str(exported / name), sess_options=options, providers=providers)
        for name in EXPERT_FILES
    ]

    python_probability = gate_replay(gate, observations, history_steps)
    start = history_steps - 1
    probability_error = np.abs(python_probability - cpp_probability[start:])

    expert_targets = []
    for session in expert_sessions:
        expert_scale, expert_offset = metadata_contract(session)
        adapted = observations.copy()
        adapted[:, 11:35] += common_offset - expert_offset
        previous_target = common_offset + common_scale * observations[:, 59:83]
        adapted[:, 59:83] = (previous_target - expert_offset) / expert_scale
        input_name = session.get_inputs()[0].name
        raw_action = np.concatenate([
            session.run(None, {input_name: adapted[index : index + 1]})[0]
            for index in range(len(adapted))
        ])
        expert_targets.append(expert_offset + expert_scale * raw_action)
    expert_targets = np.stack(expert_targets, axis=1)
    blended_target = np.sum(weights[:, :, None] * expert_targets, axis=1)
    target_action = (blended_target - common_offset) / common_scale

    replay_action = np.zeros_like(cpp_action)
    replay_action[:start] = cpp_action[:start]
    previous = cpp_action[start - 1].copy()
    crossfade_from = previous.copy()
    crossfade_step = 10
    for index in range(start, len(rows)):
        if selected[index] != selected[index - 1]:
            crossfade_from = previous.copy()
            crossfade_step = 0
        if crossfade_step < 10:
            crossfade_step += 1
            alpha = crossfade_step / 10.0
            current = (1.0 - alpha) * crossfade_from + alpha * target_action[index]
        else:
            current = target_action[index]
        replay_action[index] = current
        previous = current
    replay_target = common_offset + common_scale * replay_action

    action_error = np.abs(replay_action[start:] - cpp_action[start:])
    target_error = np.abs(replay_target[start:] - cpp_target[start:])
    mapping_error = np.abs(
        common_offset + common_scale * cpp_action[start:] - cpp_target[start:]
    )
    report = {
        "trace": str(args.trace.resolve()),
        "package": str(args.package.resolve()),
        "evaluated_samples": len(rows) - start,
        "history_steps": history_steps,
        "python_onnx_vs_cpp_probabilities": {
            "max_abs_error": float(probability_error.max()),
            "mean_abs_error": float(probability_error.mean()),
            "argmax_agreement": float(np.mean(
                np.argmax(python_probability, axis=1)
                == np.argmax(cpp_probability[start:], axis=1)
            )),
        },
        "python_onnx_target_blend_vs_cpp": {
            "max_abs_action_error": float(action_error.max()),
            "mean_abs_action_error": float(action_error.mean()),
            "max_abs_joint_target_error_rad": float(target_error.max()),
            "mean_abs_joint_target_error_rad": float(target_error.mean()),
        },
        "cpp_action_to_target_csv_consistency": {
            "max_abs_error_rad": float(mapping_error.max()),
        },
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
