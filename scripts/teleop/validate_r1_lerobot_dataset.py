#!/usr/bin/env python3
"""Validate an R1 arms+head LeRobot dataset before GR00T training."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
UPSTREAM_SCRIPT_DIR = ROOT / "third_party/lerobot/r1_lerobot_data_pipeline/scripts"
sys.path.insert(0, str(UPSTREAM_SCRIPT_DIR))

from r1_schema import arm_names  # noqa: E402
from teleop.r1.lerobot_conversion import HEAD_FEATURE_NAMES  # noqa: E402


def output(report: dict, path: Path | None) -> int:
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    print(text, end="")
    if path:
        destination = path.expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    return 0 if report["status"] == "passed" else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--profile", choices=("a5", "a7"), default="a5")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--decode-images", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = args.root.expanduser().resolve()
    failures: list[str] = []
    warnings: list[str] = []
    try:
        dataset = LeRobotDataset(args.repo_id, root=root)
    except Exception as exc:
        return output(
            {
                "schema": "happy_baby_r1.training_dataset_validation",
                "schema_version": 2,
                "status": "failed",
                "repo_id": args.repo_id,
                "root": str(root),
                "joint_contract": "arms_head",
                "failures": [f"cannot load dataset: {type(exc).__name__}: {exc}"],
                "warnings": [],
            },
            args.report,
        )

    expected_names = [*arm_names(args.profile), *HEAD_FEATURE_NAMES]
    features = dataset.features
    for key in ("observation.state", "action"):
        feature = features.get(key)
        if not feature:
            failures.append(f"missing feature: {key}")
            continue
        names = list(feature.get("names") or [])
        if names != expected_names:
            failures.append(f"{key} names mismatch: expected {expected_names}, got {names}")
        try:
            values = np.asarray(dataset.hf_dataset[key], dtype=np.float32)
        except Exception as exc:
            failures.append(f"cannot read {key}: {type(exc).__name__}: {exc}")
            continue
        expected_shape = (dataset.num_frames, len(expected_names))
        if values.shape != expected_shape:
            failures.append(f"{key} shape mismatch: expected {expected_shape}, got {values.shape}")
        elif not np.isfinite(values).all():
            failures.append(f"{key} contains NaN or Inf")

    cameras = sorted(
        key
        for key, value in features.items()
        if key.startswith("observation.images.") and value.get("dtype") in {"image", "video"}
    )
    if not cameras:
        failures.append("dataset has no observation.images.* feature")

    try:
        episode_index = np.asarray(dataset.hf_dataset["episode_index"], dtype=np.int64)
        frame_index = np.asarray(dataset.hf_dataset["frame_index"], dtype=np.int64)
        timestamp = np.asarray(dataset.hf_dataset["timestamp"], dtype=np.float64)
        if not (len(episode_index) == len(frame_index) == len(timestamp) == dataset.num_frames):
            failures.append("episode/frame/timestamp columns do not match total frame count")
        for episode in np.unique(episode_index):
            mask = episode_index == episode
            local_frames = frame_index[mask]
            local_time = timestamp[mask]
            if not np.array_equal(local_frames, np.arange(len(local_frames))):
                failures.append(f"episode {int(episode)} frame_index is not contiguous from zero")
            if len(local_time) > 1 and np.any(np.diff(local_time) <= 0):
                failures.append(f"episode {int(episode)} timestamps are not strictly increasing")
    except Exception as exc:
        failures.append(f"cannot validate episode layout: {type(exc).__name__}: {exc}")
        episode_index = np.empty((0,), dtype=np.int64)

    decoded_samples = 0
    if args.decode_images and len(episode_index):
        probe_indices: set[int] = set()
        for episode in np.unique(episode_index):
            positions = np.flatnonzero(episode_index == episode)
            probe_indices.update((int(positions[0]), int(positions[-1])))
        for index in sorted(probe_indices):
            try:
                sample = dataset[index]
                for camera in cameras:
                    image = np.asarray(sample[camera])
                    if image.ndim != 3 or not np.isfinite(image).all():
                        failures.append(f"frame {index} camera {camera} is invalid: shape={image.shape}")
                decoded_samples += 1
            except Exception as exc:
                failures.append(f"cannot decode frame {index}: {type(exc).__name__}: {exc}")

    if dataset.num_episodes < 10:
        warnings.append("fewer than 10 episodes: suitable for smoke testing, too small for training")
    return output(
        {
            "schema": "happy_baby_r1.training_dataset_validation",
            "schema_version": 2,
            "status": "passed" if not failures else "failed",
            "repo_id": args.repo_id,
            "root": str(root),
            "profile": args.profile,
            "joint_contract": "arms_head",
            "joint_names": expected_names,
            "state_dim": len(expected_names),
            "action_dim": len(expected_names),
            "fps": dataset.fps,
            "frames": dataset.num_frames,
            "episodes": dataset.num_episodes,
            "cameras": cameras,
            "decoded_samples": decoded_samples,
            "failures": failures,
            "warnings": warnings,
        },
        args.report,
    )


if __name__ == "__main__":
    raise SystemExit(main())
