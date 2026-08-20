#!/usr/bin/env python3
"""Run M001 from its editable config without changing project defaults."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from validate_config import EXPERIMENT_DIR, load_config, mjlab_overrides


ROOT = EXPERIMENT_DIR.parents[3]
EVIDENCE_RUNS = EXPERIMENT_DIR.parent / "runs"


def _new_run_id(config: dict[str, object]) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{config['experiment_id']}_{stamp}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", help="Optional unique evidence run ID; generated when omitted.")
    parser.add_argument("--dry-run", action="store_true", help="Print the command without launching MJLab.")
    parser.add_argument("--smoke", action="store_true", help="Use disposable output only; never evidence.")
    args = parser.parse_args()

    config = load_config()
    overrides = mjlab_overrides(config)
    training = config["training"]
    capture = config["capture"]
    execution = config["execution"]
    assert isinstance(training, dict)
    assert isinstance(capture, dict)
    assert isinstance(execution, dict)
    cuda_visible_devices = str(execution["cuda_visible_devices"])
    run_env = os.environ.copy()
    run_env["CUDA_VISIBLE_DEVICES"] = cuda_visible_devices
    run_id = args.run_id or _new_run_id(config)
    command = [
        "conda", "run", "--no-capture-output", "-n", "r1_env", "python",
        "scripts/training/r1_policy_workspace.py", "train", "mjlab",
        "--terrain", "flat", "--run-id", run_id,
        "--num-envs", str(int(training["num_envs"])),
        "--max-iterations", str(int(training["max_iterations"])),
        "--save-interval", str(int(training["save_interval"])),
        "--run-name", str(training["run_name"]),
        "--video",
        "--video-length", str(int(capture["training_video_length_steps"])),
        "--video-interval", str(int(capture["training_video_interval_steps"])),
    ]
    if args.smoke:
        command.append("--smoke")
    if args.dry_run:
        command.append("--dry-run")
    command.extend(overrides)
    print(f"CUDA_VISIBLE_DEVICES={cuda_visible_devices} " + " ".join(command))
    if args.dry_run:
        return 0

    result = subprocess.run(command, cwd=ROOT, env=run_env, check=False)
    final_returncode = result.returncode
    if not args.smoke:
        run_dir = EVIDENCE_RUNS / run_id
        if run_dir.is_dir():
            shutil.copy2(EXPERIMENT_DIR / "config.json", run_dir / "experiment_config.json")
            (run_dir / "experiment_runner_command.txt").write_text(" ".join(command) + "\n", encoding="utf-8")
            completeness: dict[str, object] = {
                "schema_version": 1,
                "training_execution": "completed" if result.returncode == 0 else "failed",
                "required_after_training": [
                    "model_*.pt", "events.out.tfevents.*", "training video",
                    "derived/training/training_scalars.csv", "derived/training/plots/*.png",
                ],
                "required_before_positive_conclusion": [
                    "raw evaluation state/action/command trace", "evaluation metrics CSV",
                    "evaluation video", "evaluation manifest",
                ],
                "evaluation_evidence": "pending",
            }
            if result.returncode == 0:
                derive_command = [
                    "conda", "run", "--no-capture-output", "-n", "r1_env", "python",
                    "scripts/training/extract_tensorboard_metrics.py", "--log-dir", str(run_dir / "logs"),
                    "--output-dir", str(run_dir / "derived" / "training"),
                ]
                derived = subprocess.run(derive_command, cwd=ROOT, env=run_env, check=False)
                completeness["training_derivation"] = "completed" if derived.returncode == 0 else "failed"
                completeness["training_derivation_command"] = derive_command
            required = {
                "checkpoint": bool(list(run_dir.rglob("model_*.pt"))),
                "tensorboard_event": bool(list(run_dir.rglob("events.out.tfevents.*"))),
                "training_video": bool(list(run_dir.rglob("*.mp4"))),
                "scalar_csv": (run_dir / "derived" / "training" / "training_scalars.csv").is_file(),
                "diagnostic_plot": bool(list((run_dir / "derived" / "training" / "plots").glob("*.png"))),
                "experiment_snapshot": (run_dir / "experiment_config.json").is_file(),
            }
            completeness["training_artifacts"] = required
            completeness["missing_training_artifacts"] = [name for name, present in required.items() if not present]
            completeness["training_evidence"] = (
                "complete" if result.returncode == 0 and not completeness["missing_training_artifacts"] else "incomplete"
            )
            if result.returncode == 0 and completeness["missing_training_artifacts"]:
                final_returncode = 2
            (run_dir / "evidence_completeness.json").write_text(
                json.dumps(completeness, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
    return final_returncode


if __name__ == "__main__":
    raise SystemExit(main())
