#!/usr/bin/env python3
"""Archived P001 runner retained only to reproduce historic pilot commands."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from validate_config import EXPERIMENT_DIR, hydra_overrides, load_config


ROOT = EXPERIMENT_DIR.parents[3]
EVIDENCE_RUNS = EXPERIMENT_DIR.parent / "runs"


def _latest_checkpoint(run_dir: Path) -> Path | None:
    checkpoints = sorted(run_dir.rglob("model_*.pt"), key=lambda path: path.stat().st_mtime)
    return checkpoints[-1] if checkpoints else None


def _new_run_id(config: dict[str, object]) -> str:
    prefix = str(config["experiment_id"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{prefix}_{stamp}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", help="Optional unique evidence run ID; generated when omitted.")
    parser.add_argument("--dry-run", action="store_true", help="Print the command without launching Isaac Sim.")
    parser.add_argument("--smoke", action="store_true", help="Use disposable output only; never evidence.")
    args = parser.parse_args()

    config = load_config()
    execution = config["execution"]
    assert isinstance(execution, dict)
    cuda_visible_devices = str(execution["cuda_visible_devices"])
    run_env = os.environ.copy()
    run_env["CUDA_VISIBLE_DEVICES"] = cuda_visible_devices
    run_id = args.run_id or _new_run_id(config)
    command = [
        str(ROOT / "scripts" / "training" / "run_r1_isaaclab.sh"),
        "python",
        "scripts/training/r1_policy_workspace.py",
        "train",
        "rl_lab",
        "--run-id",
        run_id,
    ]
    if args.smoke:
        command.append("--smoke")
    if args.dry_run:
        command.append("--dry-run")
    command.extend(hydra_overrides(config))
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
            completeness = {
                "schema_version": 1,
                "training_execution": "completed" if result.returncode == 0 else "failed",
                "required_after_training": [
                    "model_*.pt", "events.out.tfevents.*", "training video", "policy.onnx", "policy.pt",
                    "derived/training/training_scalars.csv", "derived/training/plots/*.png"
                ],
                "required_before_positive_conclusion": [
                    "raw evaluation state/action/command trace", "evaluation metrics CSV", "evaluation video", "evaluation manifest"
                ],
                "evaluation_evidence": "pending",
            }
            artifacts = config.get("artifacts", {})
            if result.returncode == 0 and isinstance(artifacts, dict) and artifacts.get("export_policy_after_training") is True:
                checkpoint = _latest_checkpoint(run_dir)
                if checkpoint is None:
                    completeness["policy_export"] = "failed_no_checkpoint"
                else:
                    export_command = [
                        str(ROOT / "scripts" / "training" / "run_r1_isaaclab.sh"), "python",
                        "scripts/training/r1_policy_workspace.py", "export", "rl_lab",
                        "--checkpoint", str(checkpoint),
                    ]
                    exported = subprocess.run(export_command, cwd=ROOT, env=run_env, check=False)
                    completeness["policy_export"] = "completed" if exported.returncode == 0 else "failed"
                    completeness["policy_export_command"] = export_command
            if result.returncode == 0 and bool(config["capture"]["convert_tensorboard_to_csv_and_plots"]):
                derive_command = [
                    str(ROOT / "scripts" / "training" / "run_r1_isaaclab.sh"),
                    "python",
                    "scripts/training/extract_tensorboard_metrics.py",
                    "--log-dir",
                    str(run_dir / "logs"),
                    "--output-dir",
                    str(run_dir / "derived" / "training"),
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
                "policy_onnx": bool(list(run_dir.rglob("policy.onnx"))),
                "policy_jit": bool(list(run_dir.rglob("policy.pt"))),
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
