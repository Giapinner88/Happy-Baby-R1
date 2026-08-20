#!/usr/bin/env python3
"""Run I002 from its editable config without changing P001 or shared defaults."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from validate_config import EXPERIMENT_DIR, hydra_overrides, load_config


ROOT = EXPERIMENT_DIR.parents[3]
EVIDENCE_RUNS = EXPERIMENT_DIR.parent / "runs"


def _latest_checkpoint(run_dir: Path) -> Path | None:
    checkpoints = sorted(run_dir.rglob("model_*.pt"), key=lambda path: path.stat().st_mtime)
    return checkpoints[-1] if checkpoints else None


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit() -> str | None:
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else None


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _new_run_id(config: dict[str, object]) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{config['experiment_id']}_{stamp}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", help="Optional unique evidence ID; generated when omitted.")
    parser.add_argument("--dry-run", action="store_true", help="Print the command without launching Isaac Sim.")
    parser.add_argument("--smoke", action="store_true", help="Write disposable output only; never I002 evidence.")
    args = parser.parse_args()

    config = load_config()
    execution = config["execution"]
    assert isinstance(execution, dict)
    run_env = os.environ.copy()
    run_env["CUDA_VISIBLE_DEVICES"] = str(execution["cuda_visible_devices"])
    run_id = args.run_id or _new_run_id(config)
    run_dir = (
        ROOT / "results" / "smoke" / "training" / "isaaclab" / run_id
        if args.smoke else EVIDENCE_RUNS / run_id
    )
    command = [
        "conda",
        "run",
        "--no-capture-output",
        "-n",
        str(config["environment"]),
        "python",
        str(ROOT / "scripts" / "training" / "r1_rl_lab_train.py"),
        "--headless",
        "--task",
        str(config["task"]),
    ]
    command.extend(hydra_overrides(config))
    print(f"CUDA_VISIBLE_DEVICES={run_env['CUDA_VISIBLE_DEVICES']} " + " ".join(command))
    if args.dry_run:
        return 0

    if run_dir.exists():
        raise SystemExit(f"Refusing to overwrite existing run: {run_dir}")
    run_dir.mkdir(parents=True)
    if not args.smoke:
        _write_json(run_dir / "resolved_config.json", {
            "schema_version": 1,
            "experiment_config": config,
            "hydra_overrides": hydra_overrides(config),
        })
        _write_json(run_dir / "metadata.json", {
            "schema_version": 2,
            "record_type": "experiment_run_metadata",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "run": {
                "id": run_id,
                "output_class": "evidence",
                "experiment": str(EXPERIMENT_DIR.relative_to(ROOT)),
            },
            "execution": {
                "command": command,
                "working_directory": str(run_dir),
                "environment": {
                    "python_executable": sys.executable,
                    "python_version": sys.version,
                    "platform": platform.platform(),
                    "variables": {
                        "CONDA_DEFAULT_ENV": str(config["environment"]),
                        "CUDA_VISIBLE_DEVICES": run_env["CUDA_VISIBLE_DEVICES"],
                    },
                },
            },
            "source": {"git_commit": _git_commit()},
            "assets": {
                "r1_usd": {
                    "path": "assets/R1/R1.usd",
                    "sha256": _sha256(ROOT / "assets" / "R1" / "R1.usd"),
                }
            },
        })
        _write_json(run_dir / "status.json", {
            "schema_version": 1,
            "execution_status": "running",
            "scientific_outcome": "unassessed",
            "updated_at": datetime.now(timezone.utc).isoformat(),
        })
    result = subprocess.run(command, cwd=run_dir, env=run_env, check=False)
    final_returncode = result.returncode
    if args.smoke:
        return final_returncode
    status = "completed" if result.returncode == 0 else "failed"
    _write_json(run_dir / "status.json", {
        "schema_version": 1,
        "execution_status": status,
        "scientific_outcome": "unassessed",
        "reason": None if result.returncode == 0 else f"training command exited with {result.returncode}",
        "updated_at": datetime.now(timezone.utc).isoformat(),
    })
    shutil.copy2(EXPERIMENT_DIR / "config.json", run_dir / "experiment_config.json")
    (run_dir / "experiment_runner_command.txt").write_text(" ".join(command) + "\n", encoding="utf-8")
    completeness: dict[str, object] = {
        "schema_version": 1,
        "training_execution": "completed" if result.returncode == 0 else "failed",
        "required_after_training": [
            "model_*.pt", "events.out.tfevents.*", "training video", "policy.onnx", "policy.pt",
            "derived/training/training_scalars.csv", "derived/training/plots/*.png",
        ],
        "required_before_positive_conclusion": [
            "raw standing evaluation trace", "standing metrics CSV", "standing evaluation video", "evaluation manifest",
        ],
        "evaluation_evidence": "pending",
    }
    artifacts = config["artifacts"]
    assert isinstance(artifacts, dict)
    if result.returncode == 0 and artifacts.get("export_policy_after_training") is True:
        checkpoint = _latest_checkpoint(run_dir)
        if checkpoint is None:
            completeness["policy_export"] = "failed_no_checkpoint"
        else:
            export_command = [
                "conda",
                "run",
                "--no-capture-output",
                "-n",
                str(config["environment"]),
                "python",
                str(ROOT / "scripts" / "training" / "r1_rl_lab_export.py"),
                "--headless",
                "--task",
                str(config["task"]),
                "--checkpoint",
                str(checkpoint),
                "--output-dir",
                str(checkpoint.parent / "exported"),
            ]
            exported = subprocess.run(export_command, cwd=ROOT, env=run_env, check=False)
            completeness["policy_export"] = "completed" if exported.returncode == 0 else "failed"
            completeness["policy_export_command"] = export_command
    capture = config["capture"]
    assert isinstance(capture, dict)
    if result.returncode == 0 and capture.get("convert_tensorboard_to_csv_and_plots") is True:
        derive_command = [
            "conda",
            "run",
            "--no-capture-output",
            "-n",
            str(config["environment"]),
            "python",
            str(ROOT / "scripts" / "training" / "extract_tensorboard_metrics.py"),
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
