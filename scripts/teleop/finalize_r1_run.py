#!/usr/bin/env python3
"""Generate derived artifacts and enforce the canonical run evidence contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SIM_DATA = (
    "raw_commands.jsonl", "targets.json", "control_loop.json", "dynamics_trace.npz",
    "metrics.json", "metadata.json", "resolved_config.json", "experiment_config.json",
    "status.json", "bridge_connection.jsonl",
)
HARDWARE_DATA = (
    "bridge_connection.jsonl", "hardware_targets.jsonl", "upstream_solver_stats.json",
    "pipeline_status.json", "robot/metadata.json", "robot/samples.jsonl",
)


def _nonempty(path: Path) -> bool:
    return path.is_file() and path.stat().st_size > 0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_jsonl(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{number} is not a JSON object")
        rows.append(value)
    return rows


def _hardware_arrays(run_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    rows = _read_jsonl(run_dir / "robot" / "samples.jsonl")
    if not rows:
        raise ValueError("robot/samples.jsonl contains no samples")
    names = [str(value) for value in rows[0]["joint_names"]]
    time_s = np.asarray([float(row["monotonic_s"]) for row in rows], dtype=float)
    target = np.asarray([row["target_q"] for row in rows], dtype=float)
    observed = np.asarray([row["observed_q"] for row in rows], dtype=float)
    if target.shape != observed.shape or target.ndim != 2 or target.shape[1] != len(names):
        raise ValueError("robot samples have inconsistent joint vectors")
    if not np.all(np.isfinite(target)) or not np.all(np.isfinite(observed)):
        raise ValueError("robot samples contain non-finite joint values")
    return time_s - time_s[0], target, observed, names


def _render_hardware(run_dir: Path, fps: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import imageio.v2 as imageio

    time_s, target, observed, names = _hardware_arrays(run_dir)
    figures = run_dir / "figures"
    figures.mkdir(exist_ok=False)
    figure, axes = plt.subplots(4, 3, figsize=(15, 10), sharex=True, layout="constrained")
    for index, axis in enumerate(axes.flat):
        if index >= len(names):
            axis.set_visible(False)
            continue
        axis.plot(time_s, target[:, index], label="target", linewidth=0.9)
        axis.plot(time_s, observed[:, index], label="observed", linewidth=0.9)
        axis.set_title(names[index].replace("_joint", ""), fontsize=8)
        axis.set_ylabel("rad")
    axes.flat[0].legend(fontsize=7)
    for axis in axes[-1]:
        axis.set_xlabel("time (s)")
    figure.savefig(figures / "hardware_joint_tracking.png", dpi=160)
    plt.close(figure)

    error = np.abs(target - observed)
    metrics = {
        "schema_version": 1,
        "mode": "hardware",
        "sample_count": int(len(time_s)),
        "duration_s": float(time_s[-1]) if len(time_s) > 1 else 0.0,
        "absolute_joint_error_rad": {
            name: {"mean": float(np.mean(error[:, index])), "max": float(np.max(error[:, index]))}
            for index, name in enumerate(names)
        },
    }
    (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")

    # This is synchronized telemetry evidence, not a camera recording of the robot.
    video_figure, video_axis = plt.subplots(figsize=(8, 4.5), layout="constrained")
    video_path = run_dir / "telemetry_tracking.mp4"
    indices = np.unique(np.linspace(0, len(time_s) - 1, min(len(time_s), 300), dtype=int))
    with imageio.get_writer(video_path, fps=fps, macro_block_size=None) as writer:
        for end in indices:
            video_axis.clear()
            video_axis.plot(time_s[: end + 1], np.max(error[: end + 1], axis=1), color="tab:red")
            video_axis.set(xlim=(0.0, max(float(time_s[-1]), 1.0)), ylim=(0.0, max(float(np.max(error)), 0.01)),
                           xlabel="time (s)", ylabel="max |target-observed| (rad)",
                           title="R1 hardware telemetry (not camera footage)")
            video_figure.canvas.draw()
            writer.append_data(np.asarray(video_figure.canvas.buffer_rgba())[..., :3])
    plt.close(video_figure)


def _render_simulation(run_dir: Path) -> None:
    figures = run_dir / "figures"
    if figures.exists():
        return
    environment = os.environ.copy()
    environment.setdefault("MPLCONFIGDIR", "/tmp/hb-matplotlib")
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/teleop/plot_r1_baseline_dynamics.py"), str(run_dir)],
        cwd=ROOT,
        env=environment,
        check=True,
    )


def evaluate_artifacts(run_dir: Path, kind: str) -> dict[str, object]:
    required = SIM_DATA if kind == "simulation" else HARDWARE_DATA + ("metrics.json",)
    missing_data = [name for name in required if not _nonempty(run_dir / name)]
    figures = sorted(path for path in (run_dir / "figures").glob("*.png") if _nonempty(path))
    video_name = "simulator_view.mp4" if kind == "simulation" else "telemetry_tracking.mp4"
    return {
        "schema_version": 1,
        "kind": kind,
        "data": not missing_data,
        "missing_data": missing_data,
        "figures": bool(figures),
        "figure_files": [str(path.relative_to(run_dir)) for path in figures],
        "video": _nonempty(run_dir / video_name),
        "video_file": video_name,
        "video_semantics": "simulator camera" if kind == "simulation" else "target/encoder telemetry animation; not camera footage",
    }


def _write_manifest(run_dir: Path, completeness: dict[str, object]) -> None:
    files = []
    for path in sorted(run_dir.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        files.append({"path": str(path.relative_to(run_dir)), "bytes": path.stat().st_size, "sha256": _sha256(path)})
    payload = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "complete": bool(completeness["data"] and completeness["figures"] and completeness["video"]),
        "groups": completeness,
        "files": files,
    }
    (run_dir / "artifact_manifest.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("simulation", "hardware"))
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--video-fps", type=float, default=15.0)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise SystemExit(f"Run directory does not exist: {run_dir}")
    prior_completeness: dict[str, object] | None = None
    prior_path = run_dir / "evidence_completeness.json"
    if prior_path.is_file():
        try:
            loaded = json.loads(prior_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                prior_completeness = dict(loaded.get("runner_checks", loaded))
        except (OSError, json.JSONDecodeError):
            pass
    try:
        if args.kind == "simulation":
            _render_simulation(run_dir)
        else:
            _render_hardware(run_dir, args.video_fps)
    except Exception as exc:
        print(f"[ARTIFACT] generation failed: {exc}", file=sys.stderr)
    completeness = evaluate_artifacts(run_dir, args.kind)
    if prior_completeness is not None:
        completeness["runner_checks"] = prior_completeness
    complete = bool(completeness["data"] and completeness["figures"] and completeness["video"])
    (run_dir / "evidence_completeness.json").write_text(
        json.dumps(completeness, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_manifest(run_dir, completeness)
    if not complete:
        print(f"[ARTIFACT] incomplete run: {json.dumps(completeness, sort_keys=True)}", file=sys.stderr)
        return 1
    print(f"[ARTIFACT] complete data + figures + video: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
