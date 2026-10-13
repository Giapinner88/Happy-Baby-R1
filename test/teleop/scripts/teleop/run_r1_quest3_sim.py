#!/usr/bin/env python3
"""Replay normalized Quest 3 commands through the simulation-only R1 teleop path.

This runner deliberately has no DDS, ROS, Unitree SDK, or hardware deployment
imports. A live Quest bridge may write newline-delimited R1TeleopCommand JSON to
stdin; deterministic traces use --input-trace for reproducible simulation tests.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)

from teleop.r1 import (  # noqa: E402
    FakeIsaacLabSink,
    R1TeleopCommand,
    R1TeleopMapper,
    SimulationOnlyAdapter,
    TeleopCalibration,
    TeleopLimits,
    Vector3,
    validate_isaaclab_velocity_policy,
)


def _load_config(path: Path) -> dict[str, object]:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load teleop JSON config {path}: {exc}") from exc
    if config.get("mode") != "simulation_only":
        raise SystemExit("R1 teleop v1 only permits mode='simulation_only'.")
    return config


def _mapper_from_config(config: dict[str, object], policy_manifest: Path | None) -> tuple[R1TeleopMapper, dict[str, object] | None]:
    calibration = config.get("calibration")
    velocity = config.get("velocity")
    if not isinstance(calibration, dict) or not isinstance(velocity, dict):
        raise SystemExit("Teleop config requires calibration and velocity objects.")
    translation = calibration.get("translation_m")
    if not isinstance(translation, list) or len(translation) != 3:
        raise SystemExit("calibration.translation_m must contain three values.")
    velocity_enabled = bool(velocity.get("enabled", False))
    gate: dict[str, object] | None = None
    if velocity_enabled and bool(velocity.get("requires_evaluation_manifest", True)):
        if policy_manifest is None:
            raise SystemExit("Velocity is enabled but no --policy-manifest was supplied; refusing simulator velocity.")
        try:
            gate = validate_isaaclab_velocity_policy(policy_manifest, ROOT / "assets" / "R1" / "R1.usd")
        except ValueError as exc:
            raise SystemExit(f"Velocity policy gate failed: {exc}") from exc
    return R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(calibration.get("yaw_rad", 0.0)),
            source_frame=str(config.get("source_frame", "quest_headset")),
            robot_frame=str(config.get("robot_frame", "r1_base")),
        ),
        TeleopLimits(
            command_timeout_s=float(config.get("command_timeout_s", 0.0)),
            allow_velocity=velocity_enabled,
            max_vx_mps=float(velocity.get("max_vx_mps", 0.0)),
            max_vy_mps=float(velocity.get("max_vy_mps", 0.0)),
            max_yaw_rate_radps=float(velocity.get("max_yaw_rate_radps", 0.0)),
        ),
    ), gate


def _read_lines(args: argparse.Namespace) -> list[str]:
    if args.input_trace:
        return Path(args.input_trace).expanduser().read_text(encoding="utf-8").splitlines()
    if args.input_stdin:
        return [line.rstrip("\n") for line in sys.stdin if line.strip()]
    raise SystemExit("Specify --input-trace for a replay or --input-stdin for a live normalized Quest bridge.")


def _default_output_dir() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / "results" / "smoke" / "r1_teleop" / stamp


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _teleop_source_hashes() -> dict[str, str]:
    source_files = (
        ROOT / "scripts" / "teleop" / "run_r1_quest3_sim.py",
        ROOT / "teleop" / "r1" / "mapping.py",
        ROOT / "teleop" / "r1" / "schema.py",
        ROOT / "teleop" / "r1" / "simulator.py",
    )
    return {str(path.relative_to(ROOT)): _sha256(path) for path in source_files}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T001/config/r1_quest3_sim_v1.json",
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input-trace", help="Newline-delimited R1TeleopCommand JSON trace.")
    source.add_argument("--input-stdin", action="store_true", help="Read normalized R1TeleopCommand JSON lines from stdin.")
    parser.add_argument("--output-dir", type=Path, help="Output directory; defaults to disposable results/smoke.")
    parser.add_argument(
        "--policy-manifest",
        type=Path,
        help="Promotion manifest required whenever the config enables simulator base velocity.",
    )
    parser.add_argument(
        "--replay-receive-lag-s",
        type=float,
        default=0.0,
        help="Deterministic receipt lag added to every trace timestamp; use only with --input-trace.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Validate config and command schema without writing output.")
    args = parser.parse_args()

    if args.replay_receive_lag_s < 0.0:
        raise SystemExit("--replay-receive-lag-s must be non-negative.")
    if args.input_stdin and args.replay_receive_lag_s != 0.0:
        raise SystemExit("--replay-receive-lag-s is only valid with --input-trace.")

    config = _load_config(args.config.expanduser().resolve())
    mapper, policy_gate = _mapper_from_config(config, args.policy_manifest)
    lines = _read_lines(args)
    sink = FakeIsaacLabSink()
    adapter = SimulationOnlyAdapter(sink, mapper.ownership)
    targets = []
    previous_sequence = -1
    for line_number, line in enumerate(lines, start=1):
        try:
            command = R1TeleopCommand.from_dict(json.loads(line))
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            raise SystemExit(f"Invalid command on line {line_number}: {exc}") from exc
        if command.sequence_id <= previous_sequence:
            raise SystemExit("Quest command sequence_id must increase strictly within one replay.")
        previous_sequence = command.sequence_id
        received_time = (
            command.timestamp_monotonic_s + args.replay_receive_lag_s
            if args.input_trace
            else time.monotonic()
        )
        target = mapper.map(command, received_time)
        adapter.apply(target)
        target_record = asdict(target)
        target_record["latency_s"] = received_time - command.timestamp_monotonic_s
        targets.append(target_record)

    summary = {
        "schema_version": 1,
        "mode": "simulation_only",
        "velocity_enabled": mapper.limits.allow_velocity,
        "command_count": len(targets),
        "enabled_count": sum(1 for target in targets if target["enabled"]),
        "disabled_reasons": sorted({target["reason"] for target in targets if target["reason"]}),
        "watchdog_or_safety_events": [target["reason"] for target in targets if target["reason"]],
        "latency_s": [target["latency_s"] for target in targets],
        "sink_events": [event[0] for event in sink.events],
        "policy_gate": policy_gate,
    }
    if args.dry_run:
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    output_dir = (args.output_dir or _default_output_dir()).expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite teleop output: {output_dir}")
    output_dir.mkdir(parents=True)
    (output_dir / "raw_commands.jsonl").write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    write_resolved_config(output_dir, config)
    write_experiment_config(output_dir, config)
    write_runner_command(output_dir)
    (output_dir / "targets.json").write_text(json.dumps(targets, indent=2) + "\n", encoding="utf-8")
    (output_dir / "metrics.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_status(output_dir, "completed")
    trace_path = Path(args.input_trace).expanduser().resolve() if args.input_trace else None
    metadata_extra = {
        "record_type": "experiment_run_provenance",
        "run": {
            "id": output_dir.name,
            "output_scope": (
                "experiment_evidence"
                if output_dir.is_relative_to(ROOT / "experiments")
                else "smoke_or_external"
            ),
            "path": str(output_dir),
        },
        "execution": {
            "working_directory": str(ROOT),
            "python_executable": sys.executable,
        },
        "source": {"teleop_source_sha256": _teleop_source_hashes()},
        "inputs": {
            "trace": {
                "path": str(trace_path) if trace_path else "stdin",
                "sha256": _sha256(trace_path) if trace_path else None,
            },
            "replay_receive_lag_s": args.replay_receive_lag_s,
        },
    }
    write_metadata(output_dir, ROOT, metadata_extra)
    write_evidence_completeness(
        output_dir,
        {
            "metrics": (output_dir / "metrics.json").is_file(),
            "raw_commands": (output_dir / "raw_commands.jsonl").is_file(),
            "targets": (output_dir / "targets.json").is_file(),
            "video": False,
            "video_reason": "This replay uses FakeIsaacLabSink; it does not run a physics simulator.",
        },
    )
    print(f"Simulation-only R1 teleop replay recorded in: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
