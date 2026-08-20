"""Shared two-process Quest pilot launcher.

The Quest vendor wrapper and IsaacLab live in different Conda environments, so
every live pilot is a bridge process piped into a simulator process. Both sides
must agree on one run id, one stop file and one evidence directory, which is why
allocation happens here and not in either child.

This module owns only the launch mechanics. Which protocol is being run, which
experiment directory it writes to and which simulator flags it needs are
supplied by the calling entry point.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

from evidence.run_id import allocate_run_id


BRIDGE_ENV = "tv"
SIM_ENV = "unitree_sim_env"


@dataclass(frozen=True)
class PilotLaunchSpec:
    """One protocol's launch definition."""

    protocol: str
    run_root: Path
    repo_root: Path
    host_ip: str
    duration_s: float
    cert_file: Path
    key_file: Path
    physics_hz: float
    control_hz: float
    trigger_value_threshold: float
    stop_file_dir: Path
    disable_self_collisions: bool
    extra_sim_args: list[str] = field(default_factory=list)
    idle_stop_s: float | None = None


def build_commands(spec: PilotLaunchSpec, output_dir: Path, stop_file: Path, connection_log: Path):
    bridge_command = [
        "conda", "run", "--no-capture-output", "-n", BRIDGE_ENV,
        "python", "scripts/teleop/quest_bridge.py",
        "--host-ip", spec.host_ip,
        "--duration-s", str(spec.duration_s),
        "--trigger-value-threshold", str(spec.trigger_value_threshold),
        "--cert-file", str(spec.cert_file.expanduser()),
        "--key-file", str(spec.key_file.expanduser()),
        "--stop-file", str(stop_file),
        "--connection-log", str(connection_log),
    ]
    sim_command = [
        "conda", "run", "--no-capture-output", "-n", SIM_ENV,
        "python", "scripts/teleop/run_r1_quest3_live.py",
        "--output-dir", str(output_dir),
        "--duration-s", str(spec.duration_s),
        "--physics-hz", str(spec.physics_hz),
        "--control-hz", str(spec.control_hz),
        "--stop-file", str(stop_file),
    ]
    if spec.disable_self_collisions:
        sim_command.append("--disable-self-collisions")
    if spec.idle_stop_s is not None:
        sim_command += ["--idle-stop-s", str(spec.idle_stop_s)]
    sim_command += spec.extra_sim_args
    return bridge_command, sim_command


def run_pilot(spec: PilotLaunchSpec, dry_run: bool = False) -> int:
    """Allocate one run id, then run the bridge piped into the simulator."""

    if spec.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive.")
    if not 0.0 <= spec.trigger_value_threshold < 10.0:
        raise SystemExit("--trigger-value-threshold must be in [0, 10).")
    for path in (spec.cert_file, spec.key_file):
        if not path.expanduser().is_file():
            raise SystemExit(f"Certificate file does not exist: {path}")

    spec.run_root.mkdir(parents=True, exist_ok=True)
    run_id = allocate_run_id(spec.run_root, spec.protocol)
    output_dir = spec.run_root / run_id
    # The simulator refuses a pre-existing evidence directory, while the bridge
    # starts first and needs a writable log path. Stage the log as a sibling and
    # place it inside the immutable run after both processes exit.
    staging_root = spec.run_root / ".staging"
    staging_root.mkdir(parents=True, exist_ok=True)
    connection_log = staging_root / f"{run_id}.bridge.jsonl"
    final_connection_log = output_dir / "bridge_connection.jsonl"
    stop_file = spec.stop_file_dir.expanduser() / f"{run_id}.stop"
    if stop_file.exists():
        raise SystemExit(f"Refusing to start: stop file already exists: {stop_file}")

    bridge_command, sim_command = build_commands(spec, output_dir, stop_file, connection_log)

    print(f"Protocol:       {spec.protocol}", file=sys.stderr, flush=True)
    print(f"Run id:         {run_id}", file=sys.stderr, flush=True)
    print(f"Evidence dir:   {output_dir}", file=sys.stderr, flush=True)
    print(f"Connection log: {final_connection_log} (staged at {connection_log})", file=sys.stderr, flush=True)
    print(f"Stop file:      touch {stop_file}", file=sys.stderr, flush=True)
    print("", file=sys.stderr, flush=True)
    if dry_run:
        print("bridge: " + " ".join(bridge_command), file=sys.stderr)
        print("sim:    " + " ".join(sim_command), file=sys.stderr)
        return 0

    print(
        "In the headset: open the printed URL, ENTER VR, hold the right trigger and move.\n"
        "If no 'connected' line appears within ~20 s, stop and retry: the immersive\n"
        "session did not start and a longer run only records an empty one.\n",
        file=sys.stderr,
        flush=True,
    )

    # SIGINT must reach the children as a graceful stop, and the launcher must not
    # die first and orphan a running Isaac Sim.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    bridge = subprocess.Popen(bridge_command, cwd=spec.repo_root, stdout=subprocess.PIPE)
    try:
        simulator = subprocess.Popen(sim_command, cwd=spec.repo_root, stdin=bridge.stdout)
    except OSError:
        bridge.terminate()
        raise
    # Only the simulator holds the read end now; otherwise the bridge never sees
    # a closed pipe if the simulator exits first.
    assert bridge.stdout is not None
    bridge.stdout.close()

    simulator_status = simulator.wait()
    bridge_status = bridge.wait()
    if connection_log.is_file() and output_dir.is_dir():
        if final_connection_log.exists():
            raise RuntimeError(f"Refusing to overwrite bridge evidence: {final_connection_log}")
        connection_log.replace(final_connection_log)
        completeness_path = output_dir / "evidence_completeness.json"
        if completeness_path.is_file():
            completeness = json.loads(completeness_path.read_text(encoding="utf-8"))
            completeness["bridge_connection_log"] = True
            completeness_path.write_text(
                json.dumps(completeness, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )

    print("", file=sys.stderr, flush=True)
    print(f"bridge exit={bridge_status}  simulator exit={simulator_status}", file=sys.stderr, flush=True)
    if final_connection_log.is_file():
        print(f"Connection log saved: {final_connection_log}", file=sys.stderr, flush=True)
    print(
        f"Inspect with: python3 scripts/experiments/r1_experiments.py show r1_teleop {run_id}",
        file=sys.stderr,
        flush=True,
    )
    return simulator_status or bridge_status


__all__ = ["BRIDGE_ENV", "SIM_ENV", "PilotLaunchSpec", "build_commands", "run_pilot"]
