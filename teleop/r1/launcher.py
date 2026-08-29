"""Shared multi-process Quest pilot launcher.

The Quest vendor wrapper and IsaacLab live in different Conda environments, so
every live pilot is a bridge process piped into a simulator process. Both sides
must agree on one run id, one stop file and one evidence directory, which is why
allocation happens here and not in either child.

A pilot may insert one more process between the two: a solver that reads the
command stream and writes it back out with joint targets attached. The vendor
`xr_teleoperate` IK needs this, because it runs in the bridge environment and
not the simulator's. The stage is optional and the pipeline is otherwise
identical, so a pilot that does its own solving inside the simulator is
unaffected.

This module owns only the launch mechanics. Which protocol is being run, which
experiment directory it writes to and which simulator flags it needs are
supplied by the calling entry point.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
import time
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
    quest_ready_timeout_s: float | None = None
    solver_args: list[str] | None = None
    """Arguments to a solver stage placed between the bridge and the simulator.

    `None` keeps the original two-process pipeline. When set, these are appended
    to a `conda run -n <bridge env> python` invocation, because a solver that
    needs the bridge environment is the only reason this stage exists.
    """


def build_commands(spec: PilotLaunchSpec, output_dir: Path, stop_file: Path, connection_log: Path):
    # When the launcher waits for WebXR, the bridge must stay alive for both the
    # readiness window and the requested simulator duration.
    bridge_duration_s = spec.duration_s + (spec.quest_ready_timeout_s or 0.0)
    bridge_command = [
        "conda", "run", "--no-capture-output", "-n", BRIDGE_ENV,
        "python", "scripts/teleop/quest_bridge.py",
        "--host-ip", spec.host_ip,
        "--duration-s", str(bridge_duration_s),
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
    solver_command = None
    if spec.solver_args is not None:
        solver_command = [
            "conda", "run", "--no-capture-output", "-n", BRIDGE_ENV, "python",
            *spec.solver_args,
        ]
    return bridge_command, solver_command, sim_command


def _wait_for_quest_ready(
    connection_log: Path,
    bridge: subprocess.Popen,
    timeout_s: float,
) -> bool:
    """Wait until the bridge records a validated motion-ready WebXR sample."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if connection_log.is_file():
            for line in connection_log.read_text(encoding="utf-8").splitlines():
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if record.get("event") == "connected":
                    return True
        if bridge.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def run_pilot(spec: PilotLaunchSpec, dry_run: bool = False) -> int:
    """Allocate one run id, then run the bridge piped into the simulator."""

    if spec.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive.")
    if not 0.0 <= spec.trigger_value_threshold < 10.0:
        raise SystemExit("--trigger-value-threshold must be in [0, 10).")
    if spec.quest_ready_timeout_s is not None and spec.quest_ready_timeout_s <= 0.0:
        raise SystemExit("Quest readiness timeout must be positive when enabled.")
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

    bridge_command, solver_command, sim_command = build_commands(
        spec, output_dir, stop_file, connection_log
    )

    print(f"Protocol:       {spec.protocol}", file=sys.stderr, flush=True)
    print(f"Run id:         {run_id}", file=sys.stderr, flush=True)
    print(f"Evidence dir:   {output_dir}", file=sys.stderr, flush=True)
    print(f"Connection log: {final_connection_log} (staged at {connection_log})", file=sys.stderr, flush=True)
    print(f"Stop file:      touch {stop_file}", file=sys.stderr, flush=True)
    print("", file=sys.stderr, flush=True)
    if dry_run:
        print("bridge: " + " ".join(bridge_command), file=sys.stderr)
        if solver_command is not None:
            print("solver: " + " ".join(solver_command), file=sys.stderr)
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
    if spec.quest_ready_timeout_s is not None:
        print(
            f"Waiting up to {spec.quest_ready_timeout_s:g} s for Quest Enter VR before starting Isaac...",
            file=sys.stderr,
            flush=True,
        )
        if not _wait_for_quest_ready(connection_log, bridge, spec.quest_ready_timeout_s):
            bridge.terminate()
            try:
                bridge.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait()
            print(
                "Quest readiness timeout: Isaac was not started. Open the URL, accept the "
                "certificate and Enter VR during the readiness window.",
                file=sys.stderr,
                flush=True,
            )
            return 2
        print("Quest motion stream ready; starting Isaac.", file=sys.stderr, flush=True)
    solver = None
    try:
        if solver_command is not None:
            solver = subprocess.Popen(
                solver_command,
                cwd=spec.repo_root,
                stdin=bridge.stdout,
                stdout=subprocess.PIPE,
            )
            # Each stage keeps only the ends it uses, so a closed pipe still
            # propagates back up the chain when a later stage exits first.
            assert bridge.stdout is not None
            bridge.stdout.close()
            simulator = subprocess.Popen(sim_command, cwd=spec.repo_root, stdin=solver.stdout)
            assert solver.stdout is not None
            solver.stdout.close()
        else:
            simulator = subprocess.Popen(sim_command, cwd=spec.repo_root, stdin=bridge.stdout)
            assert bridge.stdout is not None
            bridge.stdout.close()
    except OSError:
        if solver is not None:
            solver.terminate()
        bridge.terminate()
        raise

    simulator_status = simulator.wait()
    solver_status = solver.wait() if solver is not None else 0
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
    solver_note = f"  solver exit={solver_status}" if solver is not None else ""
    print(
        f"bridge exit={bridge_status}{solver_note}  simulator exit={simulator_status}",
        file=sys.stderr,
        flush=True,
    )
    if final_connection_log.is_file():
        print(f"Connection log saved: {final_connection_log}", file=sys.stderr, flush=True)
    print(
        f"Inspect with: python3 scripts/experiments/r1_experiments.py show r1_teleop {run_id}",
        file=sys.stderr,
        flush=True,
    )
    return simulator_status or solver_status or bridge_status


__all__ = [
    "BRIDGE_ENV",
    "SIM_ENV",
    "PilotLaunchSpec",
    "build_commands",
    "run_pilot",
]
