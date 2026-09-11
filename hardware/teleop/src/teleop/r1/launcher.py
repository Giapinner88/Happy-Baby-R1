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

import ipaddress
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

from evidence.run_id import allocate_run_id


BRIDGE_ENV = "tv"
SIM_ENV = "unitree_sim_env"


def ensure_self_signed_certificate(host_ip: str, cert_file: Path, key_file: Path) -> bool:
    """Ensure a current local HTTPS certificate for ``host_ip`` exists.

    A valid existing pair is reused so the headset does not need to trust a
    different certificate on every launch. An expired, malformed, or wrong-SAN
    pair is atomically replaced. A partial pair is not repaired by overwriting
    the remaining credential; the operator must inspect it first.
    """

    try:
        address = ipaddress.ip_address(host_ip)
    except ValueError as exc:
        raise SystemExit(f"Cannot create certificate: invalid --host-ip {host_ip!r}.") from exc

    cert_path = cert_file.expanduser()
    key_path = key_file.expanduser()
    cert_exists = cert_path.is_file()
    key_exists = key_path.is_file()
    if cert_exists and key_exists:
        inspection = subprocess.run(
            [
                "openssl", "x509", "-in", str(cert_path), "-noout",
                "-checkend", "0", "-ext", "subjectAltName",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if inspection.returncode == 0 and f"IP Address:{address}" in inspection.stdout:
            return False
    if cert_exists != key_exists:
        missing = key_path if cert_exists else cert_path
        existing = cert_path if cert_exists else key_path
        raise SystemExit(
            f"Refusing to replace a partial certificate pair: {existing} exists but "
            f"{missing} does not. Move or repair the pair, then retry."
        )

    cert_path.parent.mkdir(parents=True, exist_ok=True)
    key_path.parent.mkdir(parents=True, exist_ok=True)
    cert_handle = tempfile.NamedTemporaryFile(
        prefix=f".{cert_path.name}.", dir=cert_path.parent, delete=False
    )
    key_handle = tempfile.NamedTemporaryFile(
        prefix=f".{key_path.name}.", dir=key_path.parent, delete=False
    )
    temporary_cert = Path(cert_handle.name)
    temporary_key = Path(key_handle.name)
    cert_handle.close()
    key_handle.close()
    try:
        result = subprocess.run(
            [
                "openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256",
                "-nodes", "-days", "365", "-keyout", str(temporary_key),
                "-out", str(temporary_cert), "-subj", f"/CN={address}",
                "-addext", f"subjectAltName=IP:{address}",
                "-addext", "keyUsage=digitalSignature,keyEncipherment",
                "-addext", "extendedKeyUsage=serverAuth",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or "unknown OpenSSL error"
            raise SystemExit(f"Failed to create HTTPS certificate with OpenSSL: {detail}")
        os.chmod(temporary_key, 0o600)
        os.chmod(temporary_cert, 0o644)
        temporary_key.replace(key_path)
        temporary_cert.replace(cert_path)
    except OSError as exc:
        raise SystemExit(f"Failed to create HTTPS certificate: {exc}") from exc
    finally:
        temporary_cert.unlink(missing_ok=True)
        temporary_key.unlink(missing_ok=True)
    return True


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
    if ensure_self_signed_certificate(spec.host_ip, spec.cert_file, spec.key_file):
        print(
            f"Created self-signed Quest certificate for {spec.host_ip}: "
            f"{spec.cert_file.expanduser()}",
            file=sys.stderr,
            flush=True,
        )

    spec.run_root.mkdir(parents=True, exist_ok=True)
    run_id = allocate_run_id(spec.run_root, spec.protocol)
    output_dir = spec.run_root / run_id
    # The simulator refuses a pre-existing evidence directory, while the bridge
    # starts first and needs a writable log path. Stage the log as a sibling and
    # place it inside the immutable run after both processes exit.
    staging_root = spec.run_root / ".staging"
    if not dry_run:
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

    finalizer_status = 0
    if output_dir.is_dir():
        finalizer_environment = os.environ.copy()
        finalizer_environment.setdefault("MPLCONFIGDIR", "/tmp/hb-matplotlib")
        finalizer_status = subprocess.run(
            [
                "conda", "run", "--no-capture-output", "-n", SIM_ENV,
                "python", "scripts/teleop/finalize_r1_run.py", "simulation", str(output_dir),
            ],
            cwd=spec.repo_root,
            env=finalizer_environment,
        ).returncode

    print("", file=sys.stderr, flush=True)
    solver_note = f"  solver exit={solver_status}" if solver is not None else ""
    print(
        f"bridge exit={bridge_status}{solver_note}  simulator exit={simulator_status}  "
        f"artifact gate exit={finalizer_status}",
        file=sys.stderr,
        flush=True,
    )
    if final_connection_log.is_file():
        print(f"Connection log saved: {final_connection_log}", file=sys.stderr, flush=True)
    print(f"Run output: {output_dir}", file=sys.stderr, flush=True)
    return simulator_status or solver_status or bridge_status or finalizer_status


__all__ = [
    "BRIDGE_ENV",
    "SIM_ENV",
    "PilotLaunchSpec",
    "build_commands",
    "ensure_self_signed_certificate",
    "run_pilot",
]
