"""Writing run records against the shared contract.

Runners call this instead of hard-coding filenames, so a change to the contract
does not have to be chased through every experiment script. It is pure standard
library and therefore importable from every environment in this repository
(`tv`, `unitree_sim_env`, `mjlab_env`, and the host interpreter).

It writes bookkeeping only. Measurements, metrics and scientific outcomes are
the runner's responsibility; nothing here derives a result.
"""

from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from .contract import EXECUTION_STATUS, SCHEMA_VERSION, canonical_name


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: object) -> Path:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def git_commit(repo_root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo_root, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def write_runner_command(run_dir: Path, argv: list[str] | None = None) -> Path:
    """Record the exact command line, so a run can be repeated verbatim."""

    command = argv if argv is not None else [sys.executable, *sys.argv]
    path = run_dir / canonical_name("runner_command")
    path.write_text(" ".join(command) + "\n", encoding="utf-8")
    return path


def write_metadata(run_dir: Path, repo_root: Path, extra: dict[str, object] | None = None) -> Path:
    """Provenance: what produced this run and on what."""

    import platform

    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "command": [sys.executable, *sys.argv],
        "git_commit": git_commit(repo_root),
        "python": sys.version,
        "platform": platform.platform(),
    }
    if extra:
        payload.update(extra)
    return write_json(run_dir / canonical_name("metadata"), payload)


def write_resolved_config(run_dir: Path, resolved: dict[str, object]) -> Path:
    return write_json(run_dir / canonical_name("resolved_config"), resolved)


def write_experiment_config(run_dir: Path, config: dict[str, object]) -> Path:
    """Immutable snapshot of the editable protocol config the run started from."""

    return write_json(run_dir / canonical_name("experiment_config"), config)


def write_status(
    run_dir: Path,
    execution_status: str,
    scientific_outcome: str = "unassessed",
    reason: str | None = None,
    extra: dict[str, object] | None = None,
) -> Path:
    """Status, with `unassessed` as the default scientific outcome.

    A runner knows whether its process finished; it does not know whether the
    declared criteria were met. Leaving the outcome `unassessed` keeps that
    judgement with whoever checks the run against the criteria.
    """

    if execution_status not in EXECUTION_STATUS:
        raise ValueError(f"execution_status must be one of {list(EXECUTION_STATUS)}, got {execution_status!r}")
    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "execution_status": execution_status,
        "scientific_outcome": scientific_outcome,
        "reason": reason,
        "updated_at": utc_now(),
    }
    if extra:
        payload.update(extra)
    return write_json(run_dir / canonical_name("status"), payload)


def write_evidence_completeness(run_dir: Path, present: dict[str, object]) -> Path:
    """Declare which expected artifacts exist, so gaps are explicit."""

    return write_json(run_dir / canonical_name("evidence_completeness"), present)


__all__ = [
    "git_commit",
    "utc_now",
    "write_evidence_completeness",
    "write_experiment_config",
    "write_json",
    "write_metadata",
    "write_resolved_config",
    "write_runner_command",
    "write_status",
]
