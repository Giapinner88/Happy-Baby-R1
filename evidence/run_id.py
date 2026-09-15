"""Allocating fresh run identifiers.

A run id is `<protocol>_<UTC timestamp>`, matching the convention the locomotion
experiments already use (`I002_nominal_stand_20260728T105515Z`). The protocol
prefix is what attributes the run to its protocol, so it must be one the
experiment declares in `experiments/registry.json`.

Allocation is deliberately paranoid about collisions. Reusing an id is the one
mistake that stops a run before the operator is even involved, because runners
refuse to overwrite an existing directory, and it is also the mistake most likely
to destroy evidence if a runner were ever less careful.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path


RUN_ID_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]*$")


def timestamp_suffix(moment: datetime | None = None) -> str:
    """UTC stamp with second resolution, e.g. `20260802T103812Z`."""

    return (moment or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")


def new_run_id(protocol: str, moment: datetime | None = None) -> str:
    """Compose a run id from a protocol prefix and a UTC timestamp."""

    if not protocol or not RUN_ID_PATTERN.match(protocol):
        raise ValueError(f"Protocol prefix is not a usable run-id component: {protocol!r}")
    return f"{protocol}_{timestamp_suffix(moment)}"


def allocate_run_id(run_root: Path, protocol: str, sidecar_suffixes: tuple[str, ...] = ()) -> str:
    """Return a run id whose directory and sidecar paths are all free.

    `sidecar_suffixes` names files created beside the run directory rather than
    inside it — the bridge connection log, for instance. They are checked too,
    because a half-free id would fail one process after the other had started.

    Within the same second the timestamp repeats, so a numeric discriminator is
    appended rather than returning an id that is already taken.
    """

    base = new_run_id(protocol)
    candidate = base
    attempt = 1
    while _is_taken(run_root, candidate, sidecar_suffixes):
        candidate = f"{base}_{attempt}"
        attempt += 1
        if attempt > 1000:
            raise RuntimeError(f"Cannot allocate a free run id under {run_root} for {protocol!r}.")
    return candidate


def _is_taken(run_root: Path, run_id: str, sidecar_suffixes: tuple[str, ...]) -> bool:
    if (run_root / run_id).exists():
        return True
    return any((run_root / f"{run_id}{suffix}").exists() for suffix in sidecar_suffixes)


def protocol_of(run_id: str, protocols: list[str]) -> str | None:
    """Which declared protocol a run id belongs to; longest prefix wins."""

    for protocol in sorted(protocols, key=len, reverse=True):
        if run_id.startswith(protocol):
            return protocol
    return None


__all__ = ["RUN_ID_PATTERN", "allocate_run_id", "new_run_id", "protocol_of", "timestamp_suffix"]
