"""Discovering every experiment and run in the repository from one registry.

`experiments/registry.json` is the single place that says which experiments
exist, where their runs live, and which document is authoritative for each.

Each protocol may own a separate run layout, for example
`T001/runs/<run-id>` or `T003/runs/<run-id>`. The run id is still prefixed by
the protocol that owns it (`t001_a_20260802_quest01` belongs to protocol
`t001_a`). The protocol sequence and its gates live in the experiment's own
pipeline document, not here — this module only needs to know which prefixes are
legitimate and the roots in which their evidence lives.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .record import RunRecord


REGISTRY_RELATIVE_PATH = "experiments/registry.json"


class CatalogError(RuntimeError):
    """Raised when the registry is missing, malformed, or points nowhere."""


@dataclass
class Experiment:
    """One registered experiment and everything needed to report on it."""

    id: str
    title: str
    root: Path
    run_root: str
    run_roots: dict[str, str] = field(default_factory=dict)
    protocol_records: dict[str, str] = field(default_factory=dict)
    record: str | None = None
    module: str | None = None
    environment: str | None = None
    how_to_run: str | None = None
    pipeline: str | None = None
    protocols: list[str] = field(default_factory=list)

    @property
    def run_root_path(self) -> Path:
        """Legacy/default run root for callers that need one display path.

        Multi-protocol experiments should use :meth:`runs`; it searches every
        declared root. Keeping this property preserves the public API for the
        single-root experiments already registered.
        """

        return self.root / self.run_root

    @property
    def run_root_paths(self) -> list[Path]:
        """Distinct evidence roots in deterministic registry order."""

        roots = self.run_roots.values() if self.run_roots else (self.run_root,)
        paths: list[Path] = []
        for relative in roots:
            path = self.root / relative
            if path not in paths:
                paths.append(path)
        return paths

    def runs(self) -> list[RunRecord]:
        """Every run directory belonging to this experiment.

        A directory counts as a run when it holds at least one contract record
        file, so placeholders and bulk output subdirectories are not mistaken
        for runs.
        """

        discovered: list[RunRecord] = []
        seen: set[Path] = set()
        for root in self.run_root_paths:
            if not root.is_dir():
                continue
            for run_dir in sorted(root.iterdir()):
                resolved = run_dir.resolve()
                if resolved not in seen and _is_run_directory(run_dir):
                    discovered.append(RunRecord(run_dir.name, run_dir, experiment=self.id))
                    seen.add(resolved)
        return sorted(discovered, key=lambda run: run.run_id)

    def protocol_of(self, run: RunRecord) -> str | None:
        """Which declared protocol owns a run, by run-id prefix.

        Longest prefix wins, so `t001_a` is not shadowed by a shorter `t001`.
        """

        for protocol in sorted(self.protocols, key=len, reverse=True):
            if run.run_id.startswith(protocol):
                return protocol
        return None

    def runs_by_protocol(self) -> dict[str, list[RunRecord]]:
        """Runs grouped under every declared protocol, including empty ones.

        Protocols with no runs are kept in the mapping: a protocol that has
        produced no evidence is a fact worth showing, not an absence to hide.
        Runs whose prefix matches nothing are grouped under `None`.
        """

        grouped: dict[str, list[RunRecord]] = {protocol: [] for protocol in self.protocols}
        for run in self.runs():
            grouped.setdefault(self.protocol_of(run), []).append(run)
        return grouped


def _is_run_directory(path: Path) -> bool:
    if not path.is_dir():
        return False
    from .contract import ARTIFACTS, accepted_names

    names = {name for key in ARTIFACTS for name in accepted_names(key)}
    return any((path / name).is_file() for name in names)


def load_registry(repo_root: Path) -> list[Experiment]:
    """Read `experiments/registry.json` into `Experiment` objects."""

    registry_path = repo_root / REGISTRY_RELATIVE_PATH
    if not registry_path.is_file():
        raise CatalogError(f"Experiment registry not found: {registry_path}")
    try:
        payload = json.loads(registry_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise CatalogError(f"Experiment registry is not valid JSON: {exc}") from exc

    entries = payload.get("experiments")
    if not isinstance(entries, list) or not entries:
        raise CatalogError("Experiment registry contains no experiments.")

    experiments: list[Experiment] = []
    for entry in entries:
        root = repo_root / str(entry["root"])
        if not root.is_dir():
            raise CatalogError(f"Registered experiment root does not exist: {root}")
        raw_run_roots = entry.get("run_roots", {})
        if not isinstance(raw_run_roots, dict) or not all(
            isinstance(protocol, str) and isinstance(relative, str)
            for protocol, relative in raw_run_roots.items()
        ):
            raise CatalogError(f"Registered run_roots must be a protocol-to-path mapping: {entry['id']}")
        raw_protocol_records = entry.get("protocol_records", {})
        if not isinstance(raw_protocol_records, dict) or not all(
            isinstance(protocol, str) and isinstance(relative, str)
            for protocol, relative in raw_protocol_records.items()
        ):
            raise CatalogError(
                f"Registered protocol_records must be a protocol-to-path mapping: {entry['id']}"
            )
        run_root = str(entry.get("run_root", "runs"))
        run_roots = {str(protocol): str(relative) for protocol, relative in raw_run_roots.items()}
        for relative in set(run_roots.values()) or {run_root}:
            path = root / relative
            if not path.is_dir():
                raise CatalogError(f"Registered run root does not exist: {path}")
        experiments.append(
            Experiment(
                id=str(entry["id"]),
                title=str(entry["title"]),
                root=root,
                run_root=run_root,
                run_roots=run_roots,
                protocol_records={
                    str(protocol): str(relative)
                    for protocol, relative in raw_protocol_records.items()
                },
                record=entry.get("record"),
                module=entry.get("module"),
                environment=entry.get("environment"),
                how_to_run=entry.get("how_to_run"),
                pipeline=entry.get("pipeline"),
                protocols=[str(value) for value in entry.get("protocols", [])],
            )
        )
    return experiments


def find_experiment(experiments: list[Experiment], needle: str) -> Experiment:
    """Resolve an experiment by exact id, then by unique substring."""

    for experiment in experiments:
        if experiment.id == needle:
            return experiment
    matches = [experiment for experiment in experiments if needle in experiment.id]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise CatalogError(f"No experiment matches {needle!r}. Known: {[e.id for e in experiments]}")
    raise CatalogError(f"{needle!r} matches several experiments: {[e.id for e in matches]}")


__all__ = [
    "REGISTRY_RELATIVE_PATH",
    "CatalogError",
    "Experiment",
    "find_experiment",
    "load_registry",
]
