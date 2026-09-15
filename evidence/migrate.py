"""Migrating pre-unification run directories onto the shared contract.

Migration renames legacy record files to their canonical names and fills in
`status.json` bookkeeping fields that the old teleop writer never emitted. It is
deliberately conservative:

- It never invents a scientific outcome, a git commit, or any measurement.
- It never touches evidence data files.
- It never overwrites an existing canonical file; a directory holding both names
  is reported as a conflict for a human to resolve.
- It writes `migration_record.json` into the run so the rename stays visible.
  Provenance recorded before migration names files that have moved, and that
  record is what reconciles the two.

`plan` reports what would change and touches nothing. `apply` performs it.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .contract import ALIASES, ARTIFACTS, SCHEMA_VERSION, canonical_name
from .record import RunRecord


MIGRATION_RECORD = "migration_record.json"


@dataclass
class RunMigration:
    """What migrating one run would change."""

    run_id: str
    path: Path
    renames: list[tuple[str, str]] = field(default_factory=list)
    status_additions: dict[str, object] = field(default_factory=dict)
    conflicts: list[str] = field(default_factory=list)
    already_migrated: bool = False

    @property
    def is_noop(self) -> bool:
        return not self.renames and not self.status_additions and not self.conflicts

    def describe(self) -> str:
        if self.already_migrated:
            return f"{self.run_id}: already migrated"
        if self.is_noop:
            return f"{self.run_id}: nothing to change"
        parts = []
        for source, target in self.renames:
            parts.append(f"rename {source} -> {target}")
        if self.status_additions:
            parts.append(f"add to status.json: {sorted(self.status_additions)}")
        for conflict in self.conflicts:
            parts.append(f"CONFLICT {conflict}")
        return f"{self.run_id}: " + "; ".join(parts)


def plan_run(run: RunRecord) -> RunMigration:
    """Compute the migration for one run without modifying anything."""

    migration = RunMigration(run_id=run.run_id, path=run.path)
    if (run.path / MIGRATION_RECORD).is_file():
        migration.already_migrated = True
        return migration

    for key in ARTIFACTS:
        target_name = canonical_name(key)
        # Look for legacy files directly rather than through `RunRecord.resolve`,
        # which prefers the canonical name and would therefore hide exactly the
        # case that matters: both names present, holding possibly different data.
        for legacy_name in ALIASES.get(target_name, ()):
            if not (run.path / legacy_name).is_file():
                continue
            if (run.path / target_name).is_file():
                migration.conflicts.append(
                    f"{legacy_name} and {target_name} both exist; resolve by hand"
                )
                continue
            migration.renames.append((legacy_name, target_name))

    status_path = run.resolve("status")
    if status_path is not None:
        status = run.status
        if "schema_version" not in status:
            migration.status_additions["schema_version"] = SCHEMA_VERSION
        if "updated_at" not in status:
            # The record never carried a timestamp, so the file mtime is the only
            # honest source. It is stored under a distinct key that says so.
            mtime = datetime.fromtimestamp(status_path.stat().st_mtime, tz=timezone.utc)
            migration.status_additions["updated_at"] = mtime.isoformat()
            migration.status_additions["updated_at_source"] = "status_file_mtime_at_migration"
    return migration


def apply_run(run: RunRecord, migration: RunMigration) -> RunMigration:
    """Perform a planned migration. Refuses when the plan found a conflict."""

    if migration.already_migrated or migration.is_noop:
        return migration
    if migration.conflicts:
        raise RuntimeError(
            f"Refusing to migrate {run.run_id}: " + "; ".join(migration.conflicts)
        )

    for source, target in migration.renames:
        (run.path / source).rename(run.path / target)

    if migration.status_additions:
        status_path = run.path / canonical_name("status")
        status = json.loads(status_path.read_text(encoding="utf-8"))
        status.update(migration.status_additions)
        status_path.write_text(json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    (run.path / MIGRATION_RECORD).write_text(
        json.dumps(
            {
                "schema_version": SCHEMA_VERSION,
                "migrated_at": datetime.now(timezone.utc).isoformat(),
                "renamed_files": [
                    {"from": source, "to": target} for source, target in migration.renames
                ],
                "status_fields_added": migration.status_additions,
                "note": (
                    "Record files were renamed to the shared run contract. Any provenance or "
                    "metadata written before this migration refers to the pre-migration names "
                    "listed above; no evidence data file was modified."
                ),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return migration


__all__ = ["MIGRATION_RECORD", "RunMigration", "apply_run", "plan_run"]
