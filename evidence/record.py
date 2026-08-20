"""Reading one run directory as a record, in either the old or new vocabulary."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .contract import (
    ARTIFACTS,
    EXECUTION_STATUS,
    RECOMMENDED,
    REQUIRED,
    SCHEMA_VERSION,
    SCIENTIFIC_OUTCOME,
    ValidationResult,
    accepted_names,
    canonical_name,
)


# Directories inside a run that hold bulk generated output rather than record
# files. They are summarized by size and count instead of being walked into.
BULK_SUBDIRECTORIES = ("logs", "outputs", "videos", "derived", "sidecar_evaluations", "analysis")


def _read_json(path: Path) -> dict[str, object] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


@dataclass
class RunRecord:
    """One run directory, resolved against the shared contract."""

    run_id: str
    path: Path
    experiment: str
    stage: str | None = None

    def resolve(self, key: str) -> Path | None:
        """Path of an artifact, accepting its legacy name if that is what exists."""

        for name in accepted_names(key):
            candidate = self.path / name
            if candidate.is_file():
                return candidate
        return None

    def uses_legacy_name(self, key: str) -> str | None:
        """Legacy filename in use for this artifact, or None if canonical/absent."""

        resolved = self.resolve(key)
        if resolved is None or resolved.name == canonical_name(key):
            return None
        return resolved.name

    def load(self, key: str) -> dict[str, object] | None:
        path = self.resolve(key)
        return _read_json(path) if path is not None else None

    @property
    def status(self) -> dict[str, object]:
        return self.load("status") or {}

    @property
    def metadata(self) -> dict[str, object]:
        return self.load("metadata") or {}

    @property
    def execution_status(self) -> str:
        return str(self.status.get("execution_status", "unknown"))

    @property
    def scientific_outcome(self) -> str:
        return str(self.status.get("scientific_outcome", "unassessed"))

    @property
    def updated_at(self) -> str | None:
        value = self.status.get("updated_at")
        if value:
            return str(value)
        # Legacy teleop records have no timestamp field; fall back to the
        # status file's mtime and label it as derived, never as recorded.
        path = self.resolve("status")
        if path is None:
            return None
        return datetime.fromtimestamp(path.stat().st_mtime, tz=timezone.utc).isoformat()

    @property
    def git_commit(self) -> str | None:
        value = self.metadata.get("git_commit")
        return str(value) if value else None

    def data_files(self) -> list[Path]:
        """Evidence data files: everything that is not a contract record file."""

        from .migrate import MIGRATION_RECORD

        record_names = {name for key in ARTIFACTS for name in accepted_names(key)}
        record_names.add(MIGRATION_RECORD)
        return sorted(
            path
            for path in self.path.iterdir()
            if path.is_file() and path.name not in record_names and not path.name.startswith(".")
        )

    def bulk_directories(self) -> dict[str, dict[str, int]]:
        """File count and byte size of each bulk output subdirectory present."""

        summary: dict[str, dict[str, int]] = {}
        for name in BULK_SUBDIRECTORIES:
            directory = self.path / name
            if not directory.is_dir():
                continue
            files = [path for path in directory.rglob("*") if path.is_file()]
            summary[name] = {
                "file_count": len(files),
                "byte_size": sum(path.stat().st_size for path in files),
            }
        return summary

    def total_byte_size(self) -> int:
        return sum(path.stat().st_size for path in self.path.rglob("*") if path.is_file())

    def validate(self) -> ValidationResult:
        """Check this run against the shared contract.

        Missing required records are errors because the run cannot be traced.
        Everything else is a warning: a run that failed early is allowed to be
        incomplete, and reporting that honestly is the point.
        """

        result = ValidationResult(run_id=self.run_id)

        for key in REQUIRED:
            if self.resolve(key) is None:
                result.add("error", "missing_required_artifact", f"{canonical_name(key)} is absent")
        for key in RECOMMENDED:
            if self.resolve(key) is None:
                result.add("warning", "missing_recommended_artifact", f"{canonical_name(key)} is absent")

        for key in ARTIFACTS:
            legacy = self.uses_legacy_name(key)
            if legacy is not None:
                result.add(
                    "warning",
                    "legacy_artifact_name",
                    f"{legacy} still uses the pre-unification name for {canonical_name(key)}",
                )

        status = self.status
        if status:
            execution = status.get("execution_status")
            if execution not in EXECUTION_STATUS:
                result.add(
                    "warning",
                    "unknown_execution_status",
                    f"execution_status={execution!r} is outside {list(EXECUTION_STATUS)}",
                )
            outcome = status.get("scientific_outcome")
            if outcome is not None and outcome not in SCIENTIFIC_OUTCOME:
                result.add(
                    "warning",
                    "experiment_specific_outcome",
                    f"scientific_outcome={outcome!r} is an experiment-specific string",
                )
            if "schema_version" not in status:
                result.add("warning", "missing_schema_version", "status.json has no schema_version")
            elif status.get("schema_version") != SCHEMA_VERSION:
                result.add(
                    "warning",
                    "schema_version_mismatch",
                    f"status.json schema_version={status.get('schema_version')!r}, expected {SCHEMA_VERSION}",
                )
            if "updated_at" not in status:
                result.add("warning", "missing_updated_at", "status.json has no updated_at timestamp")
            if execution == "running":
                result.add(
                    "warning",
                    "run_still_marked_running",
                    "execution_status is 'running'; confirm the process is alive or correct the record",
                )

        if not self.data_files() and not self.bulk_directories():
            result.add("warning", "no_data_files", "the run directory holds record files only")

        return result


__all__ = ["BULK_SUBDIRECTORIES", "RunRecord"]
