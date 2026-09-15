"""The run-record contract shared by every experiment in this repository.

One vocabulary, one set of filenames, one status vocabulary. Before this module
the locomotion experiments wrote `metadata.json` / `resolved_config.json` while
the teleop experiment wrote `provenance.json` / `config.resolved.json` for the
same concepts, so nothing could report across them.

The canonical names are the locomotion ones, because more existing evidence
already used them and evidence directories are the thing least safe to churn.
Legacy names remain readable through `ALIASES` so a record written before the
unification is never silently reported as missing.

This module holds no experiment-specific knowledge: it does not know what a
teleop hold event or a locomotion reward curve is. Per-experiment expectations
belong in that experiment's declared data contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field


SCHEMA_VERSION = 1


# Canonical artifact key -> filename inside a run directory.
ARTIFACTS: dict[str, str] = {
    "metadata": "metadata.json",
    "experiment_config": "experiment_config.json",
    "resolved_config": "resolved_config.json",
    "status": "status.json",
    "evidence_completeness": "evidence_completeness.json",
    "runner_command": "experiment_runner_command.txt",
}

# Canonical filename -> filenames that meant the same thing before unification.
ALIASES: dict[str, tuple[str, ...]] = {
    "metadata.json": ("provenance.json",),
    "resolved_config.json": ("config.resolved.json",),
}

# A run without these cannot be traced back to what produced it.
REQUIRED: tuple[str, ...] = ("metadata", "resolved_config", "status")

# Expected for a complete record, but their absence is reported rather than
# fatal: a run that died early legitimately never wrote them.
RECOMMENDED: tuple[str, ...] = ("experiment_config", "evidence_completeness", "runner_command")


EXECUTION_STATUS: tuple[str, ...] = ("running", "completed", "failed", "aborted")
"""How the process ended. This is a fact about execution, not about science."""

SCIENTIFIC_OUTCOME: tuple[str, ...] = ("unassessed", "pass", "fail", "inconclusive")
"""Whether the declared criteria were met.

`unassessed` is the honest default and must stay the default: a completed run is
not a passing run until someone checks it against the criteria that were declared
before execution. An experiment may record a more specific outcome string, which
is preserved verbatim and reported as `other`.
"""


@dataclass
class ValidationIssue:
    """One contract deviation found in a run directory."""

    severity: str  # "error" | "warning"
    code: str
    detail: str

    def __str__(self) -> str:
        return f"[{self.severity}] {self.code}: {self.detail}"


@dataclass
class ValidationResult:
    run_id: str
    issues: list[ValidationIssue] = field(default_factory=list)

    @property
    def errors(self) -> list[ValidationIssue]:
        return [issue for issue in self.issues if issue.severity == "error"]

    @property
    def warnings(self) -> list[ValidationIssue]:
        return [issue for issue in self.issues if issue.severity == "warning"]

    @property
    def ok(self) -> bool:
        return not self.errors

    def add(self, severity: str, code: str, detail: str) -> None:
        if severity not in ("error", "warning"):
            raise ValueError(f"Unknown severity: {severity}")
        self.issues.append(ValidationIssue(severity, code, detail))


def canonical_name(key: str) -> str:
    """Return the canonical filename for an artifact key."""

    try:
        return ARTIFACTS[key]
    except KeyError:
        raise KeyError(f"Unknown run artifact key: {key}. Known keys: {sorted(ARTIFACTS)}") from None


def accepted_names(key: str) -> tuple[str, ...]:
    """Canonical filename first, then any legacy names that still mean it."""

    name = canonical_name(key)
    return (name, *ALIASES.get(name, ()))


__all__ = [
    "ALIASES",
    "ARTIFACTS",
    "EXECUTION_STATUS",
    "RECOMMENDED",
    "REQUIRED",
    "SCHEMA_VERSION",
    "SCIENTIFIC_OUTCOME",
    "ValidationIssue",
    "ValidationResult",
    "accepted_names",
    "canonical_name",
]
