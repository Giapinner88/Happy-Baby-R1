"""Shared run-evidence records for every experiment in this repository.

This package owns the contract that experiments record against: what a run
directory must contain, what the status vocabulary means, how to read a run
written before the contract existed, and how to report across experiments.

It depends on no experiment, no generated artifact, and no simulator.
"""

from .catalog import CatalogError, Experiment, find_experiment, load_registry
from .contract import (
    ARTIFACTS,
    EXECUTION_STATUS,
    RECOMMENDED,
    REQUIRED,
    SCHEMA_VERSION,
    SCIENTIFIC_OUTCOME,
    ValidationIssue,
    ValidationResult,
    accepted_names,
    canonical_name,
)
from .migrate import RunMigration, apply_run, plan_run
from .record import RunRecord
from .run_id import allocate_run_id, new_run_id, timestamp_suffix

__all__ = [
    "ARTIFACTS",
    "EXECUTION_STATUS",
    "RECOMMENDED",
    "REQUIRED",
    "SCHEMA_VERSION",
    "SCIENTIFIC_OUTCOME",
    "CatalogError",
    "Experiment",
    "RunMigration",
    "RunRecord",
    "ValidationIssue",
    "ValidationResult",
    "accepted_names",
    "allocate_run_id",
    "apply_run",
    "canonical_name",
    "find_experiment",
    "load_registry",
    "new_run_id",
    "plan_run",
    "timestamp_suffix",
]
