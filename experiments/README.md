# Experiments

Every experiment in this repository is registered in
[`registry.json`](registry.json) and records evidence against one shared run
contract. Tools read the registry; nothing discovers experiments by scanning
directories, so an experiment that is not registered does not exist as far as
reporting is concerned.

## Current experiments

| Experiment | Module | Environment | Record |
|---|---|---|---|
| `r1_teleop/quest3_sim_v1` | `teleop/r1/` | `tv` + `unitree_sim_env` | [experiment.md](r1_teleop/quest3_sim_v1/experiment.md) |
| `r1_locomotion/isaaclab` | `training/isaaclab/` | `unitree_sim_env` | [experiment.md](r1_locomotion/isaaclab/experiment.md) |
| `r1_locomotion/mjlab` | `training/mjlab/` | `mjlab_env` | [experiment.md](r1_locomotion/mjlab/experiment.md) |

Rather than reading that table, ask the tool — it reports what the records
actually say right now:

```bash
python3 scripts/experiments/r1_experiments.py index
```

## Layout

Most experiments use `runs/<run-id>/`. When a study has several independently
owned protocols, the registry may instead declare protocol-owned roots such as
`T001/runs/<run-id>/` and `T003/runs/<run-id>/`; Quest 3 teleop uses that form.
The run ID remains prefixed by its protocol —
`I002_nominal_stand_20260728T105527Z` belongs to `I002_nominal_stand`, and
`t001_a_20260802_quest01` belongs to `t001_a`. The registry lists legitimate
prefixes and their run roots; the longest matching prefix wins, so `t001_a` is
not shadowed by a shorter `t001`.

The protocol **sequence** and its gates are not modelled here. They belong to
each experiment's own pipeline document — the teleop T-series lives in
[arm_wrist_simulation_study_plan.md](r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md)
— because gate logic is scientific reasoning, not bookkeeping, and duplicating it
into a manifest creates two sources of truth that drift apart.

## The run contract

Every run directory, in either layout, records against the same contract. The
canonical names are defined once in [`evidence/contract.py`](../evidence/contract.py);
nothing else should hard-code them.

| File | Required | Holds |
|---|---|---|
| `metadata.json` | yes | Provenance: command, git commit, python, platform, asset hashes. |
| `resolved_config.json` | yes | The configuration actually in force, after resolution. |
| `status.json` | yes | `execution_status`, `scientific_outcome`, `reason`, `updated_at`, `schema_version`. |
| `experiment_config.json` | recommended | Immutable snapshot of the editable protocol config the run started from. |
| `evidence_completeness.json` | recommended | Which expected artifacts exist, and why any are missing. |
| `experiment_runner_command.txt` | recommended | The exact command line, to repeat the run verbatim. |

Everything else in a run directory is evidence data: traces, metrics, plots,
videos, checkpoints. Bulk generated output belongs in `logs/`, `outputs/`,
`videos/`, `derived/`, `sidecar_evaluations/` or `analysis/`, which reporting
summarizes by count and size instead of listing file by file.

Required files are required because without them a run cannot be traced back to
what produced it. The recommended ones are reported when absent rather than
enforced, because a run that died early legitimately never wrote them, and
recording that honestly is more useful than a fabricated file.

### Status vocabulary

`execution_status` is a fact about the process: `running`, `completed`, `failed`,
`aborted`.

`scientific_outcome` is a judgement about the declared criteria: `unassessed`,
`pass`, `fail`, `inconclusive`. **`unassessed` is the default and must stay the
default.** A runner knows its process exited zero; it does not know whether the
criteria declared before execution were met. Promoting a run to `pass` is a human
act performed against the experiment's own criteria. An experiment may record a
more specific string (the teleop capture uses `no_transport_data_received`); it
is preserved verbatim and reported as experiment-specific.

## Working with runs

```bash
# every experiment, its structure, and the outcome spread of its runs
python3 scripts/experiments/r1_experiments.py index

# declared protocols, their runs, and which have a record written
python3 scripts/experiments/r1_experiments.py protocols r1_teleop/quest3_sim_v1

# run table for one experiment, or all of them
python3 scripts/experiments/r1_experiments.py runs r1_locomotion/isaaclab

# everything one run holds, including contract issues
python3 scripts/experiments/r1_experiments.py show r1_teleop/quest3_sim_v1 t001_a_20260802

# check runs against the contract; exits non-zero on errors
python3 scripts/experiments/r1_experiments.py validate
```

All of these run on the host Python 3 with no simulator, ROS, or GPU dependency,
so the state of every experiment is readable without entering a Conda
environment.

## Writing a new run

Runners must not hard-code record filenames. Use the shared writer, which is pure
standard library and importable from every environment here:

```python
from evidence.writer import (
    write_metadata, write_resolved_config, write_experiment_config,
    write_runner_command, write_status, write_evidence_completeness,
)

write_experiment_config(run_dir, config)     # what you were asked to run
write_resolved_config(run_dir, resolved)     # what is actually in force
write_runner_command(run_dir)
write_metadata(run_dir, REPO_ROOT, {"asset_sha256": ...})
...                                          # produce evidence
write_evidence_completeness(run_dir, {...})
write_status(run_dir, "completed")           # scientific_outcome stays unassessed
```

Two rules that the tooling cannot enforce for you: never overwrite an existing
run directory, and never write a `scientific_outcome` your runner did not
actually evaluate.

## Migration

Runs recorded before this contract used `provenance.json` and
`config.resolved.json` for what are now `metadata.json` and
`resolved_config.json`. Both names remain readable, so no old record is reported
as missing. To move a legacy run onto the canonical names:

```bash
python3 scripts/experiments/r1_experiments.py migrate            # dry run
python3 scripts/experiments/r1_experiments.py migrate --apply
```

Migration renames record files and backfills `schema_version` and `updated_at`.
It never touches evidence data, never invents an outcome, and refuses any
directory holding both the old and new name. It writes `migration_record.json`
into the run listing every rename, because provenance recorded before migration
names files that have since moved and that record is what reconciles the two.
