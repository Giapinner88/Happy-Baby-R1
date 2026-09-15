# R1 Quest 3 Teleop Simulation v1

This protocol evaluates the simulation-only R1 Quest 3 command path. Its
experiment pipeline is
[arm_wrist_simulation_study_plan.md](arm_wrist_simulation_study_plan.md), which
owns the T-series questions, their evidence requirements and their gates. Each
T-series directory owns its own human record, editable configuration, metadata,
derived figures and immutable run evidence.

The v1 protocol controls R1 head and upper-body targets plus a bounded base
velocity command interface. It never opens a hardware command channel. Velocity
execution remains disabled until a separately recorded IsaacLab locomotion
evaluation accepts a matching policy signature.

The T001-A/T001-B split and the reserved hardware protocols T009/T010 are fixed
by [D001](../../../decisions/r1_teleop/D001_teleop_stage_gates.md).

## Current state

The retained runs are indexed by evaluation part and case in the owning
T-directory metadata: [T001](T001/T001.md), [T002](T002/T002.md), and
[T003](T003/T003.md). Those catalogs, rather than this prose, record the
selected, excluded, superseded and partial evidence.
T003 is now split into T003-A nominal tracking and T003-B atomic safety-hold
cases. Calibration, synchronization and mimic questions still inherit those
gates; none may be claimed before their own protocol evidence exists.

Read the live state rather than this paragraph:

```bash
python3 scripts/experiments/r1_experiments.py protocols r1_teleop
python3 scripts/experiments/r1_experiments.py runs r1_teleop
```

## Runs

Runs live under their owning experiment: `T001/runs/<run-id>/`,
`T002/runs/<run-id>/`, and so on. A run ID remains prefixed by its protocol
(`t001_a_20260802_quest01` belongs to `T001`). Directories are never overwritten
and follow the shared run contract in [`experiments/README.md`](../../README.md).

The root keeps only [`inputs/`](inputs/README.md) for reusable deterministic
fixtures plus this cross-T pipeline. T001–T008 each own `config/`, `figures/`,
`runs/` and their Markdown record; T001–T003 also have editable `metadata/`.

The repository retains deterministic input traces
([inputs/example_trace.jsonl](inputs/example_trace.jsonl),
[inputs/t001_mapper_trace.jsonl](inputs/t001_mapper_trace.jsonl),
[inputs/t001_sequence_violation_trace.jsonl](inputs/t001_sequence_violation_trace.jsonl)) for
the unnumbered mapper preflight. Preflight output is not T001 evidence: nothing
may be inferred from `FakeIsaacLabSink` about a Quest connection or a physics
simulator.
