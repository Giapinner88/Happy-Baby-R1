# Experiment record — M001 planar stand and slow walk

## Identity and status

- **Experiment ID:** `M001_flat_stand_walk`
- **System:** R1 `Unitree-R1-Flat`, MJLab/Unitree RL MJLab in `r1_env`
- **Status:** planned; no evidence run has been started.
- **Protocol config:** [`config.json`](config.json)

## Question and declared outcome

Can the unchanged R1 MJLab flat task complete a planar standing and slow-walk
PPO pilot while preserving a complete training evidence pack?

The primary outcome is execution/provenance completeness. A training-valid run
must complete its configured iterations and contain every artifact checked by
[`run.py`](run.py). A crash, NaN, protocol mismatch, or missing artifact makes
the training evidence incomplete. Stable locomotion remains unassessed until a
separate evaluation replay is recorded.

## Protocol

The underlying `Unitree-R1-Flat` task supplies plane terrain. M001 sets 30%
standing commands, forward/backward training commands limited to ±0.25 m/s,
zero lateral/yaw commands, and disables the velocity curriculum and interval
pushes. Its MJLab command implementation masks a sampled command whose norm is
at or below 0.1; hence ±0.10 m/s alone would not exercise walking. Evaluation
must consequently test 0, ±0.05 and ±0.10 m/s separately.

The R1 asset, task implementation, PPO architecture and reward definition are
otherwise unchanged. Units are metres, seconds, metres/second and radians/second
as named in the config.

## Execution contract

Run from repository root. The environment is `r1_env`; the runner invokes the
public workspace command and creates an immutable record beneath
`experiments/r1_locomotion/mjlab/runs/<run-id>/`. A fresh UTC ID is generated
unless a unique ID is supplied. An interrupted run is retained as an execution
failure; M001 v1 has no resume protocol.

## Evidence and handoff

Training must create checkpoints, TensorBoard events, periodic MP4 video,
scalar CSV and diagnostic plots. `evidence_completeness.json` inventories them.
Before any positive locomotion claim, collection or promotion, a follow-up
evaluation record must add raw state/action/command traces, metric CSV, video,
fall/timeout events and a pass/fail manifest.

## Run inventory

| Run ID | Configuration | Execution status | Scientific status | Included later? | Reason |
| --- | --- | --- | --- | --- | --- |
| — | `config.json` | not started | unassessed | no | M001 protocol created; no execution yet |
