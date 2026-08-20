# Experiment record — P001 archived velocity-task port pilot

## Identity and status

- **Experiment ID:** `P001_velocity_port_pilot`
- **System:** R1 `Unitree-R1-Velocity`, IsaacLab/Unitree RL Lab, directly
  installed `unitree_sim_env`
- **Status:** archived negative pilot; it is not an active training protocol.
- **Protocol config:** [`config.json`](config.json)

## Question and declared outcome

Could the unchanged R1 IsaacLab velocity task complete a planar standing
pilot while preserving a complete training evidence pack?

The primary outcome for P001 was execution/provenance completeness, not a
locomotion performance claim. A training-valid run must complete the configured
iterations and contain every `training_evidence` artifact enforced by
[`run.py`](run.py). A missing artifact, crash, NaN, or protocol mismatch makes
the training evidence incomplete. Stable locomotion remains unassessed until
the separately declared evaluation replay exists.

## Protocol

`config.json` fixes a plane, forward/backward commands limited to ±0.1 m/s,
zero lateral/yaw command, 30% standing commands, no curricula, and no interval
pushes. R1 asset, task implementation, PPO architecture, and reward definition
remain project defaults. Units are metres, seconds, metres/second, and
radians/second as named in the config.

## Execution contract

Working directory is repository root. Validate first, then use `run.py` with a
new ID. The public launcher records an immutable run below
`experiments/r1_locomotion/isaaclab/runs/<run-id>/`; the former protocol copied its editable
config and exact orchestration command into that directory after execution.

The run may be interrupted, but it is never overwritten. Resume is not part of
The archived protocol has no resume path; an interrupted run remains in the inventory as an execution failure.

## Evidence and handoff

Training creates checkpoints, TensorBoard events, MP4 video, scalar CSV and
diagnostic plots. `evidence_completeness.json` states which artifacts are
present. Raw state/action/command traces, simulation metrics, evaluation video,
and pass/fail manifest belong to the follow-up evaluation record. Do not use a
checkpoint, reward curve, or video alone to promote policy or modify shared
workflow code.

## Run inventory

| Run ID | Configuration | Execution status | Scientific status | Included later? | Reason |
| --- | --- | --- | --- | --- | --- |
| `P001_velocity_port_pilot_20260727T033124Z` | archived velocity-task snapshot | completed | unassessed | no | retained historical pilot evidence |
| `P001_velocity_port_pilot_20260728T025316Z` | archived velocity-task snapshot | stale `running` record | unassessed | no | no active process observed during archive |
| `P001_velocity_port_pilot_20260728T025327Z` | archived velocity-task snapshot | completed | negative pilot evidence | no | orientation termination dominated training; see `analysis.md` |
