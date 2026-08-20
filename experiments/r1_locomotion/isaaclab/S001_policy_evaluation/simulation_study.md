# S001 — IsaacSim evaluation of archived P001 locomotion candidates

## Identity and status

- **Study ID:** `S001_policy_evaluation`
- **Status:** planned; execution is blocked until the workstation exposes CUDA.
- **Candidates:** archived P001 candidates in [`config.json`](config.json)
- **Scope:** IsaacSim task-native evaluation first. MuJoCo is a separate transfer gate, not a second execution backend yet.

## Question and design

Which archived P001 checkpoint has the better observed standing and slow forward/backward
velocity behavior under the same nominal IsaacSim protocol? The matrix is two
candidates × five commands × three fixed seeds. A termination before 20 seconds
is a scientific failure; execution failures and missing evidence remain visible.

Each case must preserve resolved config, command, checkpoint hash, raw
state/action/command trace, termination events, metric CSV, MP4 and status. The
primary metrics are velocity tracking error and completed-duration rate; root
height, projected-gravity, joint-limit margin, action rate and termination time
are diagnostics. S001 ranks candidates; it does not authorize promotion.

The present MuJoCo runtime cannot receive these policies directly: it hard-codes
24 actions/83 observations while the IsaacLab R1 task applies actions to all
R1 joints. A MuJoCo run requires a documented observation/action/controller
parity method and an independent compatibility experiment first.

## Execution contract

The future evidence runner must create
`runs/S001_<utc>/<candidate>/<case>/<seed>/` without overwrite. Until that
runner is implemented, the root demo below is disposable only and cannot be
entered into this study inventory.

## Run inventory

| Run ID | Status | Reason |
| --- | --- | --- |
| — | not started | CUDA driver unavailable during the export smoke attempt |
