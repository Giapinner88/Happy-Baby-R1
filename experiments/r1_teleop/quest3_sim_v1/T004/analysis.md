# T004 initial-sweep analysis

## Intake and scope

Source is the complete 22-case selection in
[`metadata/initial_sweep_selection.json`](metadata/initial_sweep_selection.json)
and its unfiltered aggregate table in
[`figures/t004_initial_20260811T044700Z/case_table.csv`](figures/t004_initial_20260811T044700Z/case_table.csv).
All cases completed, every trace converged, no case was clamped, and all
repeatability checks passed. There are no execution failures, scientific
failures or missing cases to exclude.

## Direct observations

The reported minimum joint-limit margin ranges from 0.08143 to 0.40520 rad
across the declared one-at-a-time perturbations. A ±10° yaw changes the mapped
target by up to 47.65 mm; every ±30 mm translation changes it by exactly 30 mm.
These are direct outputs of the yaw-then-translation mapper plus fixed IK
solver, not empirical calibration-error distributions.

## Limitation and next action

The source is a synthetic identity-calibrated trace reconstructed from T003-A,
with the left trace mirrored from the right. It has no live Quest calibration
estimate, Isaac dynamics, contact/collision, or hardware evidence. The next
smallest discriminating action is a T004 follow-up that expands only the
lowest-margin direction around the observed 30 mm / 10° screen, before opening
T005.

