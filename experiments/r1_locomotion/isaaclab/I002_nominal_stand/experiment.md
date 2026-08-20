# Experiment record — I002 nominal stable standing

## Identity and status

- **Experiment ID:** `I002_nominal_stand`
- **System:** R1 `Unitree-R1-Stand`, IsaacLab/Unitree RL Lab, directly
  installed `unitree_sim_env`
- **Status:** executable; no I002 evidence run has started.
- **Protocol config:** [`config.json`](config.json)
- **Evaluation protocol:** [`evaluation.json`](evaluation.json)

## Scientific question and criteria declared before execution

Can a nominal R1 IsaacLab policy hold the default upright posture on a flat
plane at a zero velocity command for 20 seconds?

The primary outcome is the fixed replay in `evaluation.json`, not a training
video or a reward curve. It passes only when it has no termination, maximum
tilt at most `0.35 rad`, base height never below `0.68 m`, post-settling body
XY speed RMS at most `0.10 m/s`, and post-settling XY displacement at most
`0.20 m`. A completed replay that misses any criterion is a valid scientific
failure. Crash, missing outputs, NaN, changed protocol, or missing provenance
is execution failure or invalid evidence, as appropriate.

## Protocol

I002 is deliberately a clean nominal baseline, not a locomotion or robustness
claim: plane terrain; zero command; fixed initial base pose; zero reset joint
velocity; no startup material or base-mass randomization; no pushes; no policy
observation corruption. It uses a separate task so P001's velocity task and
its existing evidence are not changed.

The standing task disables gait and foot-clearance rewards, adds posture and
two-foot-contact terms, penalizes termination, and uses R1 position-PD groups
and lower-body action scales aligned with the project-owned MJLab R1 recipe.
Arms and head remain held at their default posture and are not a learned
subtask. This alignment is a simulator configuration decision only; it is not
evidence of hardware compatibility.

## Execution contract

Run from repository root with the directly installed `unitree_sim_env`. Each
training invocation creates a unique immutable directory under
`experiments/r1_locomotion/isaaclab/runs/`; it never writes to P001 or legacy
`data/runs/rl_lab`. The runner records resolved configuration, exact command,
commit/environment metadata, checkpoints, exports, TensorBoard data, video,
CSV and plots. It refuses to regard a missing required artifact as complete.

Evaluation reads one stable checkpoint and writes a new, empty directory below
that run. It records raw CSV/NPZ trace, metrics, plots, MP4, protocol snapshot,
asset/checkpoint hashes and a promotion-compatible evaluation manifest.

## Run inventory

| Run ID | Configuration | Execution status | Scientific status | Included later? | Reason |
| --- | --- | --- | --- | --- | --- |
| — | `config.json` | not started | unassessed | no | I002 protocol created; no execution yet |

## Handoff

If an I002 checkpoint passes the declared simulation replay, the next
artifact is an analysis record or a separately declared disturbance/slow-walk
experiment. It does not authorize hardware use, teleop base velocity, or
promotion by itself.
