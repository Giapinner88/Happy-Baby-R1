# P001 archive analysis — velocity-task port pilot

## Identity and scope

- **Analysis ID:** `P001_velocity_port_pilot_training_analysis_v1`
- **Source run:** `P001_velocity_port_pilot_20260728T025327Z`, commit
  `7e223e492bff7622c4f4044344a3fd6546b4ed82`
- **Status:** completed archival analysis; no independent replay was executed.

## Evidence intake

The run completed 2500 iterations and `evidence_completeness.json` records all
declared training products, including six checkpoints, TensorBoard event data,
training MP4s, derived scalar CSV/plots, and JIT/ONNX export. Its original
task was `Unitree-R1-Velocity` with a plane and zero velocity command. The
separate deterministic checkpoint evaluation required by the experiment record
was never run, so `status.json` remains scientifically unassessed.

## Direct observations

- `Train/mean_episode_length` fell from 13.0 at iteration 0 to 6.08 control
  steps at iteration 2499. With the recorded 20 ms control step this is about
  0.12 s per episode.
- `Episode_Termination/bad_orientation` was 1.0 at iteration 2499; base-height
  termination was 0.0. The dominant observed failure is therefore orientation,
  not the configured height threshold.
- `Train/mean_reward` improved from -1.951 to -0.797, but that scalar did not
  coincide with longer episodes or reduced orientation termination.
- The final training video `rl-video-step-55000.mp4` visibly contains many
  fallen or strongly bent robots throughout its 20-second recording. It is a
  512-environment training view, not a deterministic evaluation replay.

## Candidate finding and limitation

Under this ported G1 velocity-task configuration, zero-command R1 standing
was not learned in the observed training distribution. Evidence strength is
**exploratory**, because no fixed single-environment replay/trace was recorded
for the final checkpoint. The data support archiving this as a negative pilot;
they do not identify a unique physical cause or establish that every exported
checkpoint fails.

## Next action

Run I002's clean nominal standing protocol. It changes the declared task and
is not comparable as a continuation of this pilot. Do not promote or deploy
the exported P001 policy.
