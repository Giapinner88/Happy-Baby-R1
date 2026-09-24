# Architecture and compatibility

## Runtime paths

The single-policy path is unchanged at the configuration boundary:

```text
config/tuning.yaml
  -> locomotion_policy
  -> inspect ONNX input dimension
  -> 83-D Flat | 332-D FlatPlus H4 | 270-D Rough
  -> raw action
  -> metadata action scale/default position
  -> PD target sent over DDS
```

The meta path is enabled only by the explicit `--meta-package` option used by
`./run_sim.sh meta ...`:

```text
83-D observation
  -> meta-only command lock, training-range clamp and slew-rate governor
  -> fixed 50-frame history replay through recurrent slope_meta gate
  -> probabilities: slope_up, slope_down, NEUTRAL
  -> cold-start/confidence/agreement/dwell selector
  -> per-expert observation contract adapter
  -> slope_up and slope_down actor inference
  -> convert both actions to joint targets
  -> weighted target blend + output cross-fade
  -> convert to the common raw action contract
  -> common target position and common PD gains
```

No camera or terrain-height observation is added to the meta actor contract.
Both experts and the gate use the hardware-reproducible 83-D proprioceptive
observation in this order:

1. base angular velocity: 3
2. projected gravity: 3
3. velocity command: 3
4. gait phase: 2
5. joint position relative to the active common default: 24
6. joint velocity: 24
7. previous common raw action: 24

## Contract checks before motion

Construction of `SlopeMetaRuntime` fails closed unless all of these agree:

- registry semantics version and class order;
- gate input/output count, history length, expert names and class names;
- expert input/output dimensions;
- canonical R1 joint order and 83-D observation term order;
- action scale/default position metadata;
- common/expert PD stiffness and damping;
- finite values and valid action scales.

The package `SHA256SUMS` also protects the copied artifacts. The original
single-policy ONNX files are neither renamed nor modified.

## Selector and safety behavior

The packaged settings are 50 Hz, 50 history frames, one-second cold start,
0.75 enter confidence, 0.55 exit confidence, five agreement frames, 0.5 second
minimum dwell and 0.2 second cross-fade.

During one-second `STARTUP`, requested commands are forced to zero and the
unbiased 50/50 expert blend actively balances the robot. This avoids both an
unstable passive default-pose hold and a bootstrap expert that self-selects from
its own history. Its output is cross-faded from the common default target.

After startup, commands are clamped to the exported training ranges and slew at
0.5 m/s² for vx/vy and 1.0 rad/s² for yaw. `NEUTRAL`, including low-confidence
transition frames, preserves the command and last/initial expert weights.
`SAFETY_HOLD` is reserved for invalid observations or non-finite ONNX outputs:
it keeps the last commanded joint target and does not run either expert. A
continuous two-second invalid-data hold latches the target until `Reset()`.

## Compatibility promise

- `./run_sim.sh single ...` reads the existing `locomotion_policy` entry from
  `config/tuning.yaml`.
- Direct invocation of `build/run_policy` behaves as before when no meta option
  is supplied.
- Existing stack and simulator scripts remain intact.
- Existing policy artifacts stay in their original directories.
- Meta artifacts live in a new self-contained versioned package.
- The old Flat/Rough/scene contract tests run together with the new meta tests.

## Adding a future expert

Add its ONNX file and an `experts` entry to a new versioned package, then export
a gate whose `expert_names`, `class_names`, probability dimension and metadata
match that registry. Do not edit `slope_meta_v6` in place: create a new package
directory so the validated version remains reproducible.
