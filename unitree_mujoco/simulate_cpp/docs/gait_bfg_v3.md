# Gait B/F/G v3 in `simulate_cpp`

This route is opt-in. Existing `tuning.yaml` and the v1 gait scheduler remain
the rollback baseline.

## Artifact contract

The ONNX must be a 335-D `flat_plus_gait_h4_v1` model with complete metadata.
The runtime accepts only:

- `flat_plus_gait_fsm_v3b` for B;
- `flat_plus_gait_fsm_v3f` for F and G.

F/G share the gather-first scheduler. A checkpoint whose metadata only says
`flat_plus_gait_fsm_v3` is rejected; the training exporter must be called with
an explicit variant (`gait_metadata("b")`, `gait_metadata("f")`, or
`gait_metadata("g")`).

The deploy package is a reference bundle and does not contain the ONNX files.
Stage the re-exported artifact separately and record its SHA256 before testing.

## Preflight and run

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp
./run_sim.sh build
./run_sim.sh gait-preflight /abs/path/policy_b_or_f_or_g.onnx
./run_sim.sh gait_v3b flat --locomotion-policy /abs/path/policy_b.onnx
./run_sim.sh gait_v3f flat --locomotion-policy /abs/path/policy_f_or_g.onnx
```

The v3 commands load an example tuning file through
`SIMULATE_CPP_TUNING_CONFIG`; they never overwrite the live config. The command
slew, FK stance gate, W2S `GATHER/SETTLE`, push-exit, and restartable phase clock
are checked against ONNX metadata before the control loop starts.

## Evidence gates

Run contract/unit tests first:

```bash
ctest --test-dir build --output-on-failure -R 'gait_|single_policy_.*gait'
```

Then run closed-loop MuJoCo cases for forward, lateral, yaw stop, disturbance,
and reset/restart. Record mode, substate, phase, stance errors, filtered gyro/
joint RMS, and fall outcome. Passing an ONNX contract or unit test alone is not
behavioral or hardware acceptance.
