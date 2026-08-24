# R1-A5 Quest 3 hardware teleop bundle

This manifest defines the reproducible file set for the **suspended, fixed
arms/head pilot**. It does not authorize on-floor use, locomotion, waist
control or deployment without the hardware gate and an E-stop operator. A new
laptop or AI should begin with the bundle-root `START_HERE.md`; the package
also carries the repository context, safety/operations documents and compact
experiment definitions needed to interpret this file set.

## Why the bundle combines two revisions

The current `teleop` branch contains the Quest/mapping/IK pipeline, but its
working tree does not contain the sole-owner C++ high-level source. That source
exists at `hardware/high_level/` in commit `bb70a20` (`develop`). The exporter
combines the current workstation files with that pinned high-level tree and
records both provenances. Do not deploy `Operation_Khanh/high_level/` as a
substitute: its current `RobotSpec.hpp` still labels head slots 29/30 in the
opposite order and it has no UTL1 receiver.

## File set

Workstation path:

- `scripts/teleop/quest_bridge.py`: Quest/Vuer telemetry to
  `R1TeleopCommand` JSONL at 30 Hz.
- `scripts/teleop/run_r1_quest3_hardware_targets.py`: emits the 12 joint
  targets the robot receiver consumes. By default it solves nothing: joints
  arrive already solved by the vendor `xr_teleoperate` IK and this process only
  enforces the hardware envelope (asset joint limits, velocity/acceleration
  ceilings). `--coupled-ik` selects the previous in-repo solver instead.
- `scripts/teleop/run_r1_upstream_ik_stream.py`: the vendor solver stage, run
  unmodified in the `tv` environment. Present on the hardware path for the same
  reason as in simulation -- CasADi and the Pinocchio 3 bindings live only
  there, and the robot side needs no kinematic model at all.
- `scripts/teleop/run_r1_quest3_hardware.sh`: foreground launch, checks and
  evidence directory. `HB_TELEOP_SOLVER=upstream` (default) inserts the vendor
  solver between bridge and targets; `coupled` reproduces the older pipeline.
- `teleop/r1/`: authoritative schema, mapping, kinematics, IK, rate limiting and
  watchdog logic.
- `assets/R1.urdf`: geometry, joint axes, limits and signs used by the IK.
- `experiments/r1_teleop/quest3_sim_v1/T001/config/r1_quest3_sim_v1.json`:
  source/robot frames, calibration and command timeout.
- `experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_whole_upper_body_live.json`:
  simulation IK method and numerical parameters.
- `third_party/xr_teleoperate/teleop/televuer/src/`: pinned Vuer transport
  wrapper; it remains vendor code and must not be edited.
- `AGENTS.md`, root `README.md`, `docs/`, `evidence/`, the experiment registry,
  definitions T001–T008 and compact T001–T006 evidence: context and traceability
  for a new machine. Bulk T007 outputs are deliberately excluded; one small
  contract-complete T007 run is retained so registry checks remain executable.
- `third_party/xr_teleoperate_v1_6/`: only the small README/changelog, R1-A5
  URDF and upstream arm-control reference used to audit conventions.

Robot path:

- `hardware/teleop/src/teleop/hardware/high_level_sidecar.py`: read-only
  `rt/lowstate` subscriber and loopback UTL1 sender; it creates no DDS command
  publisher.
- `hardware/high_level/src/input/TeleopReceiver.hpp`: loopback-only UTL1
  receiver, sequence validation and 300 ms freshness watchdog.
- `hardware/high_level/src/robot/LowCmdSender.hpp`: sole `rt/lowcmd` writer;
  legs/waist stay zero torque while selected arms/head receive PD targets.
- `hardware/high_level/src/app/Application.*`: ZERO TORQUE arbitration, R3
  priority, ownership and state transitions.
- `hardware/high_level/src/config/RobotSpec.hpp`: authoritative R1-A5 IDL slots;
  head pitch is 29 and head yaw is 30.
- `hardware/teleop/config/high_level_teleop_suspended.yaml`: minimal runtime
  profile copied to `hardware/high_level/config/tuning.yaml` by the exporter.
- `hardware/high_level/policies/flat/policy_11_07.onnx` from the pinned
  high-level revision: verified as input `obs[1,83]`, output `actions[1,24]`;
  included because high-level loads a locomotion controller before ZERO TORQUE.

## Shared simulation definitions versus hardware safeguards

| Quantity | Value/source | Role |
| --- | --- | --- |
| Units | m, rad, s | Same end-to-end |
| Controlled order | left arm 5, right arm 5, head pitch, head yaw | Same model and stream order |
| IDL slots | arms 15–19 and 22–26; head pitch 29, yaw 30 | R1-A5 hardware mapping |
| URDF | `assets/R1.urdf` | Same geometry, axes and joint limits as sim pilot |
| Calibration | translation `[0,0,0]`, yaw `0` | Provisional identity; must be measured before general use |
| IK tolerance | position 0.002 m; head 0.03 rad | Same T007 numerical profile |
| IK solver | 40 iterations, damping 0.02, max step 0.12 rad | Same T007 numerical profile |
| Quest rate | 30 Hz | Source transport |
| IK dispatch rate | 10 Hz in the current launcher | Measured pipeline bottleneck; not a motor loop |
| High-level loop | 500 Hz | Sole DDS command loop |
| Sidecar send rate | 100 Hz | Local target transport |
| Command timeout | mapping 0.5 s; sidecar 0.75 s; high-level 0.30 s | Cascaded fail-closed watchdogs |
| Session envelope | ±0.15 rad from encoder anchor | Hardware-only safeguard |
| Final slew limit | 0.30 rad/s | Hardware-only safeguard |
| Arm PD | Kp 40, Kd 2 | Provisional suspended-pilot setting |
| Head PD | Kp 15, Kd 1 | Provisional suspended-pilot setting |

The simulator's `1.5 rad/s` velocity and `4.0 rad/s^2` acceleration are tuning
values, not approved actuator limits. The workstation hardware adapter caps
them before output, and the sole owner applies the final `0.30 rad/s` slew
limit. “Same as sim” therefore means the same frames, URDF, joint order,
mapping and IK—not blindly copying simulation actuator dynamics.

The current T007 profile permits projected solutions for unreachable targets.
That behavior is still an open hardware-gate item. The ±0.15 rad session
envelope bounds the suspended pilot, but it is not collision or torque proof.

## Export

From the repository root:

```bash
scripts/teleop/export_r1_quest3_hardware_bundle.sh
```

The command creates an ignored archive under `results/smoke/`, records source
revisions and SHA-256 hashes, and never connects to the robot. Certificates,
private keys, real environment files and runtime logs are excluded.

The bundle also contains
`hardware/teleop/tests/test_high_level_utl1_receiver.cpp`, a standalone
loopback smoke test for the exact 60-byte packet, receiver freshness and STOP
behavior. The older `hardware/high_level/tests/test_teleop_input.cpp` from
`develop` is intentionally excluded because it references a non-existent
`TeleopInput.hpp` and is not evidence for this receiver.

Before any robot command, follow
`docs/operations/r1_quest3_teleop_hardware.md`, keep the robot suspended and
fixed, keep an operator on the R3 E-stop, and close every item in
`hardware/teleop/docs/hardware_gate.md` relevant to the pilot.
