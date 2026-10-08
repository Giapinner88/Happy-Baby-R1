# ALMI R1 `loco-teleop` policy package

This directory contains the five native ALMI R1 policies exported from Isaac
Lab. Each stage has its own checkpoint, provenance, parameters, and ONNX actor.

The exported actor contract is:

- observation: 105 floats
- action: 24 floats
- policy contract: `r1_almi_v2`
- output file: `policy.onnx`

Stages:

| Directory | Task | Checkpoint |
|---|---|---|
| `locomotion/` | `ALMI-R1-Locomotion-v1` | `model_9999.pt` |
| `squat/` | `ALMI-R1-Squat-v1` | `model_14998.pt` |
| `teleop_gesture/` | `ALMI-R1-TeleopGesture-v1` | `model_24997.pt` |
| `walk_teleop/` | `ALMI-R1-WalkTeleop-v1` | `model_34996.pt` |
| `unified/` | `ALMI-R1-Unified-v1` | `model_41000.pt` |

These artifacts are packaged for ALMI integration and are not directly
selectable by the current `controller` runtime. That runtime currently
expects the separate 335-D `flat_plus` locomotion contract. A 105-D ALMI
adapter and hardware validation are required before deployment to the robot.

Integrity information is recorded in `MANIFEST.yaml` and `SHA256SUMS`.
