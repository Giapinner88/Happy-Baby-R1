# R1 hardware teleop arbitration

Status: accepted method for suspended arms/head pilot only  
Decision: `decisions/r1_teleop/D003_single_lowcmd_owner.md`

## Purpose and boundary

This method lets Quest command the R1-A5 arms and head without creating a
second DDS motor writer. `hb_high_level` remains responsible for all 35
`LowCmd` slots, gamepad/E-stop handling, mode transitions and Zero Torque. The
sidecar owns only validated local target transport and evidence capture.

```text
Quest/Vuer -> IK on workstation -> JSONL/SSH -> loopback UDP sidecar
                                             -> high-level ZERO TORQUE arbitration
                                             -> sole rt/lowcmd publisher
```

It excludes legs, waist, balance, locomotion and on-ground operation.

## Interface and conventions

The loopback UDP schema is a packed 60-byte little-endian `UTL1` packet:

```text
uint32 magic=0x314c5455, uint32 sequence
uint8 enable, uint8 arm_valid, uint8 head_valid, uint8 pad
float arm_q[10], float head_yaw, float head_pitch
```

The vector order and R1-A5 IDL motor slots are:

| Offset | Joint group | IDL slots |
| --- | --- | --- |
| 0–4 | left shoulder pitch/roll/yaw, elbow, wrist roll | 15–19 |
| 5–9 | right shoulder pitch/roll/yaw, elbow, wrist roll | 22–26 |
| 10 | head pitch | 29 |
| 11 | head yaw | 30 |

Angles are radians. High-level binds only `127.0.0.1:5560`; the sidecar creates
no DDS publisher and packets cannot arrive directly from the LAN.

## Accepted control method

The sidecar saves the first Quest vector as source zero and current encoder
values as the robot anchor. For source vector `s`, source zero `s0` and encoder
anchor `q0`, each desired joint is

```text
q_des[i] = q0[i] + clamp(s[i] - s0[i], -0.15, +0.15) rad.
```

The sidecar and sole sender both bound command motion. High-level emits one
complete command: selected arm/head slots get PD while every leg/waist and
unselected slot remains zero torque. On release arms ramp out over 0.5 s and
the whole command returns to zero torque.

R3 `L2+B` causes immediate Damping/IDLE. A lost R3 link, stale UDP, STOP,
stream closure, mode change or sidecar failure removes arm/head authority.
Packets are acted on only in `ZERO TORQUE`; other states ignore them. R3
E-stop always has priority.

## Assumptions and validity

- R1-A5, `mode_machine=1`, robot suspended and fixed to its test fixture.
- One safety operator holds the R3 E-stop combination and another operates
  Quest.
- The initial robot pose is already mechanically valid; the ±0.15 rad envelope
  is relative to that pose.
- Quest IK remains responsible for URDF joint-limit compliance. The hardware
  layer adds a smaller relative envelope but no collision model.

This method does not establish tracking accuracy, torque safety, collision
clearance, balance or permission to run on the floor.

## Implementation and checks

| Responsibility | Implementation |
| --- | --- |
| IPC parsing/freshness | `hardware/high_level/src/input/TeleopReceiver.hpp` |
| ZERO TORQUE arbitration/E-stop | `hardware/high_level/src/app/Application.cpp` |
| sole LowCmd construction | `hardware/high_level/src/robot/LowCmdSender.hpp` |
| read-only DDS sidecar | `hardware/teleop/src/teleop/hardware/high_level_sidecar.py` |
| operator entry point | `scripts/teleop/run_r1_quest3_hardware.sh` |
| tests | loopback C++ receiver smoke test, `hardware/teleop/tests/` |

Executed workstation checks include a standalone C++ protocol parser build,
the full relevant Python test set and a clean high-level CMake build. Hardware
tracking and E-stop priority still require the bounded suspended validation
described in the operating SOP.
