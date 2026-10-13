# HB safety changes — 2026-09-08

## Disarm before stop/restart/auto-calib

`IDLE` and `ZERO TORQUE` still publish motor packets; neither means DISARMED.
The normal release path is now:

1. Physically support/suspend the robot before entering ZERO TORQUE (Mode Z).
2. Release Quest deadman and all R3 buttons. Wait for teleop to retract fully.
3. With robot stationary, hold **exact R1+R2** for `arm_hold_s` (currently 1 s).
4. Wait for `DISARMED` / `high_armed=0`, then use `stop-high` before auto-calib.

Only passive Mode Z permits release. A fresh queued UDP command, active/retracting
teleop, invalid/stale LowState, raw gyro >0.15 rad/s or raw joint speed >0.10 rad/s
(including head motors) blocks it. Invalid head feedback also blocks handover.
Software cannot establish that a stand/person is supporting the robot's weight.
Disarm itself and all subsequent disarmed ticks publish no motor commands.
To arm again: release all buttons, perform a new hold, and satisfy the original
built-in observation/silence gate. Old arm intent and built-in counters are cleared.
This route requires `arm_gate_enabled: true`.

E-stop remains damping/armed. The explicitly retained simultaneous L2+B+Y behavior
ends in ZERO TORQUE; a regression test records this decision. Use E-stop alone for
damping; entering limp still requires physical support.

## DDS and button timing

`state_timeout_ms` controls PD freshness. `remote_state_timeout_ms` independently
expires cached wireless-remote data. Both remain **1000 ms** in active config;
the PD value is not an accepted hardware deadline. Measure packet age/jitter on PC2
under deployment load before reducing it. Remote expiry must be >= PD deadline.

Each LowState callback increments a local sequence; replay of the same sample
does not advance hold timers, generate edges or refresh remote recovery. Observed
sample intervals <=20 ms contribute to holds; longer gaps contribute zero and
preserve prior progress until remote expiry. This 20 ms input timing tolerance is
separate from the PD deadline. Double-click uses real elapsed time so a long gap
cannot extend its acceptance window. Truly expired/invalid remote resets holds.

Arm/disarm require a new valid sample when completing the operation. PD timeout
latches all armed states, including Mode Z, into IDLE. Reconnection resynchronizes
the slew origin to encoders and does not restart a policy or teleop automatically.
The callback counter detects local arrival, not upstream sensor freshness/CRC.

## Motion and tensor contracts

Both motion loaders now retain/check NPY dtype kind and accept C-order float32 or
float64 with explicit conversion. Integer arrays of the same byte width, NaN/Inf,
float32 overflow, empty arrays, invalid FPS and zero/overflow quaternion norms
are rejected. FPS defaults to 50 only when absent; float32 and float64 FPS work.
Failed loads leave an empty, non-playable object, never a partially loaded clip.
Torso quaternions are normalized only after validation.

ONNX inputs/outputs must be float32 rank 2, batch 1 (symbolic batch allowed at
load), with exact feature dimensions. Output type/shape is checked again after
inference before reading the 24 actions. The optional generated-file host test
uses `-DHB_TEST_PYTHON=/path/to/python-with-onnx-and-numpy`.

## Head targets

Recording remains `[pitch(29), yaw(30)]`; Application converts once to
`HeadTarget{yaw,pitch}`. No capture/model file was rewritten.

The common sender now bounds head targets asymmetrically, without joint slack:

| Axis | Minimum | Maximum |
|---|---:|---:|
| Pitch | -0.14 rad | +0.64 rad |
| Yaw | -0.65 rad | +0.44 rad |

These provisional bounds are rounded inward from 66,774 recorded encoder frames.
They are not certified mechanical/thermal/load limits. Commands approach them
through the existing slew limiter; an encoder origin outside the envelope is not
snapped into range. All head producers, including Mode Z, use the common sender.
Gesture/teleop producer magnitude caps are also reduced to pitch 0.64, yaw 0.65.

## Local verification

Fresh host builds on 2026-09-08 passed **27/27 CTest** each:

- Debug: `/tmp/hb-safety-fix-build`.
- Release: `/tmp/hb-safety-fix-release.aSTcVg`.
- ASan + UBSan: `/tmp/hb-safety-fix-san.54o5R6`, with
  `ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1`. Leak checking was
  disabled; this is not a leak-free certification of bundled runtime libraries.

The suite includes Application packet-sink transitions, cached-input holds,
motion dtype/finite/quaternion failures, head output limits, and nine generated
ONNX files with valid or incompatible tensor contracts. Existing golden traces
and policy-profile preflight tests also pass. This is not a malformed ZIP/NPY
parser fuzz audit; archive structural hardening remains outside this patch.

Release `run_r1 --preflight`, read-only model-manifest verification and staged
integration `preflight.sh high` passed without initializing DDS/motor output.
This 2026-09-08 snapshot recorded the then-active baseline `legacy_83`,
`policies/flat/policy_goc_2.onnx`, SHA-256
`ee1cd73469ba0f16eff7542fc4758396a7301398ebdfb2e0926713b3c1954500`.
The current 2026-09-10 checkout instead selects the staged
`flat_plus_gait_h4_v1` v2 walk-quality policy; this does not change the
DISARMED/E-stop requirements below.
Existing preflight warnings remain: skipped dance slot 5 and 0.048 rad default-pose
differences for loaded dances. No deploy, service restart, ARM64 build or physical
robot acceptance was performed.

## Acceptance still open

Local CTest and packet-sink tests cover software contracts and transitions, not
physical balance or motor firmware behavior. PC2 timing, supported robot handover,
head range/load and hardware E-stop acceptance remain required. Remote-loss behavior
in Mimic/get-up/lie-down and warning-only stale BMS are unchanged by this patch.
The existing dance slot 5 filename mismatch is also unchanged; do not silently
enable a previously skipped dance as part of a safety patch.
