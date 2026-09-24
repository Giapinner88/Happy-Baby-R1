# HB locomotion A/B simulation — 2026-09-23

## Scope

Compared the HB checkout's active locomotion artifact (0917) with the requested 0917V5A1 artifact in the headless C++ MuJoCo stack on the 0-degree scene. Both runs used gait period 0.6 s and settle dwell 0.5 s. The simulator override is isolated in this results directory; the active HB config/locomotion.yaml still selects policy.onnx.

- Active artifact: HB/high_level_2/policies/locomotion/flat_plus/policy.onnx, SHA-256 df5807e3af9d4e5b7e68c84a2494c4d7ee888558332007d8e24dbdb998711de7, runtime metadata variant 0917.
  This active filename/hash is not listed in the existing flat_plus/SHA256SUMS manifest.
- Requested candidate: HB/high_level_2/policies/locomotion/flat_plus/policy_flat_plus_gait_0917_v5_a1.onnx, staged byte-for-byte at policy/locomotion/candidates/hb_flat_plus_gait_0917_v5_a1_20260923/policy.onnx, SHA-256 dac953274e45d25ce057b8a2f76b0a7940c39c81d301ce7586ac7cc5e009d3f0, runtime metadata variant 0917V5A1.
- Both loaded as 335-D observation to 24-D action. The candidate ran with its matching 0.5 s settle dwell; results/hb_v5_a1_20260923/tuning.yaml differs from simulator default only in that dwell value.

## Closed-loop results

| Run | Max tilt | Base height drop | Final displacement (x, y) | Final yaw error |
|---|---:|---:|---:|---:|
| Active 0917, flat_demo | 3.08° | 1.8 mm | (+0.382, −0.408) m | −0.017 rad |
| V5A1, flat_demo | 3.75° | 7.1 mm | (−0.162, −0.047) m | −1.058 rad |
| Active 0917, vx=0.6 m/s for 12 s | 3.41° | 4.3 mm | (+6.460, −1.765) m | −0.343 rad |
| V5A1, vx=0.6 m/s for 12 s | 4.06° | 6.4 mm | (+5.566, −3.452) m | −0.981 rad |

In flat_demo, the active policy turned about +1.44 rad on the +0.4 rad/s phase and returned to about 0 rad after the −0.4 rad/s phase. V5A1 turned about +0.97 rad, then ended at −1.06 rad after the reverse turn and stop. Thus V5A1 has materially worse yaw/heading behavior here.

All four autotests report RESULT=PASS. That result means the run stayed within the configured fall, tilt, and base-drop gates; it does not mean command tracking or gait quality passed.

## Foot-impact measurements

For flat forward, the physics window is 4–8 s, corresponding to the flat_demo command window at behavior time 2–6 s after its 2 s warmup. For vx=0.6, the physics window is 5–17 s, the full 12 s commanded walk after its 5 s warmup. Ground reaction values below are the 95th percentile over physics samples where that foot is in contact. Touchdown speed is the median absolute vertical foot speed on raw contact rising edges.

| Command window | Policy | Contact force p95 L/R (N) | Touchdown speed p50 absolute L/R (m/s) | Swing clearance median L/R (cm) | Knee dq p95 L/R (rad/s) |
|---|---|---:|---:|---:|---:|---:|
| Forward 0.35 m/s | Active 0917 | 287 / 290 | 0.295 / 0.259 | 2.14 / 1.45 | 3.26 / 3.59 |
| Forward 0.35 m/s | V5A1 | 462 / 480 | 0.419 / 0.545 | 5.22 / 5.42 | 8.26 / 9.65 |
| vx=0.6 m/s | Active 0917 | 308 / 307 | 0.261 / 0.297 | 3.02 / 2.09 | 3.98 / 4.26 |
| vx=0.6 m/s | V5A1 | 505 / 533 | 0.451 / 0.518 | 5.54 / 5.67 | 10.59 / 9.75 |

V5A1 raises contact-force p95 by roughly 60–74% in these windows, lands faster, lifts the feet higher, and moves the knees faster. The measurements support the report that this candidate walks more forcefully in simulation; it is not a safer replacement for reducing hard steps or shaking.

There is a separate diagnostic in the active 0917 run at vx=0.6: the right-foot raw contact recorder counted 49 rising edges in 12 s, versus 20 on the left. Twenty-nine of 50 right-foot contact episodes lasted 10 ms or less, and the median right step-length sample was 8 mm. V5A1 had 20/22 rising edges and no contact episodes at or below 10 ms. The recorder samples every 2 ms and does not apply contact hysteresis, so treat this as contact chatter in this MuJoCo model, not proof of the physical robot's contact behavior.

## Instrumentation limit

The autotest CSV's action_* and target_q_* columns are NaN for both policies. PolicyApplication.cpp only fills those trace fields for meta/ARMA controllers; this locomotion route records meta_selected=DISABLED. These columns therefore cannot be used to claim the actor emits NaN or to compare action amplitudes. Contact, foot kinematics, and knee-velocity measurements above come from the separate MuJoCo physics recorder.

## Runtime and contract changes for this evaluation

- Staged the exact A1 ONNX artifact in an isolated candidate directory with a SHA256SUMS file.
- Extended the existing 0917 C++ artifact contract to accept the specifically qualified V5A1/V5A2A metadata and added corresponding smoke-test validation. Legacy dispatch remains in the existing 0917 stand-recovery contract.
- Build completed; full CTest passed 88/88. The C++ runtime logs confirm the requested model path and variant for every A1 run.

## Assessment boundary

The active workspace config selects variant 0917; it does not establish which ONNX file was last deployed to the robot. No robot-side deployed hash or hardware telemetry was available in these results. If the robot is running V5A1, this simulation flags stronger touchdowns and poorer yaw behavior. If it is running active 0917, simulation shows a possible right-foot contact-chatter mode, but that alone does not establish the hardware cause. Do not deploy V5A1 as a fix for hard steps based on this comparison.

