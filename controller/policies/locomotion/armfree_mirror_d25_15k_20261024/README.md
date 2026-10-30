# ArmFree Mirror — damping knee/ankle2.5, 15000 iterations

Packaged candidate only. NOT selected for either walk R2+A or teleop R2+X. Active locomotion config, speed limits and integration model manifest remain unchanged. Switching either route requires an explicit user instruction. Existing bundles remain available. Simulation evidence and local contract validation only; no hardware acceptance is implied.

Source: unitree_rl_mjlab_gait_pro/logs/armfree_damping25_20261023/mirror/model_14999.pt, seed42,4096env,15000iterations; mirror loss .02. Source/checkpoint/packaged ONNX hashes and exact arrays are in packaging.json. Packaging changes metadata only; identical actor graph and inference output verified.

Contract: 335D flat_plus_gait_h4_v1 →24D raw position actions; H4 term-major oldest→newest, repeat_first, pre-inference append; previous raw policy actions retained during arm overlays. Current gait one-hot STAND/WALK/W2S is unnormalized. Embedded observation normalizer handles the332D prefix.

FSM: flat_plus_gait_fsm_legs12_stand_recovery_v1; filter signed raw gyro/joint velocity, tau.2s; RMS exactly first12leg joints; settle.5s, stand recovery tilt/gyro exit preserved. HB adds explicit metadata-gated support; old contracts still use24joint RMS. Runtime must include this update, so policy-only deployment with an old binary is insufficient.

Native PD: hip Kp100/Kd2; knee100/2.5; ankle40/2.5; waist100/2; shoulder pitch/roll40/2; shoulder yaw/elbow/wrist20/1. Exact action scales knee.1575, ankle.328125, distal arm.4125, hip/waist.15, shoulder pitch/roll.375. These belong to the selected completed policy, NOT nominal scales of the newer M1/M2/M3 training. Default pose and ordering come from the same run; no dance metadata used.

Original snapshot metadata had damping2.0 for knee/ankle and rounded scale3decimals; packaging reconstructs exact values from the verified damping2.5 run's resolved config. deployment_note records the runtime contract update; deployable=false remains because hardware validation is pending.

Final checkpoint retest: 95scenarios ×8trials=760robustness trials and64gait probe trials per policy,2evaluation seeds; compared against all3other final14999actors. This policy: XY error.0693m/s, push survival463/464, walk+strong push falls1/32; nominal gesture drift max5.21cm. Gesture completed159/160; one HB guard retract under gesture+push, robot survived. High-speed gait still imperfect: left/right step difference at1.5m/s5.24cm, greater than NoMirror2.0's3.78cm. Recovery metric is first.5s valid dwell within6s, not permanent recovery or return to original position.

Evaluation summary copied beside this README; raw trajectories and rerun entrypoints remain in the training repository at artifacts/final_four_native_20261024 and scripts/evaluate_armfree_final_four.py. Reproduce packaging there with scripts/package_armfree_locomotion_hb.py. Runtime code/config changes require a full controller build/deploy, not just a policy copy. No robot deployment was performed while preparing this bundle.
