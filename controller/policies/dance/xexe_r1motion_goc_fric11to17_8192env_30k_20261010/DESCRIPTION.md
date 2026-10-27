# Xexe GOC R1MotionLab — [1.1, 1.7]

Temporary test snapshot; training continues; checkpoint model_18500.pt (18501 updates, zero-based index).

Original training run: /home/ubuntu22/train_mujoco/robot_r1/unitree_rl_mjlab_goc/logs/rsl_rl/r1_tracking/2026-10-09_16-07-16_xexe_r1motion_goc_fric11to17_129obs_fresh_30k_8192_gpu0_20261009_160708
Immutable same-run export snapshot: /home/ubuntu22/train_mujoco/outputs/artifacts/hb_r1_dance_packaging/xexe_goc_8192_30k_20261010/update_20261010_101634/xexe_r1motion_goc_fric11to17_8192env_30k_20261010/checkpoint_18500/2026-10-09_16-07-16_xexe_r1motion_goc_fric11to17_129obs_fresh_30k_8192_gpu0_20261009_160708

Fresh training, 8192 envs, target 30000 iterations, seed 42, actor 129 obs / 24 actions, ankle tracking. Foot friction [1.1, 1.7]. NPZ after R1MotionLab, 5007 frames / 50 Hz, timing preserved. Local dynamics solution only covers 240/5007 reference frames; other poses retained.

Audio preserves the original Xexe motion/audio pairing. Package stored without selecting a dance slot. Exact deployment metadata is copied from this run, not from another policy.
