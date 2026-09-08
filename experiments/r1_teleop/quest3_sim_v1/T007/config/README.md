# T007 configuration

`r1_t007_arm_head_live.json` is the editable schema-2 live profile. It owns the
vendor virtual-endpoint definition, absolute 1:1 mapping, q=0 startup/reset,
joint-limited projection policy, and simulation rate limits. Every new T007 run
must snapshot this file as `experiment_config.json`; schema-1 runs are legacy
and must not be reinterpreted with this endpoint model.

The active `1.5 rad/s`, `4.0 rad/s²` rate pair is a simulation-only revision
selected by replaying `t007_live_arm_head_20260817T114118Z`. The earlier
`1.0 rad/s`, `2.0 rad/s²` snapshot remains authoritative for that run. Dynamic
tracking metrics across the revision must name the rate pair and are not
identical-protocol replicates.
`r1_t007_whole_upper_body_live.json` is the schema-3 coupled pilot. It gives
`waist_yaw_joint`, both five-joint arms, and both head joints to one IK solve.
It is not directly comparable with schema-2 independent-arm runs and is not a
hardware configuration.

`r1_t007_upstream_stream_live.json` is the arms/head profile for the unmodified
vendor `R1_A5_ArmIK`. Its live producer captures the third consecutive
deadman-enabled head pose as a session position/yaw anchor. Vendor wrist poses
are inverted out of the moving current-head frame and expressed against that
fixed anchor before IK, while head pitch/yaw are relative to the same initial
pose. Runs made before this calibration change are not protocol-comparable.

`r1_t007_differential_live.json` is an opt-in schema-4 live profile; default
`make teleop` remains the schema-3 pose-sequence baseline. It replaces the per-command iterative
pose solve with one differential DLS step, uses strict wrist-position before
wrist-orientation priority, and projects wrists outside the URDF-derived
conservative reach sphere before the Jacobian is evaluated. Its current scale
0.75, 3 rad/s and 30 rad/s² bounds, wrist-position/head feedforward, and
box-constrained DLS were selected by exact-timing Isaac replay of the
2026-08-20 Quest trace. The replay passed the simulation gate, but a new live
Quest run is still required before any hardware-facing claim. Select it with
`--whole-upper-body-config experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_differential_live.json`.
The current Isaac Sim 5.1 environment has no JAX installation, so the effective
backend is the validated NumPy central-difference implementation and that fact
is preserved in every run snapshot.

`r1_t007_mujoco_trajectory_replay.json` is the editable schema-4 offline
validation definition. It replays an immutable T007 Quest trace through a
100 Hz velocity-feedforward Cartesian controller in the repository's MuJoCo
R1 model. Its relative calibration, 0.4 workspace scale, reconstructed MuJoCo
head articulation, and CPU backend make it deliberately non-comparable with
the earlier absolute 1:1 IsaacLab runs. Every result snapshots this profile and
the generated resolved MuJoCo XML without changing the canonical asset.
