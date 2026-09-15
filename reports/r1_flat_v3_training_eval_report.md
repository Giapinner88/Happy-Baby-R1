# R1 Flat V3 Training And Evaluation Report

Date: 2026-07-14

## Summary

Flat v3 finished training successfully and exported:

- Run directory: `data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-10_18-04-54_r1_flat_walk_v3`
- Final checkpoint: `model_9999.pt`
- Exported policy: `policy.onnx`
- Task: `Unitree-R1-FlatV3`
- Envs: `4096`
- Iterations: `10000`
- Terrain: plane only

The policy is stable in MuJoCo direct-eval and does not fall in the tested cases.
Compared with flat v2, v3 improves several smoothness and body-stability metrics, but it does not improve forward speed tracking at `cmd_vx=0.3`.
The main regression/remaining issue is lateral drift and weak yaw behavior.

Do not promote v3 to `data/models/unitree_mujoco_policy/r1_velocity.onnx` yet.
The current default symlink was intentionally left unchanged while v3 is being evaluated.

## Training Setup

The captured config in `params/agent.yaml` shows:

- PPO runner: `OnPolicyRunner`
- Actor/critic hidden dims: `512, 256, 128`
- Observation normalization: enabled
- Entropy coefficient: `0.003`
- Save interval: `100`
- Logger: TensorBoard

The captured env config in `params/env.yaml` shows:

- Terrain type: `plane`
- Terrain generator: `null`
- Command range:
  - `lin_vel_x`: `0.0` to `0.85`
  - `lin_vel_y`: `-0.25` to `0.25`
  - `ang_vel_z`: `-0.55` to `0.55`
- Curriculum stages:
  - Step `0`: `lin_vel_x 0.0..0.35`, `lin_vel_y -0.08..0.08`, `ang_vel_z -0.2..0.2`
  - Step `90000`: `lin_vel_x 0.05..0.55`, `lin_vel_y -0.14..0.14`, `ang_vel_z -0.35..0.35`
  - Step `220000`: `lin_vel_x 0.05..0.85`, `lin_vel_y -0.25..0.25`, `ang_vel_z -0.55..0.55`

V3 intentionally increased emphasis on smoother and more stable whole-body behavior:

- `body_orientation_l2`
- `body_ang_vel`
- `angular_momentum`
- `joint_acc_l2`
- `action_rate_l2`
- `foot_slip`

## Training Curves

Read from TensorBoard event:

| Metric | First | Last | Best/Min |
| --- | ---: | ---: | ---: |
| `Train/mean_reward` | `-3.1088` | `42.0936` | max `47.1960` at step `9107` |
| `Train/mean_episode_length` | `13.75` | `1000.0` | max `1000.0` from step `1372` onward |
| `Episode_Reward/track_linear_velocity` | `0.0027` | `1.2615` | max `1.3526` |
| `Episode_Reward/track_angular_velocity` | `0.0004` | `0.2451` | max `0.2731` |
| `Episode_Metrics/mean_action_acc` | `1.7759` | `0.2694` | min `0.2492` |
| `Metrics/slip_velocity_mean` | `0.3545` | `0.0567` | min `0.0364` |
| `Metrics/angular_momentum_mean` | `1.4138` | `0.6495` | min `0.5161` |
| `Metrics/landing_force_mean` | `258.155` | `163.011` | min `95.365` |
| `Policy/mean_std` | `0.9943` | `0.1375` | min `0.1286` |

Interpretation:

- Training converged cleanly: episode length reached the timeout limit and stayed there.
- Exploration collapsed to a low final standard deviation, which is expected after convergence but can reduce recovery diversity.
- Slip velocity, angular momentum, action acceleration, and landing force all improved strongly during training.
- Training curves support the intended v3 direction: smoother and less violent locomotion.

## Evaluation Setup

All tests used the v3 ONNX directly:

`data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-10_18-04-54_r1_flat_walk_v3/policy.onnx`

Direct-eval command pattern:

```bash
PYTHONNOUSERSITE=1 conda run --no-capture-output -n r1_env \
  python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0 \
  --policy-onnx data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-10_18-04-54_r1_flat_walk_v3/policy.onnx \
  --conda-env r1_env \
  --safety-preset conservative
```

Bridge command pattern:

```bash
PYTHONNOUSERSITE=1 conda run --no-capture-output -n r1_env \
  python scripts/run_unitree_mujoco_policy.py \
  --scene scene.xml \
  --duration 16 \
  --startup-wait 3.0 \
  --cmd-vx 0.3 \
  --cmd-vy 0.0 \
  --cmd-yaw 0.0 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0 \
  --policy-onnx data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-10_18-04-54_r1_flat_walk_v3/policy.onnx \
  --conda-env r1_env \
  --safety-preset conservative \
  --bridge-actuator-mode position
```

Notes:

- Direct-eval uses MJLab-style position actuators and bypasses DDS.
- Bridge eval uses `policy_runner -> DDS -> unitree_mujoco2` with `--bridge-actuator-mode position`.
- Video was not recorded for these metrics; video rendering changes wall-time and is better treated as visualization only.
- Direct-eval metrics use the stable window after ramp, `t >= 2.5s`.
- Bridge metrics use `t >= 5.0s` to skip policy/sim startup.

## Evaluation Results

| Case | CSV | Command | Avg vx from position | vx error | Avg vy drift | Base z mean/std/min | Tilt mean/p95/max | Gyro norm mean/p95/max | Leg dq rms mean/p95 | Action delta rms mean/p95 | Fall |
| --- | --- | --- | ---: | ---: | ---: | --- | --- | --- | --- | --- | --- |
| v2 direct baseline | `data/sim_state_logs/unitree_mujoco2_17-10-23_2026-07-10.csv` | `vx=0.3 yaw=0.0` | `0.270` | `0.030` | `0.013` | `0.733 / 0.0011 / 0.730` | `0.99 / 1.36 / 1.87 deg` | `0.173 / 0.419 / 0.517` | `0.629 / 0.992` | `0.10324 / 0.29573` | no |
| v3 direct | `data/sim_state_logs/unitree_mujoco2_08-48-09_2026-07-14.csv` | `vx=0.3 yaw=0.0` | `0.238` | `0.062` | `-0.118` | `0.735 / 0.0009 / 0.732` | `0.62 / 1.02 / 1.47 deg` | `0.200 / 0.379 / 0.545` | `0.522 / 0.674` | `0.08042 / 0.25348` | no |
| v3 direct faster | `data/sim_state_logs/unitree_mujoco2_08-48-34_2026-07-14.csv` | `vx=0.5 yaw=0.0` | `0.398` | `0.102` | `-0.091` | `0.734 / 0.0021 / 0.729` | `1.31 / 2.43 / 2.70 deg` | `0.384 / 0.626 / 0.731` | `0.815 / 1.190` | `0.11714 / 0.27786` | no |
| v3 direct turn | `data/sim_state_logs/unitree_mujoco2_08-48-57_2026-07-14.csv` | `vx=0.3 yaw=0.3` | `0.084` | `0.216` | `0.214` | `0.734 / 0.0011 / 0.732` | `0.76 / 1.51 / 1.70 deg` | `0.251 / 0.444 / 0.539` | `0.527 / 0.717` | `0.08138 / 0.26007` | no |
| v2 bridge baseline | `data/sim_state_logs/policy_runner_17-25-40_2026-07-10.csv` | `vx=0.3 yaw=0.0` | `0.237` | `0.063` | `0.010` | `0.731 / 0.0012 / 0.728` | `1.13 / 1.53 / 1.72 deg` | `0.294 / 0.687 / 0.832` | `0.643 / 1.056` | `0.10557 / 0.22040` | no |
| v3 bridge | `data/sim_state_logs/policy_runner_08-51-16_2026-07-14.csv` | `vx=0.3 yaw=0.0` | `0.212` | `0.088` | `-0.118` | `0.733 / 0.0010 / 0.731` | `0.74 / 1.32 / 1.87 deg` | `0.236 / 0.508 / 0.636` | `0.555 / 0.796` | `0.08185 / 0.16726` | no |

## Findings

### 1. Stability Improved

V3 does not fall in any tested case.
`base_z` stays tightly bounded around `0.733-0.735m`.
The projected gravity/tilt metrics are also good.

At `cmd_vx=0.3`, v3 direct-eval has lower tilt than v2:

- v2 direct tilt mean/p95: `0.99 / 1.36 deg`
- v3 direct tilt mean/p95: `0.62 / 1.02 deg`

This matches the v3 goal of stabilizing the whole body.

### 2. Motion Is Smoother

At `cmd_vx=0.3`, v3 reduces leg velocity and action-change metrics:

- Direct leg dq rms mean: v2 `0.629` -> v3 `0.522`
- Direct action delta rms mean: v2 `0.10324` -> v3 `0.08042`
- Bridge leg dq rms mean: v2 `0.643` -> v3 `0.555`
- Bridge action delta rms mean: v2 `0.10557` -> v3 `0.08185`

This is the strongest positive result of v3.
The robot should look less jerky than v2 when walking straight.

### 3. Forward Speed Tracking Got Worse At `vx=0.3`

Despite better smoothness, v3 under-tracks forward velocity:

- v2 direct `vx=0.3`: actual `0.270`
- v3 direct `vx=0.3`: actual `0.238`
- v2 bridge `vx=0.3`: actual `0.237`
- v3 bridge `vx=0.3`: actual `0.212`

At `cmd_vx=0.5`, v3 reaches `0.398`, so it can move faster, but it still under-tracks by about `0.10 m/s`.

This is a tradeoff regression: v3 became smoother, but less assertive.

### 4. Lateral Drift Is The Main Problem

The biggest issue is not falling, but direction control.

For a straight command `vx=0.3, yaw=0.0`, v3 drifts sideways:

- v3 direct avg `vy=-0.118 m/s`
- v3 bridge avg `vy=-0.118 m/s`

This drift is absent in the v2 bridge baseline:

- v2 bridge avg `vy=0.010 m/s`

Because direct-eval and bridge both show the same v3 drift, this is likely a learned policy behavior or observation/command semantics issue, not only a bridge bug.

### 5. Turning Behavior Is Still Weak

For `vx=0.3, yaw=0.3`, v3 remains upright but converts much of the motion into sideways movement:

- actual forward `vx=0.084`
- actual lateral `vy=0.214`

That matches the subjective issue: turning is not clear and the robot feels heavy/uncertain when direction changes.

### 6. Bridge Parity Is Better, But V3 Policy Quality Is Not Ready

The v3 bridge run is broadly consistent with v3 direct-eval:

- Both are stable.
- Both show lower smoothness/rattle metrics than v2.
- Both show similar lateral drift.

This is good news for bridge parity: the bridge is no longer the obvious primary blocker for this specific v3 failure.
The bad news is that v3 itself needs another iteration before becoming the default walking policy.

## Likely Causes

### Smoothness Penalties May Be Too Strong Relative To Velocity Tracking

V3 improved smoothness metrics, but it lost forward tracking.
The policy appears to prefer a conservative, less aggressive gait.

Likely knobs:

- Increase `track_linear_velocity` weight or reduce its `std` slightly.
- Reduce `action_rate_l2`, `joint_acc_l2`, or `angular_momentum` penalties a little.
- Keep body orientation penalty, because it appears beneficial.

### Yaw/Heading Semantics Need Audit

The env config has:

- `heading_command: true`
- `rel_heading_envs: 1.0`
- `ang_vel_z` range still present

Runtime sends a simple command vector: `cmd_vx`, `cmd_vy`, `cmd_yaw`.
The turning test suggests the yaw/heading meaning may not fully match between training and runtime, or the policy is exploiting lateral motion to satisfy the heading task.

This should be audited before v4-style terrain work.

### No Flat-Terrain Direction Constraint

The train reward tracks linear velocity, but current eval shows straight commands can drift laterally.
For the next train, we should explicitly gate straight-walk quality:

- Penalize lateral velocity when `cmd_vy ~= 0`.
- Penalize yaw drift when `cmd_yaw ~= 0`.
- Add an eval metric and model-selection rule for `abs(avg_vy)` and heading drift.

## Terrain Insight

V3 was trained on a plane only.
It is not yet a rough-terrain policy.

Adding rough terrain now is possible, but it would hide the more basic issue: straight-line and yaw-command semantics are not clean yet.
The better order is:

1. Fix flat walking direction control.
2. Re-train/evaluate flat v3.1 or v4 until straight and turning gates pass.
3. Then introduce mild terrain randomization.

Suggested first rough-terrain curriculum after flat control is fixed:

- Stage 1: flat plane with random friction/mass/COM only.
- Stage 2: low-amplitude heightfield, about `1-2cm`.
- Stage 3: mixed plane plus shallow bumps, about `2-4cm`.
- Stage 4: slope/stairs only after the robot can maintain heading and yaw-rate on flat.

## Recommended Gates For The Next Policy

A policy should not become `r1_velocity.onnx` unless it passes:

| Gate | Command | Requirement |
| --- | --- | --- |
| Straight slow | `vx=0.3 yaw=0.0` | `avg_vx >= 0.26`, `abs(avg_vy) <= 0.03`, no fall |
| Straight medium | `vx=0.5 yaw=0.0` | `avg_vx >= 0.43`, `abs(avg_vy) <= 0.05`, no fall |
| Turn left/right | `vx=0.3 yaw=+/-0.3` | clear yaw response without forward collapse |
| Smoothness | `vx=0.3` | v3-level or better action/leg dq metrics |
| Bridge parity | same as direct | bridge `avg_vx` within about `0.05 m/s` of direct-eval |

## Decision

Flat v3 is a successful smoothness experiment, but not a successful default locomotion policy.

Keep v2 retrain as the safer default runtime policy for now.
Use v3 as evidence for the next train profile:

- preserve the body stability gains,
- recover forward speed tracking,
- add explicit lateral/yaw direction quality,
- audit heading/yaw command semantics,
- delay rough terrain until flat direction control passes.

