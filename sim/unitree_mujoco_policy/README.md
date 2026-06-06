# Unitree MuJoCo Policy Runtime

This directory contains local Happy-Baby-R1 runtime scripts for running trained
policies against Unitree MuJoCo.

Keep `third_party/unitree_mujoco` as a clean upstream vendor checkout. Local
policy scripts, logging, replay helpers, and modified simulator glue live here.

Policy/model artifacts are stored outside source code:

- `data/models/unitree_mujoco_policy/` for `.onnx` policies and motion CSVs
- `data/sim_state_logs/` for runtime CSV logs

The default G1 velocity and dance policy artifacts come from
`third_party/unitree_rl_mjlab`. Symlink them into `data/models` using the
commands in `docs/operations/unitree_mujoco_policy_runtime.md`.

Run the default G1 policy smoke test from the repository root:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98_2.py \
  --policy-onnx policy.onnx \
  --policy-warmup 2.0 \
  --domain-id 1 \
  --interface lo
```

The launcher starts the policy before the simulator. Keep that order for
humanoid policies so the robot does not fall before the controller publishes.
`run98_2.py` also holds a FixStand-style warmup before ONNX inference. Increase
`--policy-warmup` to `3.0` or `4.0` if the robot still shakes right after spawn.

By default the pygame control window is hidden for headless smoke tests. Add
`--policy-window` to show the `GAMEPAD CONTROL` window for keyboard control.
Close the window or press `Esc` to stop the policy and simulator.

Use `--policy-onnx` to override the default model picked for a policy script.
