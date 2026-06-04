# Unitree MuJoCo Policy Runtime

This directory contains local Happy-Baby-R1 runtime scripts for running trained
policies against Unitree MuJoCo.

Keep `third_party/unitree_mujoco` as a clean upstream vendor checkout. Local
policy scripts, logging, replay helpers, and modified simulator glue live here.

Policy/model artifacts are stored outside source code:

- `data/models/unitree_mujoco_policy/` for `.onnx` policies and motion CSVs
- `data/sim_state_logs/` for runtime CSV logs

Run the default G1 policy smoke test from the repository root:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98.py \
  --domain-id 1 \
  --interface lo
```

Use `--policy-onnx` to override the default model picked for a policy script.
