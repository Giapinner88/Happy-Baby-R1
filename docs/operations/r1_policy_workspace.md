# R1 Policy Workspace

This workspace keeps `third_party/unitree_rl_mjlab` and `third_party/unitree_rl_lab` as read-only upstream references. R1-specific training glue lives in the main workspace under `training/` and `scripts/`.

All generated outputs stay under `data/`:

- Train logs and checkpoints: `data/runs/<framework>/logs/rsl_rl/...`
- Collected deployable policies: `data/policies/<framework>/<run>/`

## Quick Checks

```bash
python scripts/training/r1_policy_workspace.py status
```

## Train With Mjlab

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
python scripts/training/r1_policy_workspace.py train mjlab --terrain flat --num-envs 4096
```

Mjlab already has R1 tasks:

- `Unitree-R1-Flat`
- `Unitree-R1-Rough`

During training, mjlab saves checkpoints under:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/<run>/model_<iter>.pt
```

and exports `policy.onnx` beside the checkpoint run directory at each save interval.

The workspace wrapper defaults MJLab to `--agent.logger=tensorboard` so local runs do not require WandB login. Pass `--agent.logger=wandb` explicitly if you want the upstream WandB behavior.

## Train With Unitree RL Lab

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
python scripts/training/r1_policy_workspace.py train rl_lab --num-envs 4096 --max-iterations 10001
```

The workspace overlay registers:

```text
Unitree-R1-Velocity
```

without editing `third_party/unitree_rl_lab`.

## Export And Collect Policies

For mjlab, collect an already exported policy:

```bash
python scripts/training/r1_policy_workspace.py collect mjlab
```

For Unitree RL Lab, run play once in headless one-frame mode so upstream export creates both JIT and ONNX files, then collect them:

```bash
python scripts/training/r1_policy_workspace.py export rl_lab
```

Collected outputs are placed under:

```text
data/policies/<framework>/<run>/
```

Expected files:

- `model_<iter>.pt`
- `policy.onnx`
- `policy.onnx.data`, when ONNX creates an external data file
- `policy.pt`, for Unitree RL Lab exports
