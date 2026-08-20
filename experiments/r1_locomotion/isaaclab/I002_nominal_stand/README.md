# I002 — R1 nominal stable standing on a plane

I002 isolates the first question before slow walking: can R1 remain upright
and stationary under a fixed nominal IsaacLab setup? It is separate from P001
and does not modify its running or completed evidence. Read
[`experiment.md`](experiment.md) before execution.

## 1. Preconditions

Run from repository root. I002 needs the direct `unitree_sim_env` and a
working NVIDIA driver. Do not start an evidence run if the GPU check fails.

```bash
cd "$(git rev-parse --show-toplevel)"
nvidia-smi -i 1

scripts/training/run_r1_isaaclab.sh \
  python -c "import gymnasium as gym; import training.isaaclab; print(gym.spec('Unitree-R1-Stand').id)"
```

The expected final line is `Unitree-R1-Stand`. This check only registers the
task; it does not launch Isaac Sim.

## 2. Adjusting the editable configuration

Edit [`config.json`](config.json) **before** starting a new run. The runner
copies it into the run as `experiment_config.json`; never edit that snapshot.

For I002 comparability, the following must remain fixed: zero command, plane,
no curricula/pushes/randomization/observation corruption, zero reset joint
velocity, and all capture/export flags. The validator refuses changes to these
fields. You may adjust `training.num_envs`, `max_iterations`, `save_interval`,
`seed`, `run_name`, GPU index, and video cadence. A different seed is a new
run; a modified standing protocol requires a new experiment ID.

```bash
python3 experiments/r1_locomotion/isaaclab/I002_nominal_stand/validate_config.py
python3 experiments/r1_locomotion/isaaclab/I002_nominal_stand/run.py --dry-run
```

## 3. Start an evidence run

```bash
python3 experiments/r1_locomotion/isaaclab/I002_nominal_stand/run.py
```

It creates a UTC ID such as `I002_nominal_stand_20260728T040000Z`. To assign
one manually, it must be new:

```bash
python3 experiments/r1_locomotion/isaaclab/I002_nominal_stand/run.py \
  --run-id I002_nominal_stand_seed42_rerun1
```

`--smoke` writes only disposable output below `results/smoke`; it is not I002
evidence. Training outputs include resolved config/metadata/status, upstream
checkpoints and TensorBoard events, MP4 training video, derived CSV/plots,
JIT/ONNX export, and `evidence_completeness.json`.

## 4. Evaluate a checkpoint

Do not infer standing quality from the parallel training video. After a stable
checkpoint exists, run the fixed one-environment replay. Use a new output path
for each checkpoint or repeat:

```bash
RUN=experiments/r1_locomotion/isaaclab/runs/I002_nominal_stand_<UTC_ID>
CKPT="$RUN/logs/rsl_rl/r1_velocity/<UPSTREAM_RUN>/model_250.pt"
OUT="$RUN/evaluations/$(date -u +%Y%m%dT%H%M%SZ)_model_250"

CUDA_VISIBLE_DEVICES=0 scripts/training/run_r1_isaaclab.sh \
  python experiments/r1_locomotion/isaaclab/I002_nominal_stand/evaluate.py \
  --headless --checkpoint "$CKPT" --output-dir "$OUT"
```

The evaluation records `trace.csv`, `trace.npz`, `metrics.csv`, two plots, MP4
video, `status.json`, and `evaluation_manifest.json`. Read the status and
metrics before treating a result as positive. A pass is simulation-only and
does not authorize walking, disturbance claims, policy promotion by itself, or
hardware control.
