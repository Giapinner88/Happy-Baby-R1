# S001 policy evaluation

S001 is the retained IsaacSim comparison protocol for archived P001 checkpoints.
It is not a MuJoCo approval and does not make a hardware claim. Read
[`simulation_study.md`](simulation_study.md) before executing cases.

For a replaceable root-level visual smoke/demo after CUDA and the direct
IsaacLab launcher are available:

```bash
scripts/evaluation/run_s001_isaaclab_demo.sh \
  experiments/r1_locomotion/isaaclab/runs/P001_velocity_port_pilot_20260728T025327Z/logs/rsl_rl/r1_velocity/2026-07-28_09-53-34_I001_flat_stand_walk/model_2499.pt
```

The demo writes beneath `results/demo/` and is deliberately not S001 evidence.
