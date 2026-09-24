# slope_meta_v6 package

Self-contained simulator package copied from
`unitree_rl_mjlab_meta/deploy/robots/r1/config/policy/velocity/v0`.

- `exported/slope_meta.onnx`: recurrent three-class gate.
- `exported/slope_up.onnx`: uphill expert.
- `exported/slope_down.onnx`: downhill expert.
- `params/deploy.yaml`: registry, selector and common action/PD contract.
- `evaluation/meta_gate_evaluation.json`: source evaluation record.
- `SHA256SUMS`: immutable artifact verification.

Run from the `simulate_cpp` directory:

```bash
./run_sim.sh preflight
./run_sim.sh meta mixed
```

The source registry retains `enabled: false` because it has not passed real
robot acceptance. The C++ simulator requires an explicit meta command, so this
flag is never silently promoted into hardware deployment.

The simulator runtime adds a meta-only startup supervisor around this package:
commands stay at zero during the 50-frame recurrent warm-up, the neutral expert
blend actively balances instead of passively holding the default pose, and
commands are then clamped/ramped. Low-confidence transition frames remain
command-preserving `NEUTRAL`; invalid data still fails closed.
