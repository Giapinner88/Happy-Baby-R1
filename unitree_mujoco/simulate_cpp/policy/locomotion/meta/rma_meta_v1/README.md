# rma_meta_v1 simulation candidate

Isolated RMA-inspired hierarchical meta-policy for MuJoCo C++:

- `50 x 83` proprioceptive history -> 8-D adaptation latent;
- current 83-D observation + latent + previous expert -> categorical PPO selector;
- classes: `slope_up`, `slope_down`, `flat`, `HOLD`;
- selected frozen expert -> common-PD 24-D action with target crossfade;
- no camera or simulator-only terrain/dynamics factor enters deployment inference.

The original transferred release remains available as
`params/deploy.rma_meta.yaml`; `params/deploy.yaml` is the MuJoCo C++ runtime
copy with a command slew guard added. Both `rma_meta.enabled` and legacy
`meta_policy.enabled` remain `false`; selection is opt-in through the launcher.

```bash
./run_sim.sh preflight-rma-meta
./run_sim.sh rma_meta flat
./run_sim.sh rma_meta 15-platform
./run_sim.sh rma_meta mixed
```

The source evaluation is not an acceptance result: 686/1834 completed episodes
fell and 75.05% of meta decisions were `HOLD`. Keep this package simulation-only
until the independent MuJoCo matrix passes; do not replace `meta_policy_v1` or
`slope_meta_v6`.

## MuJoCo C++ headless status (2026-09-04)

- preflight, ONNX Python/C++ parity, artifact hashes, and 67/67 CTest: pass;
- final full matrix: 62/64 control passes and 64/64 clean process exits;
- repeatable failures: Flat STAND drift `0.791 m` (`0.30 m` limit), and
  20-degree STAND height drop `0.294 m` (`0.25 m` limit);
- mixed traversal is not repeatable: one pass and two falls across three clean
  headless reset runs.

The C++ runtime is ready for further simulation, but this policy artifact is
not accepted for default selection or hardware deployment. Keep
`rma_meta.enabled: false`.
