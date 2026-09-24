# A-RMA `o83` bundle — 2026-09-08

This is the opt-in simulator package built from:

`results/arma_o83_finetune_20260908`

Contract:

- base observation: `r1_common_pd_base83_v1`, 83-D;
- adapter input: 50 time-major frames of 83-D (`[50,83]`), 50 Hz;
- actor input: current 83-D frame plus 8-D estimated latent, 91-D;
- actor output: 24 raw joint-position actions;
- bundle: `flat_plus_arma_o83_k50_z8_v1`.

The adapter is run first, then its latent is appended to the current frame.
Target-position conversion and common PD remain in `ArmaController`, matching
the existing R1 locomotion contract.

## Checks

```bash
cd <simulate_cpp>
./build/arma_preflight policy/locomotion/arma/flat_o83_k50_z8_v1
./run_sim.sh arma flat
```

The active `config/tuning.yaml` is deliberately unchanged. This bundle remains
simulation-only until the contract, latency, and closed-loop gates pass.
