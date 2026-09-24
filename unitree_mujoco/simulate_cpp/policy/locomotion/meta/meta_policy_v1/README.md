# meta_policy_v1 simulation candidate

Self-contained Flat + Slope-Up + Slope-Down meta-policy package for MuJoCo C++.
It is isolated from the validated two-expert `slope_meta_v6` package.

- `exported/slope_meta_with_flat.onnx`: recurrent four-class gate.
- `exported/flat_common_pd.onnx`: Flat cold-start/fallback expert.
- `exported/slope_up.onnx`: uphill v6 expert.
- `exported/slope_down.onnx`: downhill v6 expert.
- `params/deploy.yaml`: three-expert registry and common action/PD contract.
- `evaluation/`: checkpoint metadata and source evaluation reports.

Run from `simulate_cpp`:

```bash
./run_sim.sh preflight-meta-policy
./run_sim.sh meta_policy flat
./run_sim.sh meta_policy mixed
./run_sim.sh meta_policy 15
```

This package is a simulation candidate only. The bundled locomotion report was
recorded before the faults-only SAFETY_HOLD/active-Flat cold-start correction
and failed with a high fall rate. The corrected runtime has not yet been
re-evaluated in MjLab or accepted on hardware, so `meta_policy.enabled` remains
`false` and this package must not replace `slope_meta_v6`.

## Local MuJoCo smoke results (2026-08-27)

| Case | Result | Gate transition / observation |
| --- | --- | --- |
| Flat, stand 8 s | PASS | Flat fallback remained stable; peak relative tilt 0.15 deg |
| 15 deg platform, uphill at 0.3 m/s for 10 s | PASS | `flat -> slope_up`; peak relative tilt 6.61 deg |
| 15 deg platform, downhill at -0.3 m/s | FAIL | gate selected `slope_down`, then the fall detector triggered |
| Direct 15 deg spawn, uphill at 0.3 m/s | FAIL | Flat cold-start did not survive the direct inclined reset |
| Mixed 15 deg lane, 0.4 m/s for 24 s | PASS (partial) | `flat -> slope_up`; reached x=5.26 m near the uphill end |
| Mixed complete 15 deg lane traversal | FAIL | full-matrix run completed `flat -> slope_up -> flat -> slope_down`, but peak tilt 26.07 deg exceeded 25 deg |

The old two-expert `slope_meta_v6` passed the same 15 deg platform downhill
smoke test while remaining on its neutral expert blend. This isolates the
downhill failure to the new gate/expert transition path; do not treat passing
contract tests or the uphill smoke test as full three-terrain acceptance.

The full 64-case MuJoCo matrix (0/5/10/15/20/25/30 deg plus mixed, five
command categories and both directions) produced 28 control PASS and 36 FAIL:
12 completed with a metric violation and 24 triggered the fall detector. Two
runs also hit a CycloneDDS assertion during teardown; one had already passed
the control test. A repeated long mixed traversal fell before completion, so
the mixed transition remains non-deterministic and unaccepted.
