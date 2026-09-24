# Meta-policy ablation packages — 2026-08-28

These packages are simulation-only variants of `meta_policy_v1`. Their
`exported` directories are read-only symbolic links to the candidate artifacts;
only the YAML runtime condition changes. The validated `slope_meta_v6` package
and normal single-policy routes are not modified.

| Package | Isolated change |
| --- | --- |
| `target_one_layer` | Disable the outer action cross-fade; retain target-q weight blending. |
| `raw_action` | Blend expert raw actions before the common action contract. |
| `hard_switch` | Set the shared cross-fade duration to zero. |
| `history_1`, `history_10`, `history_25` | Zero all but the most recent 1/10/25 observations while retaining the trained 50-step ONNX contract. |
| `no_neutral` | Ignore the Neutral class for finite gate output. |
| `no_hysteresis` | Set exit confidence equal to enter confidence (0.75). |
| `no_dwell` | Reduce configured minimum dwell to zero (the runtime minimum is one control step). |

The unmodified `meta_policy_v1` package is the 50-step, target-q, two-layer
baseline. History variants test inference-time truncation, not separately
retrained GRUs, and must be described with that limitation.
