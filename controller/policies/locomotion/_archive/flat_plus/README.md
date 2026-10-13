# HB Flat-Plus locomotion artifacts

These are selectable single-model policies. The v1 artifacts were imported from
the verified `unitree_mujoco/simulate_cpp/policy/locomotion/flat_plus` bundle.
The 0917 v2, v3, v4 and selected v5 artifacts are newer 335-D training exports. They share
the same HB runtime contract; the runtime qualifies them through exact model
metadata (`gait_task_variant=0917`, `0917V3`, `0917V4`, `0917V5A1` or
`0917V5A2A`), while the deploy
manifest records the active artifact hash.

| Contract | Input | Observation |
|---|---:|---|
| `flat_plus_h4_v1` | 332D | 4 canonical 83D frames, term-major, oldest to newest |
| `flat_plus_h5_v1` | 415D | 5 canonical 83D frames, term-major, oldest to newest (runtime only; no artifact in this repo) |
| `flat_plus_gait_h4_v1` | 335D | H4 view plus current `STAND,WALK,W2S` one-hot |

`policy_flat_plus_gait_walkquality_v1.onnx`,
`policy_flat_plus_gait_0917_v2.onnx`, and
`policy_flat_plus_gait_0917_v3.onnx` and
`policy_flat_plus_gait_0917_v4.onnx` use the same
`flat_plus_gait_h4_v1` runtime contract. Check `config/locomotion.yaml` for
the active selection; v2/v3/v4 and v1 remain available for comparison.

The legacy `legacy_83` route with `policies/flat/policy_goc_2.onnx` remains
available for rollback, but is not the active route in this checkout.
These models do not support the legacy arm overlay. ONNX contract metadata is
checked before inference; `SHA256SUMS` records artifact provenance.

## 0917 v5 trained variants (opt-in)

| Variant | Artifact | Training checkpoint | Evaluation note |
|---|---|---:|---|
| A1 | `policy_flat_plus_gait_0917_v5_a1.onnx` | 20,000 iterations (`model_19999.pt`) | Better translation and push tracking than A2A in the paired seed-42 play checks. |
| A2A | `policy_flat_plus_gait_0917_v5_a2a.onnx` | 11,300 iterations (`model_11299.pt`) | Better yaw tracking than A1; weaker translation tracking in those checks. |

Both are exports using `flat_plus_gait_h4_v1` (335-D observation,
24-D action) with a 0.6 s gait period. Their training YAMLs and seed-42 speed
and push evaluation JSONs are stored under `provenance/0917_v5_a1/` and
`provenance/0917_v5_a2a/`. The ONNX files retain their embedded policy and
training metadata. HB accepts only the two V5 variants listed above and checks
the V5 ablation metadata. Matching opt-in examples live in `config/` with a
0.5 s settle dwell. Changing the active model remains a separate operator choice.
