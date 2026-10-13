# Flat-Plus locomotion in HB

P0/P1/P2 are implemented as single-model routes. The current checkout has the
335-D gait 0917 v4 route active:

```yaml
flat_policy_contract: flat_plus_gait_h4_v1
flat_model: policy_flat_plus_gait_0917_v4.onnx
```

The new artifacts live under `policies/locomotion/flat_plus/` and are selected
by the contract. The loader resolves legacy models from `policies/flat/` and
the new contracts from `policies/locomotion/flat_plus/`. The legacy pair remains
available for rollback.

Available profiles:

| Contract | Model | Input |
|---|---|---:|
| `flat_plus_h4_v1` | `policy_flat_plus_h4_v1.onnx` | 332D |
| `flat_plus_h5_v1` | `policy_flat_plus_h5_v1.onnx` | 415D |
| `flat_plus_gait_h4_v1` | `policy_flat_plus_gait_h4_v1.onnx` | 335D |
| `flat_plus_gait_h4_v1` | `policy_flat_plus_gait_walkquality_v1.onnx` / `policy_flat_plus_gait_0917_v2.onnx` / `policy_flat_plus_gait_0917_v3.onnx` / `policy_flat_plus_gait_0917_v4.onnx` | 335D |

H4/H5 use the canonical 83-D frame history in term-major,
oldest-to-newest order with `repeat_first` startup padding. The gait profile
appends one current-only `STAND,WALK,W2S` one-hot. Its FSM consumes command,
raw gyro and raw joint velocity; it does not use camera, terrain or contact
truth. Artifacts with `flat_plus_gait_fsm_v1_stand_recovery_v1` additionally
use onboard projected gravity to gate tilted STAND entry and trigger immediate
or sustained push recovery. For this qualified 0917 overlay, phase is zero and
frozen in STAND, then restarts at zero on STAND exit. Legacy gait artifacts do
not enable this overlay and retain their previous FSM and phase behavior.

Every selected model must pass ONNX shape, contract metadata, qualified gait
FSM metadata and mandatory PD metadata checks before preflight succeeds.
Artifact hashes are recorded in `policies/locomotion/flat_plus/SHA256SUMS`.

## Deploy chain

`integration/scripts/preflight.sh` and `update_model_manifest.sh` no longer
carry their own contract tables. Both run `run_r1 --preflight` and read the
machine-readable line it prints:

```
[Preflight] MODEL contract=flat_plus_h5_v1 rel=policies/locomotion/flat_plus/policy_flat_plus_h5_v1.onnx input=415 output=24
```

The manifest is then checked against that line (path, input dim, output dim,
contract) plus the SHA-256 of the file the runtime actually loads. Adding a
contract means touching `FlatPolicyProfile.hpp` and `Tuning::Validate()` only —
no shell change. Before this, the two scripts hard-coded `policies/flat/$model`
and a contract list that had drifted in both directions: they accepted
`r1_unified_v10`, which the runtime rejects, and hard-failed `flat_plus_h5_v1`
and `flat_plus_gait_h4_v1`, which the runtime runs. Only `legacy_83` could be
deployed.

To test one profile without changing the active config, use the CTest target:

```bash
cmake -S . -B build_p0p2_20260908 -DBUILD_TESTING=ON
cmake --build build_p0p2_20260908 -j4
ctest --test-dir build_p0p2_20260908 --output-on-failure
```

To switch profiles, copy only the matching example values into
`config/locomotion.yaml`, run `./build/run_r1 --preflight`, update the model
manifest, and restart the application. Rollback is the unchanged legacy pair
above. New profiles do not support the legacy arm gesture overlay.
