# A-RMA runtime bundle attached to one Flat policy

The runtime accepts one self-contained package directory. The first staged
bundle is the `o83` finetune from 2026-09-08:

```text
flat_o83_k50_z8_v1/
  params/deploy.yaml
  models/adapter.onnx
  models/actor.onnx
```

Available package:

`policy/locomotion/arma/flat_o83_k50_z8_v1/`

It is opt-in and does not replace the active single-policy configuration.

Minimal `params/deploy.yaml`:

```yaml
arma:
  config_version: 1
  bundle_contract: flat_plus_arma_o83_k50_z8_v1
  adapter_contract: flat_plus_arma_adapter_k50_z8_v1
  base_policy_contract: flat_plus_h4_v1
  actor_view: flat_plus_h4
  actor_base_dim: 332
  actor_history_steps: 4
  observation_dim: 83
  action_dim: 24
  history_steps: 50
  latent_dim: 8
  actor_input_dim: 340      # 332 + 8; see policy README for other views
  adapter_decimation: 1
  max_latent_age_steps: 1
  adapter_model: models/adapter.onnx
  actor_model: models/actor.onnx
```

The adapter input is time-major `[oldest ... newest]` history of canonical
83-D frames. The actor input is the declared base view plus latent. Both
models must be float32, single-input, single-output, and carry matching
bundle/role/base-policy/actor-view/sha256 metadata. The actor also repeats
`policy_contract=<base_policy_contract>` so a wrong base actor cannot be silently
attached. In addition, the adapter declares
`base_observation_schema=r1_common_pd_base83_v1` and
`history_order=time_major_oldest_to_newest`; the actor declares the same base
schema plus the matching `actor_input_schema`, history order/padding, and
observation term list. A gait-conditioned actor additionally declares the
same one-hot gait metadata as `flat_plus_gait_h4_v1`. Missing or inconsistent
metadata is a hard preflight failure. Do not reuse this actor with another
Flat base policy: export a new bundle and train it with that policy's actor
observation contract.

The current `o83` export carries component-local ARMA metadata and keeps the
train-time action/PD arrays in its `params/deploy.yaml`. Its actor prefix is
the canonical 83-D frame, so its actor input is 91-D rather than the H4 example
above.

Run from this directory's parent:

```bash
./run_sim.sh single flat
```
