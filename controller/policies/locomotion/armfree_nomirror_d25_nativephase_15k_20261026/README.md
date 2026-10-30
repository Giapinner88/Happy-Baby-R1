# AB_NoMirror_D25 — final checkpoint14999

D25 comparison: knee/ankle damping2.5, hip damping2.0; preserve this native PD.

Packaged as an opt-in locomotion bundle. Source checkpoint: `/home/ubuntu22/train_mujoco/unitree_rl_mjlab_gait_pro/logs/armfree_damping25_20261023/no_mirror/model_14999.pt`. Seed42,4096env,15000iterations, no mirror. Exact default pose, joint order, PD, effort limits and action scales are resolved from this same native configuration and recorded in `packaging.json`; source checkpoint, ONNX and package SHA256 are locked there. Metadata is deduplicated using ONNX Runtime effective values; the actor graph and inference outputs are unchanged.

Interface:335D=332D H4 term-major oldest→newest plus raw3D one-hot STAND,WALK,W2S;24raw actions;332D normalizer is embedded. FSM is `flat_plus_gait_fsm_legs12_stand_recovery_v1`, signed-vector EMA with12leg RMS, dwell0.5s, upright and push exit inherited.

**Native phase contract:** continuous time from policy reset, first observation at t=0, sin/cos masked to0 when command3-norm<0.1. STAND does not freeze the clock. This differs from the older HB stand-reset clock. The explicit `gait_phase_contract` metadata opts into the new runtime branch; existing bundles without this metadata keep their previous behavior. Build the updated HB runtime before selecting this bundle.

A source ONNX with `training_phase_clock` metadata must not be treated as a stand-reset actor just because the335D shape matches. The original source remains unchanged. The additional metadata describes the exact training phase, not a new learned policy.

Simulation findings: priority retest288trials/policy includes motion, turn, stand, stop, push and gesture. Both source Speed20 actors still have lateral swing at high speed and transient arm offsets during stopping. No hardware acceptance is implied. H4 overlay is interface-compatible; arbitrary teleoperation is not a specialized training objective.

Native command ranges: vx(-1.0, 1.5)m/s, vy(-0.6, 0.6)m/s, yaw(-1.0, 1.0)rad/s. These metadata ranges do not change HB operator limits. To select later, explicitly set `walk_policy`/`teleop_policy` in `controller/config/locomotion.yaml` to `armfree_nomirror_d25_nativephase_15k_20261026` and ensure gait_period_s0.6 and gait_settle_dwell_s0.5; active configs and integration model manifest are unchanged by packaging.

Reproduce from the training repo: `python scripts/package_armfree_locomotion_hb.py --variant selected`. All exact hashes/parity results live in `packaging.json`; the selection rationale is in HB `docs/policies.md`.

Selecting only the walk route while retaining the current `u2a_seed42` teleop bundle is rejected by the existing H4 gate (`teleop_h4:false` on u2a). The locally verified selection uses this bundle for both walk_policy and teleop_policy; packaging does not modify either active key.
