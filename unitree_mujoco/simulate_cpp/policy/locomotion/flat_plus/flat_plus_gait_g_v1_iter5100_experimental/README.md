# flat_plus_gait_g_v1_iter5100_experimental

**NOT a release candidate. Do not activate in `config/locomotion.yaml`.**

Gait-G checkpoint at iteration 5100 of a 10000-iteration run (512 envs), the
best `Train/mean_reward` found *after* the C2 curriculum stage (full reward
ramp + push/payload disturbance training) was fully active. Packaged for
internal comparison against the flat_plus_gait_h4_v1 (Gait-C) family, not
because it meets the release bar -- it does not.

See `reports/reward_trajectory_notes.json` for why this specific checkpoint
was chosen over the final one (iter 9999), and `reports/check_flat_plus_gait.txt`
for the structural checker output (dimensions/one-hot/FSM/normalizer all pass;
the one FAIL is `check_flat_plus_gait.py`'s own hard-coded comparison against
variant B metadata -- a known limitation of that script, not a defect in this
export. `gait_task_variant=G` is correct and was attached by the same
`FlatPlusGaitGOnPolicyRunner.save()` path real training uses, which validates
tensor shapes and normalizer width before allowing the export to survive on
disk).

Release-gate check (`scripts/package_gait_h4_to_hb.py::enforce_release_gate`):
**FAILS** -- `mean_reward = -2.07`, required `>= 45.0`. Reward peaked at
+24.2 around iteration 2000 then declined; root cause not yet confirmed
(joint_target_smoothness ruled out by ablation; envs-count ablation in
progress). Do not deploy until this is resolved and a run passes the gate.

HB does not yet implement the v3 GATHER/SETTLE FSM this checkpoint needs
regardless (see `/home/ubuntu22/train_mujoco/gait_bfg_deploy_package/PLAN.md`).
