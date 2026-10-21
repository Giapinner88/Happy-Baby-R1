# Roll V7 HOME V1 — completed training export

This is the HOME entry/exit V1 fine-tune resumed from V1 model_21000.pt,
not the raw V7 policy, cancelled contact-step V3 policy, or preview-only V4.
Original source: NUS 0012_JumpAndRoll001, selected roll segment retargeted
with the balanced whole-body V7 pipeline.

Motion: v7_home_entry_exit_20261020_v1/roll_v7_home_entry_exit_50hz.npz.
308 frames at 50 Hz, fps float32. HOME bridges are 1.5 seconds at each end,
with 0.3-second holds. Quintic joint transitions use ankle-midpoint XY
anchoring and collision grounding. V1 can slide individual feet when
changing stance width; it does not enforce per-foot stance constraints.

Training: Unitree-R1-Tracking-No-State-Estimation, GOC, 4096 environments,
GPU 0. Training completed with final checkpoint model_29998.pt (approximately
30k total). This is the completed run export, NOT a hardware-validated policy.
The previously observed head-floor contact has not been fixed by packaging;
the policy still has 24 actions and no trained head joints.
Metadata and gains are taken verbatim from this same run's framework ONNX.
No MP3, as requested. Installed inactive; dance.yaml is unchanged.

Stable package name: roll_v7_home_v1_goc_4096env_20261020.
This folder replaces the earlier model_25500.pt snapshot, using final
checkpoint and matching same-run exports. The training motion is unchanged.
