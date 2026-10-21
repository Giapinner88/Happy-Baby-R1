# ChaCha V10 HOME V1 — completed training export

This is the ChaCha HOME V1 fine-tune from original ChaCha V10 model_19999.pt,
not the original no-HOME policy. Source: NUS 0008_ChaCha001, V10 arm-smoothed
whole-body retargeting, with the same HOME bridge method as roll HOME V1.

Motion: v10_home_entry_exit_20261020_v1/chacha_v10_home_entry_exit_50hz.npz.
2910 frames at 50 Hz, fps float32. HOME bridges are 1.5 seconds at each end,
with 0.3-second holds. Central source poses/timing are preserved; source
boundaries are eased. Quintic joint transitions use ankle-midpoint XY
anchoring and collision grounding. V1 does not lock individual stance feet.

Training: Unitree-R1-Tracking-No-State-Estimation, GOC, 4096 environments,
GPU 1, 10k additional iterations toward approximately 30k total.
Training completed with final checkpoint model_29998.pt (approximately 30k
total). This is the completed run export, NOT a hardware-validated policy.
Metadata and gains come verbatim from this same run's framework ONNX.
No MP3, as requested. Existing user selection dance_6, speed/volume/trim and
dance.yaml are preserved.

Stable package name: chacha_v10_home_v1_goc_4096env_20261020.
This folder replaces the earlier model_25000.pt snapshot with final
checkpoint and matching same-run exports; the training motion is unchanged.
