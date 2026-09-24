# Flat-Plus history ablation bundle

Created 2026-09-06T23:37:24 on the training machine.
Repo revision `11b3d6d7061481e7bad21728943df5107c82d5e4`.

Base observation schema `r1_common_pd_base83_v1`, critic
**98**, action **24** -- shared by
every arm. The actor width is what differs between arms; see the list below and
`MANIFEST.yaml`, never the folder name.

## What to do with this folder

1. Copy the whole directory (or the `.tar.gz` next to it) to the dev machine.
2. `sha256sum -c SHA256SUMS` -- transfer corruption is silent otherwise.
3. On the dev machine, run the checker against the selected ONNX:

   ```
   python scripts/check_flat_plus.py \
       --history-length 4 \
       --onnx <this folder>/DEPLOY/policy_flat_plus_h4_v1.onnx --strict
   ```

4. Only after the simulator closed-loop gate passes does the ONNX go anywhere
   near `HB/high_level_2/config/locomotion.yaml`.

`DEPLOY/` holds the selected arm (**h4_scratch**, `flat_plus_h4_v1`). These two files, and only these two, are what the dev machine needs:

- `policy_flat_plus_h4_v1.onnx` -> `HB/high_level_2/policies/flat/` and the simulator
- `model_flat_plus_h4_v1.pt` -> kept for provenance and for any re-export


## Arms collected

- `arms/h4_scratch/` -- `flat_plus_h4_v1`, actor 332D, from `logs/rsl_rl/r1_flat_plus/2026-09-06_01-24-19_h4_scratch_seed42` (checkpoint `model_20000.pt`)
- `arms/h5_candidate/` -- `flat_plus_h5_v1`, actor 415D, from `logs/rsl_rl/r1_flat_plus_h5/2026-09-06_01-24-19_h5_scratch_seed42` (checkpoint `model_20000.pt`)

## What is deliberately NOT here

- No optimizer state: this bundle is for evaluation and deployment, not for
  resuming training. Resume from the run directory on the training machine.
- No baseline artifacts. `policy_goc_2.onnx` and `model_flat_common_pd.pt` stay
  exactly where they are; nothing in this bundle overwrites them.
