# Training entry points

`r1_policy_workspace.py` là entry point chính cho `status`, `train`, `export`
và `collect`. Các launcher `r1_mjlab_*` và `r1_rl_lab_*` là adapter giữ vendor
sạch và nạp R1 overlay tại `training/mjlab/` hoặc `training/isaaclab/`.
