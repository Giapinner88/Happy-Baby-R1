# R1 training overlays

Code/config R1 do dự án sở hữu, được nạp bên trên vendor mà không sửa
`third_party/unitree_rl_mjlab` hoặc `third_party/unitree_rl_lab`. Cấu trúc chỉ
còn hai nhánh trực tiếp, một nhánh cho mỗi framework.

- `mjlab/`: MJLab robot config và profile train.
- `isaaclab/`: Isaac Lab/Unitree RL Lab task overlay.

Entry point: `scripts/training/r1_policy_workspace.py`. Checkpoint/log phải
vào `data/runs/`; export phát hành vào `data/policies/` rồi chọn model active
qua `data/models/`.
