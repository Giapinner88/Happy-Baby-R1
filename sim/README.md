# Internal simulation

Mã mô phỏng do dự án sở hữu, tách với `third_party/`:

- `unitree_mujoco_policy/`: R1 policy runtime, state logger và DDS glue.
- `isaac_lab_env/`: điểm đặt env nội bộ cho Isaac Lab.
- `mujoco_lab_env/`: prototype MuJoCo/chatbot legacy.
- các file `unitree_r1_*_sim.py`: smoke simulation state/control đơn giản.

Train policy ở `scripts/training/`; chỉ dùng bridge sau khi policy qua
direct-eval theo MJLab semantics.
