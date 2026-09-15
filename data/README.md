# Data and runtime artefacts

Mọi dữ liệu lớn và output tái sinh đều ở đây, tách khỏi source/vendor. Nội dung
ngoài README/.gitkeep bị Git ignore mặc định.

- `datasets/`, `processed/`: dữ liệu đầu vào và dữ liệu đã chuẩn hóa.
- `models/`: model đang được runtime dùng; kiểm tra symlink trước khi chạy.
- `policies/`: bản sao policy/checkpoint đã thu thập theo run.
- `runs/`: log/checkpoint/video của training và launcher.
- `rosbags/`: recording từ ROS 2/hardware.
- `sim_state_logs/`: CSV state/action của simulator.
- `cache/`: cache pip, Warp, matplotlib và Isaac Lab; có thể tái tạo.
