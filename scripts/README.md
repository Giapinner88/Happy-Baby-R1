# Workspace entry points

Đây là các lệnh ổn định của workspace, không phải nơi đặt logic framework lớn.
Chạy từ repo root và chọn nhóm đúng mục đích:

- `training/`: train/play/export MJLab và Isaac Lab.
- `simulation/`: viewer MuJoCo, Docker Isaac Lab và controller reference.
- `bridge/`: DDS bridge và policy runtime local.
- `assets/`: đồng bộ asset nguồn sang runtime MJCF.

Không gọi script trong `third_party/` trực tiếp khi workspace đã có wrapper.
