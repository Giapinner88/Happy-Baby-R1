# Configuration

Cấu hình dùng chung nhưng không chứa secret:

- `cyclonedds_config.xml`: cấu hình CycloneDDS cho ROS 2/DDS.
- `netplan_static_ethernet.yaml`: mẫu mạng Ethernet tĩnh.
- `ros-archive-keyring.gpg`: keyring dùng khi cài ROS từ apt.
- `r1_quest3_sim.json`: frame, timeout và calibration mặc định dùng chung cho
  Quest-to-R1 simulation/diagnostics.

Config baseline có semantics riêng nằm cạnh experiment; resolved config thuộc
run đã sinh và không được dùng làm editable default.

Thông tin IP, domain và interface dành cho một buổi chạy phải được xác nhận
theo SOP trước khi dùng với robot thật.
