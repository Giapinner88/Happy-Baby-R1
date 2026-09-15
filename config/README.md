# Configuration

Cấu hình dùng chung nhưng không chứa secret:

- `cyclonedds_config.xml`: cấu hình CycloneDDS cho ROS 2/DDS.
- `netplan_static_ethernet.yaml`: mẫu mạng Ethernet tĩnh.
- `ros-archive-keyring.gpg`: keyring dùng khi cài ROS từ apt.

Mọi YAML cấu hình do workspace sở hữu nằm ở đây. `package.xml` nằm cạnh ROS 2
package vì là manifest build, còn config/checkpoint sinh theo từng run vẫn nằm
trong `data/runs/` để không trộn runtime artefact với cấu hình chuẩn.

Thông tin IP, domain và interface dành cho một buổi chạy phải được xác nhận
theo SOP trước khi dùng với robot thật.
