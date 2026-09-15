# System Communication Topology
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-ARCH-002
**Author:** Operation & Data Lead
**Status:** Draft / Working

Tài liệu này chuẩn hóa sơ đồ truyền tin giữa máy trạm, robot, Lightning AI và các máy remote nghiên cứu.

## 1. Vai trò từng máy

| Thành phần | OS mục tiêu | Vai trò chính | Có vận hành robot trực tiếp? |
| :--- | :--- | :--- | :--- |
| Máy trạm vận hành | Ubuntu 20.04 LTS | Low-level command, ROS 2 Foxy, CycloneDDS, mô phỏng local, ghi log | Có |
| Robot Jetson Orin | Ubuntu tương thích Unitree/R1 | Nhận low-level command, phản hồi trạng thái robot | Có, thông qua máy trạm |
| Lightning AI | Studio Ubuntu 24.04 host + Docker Isaac Lab/Isaac Sim container | Training high-level bằng Isaac Lab từ dữ liệu đầu vào/đầu ra đã chuẩn hóa | Không |
| 3 máy remote nghiên cứu | Không chốt làm baseline vận hành | Nghiên cứu, phân tích, thử nghiệm thuật toán hoặc đọc dữ liệu | Không |

## 2. Luồng điều khiển và dữ liệu

```text
Máy trạm Ubuntu 20.04
  -> low-level command qua Ethernet/DDS
  -> Robot Jetson Orin
  -> robot state / telemetry
  -> Máy trạm ghi log, rosbag2, kiểm tra an toàn
  -> dữ liệu đã chuẩn hóa
  -> Lightning AI Studio + Docker Isaac Lab training high-level
  -> model/output high-level quay lại pipeline kiểm thử
```

## 3. Nguyên tắc đồng bộ

* Máy trạm và robot là cặp vận hành trực tiếp, nên ưu tiên đồng bộ hệ điều hành, ROS 2, DDS và network config.
* Lightning AI không dùng để vận hành trực tiếp. Với Lightning AI, chỉ chuẩn hóa dữ liệu đầu vào, dữ liệu đầu ra, version training code và artifact model.
* Các máy remote nghiên cứu không được xem là node điều khiển robot. Nếu cần dùng kết quả từ các máy này, kết quả phải quay về máy trạm qua quy trình review/test riêng.
* Build ROS workspace từ root repo phải giới hạn `--base-paths src`; `third_party` được quản lý như nguồn vendor/build riêng.
* `third_party/unitree_mujoco` giữ sạch theo upstream; policy/runtime mô phỏng nội bộ đặt ở `sim/unitree_mujoco_policy`, model đặt ở `data/models/unitree_mujoco_policy`, log đặt ở `data/sim_state_logs`.

## 4. Tài liệu liên quan

* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
* Cấu hình Ethernet tĩnh: [../operations/network_configuration_static_ethernet.md](../operations/network_configuration_static_ethernet.md)
* Quy trình third-party: [../operations/third-party_build.md](../operations/third-party_build.md)
* Quy trình rosbag2: [../operations/rosbag2_operation.md](../operations/rosbag2_operation.md)
* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)
