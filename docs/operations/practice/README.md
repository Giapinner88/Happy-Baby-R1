# Bộ file thực hành vận hành hệ thống
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-000
**Author:** Operation & Data Lead
**Status:** Draft / Working

Tài liệu này là lộ trình thực hành ngắn để hiểu cách hệ thống vận hành từ góc nhìn vận hành, dữ liệu và tích hợp.

## Mục tiêu học

Sau khi đọc xong bộ tài liệu này, bạn sẽ nắm được:

1. Hệ thống gồm những lớp nào và mỗi lớp làm gì.
2. Trình tự khởi động một buổi làm việc an toàn.
3. Dữ liệu đi qua hệ thống như thế nào khi chạy ROS 2 / DDS / rosbag2.
4. Cách ghi nhận một buổi test để truy vết lại được.
5. Ranh giới giữa vendor upstream trong `third_party` và runtime local trong `sim`.

## Thứ tự đọc đề xuất

1. [01_system_map.md](01_system_map.md)
2. [02_boot_and_run_exercise.md](02_boot_and_run_exercise.md)
3. [03_data_flow_and_dds_exercise.md](03_data_flow_and_dds_exercise.md)
4. [04_test_log_practice.md](04_test_log_practice.md)
5. [05_third_party_bridge_exercise.md](05_third_party_bridge_exercise.md)
6. [07_ros2_conda_communication_test.md](07_ros2_conda_communication_test.md)
7. [08_state_control_sim.md](08_state_control_sim.md)
8. [09_ubuntu_20_22_dds_compatibility_test.md](09_ubuntu_20_22_dds_compatibility_test.md)
9. [06_quick_self_check.md](06_quick_self_check.md)

## Cách dùng

* Đọc từng file như một bài tập.
* Thực hiện các lệnh trong môi trường simulation trước, sau đó mới áp dụng lên hardware thật.
* Ghi lại mọi quan sát vào một bản log riêng.
* Nếu một bước chưa rõ, quay lại file 01 để kiểm tra lại mô hình tổng thể.

## Tài liệu liên quan

* Quy trình vận hành: [../SOP_v0.md](../SOP_v0.md)
* Thiết lập mạng/DDS: [../network_setup_checklist.md](../network_setup_checklist.md)
* Quy trình rosbag2: [../rosbag2_operation.md](../rosbag2_operation.md)
* Quy trình build third-party: [../third-party_build.md](../third-party_build.md)
* Runtime policy MuJoCo: [../unitree_mujoco_policy_runtime.md](../unitree_mujoco_policy_runtime.md)
