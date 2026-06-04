# Docs Map - Happy Baby R1

**Mục tiêu:** Đây là bản đồ tài liệu trung tâm cho toàn bộ thư mục `docs`, giúp tra cứu nhanh theo chủ đề và theo vai trò làm việc.

## 1. Cách dùng bản đồ này

1. Chọn lộ trình đọc theo vai trò ở mục 2.
2. Nếu cần đi sâu theo chuyên đề, vào mục 3.
3. Khi cần tra cứu đầy đủ, dùng chỉ mục ở mục 4.

## 2. Lộ trình đọc theo vai trò

### 2.1. Người vận hành (Operator)

1. [Quy trình vận hành SOP](operations/SOP_v0.md)
2. [Quy tắc an toàn tổng](safety/safety_rules.md)
3. [Checklist thiết lập mạng/DDS](operations/network_setup_checklist.md)
4. [Quy trình rosbag2](operations/rosbag2_operation.md)
5. [Mẫu log test](templates/test_log_template.md)

### 2.2. Kỹ sư tích hợp (Integration)

1. [Third-party build & integration](operations/third-party_build.md)
2. [Cấu hình mạng static Ethernet](operations/network_configuration_static_ethernet.md)
3. [Triển khai DDS](operations/dds_implementation.md)
4. [Rationale kiến trúc mạng/DDS](architecture/network_dds_rationale.md)
5. [Golden machine spec](hardware/golden_machine_spec.md)

### 2.3. Kỹ sư mô phỏng (Simulation)

1. [Development environment setup](operations/development_environment_setup_guide.md)
2. [Ubuntu 20.04 setup guide](operations/ubuntu_20_04_lts_setup_guide.md)
3. [Unitree MuJoCo policy runtime](operations/unitree_mujoco_policy_runtime.md)
4. [Quy trình rosbag2](operations/rosbag2_operation.md)
5. [Quy ước đặt tên](operations/naming_convention.md)
6. [Bộ thực hành vận hành hệ thống](operations/practice/README.md)

### 2.4. Thành viên mới (Onboarding nhanh)

1. [Bộ thực hành vận hành hệ thống](operations/practice/README.md)
2. [Bản đồ hệ thống](operations/practice/01_system_map.md)
3. [Khởi động và chạy buổi vận hành](operations/practice/02_boot_and_run_exercise.md)
4. [Dòng dữ liệu và DDS](operations/practice/03_data_flow_and_dds_exercise.md)
5. [Tự kiểm nhanh](operations/practice/06_quick_self_check.md)

## 3. Bản đồ theo chuyên đề

### 3.1. Safety

- [Safety rules (index)](safety/safety_rules.md)
- [Hardware safety rules](safety/hardware_safety_rules.md)
- [Software safety rules](safety/software_safety_rules.md)

### 3.2. Operations

- [SOP vận hành](operations/SOP_v0.md)
- [Network setup checklist](operations/network_setup_checklist.md)
- [Network static Ethernet](operations/network_configuration_static_ethernet.md)
- [DDS implementation](operations/dds_implementation.md)
- [rosbag2 operation](operations/rosbag2_operation.md)
- [Naming convention](operations/naming_convention.md)
- [Work completion report](operations/work_completion_report_2026-05-05.md)
- [Third-party build](operations/third-party_build.md)
- [Unitree MuJoCo policy runtime](operations/unitree_mujoco_policy_runtime.md)
- [Development environment setup](operations/development_environment_setup_guide.md)
- [Ubuntu 20.04 setup](operations/ubuntu_20_04_lts_setup_guide.md)

### 3.3. Architecture & Layout

- [System communication topology](architecture/system_communication_topology.md)
- [Network DDS rationale](architecture/network_dds_rationale.md)
- [Laboratory layout](Lab%20setup/laboratory_layout.md)

### 3.4. Hardware baseline

- [Golden machine spec](hardware/golden_machine_spec.md)

### 3.5. Templates

- [Test log template](templates/test_log_template.md)

### 3.6. Practice

- [Practice index](operations/practice/README.md)
- [01 - System map](operations/practice/01_system_map.md)
- [02 - Boot and run](operations/practice/02_boot_and_run_exercise.md)
- [03 - Data flow and DDS](operations/practice/03_data_flow_and_dds_exercise.md)
- [04 - Test log practice](operations/practice/04_test_log_practice.md)
- [05 - Third-party bridge](operations/practice/05_third_party_bridge_exercise.md)
- [06 - Quick self-check](operations/practice/06_quick_self_check.md)
- [07 - ROS 2 and Conda communication](operations/practice/07_ros2_conda_communication_test.md)
- [08 - State/control simulation](operations/practice/08_state_control_sim.md)
- [Practice tests index](operations/practice/practice_tests_index.md)

## 4. Chỉ mục đầy đủ tài liệu

### 4.1. Markdown

- [architecture/network_dds_rationale.md](architecture/network_dds_rationale.md)
- [architecture/system_communication_topology.md](architecture/system_communication_topology.md)
- [Lab setup/laboratory_layout.md](Lab%20setup/laboratory_layout.md)
- [hardware/golden_machine_spec.md](hardware/golden_machine_spec.md)
- [operations/SOP_v0.md](operations/SOP_v0.md)
- [operations/dds_implementation.md](operations/dds_implementation.md)
- [operations/development_environment_setup_guide.md](operations/development_environment_setup_guide.md)
- [operations/naming_convention.md](operations/naming_convention.md)
- [operations/network_configuration_static_ethernet.md](operations/network_configuration_static_ethernet.md)
- [operations/network_setup_checklist.md](operations/network_setup_checklist.md)
- [operations/rosbag2_operation.md](operations/rosbag2_operation.md)
- [operations/third-party_build.md](operations/third-party_build.md)
- [operations/unitree_mujoco_policy_runtime.md](operations/unitree_mujoco_policy_runtime.md)
- [operations/ubuntu_20_04_lts_setup_guide.md](operations/ubuntu_20_04_lts_setup_guide.md)
- [operations/work_completion_report_2026-05-05.md](operations/work_completion_report_2026-05-05.md)
- [operations/practice/README.md](operations/practice/README.md)
- [operations/practice/01_system_map.md](operations/practice/01_system_map.md)
- [operations/practice/02_boot_and_run_exercise.md](operations/practice/02_boot_and_run_exercise.md)
- [operations/practice/03_data_flow_and_dds_exercise.md](operations/practice/03_data_flow_and_dds_exercise.md)
- [operations/practice/04_test_log_practice.md](operations/practice/04_test_log_practice.md)
- [operations/practice/05_third_party_bridge_exercise.md](operations/practice/05_third_party_bridge_exercise.md)
- [operations/practice/06_quick_self_check.md](operations/practice/06_quick_self_check.md)
- [operations/practice/07_ros2_conda_communication_test.md](operations/practice/07_ros2_conda_communication_test.md)
- [operations/practice/08_state_control_sim.md](operations/practice/08_state_control_sim.md)
- [operations/practice/practice_tests_index.md](operations/practice/practice_tests_index.md)
- [safety/safety_rules.md](safety/safety_rules.md)
- [safety/hardware_safety_rules.md](safety/hardware_safety_rules.md)
- [safety/software_safety_rules.md](safety/software_safety_rules.md)
- [templates/test_log_template.md](templates/test_log_template.md)

### 4.2. PDF tham khảo trong docs

- [user-manual.pdf](user-manual.pdf)
- [UNITREER1MODELLUBERSICHT.pdf](UNITREER1MODELLUBERSICHT.pdf)
- [Student's role.pdf](Student's%20role.pdf)
- [ts_p360_ubuntu_22.04_lts_installation_guide.pdf](ts_p360_ubuntu_22.04_lts_installation_guide.pdf) - PDF tham khảo cũ, không phải baseline OS hiện tại.

## 5. Quy ước cập nhật

Khi thêm tài liệu mới vào `docs`, hãy cập nhật:

1. Mục 3 (theo chuyên đề).
2. Mục 4 (chỉ mục đầy đủ).
3. Lộ trình ở mục 2 nếu tài liệu mới là tài liệu điều hướng chính.
