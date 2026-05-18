# Practice Tests Index
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-INDEX
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Muc tieu

- Tong hop toan bo bai thuc hanh trong thu muc practice.
- Cho biet muc tieu, dau vao, dau ra, va tieu chi hoan thanh.
- Giup chon bai tap phu hop theo muc do va vai tro.

## 2. Tong quan nhanh

| ID | Bai tap | Muc tieu chinh | Dau ra mong doi | Link |
| --- | --- | --- | --- | --- |
| HB-PRAC-001 | Ban do he thong | Hieu cac lop he thong va luong van hanh | Mo ta chuoi van hanh ngan | [01_system_map.md](01_system_map.md) |
| HB-PRAC-002 | Khoi dong va chay buoi test | Mo phong trinh tu van hanh co ban | Log test ngan + checklist da qua | [02_boot_and_run_exercise.md](02_boot_and_run_exercise.md) |
| HB-PRAC-003 | Dong du lieu va DDS | Hieu topic, DDS, rosbag2 | Mo ta topic va duong di du lieu | [03_data_flow_and_dds_exercise.md](03_data_flow_and_dds_exercise.md) |
| HB-PRAC-007 | Giao tiep ROS2 va conda | Kiem tra DDS lowstate/lowcmd | Conda nhan lowstate, ROS2 nhan lowcmd | [07_ros2_conda_communication_test.md](07_ros2_conda_communication_test.md) |
| HB-PRAC-008 | Mo phong state/control | Mo phong 4-DOF va kenh cmd | In state/imu/cmd 2 terminal | [08_state_control_sim.md](08_state_control_sim.md) |
| HB-PRAC-004 | Ghi log buoi test | Tao log co the truy vet | Ban log theo template | [04_test_log_practice.md](04_test_log_practice.md) |
| HB-PRAC-005 | Cau noi third-party | Hieu ranh gioi SDK/wrapper/binding | Tom tat vai tro tung lop | [05_third_party_bridge_exercise.md](05_third_party_bridge_exercise.md) |
| HB-PRAC-006 | Tu kiem nhanh | Tu danh gia sau khi doc | Tra loi >= 4/5 cau hoi | [06_quick_self_check.md](06_quick_self_check.md) |

## 3. Cau truc de xuat

- Nhom tong quan: HB-PRAC-001, HB-PRAC-005
- Nhom van hanh: HB-PRAC-002, HB-PRAC-004
- Nhom du lieu va DDS: HB-PRAC-003, HB-PRAC-007
- Nhom mo phong va flow dieu khien: HB-PRAC-008
- Tu kiem: HB-PRAC-006

## 4. Chi tiet tung bai

### HB-PRAC-001 - Ban do he thong

- Muc tieu: Xac dinh cac lop he thong va luong van hanh.
- Dau vao: [README.md](../../../README.md).
- Dau ra: Mo ta chuoi van hanh theo format "Operator -> ROS 2 -> Wrapper -> SDK -> Robot/Simulator -> Log".
- Tieu chi dat: Tra loi duoc 4 cau hoi o muc 1 cua bai.

### HB-PRAC-002 - Khoi dong va chay buoi test

- Muc tieu: Mo phong mot buoi van hanh ngan co trinh tu ro rang.
- Dau vao: [../network_setup_checklist.md](../network_setup_checklist.md), [../../templates/test_log_template.md](../../templates/test_log_template.md).
- Dau ra: Log test ngan + ghi chu trang thai truoc/sau test.
- Tieu chi dat: Hoan thanh day du 4 pha A-D va co log.

### HB-PRAC-003 - Dong du lieu va DDS

- Muc tieu: Hieu vai tro DDS, topic, rosbag2.
- Dau vao: [../rosbag2_operation.md](../rosbag2_operation.md).
- Dau ra: Mo ta 1 topic (loai du lieu, publish/subscribe, muc dich su dung).
- Tieu chi dat: Tra loi day du checklist tu kiem.

### HB-PRAC-007 - Giao tiep ROS2 va conda

- Muc tieu: Xac minh ROS2 low code va conda high code giao tiep qua DDS.
- Dau vao: ROS2 Humble setup, conda env, [../network_setup_checklist.md](../network_setup_checklist.md).
- Dau ra: Conda nhan `lowstate`, ROS2 nhan `/lowcmd`.
- Tieu chi dat: Conda in IMU (rpy) va ROS2 thay `/lowcmd`.

### HB-PRAC-008 - Mo phong state/control

- Muc tieu: Mo phong robot gui state va controller gui cmd qua 2 terminal.
- Dau vao: [../../sim/unitree_r1_robot_sim.py](../../sim/unitree_r1_robot_sim.py) va [../../sim/unitree_r1_controller_sim.py](../../sim/unitree_r1_controller_sim.py).
- Dau ra: In ra state/imu/cmd theo thoi gian.
- Tieu chi dat: Ca 2 terminal in du lieu lien tuc trong suot `--duration`.

### HB-PRAC-004 - Ghi log buoi test

- Muc tieu: Ghi log de truy vet lai buoi test.
- Dau vao: [../../templates/test_log_template.md](../../templates/test_log_template.md).
- Dau ra: 1 log hoan chinh theo mau.
- Tieu chi dat: Log tra loi duoc 6 cau hoi o muc 5.

### HB-PRAC-005 - Cau noi third-party

- Muc tieu: Hieu ranh gioi giua core SDK, ROS 2 wrapper, Python binding.
- Dau vao: [../third-party_build.md](../third-party_build.md).
- Dau ra: 3 cau tom tat + 2 cau tra loi ve wrapper va binding.
- Tieu chi dat: Trinh bay duoc ly do can tach ranh gioi tich hop.

### HB-PRAC-006 - Tu kiem nhanh

- Muc tieu: Tu danh gia muc do hieu tai lieu.
- Dau vao: [README.md](README.md), [../SOP_v0.md](../SOP_v0.md).
- Dau ra: Tra loi >= 4/5 cau hoi.
- Tieu chi dat: Dat nguong tu kiem va biet phan doc lai neu thieu.

## 5. Ghi chu chung

- Uu tien thuc hanh tren simulation, sau do moi lam tren hardware.
- Luon ghi log va luu du lieu theo thu muc quy dinh.
- Neu gap bat thuong, doi chieu lai phan SOP va network/DDS.
