# Checklist thiết lập mạng và DDS cho Unitree R1
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SOP-002
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Approved / Final

## 1. Kết nối vật lý

1. Sử dụng cáp Ethernet Cat6 trở lên để nối máy tính với robot R1.
2. Kiểm tra đèn Link/Act tại cả hai đầu cáp.

## 2. Cấu hình IP tĩnh

1. Mở phần cài đặt mạng trên Ubuntu.
2. Chọn interface Ethernet tương ứng và chuyển sang chế độ Manual.
3. Thiết lập các thông số chuẩn cho hệ sinh thái Unitree:
	* Address: `192.168.123.x`.
	* Netmask: `255.255.255.0`.
	* Gateway: để trống hoặc `192.168.123.1` nếu cần.
4. Nhấn Apply và tắt bật lại interface để nhận cấu hình mới.

## 3. Cấu hình CycloneDDS

1. Cài package `ros-humble-rmw-cyclonedds-cpp`.
2. Thiết lập biến môi trường trong `.bashrc` hoặc `.zshrc`:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/Projects/Happy-Baby-R1/config/cyclonedds_config.xml
```

3. Kiểm tra file cấu hình XML trong repo và xác nhận interface mạng đúng với máy đang dùng.

## 4. Kiểm tra kết nối

1. Chạy `ping 192.168.123.1` hoặc IP mặc định của robot để kiểm tra thông suốt.
2. Chạy demo pub/sub của ROS 2 để xác nhận giao tiếp nội bộ:

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

## 5. Kiểm thử Unitree SDK2

1. Di chuyển vào thư mục `unitree_sdk2_python`.
2. Chạy demo `python3 examples/helloworld.py`.
3. Chạy script đọc trạng thái robot để xác nhận dữ liệu IMU và joint trả về thành công.

## 6. Tài liệu liên quan

* Giải thích kiến trúc mạng: [../architecture/network_dds_rationale.md](../architecture/network_dds_rationale.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
* An toàn phần cứng: [../safety/hardware_safety_rules.md](../safety/hardware_safety_rules.md)
* An toàn phần mềm: [../safety/software_safety_rules.md](../safety/software_safety_rules.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)