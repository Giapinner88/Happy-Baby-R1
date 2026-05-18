# CycloneDDS Middleware Implementation & Performance Verification
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-006
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Draft / Pending Review

Tài liệu này tập trung vào việc thiết lập, tinh chỉnh và xác thực lớp trung gian CycloneDDS trong hệ sinh thái ROS 2 Humble. Mục tiêu cốt lõi là đảm bảo tính ổn định của luồng dữ liệu (Data Pipeline) giữa máy trạm và robot Unitree R1, triệt tiêu hiện tượng mất gói tin (Packet loss) và giảm thiểu độ trễ biến thiên (Jitter) trong các tác vụ điều khiển thời gian thực.

## 1. Cơ sở lý thuyết và Lựa chọn Middleware

Trong kiến trúc của Unitree R1, việc lựa chọn CycloneDDS thay vì các bộ triển khai khác (như FastDDS) dựa trên khả năng xử lý thông điệp nhẹ và tuân thủ chặt chẽ tiêu chuẩn RTPS (Real-Time Publish-Subscribe). 

Bản chất của việc điều khiển Robot hình người là duy trì sự hội tụ của các thuật toán cân bằng. Nếu gọi $\tau_{latency}$ là độ trễ truyền dẫn và $T_{control}$ là chu kỳ điều khiển ($1ms$ cho Low-level), hệ thống phải thỏa mãn điều kiện:
$$\tau_{latency} + \tau_{compute} < T_{control}$$

CycloneDDS giúp tối ưu hóa $\tau_{latency}$ bằng cách sử dụng cơ chế Zero-copy (nếu có thể) và quản lý bộ nhớ đệm (Buffer) hiệu quả ở tầng User-space.

---

## 2. Cấu hình CycloneDDS nâng cao

Ngoài việc xác định Interface mạng, chúng ta cần cấu hình các tham số nội tại để phù hợp với đặc thù của Robot hình người (dữ liệu truyền tải liên tục nhưng kích thước gói tin nhỏ).

### 2.1. Thiết lập biến môi trường (Environment Variables)
Để hệ thống nhận diện đúng tệp cấu hình tùy chỉnh khi test qua mang, bien `CYCLONEDDS_URI` nen duoc set trong terminal. Dieu nay da duoc tich hop vao alias `load_ros` trong `~/.zshrc`. Neu chi test local, co the `unset CYCLONEDDS_URI` de bo qua XML.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/Projects/Happy-Baby-R1/config/cyclonedds_config.xml
```

### 2.2. Giải thích các tham số trong tệp XML
Tệp `cyclonedds_config.xml` của dự án chứa các khối điều khiển quan trọng:
- `<NetworkInterfaceAddress>`: Ràng buộc DDS vào đúng card mạng Ethernet nối với R1 để tránh nhiễu từ Wifi.
- `<AllowMulticast>`: Thiết lập `true` để cho phép cơ chế tự động khám phá (Discovery) các Node trong mạng nội bộ.
- `<WatermarkPings>`: Thiết lập `false` để giảm thiểu các gói tin kiểm tra không cần thiết, tiết kiệm băng thông cho dữ liệu điều khiển chính.

*Luu y:* Neu CycloneDDS bao `unknown element` cho `WatermarkPings`, hay bo khoi file XML hoac cap nhat theo schema phu hop version hien tai.

---

## 3. Quy trình xác thực hoạt động của DDS (Verification)

Việc kiểm tra phải đi từ khả năng kết nối cơ bản đến hiệu suất truyền tin thực tế.

### Bước 3.1: Kiểm tra khả năng khám phá (Discovery Check)
Sử dụng công cụ `ros2 doctor` để quét toàn bộ mạng và phát hiện các xung đột cấu hình.
```bash
load_ros
ros2 doctor --report
```
**Yêu cầu:** Phần `Middleware` phải hiển thị `rmw_cyclonedds_cpp`. Nếu báo lỗi "Mismatched Domain ID", hãy kiểm tra xem biến `ROS_DOMAIN_ID` trên Host và Robot có trùng nhau không (mặc định là 0).

### Bước 3.2: Phân tích độ trễ và tần số (Performance Profiling)
Khi robot gửi dữ liệu trạng thái (State), chúng ta cần đo lường sự ổn định của tần số.
```bash
ros2 topic hz /unitree/lowstate --window 100
```
**Tiêu chuẩn xác thực:**
- **Average Rate:** Phải đạt xấp xỉ $1000\text{ Hz}$ (cho Low-level).
- **Min/Max Delta:** Sự chênh lệch giữa hai gói tin không được quá $0.2\text{ms}$. Nếu Delta lớn, hệ thống đang gặp hiện tượng Jitter do xung đột tài nguyên trên Host.

### Bước 3.3: Debug chuyên sâu (Trace Mode)
Trong trường hợp Node không nhìn thấy nhau dù đã chung mạng, kích hoạt chế độ log chi tiết của CycloneDDS:
```bash
export CYCLONEDDS_DEBUG=trace
ros2 run r1_bringup r1_low_level_node
```
Hệ thống sẽ tạo ra tệp log chi tiết các bước bắt tay (handshake) của UDP. Bạn cần tìm dòng `Selected interface: <IP_ADDRESS>` để xác nhận DDS đã chọn đúng card mạng Ethernet.

---

## 4. Troubleshooting (Xử lý sự cố thường gặp)

1. **Node không thấy nhau:** Thường do tường lửa (Firewall) của Ubuntu chặn cổng UDP. Chạy `sudo ufw disable` để kiểm tra nhanh.
2. **Tần số bản tin bị sụt giảm:** Kiểm tra xem có Node nào đang thực hiện các tác vụ nặng (như xử lý ảnh) trên cùng một Core CPU với Node truyền tin không. Khuyến khích sử dụng `taskset` để ghim Node DDS vào một Core riêng biệt.
3. **`rmw_create_node` fail sau khi set `CYCLONEDDS_URI`:** Thu `unset CYCLONEDDS_URI` de xac minh ROS 2 van chay, sau do cap nhat XML cho dung schema (neu log co `deprecated element`/`unknown element`).

## 5. Tài liệu liên quan

* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Cấu hình mạng tĩnh: [network_configuration_static_ethernet.md](network_configuration_static_ethernet.md)
* Giải thích kiến trúc mạng: [../architecture/network_dds_rationale.md](../architecture/network_dds_rationale.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)