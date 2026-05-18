# Network Configuration & Interface Verification Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-005
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Draft / Pending Review

Tài liệu này chuẩn hóa quy trình cấu hình mạng IP tĩnh (Static IP) và thiết lập lớp giao tiếp CycloneDDS để kết nối máy trạm (Host Machine) với robot Unitree R1 (Onboard Computer). Mục tiêu cốt lõi là thiết lập một đường truyền Real-time với độ trễ tối thiểu và không bị nhiễu sóng từ các card mạng khác.

## 1. Bản chất của hệ thống mạng phân tán (DDS)

Khác với các giao thức TCP truyền thống, ROS 2 Middleware sử dụng UDP/RTPS để đáp ứng thời gian thực. Theo thông số kỹ thuật, hệ thống R1 yêu cầu tần số điều khiển Low-level là $f = 1000\text{ Hz}$. Điều này đồng nghĩa với việc chu kỳ thời gian lý tưởng cho mỗi vòng lặp điều khiển là:

$$T_{cycle} = \frac{1}{f} = 1\text{ ms}$$

Để đảm bảo tổng thời gian xử lý và truyền dẫn qua lại (Round-trip Time) không vượt quá $T_{cycle}$, chúng ta phải loại bỏ giao thức cấp phát IP động (DHCP) và cấu hình Static IP trực tiếp trên Layer 3. Hai thiết bị (Host và Jetson Orin) phải nằm trên cùng một Subnet để các gói tin có thể đi thẳng tới đích mà không qua Router định tuyến (Routing hop).

---

## 2. Cấu hình Static IP trên Host (Ubuntu 22.04 LTS)

Robot Unitree R1 xuất xưởng với cấu hình mặc định là `192.168.123.164`. Do đó, Host Machine bắt buộc phải được gắn một IP tĩnh trong dải `/24` của mạng này.

### Bước 2.1: Xác định tên giao diện mạng
Cắm cáp Ethernet kết nối trực tiếp từ Host tới cổng mạng của R1. Xác định tên card mạng vật lý bằng lệnh:
```bash
ip link show
```
*(Giả sử kết quả trả về tên giao diện Ethernet của bạn là `eth0` hoặc một tên chuỗi tương đương do hệ thống đặt như `enp3s0` / `Wired connection 1`).*

### Bước 2.2: Cấu hình Static IP bằng Netplan (Phương pháp khuyến nghị)

**Phương pháp này sử dụng Netplan - công cụ mạng tiêu chuẩn trên Ubuntu 22.04 LTS.**

#### Cách A: Dùng file YAML từ repo (Nhanh & Chuẩn)
1. Xác định tên interface từ kết quả `ip link show` (ví dụ: `enp3s0`, `eth0`)
2. Copy template Netplan từ repo:
   ```bash
   # Chỉnh sửa tên interface trong file config/netplan_static_ethernet.yaml
   # Thay 'eth0' bằng interface name thực tế của bạn
   sudo cp config/netplan_static_ethernet.yaml /etc/netplan/99-static-ethernet.yaml
   ```
3. Chỉnh sửa file nếu cần:
   ```bash
   sudo nano /etc/netplan/99-static-ethernet.yaml
   # Thay đổi:
   # - 'eth0' -> tên interface thực tế (enp3s0, enp4s0, ...)
   # - '192.168.123.100' nếu muốn dùng IP khác
   ```
4. Áp dụng cấu hình:
   ```bash
   sudo netplan apply
   ```
5. Kiểm tra:
   ```bash
   ip addr show <interface_name>
   # Output sẽ hiển thị: inet 192.168.123.100/24
   ```

#### Cách B: Tạo file Netplan thủ công (Nếu không có file repo)
Tạo file `/etc/netplan/99-static-ethernet.yaml` với nội dung:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp3s0:  # Thay bằng tên interface thực tế
      dhcp4: false
      dhcp6: false
      addresses:
        - address: 192.168.123.100
          prefix: 24
```
Rồi chạy:
```bash
sudo netplan apply
```

### Bước 2.3: Phương pháp dự phòng - Dùng NetworkManager (nếu Netplan không khả dụng)
Thực thi chuỗi lệnh sau để ép cứng IP thành `192.168.123.100`:
```bash
sudo nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.123.100/24 \
    ipv4.method manual
sudo nmcli connection up "Wired connection 1"
```

---

## 3. Ràng buộc Interface cho CycloneDDS

Khi Host Machine có nhiều kết nối mạng (ví dụ: vừa cắm Ethernet nối với Robot, vừa bật Wifi nối Internet), CycloneDDS có thể phát gói tin Multicast sai cổng. Cần phải trói (bind) Middleware trực tiếp vào IP tĩnh vừa tạo.

Chỉnh sửa tệp cấu hình `cyclonedds_config.xml` của dự án (nằm tại thư mục `config/`):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="[https://cdds.io/config](https://cdds.io/config)" xmlns:xsi="[http://www.w3.org/2001/XMLSchema-instance](http://www.w3.org/2001/XMLSchema-instance)" xsi:schemaLocation="[https://cdds.io/config](https://cdds.io/config) [https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd)">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>192.168.123.100</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Internal>
            <WatermarkPings>false</WatermarkPings>
        </Internal>
    </Domain>
</CycloneDDS>
```

*Luu y:* Neu log CycloneDDS bao `unknown element` cho `WatermarkPings`, hay bo dong nay khoi file XML hoac cap nhat theo schema phu hop version hien tai.

---

## 4. Kịch bản kiểm thử (Verification Pipeline)

Quy trình xác thực được thực hiện theo nguyên tắc từ thấp lên cao (Bottom-up). Bất kỳ bài test nào thất bại ở bước dưới đều phải được giải quyết trước khi chuyển sang bước tiếp theo.

### Test 1: Xác thực Lớp Vật Lý & Định tuyến (ICMP Ping)
Kiểm tra xem hai máy tính đã nhận diện được phần cứng của nhau qua cáp Ethernet chưa.
```bash
ping -c 4 192.168.123.164
# Output mong đợi: 4 packets transmitted, 4 received, 0% packet loss.
# Thời gian trả về (time) phải liên tục < 1ms.
```

### Test 2: Xác thực Cơ chế Khám phá (DDS Multicast Discovery)
Mở hai Terminal.
* **Terminal 1 (SSH vào Jetson của R1):**
    ```bash
    ros2 multicast receive
    ```
* **Terminal 2 (Trên Host Machine, đã load môi trường ROS 2):**
    ```bash
    exec zsh
    load_ros
    ros2 multicast send
    ```
**Output mong đợi:** Terminal 1 hiển thị `Received from <IP Host>: 'Hello World!'`. Nếu không nhận được, cấu hình Multicast hoặc tường lửa (UFW) đang chặn giao tiếp UDP.

### Test 3: Xác thực Băng thông Bản tin (Topic Frequency)
Kiểm tra xem hệ thống có duy trì được tần số giao tiếp của các Controller hay không. Trên Host Machine:
```bash
load_ros
ros2 topic hz /unitree/highstate
# Output mong đợi: Tần số (average rate) phải duy trì ổn định ở mức ~500Hz.
```

## 5. Tài liệu liên quan

* Thiết lập mạng/DDS (checklist): [network_setup_checklist.md](network_setup_checklist.md)
* Triển khai CycloneDDS chi tiết: [dds_implementation.md](dds_implementation.md)
* Giải thích kiến trúc mạng: [../architecture/network_dds_rationale.md](../architecture/network_dds_rationale.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)