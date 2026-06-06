# Hướng dẫn chạy test DDS qua WiFi giữa Ubuntu 20.04 và Ubuntu 22.04

Mục tiêu của test này là kiểm tra máy trạm Ubuntu 20.04 và laptop Ubuntu 22.04 có nhìn thấy nhau qua DDS khi cùng dùng một mạng WiFi. Đây là bài test mạng/DDS độc lập, chưa phải hướng dẫn kết nối robot.

## 1. File cần dùng

- Máy trạm Ubuntu 20.04: `test/dds_wifi_workstation_ubuntu20.sh`
- Laptop Ubuntu 22.04: `test/dds_wifi_laptop_ubuntu22.sh`
- Hướng dẫn này: `docs/operations/practice/dds_wifi_ubuntu20_22_run_guide.md`

## 2. Điều kiện trước khi chạy

Trên máy trạm Ubuntu 20.04:

- Đã cài ROS 2 Foxy.
- Đã cài CycloneDDS RMW: `ros-foxy-rmw-cyclonedds-cpp`.
- Máy đang vào cùng mạng WiFi với laptop.

Trên laptop Ubuntu 22.04:

- Đã cài ROS 2 Humble.
- Đã cài CycloneDDS RMW: `ros-humble-rmw-cyclonedds-cpp`.
- Laptop đang vào cùng mạng WiFi với máy trạm.

## 3. Kiểm tra IP hai máy

Chạy trên cả hai máy:

```bash
ip -br addr
ip route
```

Ghi lại IP WiFi của từng máy. Ví dụ:

```text
Máy trạm Ubuntu 20.04: 192.168.1.20
Laptop Ubuntu 22.04:   192.168.1.21
```

Kiểm tra ping hai chiều:

```bash
ping -c 3 <IP_MAY_CON_LAI>
```

Nếu ping không được, sửa kết nối WiFi trước. DDS chưa cần kiểm tra ở bước này.

## 4. Chạy test

Mở terminal trên máy trạm Ubuntu 20.04:

```bash
cd ~/Projects/Happy-Baby-R1
bash test/dds_wifi_workstation_ubuntu20.sh
```

Mở terminal trên laptop Ubuntu 22.04:

```bash
cd ~/Projects/Happy-Baby-R1
bash test/dds_wifi_laptop_ubuntu22.sh
```

Hai script nên được chạy gần như đồng thời. Mỗi bên sẽ publish một topic và subscribe topic của bên kia:

```text
Ubuntu 20.04 publish: /hb/dds_wifi/ubuntu20_to_ubuntu22
Ubuntu 20.04 listen:  /hb/dds_wifi/ubuntu22_to_ubuntu20

Ubuntu 22.04 publish: /hb/dds_wifi/ubuntu22_to_ubuntu20
Ubuntu 22.04 listen:  /hb/dds_wifi/ubuntu20_to_ubuntu22
```

Mặc định test chạy trong 60 giây, dùng `ROS_DOMAIN_ID=42` và tự chọn interface theo default route.

## 5. Khi cần chỉ định interface WiFi

Nếu máy có nhiều card mạng hoặc script chọn nhầm interface, xem tên WiFi bằng:

```bash
ip -br addr
```

Sau đó chạy lại với `WIFI_INTERFACE`.

Ví dụ trên máy trạm:

```bash
WIFI_INTERFACE=wlp4s0 bash test/dds_wifi_workstation_ubuntu20.sh
```

Ví dụ trên laptop:

```bash
WIFI_INTERFACE=wlp2s0 bash test/dds_wifi_laptop_ubuntu22.sh
```

## 6. Khi cần đổi domain hoặc thời lượng

Đổi domain DDS:

```bash
ROS_DOMAIN_ID=55 bash test/dds_wifi_workstation_ubuntu20.sh
ROS_DOMAIN_ID=55 bash test/dds_wifi_laptop_ubuntu22.sh
```

Chạy lâu hơn, ví dụ 5 phút:

```bash
DURATION_SEC=300 bash test/dds_wifi_workstation_ubuntu20.sh
DURATION_SEC=300 bash test/dds_wifi_laptop_ubuntu22.sh
```

Hai máy phải dùng cùng `ROS_DOMAIN_ID`.

## 7. Kết quả đạt

Trên cả hai máy cần thấy log dạng:

```text
TX 1: from=...
RX 1: from=...
SUMMARY role=... sent=... received=...
```

Tiêu chí đạt:

- Cả hai bên đều có `received > 0`.
- Trong test 60 giây, mỗi bên nên nhận gần 60 message nếu WiFi ổn.
- Nếu có mất vài message nhưng hai bên vẫn nhận đều, DDS discovery và data path cơ bản đã hoạt động.

## 8. Nếu không nhận được message

Kiểm tra theo thứ tự:

1. Hai máy ping được nhau chưa.
2. Hai máy có cùng `ROS_DOMAIN_ID` không.
3. `ROS_LOCALHOST_ONLY` có bằng `0` không.
4. Đã dùng `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` chưa.
5. Script có chọn đúng WiFi interface không.
6. Router WiFi có bật client isolation/AP isolation không. Nếu có, tắt chế độ này hoặc dùng hotspot/router khác.
7. Firewall có chặn UDP multicast không. Có thể kiểm tra nhanh bằng cách tạm tắt firewall trong một phiên test có kiểm soát.

## 9. Ghi chú để nâng thành hướng dẫn robot

Khi đổi bài test này thành hướng dẫn kết nối robot:

- Thay topic heartbeat bằng topic/read-only state của robot trước.
- Không publish lệnh điều khiển cho tới khi đã xác nhận read-only ổn định.
- Chuyển từ WiFi sang Ethernet trực tiếp nếu cần độ trễ thấp và ổn định hơn.
- Ghi rõ IP tĩnh, interface robot, domain id, topic state/cmd và bước dừng khẩn cấp.
