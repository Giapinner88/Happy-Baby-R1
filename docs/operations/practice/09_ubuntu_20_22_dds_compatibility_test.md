# Thực hành 09 - Kiểm tra DDS giữa Ubuntu 20.04 và 22.04
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-009
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này kiểm tra việc một máy Ubuntu 20.04 hiện tại có thể giao tiếp DDS với một máy Ubuntu 22.04 trước khi quyết định nâng cấp máy trạm vận hành. Mục tiêu là tách lỗi mô phỏng/physics/runtime khỏi lỗi truyền thông DDS.

Kết quả cần trả lời được ba câu hỏi:

1. Hai máy có thấy nhau ở tầng IP không.
2. ROS 2 Foxy trên Ubuntu 20.04 và ROS 2 Humble trên Ubuntu 22.04 có truyền được demo topic qua CycloneDDS không.
3. Unitree SDK2 Python DDS có truyền được message trực tiếp qua cùng interface không.

## 2. Nguyên tắc an toàn

- Chạy qua simulator hoặc HelloWorld trước, chưa chạy script low-level publish lệnh motor.
- Nếu nối vào robot thật, ưu tiên test read-only: chỉ đọc `lowstate`, không publish `lowcmd`.
- Không để cùng `ROS_DOMAIN_ID`/`DOMAIN_ID` với buổi vận hành thật khi đang thử nghiệm.
- Ghi lại OS, ROS distro, CycloneDDS version, interface, domain id và IP của cả hai máy.

## 3. Sơ đồ test đề xuất

```text
Máy A: Ubuntu 20.04 + ROS 2 Foxy + repo Happy-Baby-R1
  IP ví dụ: 192.168.123.100
  DDS interface ví dụ: enp3s0

Máy B: Ubuntu 22.04 + ROS 2 Humble + repo Happy-Baby-R1
  IP ví dụ: 192.168.123.101
  DDS interface ví dụ: enp2s0
```

Dùng một domain riêng cho bài test, ví dụ `42`.

## 4. Chuẩn bị chung trên cả hai máy

Kiểm tra interface và IP:

```bash
ip -br addr
ping -c 3 <IP_MAY_CON_LAI>
```

Tắt localhost-only và ép CycloneDDS:

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Nếu máy có nhiều card mạng, tạo XML tạm theo đúng tên interface của từng máy. Ví dụ trên máy A:

```bash
cat >/tmp/cyclonedds_hb_compat.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enp3s0"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI=file:///tmp/cyclonedds_hb_compat.xml
```

Trên máy B, đổi `enp3s0` thành interface của máy B.

## 5. Test A - ROS 2 demo pub/sub qua hai distro

### 5.1. Máy Ubuntu 20.04 / Foxy làm talker

```bash
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclonedds_hb_compat.xml
ros2 run demo_nodes_cpp talker
```

### 5.2. Máy Ubuntu 22.04 / Humble làm listener

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclonedds_hb_compat.xml
ros2 run demo_nodes_cpp listener
```

Đảo chiều thêm một lần: Humble talker, Foxy listener.

Tiêu chí đạt: listener nhận message liên tục ít nhất 60 giây ở cả hai chiều.

## 6. Test B - Unitree SDK2 Python DDS HelloWorld qua hai máy

Test này không phụ thuộc ROS graph. Nó kiểm tra đường DDS trực tiếp giống hướng Unitree SDK2 đang dùng.

Trên máy subscriber:

```bash
cd ~/Projects/Happy-Baby-R1
load_ml
python test/test_unitree_dds_helloworld_endpoint.py \
  --role subscriber \
  --domain-id 42 \
  --interface enp2s0 \
  --count 30
```

Trên máy publisher:

```bash
cd ~/Projects/Happy-Baby-R1
load_ml
python test/test_unitree_dds_helloworld_endpoint.py \
  --role publisher \
  --domain-id 42 \
  --interface enp3s0 \
  --count 30 \
  --period 1.0
```

Đảo chiều để chắc chắn multicast/discovery và unicast đều ổn với cả hai host.

Tiêu chí đạt: phía subscriber in `success` liên tục và nhận được ít nhất 25/30 message ở mỗi chiều.

## 7. Test C - Quan sát topic Unitree ở chế độ chỉ đọc

Chỉ làm bước này sau khi Test A và B đạt. Nếu đang dùng robot thật, không chạy script publish `lowcmd`.

Trên máy đang chạy robot/simulator:

```bash
export DOMAIN_ID=42
export INTERFACE=<interface_dds>
```

Trên máy còn lại, chỉ đọc trạng thái bằng tool phù hợp với setup hiện có:

```bash
ros2 topic list | grep -E "lowstate|lowcmd"
ros2 topic hz /lowstate --window 100
```

Nếu dùng Unitree SDK2 Python trực tiếp, tạo hoặc dùng script subscriber read-only cho model tương ứng. Không dùng các ví dụ `*_low_level_example.py` nếu chưa xác nhận chúng không gửi lệnh điều khiển.

## 8. Diễn giải kết quả

| Kết quả | Ý nghĩa | Hành động tiếp theo |
| --- | --- | --- |
| Test A đạt, Test B đạt | 20.04 và 22.04 giao tiếp DDS cơ bản ổn | Có thể tiếp tục test simulator/robot read-only |
| Test A lỗi, Test B đạt | ROS 2 graph/QoS/distro có vấn đề, DDS trực tiếp vẫn ổn | Ưu tiên Unitree SDK2 path hoặc kiểm tra message/QoS ROS |
| Test A đạt, Test B lỗi | ROS demo ổn nhưng Unitree SDK2 binding/interface lỗi | Kiểm tra `unitree_sdk2_python`, interface name, domain id |
| Cả hai lỗi | Lỗi mạng/interface/firewall/multicast | Sửa IP, firewall, XML CycloneDDS trước |

Lưu ý: ROS 2 Foxy và Humble khác distro nên không nên xem test demo ROS 2 là cam kết tương thích hoàn toàn cho mọi custom message. Với luồng Unitree, tín hiệu quan trọng hơn là SDK2 DDS direct và read-only `lowstate` chạy ổn trên cùng domain/interface.

## 9. Ghi log

Dùng [../../templates/test_log_template.md](../../templates/test_log_template.md) và ghi tối thiểu:

- OS + kernel của mỗi máy: `lsb_release -a`, `uname -a`.
- ROS distro và RMW: `printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID ROS_LOCALHOST_ONLY`.
- Interface/IP: `ip -br addr`.
- Lệnh đã chạy ở từng máy.
- Kết quả từng test: đạt/lỗi, số message nhận được, lỗi terminal nếu có.

## 10. Tài liệu liên quan

- [../network_setup_checklist.md](../network_setup_checklist.md)
- [../dds_implementation.md](../dds_implementation.md)
- [07_ros2_conda_communication_test.md](07_ros2_conda_communication_test.md)
- [08_state_control_sim.md](08_state_control_sim.md)
