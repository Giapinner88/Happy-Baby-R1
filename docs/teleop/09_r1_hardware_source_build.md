# 09 — R1 Hardware Source Build and Command Path

## 1. Mục tiêu và phạm vi

Tài liệu này nối tiếp `08_development_baseline.md` và mô tả cách dựng **mã nguồn điều khiển phần cứng R1** từ source chính thức của Unitree, sau đó đặt nó vào đúng boundary của pipeline teleoperation hiện tại.

Phạm vi ở đây là:

```text
application source
→ Unitree SDK2
→ CycloneDDS
→ R1 DDS topics
→ motion layer / motor command path
```

Không gọi phần này là firmware flashing.

Hiện các repository public được dùng trong baseline cho phép:
- build `unitree_sdk2`;
- build R1 low-level examples;
- build R1 high-level arm example;
- build/install Python SDK;
- publish/subscribe DDS với robot.

Tài liệu này **không xác nhận** có public source/build procedure cho firmware của motor-controller/MCU bên trong R1.

Do đó thuật ngữ dùng trong project:

```text
hardware source build
=
build host-side R1 controller / DDS command writer
```

không phải:

```text
MCU firmware build + flash
```

---

## 2. Quan hệ với các tài liệu trước

Pipeline tổng thể đã được định nghĩa:

```text
Quest 3
→ frame mapping
→ R1-A5 IK
→ joint trajectory
→ simulation
→ hardware boundary
```

`07_hardware_boundary.md` quy định:

```text
Quest/Vuer
→ IK on workstation
→ JSONL/SSH
→ loopback UDP sidecar
→ hb_high_level
→ sole rt/lowcmd publisher
```

và:

```text
hb_high_level = DDS motor writer duy nhất
```

Tài liệu `09` không thay đổi rule này.

Mục tiêu của việc build source Unitree là:

1. có một **vendor-grounded executable** để kiểm tra robot communication;
2. hiểu chính xác cách Unitree tạo `LowCmd`, đọc `LowState`, dùng `mode_machine`, CRC và DDS;
3. hiểu đường high-level `rt/arm_sdk`;
4. tách lỗi network/DDS khỏi lỗi mapper/IK;
5. sau khi baseline chạy đúng mới port phần cần thiết vào `hb_high_level`.

Không được lấy một example Unitree và chạy đồng thời với `hb_high_level` khi cả hai cùng publish `rt/lowcmd`.

---

## 3. Upstream source baseline

### 3.1 C++ SDK

Repository:

```text
https://github.com/unitreerobotics/unitree_sdk2
```

Các source R1 chính:

```text
example/r1/low_level/r1A_wrist_swing_example.cpp
example/r1/low_level/r1_ankle_swing_example.cpp
example/r1/high_level/r1_arm_sdk_dds_example.cpp
include/unitree/dds_wrapper/robots/r1/
```

Đối với upper-body R1-A5, source low-level sát nhất là:

```text
r1A_wrist_swing_example.cpp
```

Source high-level chính:

```text
r1_arm_sdk_dds_example.cpp
```

### 3.2 Python SDK

Repository:

```text
https://github.com/unitreerobotics/unitree_sdk2_python
```

Source R1:

```text
example/r1/low_level/r1_low_level_example.py
```

Python example phù hợp để:
- kiểm tra nhanh NIC;
- kiểm tra DDS discovery;
- đọc `rt/lowstate`;
- xác nhận command format;
- test `mode_machine`;
- test CRC;
- cô lập lỗi trước khi build C++ controller chính.

### 3.3 XR reference

Repository:

```text
https://github.com/unitreerobotics/xr_teleoperate
```

Project hiện đã có vendor tree pinned riêng cho XR/IK.

`xr_teleoperate` là reference cho:

```text
Quest pose
→ retargeting
→ R1_A5 IK
→ hardware command
```

nhưng hardware actuation trong project vẫn phải tuân theo boundary ở `07`.

---

## 4. Build `unitree_sdk2` từ source

Official SDK baseline ghi môi trường prebuild:

```text
Ubuntu 20.04 LTS
gcc 9.4.0
CMake >= 3.10
aarch64 / x86_64
```

Project có thể thử trên môi trường mới hơn, nhưng đó phải được coi là project compatibility test, không phải vendor-certified baseline.

### 4.1 Dependencies

Trên Ubuntu:

```bash
sudo apt update

sudo apt install -y \
    cmake \
    g++ \
    build-essential \
    libyaml-cpp-dev \
    libeigen3-dev \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev
```

### 4.2 Clone

Khuyến nghị không sửa trực tiếp clone upstream.

```bash
mkdir -p ~/unitree_ws/vendor
cd ~/unitree_ws/vendor

git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
```

Ghi revision trước khi test:

```bash
git rev-parse HEAD
git status
```

Project evidence nên lưu:

```text
repository
commit SHA
build date
host OS
architecture
compiler
CMake version
```

Ví dụ:

```bash
uname -a
uname -m
gcc --version
cmake --version
git rev-parse HEAD
```

### 4.3 Build toàn bộ SDK examples

```bash
cd ~/unitree_ws/vendor/unitree_sdk2

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$(nproc)"
```

SDK đặt runtime binaries trong:

```text
build/bin/
```

Do top-level CMake có:

```text
BUILD_EXAMPLES = ON
```

và `example/CMakeLists.txt` thêm:

```text
add_subdirectory(r1)
```

nên R1 examples được build cùng project.

Kiểm tra:

```bash
find build/bin -maxdepth 1 -type f | sort | grep r1
```

Expected relevant binaries gồm ít nhất:

```text
r1A_wrist_swing_example
r1_ankle_swing_example
r1_arm_sdk_dds_example
```

Tên binary thực tế phải được xác nhận từ build output của revision đang dùng.

### 4.4 Chỉ build target cần thiết

Sau lần configure đầu tiên có thể build từng target:

```bash
cmake --build build --target r1A_wrist_swing_example -j"$(nproc)"
```

hoặc:

```bash
cmake --build build --target r1_arm_sdk_dds_example -j"$(nproc)"
```

Cách này nên dùng khi đang sửa controller vì giảm compile time.

---

## 5. Install SDK cho project riêng

Nếu muốn `hb_high_level` hoặc controller riêng dùng SDK qua `find_package`, official SDK hỗ trợ install.

Khuyến nghị prefix:

```text
/opt/unitree_robotics
```

Build/install:

```bash
cd ~/unitree_ws/vendor/unitree_sdk2

cmake -S . -B build-install \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics

cmake --build build-install -j"$(nproc)"

sudo cmake --install build-install
```

Nếu project không tìm được SDK:

```bash
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
```

Sau đó project riêng có thể dùng:

```cmake
find_package(unitree_sdk2 REQUIRED)

target_link_libraries(
    hb_high_level
    PRIVATE
    unitree_sdk2
)
```

Không copy thủ công header/library từ build tree sang project nếu chưa có lý do cụ thể.

---

## 6. Low-level R1 command path

### 6.1 DDS topics

Official R1 low-level C++ example dùng:

```text
command:       rt/lowcmd
state:         rt/lowstate
torso IMU:     rt/secondary_imu
```

Đường tín hiệu:

```text
R1
 │
 ├── rt/lowstate ─────────────► controller
 │                              │
 │                              ├─ q
 │                              ├─ dq
 │                              ├─ mode_machine
 │                              └─ other state
 │
 ◄── rt/lowcmd ◄─────────────── controller
                                │
                                ├─ q_target
                                ├─ dq_target
                                ├─ kp
                                ├─ kd
                                ├─ tau_ff
                                ├─ mode_pr
                                ├─ mode_machine
                                └─ CRC
```

Low-level nghĩa là application trực tiếp tạo motor command message.

Đây là path có quyền điều khiển mạnh nhất và do đó cũng có risk cao nhất.

### 6.2 R1-A5 upper-body joint mapping

Official `r1A_wrist_swing_example.cpp` dùng 12 upper-body channels:

```text
logical index                  IDL slot

LeftShoulderPitch       0  →   15
LeftShoulderRoll        1  →   16
LeftShoulderYaw         2  →   17
LeftElbow               3  →   18
LeftWristRoll           4  →   19

RightShoulderPitch      5  →   22
RightShoulderRoll       6  →   23
RightShoulderYaw        7  →   24
RightElbow              8  →   25
RightWristRoll          9  →   26

HeadPitch              10  →   29
HeadYaw                11  →   30
```

Mapping này khớp hardware boundary hiện tại ở `07`.

Do đó application không nên giả định:

```text
logical joint index == motor_cmd[] index
```

Mà phải dùng explicit mapping:

```text
project joint order
→ Unitree IDL slot
```

### 6.3 Control period

Official R1 low-level example đặt:

```text
control_dt = 2 ms
```

tương ứng nominal:

$$
f_c = 500\ \text{Hz}.
$$

Điều này là reference của vendor example, không đồng nghĩa pipeline Quest → IK hiện tại phải giải IK ở 500 Hz.

Project có thể tách:

```text
XR / IK rate
      ↓
latest desired q
      ↓
hardware servo loop
```

Ví dụ:

```text
Quest               ~30 Hz
IK                  measured available rate
command sidecar      latest-wins
hardware writer      fixed high-rate loop
```

Hardware loop phải hold/interpolate/rate-limit target theo method đã được chấp nhận, không gọi IK lại ở mỗi 2 ms nếu compute budget không cho phép.

### 6.4 Command fields

Một motor command về mặt controller có dạng:

$$
u_i =
\left(
q_{d,i},
\dot q_{d,i},
K_{p,i},
K_{d,i},
\tau_{ff,i}
\right).
$$

Torque-level interpretation điển hình của joint servo:

$$
\tau_i
\approx
K_{p,i}(q_{d,i}-q_i)
+
K_{d,i}(\dot q_{d,i}-\dot q_i)
+
\tau_{ff,i}.
$$

Đây là cách diễn giải control interface; exact embedded motor-controller implementation không được suy ra nếu vendor không public source tương ứng.

### 6.5 `mode_machine`

Low-state trả về machine mode của robot.

Official R1 examples lấy state này rồi đưa vào outgoing `LowCmd`.

Project không nên hard-code một giá trị tùy ý nếu chưa audit đúng operating mode.

Required diagnostic:

```text
receive rt/lowstate
→ print mode_machine
→ verify expected state
→ only then enable command
```

Hardware pilot hiện tại ở `07` đang ghi assumption:

```text
mode_machine = 1
```

Assumption này phải được runtime verify trước khi coi một run là valid evidence.

### 6.6 CRC

Low-level path cần tạo CRC đúng trên outgoing message.

Conceptual sequence:

```text
fill LowCmd
→ set all relevant fields
→ compute CRC last
→ publish
```

Không sửa field sau khi đã tính CRC.

---

## 7. Build và chạy low-level vendor baseline

### 7.1 Xác định NIC nối robot

Không đoán tên interface.

```bash
ip -br link
ip -br addr
```

Ví dụ có thể thấy:

```text
enp3s0
enp4s0
eth0
```

Gọi:

```text
<R1_NIC>
```

cho interface thực sự nối R1.

### 7.2 Build

```bash
cd ~/unitree_ws/vendor/unitree_sdk2

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build \
    --target r1A_wrist_swing_example \
    -j"$(nproc)"
```

### 7.3 Trước khi execute

Low-level example có thể làm robot chuyển động.

Không dùng test movement nếu chưa có:
- robot mechanically secured;
- vùng xung quanh clear;
- operator E-stop;
- known initial configuration;
- đúng operating mode;
- không có publisher khác trên `rt/lowcmd`.

Đặc biệt:

```text
STOP hb_high_level
```

trước khi chạy vendor low-level writer.

Rule:

```text
exactly one rt/lowcmd owner
```

tại mọi thời điểm.

### 7.4 Run

Vendor C++ example nhận network interface ở command line:

```bash
./build/bin/r1A_wrist_swing_example <R1_NIC>
```

Ví dụ:

```bash
./build/bin/r1A_wrist_swing_example enp3s0
```

Không coi việc executable khởi động được là communication success.

Phải quan sát ít nhất:

```text
DDS state received?
mode_machine valid?
state updates continuously?
command path active?
robot response expected?
```

---

## 8. Python low-level baseline

Python SDK là cách nhanh hơn để tách build issue khỏi DDS issue.

### 8.1 Install

```bash
cd ~/unitree_ws/vendor

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

cd unitree_sdk2_python

python3 -m pip install -e .
```

Official Python SDK yêu cầu:

```text
Python >= 3.8
cyclonedds == 0.10.2
numpy
opencv-python
```

Nếu install báo không tìm thấy CycloneDDS, build CycloneDDS 0.10.x rồi đặt `CYCLONEDDS_HOME` theo hướng dẫn upstream.

Không cài đè vào Isaac environment nếu dependency conflict.

Nên dùng environment riêng, ví dụ:

```text
unitree_hw_env
```

### 8.2 Run R1 example

```bash
python3 example/r1/low_level/r1_low_level_example.py <R1_NIC>
```

Ví dụ:

```bash
python3 example/r1/low_level/r1_low_level_example.py enp3s0
```

Python example cũng là active low-level controller.

Do đó vẫn áp dụng:

```text
one rt/lowcmd publisher only
```

Không chạy song song với `hb_high_level`.

---

## 9. High-level R1 arm path

### 9.1 Topic

Official R1 high-level arm example publish:

```text
rt/arm_sdk
```

và subscribe robot joint feedback qua:

```text
rt/lowstate
```

High-level path:

```text
controller
   │
   ├── read rt/lowstate
   │
   └── write rt/arm_sdk
              ↓
       Unitree motion layer
              ↓
          arm motion
```

Khác với low-level:

```text
controller
   ↓
rt/lowcmd
   ↓
motor command layer
```

### 9.2 Ý nghĩa đối với project

High-level path hữu ích để trả lời câu hỏi:

```text
robot communication có hoạt động không?
```

mà chưa cần takeover toàn bộ low-level motor path.

Nếu:

```text
rt/lowstate works
rt/arm_sdk works
rt/lowcmd path fails
```

thì lỗi có khả năng nằm ở:
- low-level mode ownership;
- motion switching;
- `LowCmd` construction;
- `mode_machine`;
- CRC;
- competing publisher.

Nếu cả `rt/arm_sdk` và `rt/lowcmd` đều không hoạt động nhưng state vẫn đọc được thì cần kiểm tra command-side permissions/mode/service.

### 9.3 Build

```bash
cd ~/unitree_ws/vendor/unitree_sdk2

cmake --build build \
    --target r1_arm_sdk_dds_example \
    -j"$(nproc)"
```

Run syntax phải đọc từ chính revision đang dùng trước khi execute:

```bash
./build/bin/r1_arm_sdk_dds_example ...
```

Không hard-code CLI arguments trong project docs nếu source revision thay đổi.

### 9.4 Enable → move → release

High-level example cung cấp controller flow dạng:

```text
read current q
→ enable
→ initialize target from current q
→ move toward desired q
→ release
```

Đây là pattern an toàn hơn so với:

```text
enable
→ instantly command arbitrary q_target
```

Project nên giữ invariant:

$$
q_d(t_0) \approx q(t_0)
$$

khi takeover control.

---

## 10. Diagnostic ladder cho communication

Không debug Quest, mapper, IK và hardware cùng lúc.

Dùng từng tầng.

### Stage H0 — Build validity

```text
source clone
→ configure
→ compile
→ executable exists
```

Pass criteria:

```text
R1 target builds without local source modification
```

### Stage H1 — Network interface

```bash
ip -br addr
```

Xác nhận:
- đúng physical NIC;
- link up;
- host nằm đúng robot subnet;
- route không đi nhầm Wi-Fi/VPN/interface khác.

### Stage H2 — DDS receive only

Mục tiêu:

```text
rt/lowstate
```

phải update liên tục.

Log:

```text
timestamp
sequence if available
q
dq
mode_machine
state age
```

Nếu H2 fail:

```text
DO NOT debug IK
DO NOT debug LowCmd
```

Lỗi nằm trước command generation:

```text
NIC
→ subnet
→ DDS discovery/configuration
→ robot communication
```

### Stage H3 — High-level arm command

Dùng official:

```text
r1_arm_sdk_dds_example
```

Mục tiêu:

```text
state receive OK
+
high-level command accepted
```

### Stage H4 — Low-level isolated vendor command

Dùng official:

```text
r1A_wrist_swing_example
```

với robot secured.

Mục tiêu:

```text
vendor LowCmd path works
```

### Stage H5 — Project hardware writer

Chỉ sau H4:

```text
vendor low-level executable
        ↓ replace
hb_high_level
```

Giữ:
- same NIC;
- same DDS domain assumptions;
- same topic names;
- same IDL slot mapping;
- same `mode_machine` handling;
- same CRC semantics.

### Stage H6 — Sidecar injection

Sau khi `hb_high_level` standalone ổn:

```text
fixed q target
→ UTL1
→ UDP 127.0.0.1:5560
→ hb_high_level
→ rt/lowcmd
```

### Stage H7 — IK target

Sau H6:

```text
replayed Quest pose
→ mapper
→ IK
→ q_d
→ sidecar
→ hb_high_level
```

### Stage H8 — Live Quest

Cuối cùng mới:

```text
Quest live
→ full pipeline
→ R1
```

---

## 11. Integration với `hb_high_level`

Project architecture hiện tại:

```text
run_r1_quest3_hardware_targets.py
        │
        │ JSONL / target stream
        ▼
high_level_sidecar.py
        │
        │ UTL1 / UDP
        ▼
127.0.0.1:5560
        │
        ▼
hb_high_level
        │
        │ sole LowCmd construction
        ▼
rt/lowcmd
        │
        ▼
R1
```

Sau khi audit vendor example, phần cần port vào `hb_high_level` nên giới hạn ở hardware adapter:

```text
DDS initialization
LowState subscriber
state buffer
mode_machine acquisition
R1 IDL mapping
LowCmd packing
CRC
fixed-rate publish
shutdown behavior
```

Không đưa vào hardware writer:
- Quest transforms;
- IK;
- retargeting;
- XR validity logic.

Boundary phải giữ:

```text
teleop semantics
        │
        ▼
q_des
──────── HARDWARE BOUNDARY ────────
        │
        ▼
R1 command transport
```

---

## 12. Suggested internal source structure

Để tránh một file hardware writer quá lớn:

```text
hardware/r1/
├── CMakeLists.txt
├── include/
│   └── r1/
│       ├── joint_map.hpp
│       ├── lowcmd_writer.hpp
│       ├── lowstate_reader.hpp
│       └── safety_gate.hpp
└── src/
    ├── hb_high_level.cpp
    ├── lowcmd_writer.cpp
    ├── lowstate_reader.cpp
    └── safety_gate.cpp
```

Responsibility:

```text
joint_map.hpp
    project joint order ↔ Unitree IDL slot

lowstate_reader
    rt/lowstate → immutable latest state

lowcmd_writer
    q_des + gains + mode → LowCmd → CRC → rt/lowcmd

safety_gate
    enable
    command age
    joint envelope
    initial anchor
    mode validation

hb_high_level
    orchestration only
```

Không bắt buộc refactor ngay; đây là target structure sau khi baseline communication ổn.

---

## 13. Hardware safety gates bắt buộc cho project path

Vendor example là reference communication, không phải project safety policy.

Project writer phải tiếp tục enforce `07`:

### 13.1 Initial anchor

$$
q_{0}=q(t_{\text{enable}}).
$$

### 13.2 Pilot envelope

$$
q_{des,i}
=
q_{0,i}
+
\operatorname{clamp}
\left(
s_i-s_{0,i},
-0.15,
+0.15
\right).
$$

### 13.3 Stale command

Nếu command age vượt threshold:

```text
do not extrapolate Quest motion
do not invent new target
```

Fallback behavior phải được định nghĩa riêng và audit trên hardware writer.

### 13.4 Single publisher

Invariant:

$$
\boxed{
N_{\text{publisher}}(rt/lowcmd)=1
}
$$

Không chạy đồng thời:
- Python R1 low-level example;
- C++ R1 low-level example;
- `hb_high_level`;
- một node debug khác cùng publish `rt/lowcmd`.

---

## 14. Build record cho mỗi hardware run

Mỗi run nên lưu tối thiểu:

```yaml
hardware_source:
  repository: unitreerobotics/unitree_sdk2
  commit: <sha>
  build_type: Release
  compiler: <gcc version>
  cmake: <cmake version>
  architecture: <x86_64/aarch64>

robot:
  model: R1-A5
  network_interface: <nic>
  mode_machine: <runtime value>

command_path:
  level: low_level | high_level
  topic: rt/lowcmd | rt/arm_sdk
  owner: <process name>

project:
  hb_high_level_commit: <sha>
  sidecar_commit: <sha>
  ik_method: <method id>
```

Điều này cần thiết vì:
- Unitree upstream có thể thay đổi;
- high-level R1 support mới hơn các path cũ;
- DDS behavior phụ thuộc source revision và runtime environment;
- evidence hardware không nên tách khỏi binary đã build.

---

## 15. Recommended development baseline sau `08`

Thứ tự triển khai:

```text
09.1
build untouched unitree_sdk2
        ↓
09.2
receive R1 rt/lowstate
        ↓
09.3
run isolated official high-level arm example
        ↓
09.4
run isolated official low-level upper-body example
        ↓
09.5
compare vendor behavior with hb_high_level
        ↓
09.6
fixed target through UTL1 sidecar
        ↓
09.7
replay q trajectory
        ↓
09.8
replay Quest pose → IK → q trajectory
        ↓
09.9
live Quest hardware pilot
```

Không chuyển sang bước sau nếu bước trước chưa có evidence rõ.

---

## 16. Acceptance criteria

### Build accepted

```text
official SDK revision recorded
clean build succeeds
R1 binaries generated
```

### Communication accepted

```text
rt/lowstate received continuously
runtime mode recorded
no unexplained disconnect
```

### High-level accepted

```text
official R1 high-level example can command expected upper-body motion
```

### Low-level vendor baseline accepted

```text
official R1 low-level example can command expected secured test motion
```

### Project writer accepted

```text
hb_high_level reproduces required communication behavior
without a second rt/lowcmd publisher
```

### Full teleop hardware accepted

Chưa được claim chỉ từ communication success.

Full teleop còn cần evidence từ:
- frame correctness;
- IK correctness;
- tracking error;
- command age;
- achieved rate;
- joint saturation;
- safety envelope.

---

## 17. Source references

Official C++ SDK:

```text
https://github.com/unitreerobotics/unitree_sdk2
```

R1 low-level upper-body:

```text
https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/r1/low_level/r1A_wrist_swing_example.cpp
```

R1 low-level whole-R1 / ankle reference:

```text
https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/r1/low_level/r1_ankle_swing_example.cpp
```

R1 high-level arm:

```text
https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/r1/high_level/r1_arm_sdk_dds_example.cpp
```

Official Python SDK:

```text
https://github.com/unitreerobotics/unitree_sdk2_python
```

R1 Python low-level:

```text
https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/r1/low_level/r1_low_level_example.py
```

XR reference:

```text
https://github.com/unitreerobotics/xr_teleoperate
```

---

## 18. Kết luận

Hardware implementation baseline của project nên được hiểu là:

$$
\boxed{
q_d
\rightarrow
\text{project safety gate}
\rightarrow
\text{Unitree-compatible LowCmd}
\rightarrow
\text{DDS}
\rightarrow
R1
}
$$

Hai command paths cần phân biệt rõ:

```text
HIGH LEVEL
q_d
→ rt/arm_sdk
→ Unitree motion layer
→ R1 upper body
```

và:

```text
LOW LEVEL
q_d + dq_d + Kp + Kd + tau_ff
→ LowCmd + CRC
→ rt/lowcmd
→ R1 motor command layer
```

Đối với pipeline teleoperation hiện tại, final accepted hardware path vẫn là:

```text
Quest
→ mapper
→ IK
→ rate/safety limit
→ sidecar
→ hb_high_level
→ sole rt/lowcmd
→ R1
```

Official Unitree examples được dùng để **build, audit và validate communication semantics**, không thay thế evidence boundary và safety policy đã định nghĩa trong `07`.
