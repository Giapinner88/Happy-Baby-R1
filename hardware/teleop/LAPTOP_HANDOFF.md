# START HERE — bàn giao Quest 3 teleop cho Unitree R1

Tài liệu này giúp một người hoặc một AI mới mở bundle trên laptop khác hiểu
đúng phạm vi, kiến trúc, nguồn code và trạng thái bằng chứng trước khi chạy bất
kỳ lệnh nào.

## 1. Phạm vi hiện tại

Bundle phục vụ pilot **robot treo/cố định, 10 khớp tay + 2 khớp đầu**. Nó chưa
chứng minh robot an toàn khi đứng/chạy trên sàn, không điều khiển chân hoặc eo,
và không tự cấp quyền ghi motor. Mục tiêu trước mắt là tái lập đúng pipeline
đã dùng trong mô phỏng, rồi đóng các cổng an toàn trên robot thật.

“Giống trong sim” ở đây có nghĩa là dùng cùng:

- hệ trục và phép calibration Quest-to-robot;
- `assets/R1.urdf`, trục khớp, giới hạn hình học và thứ tự khớp;
- mapping pose, coupled IK và quy ước đơn vị m/rad/s;
- cấu hình T001/T007 và schema lệnh.

Nó **không** có nghĩa là sao chép mù giới hạn vận tốc/gia tốc mô phỏng sang
motor thật. Hardware thêm watchdog, session envelope, slew limit và quyền ghi
duy nhất; các giới hạn actuator cuối cùng vẫn phải được xác nhận trên đúng
robot R1-A5.

## 2. Thứ tự đọc đề nghị

1. File này.
2. `AGENTS.md` — mục tiêu nghiên cứu, quy tắc bằng chứng và an toàn của repo.
3. `README.md` — hai tầng Ubuntu 22.04 workstation / Ubuntu 20.04 robot.
4. `hardware/teleop/BUNDLE_MANIFEST.md` — file nào là authoritative và thông
   số runtime hiện tại.
5. `docs/teleop/01_system_architecture.md`,
   `docs/teleop/02_quest_to_robot_frames.md` và
   `docs/teleop/05_project_solvers_and_evidence.md` — pipeline, frame và IK.
6. `docs/operations/r1_quest3_teleop_sim.md` — cách dựng baseline mô phỏng.
7. `docs/operations/r1_quest3_teleop_hardware.md` và
   `hardware/teleop/docs/hardware_gate.md` — SOP và các gate chưa đóng.
8. `experiments/r1_teleop/quest3_sim_v1/experiment.md`, rồi T001 đến T007 —
   lịch sử câu hỏi, config, quan sát và giới hạn của bằng chứng.

Đừng đọc riêng một script rồi suy đoán kiến trúc. `PROVENANCE.txt`,
`WORKTREE.patch` và `SHA256SUMS` là hồ sơ nguồn chính xác của bundle.

## 3. Sơ đồ hệ thống

```text
Meta Quest 3 browser
  -> HTTPS/WSS Vuer :8012
  -> quest_bridge.py, R1TeleopCommand JSONL, 30 Hz
  -> run_r1_quest3_hardware_targets.py
       calibration + mapping + coupled URDF IK, arms_head
  -> JSONL qua SSH stdin
  -> high_level_sidecar.py trên robot
       read-only rt/lowstate + UDP loopback UTL1 :5560
  -> hardware/high_level TeleopReceiver
       sequence/freshness/session guards
  -> LowCmdSender, publisher rt/lowcmd duy nhất, 500 Hz
  -> 10 khớp tay + head pitch/yaw
```

Workstation chạy Ubuntu 22.04 và hai môi trường Conda tách biệt: môi trường
`tv` cho Vuer/Quest transport và `unitree_sim_env` cho mapping/IK/simulation.
Máy nhúng R1 chạy Ubuntu 20.04, ROS 2 Foxy, CycloneDDS và Unitree SDK. Các môi
trường này là dependency ngoài bundle, không được đóng gói dưới dạng binary.

## 4. Nguồn code và quyền sở hữu motor

Phần workstation lấy từ working tree hiện tại; commit và trạng thái dirty được
ghi trong `PROVENANCE.txt`, còn thay đổi chưa commit nằm trong
`WORKTREE.patch`. Phần sole-owner C++ được lấy từ revision `bb70a20` ở
`hardware/high_level/`, vì branch teleop hiện tại không có cây source đó.

Không thay bằng `Operation_Khanh/high_level/`: bản đó không có UTL1 receiver và
mapping/comment đầu không phải nguồn authoritative hiện tại. Trong kiến trúc
này chỉ `hardware/high_level` được tạo publisher `rt/lowcmd`. Sidecar teleop
không được tạo DDS command publisher; nó chỉ đọc state và gửi target qua
`127.0.0.1:5560`.

Thứ tự target 12 khớp:

```text
left_arm[5], right_arm[5], head_pitch, head_yaw
```

R1-A5 IDL slots: tay trái 15–19, tay phải 22–26, head pitch 29 và head yaw 30.

## 5. Thông số runtime đang mang theo

Nguồn authoritative là
`hardware/teleop/config/high_level_teleop_suspended.yaml`, được exporter chép
thành `hardware/high_level/config/tuning.yaml`.

| Thông số | Giá trị | Ý nghĩa |
| --- | ---: | --- |
| Quest source rate | 30 Hz | telemetry nguồn |
| IK dispatch hiện tại | 10 Hz | bottleneck đã đo; chưa đạt mục tiêu T007 20 Hz |
| Sidecar send rate | 100 Hz | transport local target |
| High-level motor loop | 500 Hz | sole-owner DDS loop |
| Mapping timeout | 0.50 s | giữ/dừng khi mất command |
| Sidecar timeout | 0.75 s | ngắt stream target |
| High-level freshness | 0.30 s | fail-closed cuối cùng |
| Session envelope | ±0.15 rad | quanh encoder anchor lúc bắt đầu |
| Final slew limit | 0.30 rad/s | giới hạn ở sole owner |
| Arm PD | Kp 40, Kd 2 | provisional, chỉ pilot treo |
| Head PD | Kp 15, Kd 1 | provisional, chỉ pilot treo |
| IK iterations/damping/max step | 40 / 0.02 / 0.12 rad | profile T007 |
| IK position/head tolerance | 0.002 m / 0.03 rad | profile T007 |

Các số `1.5 rad/s` và `4.0 rad/s^2` trong simulator chỉ là tuning mô phỏng,
không phải giới hạn actuator được phê duyệt.

## 6. Bundle có và không có gì

Có: source teleop, URDF, cấu hình editable, test, script dựng/chạy, tài liệu
kiến trúc/SOP/safety, định nghĩa thí nghiệm T001–T007 (không kèm thư mục run
lớn), evidence utilities, vendor reference tối thiểu, ONNX tối thiểu mà
high-level cần để preflight, hash và provenance.

Không có: certificate/private key, file `.env` thật, mật khẩu, toàn bộ Conda
environment, ROS/Unitree SDK đã cài, log robot thật, các run/figure dung lượng
lớn, hay toàn bộ repo training. Vì vậy bundle đủ để bàn giao và kiểm tra
pipeline này; để train hoặc chạy đầy đủ Isaac Lab/MJLab nên clone/copy toàn bộ
repo rồi dùng bundle làm hồ sơ phiên bản.

## 7. Kiểm tra an toàn trên laptop mới

Từ thư mục chứa archive, kiểm tra trước khi giải nén:

```bash
sha256sum -c <TEN_ARCHIVE>.tar.gz.sha256
tar -xzf <TEN_ARCHIVE>.tar.gz
cd <TEN_BUNDLE>
sha256sum -c SHA256SUMS
```

Các kiểm tra source không kết nối robot:

```bash
bash -n scripts/teleop/export_r1_quest3_hardware_bundle.sh
python3 -m pytest -q tests/teleop hardware/teleop/tests

cmake -S hardware/high_level -B /tmp/hb_high_level_build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/hb_high_level_build -j2
HB_PROJECT_DIR="$PWD/hardware/high_level" \
  /tmp/hb_high_level_build/run_r1 --preflight
```

Test UTL1 độc lập nằm ở
`hardware/teleop/tests/test_high_level_utl1_receiver.cpp`; nó kiểm tra packet
60 byte, sequence, freshness và STOP qua loopback, không tạo lệnh motor.

Ở máy nguồn ngày 2026-08-22, source trong bundle đã đạt: build C++ high-level,
ONNX preflight (`obs[1,83]`, `actions[1,24]`), UTL1 loopback, pipeline synthetic
12-target, và bộ test teleop mục tiêu `232 passed, 1 skipped`. Hai lỗi của lần
chạy toàn bộ pytest chỉ do checkout không chứa các run artifact đã ignore mà
evidence-catalog test tham chiếu; đó không phải lỗi controller.

## 8. Việc chưa được phép bỏ qua

- T007 gần nhất vẫn `unassessed`, 9.98 Hz so với mục tiêu 20 Hz và đa số target
  là nghiệm projected.
- Chưa đối chiếu đầy đủ dấu/thứ tự FK với pose thật và đúng revision robot.
- Chưa phê duyệt giới hạn vận tốc, gia tốc, torque và collision guard phần cứng.
- Chưa duyệt hành vi nghiệm projected trên robot.
- Phải chứng minh sole-owner, chuyển mode, STOP/watchdog và E-stop bằng bounded
  suspended run mới; plumbing direct-lowcmd lịch sử không đủ cho kiến trúc này.
- Không có bằng chứng cho phép robot đứng/chạy trên sàn.

Trước hardware: xác nhận đúng model R1-A5 và interface, robot treo/cố định, có
người giữ E-stop, chạy dry-run, lưu log theo template, và đóng các mục liên quan
trong `hardware/teleop/docs/hardware_gate.md`. Không đưa secret vào repo hoặc
bundle, không sửa `third_party/`, và không tuyên bố hardware-ready từ kết quả
mô phỏng/preflight.

## 9. Tái tạo bundle từ full workspace

```bash
scripts/teleop/export_r1_quest3_hardware_bundle.sh
```

Exporter chỉ đọc/đóng gói source và tạo archive trong `results/smoke/`; nó
không SSH, không kết nối robot, không bật service và không gửi motor command.
