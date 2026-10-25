# Happy Baby R1

> **Nội bộ AiRA-Laboratory.** Không chia sẻ mã nguồn, tài liệu, log vận hành, hình ảnh robot hoặc dữ liệu test ra bên ngoài khi chưa được phép.

Nhánh `develop` là **bản tích hợp chuẩn để chạy trên robot Unitree R1**. Kết quả từ các nhánh năng lực (đi bộ, nhảy, teleop, giọng nói) được gom vào đây; mã huấn luyện, mô phỏng và thí nghiệm ở lại nhánh gốc.

## Mô hình triển khai

```text
máy trạm (repo git, sửa code, test)  ──./hb deploy (SSH, LAN hoặc Tailscale)──►  robot: /home/unitree/HB
```

- **Máy trạm là nguồn sự thật duy nhất.** Repo chỉ được clone và sửa trên máy trạm; robot không có git, không clone, không sửa tại chỗ.
- `./hb deploy` tự thấy phần nào đổi và chỉ deploy phần đó; controller chỉ được ghi và restart khi robot `DISARMED`. Lệnh và quy tắc: [docs/deploy.md](docs/deploy.md).
- Lỡ sửa config trên robot thì kéo về bằng `./hb pull` trước khi deploy, nếu không lần deploy sau sẽ ghi đè.

## Buổi chạy

```bash
# 1. Treo robot lên giá, bật nguồn. Chưa bấm L2+R2 / R1+R2 (robot phải DISARMED để controller được restart).
./hb deploy         # hoặc ./hb field (qua Tailscale); chỉ deploy phần có thay đổi
# 2. Bật tay cầm R3; thao tác theo docs/controls.md (bàn giao L2+R2 rồi giữ R1+R2, ...)
./hb logs high      # theo dõi controller (hoặc ./hb ssh để vào shell robot)
```

Không đổi gì thì không cần deploy; robot tự chạy stack khi bật nguồn. Bảng lệnh: [docs/deploy.md](docs/deploy.md). Thao tác nút: [docs/controls.md](docs/controls.md).

## Chuẩn bị (một lần)

**Máy trạm** (Ubuntu x86_64):

- Clone repo này; `cmake`, `g++`, `rsync`, `python3`.
- Tailscale đăng nhập cùng tailnet với robot (`tailscale status` thấy `robot-r1`). Robot có IP Tailscale cố định `100.82.165.36`; đổi bằng `HB_ROBOT_TAILSCALE=<ip>`.
- Khuyến nghị: `~/.config/hb/robot.env` chứa `ROBOT=unitree@100.82.165.36`, để mọi lệnh `./hb` (không chỉ `field`/`ssh`) đi thẳng Tailscale thay vì thử IP LAN cũ trước.

**Robot** (Ubuntu 20.04 aarch64, DDS nội bộ `eth10`), chỉ cần có sẵn — không clone repo:

- Tailscale đã đăng nhập tailnet; user `unitree` có sudo.
- Unitree SDK2 C++ cài vào hệ thống (`find_package(unitree_sdk2)`), mã nguồn SDK2 có audio R1 ở `~/unitree_sdk2` hoặc `~/unitree_sdk2-main` (voice bridge), `uv` ở `~/.local/bin/uv`, `cmake`, `g++`, zlib. ONNX Runtime đã có trong `controller/thirdparty/`.
- Secret (API key) trong `/etc/hb/stack.env` (deploy tạo từ mẫu nếu chưa có, không bao giờ ghi đè hay đồng bộ file này).

Robot mới: `./hb deploy all` (tạo `/home/unitree/HB`, cài service systemd, env Python voice và runtime teleop).

## Cấu trúc

```text
controller/       run_r1 (C++): policy ONNX/NPZ, trạng thái, an toàn — tiến trình DUY NHẤT ghi rt/lowcmd
  config/         cấu hình chạy (tuning.yaml là entry point; locomotion.yaml chọn walk_policy / teleop_policy)
  policies/       locomotion/<bundle>, dance, gestures, flat (legacy)
integration/      service coordinator, systemd, script build/preflight/cài đặt/deploy/dò robot
voice/            trò chuyện giọng nói (Python + bridge loa C++), không chạm động cơ
voice_presets/    phát câu thu sẵn offline (L1 + phím hướng)
teleop/           Quest 3 → IK → sidecar (chạy trên robot), không chạm động cơ trực tiếp
docs/             tài liệu vận hành
test/             thí nghiệm/mô phỏng gộp từ HB, KHÔNG deploy lên robot (xem test/README.md)
hb                lệnh duy nhất: field / deploy / ssh / start-stop-restart / status / logs / build / test (./hb help)
Makefile          bí danh `make <tên>` cho ./hb
```

Service trên robot: `hb_high_level` (controller), `hb_integration`, `hb_voice`, `hb_voice_presets`, `hb_teleop_cert` + `hb_teleop_runtime` (teleop), gom bởi `hb-stack.target`.

## Tài liệu

| Tài liệu                                            | Nội dung                                                                                               |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| [docs/safety.md](docs/safety.md)                       | Quy tắc an toàn, DISARMED trước khi dừng/restart, giới hạn đầu —**đọc trước tiên** |
| [docs/controls.md](docs/controls.md)                   | Nút R3 (nhóm theo modifier), phím, cổng/giao diện, lệnh`./hb`                                   |
| [docs/operation.md](docs/operation.md)                 | Vận hành chi tiết: bàn giao, đi bộ, nhảy, ngồi, nằm/đứng, an toàn tự động, sự cố       |
| [docs/deploy.md](docs/deploy.md)                       | Đồng bộ từ máy trạm, xác thực, rollback, calibration                                            |
| [docs/policies.md](docs/policies.md)                   | Bundle policy đi bộ, contract, đưa kết quả từ nhánh khác vào                                  |
| [docs/teleop.md](docs/teleop.md)                       | Teleop Quest 3 trên robot và các cổng phần cứng còn mở                                          |
| [docs/voice.md](docs/voice.md)                         | Vận hành OpenAI + ElevenLabs                                                                          |
| [docs/test_log_template.md](docs/test_log_template.md) | Mẫu log mỗi buổi chạy                                                                               |
| [docs/user-manual.pdf](docs/user-manual.pdf)           | Sách hướng dẫn Unitree R1                                                                           |

## Kiểm thử

```bash
./hb test          # trên máy trạm: ctest của controller + pytest của teleop, voice, voice_presets, integration
```

Kiểm thử phần mềm chỉ chứng minh hợp đồng phần mềm, không phải thăng bằng hay an toàn vật lý. Mỗi buổi chạy robot ghi log theo mẫu và tuân thủ [docs/safety.md](docs/safety.md).
