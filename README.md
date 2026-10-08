# Happy Baby R1

> **Nội bộ AiRA-Laboratory.** Không chia sẻ mã nguồn, tài liệu, log vận hành, hình ảnh robot hoặc dữ liệu test ra bên ngoài khi chưa được phép.

Nhánh `develop` là **bản tích hợp chuẩn để chạy trên robot Unitree R1**. Kết quả từ các nhánh năng lực (đi bộ, nhảy, teleop, giọng nói) được gom vào đây; mã huấn luyện, mô phỏng và thí nghiệm ở lại nhánh gốc. Clone repo này lên robot, build một lần là chạy.

## Chạy trên robot

Robot: Ubuntu 20.04 aarch64, DDS nội bộ `eth10`.

```bash
git clone <repo> ~/HB && cd ~/HB
make build        # controller + voice bridge + integration + môi trường voice
make preflight    # kiểm tra model/config/asset, không xuất lệnh động cơ
make install      # cài và bật service (sudo); controller chỉ restart khi robot DISARMED
make status
```

Cần có sẵn trên robot: `cmake`, `g++`, zlib, Unitree SDK2 C++ cài vào hệ thống (`find_package(unitree_sdk2)`), mã nguồn SDK2 có audio R1 ở `~/unitree_sdk2` hoặc `~/unitree_sdk2-main` (cho voice bridge), `uv` ở `~/.local/bin/uv`. ONNX Runtime đã có sẵn trong `controller/thirdparty/`. Secret (API key) đặt trong `/etc/hb/stack.env`, không bao giờ vào repo.

Teleop Quest (chỉ khi robot treo trên giá): `make teleop-runtime` — xem [docs/teleop.md](docs/teleop.md).

Từ máy dev: `make deploy` (toàn bộ) hoặc `make deploy-policy` (chỉ policy); robot được tự dò, hoặc `ROBOT=unitree@<ip>`. `make help` liệt kê mọi lệnh.

## Cấu trúc

```text
controller/       run_r1 (C++): policy ONNX/NPZ, trạng thái, an toàn — tiến trình DUY NHẤT ghi rt/lowcmd
  config/         cấu hình chạy (tuning.yaml là entry point)
  policies/       policy đi bộ, nhảy, gesture đang dùng
integration/      service coordinator, systemd, script build/preflight/cài đặt/deploy
voice/            trò chuyện giọng nói (Python + bridge loa C++), không chạm động cơ
voice_presets/    phát câu thu sẵn offline (L1 + phím hướng)
teleop/           Quest 3 → IK → sidecar (chạy trên robot), không chạm động cơ trực tiếp
docs/             tài liệu vận hành
Makefile          mọi lệnh build/vận hành/test
```

Service trên robot: `hb_high_level` (controller), `hb_integration`, `hb_voice`, `hb_voice_presets`, `hb_teleop_cert` + `hb_teleop_runtime` (teleop), gom bởi `hb-stack.target`.

## Tài liệu

| Tài liệu | Nội dung |
| --- | --- |
| [docs/safety.md](docs/safety.md) | Quy tắc an toàn, DISARMED trước khi dừng/restart, giới hạn đầu — **đọc trước tiên** |
| [docs/controls.md](docs/controls.md) | Nút R3, phím, cổng/giao diện, lệnh `make` |
| [docs/operation.md](docs/operation.md) | Hướng dẫn vận hành chi tiết: bàn giao, đi bộ, nhảy, ngồi, nằm/đứng, chức năng an toàn tự động, xử lý sự cố |
| [docs/teleop.md](docs/teleop.md) | Teleop Quest 3 trên robot và các cổng phần cứng còn mở |
| [docs/voice.md](docs/voice.md) | Vận hành OpenAI + ElevenLabs |
| [docs/deploy.md](docs/deploy.md) | Deploy, rollback, calibration từ máy dev |
| [docs/policies.md](docs/policies.md) | Chọn/đổi policy, đưa kết quả từ nhánh khác vào |
| [docs/test_log_template.md](docs/test_log_template.md) | Mẫu log mỗi buổi chạy |
| [docs/user-manual.pdf](docs/user-manual.pdf) | Sách hướng dẫn Unitree R1 |

## Kiểm thử

```bash
make test          # ctest của controller + pytest của teleop, voice, voice_presets, integration
```

Kiểm thử phần mềm chỉ chứng minh hợp đồng phần mềm, không phải thăng bằng hay an toàn vật lý. Mỗi buổi chạy robot ghi log theo mẫu và tuân thủ [docs/safety.md](docs/safety.md).
