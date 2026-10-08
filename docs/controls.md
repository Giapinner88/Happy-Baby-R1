# Nút, phím, cổng — tra nhanh

Bảng tra cho người vận hành. Chi tiết từng thao tác: [operation.md](operation.md). Quy tắc an toàn: [safety.md](safety.md).
Nguồn sự thật là mã và cấu hình (`controller/src/input/GamepadR3.hpp`, `controller/config/*.yaml`, `teleop/config/teleop.yaml`); bảng này lệch thì sửa bảng.

## 1. Tay cầm R3 / bàn phím

| Việc | Tay cầm R3 | Bàn phím (chỉ khi chạy tay có màn hình) |
| --- | --- | --- |
| Bàn giao từ controller hãng rồi arm `run_r1` | **L2+R2** (một lần — toggle của hãng), rồi giữ **R1+R2** ~1 s | — |
| Khóa đứng; đang nhảy thì huỷ điệu | **L2+Lên** | 0 |
| Đi bộ (từ khóa đứng) | **R2+A** | 1 |
| Nhảy slot 2 / 3 / 4 / 5 | **R1+Lên / Phải / Xuống / Trái** | 2–5 |
| Nhảy slot 6 / 7 / 8 | **R1+Y / R1+B / R1+A** | 6 / 7 / 8 |
| Tiến–lùi / sang ngang / xoay | Stick trái ↑↓ / stick trái ←→ / stick phải ←→ | W S / A D / Q E |
| Tốc độ nhanh / chậm | **R2+Lên / R2+Xuống** | Tab (đảo) |
| Reset policy | — | R |
| Ngồi ghế (ghế cao 43 cm) | **L2+Trái** | 9 |
| Nằm xuống; đứng dậy (2 bước) | **L2+X**; **L2+Lên** rồi **L2+X** | — |
| **Dừng khẩn cấp** (damping) | **L2+B** (giữ) | ESC (damping rồi thoát) |
| Zero-torque (chỉ từ IDLE; robot đã nằm hoặc được đỡ) | **L2+Y** | — |
| Động tác tay (bấm lại để thu) | double-click nút trần: Lên=1, Xuống=2, Trái=3, Phải=4, A=5, B=6, X=7, Y=8 | — |
| Cấp/thu quyền teleop (LOCOMOTION hoặc ZERO TORQUE) | nhấn **START** một lần (nút trần) — xem [teleop.md](teleop.md) | — |
| Voice: nói (push-to-talk) / ngắt lượt | giữ **SELECT** / double-tap **SELECT** | — |
| Voice: rảnh tay / đổi nguồn mic | double-click **F1** / nhấn **F2** một lần | — |
| Voice preset | giữ **L1** một mình 3 s để bật/tắt; giữ **L1** + Lên/Phải/Xuống/Trái để phát | — |

Tên điệu ở mỗi slot nằm trong `controller/config/dance.yaml` (`dance_2` … `dance_8`); slot không có artifact sẽ bị bỏ qua lúc khởi động (xem [policies.md](policies.md) §4).
Không gán chức năng mới lên **L2+R2**, **L1+L2** (của hãng) hay **SELECT/F1/F2** (của voice).

## 2. Cổng và giao diện

| Thành phần | Giá trị | Cấu hình |
| --- | --- | --- |
| DDS trên robot | `eth10` (kiểm `ip a`) | `controller/config/runtime.yaml: network_interface` |
| Lệnh động cơ | DDS `rt/lowcmd`, chỉ `hb_high_level` publish | — |
| Trạng thái robot | DDS `rt/lowstate` | — |
| Teleop → `run_r1` | UDP `127.0.0.1:5560`, chỉ loopback | `controller/config/gestures.yaml: teleop_udp_port` |
| Quest → robot (WebXR) | `https://<IP wlan0>:8012` | `teleop/config/teleop.yaml: robot_runtime.port` |
| Timeout mất gói teleop | 300 ms | `gestures.yaml: teleop_timeout_ms` |
| Trạng thái stack | `/run/hb/status.env` (`high_alive`, `high_armed`, `high_state`) | — |
| Secret, cấu hình riêng robot | `/etc/hb/stack.env`, `/etc/hb/teleop/runtime.env` | — |

## 3. Lệnh

| Trên robot | Từ máy dev |
| --- | --- |
| `make build`, `make preflight`, `make install` | `make deploy` — backup, sync, build ARM64, preflight, restart khi DISARMED |
| `make status` — service active, `high_alive=1` | `make deploy-policy [ARGS=--with-dance]` |
| `make stop-high` / `stop-all` / `restart-all` — từ chối nếu chưa DISARMED | `make rollback` — không chạy khi robot armed |
| `make start-high` / `start-all` | `ROBOT=unitree@<ip>` để ép địa chỉ robot |
| `make teleop-runtime`, `make teleop-check` | |
| `journalctl -u hb_high_level -f`, `pgrep -a run_r1` (phải đúng 1 dòng) | |

Trình tự chuẩn: treo/đỡ robot → bật nguồn → **L2+R2**, giữ **R1+R2** → **L2+Lên** → đặt xuống đất → **R2+A** → điệu / động tác → **L2+Trái** khi xong. Bấm phím trước, đỡ robot sau.
