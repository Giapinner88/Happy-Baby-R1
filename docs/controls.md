# Nút, phím, cổng — tra nhanh

Bảng tra cho người vận hành. Chi tiết từng thao tác: [operation.md](operation.md). Quy tắc an toàn: [safety.md](safety.md).
Nguồn sự thật là mã và cấu hình (`controller/src/input/GamepadR3.hpp`, `controller/config/*.yaml`, `teleop/config/teleop.yaml`); bảng này lệch thì sửa bảng.

## 1. Tay cầm R3 / bàn phím

Mỗi modifier là một nhóm việc, không chồng chéo: **L2 = an toàn / tư thế**, **R2 = di chuyển**, **R1 = nhảy**, **nút trần = động tác tay / teleop**. Tay cầm phải được bàn giao trước: **L2+R2** (một lần, toggle của hãng) rồi giữ **R1+R2** ~1 s.

**L2 — an toàn / tư thế**

| Việc | Tay cầm R3 | Bàn phím |
| --- | --- | --- |
| **Dừng khẩn cấp** (damping) | **L2+B** (giữ) | ESC (damping rồi thoát) |
| Khóa đứng; đang nhảy thì huỷ điệu | **L2+Lên** | 0 |
| Ngồi ghế (ghế cao 43 cm) | **L2+Trái** (giữ) | 9 |
| Nằm xuống; đứng dậy (2 bước) | **L2+X** (giữ); **L2+Lên** rồi **L2+X** | — |
| Zero-torque (chỉ từ IDLE; robot đã nằm hoặc được đỡ) | **L2+Y** | — |

**R2 — di chuyển** (hai chế độ đi bộ = hai policy chọn trong `locomotion.yaml`)

| Việc | Tay cầm R3 | Bàn phím |
| --- | --- | --- |
| Chọn `walk_policy` | **R2+A** | 1 |
| Chọn `teleop_policy` | **R2+X** | T |
| Tốc độ nhanh / chậm | **R2+Lên / R2+Xuống** | Tab (đảo) |
| Tiến–lùi / sang ngang / xoay | Stick trái ↑↓ / stick trái ←→ / stick phải ←→ | W S / A D / Q E |
| Reset policy | — | R |

Từ STAND LOCK, R2+A / R2+X vào thẳng chế độ tương ứng. Đang đi bộ, bấm chế độ kia thì robot **dừng chân, thu tay, chờ gait đứng yên ổn định** (`policy_swap_dwell_s`, mặc định 0.5 s) rồi mới đổi policy; bấm lại chế độ hiện tại để huỷ, quá `policy_swap_timeout_s` (6 s) cũng tự huỷ. Sau IDLE (E-stop, ngã) robot luôn quay về chế độ đi bộ thường.

**R1 — nhảy** (từ LOCOMOTION ở chế độ nào cũng được; xong tự về chế độ đó)

| Việc | Tay cầm R3 | Bàn phím |
| --- | --- | --- |
| Nhảy slot 2 / 3 / 4 / 5 | **R1+Lên / Phải / Xuống / Trái** | 2–5 |
| Nhảy slot 6 / 7 / 8 | **R1+Y / R1+B / R1+A** | 6 / 7 / 8 |

**Nút trần — tay, teleop, voice**

| Việc | Tay cầm R3 |
| --- | --- |
| Động tác tay (bấm lại để thu) | double-click: Lên=1, Xuống=2, Trái=3, Phải=4, A=5 (voice-auto), B=6, X=7, Y=8 |
| Cấp/thu quyền tay teleop | nhấn **START** một lần ở **R2+A, R2+X** hoặc ZERO TORQUE. R2+A: thả cần về 0, base sẽ khóa; R2+X: được vừa đi vừa teleop; xem [teleop.md](teleop.md) |
| Voice: nói (push-to-talk) / ngắt lượt | giữ **SELECT** / double-tap **SELECT** |
| Voice: rảnh tay / đổi nguồn mic | double-click **F1** / nhấn **F2** một lần |
| Voice preset | giữ **L1** một mình 3 s để bật/tắt; giữ **L1** + Lên/Phải/Xuống/Trái để phát |

Ở R2+X, gesture double-click và gesture gọi bằng voice vẫn dùng được khi H4 chưa chiếm quyền. Bật START nhưng chưa bóp cò Quest vẫn dùng gesture được. Khi H4 nhận quyền giữa một gesture, tay thu về và hoàn tất handover trước khi H4 chạy. Voice gesture chỉ bắt đầu khi robot không có lệnh di chuyển.

Tên điệu ở mỗi slot nằm trong `controller/config/dance.yaml` (`dance_2` … `dance_8`); slot không có artifact sẽ bị bỏ qua lúc khởi động (xem [policies.md](policies.md) §4).
Không gán chức năng mới lên **L2+R2**, **L1+L2** (của hãng) hay **SELECT/F1/F2** (của voice). Thêm chức năng mới thì đặt vào đúng nhóm modifier ở trên.

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

Chạy từ **máy trạm** ở gốc repo; lệnh service/status/logs tự chạy trên robot qua SSH (Tailscale). Robot không giữ repo. `./hb help` liệt kê đầy đủ, chi tiết ở [deploy.md](deploy.md).

| Việc | Lệnh |
| --- | --- |
| Đồng bộ toàn bộ (mặc định) | `./hb field` — chờ robot trên Tailscale, sync, build ARM64 trên robot, preflight, restart khi DISARMED |
| Đồng bộ từng phần | `./hb deploy policy` / `dance` / `high` / `voice` / `integration` / `presets` / `teleop` |
| Shell trên robot | `./hb ssh [lệnh]` |
| Policy đi bộ | `./hb policy list` / `./hb policy add <file.onnx> <tên> [--teleop]` — chọn bằng `locomotion.yaml: walk_policy / teleop_policy` |
| Xem trước / quay lại / kéo về | `./hb diff` / `./hb rollback` / `./hb pull` |
| Start / stop / restart | `./hb start\|stop\|restart <all\|high\|integration\|voice\|presets\|teleop>` — high/all từ chối nếu chưa DISARMED; teleop lúc nào cũng được |
| Trạng thái / log | `./hb status` / `./hb logs [high\|voice\|teleop\|…]` |
| Build / kiểm tra (máy trạm) | `./hb build`, `./hb test` |
| Ép địa chỉ robot | `ROBOT=unitree@<ip> ./hb …`, hoặc `HB_ROBOT_TAILSCALE=<ip>` cho `field`/`ssh` |
| Kiểm không chạy trùng | `./hb ssh pgrep -a run_r1` (phải đúng 1 dòng) |

Trình tự chuẩn: treo/đỡ robot → bật nguồn → **L2+R2**, giữ **R1+R2** → **L2+Lên** → đặt xuống đất → **R2+A** (thả cần về 0) hoặc **R2+X** → **START** nếu teleop → điệu / động tác → **L2+Trái** khi xong. Bấm phím trước, đỡ robot sau.
