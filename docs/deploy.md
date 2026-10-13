# Đồng bộ, build và điều khiển service

Repo chỉ nằm trên **máy trạm**; robot nhận bản đồng bộ qua SSH (Tailscale) vào
`/home/unitree/HB` và không bao giờ được clone hay sửa tại chỗ. Mọi thao tác chạy
từ máy trạm bằng một lệnh `./hb` ở gốc repo (`./hb help` liệt kê đầy đủ); lệnh
service/status/logs tự chạy trên robot qua SSH. `./hb` có mặt trên robot (đi theo
bản đồng bộ) chỉ để debug tại chỗ; lệnh tự nhận biết robot (aarch64) hay máy trạm,
ép bằng `HB_MODE=robot|dev`.
Muốn gọi `hb` từ bất kỳ đâu: `ln -s "$PWD/hb" ~/.local/bin/hb`.
Không rsync hay restart service thủ công: các guard (backup, preflight, SHA,
DISARMED, xác thực sudo) nằm trong script mà `./hb` gọi.

## Đường kết nối: Tailscale

Robot `robot-r1` có IP Tailscale cố định `100.82.165.36` (đổi bằng
`HB_ROBOT_TAILSCALE=<ip>`), tới được dù robot đang ở hotspot, LAN hiện trường hay
mạng khác — chỉ cần robot có Internet. `./hb field` và `./hb ssh` luôn đi đường
này. Các lệnh khác dò robot theo `integration/scripts/find_robot.sh`: biến `ROBOT`,
rồi `~/.config/hb/robot.env`, rồi danh sách IP LAN, Tailscale cuối cùng. Đặt
`ROBOT=unitree@100.82.165.36` trong `~/.config/hb/robot.env` để mọi lệnh đi thẳng
Tailscale.

Tailscale SSH trên robot đang ở chế độ **check**: lần SSH đầu (và định kỳ, mặc
định 12 giờ) in `To authenticate, visit: https://login.tailscale.com/a/...`. Mở
link, đăng nhập tailnet, xác nhận; lệnh đang chờ tự chạy tiếp. Muốn bỏ bước này ở
hiện trường: đổi rule SSH của robot trong Tailscale admin từ `check` sang `accept`
(quyết định về quyền truy cập tailnet).

## Bảng lệnh

| Việc | Lệnh | Ghi chú |
| --- | --- | --- |
| **Đồng bộ toàn bộ (mặc định)** | `./hb field [--no-restart]` | chờ robot online trên Tailscale → như `./hb deploy` → status. Dùng ở lab và hiện trường. |
| Shell trên robot | `./hb ssh [lệnh]` | qua Tailscale |
| Deploy toàn bộ (tự dò địa chỉ) | `./hb deploy` | backup → sync → build ARM64 → preflight → restart an toàn → teleop. Chấp nhận policy đang chọn. |
| Deploy toàn bộ, không restart | `./hb deploy all --no-restart` | |
| Chỉ policy đi bộ | `./hb deploy policy [--dry-run]` | preflight local, sync bundle walk + teleop + `locomotion.yaml` + `gestures.yaml` + manifest, restart high-level khi DISARMED |
| Policy + dance | `./hb deploy dance [--dry-run]` | thêm `dance.yaml` + mirror `policies/dance/`, một lần restart |
| Controller C++ | `./hb deploy high [--no-restart]` | build trên robot; restart khi DISARMED |
| Voice / integration / presets | `./hb deploy voice` · `integration` · `presets` | không đụng high-level |
| Teleop Quest | `./hb deploy teleop` | sync + restart runtime; không cần DISARMED |
| Xem trước / quay lại / kéo về | `./hb diff` · `./hb rollback` · `./hb pull [--dry-run]` | |
| Start / stop / restart | `./hb start\|stop\|restart <all\|high\|integration\|voice\|presets\|teleop>` | high, all từ chối khi chưa DISARMED; teleop lúc nào cũng được |
| Trạng thái / log | `./hb status` · `./hb logs [all\|high\|voice\|teleop\|…]` | |
| Build | `./hb build [--clean]` | máy trạm: `run_r1` + test (dùng cho preflight policy local). Build ARM64 do deploy làm trên robot |
| Kiểm tra | `./hb test [all\|cpp\|py]` · `./hb preflight` | không xuất lệnh động cơ; preflight trên robot đã nằm trong deploy |
| Cài service | (trong deploy) | `./hb field` / `./hb deploy` tự cài/refresh systemd, env voice, runtime teleop. `./hb install` chỉ còn để sửa chữa tại chỗ qua `./hb ssh` |
| Dance | `./hb dance list\|prune [--apply]\|restore <tên>` | |

Trên máy trạm, lệnh service/status/logs chạy trên robot qua SSH; chạy trên robot thì tại chỗ.
`make <tên>` cũ vẫn chạy (bí danh của `./hb`).

## Đồng bộ toàn bộ

```bash
./hb field                               # qua Tailscale, chờ robot online nếu chưa lên
./hb deploy                              # tự dò địa chỉ (LAN trước, Tailscale cuối)
ROBOT=unitree@<ip> ./hb deploy           # ép địa chỉ
```

Lệnh đồng bộ đúng trạng thái thư mục máy trạm, **kể cả thay đổi chưa commit**.
File đã xóa trên máy trạm bị xóa theo trên robot với `voice/`, `integration/`,
`voice_presets/` và `controller/policies/dance/`; phần còn lại của `controller/`
không xóa file thừa trên robot (vd. log, quỹ đạo ghi tại chỗ, policy cũ vẫn còn). Lệnh kiểm tra dung lượng `/tmp`, chấp nhận SHA của
`walk_policy` đang chọn, rồi
backup → sync → build ARM64 → preflight → restart an toàn, gồm cả teleop
robot-local. **Có chấp nhận policy tự động**: chỉ dùng khi checkpoint đã được
nghiệm thu.

Khi robot chưa `DISARMED`: `hb_integration` + `hb_voice` + `hb_voice_presets`
**vẫn được restart**, rồi script in `PENDING_RESTART` và thoát **mã 3**. Chỉ
`run_r1` chưa kích hoạt; teleop robot-local cũng chưa restart. Deploy báo đỏ
không có nghĩa là chưa có gì lên robot.

## Deploy nhanh: policy và dance

```bash
./hb deploy policy          # chỉ policy đi bộ / locomotion.yaml
./hb deploy dance           # + dance.yaml và policies/dance, một lần restart
```

Chạy preflight bằng `controller/build/run_r1` của máy trạm (tự build nếu chưa có;
sau khi đổi C++ thì `./hb build` trước), tự cập nhật manifest theo policy đang
chọn, sync, rồi chỉ restart `hb_high_level` khi robot `DISARMED`. Nếu contract
mới không qua preflight local hoặc cần build C++ mới trên robot, dùng `./hb deploy`.
`dance` mirror `policies/dance/` (xóa asset dance trên robot nếu không còn ở máy
dev); manifest chỉ ghi nhận locomotion.

Không đặt API key vào repository hay dòng lệnh. Secret và cấu hình phần cứng
của robot nằm trong `/etc/hb/stack.env` và deploy không ghi đè file này.

## Xác thực SSH/sudo

Mọi lệnh deploy xác thực **ngay từ đầu**: mở một SSH master dùng chung, và với lệnh cần sudo thì hỏi mật khẩu sudo một lần trước khi sync/build, sau đó dùng `sudo -n` nên không bao giờ dừng giữa chừng để hỏi mật khẩu. Lần đầu, deploy cài `/etc/sudoers.d/90-hb-deploy-auth` trên robot (`Defaults:unitree timestamp_type=global`) để các kênh SSH dùng chung cache sudo; quy tắc này không cấp thêm quyền lệnh nào và giữ timeout bình thường. Tên robot dò được và cơ chế auth nằm ở `integration/scripts/find_robot.sh` và `deploy_auth.sh`.

Deploy không đồng bộ checkpoint huấn luyện (`*.pt`, `*.pth`) và thư mục `build_*/`; robot chỉ nạp ONNX.

Khi bật teleop robot-local, deploy chỉ báo xong khi: hai service active, cổng WebXR mở, sidecar H4 đã nhận `rt/lowstate`, kênh tracking `xr-frame-v3` trả lời, mDNS/chứng chỉ khớp và (nếu bật camera) camera nguồn đọc được trong **lần chạy hiện tại**. Camera lỗi → thoát mã 2 nhưng teleop tay/đầu vẫn chạy.

## Kiểm tra sau deploy

```bash
./hb status
```

Kết quả cần có high-level, integration, voice, presets và teleop runtime
`active`, `high_alive=1`, và voice có
`openai_ready=1`, `mic_ready=1`. Cảnh báo `remote_alive=0` nghĩa là tay cầm
R3-1 chưa được phát hiện; PTT sẽ giữ fail-closed cho tới khi remote hoạt động.

## Voice preset offline

Đặt recordings tại `voice_presets/assets/`, gán chúng cho `UP`, `RIGHT`,
`DOWN`, `LEFT` trong `voice_presets/config/presets.yaml`, rồi deploy riêng:

```bash
./hb deploy presets
```

Lệnh này không sync/build/restart `controller`. Giữ `L1` một mình ba giây để
bật/tắt mode (có cue), kể cả khi `run_r1` đã armed bằng `R1+R2`. Khi mode bật,
giữ `L1` và nhấn `UP`, `RIGHT`, `DOWN`, hoặc `LEFT` để phát slot tương ứng.
Phím hướng riêng lẻ không phát preset và tổ hợp L1+phím hướng không kích hoạt
gesture phím hướng trần. Nhấn lặp cùng tổ hợp sau 1,5 giây để hủy; lần lặp sớm
hơn được bỏ qua. `B` vẫn là nút dừng khẩn cấp tùy chọn.
Preset chờ thêm `playback.end_tail_s` (mặc định 0,4 giây)
sau PCM cuối để không cắt đuôi loa. Xem [cấu hình preset](../voice_presets/README.md)
để đổi file/volume/thời gian đệm.

## Khi full deploy chờ restart (`PENDING_RESTART`)

Code đã được sync, build và preflight trên robot; chỉ `run_r1` chưa nạp bản mới
và teleop chưa được cài lại. Không cần deploy lại:

```bash
./hb deploy teleop      # dùng được ngay, kể cả khi robot còn ARMED
# Khi robot đã DISARMED (giữ R1+R2 khi đứng yên):
./hb restart high
./hb status
```

## Rollback

Nếu preflight/runtime sau deploy không đạt, rollback về backup gần nhất rồi
kiểm tra lại:

```bash
./hb rollback
./hb status
```

Không chạy rollback hoặc restart high-level khi robot đang armed.

## Dừng/bật service có kiểm soát

`stop all`, `stop high`, `restart all`, `restart high` chỉ chạy khi robot đã
`DISARMED` (hoặc high-level không chạy). Teleop không ghi động cơ nên
start/stop/restart lúc nào cũng được. Đây là guard bắt
buộc để không bỏ motor runner lúc robot còn armed.

```bash
# Dừng toàn bộ stack: high-level + integration + voice + presets (+ teleop nếu đã cài).
./hb stop all

# Dừng riêng motor runner — dùng trước calibration.
./hb stop high

# Dừng riêng PTT/audio coordinator; high-level và voice vẫn chạy.
./hb stop integration

# Dừng riêng voice; high-level và integration vẫn chạy.
./hb stop voice

# Dừng riêng preset player; các phần còn lại vẫn chạy.
./hb stop presets
```

Bật lại từng service khi cần:

```bash
./hb start high
./hb start integration
./hb start voice
./hb start presets

# Hoặc bật lại cả stack (teleop cũng bật nếu đã cài).
./hb start all

# Restart riêng một service (vd sau khi sửa config).
./hb restart voice
./hb restart high          # cần DISARMED
```

## Calibration

Trước calibration, robot phải `DISARMED`, sau đó dừng riêng high-level:

```bash
./hb stop high
```

Sau calibration, khởi động lại high-level và kiểm tra status:

```bash
./hb start high
./hb status
```

## Sự cố thường gặp

| Hiện tượng                                                    | Xử lý                                                                                                                           |
| ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `No space left on device` khi local preflight                  | Giải phóng dung lượng an toàn trên máy trạm, đặc biệt`/tmp`, rồi chạy lại `./hb deploy`.                          |
| `walk_policy and flat_model/flat_policy_contract are mutually exclusive` | Chọn policy bằng đúng một cách trong `locomotion.yaml` (bundle khuyến nghị). |
| `manifest=... but tuning selects ...`                          | Chạy `./hb deploy`; nó cập nhật manifest theo `walk_policy` hiện tại. |
| `[AUTH] Xác thực SSH ...` đứng im                             | Tailscale SSH đang chờ xác nhận: chạy lệnh trong terminal để thấy link `login.tailscale.com`, mở và xác nhận. |
| `./hb field` đứng ở `Chờ robot online trên Tailscale`         | Robot chưa có Internet hoặc chưa bật Tailscale; kiểm `tailscale status` trên máy trạm (`robot-r1` phải `active`). |
| `runtime health check failed` / `microphone stream is not ready` sau `PREFLIGHT_OK` | Tay cầm R3 chưa bật nên mic giữ fail-closed; code đã lên robot. Bật R3, rồi `./hb deploy teleop` nếu teleop chưa restart. |
| `PENDING_RESTART`                                              | Đưa robot về `DISARMED`, rồi `./hb restart high` (code đã build trên robot) và `./hb deploy teleop`. |
