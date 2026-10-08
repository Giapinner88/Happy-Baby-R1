# integration — điều phối service và triển khai

Lớp điều phối an toàn giữa `controller` và `voice`.

## Chức năng

- Subscribe read-only `rt/lowstate` để đọc nút `select`; không publish DDS.
- Nhận heartbeat và `BUSY/IDLE` từ high-level qua `/run/hb/integration.sock`.
- Gửi gate PTT/loa tới voice qua `/run/hb/voice_gate.sock`.
- Mirror read-only nút R3-1 và trạng thái high-level sang `voice_presets` qua
  Unix datagram; preset chỉ sở hữu loa trong một lease ngắn, không chạm motor.
- Gói R3-1 bằng 0 đóng PTT/mic ngay; sau reconnect phải nhả `select` trước khi mở lại.
- Mất dữ liệu R3-1 quá 3 giây thì `remote_alive=0`; timeout này khớp watchdog high-level.
- Chạy package `voice/hb_voice` trực tiếp bằng transport headless, không
  WebRTC hoặc runtime Python nằm chéo trong thư mục integration.
- Quản lý sync, build ARM64, preflight, systemd, health-check và rollback.

## Một lệnh deploy

Quy trình đầy đủ, gồm policy acceptance, kiểm tra sau deploy và rollback: [docs/deploy.md](../docs/deploy.md).

```bash
make deploy        # từ gốc repo trên máy dev
```

`make deploy` (`scripts/deploy.sh`) tự chấp nhận model đang chọn, backup, sync, build ARM64, preflight
và restart-safe. Deploy không restart high-level nếu không chứng minh được trạng
thái `DISARMED`; khi đó trả về `PENDING_RESTART`. Script không lưu mật khẩu SSH/API.
Dừng/bật service: `make status|stop-*|start-*|restart-all` (trên robot, qua `scripts/stack_ctl.sh`; lệnh dừng/restart controller từ chối khi chưa `DISARMED`).

## Sau khi bật robot

1. Chờ bốn service active; robot vẫn ở `DISARMED`.
2. Thực hiện bàn giao Development Mode và arm high-level như hướng dẫn vận hành hiện có.
3. Giữ `select` để nói, nhả để kết thúc lượt.
4. Khi high-level phát âm thanh, PTT bị bỏ qua; nhả rồi giữ lại sau khi âm thanh kết thúc.

## Service

```text
hb_integration.service  read-only PTT/audio coordinator
hb_high_level.service   motor runner, boot DISARMED
hb_voice.service        P4b headless voice supervisor
hb_voice_presets.service offline preset player (L1 + phím hướng)
hb-stack.target         bật cả stack khi boot
```

Runtime config/secret: `/etc/hb/stack.env`. Trạng thái không secret:
`/run/hb/status.env`.

Nguồn mic, model, voice, tốc độ, volume câu trả lời và noise reduction nằm tại
`voice/config/tuning.yaml`; prompt sự kiện nằm tại
`voice/config/prompt.txt`. Hai file này được đồng bộ từ máy dev. API key và
cấu hình phần cứng vẫn chỉ nằm trong `/etc/hb/stack.env` trên robot.
Script deploy chỉ tạo file này khi chưa tồn tại và chỉnh quyền `root:600`; nội
dung file hiện có không bị đồng bộ hoặc ghi lại khi cài/restart service.

## Offline voice presets

Thêm file local trong `voice_presets/assets/`, rồi gán file/volume cho bốn slot
`UP`, `RIGHT`, `DOWN`, `LEFT` tại `voice_presets/config/presets.yaml`. Giữ `L1`
một mình ba giây để bật/tắt mode (có cue); thao tác này vẫn dùng được sau khi
`run_r1` đã armed bằng `R1+R2`. Khi mode bật, giữ `L1` và nhấn một phím hướng
để phát slot tương ứng. Phím hướng riêng lẻ không phát preset; tổ hợp L1+phím
hướng không kích hoạt gesture dùng phím hướng trần.
Nhấn lặp cùng tổ hợp sau 1,5 giây để dừng; lần lặp sớm hơn được bỏ qua. Service
hủy phát khi mất remote hoặc high-level báo `BUSY`.
PCM được đẩy theo thời gian thực và chỉ chọn stream audio trong file; mất remote
hoặc high-level `BUSY` hủy request, không tự phát lại. Lỗi
decoder/bridge thì retry clip từ đầu. Sau PCM cuối, preset giữ bridge mở thêm
`playback.end_tail_s` (mặc định 0,4 giây) để không cắt đuôi audio còn nằm trong
bộ đệm loa.

Deploy phần này bằng `./integration/scripts/deploy_stack.sh deploy-presets`;
lệnh chỉ sync/build coordinator + preset/bridge và không restart high-level.
Chi tiết cấu hình: [voice_presets/README.md](../voice_presets/README.md).

## Đổi Wi-Fi

Đổi Wi-Fi thường chỉ làm đổi IP SSH. Có thể bỏ qua cơ chế dò bằng:

```bash
ROBOT=unitree@<IP_MOI> ./scripts/deploy_stack.sh status
```

Không đổi `UNITREE_NETWORK_INTERFACE=eth10` sang card Wi-Fi: `eth10` là mạng
nội bộ DDS/audio của R1, độc lập với đường Internet.
