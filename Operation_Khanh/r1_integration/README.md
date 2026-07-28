# HB R1 Integration

Lớp điều phối an toàn giữa `high_level_2` và `voice_r1`.

## Chức năng

- Subscribe read-only `rt/lowstate` để đọc nút `select`; không publish DDS.
- Nhận heartbeat và `BUSY/IDLE` từ high-level qua `/run/hb/integration.sock`.
- Gửi gate PTT/loa tới voice qua `/run/hb/voice_gate.sock`.
- Gói R3-1 bằng 0 đóng PTT/mic ngay; sau reconnect phải nhả `select` trước khi mở lại.
- Mất dữ liệu R3-1 quá 3 giây thì `remote_alive=0`; timeout này khớp watchdog high-level.
- Chạy package `voice_r1/hb_voice` trực tiếp bằng transport headless, không
  WebRTC hoặc runtime Python nằm chéo trong thư mục integration.
- Quản lý sync, build ARM64, preflight, systemd, health-check và rollback.

## Một lệnh deploy

```bash
ROBOT=unitree@192.168.12.2 ./scripts/deploy_stack.sh diff
ROBOT=unitree@192.168.12.2 ./scripts/deploy_stack.sh deploy
./scripts/deploy_stack.sh status
```

Deploy không restart high-level nếu không chứng minh được trạng thái `DISARMED`; khi đó
trả về `PENDING_RESTART`. Script không lưu mật khẩu SSH/API.

## Sau khi bật robot

1. Chờ ba service active; robot vẫn ở `DISARMED`.
2. Thực hiện bàn giao Development Mode và arm high-level như hướng dẫn vận hành hiện có.
3. Giữ `select` để nói, nhả để kết thúc lượt.
4. Khi high-level phát âm thanh, PTT bị bỏ qua; nhả rồi giữ lại sau khi âm thanh kết thúc.

## Service

```text
hb_integration.service  read-only PTT/audio coordinator
hb_high_level.service   motor runner, boot DISARMED
hb_voice.service        P4b headless voice supervisor
hb-stack.target         bật cả stack khi boot
```

Runtime config/secret: `/etc/hb/stack.env`. Trạng thái không secret:
`/run/hb/status.env`.

Nguồn mic, model, voice, tốc độ, volume câu trả lời và noise reduction nằm tại
`voice_r1/config/tuning.yaml`; prompt sự kiện nằm tại
`voice_r1/config/prompt.txt`. Hai file này được đồng bộ từ máy dev. API key và
cấu hình phần cứng vẫn chỉ nằm trong `/etc/hb/stack.env` trên robot.
Script deploy chỉ tạo file này khi chưa tồn tại và chỉnh quyền `root:600`; nội
dung file hiện có không bị đồng bộ hoặc ghi lại khi cài/restart service.

## Đổi Wi-Fi

Đổi Wi-Fi thường chỉ làm đổi IP SSH. Có thể bỏ qua cơ chế dò bằng:

```bash
ROBOT=unitree@<IP_MOI> ./scripts/deploy_stack.sh status
```

Không đổi `UNITREE_NETWORK_INTERFACE=eth10` sang card Wi-Fi: `eth10` là mạng
nội bộ DDS/audio của R1, độc lập với đường Internet.
