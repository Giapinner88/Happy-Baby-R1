# HB R1 Voice

Voice assistant headless cho Jetson ARM64 trên robot. Runtime không cần SSH,
browser hoặc nút Connect và không chứa API điều khiển motor.

## Luồng chạy

```text
R3-1 giữ select -> r1_integration mở gate -> mic R1/USB -> OpenAI Realtime
OpenAI audio -> volume voice_r1 -> r1_bridge -> loa robot
high_level_2 BUSY -> khóa mic + cắt loa/response voice ngay lập tức
```

Production luôn dùng PTT và headless. Sau khi high-level ngắt voice, người dùng
phải nhả rồi giữ lại `select` để bắt đầu lượt mới.

## Cấu trúc

```text
hb_voice/__main__.py   supervisor và headless transport
hb_voice/app.py        pipeline OpenAI Realtime
hb_voice/config.py     đọc và kiểm tra tuning/env
hb_voice/gate.py       PTT và ưu tiên âm thanh high-level
hb_voice/input.py      mic multicast R1 và mic USB/ALSA
hb_voice/output.py     volume/gain và loa R1
hb_voice/resilience.py buffer PTT, reconnect và trạng thái health
config/tuning.yaml     model, voice, mic, volume và chế độ hoạt động
config/prompt.txt      vai trò, tính cách và nội dung sự kiện
unitree_bridge/        bridge SDK2 được build trực tiếp trên ARM64
tools/test_mic.py      công cụ ghi thử mic
```

Entry point duy nhất:

```bash
cd /home/unitree/HB/voice_r1
.venv/bin/python -m hb_voice
```

Systemd gọi entry point này qua `r1_integration/scripts/run_voice.sh`. Voice
khởi động song song với high-level; khi Internet đến chậm, supervisor tự tạo
lại session OpenAI mà không cần SSH/restart thủ công.

## Tuning thường dùng

Sửa `config/tuning.yaml` trên máy dev:

```yaml
openai:
  model: "gpt-realtime-1.5"
  voice: "marin"
  speed: 1.0
  language: "vi"
  max_response_tokens: 512

input:
  source: "r1_multicast"  # đổi thành alsa_usb khi cắm mic ngoài
  gain_db: 6.0
  noise_reduction: "auto"

audio:
  response_volume_percent: 100
  response_gain: 1.0

activation:
  mode: "both"
  allow_during_startup: true
  startup_grace_s: 90

resilience:
  connect_timeout_s: 12
  reconnect_initial_s: 2
  reconnect_max_s: 30
  watchdog_interval_s: 1
```

- `voice`: nên nghe thử `marin` và `cedar`; danh sách đầy đủ nằm trong comment
  của `tuning.yaml`. Đổi voice cần restart service.
- `speed`: hợp lệ `0.25-1.5`; sự kiện nên bắt đầu quanh `0.95-1.05`.
- `response_volume_percent`: volume phần cứng riêng cho câu trả lời voice,
  `0-100`; hiện đặt `100`. Voice áp dụng lại mỗi khi giành quyền loa.
- `response_gain`: chỉ cho phép `0.0-1.0` để giảm PCM, tránh khuếch đại gây vỡ
  tiếng.
- `input.source`: chọn `r1_multicast` hoặc `alsa_usb`.
- `input.gain_db`: khuếch đại mic từ `-12` đến `+12 dB`; mặc định `+6 dB`.
  Gain khuếch đại cả giọng và nhiễu, nên giảm nếu giọng gần robot bị rè.
- `noise_reduction=auto`: mic robot dùng `far_field`, mic USB gần người nói dùng
  `near_field`.
- `activation.mode=both`: hoạt động cả khi high-level `DISARMED` và `ARMED`.
  Có thể chọn `high_disarmed` hoặc `high_armed` nếu cần giới hạn.
- `allow_during_startup=true`: PTT được dùng trong 90 giây chờ heartbeat đầu
  tiên. Sau khi đã thấy high-level, nếu heartbeat mất thì gate luôn khóa.
- Nhóm `resilience`: giới hạn thời gian kết nối và backoff; mặc định tự phục
  hồi khi Wi-Fi/DNS/OpenAI đến chậm hoặc WebSocket bị ngắt.

High-level có `voice_volume`/`dance_volume` riêng. Gate chỉ cho một bên phát tại
một thời điểm; bên giành quyền loa sẽ đặt lại volume của chính nó trước khi phát.

Prompt nội dung sự kiện nằm tại `config/prompt.txt`, không đặt chung với code.

## Secret và phần cứng

Các giá trị riêng của robot nằm trong `/etc/hb/stack.env` (owner `root`, mode
`600`) và không được đồng bộ về máy dev:

```env
OPENAI_API_KEY=...
UNITREE_NETWORK_INTERFACE=eth10
```

Mic USB tương lai:

```env
ALSA_DEVICE=plughw:CARD=RobotMic,DEV=0
ALSA_SAMPLE_RATE=16000
```

Đặt `input.source: "alsa_usb"` trong `config/tuning.yaml`, rồi cấu hình hai
biến ALSA phía trên. Không dùng địa chỉ card kiểu `hw:1,0` vì số card có thể
đổi sau reboot.

## Đổi Wi-Fi hoặc địa chỉ IP

Wi-Fi chỉ cung cấp đường Internet và SSH/deploy. Khi robot sang Wi-Fi khác,
thông thường chỉ cần dùng IP mới:

```bash
ROBOT=unitree@<IP_MOI> ./r1_integration/scripts/deploy_stack.sh status
```

Kết nối OpenAI dùng default route/DNS nên không lưu IP Wi-Fi trong source.

`eth10` và dải `192.168.123.x` là mạng nội bộ Unitree dành cho DDS, mic và loa;
nó độc lập với Wi-Fi và không được đổi thành `wlan0`. Chỉ khi tên card nội bộ
thực sự thay đổi mới sửa đồng thời:

- `high_level_2/config/tuning.yaml`: `network_interface`.
- `/etc/hb/stack.env`: `UNITREE_NETWORK_INTERFACE`.

## Deploy

Chỉ thay đổi voice/tuning:

```bash
cd HB
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy --restart-voice
./r1_integration/scripts/deploy_stack.sh status
```

Có thay đổi policy hoặc code high-level: đưa robot về `DISARMED`, tư thế an
toàn, rồi deploy toàn stack:

```bash
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy --accept-policy  # nếu đổi policy
# Hoặc: ./r1_integration/scripts/deploy_stack.sh deploy           # chỉ đổi code
./r1_integration/scripts/deploy_stack.sh status
```

Deploy tự dò các địa chỉ quen thuộc; có thể đặt `ROBOT=unitree@<IP>` để dùng IP
mới. Binary và `.venv` luôn được tạo trên robot ARM64; không copy build x86 từ
máy dev.

`deploy_stack.sh status` đọc thêm `/run/hb/voice_status.env`. Voice chỉ thực sự
sẵn sàng khi cả `openai_ready=1` và `mic_ready=1`; `systemctl active` một mình
không còn được coi là đủ.

## Bridge native

Runtime chỉ dùng hai mode:

```text
r1_bridge speaker <iface> [app_name] [volume_0_100]
r1_bridge mic <iface> [seconds] [group_ip] [port] [raw|rtp|rtp-l16be]
```

`speaker` đọc PCM s16le 16 kHz mono từ stdin. Khi nhận SIGTERM, bridge gọi
`PlayStop(app_name)` để high-level có thể cắt voice ngay.
