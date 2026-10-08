# voice — trò chuyện bằng giọng nói (OpenAI Realtime + ElevenLabs)

Voice assistant headless cho Jetson ARM64 trên robot. Runtime không cần SSH,
browser hoặc nút Connect và không chứa API điều khiển motor.

## Luồng chạy

```text
R3-1 giữ Select (chế độ giữ nút) hoặc double-click F1 (chế độ auto)
-> integration mở gate -> mic ngoài cắm vào PC2
-> OpenAI Realtime (nhận giọng + sinh text)
-> ElevenLabs streaming TTS (giọng clone) -> r1_bridge -> loa robot
controller BUSY -> khóa mic + cắt loa/response voice ngay lập tức
```

## Hai chế độ và ba nút

| | Vào bằng | Nói với robot thế nào |
|---|---|---|
| **Chế độ auto** | double-click F1 | Không bấm gì, robot tự nghe liên tục (Semantic VAD chốt lượt) |
| **Chế độ giữ nút** (mặc định) | double-click F1 lần nữa | Giữ `Select` mới mở mic, nhả ra là chốt lượt |

**Chỉ F1 đổi chế độ.** `Select` không đổi chế độ — nó là cách nói khi đang ở chế
độ giữ nút, còn **double-tap `Select`** là cắt lời robot (dùng được ở cả hai chế
độ). Ở chế độ auto, giữ `Select` là thừa vì mic đã mở sẵn.

Remote mất kết nối quá 3 giây luôn tắt chế độ auto (fail-closed) — lúc đó sẽ nghe
âm báo đi xuống dù không ai bấm gì. Sau khi high-level ngắt voice, phải nhả rồi
giữ lại `Select` để bắt đầu lượt mới.

### Đổi nguồn mic — `F2`

**Nhấn `F2` một lần** (không phải double-click như F1) để đổi qua lại giữa **mic
ngoài PC2** và **mic thân robot**. Nghe một nốt xác nhận: **nốt cao = mic ngoài**,
**nốt thấp = mic thân robot**.

Đây là cách DUY NHẤT đổi nguồn: mặc định `input.mic_switch: manual` nên mic ngoài
chết thì robot đứng im ở mic ngoài và la to trong log, **không tự tụt xuống mic
thân robot nữa**. Đặt `mic_switch: auto` trong `config/tuning.yaml` để lấy lại
hành vi tự đổi cũ.

Khác với chế độ nghe, lựa chọn nguồn mic **dính**: mất tay cầm hay restart
`hb_voice` đều không làm nó đổi — nếu không thì mic sẽ nhảy sau lưng người vận
hành giữa lúc khách đang nói. Đang chạy mic nào thì xem `mic_source` trong
`/run/hb/voice_status.env`, còn `mic_external` trong `/run/hb/status.env` là nút
F2 đang chọn cái nào.

## Cấu trúc

```text
hb_voice/__main__.py   supervisor và headless transport
hb_voice/app.py        OpenAI Realtime + ElevenLabs TTS
hb_voice/config.py     đọc và kiểm tra tuning/env
hb_voice/gate.py       PTT và ưu tiên âm thanh high-level
hb_voice/input.py      mic ngoài PC2 qua ALSA + tự tụt/nhảy về mic PC1 dự phòng
hb_voice/output.py     volume/gain, loa R1 và âm báo đổi chế độ
hb_voice/resilience.py buffer PTT, reconnect và trạng thái health
config/tuning.yaml     model, voice, mic, volume và chế độ hoạt động
config/prompt.txt      vai trò, tính cách và nội dung sự kiện
unitree_bridge/        bridge SDK2 được build trực tiếp trên ARM64
tools/test_mic.py      công cụ ghi thử mic
```

Entry point duy nhất:

```bash
cd /home/unitree/HB/voice
.venv/bin/python -m hb_voice
```

Systemd gọi entry point này qua `integration/scripts/run_voice.sh`. Voice
khởi động song song với high-level; khi Internet đến chậm, supervisor tự tạo
lại session OpenAI mà không cần SSH/restart thủ công.

## Tuning thường dùng

Sửa `config/tuning.yaml` trên máy dev:

```yaml
openai:
  model: "gpt-realtime-1.5"
  voice: "sage"  # chỉ dùng khi rollback về tts.provider=openai
  speed: 1.0
  language: "vi"
  max_response_tokens: 512

tts:
  provider: "elevenlabs"
  model: "eleven_flash_v2_5"
  language: "vi"
  stability: 0.55
  similarity_boost: 0.85
  speed: 1.0
  use_speaker_boost: true

input:
  source: "alsa_usb"  # mic ngoài PC2; r1_multicast chỉ dùng để rollback mic PC1
  gain_db: 0.0
  noise_reduction: "auto"
  fallback_to_robot_mic: true
  fallback_gain_db: 6.0
  fallback_after_failures: 3
  fallback_recover_check_s: 20
  mic_silence_s: 20

audio:
  response_volume_percent: 100
  response_gain: 1.0
  echo_tail_s: 0.6

activation:
  mode: "both"
  allow_during_startup: true
  startup_grace_s: 100

conversation:
  require_external_mic: true
  mode_cue: true
  vad_eagerness: "medium"
  fallback_vad_eagerness: "low"

resilience:
  connect_timeout_s: 12
  reconnect_initial_s: 2
  reconnect_max_s: 30
  watchdog_interval_s: 1
```

- `tts.provider=elevenlabs`: OpenAI chỉ sinh nội dung text; ElevenLabs quyết
  định chất giọng bằng `ELEVENLABS_VOICE_ID`.
- `tts.model=eleven_flash_v2_5`: model streaming độ trễ thấp cho robot.
- `tts.stability`: tăng nếu giọng thay đổi giữa các câu; giảm nếu giọng quá
  đều, thiếu tự nhiên.
- `tts.similarity_boost`: mức bám giọng mẫu. Mức quá cao có thể làm lộ noise
  hoặc lỗi từ bản thu clone.
- `tts.speed`: hợp lệ `0.7-1.2`.
- `openai.voice` và `openai.speed` chỉ có hiệu lực khi đặt
  `tts.provider: "openai"`.
- `response_volume_percent`: volume phần cứng riêng cho câu trả lời voice,
  `0-100`; hiện đặt `100`. Voice áp dụng lại mỗi khi giành quyền loa.
- `response_gain`: chỉ cho phép `0.0-1.0` để giảm PCM, tránh khuếch đại gây vỡ
  tiếng.
- `audio.echo_tail_s` (`0.0-3.0`, mặc định `0.6`): đẩy xong PCM cuối thì đợi
  thêm chừng này giây mới mở lại mic. Bù phần voice không quan sát được:
  chặng DDS sang PC1, bộ đệm của loa, tiếng dội phòng. Quá ngắn thì ở chế độ
  auto robot nghe đuôi tiếng của chính mình rồi tự trả lời tiếp. Robot hay nói
  nối đuôi -> nâng `0.9-1.2`; thấy phản hồi chậm -> hạ về `0.4`.
- `input.source=alsa_usb`: production đọc mic ngoài cắm trực tiếp vào PC2.
  `r1_multicast` chỉ dùng khi cần rollback về mic tích hợp của robot trên PC1.
- `input.gain_db`: gain phần mềm từ `-12` đến `+12 dB`; mic TTGK dùng `0 dB`
  vì capture thô đã từng chạm `0 dBFS`. Hạ ALSA Capture trước; chỉ tăng gain
  phần mềm khi bản ghi thô thực sự nhỏ và không clipping.
- `input.min_speech_peak` (`0-8000`, mặc định `500`): sàn "có tiếng người".
  Áp dụng cho **cả hai** chế độ — giữ nút thì cả lượt phải vượt sàn mới được
  gửi; auto (F1) thì chỉ đoạn vượt sàn mới chảy tới VAD của OpenAI, giữ mở
  thêm `1.5s` sau tiếng cuối để không cắt vụn câu và đệm ~3 chunk phía trước
  để không mất âm đầu. Đây là lớp chặn tiếng servo / đuôi tiếng loa kích VAD
  làm robot tự trả lời chính mình. Khách nói nhỏ mà robot làm ngơ -> hạ về
  `300`; robot đáp cả tiếng ồn -> nâng lên. Đặt `0` để tắt hẳn.
- `noise_reduction=auto`: mic PC1 dùng `far_field`, mic ngoài PC2 dùng
  `near_field`. Khi chạy mic dự phòng, giá trị này tự đổi theo mic đang sống
  chứ không giữ theo `input.source`.
- `input.fallback_to_robot_mic`: **công tắc chọn có dùng mic robot làm dự phòng
  hay không.**
  - `true` — mic ngoài chết thì tự tụt xuống mic tích hợp PC1 (sau
    `fallback_after_failures` lần khởi động hỏng, hoặc sau `mic_silence_s` giây
    câm), robot vẫn nói chuyện được. Đang chạy dự phòng thì cứ
    `fallback_recover_check_s` giây dò `/proc/asound/cards`, thấy card mic ngoài
    về là tự nhảy lại. Đây là lưới cứu sự kiện, không phải cấu hình chạy chính
    thức: mic PC1 nằm trên thân robot nên hút cả tiếng servo.
  - `false` — CHỈ dùng mic ngoài; mic chết thì robot điếc cho tới khi người vận
    hành xử lý, nhưng không bao giờ bị nghe bằng mic thân robot. Mic câm vẫn
    được ghi `ERROR` lặp lại trong log nên không điếc âm thầm.
  - Nguồn đang chạy luôn xem được ở `mic_source` trong `/run/hb/voice_status.env`.
- `input.fallback_gain_db`, `fallback_after_failures`, `fallback_recover_check_s`:
  chỉ có tác dụng khi `fallback_to_robot_mic: true`.
- **Nhịp dò hồi phục tự giãn khi thất bại.** `fallback_recover_check_s` là nhịp
  *ban đầu*. Card có mặt trên bus KHÔNG chứng minh mic nghe được: cắm cục thu vào
  mà mic đeo chưa bật thì card vẫn hiện. Mỗi lần nhảy về mic ngoài rồi vẫn câm,
  nhịp dò nhân đôi (20 → 40 → 80 → 160 → trần 300 giây) và log ghi rõ "produced
  no sound". Khi mic ngoài phát ra tiếng thật, nhịp lập tức về lại giá trị gốc.
  Không có cơ chế này thì bộ dò câm và bộ dò card đá qua đá lại mỗi ~20 giây,
  robot không dùng được bằng mic nào (đã xảy ra thật 2026-08-05 lúc 09:38-09:40).
- `input.mic_silence_s`: mic ngoài **câm** bao lâu thì coi là chết — có dự phòng
  thì chuyển nguồn, không có thì chỉ cảnh báo. Cần riêng núm này vì mic đeo hết
  pin để lại cục thu vẫn cắm USB, vẫn bơm im lặng đều đặn — `arecord` không hề
  lỗi nên bộ đếm hỏng không bao giờ chạy, robot điếc mà không ai biết (đã xảy ra
  thật 2026-08-05). Ngưỡng "câm" là đỉnh `<= 32/32767` (~-60 dBFS), đo thật trên
  robot: mic chết vẫn dao động 1-2 LSB chứ không phải 0 tuyệt đối. Mic sống
  trong phòng yên tĩnh vẫn cao hơn ngưỡng này nhiều lần. Đặt `0` để tắt.
- `activation.mode=both`: hoạt động cả khi high-level `DISARMED` và `ARMED`.
  Có thể chọn `high_disarmed` hoặc `high_armed` nếu cần giới hạn.
- `allow_during_startup=true`: PTT được dùng trong 100 giây chờ heartbeat đầu
  tiên. Sau khi đã thấy high-level, nếu heartbeat mất thì gate luôn khóa.
- `conversation.vad_eagerness`: độ chủ động Semantic VAD khi F1 bật; hiện dùng
  `medium` để phù hợp môi trường sự kiện có tiếng ồn.
- `conversation.require_external_mic=true`: hands-free F1 phải KHỞI ĐỘNG bằng mic
  TTGK qua ALSA trên PC2; cấu hình từ chối start nếu bị đổi nhầm về mic multicast
  PC1. Mất mic ngoài lúc đang chạy là việc của `input.fallback_to_robot_mic`.
- `conversation.fallback_vad_eagerness`: khi đang chạy mic dự phòng PC1, F1 vẫn
  hoạt động nhưng hạ độ nhạy xuống mức này (`low`) để tiếng servo/loa trên thân
  robot không tự kích hoạt lượt nói.
- `conversation.mode_cue=true`: phát âm báo hai nốt mỗi lần đổi chế độ —
  **đi lên** = vào **chế độ auto** (robot tự nghe liên tục), **đi xuống** = về
  **chế độ giữ nút** (chỉ mở mic khi đang giữ Select). Âm báo bám trạng thái THẬT
  sau khi đồng bộ với OpenAI, nên bấm F1 mà bị từ chối thì vẫn kêu đi xuống. Tự im
  khi high-level đang chiếm loa. Đặt `false` nếu sự kiện cần tuyệt đối yên tĩnh.
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
ELEVENLABS_API_KEY=...
ELEVENLABS_VOICE_ID=...
UNITREE_NETWORK_INTERFACE=eth10
```

`ELEVENLABS_VOICE_ID` là ID của giọng Instant/Professional Voice Clone đã tạo
và được phép sử dụng. Không đặt API key hoặc voice ID trong `tuning.yaml`,
source code hay log.

Mic ngoài trên PC2:

```env
ALSA_SAMPLE_RATE=48000
# ALSA_DEVICE=plughw:CARD=BOYALINK,DEV=0   # chỉ khi cần ghim 1 trong nhiều mic USB
```

`config/tuning.yaml` đã chọn `input.source: "alsa_usb"`, vì vậy runtime không
đăng ký multicast mic PC1. **Không cần khai báo tên card:** mỗi lần bộ thu khởi
động lại (5 giây một lần), runtime đọc `/proc/asound/cards` và lấy card USB đầu
tiên có capture, nên cắm mic model khác vào là chạy — không phải sửa file. Tên
card thuộc về model mic chứ không thuộc về robot: đổi TTGK (card `Audio`) sang
BOYALINK (card `BOYALINK`) từng làm `ALSA_DEVICE` ghim cứng trỏ vào hư không, và
log khi đó giống hệt mic rớt khỏi bus USB.

Chỉ đặt `ALSA_DEVICE` khi có từ hai mic USB trở lên trên bus và cần chọn đúng
một cái; khi đó dùng tên card, không dùng `hw:2,0` vì số card đổi theo thứ tự
cắm. Thiết bị chạy native ở 48 kHz và Pipecat resample xuống sample rate của
OpenAI Realtime.

Kiểm tra mic ngay trên PC2 trước khi restart dịch vụ (thay `BOYALINK` bằng tên
card đang thấy ở `arecord -l`):

```bash
arecord -l
arecord -D plughw:CARD=BOYALINK,DEV=0 -f S16_LE -r 48000 -c 1 -d 5 /tmp/mic-pc2.wav
aplay /tmp/mic-pc2.wav
```

Nếu `arecord -l` không liệt kê card USB nào thì đó là lỗi phần cứng, không phải
lỗi cấu hình: xem `journalctl -k | tail` lúc cắm — không có dòng
`new high-speed USB device` nghĩa là dây/cổng/dongle chưa lên bus. Sau khi
ghi/phát thử thành công, restart `hb_voice.service`; không cần sửa high-level
hay bridge loa.
Template systemd đã thêm group `audio` để voice runtime truy cập `/dev/snd`.

## Điều khiển gesture bằng giọng nói

Voice có thể nhận các yêu cầu rõ ràng như “vẫy tay”, “tạo trái tim”, “bắt tay”,
“tạo dáng ngầu”, “thể hiện quyết tâm” và “tặng quà”. Tính năng này dùng tool
call của OpenAI Realtime, nên cần có phiên Realtime đang hoạt động. Chế độ giữ
nút `Select` và chế độ nghe liên tục đều dùng được.

Đường điều khiển chỉ gửi ID hành động qua `/run/hb/gesture_owner.sock`.
`controller` là chủ sở hữu duy nhất của player, ánh xạ ID sang slot trong
`controller/config/gestures.yaml`, rồi áp dụng các gate arm, locomotion,
đứng ổn định, yêu cầu di chuyển và teleop hiện tại. Gesture voice bị từ chối
nếu robot chưa arm/điều khiển bằng locomotion, teleop đang giữ thân trên, robot
chưa đứng yên đủ dwell, asset không có hoặc asset lặp vô hạn. Clip một lần tự
thu tay dựa trên frame/FPS và tốc độ slot đã cấu hình.

Mặc định voice chỉ trò chuyện. Khi `hb_integration` xác nhận `run_r1` đã arm,
remote còn sống và state là `LOCOMOTION`, voice tự quảng bá gesture tools cho
phiên Realtime đang chạy. Rời Dev locomotion, mất remote hoặc heartbeat sẽ gỡ
tools ngay và hủy gesture do voice đang chạy để owner thu tay. Vì vậy giữ
`off` trong `config/tuning.yaml`:

```yaml
gesture_control:
  mode: "off"
  socket: "/run/hb/gesture_owner.sock"
  timeout_s: 1.0
```

- `off`: giá trị khởi động an toàn; voice vẫn nhận mic, hiểu lời nói và giao
  tiếp bình thường. Trạng thái Dev locomotion quyết định lúc tool xuất hiện,
  không cần restart voice service.

Giữ socket path giống nhau ở hai nơi: `gesture_control.socket` của voice và
`gesture_voice_command_socket` trong `controller/config/gestures.yaml`.
Không thay `gesture_voice_auto_start` hoặc thao tác double-click A: cơ chế
gesture tự chạy khi loa đang phát vẫn là một tùy chọn độc lập, mặc định tắt.

### Nhận yêu cầu và lời thoại gesture

Tool khai báo rõ sáu ID bằng enum và mô tả tiếng Việt. Các lời mời như
“bắt tay với mình được không?”, “thả tim đi”, “vẫy tay chào mọi người đi” cũng
là yêu cầu thực hiện. Câu không rõ động tác phải hỏi lại, không tự chọn clip.
Sau khi owner chấp nhận, robot dùng lời giao tiếp như “Rất vui được gặp bạn
nha!”, không tường thuật “đang thực hiện lệnh” hay hứa đã hoàn thành.

Với ElevenLabs và gesture bật, văn bản được giữ đến `response.done`: nếu
phản hồi chứa tool call thì bỏ phần văn bản đi kèm, tránh đọc lời xác nhận
trước khi biết kết quả. Phản hồi hội thoại và phản hồi sau kết quả tool vẫn
được phát theo thứ tự. Đổi lại, TTS bắt đầu sau khi Realtime viết xong phản
hồi. Native OpenAI audio chỉ dùng hướng dẫn lời thoại, không qua bộ đệm này.

Khi mọi gesture đều không chạy, đọc log **trên robot**:

```bash
journalctl -u hb_voice.service -n 250 --no-pager
ls -l /run/hb/gesture_owner.sock
```

- `Realtime session confirmed tools=[...]` phải có `perform_gesture`.
- Có transcript đúng nhưng không có `Gesture tool call_id=...`: chưa đi tới
  handler; lời nói xác nhận của robot không chứng minh đã gửi gesture.
- `result_status=unavailable reason=gesture_owner_unavailable`: voice không
  liên lạc được với owner; chỉ deploy voice không bổ sung owner cho binary
  high-level cũ.
- `result_status=rejected reason=...`: đọc nguyên nhân như chưa locomotion,
  teleop đang giữ tay, robot đang di chuyển, chưa ổn định hoặc gesture đang bận.
- `result_status=accepted`: owner đã nhận khởi chạy, vẫn cần quan sát chuyển
  động và thu tay trên robot; test socket giả không chứng minh phần này.

Deploy riêng voice từ gốc repo trên máy dev:

```bash
./integration/scripts/deploy_stack.sh deploy-voice
```

Lệnh tự dò robot, cập nhật voice và restart `hb_voice.service`, không restart
high-level. Nếu owner chưa có, cần xử lý riêng high-level khi robot ở trạng
thái phù hợp; không bỏ các safety gate để ép gesture chạy.

## Đổi Wi-Fi hoặc địa chỉ IP

Wi-Fi chỉ cung cấp đường Internet và SSH/deploy. Khi robot sang Wi-Fi khác,
thông thường chỉ cần dùng IP mới:

```bash
ROBOT=unitree@<IP_MOI> ./integration/scripts/deploy_stack.sh status
```

Kết nối OpenAI dùng default route/DNS nên không lưu IP Wi-Fi trong source.

`eth10` và dải `192.168.123.x` là mạng nội bộ Unitree dành cho DDS, mic và loa;
nó độc lập với Wi-Fi và không được đổi thành `wlan0`. Chỉ khi tên card nội bộ
thực sự thay đổi mới sửa đồng thời:

- `controller/config/runtime.yaml`: `network_interface`.
- `/etc/hb/stack.env`: `UNITREE_NETWORK_INTERFACE`.

## Deploy

Chỉ thay đổi voice/tuning:

```bash
./integration/scripts/deploy_stack.sh diff
./integration/scripts/deploy_stack.sh deploy --restart-voice
./integration/scripts/deploy_stack.sh status
```

Sau deploy, kiểm tra trên robot:

```bash
sudo systemctl status hb_voice.service --no-pager
cat /run/hb/voice_status.env
sudo journalctl -u hb_voice.service -n 100 --no-pager
```

Log khởi động hợp lệ chứa
`tts=elevenlabs/eleven_flash_v2_5 clone_configured=1` và
`OpenAI Realtime session is ready`. Hướng dẫn đầy đủ nằm trong
[docs/voice.md](../docs/voice.md).

Có thay đổi policy hoặc code high-level: đưa robot về `DISARMED`, tư thế an
toàn, rồi deploy toàn stack:

```bash
./integration/scripts/deploy_stack.sh diff
./integration/scripts/deploy_stack.sh deploy --accept-policy  # nếu đổi policy
# Hoặc: ./integration/scripts/deploy_stack.sh deploy           # chỉ đổi code
./integration/scripts/deploy_stack.sh status
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
