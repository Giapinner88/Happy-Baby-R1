# Runbook vận hành hội thoại bằng mic Unitree R1

## 1. Trạng thái đã xác nhận

Luồng sau đã hoạt động thực tế:

```text
Mic Unitree R1
  -> UDP multicast 239.168.123.161:5555
  -> r1_bridge (PCM16, 16 kHz, mono)
  -> UnitreeMicBridge (resample 24 kHz)
  -> OpenAI Realtime API
  -> UnitreeSpeakerBridge
  -> loa robot
```

Các kết quả đo đã xác nhận:

- 25 giây mic tạo `803840` byte raw; mức lý thuyết là khoảng `800000` byte.
- 10 giây kiểm tra duy trì tạo `322560` byte; mức lý thuyết là khoảng `320000` byte.
- File ghi âm nghe được đúng giọng nói.
- Mức âm thanh đo được: `mean_volume=-23.7 dB`, `max_volume=-5.0 dB`.
- OpenAI đã nhận giọng nói từ mic robot và hội thoại thành công.

## 2. Vì sao các lần kiểm tra trước không hoạt động

Có hai nguyên nhân độc lập xảy ra nối tiếp nhau.

### 2.1. Robot chưa phát luồng mic

Ban đầu PC2 join đúng multicast nhưng robot không gửi UDP tới
`239.168.123.161:5555`:

- `GetVolume ret=0`: voice RPC và đường mạng vẫn hoạt động.
- IGMP join/leave xuất hiện đúng trên `eth10`.
- Official `r1_audio_client_example` dừng tại `start record!`.
- Packet capture có `0` gói UDP mic.

Sau khi kích hoạt chức năng mic/voice gốc của robot, robot bắt đầu phát UDP mic.
Firmware hiện tại có trạng thái **mic exporter bị gate**: bridge chỉ nhận được
audio khi chức năng giao tiếp gốc đã kích hoạt publisher. Publisher có thể dừng
lại khi thoát chế độ giao tiếp hoặc sau một khoảng thời gian, vì vậy nên giữ chế
độ này hoạt động trong suốt phiên Pipecat.

### 2.2. Pipecat nhận byte nhưng tự loại bỏ frame mic robot

`UnitreeMicBridge` đưa audio robot vào hàng đợi dưới dạng
`InputAudioRawFrame`. Frame này quay lại `process_frame()` của chính processor.
Logic hiện tại coi mọi `InputAudioRawFrame` downstream là audio trình duyệt.

Khi cấu hình:

```dotenv
UNITREE_DROP_BROWSER_AUDIO=1
```

được bật, frame mic robot cũng bị `return` trước khi tới OpenAI. Vì bộ đếm debug
được tăng trước điểm loại bỏ nên log vẫn báo `robot_mic > 0`, gây cảm giác rằng
OpenAI đã nhận audio trong khi thực tế chưa nhận.

Workaround đã kiểm chứng là:

```dotenv
UNITREE_DROP_BROWSER_AUDIO=0
```

Sau đó phải mute mic trình duyệt để tránh trộn hai nguồn audio.

### 2.3. Biến môi trường truyền trước command bị `.env` ghi đè

Ứng dụng gọi:

```python
load_dotenv(override=True)
```

Vì vậy lệnh dạng:

```bash
UNITREE_SPEAKER_ENABLED=1 uv run python bot.py ...
```

không thắng giá trị đã có trong `.env`. Nếu `.env` vẫn đặt speaker bằng `0`,
pipeline chỉ phát phản hồi ra trình duyệt. Cần sửa trực tiếp `.env` rồi khởi động
lại bot.

### 2.4. `aplay` không điều khiển loa robot

`aplay` chỉ dùng ALSA của PC2. Loa Unitree nhận dữ liệu qua
`AudioClient.PlayStream`, vì vậy muốn phát file thử qua loa robot phải dùng
`r1_bridge speaker`.

## 3. Yêu cầu bắt buộc

### Robot và mạng

- Robot ở trạng thái an toàn và voice/audio service đã khởi động xong.
- Mic vật lý không bị mute; quyền riêng tư/voice trong ứng dụng Unitree cho phép
  sử dụng mic.
- PC2 có `eth10` với địa chỉ `192.168.123.164/24` và cờ multicast.
- Thiết bị voice `192.168.123.161` truy cập được từ PC2.
- Binary bridge tồn tại và chạy được tại:
  `/home/unitree/HappyBaby/ai_modules/voice_interaction/unitree_bridge/build/r1_bridge`.

### Ứng dụng

- `.env` có `OPENAI_API_KEY` hợp lệ.
- Máy có kết nối Internet tới OpenAI Realtime API.
- Trình duyệt kết nối được tới Pipecat UI tại cổng `7860`.
- Mic trình duyệt phải được mute khi dùng workaround hiện tại.

## 4. Cấu hình `.env` chuẩn hiện tại

```dotenv
HOST=0.0.0.0
PORT=7860

UNITREE_NETWORK_INTERFACE=eth10
UNITREE_BRIDGE_PATH=/home/unitree/HappyBaby/ai_modules/voice_interaction/unitree_bridge/build/r1_bridge
UNITREE_MIC_ENABLED=1
UNITREE_SPEAKER_ENABLED=1
UNITREE_AUDIO_APP_NAME=pipecat
UNITREE_AUDIO_DEBUG=1

# Workaround bắt buộc cho logic phân loại frame hiện tại.
UNITREE_DROP_BROWSER_AUDIO=0

# 0 = giữ một bridge mic chạy liên tục, tránh restart mỗi 5 giây.
UNITREE_MIC_SEGMENT_SECS=0
```

Không ghi API key thật vào tài liệu hoặc log chia sẻ.

### 4.1. Mở Pipecat UI bằng Chromium profile riêng

Đây là cách vận hành đã chọn cho mạng thử nghiệm hiện tại. Chạy trên **laptop**,
không chạy trong phiên SSH của PC2:

```bash
chromium \
  --unsafely-treat-insecure-origin-as-secure=http://192.168.12.2:7860 \
  --user-data-dir=/tmp/chromium-r1 \
  http://192.168.12.2:7860/client/
```

Không thêm `--app` và không đổi profile sang thư mục cache khác. Đóng cửa sổ
Chromium R1 cũ trước khi chạy. Chọn **Allow** nếu Chromium hỏi quyền microphone,
sau đó mute mic trình duyệt khi SmallWebRTC đã kết nối.

Chỉ mở Pipecat UI trong profile này; không đăng nhập tài khoản cá nhân hoặc
duyệt website khác bằng cửa sổ đó.

## 5. Quy trình khởi động hằng ngày

Publisher mic của firmware có thể tự tắt sau một khoảng thời gian. Kết quả duy
trì 10 giây chỉ chứng minh publisher tiếp tục chạy ngắn hạn, không đảm bảo giữ
trạng thái tới phiên bot sau. Thứ tự bắt buộc là **bridge join trước, kích hoạt
mic gốc sau**.

### Bước 1 — Kiểm tra cấu hình hiệu lực

```bash
cd /home/unitree/HappyBaby/ai_modules/voice_interaction
grep -nE \
  'UNITREE_(NETWORK_INTERFACE|BRIDGE_PATH|MIC_ENABLED|SPEAKER_ENABLED|DROP_BROWSER_AUDIO|MIC_SEGMENT_SECS|AUDIO_DEBUG)' \
  .env
```

Không dùng biến môi trường đặt trước command để ghi đè `.env`, vì ứng dụng đang
load `.env` với `override=True`.

`UNITREE_MIC_SEGMENT_SECS` phải bằng `0` để giữ một IGMP membership liên tục.

### Bước 2 — Khởi động server bot

Đảm bảo không có phiên bot hoặc bridge cũ:

```bash
pgrep -af 'bot.py|r1_bridge'
```

Nếu không có tiến trình cũ, khởi động:

```bash
cd /home/unitree/HappyBaby/ai_modules/voice_interaction
uv run python bot.py -t webrtc --host 0.0.0.0 --port 7860
```

Lúc này server chỉ phục vụ UI. Pipeline và `UnitreeMicBridge` chưa được tạo cho
tới khi một WebRTC client kết nối.

### Bước 3 — Mở UI và tạo bridge mic

1. Trên laptop, chạy lệnh Chromium profile riêng ở Mục 4.1.
2. Bấm Connect với SmallWebRTC.
3. Mute microphone của trình duyệt sau khi kết nối.
4. Chờ terminal bot xuất hiện:

```text
joining mic multicast 239.168.123.161:5555 via local ip: 192.168.123.164
```

Với segment bằng `0`, command của bridge chỉ có `mic eth10`, không có số `5`.
Nếu chưa có UDP, giữ bot chạy; không dừng bridge.

### Bước 4 — Kích hoạt mic khi bridge đang join

Trong khi terminal bot vẫn chạy và bridge đang chờ:

1. Mở chức năng mic/voice/ghi âm gốc của robot.
2. Bắt đầu một lượt ghi âm hoặc voice và nói trong 2–5 giây.
3. Quan sát terminal bot cho tới khi xuất hiện `robot_mic > 0`.
4. Giữ chức năng giao tiếp gốc hoạt động trong suốt phiên hội thoại Pipecat.

Nếu không xuất hiện `robot_mic`, xác minh chức năng gốc có tạo được một bản ghi
mới nghe được. Nếu chức năng gốc cũng không thu được tiếng, kiểm tra mute hoặc
thực hiện cold reboot theo quy trình an toàn của Unitree.

### Bước 5 — Hội thoại

1. Giữ UI, bot và bridge đang chạy.
2. Không phát file WAV thử trong khi bot đang nghe.
3. Nói trực tiếp vào mic robot.

Log đạt yêu cầu:

- Có `UnitreeMicBridge` trong pipeline.
- Có `UnitreeSpeakerBridge` giữa OpenAI và output transport.
- `robot_mic` tăng liên tục.
- Có user turn/transcript sau khi nói.
- Không có bridge return code `3` hoặc thông báo không nhận packet.

Health check ghi file chỉ dùng khi chẩn đoán, không đặt trong quy trình khởi
động thường ngày vì nó tạo thêm một chu kỳ join/leave multicast.

## 6. Dừng hệ thống

1. Ngắt kết nối UI.
2. Nhấn `Ctrl+C` tại terminal chạy bot.
3. Xác nhận không còn tiến trình:

```bash
pgrep -af 'bot.py|r1_bridge'
```

Không dừng `master_service`, `run_r1` hoặc các dịch vụ điều khiển robot để xử lý
lỗi audio.

## 7. Xử lý nhanh khi có lỗi

| Hiện tượng                                  | Nguyên nhân thường gặp                            | Xử lý                                                     |
| ---------------------------------------------- | ------------------------------------------------------ | ----------------------------------------------------------- |
| `GetVolume ret=0` nhưng raw bằng `0`     | Mic exporter chưa được kích hoạt                 | Chạy chức năng mic gốc một lần rồi health check lại |
| `robot_mic > 0` nhưng không có transcript | `UNITREE_DROP_BROWSER_AUDIO=1` làm rơi frame robot | Đặt bằng`0`, restart bot và mute mic trình duyệt    |
| Trả lời chỉ phát trên laptop              | Speaker đang tắt trong`.env`                       | Đặt`UNITREE_SPEAKER_ENABLED=1` rồi restart bot         |
| Có cả giọng laptop và robot                | Browser mic chưa mute                                 | Mute mic trong Pipecat UI/trình duyệt                     |
| Loa robot nhỏ                                 | Volume robot thấp                                     | Dùng`./r1_bridge volume eth10 60` hoặc `70`           |
| Bridge restart mỗi 5 giây                    | `UNITREE_MIC_SEGMENT_SECS=5`                         | Đặt bằng`0` để chạy liên tục                      |
| `aplay` không phát qua loa robot           | Loa robot không phải ALSA device                     | Dùng`r1_bridge speaker`                                  |

## 8. Việc cần làm để bỏ workaround

Để vận hành hoàn toàn tự động và không phải mute mic trình duyệt thủ công, cần
một thay đổi code riêng:

1. Phân biệt rõ frame do robot tự chèn với frame đến từ WebRTC.
2. Chỉ áp dụng `drop_browser_audio` cho frame WebRTC.
3. Giữ mic bridge chạy liên tục thay vì chia segment.
4. Thêm health state: `publisher_not_started`, `receiving`, `silent_audio`,
   `OpenAI_audio_sent`.
5. Xác minh cơ chế chính thức để kích hoạt raw mic exporter từ Unitree; không tự
   gọi API `1002` khi chưa có schema do Unitree cung cấp.

Cho tới khi có thay đổi này, quy trình ở Mục 5 là cách vận hành đã được kiểm
chứng.
