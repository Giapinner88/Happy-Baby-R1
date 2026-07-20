# Hướng Dẫn Toàn Diện: Robot Hạnh Phúc R1 Voice Pipeline

Tài liệu tổng hợp toàn bộ kiến trúc, cách kết nối, và cách vận hành app `voice_r1`
trên Robot Unitree R1, dựa trên `unitree_sdk2-main/R1_SDK2_GIAI_THICH.md`,
`unitree_sdk2-main/R1_MIC_TEST_GUIDE.md`, và các lần test thực tế trên robot.

## 1. Kiến trúc PC1 / PC2

SDK không định nghĩa cứng "PC1"/"PC2" — đây là cách phân vai vận hành, không phải
khái niệm trong code. Cả hai đều là DDS participant trên cùng mạng robot
(domain 0), khác nhau ở trách nhiệm:

```text
PC1: motion/control
  - Chạy service/controller chuyển động sẵn có của robot
  - Nếu cần low-level (rt/lowcmd), chỉ PC1 được publish
  - Theo dõi rt/lowstate, rt/secondary_imu

PC2: voice/AI/application  <-- đây là nơi voice_r1 chạy
  - Chạy Pipecat + OpenAI Realtime + unitree_bridge
  - Chỉ gọi high-level service: AudioClient (speaker/TTS/ASR/LED/volume),
    LocoClient (whitelist action: stand_up, stop_move, damp, start, zero_torque)
  - Không publish rt/lowcmd
```

Nguyên tắc quan trọng: không để hai tiến trình cùng gửi lệnh motion tranh chấp.
`voice_r1` chỉ dùng high-level service nên an toàn để chạy song song với PC1.

## 2. Sơ đồ mạng thực tế (đã xác nhận khi test)

PC2 (`unitree@ubuntu`) có các interface:

| Interface | IP | Vai trò |
|---|---|---|
| `eth10` | `192.168.123.164/24` | Mạng nội bộ robot — DDS + mic multicast. **Đây là interface truyền vào SDK/bridge.** |
| `eth0` | DOWN | Không dùng được |
| `wlan0` | `192.168.12.2/24` | WiFi — cùng mạng với laptop điều khiển |
| `tailscale0` | `100.82.165.36` | VPN mesh, chưa xác nhận dùng được từ laptop hiện tại |

Laptop điều khiển (máy chạy phiên làm việc này) có `wlp0s20f3` = `192.168.12.195/24`
— **cùng subnet WiFi với PC2** (`192.168.12.0/24`), nên có thể truy cập PC2 trực
tiếp qua `192.168.12.2` mà không cần SSH tunnel.

```text
Laptop (192.168.12.195) --WiFi LAN--> PC2 (192.168.12.2, unitree@ubuntu)
                                          |
                                      eth10 (192.168.123.164)
                                          |
                                    Robot DDS network (domain 0)
                                    - rt/api/voice/*  (AudioClient: TTS, speaker, volume, LED)
                                    - rt/api/sport/*  (LocoClient: high-level action)
                                    - 239.168.123.161:5555 (mic UDP multicast)
```

**Quan trọng — bài học đã gặp:**

- `eth0` tưởng là interface đúng theo default trong doc/script cũ, nhưng thực tế
  bị DOWN trên máy này. Interface thật là `eth10`. Luôn tự kiểm tra bằng
  `ip -br a` trên chính PC2, đừng tin theo giá trị mặc định trong script.
- Muốn mở giao diện web (`http://<ip>:7860/client/`) từ laptop để test WebRTC,
  **phải vào bằng IP LAN thật của PC2** (`http://192.168.12.2:7860/client/`),
  **không dùng SSH port-forward tới `localhost`**. SSH tunnel chỉ chuyển tiếp
  HTTP (tải trang, offer/answer signaling); audio WebRTC cần UDP trực tiếp
  giữa trình duyệt và PC2, tunnel không làm được, nên sẽ treo ở "connecting/loading".
- `aplay`/`play` chạy qua SSH sẽ phát âm thanh ra loa của **máy đang chạy lệnh**
  (PC2), không phải loa laptop. Muốn nghe trên laptop, phải `scp` file `.wav`
  về laptop trước rồi phát ở đó.

## 3. Mic robot

Robot **không** expose mic qua file device kiểu `/dev/snd/...`. Mic phát ra UDP
multicast:

```text
Group IP: 239.168.123.161
Port: 5555
Format: PCM 16 kHz, mono, int16 little-endian
```

Trước khi mic multicast có dữ liệu, bắt buộc phải có ít nhất 1 lần gọi RPC tới
voice service (ví dụ `AudioClient.GetVolume()`) để handshake — nếu bỏ qua, socket
join multicast thành công nhưng không nhận được gói nào.

Ngoài raw audio, robot còn có thể phát ASR (speech-to-text) dạng text qua DDS
topic `rt/audio_msg` (kiểu `std_msgs::msg::dds_::String_`), nếu firmware/voice
service hỗ trợ.

**Cách test thủ công (trên PC2, đã xác nhận hoạt động):**

```bash
cd ~/HappyBaby/ai_modules/voice_interaction/unitree_bridge/build
./r1_bridge mic eth10 5 > /tmp/r1_mic.raw
ffmpeg -y -f s16le -ar 16000 -ac 1 -i /tmp/r1_mic.raw /tmp/r1_mic.wav
```

Copy về laptop để nghe (không `aplay` trực tiếp trên PC2 qua SSH):

```bash
scp unitree@192.168.12.2:/tmp/r1_mic.wav ./r1_mic_test.wav
aplay ./r1_mic_test.wav        # hoặc phát bằng trình phát nhạc bất kỳ trên laptop
```

Kết quả đã xác nhận: file ~163 KB cho 5 giây (khớp `16000 samples/s * 2 bytes * 5s
≈ 160000 bytes`), nghe rõ giọng nói thật khi phát trên laptop.

## 4. Loa robot

Loa robot nhận audio qua RPC `AudioClient.PlayStream(app_name, stream_id, chunk)`
(service `voice`, api id `1003`), yêu cầu PCM 16-bit, 16 kHz, mono. Có
`PlayStop(stream_id)` để dừng phát.

Test thủ công:

```bash
./r1_bridge volume eth10 90
./r1_bridge tts eth10 "Xin chao, toi la Robot Hanh Phuc R1" 1
```

## 5. `unitree_bridge/r1_bridge` — lớp trung gian dùng chung

File: `ai_modules/voice_interaction/unitree_bridge/r1_bridge.cpp`. Một binary C++ duy nhất, chọn
chức năng qua mode đầu tiên trên command line:

| Mode | Cú pháp | Việc làm |
|---|---|---|
| `mic` | `r1_bridge mic <iface> [seconds]` | Join UDP multicast mic, in raw PCM ra stdout. `seconds=0` hoặc bỏ trống → chạy liên tục (dùng cho streaming sống). |
| `speaker` | `r1_bridge speaker <iface> [app_name]` | Đọc PCM từ stdin, gọi `PlayStream` liên tục lên loa robot. |
| `asr` | `r1_bridge asr <iface>` | Subscribe `rt/audio_msg`, in text ASR ra stdout. |
| `tts` | `r1_bridge tts <iface> "<text>" [speaker_id]` | Gọi `TtsMaker`. |
| `volume` | `r1_bridge volume <iface> <0-100>` | `SetVolume`. |
| `led` | `r1_bridge led <iface> <r> <g> <b>` | `LedControl`. |
| `action` | `r1_bridge action <iface> <name>` | Gọi `LocoClient` whitelist: `stand_up`, `stop_move`, `damp`, `start`, `zero_torque`. |

Build (trên PC2, kiến trúc aarch64):

```bash
cd ~/HappyBaby/ai_modules/voice_interaction/unitree_bridge
mkdir -p build && cd build
cmake .. -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2-main
make -j$(nproc)
```

## 6. Pipeline Pipecat (`ai_modules/voice_interaction/bot.py`) — cách hoạt động

```text
transport.input()  (WebRTC mic từ trình duyệt, nếu có client kết nối)
        |
[UnitreeMicBridge]   <- CHỈ chạy nếu UNITREE_MIC_ENABLED=1
        |               subprocess "r1_bridge mic <iface>" chạy liên tục,
        |               đọc PCM 16kHz, resample lên 24kHz (bắt buộc — OpenAI
        |               Realtime cố định 24kHz), đẩy InputAudioRawFrame
        |               xuống pipeline song song với audio từ WebRTC.
        v
user_aggregator (LLMContextAggregatorPair, realtime_service_mode=True)
        v
llm (OpenAIRealtimeLLMService — nhận audio, ASR + trả lời bằng giọng nói)
        v
[UnitreeSpeakerBridge]  <- CHỈ chạy nếu UNITREE_SPEAKER_ENABLED=1
        |                  subprocess "r1_bridge speaker <iface>", nhận
        |                  OutputAudioRawFrame, resample về 16kHz, ghi vào
        |                  stdin -> loa robot. Frame vẫn tiếp tục đi xuống,
        |                  browser vẫn nghe được để debug.
        v
transport.output()  (phát lại ra trình duyệt, dùng để debug)
        v
assistant_aggregator
```

Các file liên quan trong `ai_modules/voice_interaction/`:

| File | Vai trò |
|---|---|
| `bot.py` | Lắp pipeline, đọc `.env`, xử lý sự kiện client connect/disconnect |
| `unitree_mic.py` | `UnitreeMicBridge` — mic robot → pipeline (input) |
| `unitree_speaker.py` | `UnitreeSpeakerBridge` — pipeline → loa robot (output) |
| `unitree_actions.py` | `UnitreeActionBridge` — whitelist action an toàn, có cooldown |
| `test_mic.py` | Script test tay: ghi N giây mic robot ra file `.wav`, in số byte nhận được trực tiếp |
| `unitree_bridge/r1_bridge.cpp` | Binary C++ trung gian, nói chuyện trực tiếp với SDK |
| `prompt_robot_hanh_phuc_r1.txt` | System prompt / persona cho OpenAI Realtime |

## 7. Cấu hình `.env`

```env
OPENAI_API_KEY=...
OPENAI_REALTIME_MODEL=gpt-realtime
OPENAI_VOICE=echo

HOST=0.0.0.0
PORT=7860

UNITREE_NETWORK_INTERFACE=eth10          # interface thật trên PC2, KHÔNG phải eth0
UNITREE_BRIDGE_PATH=./unitree_bridge/build/r1_bridge
UNITREE_MIC_ENABLED=0                    # 1 = dùng mic robot làm input thay vì chỉ mic browser
UNITREE_SPEAKER_ENABLED=0                # 1 = trả lời phát ra loa robot
UNITREE_ACTIONS_ENABLED=0                # 1 = cho OpenAI gọi action whitelist
UNITREE_ACTION_COOLDOWN_SECS=2.0
UNITREE_AUDIO_APP_NAME=pipecat
```

Bật `UNITREE_MIC_ENABLED=1` + `UNITREE_SPEAKER_ENABLED=1` để có luồng hội thoại
đầy đủ qua phần cứng robot (mic robot → OpenAI → loa robot), giống hệt kịch bản
nói chuyện qua laptop nhưng input/output đều là robot.

## 8. Cách chạy — trên robot (PC2)

```bash
cd ~/HappyBaby/ai_modules/voice_interaction
# sửa .env: UNITREE_MIC_ENABLED=1, UNITREE_SPEAKER_ENABLED=1
uv sync
./run.sh
```

Log kỳ vọng: `🚀 Bot ready! (WebRTC)` và `Uvicorn running on http://0.0.0.0:7860`.

## 9. Cách truy cập — từ laptop

Nếu laptop cùng mạng WiFi với PC2 (đã xác nhận: `192.168.12.0/24`):

```text
http://192.168.12.2:7860/client/
```

Mở trực tiếp bằng IP LAN này, **không SSH port-forward**. Bấm Connect để khởi
động session (đây là bước bắt buộc để pipeline nhận `StartFrame` và chạy) — sau
đó nói vào mic robot (không nói vào mic laptop, tránh lẫn 2 nguồn audio cùng
một turn của OpenAI Realtime).

## 10. Troubleshooting đã gặp

| Triệu chứng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `eth0: does not match an available interface` | Sai tên interface | `ip -br a` trên PC2, dùng interface có IP `192.168.123.x` (ở đây là `eth10`) |
| File raw mic 0 byte | Chưa handshake RPC voice service, hoặc sai interface | Dùng `r1_bridge mic` (đã có `GetVolume()` handshake sẵn), kiểm tra lại interface |
| `aplay` chạy nhưng "không nghe thấy gì" | Audio phát ra loa của máy chạy lệnh (PC2), không phải laptop | `scp` file `.wav` về laptop rồi phát ở đó |
| Trang `client/` load được nhưng bấm Connect bị treo "loading" mãi | Vào bằng `localhost` qua SSH port-forward — WebRTC audio cần UDP trực tiếp, tunnel không hỗ trợ | Vào thẳng bằng IP LAN của PC2 (`http://192.168.12.2:7860/client/`) |
| `ffmpeg` hỏi "Overwrite? [y/N]" rồi treo | File output đã tồn tại, chạy qua pipe không tương tác được | Thêm `-y`, hoặc `rm` file cũ trước |

## 11. Việc còn cần tự kiểm chứng trên phần cứng (chưa test được từ xa)

- `r1_bridge mic eth10` chạy liên tục (không giới hạn giây, dùng khi
  `UNITREE_MIC_ENABLED=1`) có ổn định lâu dài không — theo dõi log, xem có bị
  rớt kết nối/subprocess chết sau vài phút không.
- Độ trễ thực tế của luồng mic robot → resample → OpenAI Realtime → loa robot.
- Nếu vô tình vẫn còn nói vào mic laptop lúc `UNITREE_MIC_ENABLED=1`, audio hai
  nguồn có gây lẫn/nhiễu câu trả lời không (theo thiết kế hiện tại là cộng dồn,
  không có cơ chế mute mic browser tự động).
