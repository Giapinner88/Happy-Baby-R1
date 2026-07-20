# Kế Hoạch Khắc Phục Luồng Mic Unitree R1

## Mục tiêu

Khôi phục và kiểm chứng luồng hội thoại:

```text
Mic Unitree R1
  -> UDP multicast / Unitree SDK2
  -> unitree_bridge
  -> Pipecat InputAudioRawFrame
  -> OpenAI Realtime API
  -> loa Unitree R1
```

Kế hoạch này tập trung vào chẩn đoán và thứ tự khắc phục. Chưa thực hiện thay đổi
code cho đến khi xác định được checkpoint đầu tiên làm mất dữ liệu.

## Kết luận kỹ thuật

Không nên sửa Pipecat hoặc OpenAI trước. Dữ liệu đang mất ở phía trước Pipecat:

- RPC voice và chiều loa hoạt động.
- `GetVolume()` trả về thành công.
- Cả `r1_bridge` và example SDK2 chính thức không nhận được packet mic tại
  `239.168.123.161:5555`.
- Log Pipecat ghi nhận `robot_mic=0 bytes`, nên không có
  `InputAudioRawFrame` để gửi tới OpenAI.

Loa hoạt động không chứng minh mic hoạt động vì hai chiều dùng hai đường truyền
khác nhau:

| Chiều | Cơ chế |
|---|---|
| PC2 -> loa robot | DDS/RPC `AudioClient.PlayStream()` |
| Mic robot -> PC2 | UDP multicast `239.168.123.161:5555` |

Pipecat/OpenAI không báo lỗi khi không có audio input là hành vi bình thường:
OpenAI Realtime chỉ gửi `input_audio_buffer.append` khi nhận được
`InputAudioRawFrame`.

## Phát hiện cần giữ làm mốc

### Endpoint mic chính thức

SDK2 R1 hiện dùng:

```text
Group: 239.168.123.161
Port: 5555
Format: PCM signed 16-bit little-endian, 16000 Hz, mono
```

R1 audio client được Unitree thêm vào SDK chính thức ngày 10/06/2026 và source
hiện tại vẫn dùng endpoint này:

- `unitree_sdk2-main/example/r1/audio/r1_audio_client_example.cpp`
- <https://github.com/unitreerobotics/unitree_sdk2/commit/1a16684f32fb75dfd9c0aace012fc8126fb9794e>

Vì vậy không nên coi `239.168.123.161:5555` là endpoint cũ cho đến khi Unitree
xác nhận firmware R1 sử dụng endpoint khác.

### Loại bỏ stream `230.1.1.1:1720`

Không tiếp tục dùng stream này làm mic input. Phân tích artifact cho thấy:

- RTP version 2, payload type động 96.
- Sequence number tăng liên tiếp với cùng timestamp.
- Payload bắt đầu bằng `5c 81`, tiếp theo là `5c 01`, và kết thúc bằng `5c 41`.
- Đây là mẫu H.264 FU-A gồm start/middle/end fragment của một video frame.

Strip RTP header hoặc đảo endian rồi diễn giải payload này thành PCM sẽ chỉ tạo
ra nhiễu gần full-scale.

### Cấu hình runtime cần xác minh

File `.env` trong workspace hiện đặt:

```text
UNITREE_MIC_ENABLED=0
UNITREE_SPEAKER_ENABLED=0
```

Nếu PC2 chạy đúng file này thì `UnitreeMicBridge` không được tạo. Cần kiểm tra
environment hiệu lực của tiến trình trên PC2 thay vì suy luận từ bản local hoặc
từ việc loa được test thủ công.

## Kế hoạch thực hiện

### P0 — Chốt baseline trên PC2

1. Ghi lại các thông tin sau trong cùng một phiên test:

   - Phiên bản firmware R1.
   - Phiên bản và trạng thái voice/audio service.
   - Commit hoặc checksum của SDK2.
   - Checksum và thời gian build của `r1_bridge`.
   - Danh sách interface và địa chỉ IP trên PC2.

2. Xác minh environment hiệu lực:

   ```text
   UNITREE_MIC_ENABLED=1
   UNITREE_NETWORK_INTERFACE=eth10
   UNITREE_BRIDGE_PATH=<đường dẫn tuyệt đối tới r1_bridge>
   UNITREE_AUDIO_DEBUG=1
   UNITREE_DROP_BROWSER_AUDIO=1
   ```

3. Kiểm tra log có đủ hai dấu hiệu:

   - `Starting Unitree mic bridge...`
   - PID và command line thực tế chứa `mic eth10`.

4. Tạm dừng Pipecat, browser mic và các chương trình audio khác trước khi test
   trực tiếp SDK/bridge.

Điều kiện hoàn thành P0: xác nhận bridge mic thực sự chạy trên PC2 với đúng
interface và binary.

### P1 — Phân định robot không phát hay PC2 không nhận multicast

Thực hiện packet capture đồng thời với hai bài test độc lập:

1. `r1_audio_client_example eth10` từ SDK2 chính thức.
2. `r1_bridge mic eth10 10` từ bridge của ứng dụng.

Packet capture cần quan sát:

- IGMP join cho group `239.168.123.161`.
- UDP đích `239.168.123.161:5555` trên `eth10`.
- Toàn bộ interface bằng capture trên `any`.
- Nếu truy cập được, capture thêm ở PC1 hoặc máy đang chạy voice firmware.

Kết luận theo ma trận sau:

| Quan sát | Kết luận | Hướng tiếp theo |
|---|---|---|
| PC1 và PC2 đều không thấy UDP 5555 | Firmware/voice service không phát raw mic | Thực hiện P2 |
| PC1 thấy packet, PC2 không thấy | Multicast bị chặn trên đường mạng | Thực hiện P3 |
| PC2 thấy packet nhưng bridge ra 0 byte | Bridge bind/join sai interface hoặc socket | Kiểm tra bridge theo P3 |
| Bridge có raw nhưng file không nghe được | Sai codec/rate/channel hoặc payload không phải PCM | Xác định format trước khi dùng Pipecat |
| Bridge thu được audio hợp lệ | Tầng Unitree hoàn thành | Chuyển sang P4 |
| ASR topic có text nhưng raw mic không có | Voice pipeline sống, riêng raw export bị gate/tắt | Kiểm tra firmware/config với Unitree |

Điều kiện hoàn thành P1:

- Khoảng `160000` byte cho 5 giây PCM 16 kHz mono 16-bit.
- File WAV nghe được và có biên độ thay đổi theo lời nói.
- Không chấp nhận chỉ tiêu “file có dung lượng” nếu nội dung là nhiễu hoặc video
  payload.

### P2 — Kiểm tra firmware và voice service

1. So khớp firmware robot với SDK2 có R1 audio client được phát hành tháng
   06/2026.
2. Sau khi lưu log và packet capture, cold reboot robot hoặc restart voice
   service theo quy trình chính thức của Unitree.
3. Kiểm tra:

   - Mute vật lý của mic.
   - Quyền riêng tư hoặc cấu hình mic trong ứng dụng Unitree.
   - Trạng thái voice/ASR service.
   - Chế độ vận hành robot có ảnh hưởng tới voice service hay không.

4. Thử wake word như một phép A/B ngắn, không coi đây là nguyên nhân đã được
   xác nhận.
5. Không tự gọi API ID `1002` để “start recording” khi chưa có schema request
   chính thức. `AudioClient::Init()` chỉ đăng ký API này và SDK không cung cấp
   wrapper khởi động mic.
6. Nếu official example vẫn treo ở `start record!`, gửi Unitree một gói bằng
   chứng gồm:

   - Firmware và voice service version.
   - SDK commit/checksum.
   - `GetVolume ret=0`.
   - Log official example.
   - PCAP tại PC1 và PC2.
   - Kết quả subscribe `rt/audio_msg`.
   - Bằng chứng mic vẫn hoạt động trong ứng dụng nội bộ của robot.

Điều kiện hoàn thành P2: robot phát raw mic multicast hoặc Unitree xác nhận
endpoint/config/firmware đúng cho model robot đang sử dụng.

### P3 — Kiểm tra multicast và interface trên PC2

Nếu phía robot có phát nhưng PC2 không nhận, kiểm tra:

1. `eth10` có đúng IP `192.168.123.164/24` và cờ `MULTICAST`.
2. Multicast route cho `239.168.123.161` đi qua `eth10`.
3. IGMP membership đã xuất hiện trên `eth10`.
4. `nftables`, `iptables`, `rp_filter` và network namespace của tiến trình.
5. IGMP snooping/querier trên switch hoặc Linux bridge giữa PC1 và PC2.
6. Có nhiều interface cùng mang địa chỉ `192.168.123.x` hay không.

`r1_bridge.cpp` hiện không ánh xạ trực tiếp từ tên `eth10` sang địa chỉ local của
socket. Nó chọn interface đầu tiên có IP bắt đầu bằng `192.168.123.`. Nếu có
nhiều interface phù hợp, bridge có thể join multicast qua sai NIC dù DDS vẫn
được khởi tạo với `eth10`.

Chỉ lên kế hoạch sửa bridge sau khi packet capture chứng minh vấn đề nằm tại
bind/join. Hướng sửa dự kiến khi được phép code:

- Chọn địa chỉ bằng đúng tên interface được truyền vào.
- Bind socket vào đúng device hoặc interface index.
- Kiểm tra và log đầy đủ kết quả `bind()`/`setsockopt()`.
- Log source IP, packet count, byte rate và packet loss.

### P4 — Xác minh Pipecat và OpenAI theo từng checkpoint

Chỉ bắt đầu bước này sau khi `r1_bridge` tạo được PCM hợp lệ.

Theo dõi dữ liệu theo thứ tự:

```text
UDP packet
  -> r1_bridge stdout
  -> resampler 16 kHz -> 24 kHz
  -> InputAudioRawFrame
  -> OpenAI input_audio_buffer.append
  -> speech_started / speech_stopped
  -> transcription
  -> response audio
```

Tiêu chí tại mỗi biên:

| Checkpoint | Tiêu chí |
|---|---|
| `r1_bridge` stdout | PCM signed 16-bit LE, mono, 16 kHz |
| Sau resample | PCM16 mono 24 kHz, xấp xỉ 48000 byte/giây |
| Pipecat | `robot_mic` lớn hơn 0 và tăng đều |
| OpenAI WebSocket | Có `input_audio_buffer.append` |
| Turn detection | Có `speech_started` và `speech_stopped` |
| Kết quả | Có transcript và response |

Lưu ý:

- `OpenAIRealtimeLLMService` không tự resample, downmix hoặc chuyển codec; nó
  gửi nguyên `frame.audio`.
- PCM session của OpenAI dùng 24 kHz. Mọi nguồn phải được chuẩn hóa cùng format
  trước khi đến service.
- Không trộn browser mic trong giai đoạn kiểm thử.
- `InputAudioNoiseReduction(type="near_field")` có thể không phù hợp với mic
  gắn trên thân robot. Chỉ thử `far_field`, gain và VAD sau khi xác nhận byte
  audio thực sự tới OpenAI.
- Nếu `robot_mic=0`, không tiếp tục điều chỉnh VAD hoặc prompt vì chúng nằm sau
  checkpoint bị lỗi.

### P5 — Ổn định hóa sau khi chạy được

1. Chuyển bridge mic sang chạy liên tục thay vì restart theo segment 5 giây.
2. Thêm health criteria cho:

   - Byte rate.
   - Thời gian không có packet.
   - Packet loss hoặc sequence discontinuity.
   - Subprocess exit/restart.
   - OpenAI append và turn events.

3. Chạy soak test tối thiểu 15–30 phút.
4. Kiểm tra echo từ loa robot lọt lại mic robot. Tùy kết quả, áp dụng AEC,
   ducking hoặc chính sách barge-in/mute input trong lúc bot nói.
5. Chỉ bật lại browser mic khi có cơ chế chọn nguồn rõ ràng; không trộn hai mic
   vào cùng một turn.

## Tiêu chí nghiệm thu cuối

- Thu được PCM mic robot ổn định từ endpoint chính thức.
- Pipecat ghi nhận khoảng 48000 byte/giây sau resample 24 kHz.
- OpenAI nhận append liên tục và phát hiện đúng đầu/cuối lượt nói.
- Có transcript tiếng Việt và phản hồi qua loa robot.
- Không có restart bridge, mất audio kéo dài hoặc vòng lặp echo trong bài test
  15–30 phút.

## Phương án dự phòng

Nếu firmware R1 không hỗ trợ raw mic export ổn định:

1. Dùng USB Audio Class mic nối trực tiếp PC2.
2. Hoặc dùng browser/laptop mic làm input và loa robot làm output.
3. Giữ interface nguồn mic riêng biệt để sau này thay lại bằng mic tích hợp mà
   không thay đổi phần OpenAI/Pipecat phía sau.

## File liên quan

- `unitree_sdk2-main/example/r1/audio/r1_audio_client_example.cpp`
- `unitree_sdk2-main/include/unitree/robot/r1/audio/audio_client.hpp`
- `ai_modules/voice_interaction/unitree_bridge/r1_bridge.cpp`
- `ai_modules/voice_interaction/unitree_mic.py`
- `ai_modules/voice_interaction/unitree_speaker.py`
- `ai_modules/voice_interaction/bot.py`
- `src/pipecat/services/openai/realtime/llm.py`
- `src/pipecat/services/openai/realtime/events.py`
