# Unitree R1 Voice Interaction

Ứng dụng hội thoại thời gian thực cho Robot Hạnh Phúc Unitree R1, sử dụng
Pipecat, OpenAI Realtime API và mic/loa tích hợp của robot.

## Luồng âm thanh

```text
Mic Unitree R1
  -> UDP multicast 239.168.123.161:5555 (PCM16, 16 kHz, mono)
  -> r1_bridge
  -> UnitreeMicBridge (resample 24 kHz)
  -> Pipecat
  -> OpenAI Realtime API
  -> UnitreeSpeakerBridge
  -> loa robot
```

SmallWebRTC client vẫn cần thiết để tạo phiên Pipecat. Khi dùng mic robot, hãy
mute mic trình duyệt để không trộn hai nguồn âm thanh.

## Thành phần

- `bot.py`: pipeline Pipecat và OpenAI Realtime.
- `unitree_mic.py`: nhận PCM từ bridge và đưa vào pipeline.
- `unitree_speaker.py`: phát audio phản hồi qua loa Unitree.
- `unitree_actions.py`: các lệnh hành động mức cao, mặc định tắt.
- `unitree_bridge/`: bridge C++ dùng Unitree SDK2.
- `test_mic.py`: kiểm tra độc lập luồng mic.
- `prompt_robot_hanh_phuc_r1.txt`: persona và system prompt.
- `docs/`: hướng dẫn vận hành và khắc phục sự cố.

## Yêu cầu

- Chạy trên PC2 của robot, có `eth10` ở mạng `192.168.123.0/24`.
- Unitree SDK2 và CycloneDDS đã được cài đặt.
- `uv`, CMake và trình biên dịch C++.
- OpenAI API key có billing/quota cho Realtime API.
- Chế độ giao tiếp gốc của robot có thể phải được giữ hoạt động để firmware
  phát luồng mic multicast.

## Build bridge trên PC2

```bash
cd ai_modules/voice_interaction/unitree_bridge
mkdir -p build
cd build
cmake .. -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2-main
make -j"$(nproc)"
```

Kiểm tra bridge:

```bash
./r1_bridge volume eth10 70
./r1_bridge tts eth10 "Xin chào, tôi là Robot Hạnh Phúc R1" 1
./r1_bridge mic eth10 10 > /tmp/r1_mic.raw
```

Nếu lệnh mic báo không có packet, hãy bật chế độ giao tiếp/voice gốc của robot
trong khi bridge đang chờ rồi thử lại.

## Cấu hình

```bash
cd ai_modules/voice_interaction
cp .env.example .env
```

Điền `OPENAI_API_KEY` vào `.env`. Không commit file `.env`.

Cấu hình Unitree quan trọng:

```dotenv
UNITREE_NETWORK_INTERFACE=eth10
UNITREE_BRIDGE_PATH=./unitree_bridge/build/r1_bridge
UNITREE_MIC_ENABLED=1
UNITREE_SPEAKER_ENABLED=1
UNITREE_DROP_BROWSER_AUDIO=0
UNITREE_MIC_SEGMENT_SECS=0
```

`UNITREE_DROP_BROWSER_AUDIO=0` là workaround cho cách phân loại frame audio hiện
tại. Vì vậy mic trình duyệt phải được mute sau khi kết nối.

## Chạy ứng dụng

```bash
cd ai_modules/voice_interaction
uv sync
./run.sh
```

Thứ tự vận hành:

1. Khởi động bot và giữ terminal chạy.
2. Mở SmallWebRTC client và bấm Connect.
3. Chờ bridge log đã join `239.168.123.161:5555` qua `eth10`.
4. Mute mic trình duyệt.
5. Bật và giữ chế độ giao tiếp gốc của robot.
6. Nói vào mic robot và kiểm tra log `robot_mic` tăng.

Trên laptop cùng mạng, mở Chromium bằng profile riêng:

```bash
chromium \
  --unsafely-treat-insecure-origin-as-secure=http://192.168.12.2:7860 \
  --user-data-dir=/tmp/chromium-r1 \
  http://192.168.12.2:7860/client/
```

Flag trên chỉ dành cho giao diện thử nghiệm nội bộ qua HTTP. Không dùng profile
này để đăng nhập tài khoản cá nhân hoặc duyệt website khác.

## Chẩn đoán nhanh

- `GetVolume ret=0` nhưng `robot_mic=0`: bridge kết nối được voice service nhưng
  firmware chưa phát mic; bật chế độ giao tiếp gốc của robot.
- `robot_mic > 0` nhưng không có transcript: kiểm tra
  `UNITREE_DROP_BROWSER_AUDIO=0` và quota OpenAI API.
- Lỗi `insufficient_quota`: mic vẫn có thể hoạt động; bổ sung credit hoặc tăng
  usage limit trong OpenAI Platform, sau đó restart bot và reconnect UI.
- Chỉ nghe ở laptop: kiểm tra `UNITREE_SPEAKER_ENABLED=1` trong `.env`.
- Bridge restart mỗi vài giây: đặt `UNITREE_MIC_SEGMENT_SECS=0`.

Quy trình đầy đủ nằm trong `docs/R1_VOICE_OPERATION_RUNBOOK.md`.
