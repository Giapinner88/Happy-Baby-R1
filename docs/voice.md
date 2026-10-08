# Vận hành OpenAI + ElevenLabs trên Unitree R1

## 1. Luồng âm thanh

Runtime `HB` chạy headless, không cần mở Chromium:

```text
Giữ Select
  -> mic ngoài cắm vào PC2, đọc qua ALSA
  -> OpenAI Realtime nhận tiếng Việt và sinh văn bản
  -> ElevenLabs biến văn bản thành giọng clone
  -> UnitreeSpeakerBridge
  -> loa robot
```

OpenAI không tạo giọng khi `tts.provider: "elevenlabs"`. Vì vậy giọng miền
Nam, âm sắc và độ ổn định phụ thuộc chủ yếu vào mẫu clone, `voice_id` và nhóm
setting `tts`.

## 2. Yêu cầu trước khi chạy

- Robot có Internet và phân giải được `api.openai.com`,
  `api.elevenlabs.io`.
- Mic ngoài xuất hiện trong `arecord -l` trên PC2.
- Có OpenAI API key còn quota.
- Có ElevenLabs API key được phép dùng Text to Speech và còn quota.
- Có giọng clone hợp lệ, được chủ giọng đồng ý sử dụng.
- Biết `voice_id` của giọng clone. Đây không phải tên hiển thị của voice.

## 3. Cấu hình secret một lần trên robot

Mở file bằng `sudoedit`:

```bash
sudoedit /etc/hb/stack.env
```

Đảm bảo file có các dòng sau:

```env
OPENAI_API_KEY=sk-...
ELEVENLABS_API_KEY=...
ELEVENLABS_VOICE_ID=...
UNITREE_NETWORK_INTERFACE=eth10
ALSA_SAMPLE_RATE=48000
# ALSA_DEVICE chỉ đặt khi có nhiều mic USB cùng lúc; để trống thì runtime tự dò card
```

Không gửi nội dung file này lên Git và không dùng `cat` để chụp/log toàn bộ
secret. Sau khi lưu:

```bash
sudo chmod 600 /etc/hb/stack.env
```

## 4. Chọn ElevenLabs trong source

File `voice/config/tuning.yaml` phải có:

```yaml
tts:
  provider: "elevenlabs"
  model: "eleven_flash_v2_5"
  language: "vi"
  stability: 0.55
  similarity_boost: 0.85
  speed: 1.0
  use_speaker_boost: true
```

Không ghi `voice_id` vào YAML. Runtime luôn đọc nó từ
`ELEVENLABS_VOICE_ID`.

## 5. Deploy từ laptop

Chạy từ **thư mục gốc repo trên máy dev** (script tự suy ra đường dẫn, không cần
sửa gì khi copy folder giữa máy dev và máy trạm):

```bash
cd <thư-mục-repo>
./integration/scripts/deploy_stack.sh diff
./integration/scripts/deploy_stack.sh deploy --restart-voice
./integration/scripts/deploy_stack.sh status
```

Địa chỉ robot được giải theo thứ tự: biến `ROBOT` -> file riêng của máy
`~/.config/hb/robot.env` -> tự dò ping. Mỗi máy khai địa chỉ của mình **một lần**:

```bash
mkdir -p ~/.config/hb
cp integration/config/robot.env.example ~/.config/hb/robot.env
# sửa dòng ROBOT trong file cho đúng máy này, ví dụ ROBOT=unitree@10.42.0.33
```

File nằm NGOÀI repo nên copy folder qua lại không đụng tới nó. Muốn ép
nhanh một lần: `ROBOT=unitree@<ip> ./integration/scripts/deploy_stack.sh ...`.
Deploy tạo môi trường Python và cài dependency ngay trên ARM64; không copy `.venv` từ
laptop.

## 6. Kiểm tra trên robot

```bash
sudo systemctl restart hb_voice.service
sudo systemctl status hb_voice.service --no-pager
cat /run/hb/voice_status.env
sudo journalctl -u hb_voice.service -n 120 --no-pager
```

Kết quả mong đợi:

- Service là `active (running)`.
- Status có `openai_ready=1`, `mic_ready=1` và `mic_source=alsa_usb`.
  `mic_source=r1_multicast` nghĩa là đang chạy mic thân robot — vẫn nói chuyện
  được, nhưng phải đi cắm/sạc lại mic ngoài rồi **nhấn `F2`** để chuyển về; nó
  không tự nhảy về.
- Log có `tts=elevenlabs/eleven_flash_v2_5 clone_configured=1`.
- Log có `OpenAI Realtime session is ready`.
- Không có lỗi thiếu key, quota, DNS hoặc WebSocket.

Theo dõi trực tiếp trong lúc thử:

```bash
sudo journalctl -u hb_voice.service -f
```

Giữ `Select`, nói một câu, rồi nhả nút. Một lượt đúng sẽ có
transcript của user, text assistant, hoạt động ElevenLabs TTS và âm thanh ở
loa robot. Không nhấn nhanh hai lần; đây là push-to-talk kiểu giữ để nói, nhả
để gửi.

## 7. Điều chỉnh giọng và độ trễ

- Giọng thay đổi giữa các câu: tăng `tts.stability` từng bước `0.05`, ví dụ
  `0.55 -> 0.60`.
- Giọng không giống mẫu: tăng nhẹ `tts.similarity_boost`.
- Giọng bị noise/rè: giảm `similarity_boost`, làm sạch hoặc tạo lại voice từ
  mẫu thu tốt hơn.
- Nói quá nhanh/chậm: chỉnh `tts.speed` trong khoảng `0.7-1.2`.
- Loa nhỏ: giữ `audio.response_gain` không quá `1.0`, tăng
  `audio.response_volume_percent` đến mức không rè.
- Trễ trước khi nói: dùng `eleven_flash_v2_5`, trả lời ngắn trong
  `voice/config/prompt.txt`, đồng thời kiểm tra độ trễ mạng tới cả OpenAI và
  ElevenLabs.

Sau mỗi lần sửa tuning:

```bash
sudo systemctl restart hb_voice.service
```

Nếu sửa trên robot trực tiếp, lần deploy tiếp theo có thể ghi đè. Cấu hình
production nên được sửa trên laptop rồi deploy.

## 8. Rollback về giọng OpenAI

Trong `voice/config/tuning.yaml`, đổi:

```yaml
tts:
  provider: "openai"
```

Khi đó runtime bỏ ElevenLabs và dùng `openai.voice`, `openai.speed` trong cùng
file. `ELEVENLABS_API_KEY` và `ELEVENLABS_VOICE_ID` không còn bắt buộc. Deploy
và restart lại `hb_voice.service`.

## 9. Chẩn đoán nhanh

### Thiếu key hoặc voice ID

Log báo `ELEVENLABS_API_KEY is missing` hoặc
`ELEVENLABS_VOICE_ID is missing`: sửa `/etc/hb/stack.env`, đặt mode `600` rồi
restart service.

### Robot nghe nhưng không nói

```bash
getent hosts api.openai.com
getent hosts api.elevenlabs.io
sudo journalctl -u hb_voice.service -n 150 --no-pager
```

Tìm lỗi quota, HTTP 401/403, voice không thuộc workspace, WebSocket hoặc DNS.

### Không có audio mic

Nếu preflight báo mic không tồn tại, ElevenLabs chưa liên quan. Runtime tự dò
card USB nên đây là chuyện phần cứng: chạy `arecord -l`, nếu không có card USB
nào thì xem `journalctl -k | tail` ngay lúc cắm (không có `new high-speed USB
device` = dây/cổng/dongle chưa lên bus). Có card thì ghi thử bằng tên card đó:

```bash
arecord -D plughw:CARD=BOYALINK,DEV=0 -f S16_LE -r 48000 -c 1 -d 5 /tmp/mic-pc2.wav
```

### Robot vẫn nói được nhưng không trả lời (điếc)

Đọc `mic_source` trong `/run/hb/voice_status.env` trước:

- `mic_source=r1_multicast` → đang chạy mic thân robot. Sạc/cắm lại mic ngoài rồi
  **nhấn `F2` một lần** để chuyển về (nghe nốt cao là đã sang mic ngoài). Mặc định
  `input.mic_switch: manual` nên **service KHÔNG tự nhảy về** — cứ chờ 20 giây như
  hướng dẫn cũ là chờ mãi.
- `mic_source=alsa_usb` mà `mic_ready=0` → mic ngoài còn trên bus nhưng không ra
  tiếng: kiểm tra mức Capture bằng `alsamixer -c Audio`. Cần nói ngay thì nhấn
  `F2` để mượn tạm mic thân robot.
- Log lặp `has been dead silent for ...s ... press F2 to switch to the robot mic`
  → mic ngoài câm hẳn. Đây là trạng thái CỐ Ý: nguồn mic chỉ đổi khi người vận
  hành bấm, vì tự đổi giữa sự kiện gây nhảy qua lại (2026-08-06: 3 lần/3 phút).

Phân biệt hỏng phần cứng với hỏng phần mềm bằng **log kernel**, đây là bằng chứng
dứt điểm — ALSA/arecord chỉ báo hậu quả:

```bash
journalctl -k --since -6h | grep -i 'usb 1-3'
```

`USB disconnect` + `cannot submit urb (err = -19)` = thiết bị rời bus thật (hết
pin, tuột dây, sụt nguồn), không phải lỗi cấu hình.

### Process bridge tự xuất hiện lại

`hb_voice.service` là supervisor. Kill riêng `r1_bridge` sẽ làm nó được tạo
lại. Dừng đúng cách:

```bash
sudo systemctl stop hb_voice.service
pgrep -af 'hb_voice|r1_bridge'
```

### Hai câu trả lời cho một lượt

Chế độ giữ nút dùng `turn_detection=False` và chỉ commit khi nhả Select. Khi
double-click F1 vào chế độ auto, service chuyển sang Semantic VAD và input bridge
không được tự gửi `UserStoppedSpeakingFrame`; chỉ OpenAI chốt lượt. Nếu vẫn có
hai câu trả lời, trước hết double-click F1 để về chế độ giữ nút và xác nhận không
có hai `hb_voice`, `bot.py` hoặc runtime thử nghiệm cùng chạy:

```bash
pgrep -af 'hb_voice|bot.py|r1_bridge'
```

### Robot tự nói một mình, giọng đổi giữa câu

Nghe như robot đổi người nói giữa chừng thì **không phải** đổi voice ID: chỉ có
một giọng được tổng hợp, `openai.voice` chỉ dùng khi `tts.provider: openai`.
Thật ra là **hai câu trả lời phát chồng lên nhau** — Semantic VAD chốt một lượt
không ai nói, OpenAI đáp lại lượt rỗng bằng câu chào xã giao, mà
`interrupt_response=False` nên lượt cũ không bị hủy.

Xác nhận bằng log — tìm lượt assistant mà **không có** user transcript trước đó:

```bash
journalctl -u hb_voice --since "-30 min" | grep -E "Transcript:|active response in progress|unable to append audio"
```

Thấy `Conversation already has an active response in progress` hoặc
`unable to append audio to context` là đúng bệnh này.

Nguồn kích thường là tiếng servo hoặc đuôi tiếng loa lọt vào mic — hay gặp nhất
khi đang chạy **mic thân robot** (`mic_source=r1_multicast` trong
`/run/hb/voice_status.env`) vì mic đó nằm ngay cạnh loa.

Xử lý theo thứ tự:

1. Sạc/cắm lại mic ngoài — mic thân robot chỉ là lưới cứu, không phải cấu hình chạy.
2. Nâng `audio.echo_tail_s` (`0.6` -> `0.9-1.2`) nếu robot hay nói nối đuôi ngay
   sau khi vừa dứt câu.
3. Nâng `input.min_speech_peak` nếu nó đáp cả tiếng ồn xung quanh.
4. Giọng lệch giữa các câu (chứ không chồng tiếng) thì nâng `tts.stability`.

### Vừa cắm mic ngoài vào thì loạn, một lúc mới ổn

Kiểm bằng:

```bash
journalctl -u hb_voice --since "-15 min" | grep "Microphone source is now"
```

Thấy `alsa_usb` ↔ `r1_multicast` đổi qua lại đều đặn mỗi ~20 giây là **cắm cục thu
rồi nhưng mic đeo chưa bật / chưa bắt sóng**. Card có mặt nên bộ dò hồi phục nhảy
về mic ngoài, nhưng tín hiệu là im lặng số nên bộ dò câm lại tụt xuống.

Từ 2026-08-05 nhịp dò tự giãn (20 → 40 → 80 → 160 → 300 giây) nên nó chỉ giật một
hai lần rồi ngồi yên, và log ghi rõ:

```text
The external mic is on the bus but produced no sound; next recovery check in 40s (was 20s)
```

**Bật mic đeo lên là xong** — có tiếng thật thì nhịp dò về lại 20 giây ngay và
robot tự nhảy về mic ngoài. Muốn nhanh hơn nữa thì `sudo systemctl restart hb_voice`.

Thứ tự cắm đúng: **bật mic đeo trước, rồi mới cắm cục thu vào PC2.**

## 10. Cách code được tổ chức

- `voice/hb_voice/config.py`: đọc provider/model/voice setting và bắt buộc hai biến
  ElevenLabs khi chọn clone.
- `voice/hb_voice/app.py`: đặt OpenAI Realtime ở chế độ output text, chèn
  `ElevenLabsTTSService` trước bridge loa và giữ đường rollback OpenAI.
- `voice/config/tuning.yaml`: chọn provider và điều chỉnh giọng, không chứa secret.
- `integration/scripts/preflight.sh`: kiểm tra integration ElevenLabs và
  DNS trước khi chạy.
- `integration/config/stack.env.example`: khai báo tên các secret cần có.

PTT/hands-free, audio gate, quyền ưu tiên high-level, systemd supervisor và
bridge loa Unitree vẫn được giữ nguyên. Mic multicast PC1 chỉ còn là đường
rollback khi đổi `input.source` về `r1_multicast`.
