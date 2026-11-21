# assistant/ trên HB

Tác vụ giọng nói hợp nhất chạy bằng chính dịch vụ HB: **tìm** (quét đầu ±25°, quay thân 180°), **quay thân về
phía vật** (vòng yaw IMU), **đi tới** cách ~1 m (đứng yên nhìn, đi từng đoạn, căn thẳng, tracker làm mới mẫu),
**tìm lại khi vật bị dời**, **chốt đồng ý camera** (chỉ chụp/gửi ảnh sau một câu nói mới của người dùng) và câu
trả lời ngắn tự nhiên. Mã xuất từ VLA (`scripts/interaction/export_r1_assistant_hb.py`); không sửa tay trong
thư mục này, `EXPORT.json` ghi hash và commit nguồn.

| Thành phần | Vai trò |
|---|---|
| `hb_assistant.service` (`integration/systemd/hb_assistant.service.in`) | Điều phối tác vụ, cổng `127.0.0.1:5595` |
| `hb_voice` (`integration/scripts/run_voice.sh`) | Nạp `assistant/overlay` khi có `/etc/hb/assistant_enabled` |
| `r1-p5-agent` | Chụp ảnh, đọc trạng thái, quay đầu/thân cho bước tìm |
| `hb_high_level` | Owner duy nhất; nhận lệnh đi tới qua APV cổng 5573 |

## Bật / tắt (robot được đỡ, DISARMED, có người giữ E-stop)

```bash
./hb deploy assistant                                   # máy trạm: đồng bộ assistant/ + integration
bash ~/HB/integration/scripts/install_assistant.sh check # robot, user unitree
sudo bash ~/HB/integration/scripts/install_assistant.sh enable --operator-ready
sudo bash ~/HB/integration/scripts/install_assistant.sh disable   # quay lại hb_local_approach/object_search voice
```

`enable` ghi cấu hình chạy vào `/etc/hb/assistant/`, đặt `R1_APPROACH_PORT=5573`, `R1_APPROACH_ALLOW_NONZERO=1`
trong `/etc/hb/stack.env` (bản sao lưu `stack.env.assistant.bak`), dừng `hb_local_approach` (dùng chung khóa
camera), restart `hb_high_level` và `hb_voice`. Dữ liệu chạy ở `/home/unitree/hb_assistant/runs` (ngoài HB nên
`./hb deploy` không xóa). Nếu stack riêng r1_assistant đang bật, chạy rollback của nó trước.

## Giọng ElevenLabs

`voice/config/tuning.yaml` chọn `tts.provider: elevenlabs`. Thiếu `ELEVENLABS_API_KEY`/`ELEVENLABS_VOICE_ID` trong
`/etc/hb/stack.env` thì `hb_voice` tự dùng giọng OpenAI và ghi cảnh báo; thêm key rồi
`sudo systemctl restart hb_voice` là đổi giọng. `HB_TTS_STRICT=1` để báo lỗi thay vì tự chuyển.

## Giới hạn đã biết

Khi đi, robot không nhìn đường và không phát hiện vật cản: cần người giữ E-stop. Điểm dừng là ước lượng
(`physical_arrival_verified` luôn false). Chưa chọn được "vật gần nhất" khi có nhiều vật giống nhau.
