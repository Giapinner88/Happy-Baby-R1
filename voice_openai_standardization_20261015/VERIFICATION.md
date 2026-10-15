# Verification record

Ngày kiểm tra: 2026-10-15.

## Đã đạt

- Python syntax/bytecode compile cho `hb_voice`, tools và tests; cache sinh ra
  trong bước kiểm tra đã được loại khỏi gói.
- `test_voice_config.py`: 21/21 pass.
- `test_audio_gate.py`: 10/10 pass, gồm fail-closed, PTT, F1 hands-free,
  gesture-dev gate và lựa chọn mic. Test cần Unix socket tạm.
- `test_voice_resilience.py`: 19/19 pass.
- `test_realtime_vision.py`: 3/3 pass cho cấu trúc image event, JPEG validation
  và trích xuất text/transcript; đây là test offline, không capture camera thật.
- Shell syntax pass cho launcher, preflight, health-check và robot build script.
- Secret scan chỉ thấy placeholder/tên biến trong tài liệu, template, test và
  validator; không thấy credential thật.

## Mâu thuẫn test có sẵn được bảo toàn để chuẩn hóa

`test_static_safety.py` đạt 25/27. Hai assertion dựa trên text implementation cũ:

1. Test tìm đúng chuỗi `if _robot_ssh_ok "$ROBOT"; then`, trong khi finder hiện
   kiểm tra biến `preferred_robot` sau khi hợp nhất env/config.
2. Test đòi cue F1 hai nốt `(660, 880)`, trong khi implementation hiện đã đổi
   sang cue ba nốt `do-mi-sol`/đảo chiều để dễ phân biệt trong phòng ồn.

Đây là test drift đã tồn tại trong source snapshot, không phải file bị mất khi
đóng gói. Không sửa test hoặc runtime trong bundle để tránh làm sai provenance.

## Chưa được xác nhận trong môi trường này

- Các test cần Pipecat/OpenAI/ElevenLabs SDK chưa chạy vì workspace không có
  `voice_r1/.venv`; dùng `uv sync --frozen` trên môi trường chuẩn rồi chạy lại.
- Không gọi OpenAI hoặc ElevenLabs thật; không ghi âm hay gửi dữ liệu ra mạng.
- Không build `r1_bridge`: cần ARM64 và Unitree SDK2 trên robot.
- Không capture camera thật: camera probe là external dependency và không có
  trong repository này.
- Chưa chạy hardware preflight/health-check. Bundle không phải bằng chứng robot
  đã sẵn sàng vận hành.
