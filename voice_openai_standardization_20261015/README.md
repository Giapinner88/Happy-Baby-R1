# HB Voice/OpenAI standardization bundle

Snapshot portable để chuyển phần voice hiện tại sang một thư mục chuẩn mà
không phải suy đoán lại dependency. Gói giữ nguyên đường dẫn tương đối dưới
`source/HB/` để có thể đối chiếu với source ban đầu.

## Phạm vi

| Thành phần | Source chính | Contract đi kèm |
| --- | --- | --- |
| OpenAI Realtime voice | `voice_r1/hb_voice/{__main__,app,config,gate,input,output,resilience}.py` | tuning, prompt, Pipecat dependency lock |
| Function calling | `voice_r1/hb_voice/{gesture_tools,gesture_response}.py` | coordinator gate, `VoiceGestureOwner`, gesture catalog |
| Picture/camera | `voice_r1/hb_voice/vision_tools.py` | probe/test tools và test offline |
| ElevenLabs | `voice_r1/hb_voice/app.py`, `voice_r1/config/tuning.yaml` | env template, runbook, `pyproject.toml`, `uv.lock` |
| R1 mic/speaker | `voice_r1/hb_voice/{input,output}.py` | `unitree_bridge` C++/CMake |
| PTT/F1/F2 và safety gate | `r1_integration/src/main.cpp` | service, preflight, health-check và launcher |

## Bố cục

```text
source/HB/voice_r1/       runtime Python, config, bridge, tools và tests
source/HB/r1_integration/ các file điều phối/deploy liên quan trực tiếp tới voice
source/HB/high_level_2/   contract và active gesture assets cho function call;
                          không phải full motor runtime
ENVIRONMENT.example      tên biến môi trường, chỉ chứa placeholder
SOURCE_MAP.tsv            phân loại file để đưa sang kiến trúc mới
MANIFEST.sha256           hash toàn bộ payload để kiểm tra sau khi di chuyển
VERIFICATION.md           kiểm tra đã chạy và giới hạn chưa kiểm chứng
```

## Ranh giới cần giữ khi chuẩn hóa

1. `voice_r1` không được publish motor command. Function call chỉ gửi action ID
   hữu hạn qua `/run/hb/gesture_owner.sock`; `high_level_2` kiểm tra và sở hữu
   chuyển động.
2. `inspect_camera` chỉ đọc một JPEG mới. Không dùng kết quả vision để cấp quyền
   chuyển động, nhận dạng danh tính hay suy ra khoảng cách an toàn.
3. API key/voice ID phải được inject lúc chạy, không đặt trong YAML, source,
   manifest hoặc archive.
4. OpenAI Realtime hiện nhận audio, hiểu ngôn ngữ và sinh text. ElevenLabs tạo
   audio khi `tts.provider: elevenlabs`; output đi qua `r1_bridge` tới loa R1.
5. PTT, F1/F2, high-level BUSY và remote-loss vẫn phải đi qua coordinator
   fail-closed trước khi mic hoặc speaker được mở.

## External dependency không nằm trong repository

- `/home/unitree/Foundation_Unitree/tools/r1_video_rpc_probe.py`: camera RPC
  reader được `vision_tools.py` gọi bằng system Python.
- Unitree SDK2 source có `audio_client.hpp`: cần để build `r1_bridge` trên ARM64.
- `ffmpeg`, `arecord`, `uv`, mạng tới OpenAI/ElevenLabs và thiết bị audio robot.

Camera probe không có trong repo này nên gói chỉ giữ contract/path và test. Cần
đưa probe vào package chuẩn riêng hoặc thay bằng interface camera chính thức.

Các `.npz` gesture top-level đang được cấu hình đã được giữ lại. Thư mục
`policies/gestures/backup/` và các file `*.bak-*` không thuộc runtime snapshot.

## Kiểm tra tính toàn vẹn

Từ thư mục bundle:

```bash
sha256sum -c MANIFEST.sha256
```

## Không được đóng gói

- `.env`, `/etc/hb/stack.env` hoặc secret thật;
- `.venv`, `__pycache__`, `.pyc`, log và cache;
- binary `r1_bridge` đã build cho một máy cụ thể;
- recording người dùng hoặc dữ liệu hội thoại.

Snapshot này phục vụ chuẩn hóa source/interface. Nó chưa tự chứng minh hardware
readiness; sau khi chuyển thư mục vẫn phải chạy unit tests, preflight ARM64 và
health-check trên robot.
