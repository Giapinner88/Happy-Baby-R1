# BÁO CÁO V5 — KẾT QUẢ TRIỂN KHAI `high_level_2` + `voice_r1`

**Cập nhật cuối:** 2026-07-22  
**Đích runtime:** Jetson ARM64 trên robot, `/home/unitree/HB`  
**Máy trạm/máy dev:** chỉ sửa source, train và chuẩn bị model/asset

## 1. Kết luận

- Đã triển khai production gồm `high_level_2`, `voice_r1` và lớp trung gian
  `r1_integration` lên robot.
- Sau reboot, cả ba service tự chạy, đều `active/running`, `NRestarts=0`.
- `high_level_2` tự mở ở `DISARMED`; auto-start không tự arm hoặc tự điều khiển motor.
- Voice headless chạy không cần SSH, browser hay nút Connect. Watchdog kiểm tra
  OpenAI và mic thật, không chỉ dựa vào trạng thái systemd.
- Voice không còn API motor. Coordinator chỉ subscribe `rt/lowstate`, không publish DDS.
- Mic và loa voice fail-closed. High-level phát âm thanh thì voice bị khóa/cắt và high-level luôn
  có ưu tiên.
- R3-1 đang được phát hiện (`remote_alive=1`); mic chỉ mở trong lúc giữ `select`.

## 2. Trạng thái robot sau triển khai

| Hạng mục | Kết quả |
|---|---|
| Kiến trúc | ARM64/aarch64, Ubuntu 20.04, interface `eth10` |
| `hb_high_level.service` | `active/running`, boot `DISARMED`, `NRestarts=0` |
| `hb_integration.service` | `active/running`, `NRestarts=0` |
| `hb_voice.service` | `active/running`, `openai_ready=1`, `mic_ready=1`, attempt 1 |
| Auto-start | Bốn unit/target đều `enabled`; reboot thực tế đã đạt |
| Policy locomotion | `policy_3.onnx`, input 83, output 24 |
| SHA-256 policy | `793ac13a4b99ed55b8ac62adf7aa3545eb009eb11727d07b994b6a8b4712c1d9` |
| Binary runtime | Cả `run_r1`, `r1_bridge`, `hb_integration` đều ARM64; `ldd` không thiếu thư viện |
| Secret | `/etc/hb/stack.env` mode `600`, owner `root`; không sync về dev |
| Voice process | 1 supervisor, 1 mic bridge, kết nối TCP 443 |

Trạng thái gate cuối:

```text
high_alive=1
high_busy=0
high_armed=1
high_state=IDLE__damping_
remote_alive=1
ptt=0
mic_allowed=0
speaker_allowed=1
```

`ready=1` xác nhận coordinator nhận đủ heartbeat high-level và dữ liệu R3-1.

## 3. Kiến trúc và quy tắc an toàn

```text
R3-1 select ──> r1_integration ──> fail-closed gate ──> voice_r1 ──> OpenAI/loa
                         ^
                         └── heartbeat + BUSY/IDLE từ high_level_2
```

Sau khi high-level đã có heartbeat, mic chỉ mở khi:

```text
high_alive AND remote_alive AND select_held
AND activation_mode cho phép
AND NOT high_audio_busy AND NOT voice_speaking
```

Mặc định `activation.mode=both`, nên voice dùng được ở Built-in/high-level
`DISARMED` và sau khi high-level `ARMED`. Trong tối đa 90 giây khởi động đầu,
voice có thể dùng trước heartbeat; sau khi đã thấy high-level, mất heartbeat
luôn làm gate khóa.

Khi high-level cần phát lời/nhạc:

1. High-level gửi `BUSY`.
2. Coordinator khóa mic và loa voice.
3. High-level gọi `PlayStop("pipecat")` và voice hủy response OpenAI.
4. High-level phát bằng app `hb_audio`.
5. Khi `IDLE`, voice được phép hoạt động lại; người dùng phải nhả rồi giữ lại `select`.

Nếu coordinator/heartbeat/tay cầm mất, mic và loa voice tự khóa trong tối đa khoảng 0,6 giây.

## 4. Cơ chế tự khởi động và thao tác người dùng

Trình tự sau khi bật nguồn:

```text
multi-user.target
  -> hb-stack.target
  -> chờ eth10 có IPv4
  -> hb_integration, hb_high_level và hb_voice khởi động song song
  -> voice nối mic + OpenAI, không chờ high-level preflight hoàn tất
  -> lỗi DNS/mạng/WebSocket thì supervisor tự tạo session mới
```

Người dùng không cần SSH hoặc mở web. Thao tác thực tế:

1. Bật robot và Jetson; hệ thống tự khởi động.
2. Ở Built-in/`DISARMED`, giữ `select`, nói rồi nhả là dùng được voice; không
   cần vào Development Mode chỉ để nói.
3. Nếu cần điều khiển high-level: vào Development Mode bằng `L2+R2`, chờ
   built-in nhả `lowcmd`, rồi giữ `R1+R2` 3 giây để bàn giao.
4. Sau khi `ARMED`, voice vẫn dùng bằng `select`. Khi high-level đang phát âm
   thanh, mic bị khóa; nhả rồi giữ lại sau khi âm thanh kết thúc.

## 5. File đã thay đổi

| Nhóm | File chính | Nội dung |
|---|---|---|
| Coordinator | `r1_integration/src/main.cpp` | Đọc `select`, heartbeat, audio gate và status; không có motor API |
| Runtime voice | `voice_r1/hb_voice/__main__.py` | Entry point headless và supervisor reconnect |
| Auto-start | `r1_integration/systemd/*` | Ba service và `hb-stack.target` |
| Deploy | `r1_integration/scripts/*` | Sync, build ARM64, preflight, install, restart-safe, health-check |
| High audio | `high_level_2/src/audio/MusicPlayer.hpp` | Resolve path, BUSY/IDLE, ưu tiên `hb_audio`, sửa `PlayStop` |
| High status | `high_level_2/src/app/Application.*` | Heartbeat/state sang coordinator; thêm preflight không publish motor |
| Model guard | `high_level_2/src/policy/OnnxPolicy.cpp` | Kiểm cả input 83 và output 24 |
| Audio path | `high_level_2/config/tuning.yaml` | Đổi `voice_*` từ `/home/unitree/HB/...` thành `src/audio/...` |
| Voice gate | `voice_r1/hb_voice/gate.py` | Gate fail-closed, chế độ armed/disarmed và startup |
| Mic | `voice_r1/hb_voice/input.py` | PTT mic robot và adapter mic USB/ALSA tương lai |
| Speaker | `voice_r1/hb_voice/output.py` | Volume riêng; cắt bridge khi high-level `BUSY` |
| Pipeline | `voice_r1/hb_voice/app.py` | OpenAI Realtime, watchdog và hủy response khi bị preempt |
| Health/retry | `voice_r1/hb_voice/resilience.py` | Nhận diện lỗi mạng và ghi `voice_status.env` |
| Tuning | `voice_r1/config/tuning.yaml` | Model, voice, mic gain, volume, activation và reconnect |
| SDK bridge | `voice_r1/unitree_bridge/r1_bridge.cpp` | Chỉ giữ hai mode runtime `mic` và `speaker` |

## 6. Đường dẫn và khả năng chuyển máy

| Dữ liệu | Vị trí/quy tắc |
|---|---|
| Source trên dev | Có thể đặt `HB` ở bất kỳ đường dẫn nào |
| Runtime mặc định robot | `/home/unitree/HB` |
| Runtime config/secret | `/etc/hb/stack.env` |
| Socket/status | `/run/hb/*.sock`, `/run/hb/status.env` |
| Systemd | Bắt buộc absolute path; installer sinh lại theo `HB_ROOT` |

- Chuyển workspace giữa máy trạm và máy dev không ảnh hưởng source vì script tự suy ra root.
- Nếu đổi vị trí `HB` trên robot, đặt `DEST`/`HB_ROOT` mới rồi chạy lại installer/deploy.
- Không copy `build/`, `.venv/`, `.env`, cache, log hay binary x86 từ dev lên robot.
- Binary và `.venv` luôn được tạo trên robot ARM64.

## 7. Đồng bộ, build và restart

Entry point duy nhất:

```bash
cd HB
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy
./r1_integration/scripts/deploy_stack.sh status
```

Các lệnh bổ sung:

```bash
./r1_integration/scripts/deploy_stack.sh deploy --no-restart
./r1_integration/scripts/deploy_stack.sh deploy --restart-voice
./r1_integration/scripts/deploy_stack.sh deploy --accept-policy
./r1_integration/scripts/deploy_stack.sh pull --dry-run
./r1_integration/scripts/deploy_stack.sh pull
./r1_integration/scripts/deploy_stack.sh rollback
```

Script dùng một SSH ControlMaster trong mỗi lượt, rsync source/assets, build incremental trên robot,
chạy preflight và chỉ restart high-level khi chứng minh được `DISARMED`. `--restart-voice` restart
coordinator + voice nhưng không tác động high-level. `--accept-policy` tự kiểm
tra policy local, cập nhật `r1_integration/config/model_manifest.conf`, rồi deploy toàn stack. `diff`
hiện không còn file khác nội dung giữa local và robot.

Backup deploy nằm tại `/home/unitree/HB_backups/HB_*.tar.gz`; backup loại secret, build, `.venv`
và log.

## 8. Kiểm thử đã thực hiện

| Kiểm thử | Kết quả |
|---|---|
| Unit/static gate, cấu hình, gain, reconnect và motor safety | 28/28 đạt |
| Bash/Python syntax | Đạt |
| Build sạch local | `high_level_2` và `r1_integration` đạt |
| Preflight policy/assets | Đạt; locomotion 83→24 và bốn policy dance nạp được |
| Build trực tiếp robot | Ba binary ARM64 đạt |
| Voice import trên Python 3.14 ARM64 | Đạt |
| Voice headless không browser | Pipeline ready, có kết nối 443 |
| Health runtime sau deploy | `openai_ready=1`, `mic_ready=1`, attempt 1 |
| Mic multicast | Nhận liên tục; 1 bridge, watchdog không báo sai |
| Restart voice 50 lần | 50/50 đạt; không orphan/process trùng |
| Mất coordinator | Voice khóa mic+loa; high PID không đổi; phục hồi tự động |
| Reboot auto-start lần hai | Ba service active, `NRestarts=0`, không lỗi boot |
| Secret/permission | Đạt mode `600` |

Chưa thể xác nhận tự động trong phiên này:

- Preempt bằng một lần high-level phát âm thanh thật; logic và fail-closed đã được test, nhưng cần
  nghe trực tiếp trên robot.
- Soak liên tục 8 giờ; stack hiện đang chạy để theo dõi tiếp.
- Mic USB vật lý chưa được cắm để nghiệm thu; adapter và cấu hình đã có.

## 9. Cleanup `voice_r1`

Đã bỏ demo WebRTC/Gradio, API action/motor, entry point trùng lặp và tài liệu cũ. Runtime Python được
gom vào package `hb_voice`; công cụ mic nằm trong `tools/`, hướng dẫn mic USB nằm trong `docs/`.
Deploy dùng `--delete-delay` cho `voice_r1` nên các file cũ trên robot cũng đã được dọn.

Trên robot vẫn giữ `.venv` và các binary ARM64 vì đó là runtime bắt buộc.

## 10. Mic USB tương lai

Chọn mic trong `voice_r1/config/tuning.yaml`:

```yaml
input:
  source: alsa_usb
```

Sau đó cấu hình đúng thiết bị trong `/etc/hb/stack.env`:

```env
ALSA_DEVICE=plughw:CARD=RobotMic,DEV=0
ALSA_SAMPLE_RATE=16000
```

Sau đó restart `hb_voice.service`. Không dùng `hw:1,0`; tên card phải ổn định. Không cần sửa
high-level, coordinator hoặc pipeline.

## 11. Các lỗi voice đã chủ động xử lý

| Nguy cơ | Cơ chế sau sửa |
|---|---|
| Boot chưa có Internet, lỗi `Errno 113` | Nhận diện đúng và tạo session mới với backoff 2–30 giây |
| WebSocket nhận bị dừng nhưng service vẫn active | Watchdog kiểm tra session/receive loop mỗi 1 giây |
| Mic bridge treo hoặc mất packet | Health báo `mic_ready=0`, restart sau 9 giây, có backoff chống loop CPU |
| Speaker SDK trả lỗi | Bridge thoát có mã lỗi; Python dọn process và tạo lại ở audio kế tiếp |
| DNS check làm chậm boot | Preflight giới hạn 2 giây; lỗi mạng được giao cho supervisor retry |
| High-level khởi động chậm | Voice chạy song song; startup grace hữu hạn và khóa sau khi mất heartbeat |

Log còn một warning từ Pipecat về turn frame của realtime service. Runtime này
tự phát `UserStarted/StoppedSpeakingFrame` theo PTT, nên warning đó không làm
mất lượt nói và không phải lỗi kết nối.
