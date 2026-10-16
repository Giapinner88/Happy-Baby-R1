# Teleop Quest 3 (chạy trên robot)

> **Chỉ dùng khi robot treo/cố định trên giá và có người giữ E-stop.** Lần thử phần cứng gần nhất
> (bounded suspended pilot) chưa đạt; chưa có bằng chứng nào cho phép teleop khi robot đứng trên sàn.

## 1. Đường dữ liệu

```text
Quest Browser (WebXR) --Wi-Fi wlan0:8012--> quest_bridge.py
  -> run_r1_vendor_ik_stream.py   (IK R1_A5_ArmIK của Unitree, pin 845b25b)
  -> run_r1_vendor_targets.py     (giới hạn khớp, contract H4)
  -> teleop.hardware.high_level_sidecar --UDP 127.0.0.1:5560--> run_r1 -> rt/lowcmd
```

Tất cả chạy trong service `hb_teleop_runtime` trên robot; không cần laptop sau khi service khởi động.
Teleop chỉ điều khiển **10 khớp tay + 2 khớp đầu**. Chân và eo luôn do policy (LOCOMOTION) hoặc zero-torque (Mode Z) giữ.
Sidecar không có DDS publisher; `run_r1` là nơi duy nhất ghi `rt/lowcmd`.

## 2. Cài đặt (từ máy trạm)

```bash
./hb deploy teleop      # đồng bộ teleop/, tạo env conda hb_teleop, cert, /etc/hb/teleop/runtime.env, bật 2 service, in Quest URL
./hb ssh ./HB/hb teleop-check   # đọc rt/lowstate, không tạo publisher: phải thấy mode_machine=1, motors=35
```

`./hb field` cũng làm bước cài này.

Cấu hình duy nhất: [`teleop/config/teleop.yaml`](../teleop/config/teleop.yaml) (`robot_runtime.*`). Mặc định: chờ Quest và `rt/lowstate` vô hạn sau khi service khởi động (`first_input_timeout_s: 0`, `initial_state_timeout_s: 0`; sidecar chỉ phát STOP cho tới khi có target hợp lệ), `control_hz: 30`, `cpu_affinity: auto` (tách bridge / IK / mapper+sidecar khi có ≥ 4 lõi). Sidecar đọc giới hạn đầu thật của `run_r1` từ `controller/config/tuning.yaml` để không gửi gói bị receiver từ chối.
Bước cài cũng cấu hình mDNS (Avahi `hb-r1.local` trên `wlan0`) và, nếu UFW đang bật, mở cổng WebXR/mDNS/WebRTC cho subnet của Quest. Restart runtime không cần DISARMED: nếu đang ARMED, tay về tư thế nền như khi nhả deadman và phải bóp lại cò sau khi runtime lên.
Sửa trên máy trạm rồi chạy lại `./hb deploy teleop`. Đặt `enabled: false` để tắt và gỡ hai service.
Chỉ khởi động lại thì `./hb restart teleop` (dùng được cả khi robot đang ARMED).

Phía `run_r1`: `config/gestures.yaml` phải có `teleop_enabled: true`; cổng UDP `teleop_udp_port: 5560`.

## 3. Chạy một phiên

1. Robot treo trên giá, người giữ E-stop sẵn sàng. Bàn giao: `L2+R2` → giữ `R1+R2` ~1 s.
2. Vào trạng thái cho phép: **ZERO TORQUE** (`L2+Y`, chỉ từ IDLE, robot đang treo), **R2+A** hoặc **R2+X**. Cả hai mode locomotion phải có bundle H4 đã qua gate. Với R2+A, thả cần để `vx/vy/yaw` về 0 trước START; sau đó base bị khóa. R2+X dùng `teleop_policy` và cho phép vừa đi vừa teleop. `teleop_policy` hiện là U1 (`u1_seed42`, chỉ điều khiển chân; tay nhận trực tiếp từ Quest hoặc gesture). Gesture nút tay cầm và gesture gọi bằng voice dùng được ở R2+X khi Quest chưa được cấp quyền H4; voice gesture vẫn chỉ bắt đầu khi không có lệnh di chuyển. U1 dùng được tới ~1.1 m/s trong mô phỏng, nên giữ tốc độ chậm (`R2+Xuống`).
3. Lấy URL: `journalctl -u hb_teleop_runtime -n 50 | grep "Quest URL"` (theo IP hiện tại của `wlan0`).
   Trên Quest: cùng Wi-Fi, mở URL, chấp nhận chứng chỉ, **Enter VR**. Tab cũ phải reload.
4. **Nhấn START một lần** (nút trần, không kèm L1/L2/R1/R2) để cấp quyền teleop trong `run_r1`; nhấn lại để thu quyền ngay. Giữ nút không lặp lại lệnh. R2+A từ chối START nếu lệnh vận tốc chưa về 0 và khóa base sau khi arm; R2+X cho phép START và vận tốc đồng thời. Đổi policy R2+A ↔ R2+X sẽ tự thu quyền để tay về nền rồi mới swap. Cấp quyền chưa làm robot cử động: còn cần deadman cò Quest (bước 6).
5. Đưa người điều khiển và robot về tư thế neutral; **đỡ đầu robot ngang** (ở ZERO TORQUE đầu rũ xuống).
6. **Giữ cò phải** (deadman). Frame đầu tiên được chốt làm `source_zero`, encoder hiện tại làm `start_q`; target sau đó là độ lệch tương đối, bị chặn trong `max_offset_rad` (mặc định 0.15 rad).
7. Di chuyển chậm. Log `run_r1` in `[Teleop] stream ACTIVE` / `stream INACTIVE reason=...` ở mỗi lần đổi trạng thái.

Cò trái không dùng. Nhả cò phải: stream gửi STOP, đầu nhả ngay, tay thu quyền rồi về trạng thái nền. Mặc định `release_debounce_s: 0`: pipeline sống tiếp sau khi nhả cò, bóp lại bất cứ lúc nào mà không khởi động lại. Mất tracking tay cầm trái giữa phiên: tay trái được giữ tại chỗ cho tới khi nhả hẳn cò phải (tay cầm vừa có lại không được tự kéo tay khi cò vẫn giữ). Gói lỗi, timestamp nguồn sai hoặc watchdog nguồn: sidecar gửi STOP và chỉ nhận target mới sau khi nhả hẳn cò.

START chỉ arm teleop; chưa bóp cò Quest thì vẫn dùng gesture tay cầm/voice ở R2+X được. Nếu bóp cò giữa một gesture, controller thu tay theo giới hạn return và đợi handover ổn định trước khi cấp quyền H4. Voice tự bật tool `execute` khi integration báo ARMED + LOCOMOTION + remote còn sống; `gesture_control.mode: off` là trạng thái khởi động. Voice-auto khi đang nói vẫn bật/tắt bằng double-click A.

**Camera robot:** nút **A tay phải** trên Quest đổi giữa passthrough và ảnh camera R1 (WebRTC VP8, phục vụ ngay trên robot ở cổng `camera_media_port` 8013, cùng origin HTTPS). Ở passthrough không đọc/giải mã frame. Tắt bằng `camera_enabled: false`. Camera lỗi không chặn teleop tay/đầu.

**Chẩn đoán nút tay cầm:** mở `https://hb-r1.local:8012/static/teleop/scripts/teleop/quest_input_probe.html` trên Quest (đóng tab teleop trước). Trang chỉ đọc WebXR, không gửi lệnh.

## 4. Dừng

- Bình thường: giữ tư thế → nhả cò phải → nhấn START để thu quyền.
- Bất thường (rung, sai chiều, va chạm, tiếng lạ, mất mạng, target không khớp người điều khiển): **E-stop ngay**, không chờ watchdog.
- Tự nhả quyền khi: mất gói > `teleop_timeout_ms` (300 ms), `enable=0`, nghiêng/gyro vượt ngưỡng, rời LOCOMOTION/ZERO TORQUE, mất tay cầm.
- Trước khi dừng/restart high-level: làm DISARMED theo [safety.md §3](safety.md). Teleop thì dừng/restart lúc nào cũng được (`./hb restart teleop`).

## 5. Mode Z: giữ chân và eo trên giá

Mặc định trong ZERO TORQUE chân/eo mềm. Bật `teleop_lock_others_enabled: true` (`config/gestures.yaml`) để giữ chân (IDL 0–11) và eo (12–13) bằng PD tại encoder chốt lúc teleop bắt đầu (`teleop_lock_kp: 20`, `teleop_lock_kd: 3`, `teleop_lock_max_rate_rad_s: 0.20`).
Chỉ dùng trên giá treo; **không** dùng để đỡ robot đứng trên sàn. Chân rung/kêu → dừng, đánh giá lại `kp/kd`.

## 6. Log

`teleop/logs/robot_runtime/<UTC>_robot_quest3/`: `quest_commands.jsonl`, `vendor_ik_stats.json`, `vendor_targets_diagnostics.jsonl`, `targets.jsonl`, `camera_webrtc.jsonl`, log từng tầng. Mẫu của sidecar ghi `stop_reason`, tuổi input/nguồn và `max_send_gap_s` mỗi khi trạng thái đổi. Log của `run_r1`: `journalctl -u hb_high_level -f`.

## 7. Cổng phần cứng còn mở

Chưa đạt: bám trên phần cứng thật (tầng IK đã đo trên máy trạm, chưa trên robot), giới hạn vận tốc/gia tốc được duyệt cho phần cứng, guard va chạm/mô-men, hành vi khi mất kết nối Quest, đối chiếu dấu/thứ tự khớp FK với robot.
Đã xác nhận trên robot: IDL 29 = head pitch, IDL 30 = head yaw (đo `rt/lowstate` 2026-08-29); `run_r1` là publisher `rt/lowcmd` duy nhất (`./hb test` kiểm tra tĩnh).
Bằng chứng chi tiết và các phép đo nằm ở nhánh `teleop`.
