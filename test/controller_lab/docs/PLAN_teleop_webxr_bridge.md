1




# PLAN — Teleop Quest 3 qua WebXR Bridge trên Jetson (Phương án C)

> Trạng thái: **PLAN v2 (đã audit code thật) — chưa code.**
> Thay thế đường `xr_teleoperate` (xem `PLAN_xr_teleop_integration.md` — giữ làm phân tích nền).
> `run_r1` = chương trình chạy trên **robot thật** (Jetson PC2), build từ `HB/controller`. Bản sim là `run_policy`.
> Robot-side teleop (TeleopReceiver UDP, overlay, L2+Phải, watchdog) **đã có sẵn**; plan này thêm **operator-side** + **1 bản vá hardening bắt buộc** (mục 3).

---

## 1. Quyết định & mục tiêu

| Quyết định              | Chọn                                                                     | Lý do                                                                               |
| -------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| Nguồn operator            | **Trang WebXR tự viết** chạy trên trình duyệt Quest 3         | Bỏ xr_teleoperate (nặng, cần host PC, không hỗ trợ R1)                         |
| Máy chạy bridge          | **Jetson PC2 của robot** (.164)                                    | Không laptop; auto-start systemd như voice                                         |
| Chế độ điều khiển v1 | **2 controller Quest**, không hand-tracking                        | Bám ổn định (không sợ che khuất/ánh sáng), grip =**deadman** vật lý |
| Chế độ nhìn            | **Passthrough AR** — nhìn robot thật xuyên kính                | Không cần stream camera → bridge siêu nhẹ, trễ thấp                           |
| Retarget                   | **Analytic 5-DoF** trong bridge — không IK số, không URDF       | Tay deploy 5 khớp/bên; toán tường minh, chỉnh dấu/scale bằng config          |
| Giao thức ra              | Gói**UDP `UTL1` 60B** sẵn có → `127.0.0.1:5560` của run_r1 | Giữ hợp đồng đã test wire; run_r1 gần như không đổi                       |
| Nhân lực khi show        | **2 người** (performer + safety op cầm R3)                       | Performer 2 tay cầm controller, không thể cầm R3 — xem mục 7.1                 |

**Mục tiêu vận hành:** bật robot → bridge tự chạy → bàn giao run_r1 như thường → performer đeo kính, mở bookmark, buông tay giữ 2 grip 1s → safety op bấm **L2+Phải** → robot theo tay. **Không laptop ở mọi bước.**

---

## 2. Kiến trúc, luồng dữ liệu & chuỗi an toàn

```text
Meta Quest 3 (trình duyệt, passthrough AR)
 │  WebXR: pose 2 controller (cổ tay) + pose đầu, 60–90Hz
 │  GRIP = deadman · giữ 2 grip 1s = calib+bắt đầu · A = calib lại · B = recenter đầu
 ▼  WSS (WebSocket over HTTPS, JSON ~50Hz, kèm token)
Jetson PC2 — teleop_bridge (python, systemd, PASSIVE — không ghi lowcmd)
 │  validate → calib-transform → retarget analytic → clamp+rate-limit → gói UTL1
 ▼  UDP 127.0.0.1:5560
run_r1 TeleopReceiver → 4 cổng (đang chạy? enabled? LOCOMOTION? L2+Phải?)
 ▼
overlay tay lên policy + mask obs + balance guard → MỘT nguồn rt/lowcmd → motor
```

**Single-writer giữ nguyên:** bridge chỉ sản xuất *dữ liệu*; run_r1 vẫn là nguồn ghi `rt/lowcmd` duy nhất — không lặp lại xung đột 2-nguồn từng gặp.

### Chuỗi an toàn 5 lớp (lớp nào hỏng, lớp dưới đỡ)

| Lớp           | Ở đâu       | Cơ chế                                                                                                                                                                                                                                    |
| -------------- | -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| L0 Operator    | Quest          | **Deadman**: giữ GRIP mới gửi `enable=1`; thả → `enable=0` → tay fade về policy. Thoát XR/khoá màn hình → hết gói                                                                                                   |
| L1 Bridge      | Jetson         | Validate NaN/range; clamp góc + rate-limit;**token trong URL** (WS sai token bị từ chối — chặn máy lạ trên LAN); **chỉ 1 phiên WS**; mất pose >150ms → tự gửi `enable=0` rồi ngừng; log xoay vòng           |
| L2 Transport   | Jetson         | UDP chỉ bắn`127.0.0.1` — cổng điều khiển không ra mạng                                                                                                                                                                           |
| L3 run_r1      | Robot          | `teleop_enabled` → **L2+Phải** (mặc định TẮT) → chỉ `kLocomotion` → balance guard tilt/gyro → watchdog 300ms → weight-ramp + smooth + slew + mask obs + **bản vá mục 3** (NaN check + clamp tay + bind local) |
| L4 Con người | R3 (safety op) | L2+Phải bấm lại = ngắt tức thì;**L2+B = E-stop** xả lực toàn thân                                                                                                                                                           |

**Auto-start bridge an toàn (như voice):** bridge không ghi lowcmd; gói phải lọt toàn bộ L3 mới thành chuyển động → chạy nền 24/7 không thể tự làm robot động đậy. (Khác run_r1 — vẫn KHÔNG auto-start.)

---

## 3. 🔴 Bản vá hardening run_r1 — BẮT BUỘC trước khi teleop robot thật

Audit code thật `TeleopReceiver.hpp` phát hiện 2 lỗ hổng + 1 phòng xa. Sửa nhỏ, gói gọn trong twin `TeleopReceiver` + `RobotSpec`/`Tuning`. **Không vá thì không lên Phase 4.**

| # | Lỗ hổng (code hiện tại)                        | Hậu quả                                                                                                  | Vá                                                     |
| - | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| 1 | **Không kiểm NaN** trong gói              | NaN lọt qua smoothing →`cur_arm_=NaN` → **lệnh NaN xuống motor**                              | `std::isfinite` toàn bộ float; fail → drop gói    |
| 2 | **Không clamp góc TAY** (chỉ clamp đầu) | Bridge lỗi gửi góc ngoài dải → tay bị kéo tới đó (slew chỉ hãm tốc độ, không hãm đích) | Bảng giới hạn 5 khớp/tay (dưới), margin ~0.05 rad |
| 3 | Cổng 5560 nhận gói từ mọi interface           | Máy lạ trên LAN robot bắn gói giả                                                                    | Khoá`teleop_bind_local: true` → bind `127.0.0.1`  |

Dải khớp tay thật (từ `r1.xml`) dùng cho clamp:

| Khớp (policy trái/phải) | Trái min…max (rad) | Phải min…max (rad)                            |
| -------------------------- | -------------------- | ----------------------------------------------- |
| shoulder_pitch (14/19)     | −3.14 … 2.09       | −3.14 … 2.09                                  |
| shoulder_roll (15/20)      | −0.23 … 2.48       | **−2.48 … 0.23** *(bất đối xứng)* |
| shoulder_yaw (16/21)       | −1.92 … 1.92       | −1.92 … 1.92                                  |
| elbow (17/22)              | −0.98 … 2.19       | −0.98 … 2.19                                  |
| wrist_roll (18/23)         | −1.92 … 1.92       | −1.92 … 1.92                                  |

> Nguyên tắc: bridge có validate (L1) nhưng **robot-side phải tự vệ độc lập** — không tin nguồn ngoài, kể cả bridge của chính mình.

---

## 4. Cấu trúc file

### 4.1 TẠO MỚI — `HB/controller/teleop_bridge/` (deploy cùng nhịp controller)

```text
teleop_bridge/
├── bridge.py              # HTTPS static + WSS server + vòng 50Hz → UDP UTL1
├── retarget.py            # pose cổ tay/đầu → 5 khớp/tay + head (dấu/scale/offset từ config)
├── utl1.py                # đóng gói UDP UTL1 (dùng chung với scripts/teleop_send_test.py)
├── config.yaml            # port, token, scale tay, clamp, sign/offset từng khớp, smooth, head on/off
├── web/
│   ├── index.html         # nút "VÀO AR", trạng thái WS/grip/Hz to rõ
│   ├── app.js             # XR session passthrough, đọc pose, WS 50Hz, deadman, calib-gộp
│   └── style.css
├── certs/                 # cert.pem/key.pem tự ký (make_cert.sh tạo; KHÔNG commit key)
├── make_cert.sh           # tạo cert 1 lần (WebXR bắt buộc HTTPS)
├── teleop-bridge.service  # systemd unit — auto-start như voice (Restart=always)
└── README.md              # cài 1 lần trên Jetson + vận hành + xử lý sự cố
```

### 4.2 SỬA

| File                                                                          | Sửa gì                                                                                                          | Ghi chú                                           |
| ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `src/input/TeleopReceiver.hpp` (twin cả 2 bên) + `RobotSpec`/`Tuning` | **Bản vá hardening mục 3**                                                                               | Sửa run_r1 duy nhất trong plan; cần bạn duyệt |
| `config/tuning.yaml`                                                        | `teleop_enabled: true`, `teleop_blend_in_s: 0.8–1.0` (takeover êm — mục 7.2), `teleop_bind_local: true` | Đổi giá trị khi deploy                         |
| `docs/huong_dan_van_hanh.md`                                                | Mục "Teleop VR": quy trình 2 người + checklist                                                                | Đồng bộ docs khi code xong                      |
| `scripts/teleop_send_test.py`                                               | Refactor nhỏ: import chung`utl1.py`                                                                            | Không đổi hành vi                              |

### 4.3 KHÔNG SỬA

- Toàn bộ C++ còn lại của run_r1 (overlay/Application/LowCmdSender/GamepadR3…) — hợp đồng UDP giữ nguyên.
- Sim `simulate_cpp` (test bằng cách trỏ UDP của bridge sang máy sim).
- `xr_teleoperate-main` — không dùng, giữ tham khảo.

---

## 5. Thiết kế thành phần

### 5.1 Trang WebXR (`web/`)

- `immersive-ar` (passthrough) — operator nhìn robot thật, không cần stream hình.
- Mỗi frame: đọc pose `gripSpace` 2 controller + `viewerPose` (đầu); gửi WS **50Hz**.
- **Deadman:** `enable=1` chỉ khi đang giữ grip (mặc định 1 trong 2 grip là đủ — config).
- **Calib gộp vào cử chỉ bắt đầu:** lần đầu **giữ CẢ 2 grip 1s, hai tay buông dọc thân** → bridge tự calib (gốc vai, chiều dài tay, hướng đầu chuẩn) rồi mới bắt đầu `enable=1`. Không có bước calib riêng phải nhớ. **A** (giữ 1s) = calib lại; **B** = recenter đầu.
- UI to rõ qua passthrough: WS ✔/✖, grip ✔/✖, Hz, cảnh báo mất kết nối.
- Tab ẩn/screensaver → XR session end → WS đóng → bridge tự `enable=0`.

### 5.2 Bridge (`bridge.py`)

- 1 process python (`asyncio` + `websockets` + `ssl` chuẩn — phụ thuộc tối thiểu), serve luôn static HTTPS.
- Vòng 50Hz: pose mới nhất → retarget → clamp góc + **rate-limit** (rad/s, config) → UTL1 `seq++` → UDP localhost:5560.
- Mất pose >150ms hoặc WS đóng → 5 gói `enable=0` → ngừng phát (watchdog run_r1 là lưới cuối).
- Token URL bắt buộc; chỉ 1 phiên WS (phiên 2 bị từ chối kèm thông báo).
- Log xoay vòng: seq, Hz thực, drop, min/max góc.

### 5.3 Retarget analytic 5-DoF (`retarget.py`) — không URDF/IK

Mỗi tay, trong hệ đã calib (gốc tại vai operator):

- Vector vai→cổ tay `v` (scale người→robot đo lúc calib): hướng `v` → `shoulder_pitch` (dấu đã kiểm chứng: **ÂM = ra trước**) + `shoulder_roll`.
- Độ duỗi `‖v‖` → `elbow` (map tuyến tính, clamp).
- Hướng cẳng tay/controller → `shoulder_yaw`; roll controller quanh trục cẳng tay → `wrist_roll`.
- Mỗi khớp có `sign/offset/scale/clamp` trong `config.yaml` → chỉnh trong sim, không sửa code.
- Đầu: quat đầu − hướng chuẩn → `head_yaw/pitch` (bật/tắt config; v1 **OFF** tới khi W2 xác nhận).
- Giới hạn phần cứng: 5-DoF → không tái tạo xoay cổ tay đầy đủ (chấp nhận).

### 5.4 Protocol WS (browser → bridge)

```json
{"t": 12345.6, "seq": 1001, "tok": "…",
 "l": {"p":[x,y,z], "q":[w,x,y,z], "grip":1},
 "r": {"p":[x,y,z], "q":[w,x,y,z], "grip":0},
 "h": {"q":[w,x,y,z]}, "btnA":0, "btnB":0}
```

Gói UDP `UTL1` ra ngoài **không đổi** (60B, magic, seq, enable/arm_valid/head_valid, arm_q[10], head[2]).

### 5.5 systemd (`teleop-bridge.service`)

`Restart=always`, `After=network-online.target`, user thường, theo mẫu `scripts/install_service.sh`. **Bạn tự cài** (nguyên tắc không tự deploy).

---

## 6. Ổn định (ngân sách trễ & tải)

| Khâu                 | Con số thiết kế                                                            |
| --------------------- | ----------------------------------------------------------------------------- |
| WebXR pose            | 60–90Hz (Quest)                                                              |
| WS                    | 50Hz, ~300B/frame → ~15KB/s                                                  |
| Bridge                | vài chục phép toán/frame →**<3% CPU Jetson**, đo thực ở Phase 3 |
| UDP localhost         | ~0ms                                                                          |
| Làm mượt cuối     | `teleop_smooth_hz=8` + slew sender (đã có) — hấp thụ jitter WiFi      |
| Tổng trễ mục tiêu | ~60–100ms (đủ cho múa/vẫy; không phải thao tác chính xác)           |

WiFi rớt giữa động tác: bridge phát hiện mất pose → `enable=0`; bridge chết → watchdog 300ms run_r1. **Hai lưới độc lập.**

---

## 7. Vận hành trong show (quy tắc BẮT BUỘC + checklist)

### 7.1 Quy tắc 2 người

Performer đeo Quest **2 tay cầm controller — không thể cầm R3**:

- **Performer** (Quest): điều khiển tay/đầu; deadman = grip.
- **Safety op** (R3): giữ suốt buổi — **L2+Phải** arm/disarm, **L2+B** E-stop. Người quyết định cho/thu quyền.

Một người kiêm cả hai = KHÔNG chấp nhận cho show thật.

### 7.2 Quy tắc chiếm quyền êm (chống tay quét)

1. Performer **buông thõng hai tay** (gần tư thế default robot) rồi giữ 2 grip.
2. Safety op mới bấm **L2+Phải**.
3. `teleop_blend_in_s` = **0.8–1.0s** khi deploy (mặc định code 0.4s hơi gắt cho takeover).

### 7.3 Checklist 30 giây trước show

- [ ] Quest: pin ≥50%, nối WiFi robot, mở bookmark → **WS ✔**
- [ ] R3: safety op cầm, pin OK
- [ ] run_r1 đã arm (bàn giao như thường), đang LOCOMOTION
- [ ] Thử 1 nhịp: grip → L2+Phải → vẫy nhẹ → thả grip (tay tự về) → L2+Phải tắt

### 7.4 Sự cố → hành động (in dán cánh gà)

| Hiện tượng             | Hành động                                                         |
| ------------------------- | -------------------------------------------------------------------- |
| Tay robot không theo     | UI kính: WS ✖? → mở lại bookmark. Grip ✔? L2+Phải bật chưa? |
| Tay lệch/ngược         | Thả grip → giữ 2 grip 1s (calib lại)                             |
| Robot lắc khi teleop     | Safety op**L2+Phải TẮT** → tay về policy; giảm biên độ |
| Mất kiểm soát bất kỳ | **L2+B (E-stop)** — xả lực toàn thân                      |

---

## 8. Lộ trình test (an toàn tăng dần — sim trước, robot sau)

| Phase | Môi trường       | Nội dung                                                                                                                                                     | Tiêu chí đạt                                                                                 |
| ----- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| 0     | —                  | **Bản vá hardening mục 3** + test đơn vị (NaN/clamp/bind)                                                                                         | Build sạch 2 bên; gói NaN/ngoài dải bị chặn                                               |
| 1     | PC dev, chưa Quest | Bridge + bộ phát pose GIẢ → sim`run_policy`                                                                                                             | Format đúng; 2 lưới watchdog chạy; tay sim bám pose giả                                   |
| 2     | PC dev + Quest      | Quest → WebXR → bridge →**sim**                                                                                                                      | Chốt dấu/scale/offset từng khớp; deadman nhả đúng; đo tilt/gyro khi vừa đi vừa teleop |
| 3     | Jetson (bạn cài)  | Bridge systemd trên robot; run_r1 chạy,**CHƯA L2+Phải**. **Kiểm combo L2+Phải** với built-in (cả lúc sống & dev mode — bài học L2+Y) | Log thấy gói, weight=0; sống qua reboot; L2+Phải sạch với built-in                         |
| 4     | Robot thật         | Đứng yên → LOCOMOTION đứng → biên nhỏ (giảm`scale`) → tăng dần → đi chậm + teleop                                                           | Không lắc quá ngưỡng; L2+Phải ngắt tức thì; thả grip nhả mượt                       |

---

## 9. Rủi ro & giảm thiểu

| Rủi ro                                      | Giảm thiểu                                                  |
| -------------------------------------------- | ------------------------------------------------------------- |
| Gói NaN / góc ngoài dải từ bridge lỗi  | **Bản vá mục 3 (bắt buộc)** — robot-side tự vệ  |
| WiFi rớt giữa động tác                  | 2 lưới watchdog → fade về policy, không đứng hình tay |
| Quest screensaver / thoát app               | XR end → WS đóng →`enable=0`                            |
| Máy lạ trên LAN mở trang / bắn UDP giả | Token URL + 1 phiên WS +`teleop_bind_local`                |
| Dấu khớp ngược                           | sign/offset trong`config.yaml`, chốt ở Phase 2 trong sim  |
| Tay quét nhanh lúc arm                     | Quy tắc 7.2 + blend_in 0.8–1.0s                             |
| L2+Phải trùng combo built-in               | Kiểm chứng Phase 3; trùng → đổi combo                   |
| Cert hết hạn (365 ngày)                   | README + log cảnh báo trước 30 ngày                      |
| Jetson quá tải                             | Bridge <3% CPU; đo thực Phase 3                             |
| Operator nhầm nút/che grip                 | UI to trong kính + checklist 7.3                             |

---

## 10. Dữ kiện còn thiếu — CẦN BẠN

| #  | Cần                                                                    | Ảnh hưởng                                                                              |
| -- | ----------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| W1 | **WiFi AP chung mạng robot** (`192.168.123.x`) cho Quest       | Tiên quyết Phase 3–4                                                                   |
| W2 | Đầu robot**có motor yaw/pitch (idl 29/30) thật** + dải góc? | Chưa xác nhận → v1`head: off`, chỉ tay                                             |
| W3 | Chốt biến thể tay: 5-DoF hay 7-DoF+bàn tay?                         | v1 luôn xuất 5 khớp (khớp run_r1). 7-DoF muốn dùng đủ → dự án mở rộng riêng |
| W4 | **Duyệt bản vá hardening mục 3** — nói "làm đi" là code  | **Bắt buộc trước robot thật** — không vá không lên Phase 4                |

---

## 11. Ngoài scope v1 (cắt chủ đích để ổn định)

- Hand-tracking tay trần (kém ổn định hơn controller — v2).
- Stream camera vào kính (không cần — passthrough).
- Bàn tay/ngón, wrist pitch/yaw (phụ thuộc W3, cần mở rộng run_r1).
- Per-arm enable (UTL1 hiện 1 cờ chung — nâng protocol phải sửa twin 2 bên).
- Ghi/replay phiên teleop thành gesture npz (hay — để v2, nối với thư viện gesture).
