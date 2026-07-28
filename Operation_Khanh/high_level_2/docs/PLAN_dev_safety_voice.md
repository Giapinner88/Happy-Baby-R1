# PLAN — Dev-mode + Safety hợp nhất + Ngồi ghế chuẩn + Voice tiếng Việt

> Trạng thái: **ĐÃ THỰC HIỆN & BUILD OK** (2026-07-12) — chờ TEST TRÊN ROBOT.
> Phạm vi: `HB/high_level_2` (runner high-level của R1).
>
> ---
> ## ⛔ PHẦN "NGỒI GHẾ" TRONG FILE NÀY ĐÃ LỖI THỜI (2026-07-14)
>
> Mọi chỗ dưới đây nói **ngồi 3 pha** và **feet-flat = `ankle = -(hip+knee)`**
> đều **KHÔNG CÒN ĐÚNG**. Công thức đó ép `hip+gối+cổ_chân = 0`, tức **khóa thân
> thẳng đứng**; khi gập gối thì hông lùi ra sau, trọng tâm rơi ngoài gót →
> **robot ngã ngửa** (đo MuJoCo: biên **−9.8 cm**).
>
> Bản đúng: **ngồi 4 pha**, thân **đổ về trước** + **vươn tay**, bàn chân bám
> **mặt đất qua IMU**. Xem **`PLAN_sit_balanced.md`** và §3.5 của
> `huong_dan_van_hanh.md`. Các phần khác của file này (dev-mode, safety, voice)
> vẫn còn hiệu lực.
> ---

## ✅ Quyết định đã chốt & đã code

| Mục | Chốt | Trạng thái |
|---|---|---|
| Voice | text→TTS / path→file; test TTS tiếng Việt trước | ✅ code (cần test loa) |
| Nút 9 | ghế cố định sau lưng, gối 90°, feet-flat | ✅ code (cần test có ghế) |
| Safe-stop khi đang đi | **GIỮ**: ép lệnh vận tốc 0 → đứng yên tại chỗ (KHÔNG squat) | ✅ code (cần test) |
| Safe-stop khi STAND_LOCK | giữ đứng | ✅ code |
| Có/không SSH | headless systemd, mất SSH vẫn R3-1 | ✅ code |
| Mất gamepad khi có bàn phím | vẫn chạy bằng bàn phím (không safe-stop) | ✅ code |

**Đính chính quan trọng (theo yêu cầu):** safe-stop khi đang đi = **dừng lại & đứng
yên tại chỗ** (ép `vx=vy=yaw=0`, giữ policy locomotion tự cân bằng), **KHÔNG** hạ
squat như bản thảo. Vì vậy tư thế squat "CoM trên chân" **bỏ khỏi safe-stop**;
yếu tố CoM/dev-kit chỉ còn liên quan nút 9 (có ghế đỡ).

### Đã build + smoke-test (không cần robot)
- `scripts/build.sh` OK trên x86_64.
- Khởi động headless nạp đủ: flat `policy_r1_1.onnx`, dance 2/3/**4=pokemon**;
  AudioClient "sẵn sàng (voice=bật)"; bàn phím tự bỏ qua khi `$DISPLAY` trống.

### ⚠ CẦN TEST TRÊN ROBOT (có người đỡ/treo)
- Chất lượng **TTS tiếng Việt** (nếu tệ → thay bằng đường dẫn file thu sẵn).
- **Nút 9 ngồi ghế** feet-flat (đặt ghế đúng chiều cao ↔ `sit_knee_deg`).
- **Safe-stop**: rút R3-1 khi đang đứng và khi đang đi (kiểm tra "đứng yên").
- Xác minh tín hiệu "gamepad còn sống" (`wireless_remote` về 0 khi rớt R3-1).

---

> Bản kế hoạch gốc bên dưới (giữ để tham chiếu).

Mục tiêu người dùng đặt ra:
1. Gom mọi xử lý an toàn (TRỪ ngã trong fall-detect = damping) về một cơ chế **theo trạng thái**:
   - đang **STAND_LOCK** → **giữ nguyên form** (không sụp);
   - đang **LOCOMOTION/MIMIC** → **về đứng** rồi **ngồi/squat xuống**, tư thế sao cho **trọng tâm (CoM) rơi vào bàn chân, tránh ngã** (lưu ý: có bộ phát triển sau lưng → **CoM lệch ra sau**, phải bù bằng nghiêng thân ra trước).
2. **Có hay không có SSH đều điều khiển được**. Mất SSH vẫn an toàn vì còn R3-1.
3. **Sửa nút 9 (ngồi xuống)** cho chuẩn: **có ghế**, **2 bàn chân phẳng với mặt đất**.
4. **Voice tiếng Việt**, để trong `tuning.yaml` cho dễ đổi, phát tại các mốc:
   1) khởi động thành công trên máy phát triển; 2) sang STAND_LOCK; 3) sang LOCOMOTION;
   4) ngồi xuống (nút 9); 5) chế độ an toàn (safe-stop/emergency); 6) từng policy mimic.

---

## 0. Nguyên tắc thiết kế (đọc trước)

- **Fall-detect giữ nguyên = Damping.** Khi đã ngã thật, mềm ra để hấp thụ va đập; khóa cứng = ngã như tấm ván, hại hộp số. KHÔNG gom vào cơ chế mới.
- **Mất DDS `rt/lowstate` giữ nguyên = Damping.** High-level bị "mù" (không có cảm biến mới) → không được gồng giữ tư thế theo dữ liệu cũ.
- **"Khóa/giữ form" chỉ an toàn từ tư thế tĩnh-ổn định.** Khóa cứng GIỮA lúc đang bước → đổ. Vì vậy safe-stop phải **theo trạng thái**, không áp một kiểu cho tất cả.
- **Mất SSH/bàn phím KHÔNG phải sự cố vật lý** nếu R3-1 còn sống → chỉ chuyển sang điều khiển thuần gamepad, robot chạy tiếp.
- **Chỉ khi mất TOÀN BỘ input** (gamepad + bàn phím) mà DDS còn sống → mới chạy **safe-stop theo trạng thái**.
- Mọi tham số mới đều nằm ở `config/tuning.yaml` (không cần build lại khi chỉnh), trừ các giá trị train-locked.

---

## 1. PHASE 0 — Sửa nhanh đã duyệt (không rủi ro)

Gộp các quyết định đã chốt ở vòng trước:

| # | Việc | File | Ghi chú |
|---|---|---|---|
| 0.1 | README: `~/HB/high_level/build` → `~/HB/high_level_2/build` (dòng 43) | `README.md` | chỉ văn bản |
| 0.2 | README: bỏ ví dụ `./run_r1 eth10 flat # không nạp dance` → `./run_r1 eth10 # override card mạng` (dòng 46) | `README.md` | chỉ văn bản |
| 0.3 | Bỏ mode `flat`: xóa biến `mode` + đọc `argv[2]`, sửa comment header | `src/main.cpp` | xóa code chết, không đổi hành vi |
| 0.4 | `flat_model` default `policy_r1_flat_2.onnx` → `policy_r1_1.onnx` (file có thật) | `src/config/Tuning.hpp:133` | tránh FATAL khi thiếu tuning.yaml |
| 0.5 | Khai `pokemon` thành `dance_4` | `config/tuning.yaml` | bấm phím 4 / R1+Xuống |

**Không xóa** tài sản nào (theo lựa chọn của bạn): giữ `_unused/`, `config.yaml`, các onnx flat dự phòng.

Ví dụ 0.5 thêm vào `tuning.yaml`:
```yaml
# Phím 4 (R1 + Xuống)
dance_4: pokemon
dance_speed_4: 1.0
dance_volume_4: 0.7
```

---

## 2. PHASE 1 — Điều khiển có/không SSH (gamepad-first)

### 2.1 Hiện trạng (đã đúng một phần)
- `GamepadR3` đọc nút từ `wireless_remote` trong lowstate → **không dính X11**. Chạy `./run_r1 eth10` qua SSH **không** `-Y` (không `$DISPLAY`) thì `KeyboardX11` tự báo "chỉ điều khiển bằng gamepad" và bỏ qua. **Gamepad đã điều khiển được toàn bộ luồng** (Stand Lock, Locomotion, Dance 2–6, tốc độ, ngồi xuống, E-stop, di chuyển).
- **Thiếu:** map nút Reset và dance 7–8 trên gamepad (nhỏ).

### 2.2 Vấn đề cần giải quyết cho "mất SSH vẫn an toàn"
1. **SIGHUP khi rớt SSH**: nếu chạy tiền cảnh trong phiên SSH, rớt SSH → shell cha chết → process nhận SIGHUP → chết. Robot mất lệnh → low-level tự damping.
2. **X11 IO error khi rớt SSH -Y**: nếu ĐÃ mở cửa sổ X11 (chạy `-Y`), rớt SSH làm đứt kết nối X → `XIOErrorCb` hiện **damp + `exit(1)`** → robot sụp + process chết.
   Lưu ý kỹ thuật: handler IO-error của Xlib **không được phép return** (Xlib sẽ tự `exit` sau đó) → **không thể** "chạy tiếp" ngay trong handler.

### 2.3 Giải pháp — hai hồ sơ chạy (launch profiles)

**(A) Hồ sơ FIELD/HEADLESS (khuyến nghị cho vận hành thật):**
- Chạy **tách khỏi phiên SSH** bằng `systemd` (Phase 5) hoặc tạm `setsid nohup ./run_r1 eth10 >log 2>&1 &` / `tmux`.
- **Không mở X11** (không có `$DISPLAY`) → không có cửa sổ nào để mất → không bao giờ dính `XIOErrorCb`.
- Kết quả: rớt SSH **không đụng** tới process điều khiển; R3-1 điều khiển liên tục. **An toàn tuyệt đối theo đúng yêu cầu.**

**(B) Hồ sơ DEV-CÓ-MÀN-HÌNH (khi ngồi máy phát triển, NoMachine/local):**
- `./run_r1 eth10` từ terminal có X → **cả bàn phím lẫn gamepad**.
- Đây là tiện ích lúc phát triển. Nếu X đứt giữa chừng, xem 2.4.

### 2.4 Thêm cờ khởi động rõ ràng
- Thêm nhận diện: nếu `$DISPLAY` rỗng **hoặc** truyền `--no-keyboard` (hoặc `mode=dev`) → **không** gọi `XOpenDisplay`, chạy thuần gamepad ngay từ đầu (log gọn, không cảnh báo đỏ).
- `KeyboardX11` bổ sung `bool available()` = "đã mở được X display & thread đang chạy". Dùng cho watchdog ở Phase 2.

### 2.5 (Tùy chọn nâng cao) Sống sót khi X đứt lúc đang chạy hồ sơ (B)
- Vì Xlib ép `exit`, muốn process **sống tiếp** khi X đứt thì phải **tách X11/bàn phím sang tiến trình con** (fork): con chết không kéo main chết; main tiếp tục vòng điều khiển bằng gamepad.
- **Đề xuất:** để **sau**; trước mắt khuyến nghị dùng hồ sơ (A) cho thực địa. Trong hồ sơ (B), đổi `XIOErrorCb`: **không damp** (vì gamepad còn) mà chỉ log + để process thoát; kèm systemd `Restart=on-failure` để tự bật lại (robot có low-level fallback trong ~1s).

**Chốt hành vi mong muốn:** mất SSH → (A) không ảnh hưởng gì; (B) tệ nhất là process thoát rồi systemd bật lại, KHÔNG chủ động sụp robot.

---

## 3. PHASE 2 — Safety hợp nhất theo trạng thái (trừ fall = damping)

### 3.1 Định nghĩa "mất toàn bộ điều khiển"
Sửa lại phép trộn `is_active` (đang hỏng — bàn phím luôn báo sống):

```
is_active = gamepad.is_active  ||  keyboard.available()
```
- `gamepad.is_active` = có gói `wireless_remote` mới trong `remote_timeout_ms`.
- `keyboard.available()` = mở được X display (chỉ đúng ở hồ sơ B).

Hệ quả:
- Hồ sơ B (có bàn phím): `keyboard.available()=true` → mất gamepad KHÔNG safe-stop (còn bàn phím). Đúng.
- Hồ sơ A (headless): `keyboard.available()=false` → **mất gamepad = mất toàn bộ → safe-stop**. Đúng.

> ⚠ Cần **xác minh trên robot thật** tín hiệu "gamepad còn sống": heuristic hiện tại refresh khi có byte `wireless_remote` ≠ 0. Phải kiểm khi rớt R3-1 thì buffer có thực sự về 0 / hay đóng băng khung cũ. (Xem Phase 6.)

### 3.2 Bảng quyết định safe-stop (khi mất toàn bộ input, DDS còn sống)

| Trạng thái lúc mất input | Xử lý | Vì sao |
|---|---|---|
| IDLE | Damping (giữ nguyên) | Đã thả lực |
| STAND_UP (đang đứng dậy) | Tiếp tục đứng dậy → xong thì HOLD | tư thế đang tiến tới ổn định |
| **STAND_LOCK** | **HOLD form** (giữ PD stand-lock, không đổi gì) | tĩnh-ổn định, giữ là an toàn nhất |
| **LOCOMOTION** | → RETURNING (về default đứng) → **SAFE_SQUAT** (không ghế, CoM trên chân) → HOLD/Damp | không được khóa giữa bước |
| **MIMIC** | dừng nhạc → RETURNING → **SAFE_SQUAT** → HOLD/Damp | như trên |
| RETURNING / SAFE_SHUTDOWN / SAFE_STOP | tiếp tục chuỗi đang chạy | đã trong quy trình an toàn |

- Cấu hình `safe_stop_from_static`: `hold` (mặc định) | `sit` | `damp` cho nhóm tĩnh.
- Cuối chuỗi động: `safe_squat_release`: `hold` (giữ squat) | `damp` (xả sau khi ổn định).

### 3.3 SAFE_SQUAT — tư thế squat tự cân bằng (KHÔNG ghế)
Khác nút 9 (nút 9 có ghế). Đây là dừng khẩn không có ghế → **CoM phải rơi vào bàn chân**:
- Squat đối xứng: `hip_pitch = ankle_pitch = -knee/2` (hông ~thẳng trên cổ chân), `knee = safe_squat_knee_deg`.
- **Bù CoM lệch sau (dev kit sau lưng):** thêm **gập thân ra trước** `safe_squat_torso_forward_deg` vào `hip_pitch` (R1 không có waist_pitch → gập bằng hip_pitch), kéo CoM về phía mũi chân.
- Giữ **feet-flat**: tính `ankle_pitch = -(hip_pitch + knee)` MỖI tick (không chỉ ở điểm cuối) để bàn chân luôn phẳng khi hạ.
- Hạ chậm (`safe_squat_time_s`) với gains cứng (dùng `stand_*` hoặc `shutdown_*`).

### 3.4 Máy trạng thái — thay đổi
- Thêm state `kSafeStop` vào `enum AppState`.
- `HandleTransitions`/watchdog: khi phát hiện "mất toàn bộ input" → gọi `EnterSafeStop()`:
  - nếu tĩnh → set `kSafeStop` biến thể HOLD;
  - nếu động → set `kReturningToDefault` với cờ `pending_safe_squat_=true`; khi RETURNING về tới default → chuyển `kSafeStop` (SAFE_SQUAT).
- `RunSafeStop()`: nội suy tới squat/hold theo `safe_squat_time_s`, feet-flat mỗi tick, phát **voice "chế độ an toàn"** (một lần), rồi HOLD hoặc Damp theo config.
- Thoát `kSafeStop`: chỉ khi có input trở lại **và** người vận hành chủ động (vd nhấn 0/L2+Lên) → về STAND_LOCK; hoặc E-stop → Damping.

### 3.5 Phân biệt rõ với watchdog cũ
- Nhánh `!cmd.is_active` cũ (chết) được thay bằng logic 3.1–3.4.
- Mất DDS (`state_timeout_ms`) và fall-detect **vẫn Damping** như cũ.
- Tham số `remote_timeout_ms` **giờ có tác dụng thật** (quyết định gamepad sống/chết cho safe-stop).

---

## 4. PHASE 3 — Nút 9 "Ngồi ghế" chuẩn (feet-flat)

### 4.1 Vấn đề của bản hiện tại
`RunSafeShutdown` hiện làm chuỗi 3 pha (hạ + **gập người ra trước** → dừng → **dựng lưng**) + fallback "ngả sau" **chưa test**. Chuỗi này thiết kế để **tự cân bằng trên chân khi CHƯA có ghế**. Nhưng yêu cầu mới là **CÓ ghế** → không cần màn gập-trước/dựng-lưng phức tạp; chỉ cần **hạ thẳng xuống thành tư thế ngồi ghế, 2 bàn chân phẳng**.

### 4.2 Thiết kế mới (đơn giản, một chiều)
- Tư thế đích "ngồi ghế": `knee = sit_knee_deg` (≈90°), `hip_pitch = -knee` (cẳng chân thẳng đứng, đùi ~ngang), `ankle_pitch = 0` → **bàn chân phẳng**, mông tựa ghế.
- **Feet-flat suốt quá trình:** tham số hạ theo một biến độ sâu `s∈[0,1]`; mỗi tick tính `ankle = -(hip(s) + knee(s))` để chân luôn phẳng (không chỉ ở cuối như bản cũ nội suy tuyến tính).
- **Bù dev-kit (CoM sau):** vì có ghế đỡ, CoM lệch sau lại **có lợi** (dồn trọng lượng lên ghế). Nhưng lúc **đang hạ, trước khi mông chạm ghế**, CoM sau có thể gây chồm ngửa → cho tuỳ chọn nhỏ `sit_torso_forward_deg` (mặc định nhỏ, vd 5–8°) nghiêng thân ra trước trong lúc hạ; khi đã ngồi thì về thẳng.
- Bỏ (hoặc tắt mặc định) pha gập-trước/dựng-lưng và fallback "ngả sau" chưa kiểm chứng. Có thể giữ lại sau cờ `sit_legacy_sequence: false`.
- Kết thúc: giữ lực ngồi `sit_hold_s` rồi `sit_release_after` (`true`=damp / `false`=giữ).
- Phát **voice "đang ngồi xuống"** khi bắt đầu.

### 4.3 Cần bạn cung cấp để chỉnh chính xác
- **Chiều cao ghế** (hoặc góc gối mong muốn khi mông chạm ghế). Nếu ghế thấp/cao khác 90° gối, chỉnh `sit_knee_deg`.
- Xác nhận quy ước dấu khớp để công thức feet-flat `ankle=-(hip+knee)` đúng chiều (bản cũ đã dùng quan hệ này — sẽ tái dùng và kiểm bằng log tư thế).

---

## 5. PHASE 4 — Voice tiếng Việt (config-driven)

### 5.1 Hạ tầng
- Robot đã có `unitree::robot::a2::AudioClient` (đang dùng cho nhạc trong `MusicPlayer`). Hai cách phát voice:
  - **(a) TTS**: `AudioClient::TtsMaker(text, speaker_id)` — đọc thẳng text trong config → **"dễ đổi" nhất** (sửa chữ trong tuning.yaml là xong).
  - **(b) File thu sẵn**: đặt file .wav/.mp3 tiếng Việt, phát qua đúng pipeline ffmpeg→PCM→`PlayStream` đã chạy tốt → **chất lượng/độ tự nhiên đảm bảo**.
- **Đề xuất linh hoạt:** một trường config cho mỗi mốc; nếu giá trị là **đường dẫn file tồn tại** → phát file (b); ngược lại coi là **text** → TTS (a). Vừa dễ đổi (gõ text), vừa cho phép nâng cấp lên file thu sẵn khi cần.

> ⚠ **Rủi ro:** chất lượng **TTS tiếng Việt** trên firmware Unitree chưa chắc tốt. **Action sớm (Phase 6):** test `TtsMaker` một câu tiếng Việt trên loa robot. Nếu tệ → dùng file thu sẵn (đã có sẵn ffmpeg pipeline).

### 5.2 Kiến trúc phần mềm
- Tách/kmở rộng `MusicPlayer` thành lớp audio dùng chung (hoặc thêm `VoicePlayer`) **dùng chung `AudioClient`** để **nối tiếp giọng nói và nhạc** (một loa) — tránh chồng tiếng:
  - `Say(event)` đưa yêu cầu voice vào cùng thread audio, **serialize** với nhạc.
  - Mimic: phát voice tên bài (ngắn) **trước**, xong mới `Play` nhạc nền.
- **InitAudio** phải chạy nếu `voice_enabled` **HOẶC** có nhạc (hiện chỉ init khi có nhạc).
- Mọi lỗi audio chỉ log, **không** ảnh hưởng vòng 500Hz (giữ nguyên triết lý thread riêng, non-blocking như `MusicPlayer` hiện tại).

### 5.3 Bảng sự kiện → voice
| Mốc | Chỗ gọi trong code | Key config (gợi ý text) |
|---|---|---|
| 1. Khởi động thành công (đã kết nối robot + audio sẵn sàng) | sau khi chuyển WAITING→IDLE và audio ready | `voice_startup: "Máy tính phát triển đã sẵn sàng"` |
| 2. STAND_LOCK | khi vào `kStandLock` | `voice_stand_lock: "Đã khóa đứng"` |
| 3. LOCOMOTION | khi `ActivatePolicy(locomotion)` | `voice_locomotion: "Bật chế độ đi bộ"` |
| 4. Ngồi xuống (nút 9) | đầu `RunSafeShutdown` | `voice_sit_down: "Đang ngồi xuống"` |
| 5. Chế độ an toàn | đầu `EnterSafeStop` (và/hoặc `EnterIdle` do E-stop/ngã) | `voice_safe_stop: "Kích hoạt chế độ an toàn"` |
| 6. Từng mimic | khi vào `kMimic` (theo `pending_dance_key_`) | `voice_mimic_2..8: "..."` |

- Thêm `voice_enabled`, `voice_volume`. Voice nên **ngắn (<2s)**, phát ở thời điểm chuyển trạng thái (không lặp).
- Cân nhắc: mốc 5 nếu do **ngã/mất DDS** thì loa vẫn phát được không? (mất DDS = mất audio DDS luôn). Ghi chú: voice "an toàn" chỉ đảm bảo phát khi kênh DDS còn sống (safe-stop do mất input); khi mất DDS thì có thể không kịp phát.

---

## 6. tuning.yaml — toàn bộ key mới (gộp)

```yaml
# ── Dev-mode / điều khiển ───────────────────────────────────
# Bỏ qua bàn phím X11, chạy thuần gamepad (tự bật nếu không có $DISPLAY).
dev_no_keyboard: false

# ── Safe-stop tự động (mất TOÀN BỘ input; KHÔNG phải ngã/mất DDS) ──
safe_stop_enabled: true
safe_stop_from_static: hold      # hold | sit | damp  (khi đang STAND_LOCK)
safe_squat_knee_deg: 70          # độ sâu squat tự cân bằng (không ghế)
safe_squat_time_s: 3.0           # thời gian hạ vào squat
safe_squat_torso_forward_deg: 12 # gập thân ra trước bù CoM lệch sau (dev kit)
safe_squat_release: hold         # hold | damp  (sau khi vào squat & ổn định)
# (remote_timeout_ms cũ giờ CÓ tác dụng: ngưỡng coi gamepad đã chết)

# ── Nút 9 "Ngồi ghế" (CÓ ghế, feet-flat) ────────────────────
sit_knee_deg: 90            # góc gối khi ngồi ghế (chỉnh theo chiều cao ghế)
sit_descent_time_s: 4.0     # thời gian hạ xuống (chậm/êm)
sit_hold_s: 0.5             # giữ lực trước khi xả
sit_torso_forward_deg: 6    # nghiêng thân ra trước lúc hạ (bù CoM sau); 0 = thẳng
sit_release_after: false    # true = damp sau khi ngồi; false = giữ lực ngồi
sit_legacy_sequence: false  # true = dùng lại chuỗi gập-trước/dựng-lưng + fallback cũ

# ── Voice tiếng Việt ────────────────────────────────────────
voice_enabled: true
voice_volume: 0.9           # 0..1
# Mỗi mốc: gõ TEXT (đọc bằng TTS) HOẶC đường dẫn file .wav/.mp3 (thu sẵn).
voice_startup:    "Máy tính phát triển đã sẵn sàng"
voice_stand_lock: "Đã khóa đứng"
voice_locomotion: "Bật chế độ đi bộ"
voice_sit_down:   "Đang ngồi xuống"
voice_safe_stop:  "Kích hoạt chế độ an toàn"
voice_mimic_2:    "Nhảy bài lắc mông một"
voice_mimic_3:    "Nhảy bài lắc mông hai"
voice_mimic_4:    "Nhảy Pokemon"
voice_mimic_5:    ""
voice_mimic_6:    ""
voice_mimic_7:    ""
voice_mimic_8:    ""
```

---

## 7. Danh sách file sẽ đụng (dự kiến)

| File | Phase | Loại thay đổi |
|---|---|---|
| `README.md` | 0 | văn bản |
| `config/tuning.yaml` | 0,2,3,4 | thêm key (0.5 pokemon + toàn bộ mục 6) |
| `src/main.cpp` | 0,1 | bỏ `flat` mode; nhận `--no-keyboard`/`$DISPLAY` |
| `src/config/Tuning.hpp` + `Tuning.cpp` | 0,2,3,4 | default `flat_model`; parse key mới |
| `src/input/KeyboardX11.hpp/.cpp` | 1,2 | `available()`; đổi `XIOErrorCb` (không damp/không exit theo hồ sơ); bỏ qua X khi dev |
| `src/input/GamepadR3.hpp` | 1(,4) | (tùy) map Reset/dance 7–8; xác minh tín hiệu "sống" |
| `src/input/InputManager.hpp` | 2 | trộn `is_active` mới |
| `src/app/Application.hpp/.cpp` | 2,3,4,5 | state `kSafeStop`; `EnterSafeStop`/`RunSafeStop`; viết lại `RunSafeShutdown` (nút 9); chèn gọi voice tại các mốc |
| `src/audio/MusicPlayer.hpp` (→ + `VoicePlayer`) | 4 | `Say()` + serialize voice/nhạc; init khi `voice_enabled` |
| `src/robot/LowCmdSender.hpp` | 3,4 | (nếu cần) tiện ích giữ-form/feet-flat |
| `scripts/` + (mới) `systemd unit` | 5 | tự khởi động headless |

**Train-locked KHÔNG đụng:** `RobotSpec.hpp` (thứ tự khớp, gains policy, action scale, gait) và logic obs/inference.

---

## 8. Thứ tự thực thi + ma trận rủi ro & test

| Bước | Nội dung | Rủi ro | Test |
|---|---|---|---|
| **A** | Phase 0 (README, flat mode, flat_model, pokemon) | Rất thấp | build thử |
| **B** | Phase 1 hồ sơ headless + `dev_no_keyboard`/`available()` | Thấp | chạy `./run_r1` không `$DISPLAY`, R3-1 điều khiển đủ luồng |
| **C** | Phase 4 Voice — hạ tầng + test TTS tiếng Việt sớm | Thấp–TB | phát thử 1 câu; quyết TTS hay file |
| **D** | Phase 3 Nút 9 ngồi ghế feet-flat | **TB–Cao** | **treo/đỡ robot + có ghế thật**; xem log tư thế |
| **E** | Phase 2 Safe-stop hợp nhất (HOLD tĩnh / squat động) | **Cao (vùng an toàn)** | **BẮT BUỘC người đỡ/treo**; mô phỏng mất R3-1 khi đứng và khi đi |
| **F** | Phase 5 systemd auto-start headless | Thấp (ops) | trên robot, thử rớt SSH |

Khuyến nghị: **A→B→C** một đợt (thấp rủi ro); **D** đợt có ghế + người đỡ; **E** đợt riêng có người đỡ, làm cẩn thận; **F** cuối.

---

## 9. Câu hỏi cần chốt trước khi code

1. **Voice:** ưu tiên **TTS text-in-config** (dễ đổi, nhưng cần test chất lượng tiếng Việt) hay **file thu sẵn** (chất lượng chắc, đổi bằng thay file)? (Đề xuất: làm cơ chế "text→TTS, path→file", test TTS trước.)
2. **Nút 9:** cho tôi **chiều cao ghế / góc gối** mong muốn (mặc định 90°). Ghế đặt cố định phía sau robot?
3. **Safe-stop kết thúc:** khi đang đi mà mất input → sau khi squat, **giữ nguyên (hold)** hay **xả lực (damp)**? (Đề xuất: hold, để robot không sụp.)
4. **Safe-stop khi STAND_LOCK:** `hold` (giữ đứng) đúng ý chứ, hay muốn tự **ngồi xuống** luôn?
5. **Hồ sơ chạy chính thức:** đồng ý **systemd headless** là cách vận hành thực địa (mất SSH vẫn chạy)? Còn cần giữ bàn phím X11 cho lúc dev không?
6. **Mất gamepad khi ĐANG dùng bàn phím** (hồ sơ B): giữ chạy bằng bàn phím (không safe-stop) — đúng chứ?

> Sau khi bạn trả lời mục 9, tôi sẽ triển khai theo thứ tự A→F, mỗi bước xong sẽ báo để bạn kiểm trước khi sang bước có rủi ro cao.
