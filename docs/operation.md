# Hướng dẫn vận hành R1 (`controller` / `run_r1`)

Bảng nút và cổng: [controls.md](controls.md). Quy tắc an toàn: [safety.md](safety.md). Teleop: [teleop.md](teleop.md).

Chạy `controller` theo **2 cách**:

- **Cách A — Headless + Auto (thực địa):** điều khiển bằng **tay cầm R3-1**, chạy nền trong robot. **Mất wifi/SSH vẫn chạy.** ← dùng chính.
- **Cách B — Chạy tay có bàn phím (dev/test):** bàn phím laptop qua `ssh -Y`. **Mất SSH sẽ dừng.**

## ⚠ 3 điều an toàn phải nhớ trước tiên

1. **KHÔNG chạy 2 bản `run_r1`.** Cả hai cùng ra `rt/lowcmd` → đánh nhau, phá robot. Muốn chạy Cách B phải **tắt Auto trước** (§4).
2. **Xung đột với controller BUILT-IN (bo .161):** bật nguồn là built-in **tự lái + publish `rt/lowcmd`**. run_r1 auto-start nhưng **nằm im quan sát**, chỉ tiếp quản sau khi bàn giao đúng. Đang lái mà nghe built-in phát lại → **NGẮT NGAY**.
3. **Quy trình bàn giao 2 nút (đúng thứ tự):**
   - Bật nguồn → run_r1 chạy nhưng **nằm im**.
   - Giữ **L2+R2** *(1 lần — đây là TOGGLE của HÃNG, bấm lại là bật built-in dậy)* → built-in vào dev mode, robot **rũ mềm** = built-in đã buông.
   - Giữ **R1+R2 ~1s** → run_r1 **tiếp quản**, nói *"Máy tính phát triển đã sẵn sàng"*.

> 🔒 run_r1 **không bao giờ arm nếu chưa nghe được built-in** (chống arm mù khi DDS lỗi). Bấm R1+R2 khi built-in còn sống → **từ chối arm, đứng im**. Giá phải trả: nếu restart run_r1 lúc built-in đã tắt sẵn thì phải **cold-boot** mới arm lại được. (Config: `arm_require_button: true`, `arm_no_builtin_timeout_s: 0`.)

---

## 1. Cấu hình theo nhóm — 4 khóa quyết định cách chạy

`config/tuning.yaml` chỉ là entrypoint. Sửa đúng file nhóm rồi restart service:
`runtime.yaml` (DDS/input), `safety.yaml`, `locomotion.yaml` (bao gồm policy/model),
`postures.yaml`, `gestures.yaml`, `dance.yaml`, `audio.yaml`.

| Khóa | Ý nghĩa | Đặt cho Cách A (headless) |
|------|---------|---------------------------|
| `runtime.yaml: network_interface` | card mạng DDS | **⚠ Phải ĐÚNG** (Auto lấy từ đây). Kiểm `ip a` trên robot, thường `eth10`. |
| `runtime.yaml: dev_no_keyboard` | `true` = không mở bàn phím X11 | **`true`** (thuần R3-1, không dính lỗi đứt X11). |
| `audio.yaml: voice_enabled` | giọng nói thông báo | **`true`** — headless không màn hình, giọng nói là kênh phản hồi chính. |
| `safety.yaml: safe_stop_enabled` | mất hết input thì đứng yên | **`true`**. |

Các tham số khác (gains, tốc độ, dance, ngồi ghế…) chỉnh theo nhu cầu; sửa xong cần `make restart-all` (chỉ chạy khi robot DISARMED).

> ⚠ **Đổi `locomotion.yaml: flat_model` thì phải kiểm 3 thứ đi kèm — không có cái nào tự động:**
> 1. `gait_period_s` phải bằng `period` của phase obs lúc train đúng model đó (mjlab:
>    `velocity_env_cfg.py` term `phase`; deploy chuẩn: `.../velocity/v0/params/deploy.yaml`
>    → `observations.gait_phase.params.period`). Sai = robot bước sai nhịp, không gain nào bù được.
>    Chạy `run_r1 --preflight` để xem số đang chạy: dòng `[Locomotion] gait_period=...`.
> 2. Trần tốc độ (`slow_*`/`fast_*`) phải nằm trong dải lệnh model đó đã train.
> 3. Sửa tên model trong `tests/tuning_include_test.cpp` và `tests/config_reader_test.sh` —
>    hai chỗ đó ghim tên có chủ đích để bắt việc đổi model ngoài ý muốn.

---

## 2. Cài Auto (systemd) — làm 1 lần

> 🛑 **BẮT BUỘC:** phải chắc controller **built-in đã TẮT** trước khi bật Auto (không thì xung đột `rt/lowcmd`). Chưa xử lý xong thì **chưa `make install`** — chạy tay Cách B an toàn hơn.

```bash
# Trên robot, ở thư mục repo (~/HB):
make build          # build controller + voice + integration
make preflight      # kiểm model/config/asset, không xuất lệnh động cơ
make install        # cài + bật service (cần sudo)
make status
```

Từ máy dev thay cho các bước trên: `make deploy` (xem [deploy.md](deploy.md)).
Xong: dịch vụ **`hb_high_level`** đã cài + bật tự khởi động.

---

## 3. CÁCH A — Vận hành headless bằng R3-1

### 3.1 Bật
- **Bật nguồn robot** → tự chạy, sẵn sàng nhận R3-1 (hoặc `make start-high`).
- Robot nói **"Máy tính phát triển đã sẵn sàng"** khi kết nối xong.

### 3.2 Bảng điều khiển

Bảng đầy đủ (tay cầm R3 và bàn phím): [controls.md](controls.md).

**Lưu ý bản đồ nút:**
- **Động tác tay (double-click):** nút phải bấm **trần** (không giữ L2/R1/R2). Robot tự thu tay khi rời locomotion / nghiêng nhiều / ngã / **vừa bấm lệnh đổi trạng thái** (đọc tên điệu, chờ settle để khoá đứng/ngồi/nằm). Cú double-click bị bỏ qua sẽ được **in lý do ra log** chứ không im lặng. *(2026-09-04: trước đây player bị đóng băng ở các trạng thái không chạy policy — khoá đứng rồi quay lại đi bộ là tay bật lại tư thế gesture cũ tức thì; nay thu tay ở mọi trạng thái.)* `legacy_83` cho phép overlay tay; FlatPlus H4 chủ động tắt overlay vì offset 83D không hợp lệ trên history 332D. Gán slot trong `config/gestures.yaml`: `gesture_slot_3: vaytay`.
  - **Thu tay AN TOÀN có trần vận tốc** (`gesture_safety_max_vel_rad_s`, 4.0): `gesture_safety_retract_s` (0.4s) là một số cố định trong khi quãng đường thu chênh nhau ~10 lần giữa các gesture, nên clip vươn xa bị giật rất mạnh đúng lúc robot đang nghiêng. Nay gesture vươn xa được **kéo dài** thời gian thu để đỉnh vận tốc không vượt trần; gesture vươn gần vẫn thu đủ 0.4s (không bao giờ nhanh hơn). Đặt 0 để tắt. Tham chiếu: `make_gesture.py` cảnh báo clip nhanh hơn 6.0 rad/s.
  - **Clip LẶP phải khép vòng.** Player nối frame cuối → frame đầu trong đúng **một frame**, nên vòng hở bao nhiêu là tay giật bấy nhiêu, lặp lại mỗi vòng. `run_r1` cảnh báo lúc nạp nếu hở > 0.15 rad; vá bằng `tools/make_gesture.py --loop-bridge-s`.
  - Tốc độ phát và thời gian thu tay của `legacy_83` dùng `gesture_slot_speed_N`, per-slot retract và `gesture_safety_retract_s` (0.4s).
- **Teleop (START, Mode Z, tự nhả quyền):** xem [teleop.md](teleop.md). START chưa được kiểm tra chống trùng với controller hãng: nếu controller hãng còn chạy, nó cũng nhận START.
- ⚠ **Tránh combo của HÃNG:** **L2+R2** = vào dev mode; **L1+L2** = hãng đã dùng. Đừng map chức năng lên đó, và cẩn thận giữ L2 lỡ chạm R2 = đổi chế độ giữa lúc chạy.
- **E-stop:** L2+B chỉ xả lực (app vẫn chạy, đứng lại bằng L2+Lên); **ESC** xả lực **rồi thoát** app.
- **Zero-torque (L2+Y):** limp hoàn toàn, bẻ khớp được (khác Damping còn hơi cứng). Bật từ IDLE. Thoát: L2+Y (→Damping), L2+B (E-stop), hoặc L2+Lên (đứng dậy gồng cứng). ⚠ Đứng thẳng mà bật là NGÃ — chỉ dùng khi robot đã nằm/được đỡ.

### 3.3 Vì sao robot hay "khựng" ~1–2s (BÌNH THƯỜNG, đừng bấm lại)

**Nguyên tắc chung: không bao giờ tắt policy giữ thăng bằng khi robot đang tự đứng một mình giữa sàn.** Vì vậy mọi lúc chuyển trạng thái, robot **giữ policy chạy để tự đứng vững** rồi mới chuyển — tạo ra khoảng dừng ngắn, cố ý:

| Khi nào | Robot làm gì | Tham số |
|---------|--------------|---------|
| Bấm khóa đứng / ngồi lúc **đang đi/nhảy** | dừng lại ~1s dưới policy rồi mới làm | `settle_time_s` |
| Bấm **khóa đứng** | nói *"đã khóa đứng"* **trước**, chờ 1.5s (kịp đưa tay đỡ) rồi mới ép cứng | `stand_lock_warn_s` |
| Bấm **điệu nhảy** | đứng yên đọc tên điệu ~2s, xong vào **tư thế mở màn** (chân lệch 28–45°) rồi mới bật nhạc | `mimic_announce_delay_s`, `mimic_warmup_s` (1.5), `return_kp_ankle`, xem ↓ |
| Kẹt ở **RETURNING** (khớp lệch, không về được default) | quá `return_timeout_s` (6s) tự **huỷ dance → STAND LOCK**; bấm `L2+Lên` để huỷ ngay | `return_timeout_s`, `return_pos_tol` |
| **Ra khỏi điệu** (bấm `0` / hết điệu / ngồi) | policy nhảy **tự đứng thẳng dậy**, khi đủ yên mới giao cho đi bộ | `mimic_cooldown_s`, xem ↓ |

> ⚙ **Độ cứng cổ chân lúc VÀO điệu** — `return_kp_ankle` (mặc định **120**, ở `config/safety.yaml`). Đoạn robot đứng chờ trước khi vào bài là lúc **không có policy nào chạy**, cổ chân bị gồng cứng bằng PD thuần. Trước đây nó ăn chung `stand_kp_leg` = 200 nên vào bài thấy khựng. Núm này tách riêng: chỉ áp cho **đoạn chờ + đoạn soft-start**, còn khóa đứng / đứng dậy vẫn giữ 200.
> **ĐỪNG hạ về 40 (mức policy).** Không có policy đỡ thì robot là con lắc ngược quanh cổ chân, trọng lực tạo độ cứng âm 168 Nm/rad; hai cổ chân góp `2×kp` nên **dưới 84 là robot đổ** — app từ chối khởi động nếu đặt dưới 100. Mềm hơn nữa thì hạ `mimic_warmup_s` hoặc `return_rate_limit`, đừng hạ gain.

> ⚙ **Bàn giao ra-khỏi-điệu theo ĐIỀU KIỆN**: giao khi cooldown đã chạy hết (`mimic_cooldown_s`) **VÀ** robot đứng đủ yên (nghiêng `< mimic_handover_tilt` VÀ gyro `< mimic_handover_gyro`); nếu mãi không yên thì trần cứng `mimic_handover_max_s`. (Sửa được lurch ~33° gần ngã của vài bài như Pokemon.)
> **2026-09-04:** trước đây chỉ cần `mimic_handover_min_s` (0.3s) nên có thể giao lúc reference mới rời tư thế nhảy ~7% — `settled` chỉ đo nghiêng/gyro của **thân**, không đo tư thế **khớp**, nên huỷ bài lúc robot ngồi xổm vững vẫn đạt điều kiện và giao một con robot đang squat cho policy đi bộ. Nay bắt buộc chờ hết cooldown.

> 🛑 **Bấm `0` / `L2+Lên` khi ĐANG NHẢY = HUỶ ĐIỆU, không phải khóa cứng.** Robot tắt nhạc, tự đứng thẳng (~1s), **về đi bộ đứng yên tại chỗ**. Muốn gồng cứng thì **bấm `0` LẦN NỮA** (khi đã tới đứng cạnh robot) — lần 2 bị chặn 2s đầu (`dance_abort_lock_block_s`) phòng bấm nhầm. Nghe là biết: huỷ điệu → *"bật chế độ đi bộ"*; khóa cứng → *"đã khóa đứng"*.

> Thao tác đúng: **bấm phím trước, cầm/đỡ robot sau** — cầm trước rồi bấm thì policy thấy chân lơ lửng.

### 3.4 Trình tự chuẩn
Treo/đỡ robot → bật nguồn → **L2+Lên** (đứng dậy) → đặt xuống đất → **R2+A** (đi bộ) → **R1+D-pad** (nhảy, tự về đi bộ khi xong) → **L2+Trái** (ngồi) khi xong.

### 3.5 Ngồi ghế

> 🪑 **CHIỀU CAO GHẾ: 43 cm** (bằng tầm gối robot khi đứng). Ghế **cao hơn** → ngồi nông → **dễ ngã ngửa**. Thấp hơn vài cm vẫn được. Ghế khác phải sửa `sit_hip_deg` / `sit_knee_deg`.

Giữ **L2+Trái**, robot ngồi **4 pha**. **L2+X không phải ngồi**; nút đó dành cho nằm xuống / đứng dậy:

| Pha | Robot làm gì | Thời gian |
|-----|--------------|-----------|
| 1. Chỉnh chân đế | đứng thẳng, **mở rộng 2 chân** | `sit_gather_time_s` (1.5s) |
| 2. Hạ người | gập hông+gối, **đổ thân về trước + vươn 2 tay** | `sit_descent_time_s` (4s) |
| 3. Ngồi hẳn | mông chạm ghế → **thu tay về**, dựng thân | `sit_settle_time_s` (1.5s) |
| 4. Giữ | giữ tư thế ngồi | vô hạn |

> 🪑 **Đổ thân + vươn tay ở pha 2 là BẮT BUỘC:** gập gối làm hông lùi ra sau, thân thẳng thì trọng tâm rơi ngoài gót → **ngã ngửa**. Đổ thân + vươn tay kéo trọng tâm về giữa bàn chân (giống người ngồi ghế).

Bàn chân bám mặt đất theo **IMU** (không vênh khi thân đổ); chỉnh `sit_ankle_gravity_gain` (0 tắt · 0.4 mặc định · 1 ép phẳng). Tham số: `sit_hip_deg`/`sit_knee_deg` (độ sâu) · `sit_lean_deg`/`sit_arm_forward` (cân bằng khi hạ) · `sit_seated_lean_deg` (dáng cuối) · `sit_spread` (rộng chân đế).

> ⚠ **`sit_rest_spread: 0.45` luôn ép cổ chân roll chạm trần.** Pha 4 giữ **vô hạn**, nên đó là giá trị cổ chân bị giữ liên tục với Kp=200. Trần này (2026-09-08) hạ từ `0.44` xuống **`0.37` rad** = giá trị lớn nhất từng ghi được trên robot thật (74.442 frame encoder trong `motions/captures*`); `0.44` nằm ngoài mọi quan sát. Lòng bàn chân **không** phẳng hoàn toàn ở `sit_rest_spread` lớn — điều này đúng cả trước lẫn sau khi đổi, vì cổ chân không với tới `0.45`. Muốn bàn chân phẳng thì **giảm `sit_rest_spread`**, đừng nới trần.

### 3.6 Đứng dậy sau ngồi / sau safe-stop
- Sau **ngồi ghế**: bấm **L2+Lên** để đứng lại, hoặc **L2+B** xả lực.
- Sau **safe-stop** (mất rồi có lại tín hiệu): robot đứng yên → cầm R3-1 điều khiển tiếp.

### 3.7 Đứng lên / Nằm xuống (L2+X)

Khác ngồi ghế: đứng/nằm **phát lại nguyên văn** quỹ đạo đã ghi từ built-in — **PD thuần, KHÔNG có policy cân bằng**. Vì vậy **lần chạy thật đầu tiên phải có người đứng đỡ sẵn**.

```
LOCOMOTION ──[L2+X: nằm]──► (dừng ~1s dưới policy) ──► phát liedown.npz
           ──► DAMPING (rũ mềm, nằm nghỉ trên sàn)

DAMPING(nằm) ──[L2+Lên: chuẩn bị]──► gồng cứng về tư thế nằm-chuẩn, GIỮ (chờ)
             ──[L2+X: đứng dậy]──► phát getup.npz ──► tự sang LOCOMOTION
```

- **Nằm** (`L2+X`): dừng (settle ~1s nếu đang đi/nhảy) → `liedown.npz` → tự rũ mềm nằm sàn.
- **Đứng = 2 bước:** (1) **L2+Lên** đưa khớp về **tư thế nằm-chuẩn** rồi GIỮ; (2) **L2+X** phát `getup.npz`, xong **tự sang LOCOMOTION**.
- Bấm L2+X thẳng khi đang nằm → robot nhắc "bấm L2+Lên trước".
- Chưa ghi file (`motions/getup.npz` / `liedown.npz`) → robot **không làm gì** (an toàn mặc định).

**Ghi quỹ đạo** (1 lần, trên robot thật):

```bash
cd ~/HB/controller/build
./record_motion eth0 ../motions/raw     # đổi eth0 thành network_interface
```

Tool **chỉ nghe DDS, không gửi gì** (an toàn chạy song song built-in). Cho robot nằm rồi đứng bằng built-in; tool tự cắt đoạn + in nhãn phím/thời điểm. Đối chiếu để copy đúng đoạn:

```bash
cp ../motions/raw/capture_001_xxx.npz ../motions/getup.npz
cp ../motions/raw/capture_002_xxx.npz ../motions/liedown.npz
```

Đổi `.npz` chỉ cần **khởi động lại run_r1**, không cần build lại. Tham số phát lại: `getup_*` / `liedown_*` trong `config/postures.yaml`.

### 3.8 Log / dừng
```bash
journalctl -u hb_high_level -f     # log realtime
make stop-high                     # dừng (từ chối nếu chưa DISARMED)
make restart-all                   # nạp lại sau khi sửa config (cần DISARMED)
```

---

## 4. CÁCH B — Chạy tay có bàn phím (dev/test)

Chỉ dùng khi có màn hình (dev PC / NoMachine / `ssh -Y`).

```bash
make stop-high                          # 1. TẮT Auto trước (bắt buộc, cần DISARMED)
# 2. Đổi dev_no_keyboard: false trong config/runtime.yaml
cd ~/HB/controller/build && ./run_r1     # 3. chạy tay (cần $DISPLAY)
```

Bàn phím **chỉ mở khi CẢ HAI**: `dev_no_keyboard: false` **và** có `$DISPLAY`. Cửa sổ "R1 ROBOT CONTROL" hiện ra — **phải click vào** mới nhận phím. Bảng phím: cột "Bàn phím" trong [controls.md](controls.md).

Xong test: thoát (ESC) → `make start-high`. Kiểm không chạy trùng: `pgrep -a run_r1` — chỉ nên thấy **1** dòng.

---

## 5. Chức năng AN TOÀN (tự động)

| Sự cố | Robot làm gì | Vì sao |
|-------|--------------|--------|
| **Ngã** khi **đi bộ / nhảy / vào tư thế mở màn** | **Damping** (xả lực) | mềm ra hấp thụ va đập. Ngưỡng `fall_*`: nghiêng thuần `> fall_tilt_deg`, hoặc "lật nhanh" `> fall_flip_tilt_deg` **kèm** gyro `> fall_flip_gyro` (hạ 4.0 để bắt cú lurch lúc bàn giao). |
| **Vung khớp loạn** — `max\|dq\|` (24 khớp) `> joint_speed_limit` quá `joint_speed_debounce_ms` | **Damping** | lớp an toàn ngoài độ nghiêng. Tắt: `joint_speed_guard_enabled: false`. |
| **Ngã** khi **khóa đứng / ngồi ghế** | ❗**KHÔNG** damp (giữ cứng) | Cố ý: 2 trạng thái này luôn có **người đứng đỡ**. |
| **Mất DDS lowstate** > 1s | **Damping** | mất cảm biến → policy "mù". |
| **LowState hỏng** — encoder/IMU trả NaN/Inf, hoặc quaternion IMU sai chuẩn (norm ngoài [0.5, 2.0], gồm cả kênh chết trả toàn 0) | **Damping** | gói hỏng KHÔNG được chép vào state (giữ mẫu hợp lệ cuối). Quaternion toàn 0 từng cho "đứng thẳng hoàn hảo" và làm bộ phát hiện ngã mù. |
| **Lệnh motor không hữu hạn** — target/gain/rate-limit là NaN/Inf | **Damping** (sau khi frame đó đã được làm sạch) | cổng an toàn cuối trong `LowCmdSender`: giữ lệnh cũ cho khớp hỏng, gain hỏng → 0, rate hỏng → bước 0. Log `[LowCmdSender]`. |
| **Policy trả output NaN/Inf** | **Damping**, giữ nguyên target frame trước | bắt tại nguồn nên log nói rõ TÊN policy (`[FlatPlus] output ONNX khong huu han...`), và action rác không nhiễm history buffer. |
| **LowState cũ + bất kỳ đường nào còn đòi cấp PD** | **mọi Kp/Kd = 0** ở `LowCmdSender` | backstop cuối, độc lập với watchdog trạng thái: Mode Z tự gate riêng đường PD của nó, nếu gate đó hỏng thì lớp này vẫn chặn. |
| **Lệnh motor vượt cổng an toàn** (rộng hơn tầm khớp ±0.5 rad) | **Clamp, KHÔNG damp** | đếm + log mỗi 1s: `target khop N ... -> clamp`. Cố ý không damp: dừng giữa lúc robot đang dồn trọng lượng lên một chân còn nguy hơn giá trị bị cắt. |
| **DDS im nhưng process còn sống** | tay cầm chuyển SUSPECT → LOST sau `remote_timeout_ms`; **không ARM được** | gói LowState cũ vẫn chứa byte tay cầm khác 0; trước đây điều đó giữ remote "healthy" vĩnh viễn và R1+R2 đông cứng vẫn arm được. |
| **Mất TOÀN BỘ input** (R3-1 + bàn phím), DDS còn | <3s: ngừng nhận lệnh; >3s + debounce: ép vận tốc 0 + cảnh báo 1 lần | còn cảm biến → policy tự cân bằng, KHÔNG damp. Reconnect trung tính 200ms. |
| **Đứt SSH/wifi** (Auto headless) | **Không ảnh hưởng** | Auto tách khỏi phiên SSH. |
| **Đứt X11/SSH** (đang chạy Cách B) | **Damping + thoát** | thư viện X ép thoát → damp trước. |
| **L2+B / ESC** | **Damping** (E-stop thủ công) | luôn thắng mọi thứ. |

**Config sai = KHÔNG khởi động.** Parser giờ fail-closed: khoá lạ (gõ sai tên), dòng thiếu `:`, số có ký tự thừa (`50deg`, `3.0.0`) và **boolean gõ nhầm** (`ture`, `Fasle`, `enable`) đều làm `run_r1` từ chối chạy thay vì lặng lẽ bỏ qua. Trước đây `fall_enabled: ture` = tắt bộ phát hiện ngã mà preflight vẫn báo xanh. Boolean hợp lệ: `true/false`, `1/0`, `yes/no`, `on/off` (không phân biệt hoa thường).

**Chặn ngay lúc khởi động (không phải lúc chạy):** gains và rate-limit của mọi state không chạy policy giờ có dải bắt buộc trong `Validate()` — `stand/sit/getup_kp_*` ∈ [10, 400], `*_kd` ∈ [0.5, 20], `*_rate_limit` ∈ [0.05, 50], `policy_kp_scale`/`policy_kd_scale` ∈ [0.5, 2.0], gain đầu ≤ 60. Ngưỡng của các lớp an toàn cũng vậy: `fall_tilt_deg` ∈ [10, 89] và phải **rộng hơn** `fall_flip_tilt_deg`, `joint_speed_limit` ∈ [5, 100], `state_timeout_ms` ∈ [20, 5000], `arm_hold_s` ∈ [0.5, 30], `battery_critical_pct` phải **nhỏ hơn** `battery_warn_pct`. Sai dải là `run_r1` **từ chối chạy** kèm dòng `[Tuning] Invalid configuration:`, thay vì đẩy thẳng số đó vào motor.

> Vì sao "mất DDS → damp" nhưng "mất tay cầm → đứng yên"? **Mất DDS = mất cảm biến** (policy không chạy được → damp); **mất tay cầm = chỉ mất lệnh** (cảm biến còn → policy vẫn tự đứng vững).

---

## 6. Xử lý sự cố

| Triệu chứng | Nguyên nhân | Cách xử lý |
|-------------|-------------|------------|
| Robot giật/loạn, chập chờn xả lực | **2 bản run_r1** cùng chạy | `pgrep -a run_r1`; tắt bớt (`systemctl stop` + kill bản tay) |
| "Chờ kết nối DDS…" mãi | sai `network_interface` | kiểm `ip a`, sửa `config/runtime.yaml`, `make restart-all` |
| Không nghe robot nói | `voice_enabled: false` hoặc TTS kém | bật `voice_enabled: true`; TTS tệ → thay `voice_*` bằng file .mp3 thu sẵn |
| R3-1 không điều khiển được | tay cầm chưa kết nối / hết pin | kiểm remote; `journalctl -u hb_high_level -f` |
| Sửa tuning.yaml không đổi | Auto đã nạp config lúc bật | `make restart-all` (cần DISARMED) |
| Muốn tắt hẳn auto-boot | — | `sudo systemctl disable hb_high_level` (bật lại: `enable`) |

---

## 7. Lệnh (cheat-sheet)

```bash
make status                              # trạng thái mọi service + health check
make start-high | stop-high              # stop từ chối nếu chưa DISARMED
make restart-all                         # cần DISARMED
sudo systemctl enable|disable hb_high_level   # bật/tắt tự khởi động lúc boot
journalctl -u hb_high_level -f           # log realtime
pgrep -a run_r1                          # số bản đang chạy (phải = 1)
```
