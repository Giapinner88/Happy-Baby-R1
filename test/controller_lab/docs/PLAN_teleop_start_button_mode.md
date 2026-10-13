# PLAN — Giữ nút START trên R3-1 để vào chế độ TELEOP (chỉ tay + đầu)

> Yêu cầu 2026-08-18: **giữ nút START** trên tay cầm R3-1 → chuyển sang teleop.
> **Teleop chỉ điều khiển tay và đầu.**
> Tiền đề quy ước đầu: `PLAN_teleop_head_convention.md` (chuẩn = `controller`).
>
> **TRẠNG THÁI 2026-08-18: ĐÃ TRIỂN KHAI** (mục 3 + 5 xong, build sạch, 10/10 ctest pass).
> Còn lại: **mục 4** (test va chạm nút START với controller built-in — cần robot) và
> **mục 6** (nghiệm thu bằng `teleop_send_test.py`). Chưa test thì đừng tin nút.

---

## 1. Sự thật hiện tại (đã đọc code, không suy đoán)

| Hạng mục | Sự thật | Vị trí |
|---|---|---|
| Bit `start` | **Chưa dùng ở đâu trong CẢ stack**: `controller` không đọc, `integration` chỉ khai báo trong struct, `voice` không đọc remote (nó đọc `voice_status.env`) | `src/input/GamepadR3.hpp:19`, `integration/src/main.cpp:55` |
| Bit `select` | ⛔ **ĐÃ DÙNG cho voice**: giữ = PTT nói; double-tap (2 cú <0.4 s trong 0.8 s) = ngắt lượt voice | `integration/src/main.cpp:203`, `:219-245` |
| `F1` / `F2` | Đã dùng cho voice: double-click F1 = chế độ rảnh tay, F2 = đổi nguồn mic | `integration/src/main.cpp:204-205` |
| Cửa vào teleop hiện tại | Giữ **L2+Phải** đủ `teleop_toggle_hold_s` (3.0 s) → `want_teleop_toggle` | `GamepadR3.hpp:195-198`, `Application.cpp:784` |
| Điều kiện teleop có hiệu lực | `teleop_enabled` **&&** `teleop_runtime_on_` **&&** `state_ == kLocomotion` **&&** `!guard_bad` **&&** có gói UDP tươi | `Application.cpp:1766` |
| Phạm vi ghi của teleop | **CHỈ** `target_q[14..23]` (tay 5+5) + head idl 29/30. Chân 0..11 và waist 12..13 do policy, teleop không chạm | `TeleopReceiver::BlendInto`, `Application.cpp:1938`, `LowCmdSender.hpp:83-84` |
| Cổng cấu hình | `teleop_enabled: false` | `config/gestures.yaml:83` |
| Nút "trần" đang dùng | Double-click Up/Down/Left/Right/A/B/X/Y (không modifier) = gesture slot 1..8. START **không** nằm trong danh sách này và **không** được tính là modifier | `GamepadR3.hpp:203-219` |

Kết luận: **yêu cầu "teleop chỉ tay + đầu" đã đúng bằng thiết kế hiện tại** — không phải làm gì thêm để giới hạn phạm vi. Việc cần làm là (a) đổi cửa vào sang giữ START, (b) làm cho "chế độ" này rõ ràng và an toàn.

---

## 2. Thiết kế

### 2.1 Cửa vào
Giữ **START** (không kèm modifier) ≥ `teleop_toggle_hold_s` → toggle `teleop_runtime_on_`.
Giữ lần nữa ≥ ngần đó → thoát. Dùng lại đúng lambda `heldFor` đang có, nên hành vi
"phải giữ có chủ ý" giống L2+Trái / L2+X.

**Bỏ L2+Phải** khỏi vai trò này (một cửa vào duy nhất, tránh 2 đường vào cùng một quyền
nguy hiểm). Quyền overlay vẫn phải được controller cho phép rõ qua capability flag.

### 2.2 "Chuyển sang teleop" nghĩa là gì — quyết định
Teleop là **overlay trên policy đi bộ**, không phải một `AppState` mới: chân vẫn phải do
policy giữ thăng bằng, nên vào teleop **chỉ từ `kLocomotion`**. Giữ START ở `kIdle` /
`kStandLock` / mimic → **từ chối + nói lý do**, không im lặng.

Thêm núm mới **`teleop_lock_base` (mặc định `true`)**: khi teleop đang ARMED thì
`cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0` bất kể cần analog → chân chỉ đứng giữ thăng bằng,
đúng nghĩa "teleop chỉ điều khiển tay và đầu". Đặt `false` nếu sau này muốn vừa đi vừa teleop.

### 2.3 Phản hồi cho người vận hành
`voice_teleop_on` / `voice_teleop_off` đã có sẵn. Bổ sung:
- Khi ARMED mà **không có gói UDP nào** trong 2 s → log + nói 1 lần: teleop đang bật nhưng
  chưa có nguồn. Không có cái này thì "bấm mà không thấy gì" bị hiểu là hỏng nút.
- Khi bị auto-nhả (mục 2.4) → nói lý do, không nhả im lặng.

### 2.4 Đường tự nhả (đã có sẵn, chỉ liệt kê để không phá)
| Nguyên nhân | Cơ chế |
|---|---|
| Mất gói > `teleop_timeout_ms` (300 ms) | weight ramp 0 theo `teleop_retract_s` → tay về policy |
| `enable=0` từ bộ phát | như trên |
| Nghiêng / gyro vượt `gesture_guard_tilt|gyro` | `guard_bad` → `use_teleop=false` |
| Rời `kLocomotion` (ngồi, nằm, mimic, E-stop, fall) | `loco=false` → `use_teleop=false` |
| Remote LOST | `ResetHeldActions()` + `is_active=false` |

Thêm: khi rời `kLocomotion` thì **đặt `teleop_runtime_on_ = false`** (hiện tại cờ vẫn giữ
true và teleop tự bật lại khi quay về locomotion — với nút mới thì đó là hành vi bất ngờ,
người vận hành đã rời chế độ rồi).

---

## 3. Thay đổi từng file

### 3.1 `src/input/GamepadR3.hpp`
- Thêm state: `TimePoint start_hold_start_{}; bool start_hold_active_ = false, start_hold_fired_ = false;`
- Trong `Parse`, thay khối `heldFor(cur.L2 && cur.right, …)`:
```cpp
// Giữ START (không modifier): vào/ra chế độ teleop thân trên. Nút trần nên
// chặn khi có modifier để không đụng combo tương lai.
const bool no_mod_start = !cur.L1 && !cur.L2 && !cur.R1 && !cur.R2;
if (heldFor(no_mod_start && cur.start, start_hold_start_, start_hold_active_,
            start_hold_fired_, teleop_hold_s_))
    cmd.want_teleop_toggle = true;
```
- `ResetHeldActions()`: reset 3 biến mới (mất remote giữa lúc đang giữ nút không được
  fire khi remote về).
- Xoá `l2right_*` nếu không còn dùng.

### 3.2 `src/input/InputCommand.hpp`
Sửa comment `want_teleop_toggle`: "giữ START ≥ `teleop_toggle_hold_s`".

### 3.3 `src/app/Application.cpp`
- `:784-796` khối `want_teleop_toggle`: đổi chữ "L2+Phải" → "START"; thêm nhánh từ chối
  khi `state_ != kLocomotion`.
- `:1741-1745` (nơi gán `cmd_vx_/vy_/yaw_`): nếu `teleop_runtime_on_ && tuning_.teleop_lock_base`
  → gán 0.
- `:1821` head target của teleop: thêm rate-limit riêng (mục 5).
- Chỗ chuyển state ra khỏi `kLocomotion`: `teleop_runtime_on_ = false`.

### 3.4 `src/config/Tuning.hpp` + `Tuning.cpp`
Thêm `bool teleop_lock_base = true;` + parse `teleop_lock_base` + (tuỳ) `teleop_head_rate_limit_rad_s`
kèm `require(...)` như các núm khác.

### 3.5 `config/gestures.yaml`
`teleop_enabled: true`, thêm `teleop_lock_base: true` + `teleop_head_rate_limit_rad_s`.

### 3.6 `docs/huong_dan_van_hanh.md`
**Bắt buộc cùng lượt** (nguyên tắc giữ docs đồng bộ): thêm dòng bảng phím
"Giữ START 3 s = vào/ra teleop (chỉ tay + đầu, phải đang LOCOMOTION)", và xoá L2+Phải nếu có.

### 3.7 KHÔNG đụng
`TeleopReceiver.hpp` (twin với `unitree_mujoco/simulate_cpp/src/core/TeleopReceiver.hpp`) —
thay đổi lần này nằm hết ở tầng nút/state nên **không phải sync twin**. Sim vẫn arm bằng bàn phím.

---

## 4. Rủi ro trước tiên: START có thể là phím của controller built-in

Đây là loại va chạm **bản đồ phím vật lý**, giống L2+Y đã gặp: `controller` chỉ **đọc thụ động**
`wireless_remote` trong `rt/lowstate` nên không có xung đột DDS, nhưng nếu built-in đang chạy
(chưa vào dev mode) thì **nó cũng nhận nút START** và có thể tự làm việc của nó.

**Phải test trước khi tin nút:** built-in đang chạy, `hb_high_level` **dừng** → giữ START 3–5 s →
quan sát robot có phản ứng gì không. Có phản ứng ⇒ hoặc luôn vào dev mode trước khi teleop,
hoặc đổi sang **SELECT**, hoặc dùng **START + một nút trần** làm combo. Ghi kết quả vào docs.

**Phương án dự phòng nếu START trùng built-in — KHÔNG dùng SELECT.** SELECT đang là PTT
của voice (giữ để nói) *và* double-tap để ngắt lượt; chiếm nó là mất đường nói. F1/F2 cũng
đã thuộc voice. Trong `controller` thì **L1 chưa dùng cho bất kỳ chức năng nào** (hiện chỉ
có combo L2+/R1+/R2+), nên dự phòng là **giữ L1+START** — hai nút càng khó trùng built-in hơn
nút đơn. Dự phòng hai: `R1+Y` hoặc `R2+X` (đều chưa dùng).

Lưu ý phân chia quyền đọc remote: `integration` (cổng voice) và `controller` **cùng đọc
thụ động** một gói `wireless_remote`, không ai ghi. Thêm START cho teleop không đụng gì tới
voice, và trong lúc teleop ARMED người vận hành vẫn giữ SELECT để nói bình thường.

---

## 5. Thiếu sót phát hiện khi lên plan (nên sửa cùng lượt)

**Đầu của teleop không có rate-limit riêng.** `Application.cpp:1821` tạo `HeadTarget` với
`max_rate_rad_s = 0` → theo `LowCmdSender.hpp:81` là kế thừa `policy_rate_limit`, trong khi
gesture dùng trần thấp hơn (`gesture_head_rate_limit_rad_s`, `Application.cpp:1848`). Nghĩa là
**một gói UDP xấu quật đầu nhanh hơn mức gesture được phép** — mà teleop là nguồn từ ngoài,
đáng ra phải bị siết chặt hơn gesture, không lỏng hơn. Đề xuất thêm
`teleop_head_rate_limit_rad_s` (khởi điểm bằng giá trị của gesture).

---

## 6. Nghiệm thu

Không cần chờ B1/B2: **`controller/scripts/teleop_send_test.py` đủ làm nguồn nghiệm thu nút.**

1. **Sim** (`unitree_mujoco/simulate_cpp`): xác nhận đường UDP + overlay còn nguyên sau khi sửa.
2. **Bàn (robot treo giá, `teleop_enabled: true`)**:
   - `kIdle` → giữ START 3 s → phải **từ chối + nói lý do**.
   - Vào LOCOMOTION → giữ START 3 s → nghe `voice_teleop_on`; chưa phát UDP → sau 2 s có cảnh báo "chưa có nguồn".
   - Chạy `teleop_send_test.py` ngay trên PC2/robot (receiver chỉ bind loopback) → tay bám; đẩy cần analog → **chân không đi** (`teleop_lock_base`).
   - `--release` (enable=0) → tay về policy trong `teleop_retract_s`.
   - Ctrl-C bộ phát (mất gói) → tay về policy trong ~300 ms + `retract`.
   - Giữ START lần 2 → nghe `voice_teleop_off`.
   - `teleop_send_test.py --hold-arms --head-yaw 0.3` → **đầu quay**; `--head-pitch 0.3` → **đầu gật**
     (chốt luôn mục tiêu của `PLAN_teleop_head_convention.md`).
3. **Đứng trên sàn**: chỉ sau khi 2 xong, có người giữ E-stop, biên độ tay nhỏ trước.

---

## 7. Thứ tự việc

| # | Việc | Phụ thuộc |
|---|---|---|
| 1 | Test va chạm nút START với built-in (mục 4) | Cần robot — **làm trước khi viết code**, kết quả quyết định chọn START hay SELECT |
| 2 | Nút + mode + lock_base + rate-limit đầu + docs (mục 3, 5) | Xong bước 1 |
| 3 | Nghiệm thu bằng `teleop_send_test.py` (mục 6) | Bước 2 |
| 4 | Đồng bộ nhãn đầu trong `teleop/hardware/` | `PLAN_teleop_head_convention.md`; chốt bằng 6.2 |
| 5 | B1 sender runtime + B2 nguồn pose | Sau cùng — **đây mới là thứ làm teleop có ích** |

Nói rõ để không kỳ vọng sai: xong bước 2 thì **nút hoạt động nhưng chưa có ai gửi lệnh** ngoài
`teleop_send_test.py`. Teleop dùng được thật cần bước 5.

## 8. Rollback

Toàn bộ thay đổi nằm trong `controller/src/input/`, `src/app/`, `src/config/`, `config/gestures.yaml`,
`docs/`. `git checkout` là xong; không có state trên robot. Đặt lại `teleop_enabled: false`
là vô hiệu hoá hoàn toàn kể cả khi code đã vào (socket không mở).
