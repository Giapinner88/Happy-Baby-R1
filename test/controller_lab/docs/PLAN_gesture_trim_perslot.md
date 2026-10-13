# Bù thăng bằng RIÊNG TỪNG GESTURE (trim per-slot)

> Triệu chứng: một số động tác tay làm robot mất thăng bằng theo kiểu riêng của
> nó. Rõ nhất là `duaqua` (slot 4): tay đưa ra trước → CoM dịch tới → policy
> bước tới → **robot trôi về phía trước**. Mỗi động tác lệch một kiểu và một
> lượng khác nhau, nên một con số chung không xử lý được.
>
> Quan hệ với [PLAN_gesture_thang_bang.md](PLAN_gesture_thang_bang.md): plan đó
> chẩn đoán đúng và dựng khung 4 mức, nhưng chỉ có gain **toàn cục**. Đây là
> phần mở rộng "Mức 1.5" — giữ nguyên cơ chế, thêm chiều per-slot — kèm một
> đính chính về nơi đo (mục 5).

---

## 1. Cơ chế đã có sẵn, đang tắt

Không cần dựng mới. Đường bù đã nối đủ, chỉ đang để 0:

| Thành phần | Vị trí | Vai trò |
|---|---|---|
| `LeanPitchProxy()` | `src/motion/ArmGesturePlayer.hpp:225` | `(vai_pitch_trái − default) + (vai_pitch_phải − default)`; tự ramp theo weight blend |
| Nhồi bias | `src/app/Application.cpp:1801-1802` | `gravity_x_bias = kg · lean`, `cmd_vx_bias = kv · lean` |
| Áp vào obs | `src/policy/LocomotionController.hpp:44-45` | `obs[3] += gravity_x_bias`, `obs[6] -= cmd_vx_bias` |
| Gain | `config/gestures.yaml:50-51` | `gesture_balance_kg: 0.0`, `gesture_balance_kv: 0.0` → **TẮT** |

Bất biến phải giữ: hai bias chỉ chạm **bản sao obs**, không chạm `cmd_vx_` thật.
Nhờ vậy `cmd_norm` và cổng gait không bị ảnh hưởng. Mọi thiết kế dưới đây phải
giữ nguyên tính chất này.

---

## 2. Chọn cơ chế: bias vào obs, KHÔNG dùng trim quaternion

Có ba cách tạo ra "trim" cho gesture. Chúng không tương đương về rủi ro:

| | Cách | Chạm tới | Rủi ro |
|---|---|---|---|
| A | `estimator_.SetPitchTrimDeg()` mỗi lần bật gesture (giống dance) | quat thật → `projected_gravity` **+ FallDetector** | Dời mốc nghiêng của bộ phát hiện ngã ngay giữa lúc đang đi. Phải khôi phục khi gesture kết thúc — mà gesture còn bị guard tilt/gyro cắt giữa chừng, quên khôi phục là trim kẹt lại vĩnh viễn |
| **B** | `gravity_x_bias` → `obs[3]` | **chỉ obs locomotion** | Không đụng FallDetector. Hết gesture là weight về 0, bias tự tắt |
| **C** | `cmd_vx_bias` → `obs[6]` | **chỉ obs locomotion** | Như B |

**Chọn B + C, loại A.** Dù người dùng quen gọi là "trim IMU", cách A nguy hiểm
hơn hẳn vì nó kéo theo FallDetector — đúng thứ không được phép lung lay khi
robot đang đi và đang giơ tay.

Phân vai giữa B và C:

- **C (`cmd_vx`) là knob chống TRÔI** — đúng triệu chứng "robot tiến về phía
  trước". Nó bảo policy nhắm tới vận tốc thấp hơn một chút.
- **B (`gravity.x`) là knob chỉnh TƯ THẾ** — làm policy tin là thân đang nghiêng
  khác thực tế, nên nó đổi cách đứng.

Với `duaqua`, bắt đầu bằng **C**. Chỉ đụng tới B nếu sau khi hết trôi mà thân
vẫn chúi.

---

## 3. Dạng tham số: hai phương án

### P1 — Per-slot ghi đè `kg`/`kv`

```
gesture_slot_balance_kv_4: 0.35     # hệ số, nhân với LeanPitchProxy
```

- Ưu: ít code nhất; giữ nguyên tính tỉ lệ theo độ vươn tay, nên tự ramp mượt
  theo blend-in/retract, không có bậc nhảy.
- Nhược: đơn vị là hệ số trên rad-vai, không hình dung được. Muốn biết nó tương
  đương bao nhiêu độ phải nhân nhẩm.

### P2 — Per-slot ghi thẳng bằng đơn vị vật lý  ← **khuyến nghị**

```
gesture_slot_trim_deg_4: -1.5       # độ, đi vào gravity.x
gesture_slot_vx_bias_4: 0.08        # m/s, đi vào cmd_vx
```

Quy đổi: `gravity_x_bias = sin(trim_deg) · Weight()`, `cmd_vx_bias = vx_bias · Weight()`.

- Ưu: đúng đơn vị người vận hành nghĩ; đọc bảng tuning là hiểu ngay động tác nào
  nặng hơn động tác nào.
- **Bắt buộc nhân `Weight()`**. Áp hằng số ngay khi gesture bật thì obs nhảy bậc
  đúng lúc tay đang giơ — policy giật, phản tác dụng. Nhân weight thì bias lên
  xuống cùng nhịp blend-in và retract, y như `LeanPitchProxy` đang làm.

Quy ước kế thừa: **0 = dùng giá trị toàn cục**, giống hệt nếp của
`gesture_slot_blend_N` / `gesture_slot_retract_N` đang có.

---

## 4. Rào an toàn bắt buộc

1. **Clamp cứng trong code**, không tin yaml: `|trim_deg| ≤ 6`, `|vx_bias| ≤ 0.3`.
   Gõ nhầm một chữ số không được phép bơm bias khổng lồ vào obs.
2. **weight = 0 ⇒ bias = 0 tuyệt đối.** Giữ bất biến "không gesture thì
   locomotion chạy y hệt như chưa từng có tính năng này".
3. **Guard cắt gesture ⇒ bias tắt theo.** Nhân weight là tự có, không cần code riêng.
4. **Không bao giờ đưa vào FallDetector.**
5. **Chỉ có tác dụng với `flat_policy_contract: legacy_83`.** V10 dùng arm
   reference nên bỏ qua hai bias này. Nếu yaml khai trim mà contract là v10 thì
   phải **log cảnh báo lúc khởi động**, đừng im lặng bỏ qua.

---

## 5. ĐÍNH CHÍNH: không tune được trong sim nữa

Plan cũ ghi "đo gain trong MuJoCo". **Bỏ bước đó.** Hai lý do, cái sau mới phát
hiện:

1. `simulate_cpp` **không còn là twin của deploy**: nó map eo theo quy ước G1
   (`R1Config.hpp:12` đổi chỗ 12↔13), trong khi R1 thật là `WaistRoll=12,
   WaistYaw=13` (đã xác nhận bằng enum chính hãng trên robot). Bridge MuJoCo và
   `run_r1_flat.py` cũng swap giống vậy. Gain đo trên nền eo sai không đem sang
   robot được.
2. Ngay cả trước khi biết điều đó, sim đã không cô lập được hiệu ứng: thiếu
   heading-hold nên robot lang thang thành cung, còn hiệu ứng pitch của gesture
   chỉ cỡ 0.05 rad — nhỏ hơn nhiễu nền.

⇒ **Tune trực tiếp trên robot thật**, theo quy trình mục 6.

---

## 6. Quy trình đo trên robot (bắt buộc theo thứ tự)

Chuẩn bị: sàn phẳng cùng loại sàn sự kiện, dán vạch mốc, có người đỡ.

### 6.1 Xác định DẤU trước, đừng đoán

Sai dấu làm trôi **gấp đôi** chứ không phải giảm. Chạy 3 cấu hình, mỗi cấu hình
lặp gesture 5 lần, đo dịch chuyển thân so với vạch mốc:

| Lần đo | `vx_bias` | Kỳ vọng |
|---|---|---|
| 1 | `0.0` | mốc chuẩn — ghi lại độ trôi gốc, ví dụ +40 cm |
| 2 | `+0.10` | nếu trôi GIẢM → dấu đúng |
| 3 | `−0.10` | nếu trôi TĂNG → xác nhận dấu ở lần 2 |

Chỉ khi lần 2 và lần 3 lệch nhau đúng chiều mới đi tiếp. Hai lần cùng chiều =
đang đo nhiễu, phải tăng số lần lặp.

### 6.2 Dò biên độ

Nội suy tuyến tính từ hai điểm trên để ước lượng giá trị triệt tiêu, rồi kiểm
lại bằng 5 lần chạy nữa. Nhận khi độ trôi còn dưới ~10 cm sau 5 lần.

### 6.3 Điều kiện đo phải cố định

Cùng lệnh vận tốc, cùng tốc độ phát gesture, cùng sàn, cùng người đỡ. Đổi
`gesture_slot_speed_N` là phải đo lại — động tác chậm hơn thì CoM dịch từ từ,
lượng bù cần cũng khác.

---

## 7. Thứ tự làm

1. **`duaqua` (slot 4)** — triệu chứng rõ nhất, dùng làm bài chuẩn hoá quy trình.
2. **`traitim` (slot 6)** — tay giơ cao; `LeanPitchProxy` đọc thiếu (mục 8), nên
   nhiều khả năng phải dùng P2 với số tự đặt thay vì tỉ lệ.
3. **`battay` / `vaytay`** — động tác một tay, lệch **ngang**, không phải trước
   sau. Hai knob hiện có không bù được (mục 8).
4. Ghi bảng kết quả vào `docs/huong_dan_van_hanh.md` cùng lượt với việc thêm
   khoá — theo nếp giữ tài liệu vận hành khớp với mapping/tuning.

---

## 8. Giới hạn đã biết, nói trước cho khỏi kỳ vọng sai

- **Proxy chỉ đọc vai-pitch.** Động tác đưa tay ra trước bằng **duỗi khuỷu**,
  hoặc giơ tay **lên cao**, đều dịch CoM mà `LeanPitchProxy` không thấy. Với các
  động tác đó, P1 (tỉ lệ theo proxy) gần như vô dụng — phải dùng P2 (số cố định
  theo weight).
- **Chỉ bù được trước–sau.** Động tác một tay làm CoM lệch sang bên; muốn bù cần
  thêm bias vào `obs[4]` (gravity.y) và `obs[7]` (cmd_vy). Đó là việc riêng,
  không gộp vào đợt này.
- **Đây là bản vá phía deploy.** Nó dạy policy Flat 83D một lời nói dối có kiểm
  soát để bù cho chuyển động tay mà policy chưa được train. Muốn sửa ở tầng
  policy phải tạo một task/contract riêng và nghiệm thu độc lập; không mở rộng
  observation của route Flat hiện tại trong plan này.
