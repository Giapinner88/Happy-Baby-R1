# Plan: Ngồi có kiểm soát thăng bằng — ĐÃ LÀM

> **Trạng thái: đã code (2026-07-14), build OK, CHƯA test trên robot thật.**
> Ngồi giờ có **4 pha**: mở chân đế → hạ người (đổ thân + vươn tay) → ngồi hẳn
> (thu tay, dựng thân) → giữ. Bàn chân bám **mặt đất qua IMU**, không bám hông.
> Tham số: `sit_hip_deg` 150, `sit_knee_deg` 100, `sit_lean_deg` 58, `sit_arm_forward`,
> `sit_spread`, `sit_ankle_gravity_gain` — xem `config/tuning.yaml`.
>
> **Chiều cao ghế: 43 cm.** Biên an toàn nhỏ nhất suốt quá trình hạ (đo MuJoCo):
> cách cũ (thân thẳng, tay buông) = **−9.8 cm → NGÃ NGỬA**; cách mới = **+9.2 cm**.
> Ngồi SÂU hơn thì AN TOÀN HƠN (gập hông nhiều → trọng tâm ra trước).
>
> ⚠️ Lưu ý đọc phần dưới: `sit_stance` (làm chân HẸP) **đã bị bỏ**, thay bằng
> `sit_spread` (làm chân RỘNG). Công thức `ankle = -(hip+knee)` cũng **đã bỏ** —
> nó khóa thân thẳng đứng và chính là nguyên nhân ngã ngửa.
>
> Mọi số liệu dưới đây đo bằng model thật `GMR/assets/unitree_r1/r1.xml` (MuJoCo),
> không phải phỏng đoán.

## 1. Hiện tượng
Bấm nút ngồi (L2+X): robot mất thăng bằng, **bàn chân vênh lên không chạm hết đất**,
**chân để sát nhau**, gối vuông góc nhưng **hông mở ra**, robot "chạng tè chạng tách".

## 2. Nguyên nhân — đo được, không phải đoán

### Lỗi gốc: code KHÓA CỨNG thân robot ở phương thẳng đứng
```cpp
target[4]  = -(target[0] + target[3]);   // ankle_pitch = -(hip_pitch + knee)
target[10] = -(target[6] + target[9]);
```
Với bàn chân phẳng trên đất, **góc nghiêng thân = hip + knee + ankle**.
Công thức trên ép **tổng = 0** → **thân LUÔN thẳng đứng**, ở mọi thời điểm hạ người.

Người ta ngồi xuống thì **đổ người về trước** để giữ trọng tâm trên bàn chân.
Robot này **bị code cấm làm điều đó**.

### Hệ quả: tư thế ngồi hiện tại KHÔNG THỂ cân bằng
Đo trên model (hip −90°, gối +90°, thân thẳng đứng):

| | giá trị |
|---|---|
| Trọng tâm so với **gót chân** | **−16.1 cm** (ra **sau** gót) |
| Trọng tâm so với **mũi chân** | +25.3 cm |
| Chiều cao hông | 0.582 m (đứng: 0.724 m) |

→ **Trọng tâm nằm ngoài chân đế 16 cm về phía sau.** Đây **không phải lỗi tham số** —
nó là **hình học của tư thế "ngồi ghế"**: đùi nằm ngang đẩy hông ra sau ~0.3 m.
Robot **bắt buộc ngả ngửa**, chỉ có cái ghế đỡ lại.

### Bàn chân vênh
`ankle = -(hip+knee)` chỉ giữ bàn chân **song song với hông**, không phải song song **mặt đất**.
Khi robot ngả ra sau, **hông nghiêng** → bàn chân **vênh lên đúng bằng góc nghiêng đó**.
Code **chưa hề dùng `projected_gravity`** (đã có sẵn trong `RobotState`) để bù.

### Chân sát nhau
`gather[1] = -stance; gather[7] = +stance` → `sit_stance` làm chân **HẸP** lại.
So với khóa đứng: `target[1] += spread; target[7] -= spread` → **ngược dấu** (làm rộng ra).
Hiện `sit_stance: 0.0` nên chân ở bề rộng mặc định (vốn hẹp) → chân đế ngang bé → dễ đổ ngang.

## 3. Giới hạn phần cứng phải chấp nhận

Quét toàn bộ không gian (hip, cổ chân, độ đổ người, vai) trên model:

| gối | hip | cổ chân | thân đổ trước | vai | hông cao | biên gót/mũi |
|---|---|---|---|---|---|---|
| 0° | +6° | −6° | 0° | +0.35 | 0.820 | +6.6 / +2.1 cm |
| 30° | −14° | −16° | 0° | +0.35 | 0.805 | +6.0 / +2.7 cm |
| 50° | −120° | +18° | **+52°** | **−1.90** | 0.683 | +1.5 / +7.5 cm |
| 70° | −132° | +8° | **+54°** | −1.50 | 0.641 | +2.7 / +6.4 cm |
| 90° | −132° | −12° | **+54°** | +0.35 | 0.619 | +3.3 / +5.7 cm |

**Kết luận quan trọng:**
- Hạ thấp nhất **mà vẫn cân bằng** = hông ở **0.619 m** (đứng là 0.724 m) → **chỉ hạ được ~10 cm**.
- Giới hạn chặn là **tầm cổ chân: chỉ [−50°, +33°]**. Không đủ để đẩy trọng tâm ra trước nhiều.
- **Không có tư thế nào vừa ngồi thấp vừa cân bằng.** Đó là **bản chất của việc ngồi**:
  ngồi = chuyển trọng lượng ra **sau** chân, lên ghế. Không thể vừa ngồi vừa đứng vững.

→ Yêu cầu *"cân bằng được trước khi mông chạm ghế"* **chỉ đạt được một phần**:
robot giữ thăng bằng **suốt quãng hạ ~10 cm đầu**, sau đó **buộc phải giao trọng lượng cho ghế**.
Việc cần làm là **rút ngắn đoạn "buông"** đó xuống còn vài cm thay vì đổ ập từ 30 cm.

## 4. Đề xuất sửa

### 4.1 Cho phép đổ người về trước (lỗi gốc)
Thêm `sit_lean_deg` — góc đổ thân về trước:
```cpp
float lean = deg2rad(sit_lean_deg);
target[4]  = -(target[0] + target[3]) - lean;   // bỏ khóa cứng thân thẳng đứng
target[10] = -(target[6] + target[9]) - lean;
```
Trong pha hạ, `lean` tăng dần **0 → sit_lean_deg** rồi **về 0** khi đã ngồi lên ghế.
Chặn theo tầm cổ chân **[−50°, +33°]** để không đòi hỏi góc không thể đạt.

### 4.2 Vươn tay ra trước làm đối trọng
Dấu đã kiểm chứng: **shoulder_pitch ÂM = tay ra trước** (kéo trọng tâm về trước).
Thêm `sit_arm_forward` (rad, ví dụ −1.5), áp cho khớp **14** (vai trái) và **19** (vai phải),
duỗi khuỷu (17, 22) về 0. Tay đưa ra trong pha hạ, **thu về sau khi gối đạt 90°** (đúng ý bạn).
Đối trọng tay đo được: kéo trọng tâm ra trước **~4 cm** — ít, nhưng cộng dồn với đổ người thì đáng kể.

### 4.3 Bàn chân phẳng theo MẶT ĐẤT, không theo hông
Dùng `rs.projected_gravity` để lấy pitch/roll **thật** của hông rồi bù vào cổ chân:
```
ankle_pitch = -(hip + knee) - pelvis_pitch_do_duoc
ankle_roll  = -hip_roll     - pelvis_roll_do_duoc
```
→ bàn chân bám đất kể cả khi thân nghiêng. Hết "vênh lên".

### 4.4 Mở rộng chân đế
Đổi `sit_stance` thành **làm rộng** (cùng dấu với `stand_lock_spread`), hoặc thêm `sit_spread`.
Chân đế ngang rộng hơn → hết "chạng tè chạng tách" theo phương ngang.

### 4.5 Thêm pha "ngồi hẳn"
Pha 0 thu chân → pha 1 **hạ có cân bằng** (đổ người + vươn tay, hạ tới ~0.62 m) →
pha 2 **giao trọng lượng cho ghế** (hip về −90°, thân về thẳng, tay thu về) →
pha 3 giữ. Ghế nên đặt sao cho mặt ghế **ngay dưới hông ở cuối pha 1** để đoạn "buông" ngắn.

## 5. Cần bạn xác nhận
- **Chiều cao mặt ghế?** Cần biết để đặt đích pha 1. Nếu ghế **quá thấp**, robot buộc phải
  "rơi" một đoạn dài → không cách nào êm.
- Có chấp nhận **robot đổ người ra trước rõ rệt** (tới ~50°) lúc hạ không? Nhìn sẽ giống
  người đang cúi ngồi xuống ghế.
