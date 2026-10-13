# Vào điệu nhảy êm (mimic soft-start) — transition v2

> **Trạng thái: đã code và đã kiểm tra preflight/MuJoCo C++.** `mimic_transition_v2: true`
> đang bật trong `config/dance.yaml`; chưa nghiệm thu trên robot thật.
> Cách `mimic_start_at_clip_pose` (ép chân open-loop) đã **bỏ hẳn** — xem mục 3.


## 1. Hiện tượng
Khi bật policy mimic, robot mất thăng bằng mạnh ở **giai đoạn chân di chuyển cho khớp với clip npz**.
Tay vung nhiều nhưng **không ảnh hưởng** — vì tay không chịu lực.

## 2. Dữ liệu đo được (log lúc khởi động + quét npz)

Khoảng lệch giữa **tư thế đứng** (`default_q`) và **tư thế frame đầu của clip**:

| Điệu | frame đang chọn | lệch CHÂN | lệch tay/waist |
|---|---|---|---|
| 2 `lacmong1` | 65 | **0.57 rad (33°)** | 0.84 rad |
| 3 `lacmong2` | 162 | **0.79 rad (45°)** | 1.49 rad |
| 4 `pokemon` | 9 | **0.50 rad (28°)** | 0.97 rad |
| 5 `doremon` | 13 | **0.51 rad (29°)** | 0.80 rad |

Chi tiết tư thế chân ở frame đầu (ví dụ điệu 3, frame 162):

| khớp | trái | phải | ghi chú |
|---|---|---|---|
| hip_pitch | −37.8° | **−51.1°** | lệch tới −45° so với đứng |
| hip_roll | +7.0° | **−16.4°** | **bất đối xứng** |
| hip_yaw | −3.1° | **+20.6°** | **bất đối xứng** |
| ankle_roll | +2.3° | **+14.6°** | **bàn chân đang NGHIÊNG** |

### Kết luận 1 — không né được bằng cách đổi frame
Quét **toàn bộ clip** (2224–4393 frame), frame "gần đứng nhất" vẫn lệch chân **0.30–0.65 rad (17–37°)**.
Điệu 2 cả clip **chưa bao giờ đứng thẳng** (pelvis cao nhất 0.778 m).
→ **Tư thế chân của điệu nhảy vốn khác tư thế đứng.** Robot BẮT BUỘC phải đổi cấu hình chân để vào điệu.

### Kết luận 2 — tư thế frame đầu KHÔNG đứng vững được
Bất đối xứng + bàn chân nghiêng = **tư thế giữa động tác**, đang dồn trọng lượng một bên,
robot chỉ giữ được nhờ **đà và điều khiển kín**. Đây **không phải** tư thế tĩnh.

## 3. Vì sao 2 cách "hiển nhiên" đều sai

**Cách A (legacy route khi transition v2 tắt):** về `default_q` rồi bật mimic.
→ Policy thấy sai số chân 28–45° ngay t=0, mà **clip vẫn chạy tiếp** (phase advance ngay tick đầu)
→ policy đuổi theo mục tiêu đang chạy trốn từ điểm xuất phát sai → **văng mạnh**.

**Cách B (đã bỏ):** ép robot về đúng tư thế frame đầu rồi mới bật mimic.
→ Đúng về mặt "sai số t=0 = 0", **nhưng** lúc ép là **policy đang TẮT**, chỉ có PD cứng (Kp=200) open-loop.
→ Ép robot vào một tư thế **bất đối xứng, bàn chân nghiêng, không tĩnh ổn định** mà **không có gì giữ thăng bằng** → **đổ**.
→ Đây chính là "giai đoạn di chuyển chân" gây mất thăng bằng.

**Đã bỏ hẳn cách B khỏi code** (không để lại flag, vì bật nhầm là robot đổ).
Kiểm chứng bằng model MuJoCo `GMR/assets/unitree_r1/r1.xml`: xem `docs/PLAN_sit_balanced.md`.

## 4. Cách đúng: để CHÍNH POLICY đưa chân vào điệu (soft-start)

Không ai ép chân open-loop cả. Bật policy mimic **ngay**, nhưng **cho nó một mục tiêu di chuyển chậm**:

```
t = 0        : reference = tư thế robot ĐANG đứng    -> sai số = 0
t = 0..T     : reference trượt dần  đứng -> clip[start_frame]   (clip ĐỨNG YÊN, chưa chạy)
t = T        : reference = clip[start_frame], robot đã tới nơi  -> sai số ~ 0
t > T        : thả clip chạy + bật nhạc                          -> vào điệu
```

Trong suốt giai đoạn 0..T (khoảng **1.0–1.5 s**):
- Policy **luôn thấy sai số nhỏ** → không văng, luôn trong vùng nó được huấn luyện.
- Policy **đang chạy** → nó **tự giữ thăng bằng**: dùng cổ chân, dồn trọng lượng, bước chân nếu cần.
- Robot **tự đi vào** tư thế mở màn thay vì **bị ép** vào.

### Vì sao cách này thắng
| | Cách A | Cách B | Soft-start |
|---|---|---|---|
| Sai số policy thấy ở t=0 | **rất lớn** | 0 | **0** |
| Có giữ thăng bằng khi chân di chuyển? | — | **KHÔNG** | **CÓ (policy)** |
| Lệch nhạc | không | không | **không** |
| Phải chọn lại frame? | — | có | **không** |

### Đã triển khai
- `MimicController` có transition v2 opt-in: chọn entry frame runtime theo q/dq hiện tại,
  rồi dùng quintic boundary trajectory để khớp cả q và dq ở đầu warmup/cooldown.
- Sau warmup, tốc độ phase tăng bằng quintic ramp `mimic_transition_clip_ramp_s`,
  nên reference velocity không nhảy từ 0 lên vận tốc clip ở tick đầu.
- Soft-stop cũng nối tiếp `last_ref_q/last_ref_dq`; handover không chỉ dựa vào tilt/gyro
  mà còn yêu cầu q gần `default_q` và max|dq| dưới ngưỡng. Nếu strict gate chưa đạt
  sau `max_s`, chỉ cho phép bounded fallback trong `0.25 rad / 1.50 rad/s`; nếu
  vượt vùng đó, HB chuyển sang STAND LOCK recovery thay vì dump sang locomotion.
- Route cũ vẫn tồn tại khi `mimic_transition_v2: false`.
- Config hiện dùng: `mimic_warmup_s: 1.5`, search 200 frame, clip ramp 0.35 s.

### Rủi ro còn lại
- Tư thế mở màn vẫn bất đối xứng → policy phải giữ nó **tĩnh** trong khoảnh khắc trước khi clip chạy.
  Nếu vẫn chông chênh: **giảm `dance_start_search_frames`/đổi `dance_start_frame`** sang frame đối xứng hơn,
  hoặc rút ngắn `mimic_warmup_s` để clip chạy sớm (có đà thì dễ giữ hơn đứng yên).
- Đã có **bảo vệ ngã** ở giai đoạn này.

## 5. Chấm điểm chọn frame transition v2
`FindSmoothStartFrame` hiện tính `score = vel + 0.1*leg_err + 0.05*tilt`.
Trọng số `leg_err` quá nhỏ, **và không hề xét đối xứng trái/phải hay bàn chân có phẳng không**.
Nên đổi thành:
```
score = leg_gap + 2.0*bat_doi_xung + 2.0*ankle_roll + 0.05*vel + 0.01*tilt
```
(giữ cửa sổ tìm kiếm nhỏ để **không lệch nhạc** — mỗi 50 frame = 1 s lệch nhạc.)

Transition v2 đã dùng score mở rộng: thêm sai số q/dq so với trạng thái thật và
phạt vận tốc chân trong ba frame kế tiếp. Preflight hiện tại ghi nhận các thay đổi
đáng kể so với legacy: dance 3 `12 -> 18`, dance 4 `23 -> 69`, dance 5/6 `13 -> 7`;
dance 2 giữ frame 0 vì score không tìm được ứng viên tốt hơn trong 200 frame đầu.

## 6. Bằng chứng kiểm tra

- `mimic_transition_test`: kiểm tra q/dq đúng ở hai đầu quintic, pass.
- HB `run_r1 --preflight`: tất cả dance ONNX/NPZ và transition config load được, pass.
- MuJoCo C++ `run_policy` build + `mimic_transition_test` + `tuning_profile_test`, pass.
- MuJoCo C++ headless stack với `--dance-test=2` đã chạy đủ locomotion -> warmup ->
  dance finished -> cooldown -> locomotion; lần chạy ghi nhận `bounded fallback`,
  tại thời điểm giao có `max_pos_err=0.211 rad`, `max_dq=0.0037 rad/s`,
  tilt `0.0203`, gyro `0.00081`; không phát hiện NaN/fall. Đây vẫn chưa phải
  hardware acceptance hay acceptance chất lượng toàn bộ bài dance.
