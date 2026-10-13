# Xử lý mất thăng bằng khi robot làm động tác tay

> Triệu chứng: khi overlay gesture tay lúc đang locomotion, robot **mất thăng bằng** — ví dụ đưa tay ra trước thì thân chúi tới, robot bước tới trước. Nguyên nhân: cử động tay **dịch trọng tâm (CoM)**, tạo nhiễu mà policy locomotion phải gánh.

---

## 1. Chẩn đoán

Code **đã có sẵn** khung bù thăng bằng feedforward khi overlay gesture, nhưng **đang tắt**.

| Thành phần         | Vị trí                                   | Vai trò                                                                                                                                     |
| -------------------- | ------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `LeanPitchProxy()` | `src/motion/ArmGesturePlayer.hpp:156`    | Tín hiệu lệch = (vai-pitch trái − default) + (vai-pitch phải − default); tự scale theo weight blend                                  |
| Nhồi bias vào obs  | `src/app/Application.cpp:1708`           | `gravity_x_bias = kg · lean` → **obs[3]** (projected gravity x = pitch); `cmd_vx_bias = kv · lean` → **obs[6]** (cmd_vx) |
| Mask obs tay         | `src/policy/LocomotionController.hpp:39` | Che q_rel/dq của khớp tay (14..23), giữ`last_action`                                                                                    |
| Gain                 | `config/tuning.yaml:351`                 | `gesture_balance_kg: 0.0`, `gesture_balance_kv: 0.0` → **compensation = OFF**                                                     |

**Kết luận:** vì `kg = kv = 0`, gesture hiện là **nhiễu thuần** → robot chúi/bước theo. Khung bù có sẵn nhưng chưa đo gain nên chưa bật (comment: *"ĐO trong sim; 0=tắt"*).

### 3 hạn chế kể cả khi đã bật gain

1. **Proxy chỉ đọc vai-pitch** → bỏ sót lệch CoM do **duỗi khuỷu** (đưa tay ra trước bằng khuỷu tay) và do **nâng tay cao** (trái tim overhead).
2. **Chỉ bù trước–sau (pitch)**, không bù **ngang (roll / vy)** — mà `vaytay`/`battay` là động tác **một tay phải** → CoM lệch phải → trôi ngang.
3. Gain **tuyến tính cố định**, chưa đo thực nghiệm.

---

## 2. Plan xử lý — 4 mức (an toàn → triệt để)

### Mức 0 — Giảm nhiễu ngay, KHÔNG đụng policy  ⏱️ làm được ngay · rủi ro: 0

- **Gesture chậm hơn**: giảm playback fps / kéo dài pha giơ → CoM đổi từ từ, policy kịp bù.
- **Giảm biên đưa tay ra trước**; giữ tay gần thân; động tác một tay thêm đối trọng nhẹ ở tay kia.
- **Gate theo tốc độ đi**: động tác nặng thăng bằng (đưa quà tay-trước, trái tim overhead) chỉ chạy khi **cmd vận tốc ≈ 0** (đứng), không chạy lúc đang bước nhanh
  - Thêm tham số `gesture_max_speed_to_play` trong `tuning.yaml`.

### Mức 1 — Bật lại bù feedforward có sẵn, đo gain trong sim  ⏱️ ~1 buổi · rủi ro: thấp

- Trong MuJoCo: robot đi + chạy từng gesture, **đo độ trôi thân**.
- Dò `gesture_balance_kg`, `gesture_balance_kv` để triệt tiêu trôi trước–sau.
- Chỉ chạm `obs[3]`/`obs[6]` → degrade an toàn, dễ rollback (đặt lại 0).
- Đúng ý đồ thiết kế sẵn có.

### Mức 2 — Nâng proxy thành CoM thật + bù ngang  ⏱️ vài ngày · rủi ro: trung bình (sửa code)

- Thay `LeanPitchProxy` bằng **CoM tay từ forward-kinematics × khối lượng link** (đọc từ `r1.xml`) theo tư thế tay hiện tại → ra lệch **(x, y)**.
- Feed `x` → `obs[3]`/`obs[6]` (như cũ) **và** thêm `y` → **obs[4]** (gravity y) / **obs[7]** (cmd_vy) để bù **trôi ngang** cho động tác một tay.
- Bắt được cả **duỗi khuỷu + nâng tay + lệch trái/phải**.

### Mức 3 — Train lại policy có nhiễu tay  ⏱️ dài · rủi ro: cần train + verify

- Trong mjlab: **domain-randomize tư thế / tải khớp tay** lúc train → policy học tự bù.
- Hết out-of-distribution, không cần feedforward. **Fix bền vững nhất.**

---

## 3. Thứ tự đề xuất

```
Mức 0 (ngay)  →  Mức 1 (đo sim, bật bù pitch)  →  [nếu vẫn trôi ngang] Mức 2  →  Mức 3 (khi có dịp train)
```

## 4. Cần xác nhận trước khi làm

- [ ] Có sẵn harness MuJoCo để chạy gesture + đo trôi thân không? (phục vụ Mức 1–2)
- [ ] Chọn mức bắt đầu.
- [ ] Nguyên tắc: **chưa sửa code** cho tới khi chốt.

## 4b. KẾT QUẢ CẢI TIẾN SIM + ĐO (2026-08-01)

Đã cải tiến `unitree_mujoco/simulate_cpp` để test gesture thật:
- **Nạp gesture npz THẬT** từ folder deploy (vaytay/battay/traitim/duaqua/namtay_anmung), thay demo wave. Phím **G**=chọn gesture, **V**=chơi/thu.
- **kg/kv đưa ra `config/tuning.yaml`** (`gesture_balance_kg/kv`) — tune không cần build lại.
- **Chế độ `--autotest <ten> [vx= kg= kv= warmup= hold= csv=]`**: chạy không cần bàn phím, kích gesture, đo **đỉnh lệch pitch thân** (metric nhiễu CoM) + trôi base, xuất CSV.
- Dùng policy deploy thật **policy_5.onnx**.

**2 BUG CODE của sim đã tìm & sửa (đây là lý do "robot đi không vững"):**
1. **Gains PD cũ (lệch train/deploy):** sim gửi `R1Config` KD chân=2.0, eo=2.0, tay KP=20/KD=1. Deploy ĐỌC gains từ metadata ONNX → KD hip/gối/eo=**3.0**, tay KP=**40**/KD=**2** (đổi 2026-07-09). Sim under-damped → loạng choạng. **Đã sửa `R1Config.hpp` khớp `kKpTrain/kKdTrain`.**
2. **Chu kỳ gait sai:** sim hardcode **0.6s**, deploy `kGaitPeriodS=`**0.8s** → phase bước nhanh 33% → sai nhịp. **Đã sửa main.cpp → 0.8s.**

**Hạn chế còn lại (chưa tune được kg/kv sạch trong sim):**
- Sim **không giữ policy_5 đứng vững** để đo cô lập: đứng yên (vx=0) là ngoài phân bố của policy walking (deploy dùng StandLock riêng) → robot lang thang vài mét; đi tới thì **thiếu heading-hold** (deploy có `heading_hold_kp`) → trôi ngang thành cung; fall-detector cắt motor ở tilt 60° đôi khi quá sớm.
- Hiệu ứng pitch của gesture (~0.05 rad với duaqua) nhỏ so với nhiễu nền của base bất ổn → **chưa ra được kg/kv tin cậy bằng auto-sweep.**
- **Cần trước khi tune kg/kv trong sim:** thêm heading-hold vào sim FlatController (khớp deploy) để robot đi thẳng → cô lập được nhiễu gesture. Hoặc tune kg/kv **thận trọng trên robot thật** (bắt đầu kg nhỏ, tăng dần, quan sát).

## 4c. GỐC RỄ vì sao policy_5 ngã trong unitree_mujoco (2026-08-01)

Deploy thật chạy **policy_5.onnx** (`tuning.yaml: flat_model: policy_5.onnx`). Kiểm chứng trong sim:
- **policy CŨ `policy_r1_flat`** trong unitree_mujoco → **đứng vững** (pitch≈0, đi 4m không ngã).
- **policy_5** → ngã (~t=12s).

**Nguyên nhân: model r1.xml của unitree_mujoco KHÁC model train policy_5 (mjlab):**
| | tổng mass | số body |
|---|---|---|
| unitree_mujoco r1.xml | **30.18 kg** | 26 |
| mjlab train (`unitree_rl_mjlab/.../unitree_r1/xmls/r1.xml`, run v9) | **28.93 kg** | 25 |

policy_5 học trên model 28.93kg → chạy trên model 30.18kg thì mất thăng bằng. **Muốn "làm chuẩn trong sim" cho policy_5 phải khớp model.**

**Không swap thẳng model mjlab được**: model mjlab thiếu sensor DDS bridge cần (`imu_quat`/`frame_pos`/`frame_vel`, tên khác) và không có `<actuator>` (mjlab áp torque trực tiếp).

### Đã TEST A/B/C (2026-08-01) — kết quả empirical

**So model chi tiết:** 2 model r1.xml GIỐNG HỆT 24 body, chỉ khác **1 body `dev_pc` (+1.25kg, máy tính dev sau lưng)** → unitree_mujoco 30.18kg vs mjlab 28.93kg.

- **(A) Khớp model trong unitree_mujoco — ĐÃ THỬ, KHÔNG ĐỦ.** Bỏ dev_pc → mass khớp 28.93kg, NHƯNG **policy_5 vẫn ngã** (pitch0≈-0.93 khi đi vx=0.3). → còn **gap physics sâu hơn** (spawn transient / solver / contact / cách bridge áp PD khác mjlab). Ngoài ra sim **ngã ngẫu nhiên lúc spawn** (robot thả rơi từ pelvis z=0.74 chân thẳng, không có keyframe đứng) — cả policy CŨ cũng ngã 2/5 lần. Muốn A chạy được: **thêm keyframe spawn đứng** + có thể phải khớp option physics. Công sức trung bình-cao, chưa chắc.
- **(B) Thêm sensor+actuator vào model mjlab** → cùng gap physics như A + nhiều plumbing. Không hơn A.
- **(C) Dùng chính mjlab sim `scripts/play_r1_xml.py` — KHUYẾN NGHỊ.** policy_5 train/eval ở đây → chạy đúng theo thiết kế. Cần port overlay gesture (ArmGesturePlayer + nạp npz + kg/kv + log drift) vào play script. **Tin cậy nhất để tune policy_5 thật.**

**Đo balance-risk từng gesture** (policy cũ đứng vững trong unitree_mujoco, vx=0.3, các lần pitch0≈-0.02 hợp lệ) — đỉnh lệch pitch thân:
| gesture | đỉnh pitch | ghi chú |
|---|---|---|
| duaqua | **0.086** | cao nhất — đưa tay ra trước, đúng dự đoán |
| namtay_anmung | 0.074 | |
| battay | 0.073 | |
| vaytay / traitim | (ngã lúc spawn, chưa đo được) | |

**ĐÃ sửa trong lần này (sim đúng hơn hẳn):** gains train (4b#1), gait 0.8s (4b#2), **heading-hold** (khớp deploy), harness autotest + nạp gesture thật + kg/kv config.

### PLAN CHÍNH XÁC (đề xuất)
1. **Faithful tuning policy_5 → Option C**: port overlay gesture vào mjlab `play_r1_xml.py`, tune kg/kv với policy_5 trên đúng model. (Việc chính, ~1–2 buổi.)
2. **HOẶC pragmatic**: thêm keyframe spawn-đứng vào unitree_mujoco → hết ngã ngẫu nhiên → tune bằng policy cũ (gần đúng), rồi xác nhận hướng kg/kv trên robot.
3. **Xử lý theo gesture** (từ ranking): duaqua = gate về đứng/làm chậm; traitim = nâng CoM đối xứng (vừa); vaytay/battay = một tay → trôi ngang; bật kg (pitch) + thêm bù ngang kv sau khi tune.

## 5. Mức độ rủi ro thăng bằng của các gesture hiện có

| Gesture           | Tay                  | Đặc điểm CoM                   | Rủi ro                         |
| ----------------- | -------------------- | ---------------------------------- | ------------------------------- |
| `duaqua`        | đưa tay ra trước | CoM dồn**trước** mạnh    | ⚠️ cao (trôi tới)           |
| `vaytay`        | phải, giơ cao      | lệch**phải** + nâng       | trung bình (trôi ngang)       |
| `battay`        | phải, giơ + lắc   | lệch**phải**               | trung bình (trôi ngang)       |
| `traitim`       | hai tay overhead     | đối xứng, CoM**lên cao** | thấp hướng, nhưng nâng CoM |
| `namtay_anmung` | hai tay              | tuỳ tư thế                      | thấp–trung bình              |
