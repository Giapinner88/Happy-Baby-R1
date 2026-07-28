# Báo cáo hoàn chỉnh — Điều khiển thân trên (tay + đầu) chồng lên Locomotion: Gesture library & Teleop

> Nguồn tham chiếu (sim): `unitree_mujoco/simulate_python/run98_3.py` + `arm_csv_player.py`
> Đích deploy: `HB/high_level_2` (robot R1 thật)
> Yêu cầu mới: (1) **dùng nhiều động tác tay** khi locomotion trong sự kiện; (2) **tương lai teleop tay + đầu** robot theo thời gian thực.
> Trạng thái: **phân tích + kế hoạch** — chưa chỉnh code. Thay thế cho `PLAN_arm_gesture_overlay.md`.

---

## 1. Kết luận nhanh (đọc trước)

- **Nhiều gesture tay khi đi:** KHẢ THI CAO, rủi ro thấp. Chỉ là mở rộng overlay của sim thành thư viện npz + trình phát có ưu tiên/blend.
- **Teleop ĐẦU:** KHẢ THI RẤT CAO, gần như không rủi ro thăng bằng — **đầu (idl 29/30) hoàn toàn tách rời policy**, chỉ do `LowCmdSender` giữ ở 0. Chỉ cần cho nó nhận target ngoài. Làm được ngay.
- **Teleop TAY:** KHẢ THI, rủi ro TRUNG BÌNH (thăng bằng). Với động tác vừa phải: dùng kiến trúc Tier 2 dưới đây (overlay + che-obs + bù thăng bằng closed-loop) là đủ, **không cần train lại**. Với teleop nặng/động (vươn xa, mang vật, vung nhanh khi đang đi): cần Tier 3 (train policy nhận lệnh thân trên).
- **Khuyến nghị:** xây **một** trừu tượng hợp nhất — *"nguồn cấp target thân trên"* (upper-body provider). Gesture playback và teleop là **hai nguồn của cùng một đường ống**. Làm Tier 2 trước (chạy được cả hai), thiết kế interface để sau này cắm Tier 3 vào mà không đập lại kiến trúc.

---

## 2. Kỹ thuật sim & đánh giá (tóm tắt)

Overlay cử chỉ tay lên policy locomotion, 3 trụ:

1. **Ghi đè output có blend** — `q = (1-w)·q_policy + w·q_ref` cho khớp tay, `w` ramp lên/xuống mượt.
2. **Che quan sát tay** (điểm hay nhất) — zero `q_rel/dq/last_action` của tay → policy "tưởng tay ở default" → không phá dáng đi.
3. **Bù thăng bằng thủ công** — bias `vx` & `projected_gravity.x` theo `w`, tuned tay từng động tác.

**Điểm yếu chí mạng cho yêu cầu mới:** trụ (3) là **open-loop, hằng số, tuned tay per-motion**. Với thư viện lớn thì tuning tay không scale; với teleop (động tác không biết trước) thì **không thể tune trước** → bắt buộc phải nâng cấp thành closed-loop / model-based.

---

## 3. Sự thật kiến trúc high_level_2 quyết định thiết kế

| Sự thật | Hệ quả thiết kế |
|---|---|
| Tay = policy idx **14–23** (5 trái, 5 phải), **được policy quan sát** (`q,dq,last_action`) | Override tay **bắt buộc** kèm che-obs, nếu không policy loạn |
| Waist = idx 12 (roll) + idx 13 (yaw) | **Roll: KHÔNG override** (dịch CoM ngang → dễ lật). **Yaw: override NHẸ được** (xoay quanh trục trọng lực, không dịch CoM) — xem §7.5 |
| **Đầu = idl 29/30, KHÔNG thuộc policy** — chỉ `LowCmdSender::SetHead` giữ ở 0 ([LowCmdSender.hpp:68-69](../src/robot/LowCmdSender.hpp#L68)) | **Teleop đầu tách rời hoàn toàn**: chỉ cần cho sender nhận target đầu. Zero tương tác policy, zero rủi ro thăng bằng |
| Vòng DDS **500Hz**, inference policy **50Hz** (`kPolicyDecimation=10`) | Override tay/đầu chạy **mỗi tick 500Hz** trước `Send` → mượt cho teleop dù policy 50Hz |
| `LowCmdSender::Send` đã có **slew rate-limit + CRC + head hold** ([LowCmdSender.hpp:41](../src/robot/LowCmdSender.hpp#L41)) | Override chỉ cần set `ai_target_q_[14..23]`; giới hạn tốc độ có sẵn |
| Điểm tiêm = `RunPolicy()` ngay **trước** `sender_.Send(...)` ([Application.cpp:1480](../src/app/Application.cpp#L1480)) | Chèn một lời gọi `manager.Apply(ai_target_q_, head_target)` tại đây |
| Kênh vào obs = `ControlContext` ([PolicyController.hpp:15](../src/policy/PolicyController.hpp#L15)) | Thêm field `arm_override_active/weight` để `BuildObservation` che-obs |
| Đã có `JointTrajectory`+`MotionData`+`tools/record_motion.cpp` (npz, nội suy theo thời gian) | Dùng npz cho gesture — **không cần CSV** |
| Đã có mẫu socket datagram (`IntegrationNotifier` AF_UNIX) | Teleop dùng UDP theo cùng phong cách, lỗi socket không làm sập motor loop |

---

## 4. Insight hợp nhất: gesture & teleop là CÙNG một bài toán

Cả hai đều là **"một nguồn bên ngoài cấp góc mục tiêu cho thân trên, ghi đè output policy cho tay, kèm che-obs + bù thăng bằng"**. Khác nhau **chỉ ở nguồn dữ liệu**:

```
                         ┌── GesturePlayer (đọc npz)         ── file, biết trước
IUpperBodyProvider  ◄────┤
                         └── TeleopReceiver (UDP stream)     ── live, không biết trước

           ┌───────────────────────────────────────────────┐
           │  UpperBodyManager                              │
           │   • chọn nguồn (ưu tiên: teleop > gesture)     │
           │   • blend in/out (w)                           │
           │   • clamp giới hạn khớp + slew                 │
           │   • balance guard (fade khi robot nghiêng)     │
           │   • watchdog (mất teleop → thu về policy)      │
           └───────────────┬───────────────────┬───────────┘
                           │                   │
              arm target[14..23]         head target[yaw,pitch]
                           │                   │
     RunPolicy: ai_target_q_[14..23] ◄──┘      └──► LowCmdSender.SetHead(target)
     + ControlContext.arm_override → che-obs
```

Thiết kế **một** interface này ngay từ đầu là điểm mấu chốt để teleop tương lai chỉ là "cắm thêm một provider".

---

## 5. Đánh giá tính khả thi chi tiết

| Khả năng | Khả thi | Rủi ro | Điều kiện cần | Bậc giải pháp |
|---|---|---|---|---|
| Nhiều gesture tay khi đi | ✅ Cao | Thấp | Thư viện npz + blend + safety | Tier 2 |
| Teleop **đầu** | ✅ Rất cao | Rất thấp | Sender nhận target đầu + clamp + watchdog | Tier 2 (làm ngay được) |
| Teleop **tay** (vừa phải, đi chậm) | ✅ Được | Trung bình | Che-obs + bù thăng bằng closed-loop + balance guard + giảm biên theo tốc độ | Tier 2 |
| Teleop **tay** nặng/động (vươn xa, mang vật, vung nhanh khi đi nhanh) | ⚠️ Giới hạn | Cao | Policy phải **học bù tay** → train lại | **Tier 3** |

---

## 6. Lộ trình 3 bậc

- **Tier 1 — bản sim:** overlay + che-obs cứng (zero) + bias hằng số. Đủ cho vài động tác nhẹ cố định. *Không scale cho thư viện lớn / teleop.*
- **Tier 2 — ĐÍCH TRƯỚC MẮT (khuyến nghị làm ngay):** provider hợp nhất + che-obs (mềm) + **bù thăng bằng closed-loop/model-based** + head decoupled + safety đầy đủ. **Không train lại.** Chạy được thư viện gesture lớn **và** teleop tay vừa phải + teleop đầu.
- **Tier 3 — ĐÍCH CUỐI (cần train):** train policy locomotion nhận **lệnh tư thế thân trên làm input**, tự tracking tay + tự bù thăng bằng. Bỏ hẳn che-obs & bù thủ công. Cho teleop nặng/động. Interface Tier 2 giữ nguyên, chỉ swap policy + tắt mask.

---

## 7. KẾ HOẠCH CHI TIẾT — Tier 2 (kỹ thuật cốt lõi)

### 7.1 Thành phần mới

**`IUpperBodyProvider`** (interface)
```
struct UpperBodyTarget {
  bool  arm_valid;  std::array<float,10> arm_q;   // policy idx 14..23
  bool  head_valid; float head_yaw, head_pitch;
  float authority;                                // 0..1 — nguồn muốn chiếm bao nhiêu
};
virtual bool Poll(float dt, UpperBodyTarget& out) = 0;
```

**`GesturePlayer : IUpperBodyProvider`** (`src/motion/GesturePlayer.hpp`)
- Nạp N gesture từ **npz** (tái dùng `JointTrajectory`, chỉ lấy 10 cột tay 14–23; tuỳ chọn 2 cột đầu).
- State machine theo **thời gian t (giây)**, KHÔNG frame index cứng: `idle → blend-in → play → hold → retract`. Bỏ magic numbers (70/78/87/112) — mốc peak/loop/hold khai báo trong metadata mỗi clip.
- Trigger theo tên; hỗ trợ loop (vẫy tay) và one-shot-hold (bắt tay).

**`TeleopReceiver : IUpperBodyProvider`** (`src/input/TeleopReceiver.hpp`)
- Thread UDP nhận gói nhỏ `{seq, t_send_us, arm_q[10], head[2], flags}` ~30–100Hz.
- Ghi vào **latest buffer atomic** + `last_recv_us`. Consumer 500Hz đọc buffer, **giữ giá trị cuối** giữa các gói.
- `authority = 0` nếu `now - last_recv > teleop_timeout_ms` (watchdog) → Manager fade tay về policy.

**`UpperBodyManager`** (`src/motion/UpperBodyManager.hpp`) — trung tâm:
- Chọn nguồn theo ưu tiên (teleop > gesture), quản `blend_weight w`.
- `Apply(std::array<float,24>& target_q, HeadTarget& head)`: ghi đè `target_q[14..23]` và điền head.
- Chứa toàn bộ **safety** (mục 7.5).

### 7.2 Tích hợp Application / RunPolicy
Trong `RunPolicy()`, **sau** khi có `ai_target_q_`, **trước** `sender_.Send(...)` ([Application.cpp:1480](../src/app/Application.cpp#L1480)):
```cpp
HeadTarget head;
upper_body_.Apply(ai_target_q_, head, rs, state_);   // ghi đè tay + head, có safety
sender_.Send(ai_target_q_, kp_eff, kd_eff, tuning_.policy_rate_limit, rs.mode_machine, head);
```
Gọi **mỗi tick (500Hz)** → tay/đầu mượt cho teleop dù policy 50Hz.

Cập nhật `ControlContext` cho che-obs:
```cpp
ctx.arm_override = upper_body_.ArmOverrideState();  // {active, weight, ref_arm_q[10]}
```

### 7.3 Che quan sát tay (obs masking)
Trong `LocomotionController::BuildObservation`, khi `ctx.arm_override.active`:
- **Mức nền (an toàn, khớp sim):** `q_rel[14..23]=0`, `dq[14..23]=0`, `last_action[14..23]=0`.
  (Offset trong obs 83-D: q_rel tại `11+i`, dq tại `35+i`, last_action tại `59+i`, tay `i=14..23`.)
- **Che mềm (thử nghiệm, khuyến nghị A/B trong sim):** giữ nguyên `last_action[arm]` (đầu ra tự thân của policy — self-consistent), chỉ che `q_rel/dq`. Trộn theo `w` để blend-in/out không tạo bước nhảy:
  `q_rel_obs[arm] = (1-w)·q_rel_true[arm] + w·0`.
  → Giảm cú sốc quan sát lúc bật/tắt override. **Phải nghiệm thu trong sim trước khi lên robot.**

### 7.4 Bù thăng bằng closed-loop / model-based (thay bias hằng số)
Thay 2 hằng số của sim bằng **feedforward theo độ lệch tay** (tự scale, dùng được cho teleop bất kỳ):
```
lean_pitch = Σ_arm  m_j · (arm_q[j] − default_arm[j])        // m_j: hệ số cánh tay đòn/khối lượng, hiệu chỉnh trong sim
gx_bias    = k_g · lean_pitch · w
vx_bias    = k_v · lean_pitch · w
proj_grav_obs.x += gx_bias ;  cmd_vx_obs -= vx_bias
```
- **Feedforward từ lệnh tay** (không phải từ IMU) → tránh tạo vòng lặp qua obs gravity; robust với động tác chưa biết.
- Tuỳ chọn *trim tích phân chậm* trên sai số pitch đo được để triệt lean tĩnh (giới hạn biên độ để không đánh nhau với policy).
- **⚠ Bias phải là trường obs RIÊNG, chỉ chạm `obs[3]` (gravity.x) và `obs[6]` (cmd_vx).** TUYỆT ĐỐI không áp `vx_bias` bằng cách sửa `cmd_vx_`: trong [RunPolicy](../src/app/Application.cpp#L1447) `cmd_norm` (từ `cmd_vx_/vy_/yaw_`) **cổng nhịp gait** (`gait_.PhaseObs(cmd_norm)`) → sửa `cmd_vx_` sẽ vô tình đổi cả gait phase. Truyền bias qua `ControlContext` để `BuildObservation` cộng thẳng vào obs.
- **Tuỳ chọn tín hiệu tốt hơn:** thay/bổ sung proxy động học `m_j·Δq` bằng `tau_est` của khớp tay (đã có trong telemetry `CaptureMimicTelemetry`) — đo **lực tay thật đang kéo** thay vì mô hình.

### 7.5 Lớp an toàn (bắt buộc cho robot thật)
1. **State gating:** chỉ cho override ở `kLocomotion`. Chặn ở mimic/standlock/idle/fall/transition.
2. **Watchdog teleop:** mất gói > `teleop_timeout_ms` → fade tay về policy, đầu về 0. (Đầu risk thấp nên timeout có thể rộng hơn.)
3. **Balance guard:** nếu `tilt=√(gx²+gy²) > tilt_max` hoặc `gyro.norm() > gyro_max` → giảm `w` theo tốc độ (fade tay về policy để bảo vệ dáng đi). Kết hợp fall-detector sẵn có.
4. **Giới hạn khớp + tốc độ:** clamp `arm_q`/`head` trong dải hợp lệ; giữ nguyên slew của `LowCmdSender` (không bypass); tuỳ chọn thêm giới hạn tốc độ tay chặt hơn khi đang đi.
5. **Giảm biên theo tốc độ:** khi `cmd_norm` lớn (đi nhanh) → giảm `w`/biên độ gesture-teleop; chỉ cho biên độ đầy đủ khi đứng/đi chậm.
6. **Waist ROLL (idx 12): KHÔNG override** (dịch CoM ngang → dễ lật). **Waist YAW (idx 13): override NHẸ được** với điều kiện: biên độ nhỏ + tốc độ thấp; **mask idx 13 trong obs** (mở rộng mask tay, tuỳ chọn gồm yaw, KHÔNG gồm roll); clamp + slew; đưa vào balance guard. **Phải kiểm chứng vị trí IMU**: nếu IMU trên torso (trên waist) thì xoay yaw bơm `gyro.z` thật → policy tưởng đang rẽ → giữ thật chậm/nhỏ và test heading trong sim. Đầu tách biệt, luôn có đường về 0.

### 7.6 Đầu (head) — thay đổi ở sender
`LowCmdSender::Send` thêm tham số `HeadTarget{valid,yaw,pitch}`; `SetHead` dùng target đó thay 0 khi valid, vẫn clamp + slew, watchdog hết hạn → target về 0. Zero tương tác policy.

### 7.7 Config `tuning.yaml` (thêm)
```
upperbody_enabled: true
gesture_folder: policies/gestures
gesture_blend_in_s: 0.4
gesture_retract_s: 1.2
teleop_enabled: false          # bật khi làm teleop
teleop_udp_port: 5560
teleop_timeout_ms: 300
balance_k_g: <đo trong sim>
balance_k_v: <đo trong sim>
balance_guard_tilt_max: <rad>
balance_guard_gyro_max: <rad/s>
head_yaw_limit / head_pitch_limit: <rad>
override_amp_vs_speed: <bảng giảm biên theo cmd_norm>
```

### 7.8 Transport teleop (đề xuất)
- **UDP** (đơn giản, độ trễ thấp, khớp phong cách `IntegrationNotifier`): operator station → gói nhỏ → `TeleopReceiver`. Lỗi socket không làm sập motor loop.
- Gói có `seq` + `t_send` để phát hiện mất gói/đảo thứ tự; drop gói cũ.
- (Thay thế) DDS topic riêng nếu muốn đồng bộ với phần còn lại — nhưng UDP đủ và nhẹ hơn.

### 7.9 Nghiệm thu (sim trước — robot sau)
1. Sim: chạy thư viện gesture khi đang đi mọi hướng/tốc độ; log tilt/gyro; A/B che-cứng vs che-mềm.
2. Sim: hiệu chỉnh `balance_k_*`, ngưỡng balance guard.
3. Sim: teleop giả lập (phát stream ghi sẵn) — kiểm watchdog, mất gói, fade.
4. Robot: đầu trước (rủi ro thấp) → gesture tay biên nhỏ → tăng dần → teleop tay đứng yên → teleop tay khi đi chậm.

### 7.10 Graceful degradation — chạy bình thường khi CHƯA có npz / CHƯA có teleop

Có thể **build & deploy toàn bộ khung ngay bây giờ**; nó nằm im (dormant) cho tới khi thả npz vào hoặc bật teleop. Locomotion không bị ảnh hưởng — **với điều kiện giữ đúng 4 bất biến sau** (đây là yêu cầu thiết kế bắt buộc, không phải tuỳ chọn):

| Thiếu | Hành vi | Ảnh hưởng locomotion |
|---|---|---|
| Chưa có npz | GesturePlayer không có gì để phát → provider `inactive` → không ghi đè | Không |
| Chưa stream teleop | TeleopReceiver không có gói → watchdog giữ `authority=0` | Không |
| Chưa dùng head teleop | Không `HeadTarget` valid → sender giữ đầu = 0 (như hiện tại) | Không |
| Không nguồn nào active | `ctx.arm_override.active=false` → obs dựng **giống hệt** hôm nay; comp cộng 0 | Không |

**4 bất biến bắt buộc:**
1. `UpperBodyManager::Apply()` **early-return khi không nguồn active** → `ai_target_q_` nguyên vẹn.
2. `ControlContext.arm_override` mặc định inactive → nhánh `BuildObservation` **giống byte** đường hiện tại khi inactive.
3. Tham số `HeadTarget` của `Send` mặc định invalid → `SetHead` giữ 0 (thêm overload/default arg, không đổi call site cũ).
4. Cờ tổng `upperbody_enabled` tắt cứng được toàn bộ.

Đây đúng là pattern codebase **đã dùng**: thiếu file dance/getup/liedown thì chỉ **khoá riêng tính năng đó**, không crash ([Application.cpp:311](../src/app/Application.cpp#L311)).

**Cách kiểm chứng "không hồi quy":** bật `upperbody_enabled=true` nhưng không nguồn → so `target_q` với build hiện tại, phải **trùng khớp** (lý tưởng bit-identical). Rủi ro thật duy nhất không phải "thiếu file", mà là **masking/comp rò rỉ khi lẽ ra inactive** (vd `w` không về đúng 0, hoặc quên gate `active`) → guard `active`/`w==0` phải chặt.

**Test đường ống KHÔNG cần asset:** cắm provider tổng hợp (giữ tư thế hiện tại, hoặc gesture sine giả) để nghiệm thu chuỗi override → blend → mask → comp trong sim trước khi có npz/teleop thật.

---

## 8. Khi gesture mạnh gây LẮC — thang xử lý & khi nào phải train lại

**Vì sao lắc:** policy đang **bị che mắt khỏi tay** (masking) → chỉ thấy nhiễu **sau khi** mô-men phản lực + dịch CoM đã hiện trong IMU → phản ứng **trễ, bị động**. Tay càng **nhanh** thì nhiễu tới nhanh hơn vòng phản ứng → lắc. Mấu chốt: mô-men phản lực tỉ lệ **gia tốc góc** của tay, **không phải biên độ** → **tốc độ** clip là thủ phạm số 1.

**Train lại là bước cuối, không phải bước đầu.** Thang xử lý từ rẻ → đắt:

| Bậc | Biện pháp (KHÔNG train) | Ghi chú |
|---|---|---|
| 1 | **Làm chậm / mượt clip** (giảm tốc phát, low-pass, slew cap tay) | Rẻ nhất; cùng động tác chậm lại **cắt mô-men rất mạnh**. Thường đủ |
| 2 | **Feedforward comp từ lệnh tay** (§7.4) | Bù **trước** khi nhiễu tới IMU → đánh trúng cái "trễ" gây lắc |
| 3 | **Giảm biên theo tốc độ** — gesture mạnh chỉ khi đứng/đi chậm | Lúc đi nhanh chân ít dư địa |
| 4 | **Không override waist ROLL** (waist YAW nhẹ thì được) | Roll dịch CoM ngang → đòn bẩy lật lớn nhất, để cho policy |

**Chỉ BUỘC train** khi một gesture **vẫn lắc ở tốc độ/biên độ bạn chấp nhận được** (không thể làm chậm/nhỏ thêm mà vẫn phải mạnh) → overlay chạm trần cứng.

**Train ĐÚNG cách** (dễ hiểu sai): không phải "cho vững hơn" chung chung, mà cho policy **BIẾT tay sắp làm gì**:

- **(a) Rẻ — DR nhiễu:** giữ masking, train với nhiễu lực/tay ngẫu nhiên mạnh → bền hơn, không đổi interface, nhưng **vẫn bị động**.
- **(b) Đúng — command-conditioned (Tier 3, §9):** đưa target tay vào obs như một lệnh → policy **bù chủ động (feedforward)**. Lệnh tay đã biết ≠ cú đẩy ngẫu nhiên; policy nhìn thấy thì pre-compensate được. → Nếu **biết trước** sự kiện cần nhiều động tác tay mạnh khi đi, **nhắm (b) từ đầu**, đừng ép overlay.

**Reframing quan trọng (đã có sẵn hạ tầng):** codebase đã có **mimic/dance** — dance npz lái **toàn thân kể cả tay** và mimic policy **đã được train để tracking**. Vậy:

- Gesture **nhẹ/vừa** khi đi → **overlay (Tier 2)**.
- Gesture **mạnh/động toàn thân** → nên là **một clip mimic/dance đã train** (đi qua `MimicController`) thay vì overlay. Đây là Tier 3 "cho một choreography cố định" mà bạn **đã có công cụ**.

**Quy trình khuyến nghị:** ship Tier 2 → **đo lắc trên chính thư viện gesture thật** (log tilt/gyro) → gesture nào vượt ngưỡng thì thử bậc 1–4 → không đạt thì chuyển thành **mimic clip** hoặc gom vào đợt **train command-conditioned**. Data "gesture nào phá vỡ" chính là đầu vào quyết định train — **đừng train mù trước khi có nó**.

---

## 9. Kế hoạch Tier 3 (train) — phác thảo cho teleop nặng

Khi teleop tay cần mạnh/động vượt khả năng overlay:
- **Observation/command:** thêm *target tư thế thân trên* vào obs của policy (tay ± đầu), như một lệnh. Policy học **tracking tay + giữ chân**.
- **Reward:** tracking tay theo reference ngẫu nhiên + reward locomotion/ổn định; phạt trượt/ngã.
- **Domain randomization:** biên độ/tốc độ tay ngẫu nhiên, tải ở tay, để chân học bù CoM.
- **Export:** gắn metadata gains vào onnx (PolicyController bắt buộc metadata — xem `hb-high-level-rewrite`).
- **Swap deploy:** provider Tier 2 trở thành **nguồn command** thay vì nguồn override; **tắt che-obs & bù thủ công**. Kiến trúc UpperBodyManager giữ nguyên.

Đây là hướng của các hệ teleop nhân hình hiện đại (whole-body / expressive control). Chỉ nên đầu tư khi overlay Tier 2 đã ổn và nhu cầu teleop vượt giới hạn.

---

## 10. Rủi ro & giảm thiểu

| Rủi ro | Giảm thiểu |
|---|---|
| Tay nặng làm lệch CoM, policy không biết | Bù closed-loop từ lệnh tay + balance guard + giảm biên theo tốc độ; nặng thật → Tier 3 |
| Teleop mất gói giữa động tác | Watchdog → fade về policy; giữ giá trị cuối ngắn hạn |
| Che-obs mềm gây bất ổn | A/B trong sim trước; mặc định che cứng (đã chứng minh ở sim) |
| Va chạm tay–thân/tay–tay khi teleop tự do | Clamp giới hạn khớp; (tuỳ chọn) kiểm tra khoảng cách đơn giản |
| Trùng phím built-in khi thêm trigger | Chọn combo chưa dùng — xem `l2y-builtin-keymap-collision`; rà `run_r1` im trước arm |
| Deploy đè built-in | Giữ cổng arm hiện có; override chỉ khi `armed_ && kLocomotion` |

---

## 11. Bảng hành động ưu tiên

| Ưu tiên | Việc | Vị trí |
|---|---|---|
| P0 | `IUpperBodyProvider` + `UpperBodyManager` (chọn nguồn, blend, safety) | `src/motion/` |
| P0 | `GesturePlayer` (npz) + tiêm vào `RunPolicy` trước `Send` | `src/motion/`, [Application.cpp:1480](../src/app/Application.cpp#L1480) |
| P0 | Che-obs tay + field `ControlContext.arm_override` | `LocomotionController`, `PolicyController` |
| P0 | Balance guard + gating chỉ `kLocomotion` + tự hủy khi ngã | `UpperBodyManager`/`Application` |
| P1 | **Teleop đầu**: sender nhận `HeadTarget` (rủi ro thấp, làm sớm) | `LowCmdSender`, `UpperBodyManager` |
| P1 | Bù thăng bằng closed-loop từ lệnh tay | `LocomotionController`/`Manager` |
| P1 | Map trigger gamepad + config `tuning.yaml` | `input/`, `config/` |
| P2 | `TeleopReceiver` (UDP) + watchdog + nghiệm thu sim | `src/input/` |
| P3 | (Tuỳ chọn) train masked/command policy — Tier 3 | pipeline train |

> Nhắc vận hành: cần bạn nói **"làm đi"** mới bắt đầu sửa code. Đổi mapping nút/safety/tuning phải cập nhật `docs/huong_dan_van_hanh.md` cùng lượt (xem `keep-ops-guide-in-sync`).
