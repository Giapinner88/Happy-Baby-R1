# PLAN — Nối XR teleop (`xr_teleoperate`) vào R1 qua run_r1, an toàn & không xung đột

> Bối cảnh: đã có teleop **robot-side** trong run_r1 (`TeleopReceiver` UDP + overlay tay lên policy đi bộ + L2+Phải arm/disarm). Cần thêm **operator-side**: XR → góc khớp tay R1 → gửi vào run_r1.
> Nguồn operator: `Mujoco/xr_teleoperate-main` (Unitree XR teleop chính hãng).
> Trạng thái: **ĐÃ CHỐT KHÔNG ĐI ĐƯỜNG NÀY** (2026-07-29) — chọn **Phương án C** (WebXR bridge tự viết trên Jetson, bỏ xr_teleoperate): xem `PLAN_teleop_webxr_bridge.md`. File này giữ làm phân tích nền (lý do loại xr_teleoperate + các khoảng cách dữ liệu).

---

## 0. Vì sao TRƯỚC ĐÂY tôi không nhắc "xung đột" khi làm phương pháp ghi đè?

Vì **phương pháp ghi đè của tôi không tạo ra xung đột** — nó single-writer **theo thiết kế**:

- Teleop/gesture của tôi đi vào run_r1 dưới dạng **dữ liệu UDP**, run_r1 sửa `target_q` rồi gọi **một** `LowCmdSender::Send` → **run_r1 là NGUỒN GHI `rt/lowcmd` DUY NHẤT**. Không có tiến trình thứ 2 ghi lowcmd → không có "2 nguồn đè nhau" như [[arm-conflict-false-positive-startup-window]] / [[r1-two-pc-architecture]].
- Xung đột **chỉ xuất hiện BÂY GIỜ** vì `xr_teleoperate` là **một chương trình riêng tự publish `rt/lowcmd`/`rt/arm_sdk`** → nếu chạy song song run_r1 sẽ thành **2 nguồn ghi lowcmd**. Đây là mô hình của G1 (arm_sdk blend trong firmware), **không hợp** kiến trúc single-writer của R1/run_r1.

**Nhận trách nhiệm:** lúc chọn UDP-vào-run_r1 tôi đã chọn đúng hướng single-writer, nhưng chỉ giải thích là "đơn giản/độ trễ thấp" — **chưa nói rõ lý do cốt lõi**: giữ run_r1 làm nguồn ghi duy nhất để né đúng cái xung đột 2-nguồn bạn từng gặp. Giờ khi kéo `xr_teleoperate` (một writer ngoài) vào, điều đó thành trung tâm.

---

## 1. Kiến trúc mục tiêu (BẮT BUỘC single-writer)

```
Kính XR (AVP/PICO)
   │  tư thế cổ tay 6-DoF (+đầu)
   ▼
xr_teleoperate: TeleVuer bắt pose  →  R1_ArmIK (pinocchio) → sol_q (góc khớp tay R1)
   │
   │  ❌ KHÔNG dùng arm_ctrl.ctrl_dual_arm() (nó publish rt/lowcmd/rt/arm_sdk — CẤM)
   │  ✅ THAY bằng ADAPTER:
   ▼
ADAPTER: sol_q(5/tay) + đầu  →  gói UDP 'UTL1' (60B)  ──►  run_r1 TeleopReceiver
                                                              │
                                          overlay lên policy đi bộ + mask obs + safety
                                                              ▼
                                              MỘT nguồn rt/lowcmd  ──►  robot
```

**Nguyên tắc bất di:** `xr_teleoperate` **chỉ tính toán**, **không chạm robot**. Mọi lệnh xuống motor đi qua **một** `LowCmdSender` của run_r1.

---

## 2. Sự thật đã xác định (từ code)

| Hạng mục | Sự thật |
|---|---|
| **Biến thể R1 deploy** | `r1.xml` (26 khớp) = **tay 5-DoF/bên**: `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll`. Khớp policy 14..18 (trái) / 19..23 (phải). Trùng `default_q` tay `[0.35,0.18,0,0.87,0]`. **KHÔNG có wrist_pitch/yaw, KHÔNG có bàn tay.** |
| Biến thể khác | `r1_with_hands.xml` (44 khớp) = tay **7-DoF** + bàn tay khéo 7 khớp/bên. **Khác** biến thể deploy. |
| Dấu khớp đã kiểm chứng | `shoulder_pitch ÂM = tay ra trước` (PLAN_sit_balanced.md). policy idx 17 = elbow. |
| Đầu | run_r1 điều khiển đầu idl 29/30 (yaw/pitch) tách rời policy. **Nhưng `r1.xml`/`r1_with_hands.xml` KHÔNG có khớp đầu** → cần xác nhận đầu thật. |
| xr_teleoperate IK | pinocchio + casadi, nạp **URDF**, build reduced-robot (khoá khớp không phải tay), IK từ **pose cổ tay 6-DoF** → `sol_q` (7/tay cho G1). Output publish DDS. Chỉ hỗ trợ **G1/H1/H2 — KHÔNG có R1**. |
| Định dạng UDP đích | Gói `TeleopReceiver::Packet` 60B, magic `UTL1`: magic,seq,enable,arm_valid,head_valid,pad, arm_q[10], head_yaw, head_pitch. Đã có `scripts/teleop_send_test.py`. |

**Hệ quả quan trọng:** tay deploy **5-DoF** (cổ tay chỉ có roll) → teleop đạt **vị trí + hướng MỘT PHẦN**, **không** xoay cổ tay đầy đủ 6-DoF. Không có teleop bàn tay (biến thể deploy không có). Đây là **giới hạn phần cứng**, không phải thiếu sót phần mềm.

---

## 3. ⚠ KHOẢNG CÁCH DỮ LIỆU — cần bạn cung cấp / xác nhận

| # | Cần gì | Vì sao | Không có thì |
|---|---|---|---|
| **G1** | **Robot đích là biến thể NÀO?** 5-DoF (`r1.xml`, không tay) hay 7-DoF+bàn tay (`r1_with_hands.xml`)? | Quyết định toàn bộ scope: 5-DoF thì teleop = tay 5 khớp, không bàn tay; 7-DoF+tay thì phải **mở rộng run_r1** điều khiển thêm wrist_pitch/yaw + ngón (lớn hơn nhiều, ngoài policy). | Mặc định giả sử **5-DoF** (khớp deploy hiện tại). |
| **G2** ⛔ | **URDF của R1** (biến thể 5-DoF) cho pinocchio IK. Ta chỉ có **MJCF** (`r1.xml`). | xr_teleoperate IK cần URDF. **Đây là blocker chính của đường VR.** | Tôi **tự dựng URDF tối giản chuỗi tay 5-DoF** từ `r1.xml` (chỉ cần chuỗi động học tay cho IK, không cần mesh) — không phụ thuộc URDF hãng. |
| **G3** | **Khớp ĐẦU thật**: R1 có head yaw/pitch (idl 29/30) không? Dải góc? | XML không có đầu; run_r1 giả định có. Teleop đầu cần biết dải + tồn tại. | Tạm **tắt teleop đầu**, chỉ teleop tay. |
| **G4** ✅ | ~~Thiết bị XR~~ **CÓ Meta Quest 3** — xr_teleoperate hỗ trợ chính thức (Device.md). | Đã đủ phần cứng. | — |
| **G4b** | Setup **televuer** (submodule chưa cài) + SSL cert + mạng `192.168.123.x` + cổng 8012 + mở trình duyệt WebXR trên Quest. | televuer là đường lấy hand-tracking từ Quest qua WebXR. Hiện `televuer installed: False`. | Làm theo README (việc môi trường của bạn; tôi hướng dẫn được). |
| **G5** | **Chuẩn dấu/zero từng khớp tay** giữa XR-retarget và deploy. | Sai dấu = tay đi ngược. Mới có 1 mốc (shoulder_pitch âm=trước). | Sẽ **hiệu chỉnh trong sim** (mục 6) — nhưng cần bạn duyệt khi lên robot. |
| G6 | (Tùy) Bảng **IDL 35-motor → tên khớp** chính thức của R1. | Tôi đã suy ra 5 khớp tay; bảng chính thức để chắc. | Dùng suy luận hiện tại (đủ tin cho 5-DoF). |

---

## 4. Kế hoạch thực hiện (giả định biến thể 5-DoF; điều chỉnh nếu G1 khác)

### Bước A — URDF cho R1 (khoá G2)
- Ưu tiên: dùng URDF hãng nếu bạn có. Nếu không: convert `r1.xml` → URDF (mujoco→urdf), kiểm định khớp/tên/dải trong meshcat.
- Kết quả: URDF tay 5-DoF/bên + frame `L_ee`/`R_ee` tại `*_wrist_roll_link`.

### Bước B — `R1_ArmIK` (bắt chước `G1_29_ArmIK`)
- Reduced-robot: khoá chân/waist/(bàn tay nếu có), giữ 5 khớp tay/bên.
- IK từ pose cổ tay trái/phải → `sol_q` (10 = 5+5), thứ tự khớp **khớp policy 14..23**.
- Vì 5-DoF: IK bám **vị trí + hướng một phần** (weight vị trí cao, hướng thấp) để không giật.
- Giới hạn khớp = dải trong URDF/RobotSpec; velocity clip sẵn ở TeleopReceiver + sender slew.

### Bước C — ADAPTER (thay chỗ publish DDS)
- Trong vòng lặp `teleop_hand_and_arm.py`: chỗ `arm_ctrl.ctrl_dual_arm(sol_q, ...)` (dòng ~368) **thay bằng** `udp_send_utl1(sol_q_10, head_yaw, head_pitch, enable)`.
- ~40 dòng: đóng gói struct `UTL1` (dùng lại layout `teleop_send_test.py`), gửi tới `run_r1_host:5560`.
- `enable` map theo nút "bắt đầu/nhả" của xr_teleoperate (hoặc luôn 1; run_r1 vẫn cần L2+Phải arm).
- **Xoá/không khởi tạo** `G1_29_ArmController` (để nó KHÔNG publish DDS).

### Bước D — run_r1 side (ĐÃ CÓ)
- `TeleopReceiver` + overlay + mask + balance guard + L2+Phải + watchdog: xong.
- Chỉ cần **xác nhận 5 khớp tay** đúng thứ tự (đã suy ra: sh_pitch, sh_roll, sh_yaw, elbow, wrist_roll).

### Bước E — Đầu & bàn tay (tùy G1/G3)
- Đầu (nếu G3 xác nhận có): adapter gửi thêm head_yaw/pitch từ XR head pose → đã có đường trong run_r1.
- Bàn tay: **chỉ nếu biến thể 7-DoF+hands** → cần mở rộng run_r1 ghi thêm khớp ngón (đường riêng ngoài policy). Ngoài scope 5-DoF.

---

## 5. An toàn / chống xung đột (checklist BẮT BUỘC)

1. **`xr_teleoperate` KHÔNG được publish `rt/lowcmd`/`rt/arm_sdk`.** Gỡ/không init `*_ArmController`. Chỉ chạy IK + adapter UDP. (Vi phạm = 2 nguồn ghi → loạn.)
2. **run_r1 là nguồn ghi duy nhất.** Giữ nguyên.
3. **L2+Phải arm/disarm** trên robot = quyền tối hậu của người vận hành; mặc định TẮT. Là công tắc ngắt nếu XR/adapter lỗi.
4. **Watchdog** teleop (mất gói > timeout) → tay tự nhả về policy. Adapter nên gửi đều (≥30Hz).
5. **Chỉ chạy khi LOCOMOTION**; balance guard nghiêng/gyro → tự nhả (đã có).
6. **Clamp** góc tay theo dải khớp + slew (đã có). Adapter cũng nên clip theo velocity như xr_teleoperate.
7. **Deploy KHÔNG tự động** (nguyên tắc [[no-auto-deploy-to-robot]]): bạn build trên Jetson + đẩy.

---

## 6. Lộ trình test (sim trước — robot sau)

1. **Sim mujoco `run_policy`** (biến thể `r1.xml`, đang Flat, bấm **T** arm teleop) + adapter chạy IK từ pose GIẢ (hoặc XR) → xem tay bám, đo tilt/gyro (lắc?).
2. Hiệu chỉnh dấu/zero (G5), giới hạn, `teleop_smooth_hz`, weight IK hướng.
3. Nếu có XR: chạy full xr_teleoperate + adapter vào sim.
4. Robot: đầu/tay biên nhỏ, đứng yên → đi chậm → tăng dần. `teleop_enabled:true`, L2+Phải khi sẵn sàng.

---

## 7. Tôi có thể làm ngay vs cần chờ dữ liệu

**Làm ngay (không cần thêm gì):**
- ADAPTER `sol_q → UDP UTL1` (Bước C) — đã có format.
- Bộ khung `R1_ArmIK` (Bước B) — nhưng cần URDF (G2) để chạy thật.
- Hạ tầng run_r1 (Bước D) — xong.

**Chờ bạn (mục 3):** G1 (biến thể robot), G2 (URDF), G3 (đầu), G4 (có XR không), G5 (duyệt hiệu chỉnh khi lên robot).

> Khuyến nghị: xác nhận **G1 + G2 + G4** trước. Nếu chưa có XR (G4), ta vẫn teleop được ngay bằng operator đơn giản (tay cầm 2 / leader-arm) → cùng UDP, không cần `xr_teleoperate`.
