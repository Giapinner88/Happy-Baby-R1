# u5_seed42 — U5: U4 + căn chân khi dừng, lệch IMU, vào STAND mượt, nhấc chân cao hơn

Thêm vào 2026-10-29; **chưa chọn** trong `locomotion.yaml`, `teleop_h4: false` (chưa duyệt overlay tay Quest).
Cùng contract và runtime với U4 — xem [`../u4_seed42/README.md`](../u4_seed42/README.md).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u5/runs/isaac/u5_seed42` — `scripts/loco/run.py train` (lệnh đầy đủ trong `provenance.json`) |
| Môi trường | Isaac Lab, seed 42; **6500 vòng, 6144 env** (U4: 5000, 4096); `vendor_hb_pose`; không random độ lỏng khớp |
| `model_6499.pt` | checkpoint cuối, khớp `artifact_manifest.json` |
| `policy.onnx` | sha256 trong `policy.yaml`, khớp `artifact_manifest.json` |

## Hợp đồng

`r1_wb_legs_gait_h4_v1_blind`, obs 287 → 12 action; metadata FSM, history, PD, default pose và action scale giống hệt U4.
Các thay đổi của U5 (`foot_alignment`, `gravity_bias_deg` 3°, `stand_entry_smoothness`, clearance −0.3,
`arm_hold_fraction` 0.3) chỉ là reward/randomization lúc train, không đổi input lúc chạy.

## Kiểm chứng runtime

`wb_legs_gait_golden_test` chạy lại trên model U5 (trace sinh bằng `controller/scripts/make_wb_legs_gait_golden.py`
với `policy.onnx` này): PASS 130 bước + replay sau Reset — mode FSM, obs 287-D, action và target khớp.

## Bằng chứng hành vi

- **Chưa có đánh giá nào.** `experiments/r1_loco/u5/experiment.md` còn ghi "running", chưa có mục kết quả; protocol
  (E1/E2/E3, push, jitter, `stop_after_walk`, push với lệch IMU 3°) đã cố định trước dữ liệu nhưng chưa chạy.
- Lỗi lùi rồi dừng (nghiêng ~10°, dậm chân 2.5–3 s) thấy trên phần cứng **không tái hiện được trong mô phỏng**, nên U5 không
  thể chứng minh đã sửa lỗi đó. Lệch IMU 3° là biện pháp phòng theo giả thuyết.
- Chưa có: sim2sim qua runtime C++.

## Quan sát trên robot (2026-10-29, journal `hb_high_level` 21:43–21:49, R2+X = U5, R2+A = armfree)

- Đứng yên (lệnh 0, |gyro| 0.01–0.05 rad/s): **pitch +6.3 … +9.9°** với U5, so với +0.5 … +1.6° với armfree cùng phiên,
  cùng IMU. Vượt ngưỡng vào STAND (5°) và thường vượt ngưỡng push exit (8°) của chính FSM U5, nên trên robot U5 đứng yên ở
  **W2S**, gần như không vào STAND — trạng thái hiếm gặp lúc train. Chưa rõ nghiêng là do policy hay do W2S kéo dài (hai
  khả năng chưa tách được); cần đo trong mô phỏng tư thế đứng của U5 ở chế độ W2S và STAND.
- Hệ quả runtime: đổi R2+X → R2+A hết 6 s rồi hủy mỗi lần (cổng đổi đòi STAND). Đã sửa 2026-10-29: với U4/U5 cổng chỉ
  đòi bộ lọc ổn định của FSM, không đòi STAND (`application_loco_mode_test`, kịch bản nghiêng 9°).
- Gesture handover cũng chờ nghiêng ≤ `gesture_handover_tilt_rad` 0.08 rad (4.6°), nên với tư thế này gesture có thể không
  nhả tay.
