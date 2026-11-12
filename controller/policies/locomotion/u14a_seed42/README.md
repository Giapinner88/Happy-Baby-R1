# u14a_seed42 — U14a: U13a + actuator `vendor_bm` + mirror trái/phải (chỉ chân)

Thêm vào 2026-11-12; chưa chọn cho R2+A/R2+X. `teleop_h4: false` (duyệt H4 là quyết định của người vận hành).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u14a/runs/isaac/u14a_seed42` — `scripts/loco/run.py train --config config/train/r1_u14a.json` (= `r1_u13a.json` + `actuator_profile: vendor_bm` + `symmetry: true`); thiết kế và đánh giá trong `experiments/r1_loco/u14a/experiment.md` |
| Train | Isaac Lab, seed 42, 4096 env, 5000 vòng (2026-11-10 → 11-11), push như U12, dwell 0.5 s như U13a |
| `model_4999.pt`, `policy.onnx`, yaml, json | khớp `artifact_manifest.json` của run (kiểm 2026-11-12); sha256 ONNX trong `policy.yaml` |

## Hợp đồng và khác biệt runtime so với U13a

Contract `r1_wb_legs_gait_h4_v1_blind` như U4/U12/U13a, obs 287 → 12 action; hằng số FSM trong metadata giống U13a.
Metadata ONNX khác U13a ở:

| Khoá | U13a | U14a |
|---|---|---|
| `actuator_profile` | `vendor_hb_pose` | `vendor_bm` |
| `joint_stiffness` hông/gối, eo, vai pitch/roll | 100, 100, 40 | 84.335 |
| `joint_stiffness` cổ chân | 40 | 31.413 |
| `joint_stiffness` vai yaw, khuỷu, cổ tay | 20 | 15.706 |
| `joint_damping` hông/gối, eo, vai pitch/roll | 2 | 5.369 |
| `action_scale` chân / cổ chân | 0.15 / 0.3125 | 0.178 / 0.398 |
| `training_symmetry` | — | `left_right_mirror_data_augmentation` |

Controller đọc kp/kd và action scale từ metadata (`PolicyController`), nên gain `vendor_bm` được gửi ở mọi khớp khi
policy chạy; controller không đổi. Ngoại lệ đã biết: khi overlay teleop cầm tay, tay nhận `teleop_arm_kp` (40) thay vì
gain lúc train; chuyển STAND → policy blend từ gain đứng/return như mọi bundle. Armature trong mô phỏng
(kp / (2π·10 Hz)²) được suy ra từ gain của vendor, không phải từ tài liệu hay nhận dạng.

## Kiểm chứng runtime

`wb_legs_gait_u14a_golden_test` (trace `controller/tests/data/wb_legs_gait_u14a_golden_trace.txt`, sinh bằng
`controller/scripts/make_wb_legs_gait_golden.py` với `loco/r1/gait_fsm.py` trùng sha256 lúc train): 130 bước + Reset.

## Bằng chứng hành vi (mô phỏng, một seed, 2026-11-11 — `experiment.md` của U14a)

| | U13a | U14a | tiêu chí |
|---|---|---|---|
| Dừng: tỉ lệ STAND trong thời gian dừng | 88 % | 88 % | ≥ 83 %: đạt |
| Dừng: median nghiêng W2S / STAND | 2.1 / 1.0° | 2.0 / 0.8° | W2S ≤ 3°: đạt |
| Dừng: median lệch gối / hông pitch L-R | 0.028 / 0.045 rad | 0.023 / 0.040 | ≤ 0.029 / 0.042: đạt |
| Đứng (FK, 1280 lần dừng): pelvis y so với hai bàn chân | 0.8 cm lệch phải | 0.1 cm lệch trái | |
| Xoay trái/phải: cả hai chân nhấc | có | có | đạt |
| Xoay: chiều cao nhấc chân | 1.9–2.7 cm | 2.0–2.2 cm | cao hơn U13a: **không đạt** |
| Đẩy 1.0 (1024 env): ngã, hồi phục trong 2 s | 1.66 %, 91 % | 0.49 %, 97 % | ≤ 1.7 %: đạt |
| Đẩy 1.0: RMSE tuyến tính / yaw | 0.230 / 0.238 | 0.221 / 0.169 | |

Mô phỏng: hết lệch tư thế đứng sang phải của U13a, êm hơn sau khi vào STAND và khi bị đẩy. Không tăng chiều cao nhấc chân,
nên lỗi xoay phải chỉ trụ chân phải trên robot chưa chắc được giải quyết. Hai thay đổi (actuator, mirror) không tách được
tác dụng. **Chưa chạy trên robot**; lần đầu phải theo `docs/safety.md` (giá treo, E-stop) và so log với log vendor theo
protocol trong `experiment.md`.
