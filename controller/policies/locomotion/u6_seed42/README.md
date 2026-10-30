# u6_seed42 — U6: U4 + chân cứng hơn, dải độ rộng chân, dừng mượt, căn chân, lệch IMU, giữ tư thế tay

Thêm vào 2026-10-30; `teleop_policy` (R2+X) trong `locomotion.yaml` do người vận hành chọn.
Cùng contract và runtime với U4 — xem [`../u4_seed42/README.md`](../u4_seed42/README.md).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u6/runs/isaac/u6_seed42` — lệnh đầy đủ trong `provenance.json`; thiết kế trong `experiments/r1_loco/u6/experiment.md` |
| Môi trường | Isaac Lab, seed 42; 5000 vòng, 4096 env; actuator profile **`vendor_hb_pose_stiff`** |
| `model_4999.pt`, `policy.onnx` | khớp `artifact_manifest.json` của run (kiểm 2026-10-30); sha256 ONNX trong `policy.yaml` |

## Hợp đồng

`r1_wb_legs_gait_h4_v1_blind`, obs 287 → 12 action; FSM, history, default pose giống U4. **Khác U4 ở PD và action scale**
(runtime đọc từ metadata ONNX, không giới hạn kp):

| | hông/gối kp | cổ chân kp | kd chân | action scale hông/gối | action scale cổ chân |
|---|---|---|---|---|---|
| U4/U5 (`vendor_hb_pose`) | 100 | 40 | 2 | 0.15 | 0.312 |
| **U6** (`vendor_hb_pose_stiff`) | **130** | **60** | 2 | **0.115** | **0.208** |

Reward/randomization thêm (`stop_upright`, orientation −5, `stop_smoothness`, độ rộng chân 0.24–0.28 m, `foot_alignment`,
lệch IMU 3°, giữ tư thế tay 30 %) chỉ có lúc train, không đổi input lúc chạy.

## Kiểm chứng runtime

`wb_legs_gait_golden_test` chạy lại trên model U6 (trace từ `controller/scripts/make_wb_legs_gait_golden.py` với
`policy.onnx` này): PASS 130 bước + replay sau Reset; runtime nạp Kp hông 130, cổ chân 60. Preflight đạt với
armfree (R2+A) + U6 (R2+X).

## Bằng chứng hành vi

- Log train, bước 4999 (số liệu lúc train, không phải đánh giá):

  | | STAND | WALK | W2S | `error_vel_xy` | episode không ngã |
  |---|---|---|---|---|---|
  | U4 | 6.8 % | 84.4 % | 8.7 % | 0.630 | 99.4 % |
  | U5 | **0.1 %** | 83.7 % | 15.9 % | 0.599 | 99.3 % |
  | **U6** | 6.1 % | 82.3 % | 9.5 % | **0.893** | **97.9 %** |

  U6 vào STAND trở lại như U4 (U5 gần như không). Sai số bám vận tốc lúc train cao hơn U4 khoảng 40 % và tỉ lệ episode
  ngã khoảng 3.5 lần U4 (2.1 % so với 0.6 %).
- **Chưa có đánh giá nào**: `experiment.md` của U6 còn ghi "running", chưa có mục kết quả. Chưa chạy trên robot.
