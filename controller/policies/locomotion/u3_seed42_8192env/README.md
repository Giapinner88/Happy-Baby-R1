# u3_seed42_8192env — U3 whole-body legs policy (stance rộng, khớp chân lỏng)

Thêm vào 2026-10-28. Người vận hành chọn làm `teleop_policy` (R2+X) và **duyệt `teleop_h4: true`** cùng ngày
(overlay tay Quest H4); chưa có bằng chứng nào cho U3 với target tay Quest thật — chỉ mô phỏng với tay theo tham chiếu ngẫu nhiên. Cùng contract với U1/U2a (`r1_wb_legs_v1_blind`):
policy điều khiển **12 khớp chân**; eo giữ `default_joint_pos`, tay thuộc teleop H4 / default.

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco`, commit `3b6be38bf7ca051d39e6faa1ec8c80302eed0b1d` (cây làm việc có thay đổi chưa commit; `provenance.json` ghi sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u3/runs/isaac/u3_seed42_8192env` — `scripts/loco/run.py train --config config/train/r1_u3_8192.json` |
| Môi trường | Isaac Sim 5.1.0-rc.19, Isaac Lab v2.3.2, rsl-rl-lib 3.1.2, torch 2.7.0; seed 42; 5000 vòng; **8192 env**; actuator profile `vendor_hb_pose` |
| `model_4999.pt` | sha256 `8203f0d15ec2c665aa527bfddf720a99b7c5fba3dd00e49cc1697eec4246ce48` |
| `policy.onnx` | export của run, sha256 `35f06a0bdd838879e1ebe291327f8f7aa1d1b058fc08d10efa3d1147a9e211eb` |

Mọi file chép sang đã kiểm khớp `artifact_manifest.json` của run (2026-10-28).

## Hợp đồng

Giống U2a: obs `[1,71]`, action `[1,12]`, `gait_period_s` 0.6, cùng HB pose, PD và action scale
(xem [`../u1_seed42/README.md`](../u1_seed42/README.md)). Runtime: `WholeBodyLegsController`.
Dải lệnh lúc train: vx −0.5…1.0, vy ±0.5, yaw ±1.0 (heading control). `slow_vy` 0.6 và `slow_vx_back` 0.6
của `locomotion.yaml` vượt dải này 0.1 m/s; toàn bộ dải `fast_*` nằm ngoài.

## Khác U2a

Thêm phạt cứng độ rộng chân ≥ 0.24 m lúc chạm đất / hai chân chạm; random độ cứng chân 0.6–1.2 và damping
0.6–1.4 (U2a 0.9–1.1); 8192 env. Chi tiết: `experiments/r1_loco/u3/experiment.md` trong nhánh loco.

## Bằng chứng hiện có (chỉ mô phỏng Isaac, một seed, đánh giá 2026-10-27)

- Độ rộng lúc chạm đất sau đẩy 1.0 m/s: trung bình 0.301 m, 1.5 % dưới 0.20 m (U2a 37.5 %); đứng yên 0.251 m (U2a 0.211).
- Ngã khi đẩy 0.5 / 1.0 / 1.5 m/s: 0.3 / 0.8 / 12.7 % (U2a 0.4 / 2.4 / 24.2 %).
- Chân kp × 0.6: ngã 0.2 % (U2a 36.3 %) — **nằm trong dải đã random lúc train**, không phải bằng chứng tổng quát hoá.
- Đi bộ: E1 0.283 m/s — **không đạt** tiêu chí ≤ 0.27 và kém nhất trong W1/U1/U2a/U3.
- Rung do nhiễu quan sát không giảm (1.66 rad/s, U2a 1.61).
- Chưa có: sim2sim MuJoCo qua obs builder triển khai, chạy trên robot. Không biết U3 có sửa được độ lỏng đã thấy trên phần cứng với U2a không.
