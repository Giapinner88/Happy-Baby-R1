# u2a_seed42 — U2a whole-body legs policy, HB default pose (teleop locomotion)

Dùng làm `teleop_policy` (R2+X), thay `u1_seed42` / `s2_seed42` (2026-10-16, theo yêu cầu của người vận hành). Cùng contract với U1:
policy điều khiển **12 khớp chân**; eo giữ `default_joint_pos`, tay nhận target teleop H4 trực tiếp.

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco`, commit `3b6be38bf7ca051d39e6faa1ec8c80302eed0b1d` (xem `provenance.json` của run) |
| Run | `experiments/r1_loco/u2/runs/isaac/u2a_seed42` — `scripts/loco/run.py train --config config/train/r1_u2a.json` |
| Môi trường | Isaac Sim, Isaac Lab, seed 42; 5000 vòng; 4096 env; objective `walk`; actuator profile `vendor_hb_pose` |
| `checkpoint.pt` | `model_4999.pt`, sha256 `10dc8842…` xem `sha256sum` (khớp `artifact_manifest.json` của run) |
| `policy.onnx` | export của run, sha256 `a57b64b4b8f299e392174ffc89aa300f54cde642e3fd577f5e004488a6eeccc8` |

## Hợp đồng (`r1_wb_legs_v1_blind`)

Giống U1 (obs `[1,71]`, actions `[1,12]`, PD, `gait_period_s` 0.6): xem [`../u1_seed42/README.md`](../u1_seed42/README.md).
Khác U1: `default_joint_pos` chân mang **HB pose** (hip roll ±0.035, hip yaw ∓0.048, ankle roll ∓0.035 rad), cùng pose với
`flat_plus_gait_0917_armfree`; lệch so với U1 tối đa 0.048 rad, nằm trong ngưỡng 0.10 của run_r1. Tay và eo trùng.

## Bằng chứng hiện có

- Là ablation U2a của U2 (U1 + HB pose + dải đi bộ, không thêm walk-quality, không smoothness). Câu hỏi: HB pose có sửa được stance hẹp mà không gây
  dao động cổ chân khi đứng như U2 không (`experiments/r1_loco/u2/experiment.md`).
- **Chưa có số đo nào cho run này.** Các lần đánh giá E1/E2/E3 và push 1.0 thất bại (Isaac error) hoặc chưa xong; experiment.md còn ghi run "running".
  Không biết rung khi đứng, sai số đi bộ, hồi phục sau đẩy.
- Tham chiếu U2 (cùng HB pose, thêm 5 thay đổi khác): đứng rung 0.708 rad/s (U1 0.105), hồi phục 36 % ở push 1.0 — đây là rủi ro cần kiểm với U2a.
- Chưa có: sim2sim MuJoCo qua obs builder triển khai, chạy trên robot.
