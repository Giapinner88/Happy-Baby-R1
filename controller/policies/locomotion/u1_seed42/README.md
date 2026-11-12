# u1_seed42 — U1 whole-body legs policy (teleop locomotion)

Dùng làm `teleop_policy` (R2+X: đi bộ + teleop tay). Policy điều khiển **12 khớp chân**; eo giữ
`default_joint_pos`, tay nhận target teleop H4 trực tiếp (hoặc giữ default khi teleop chưa bật).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco`, commit `3b6be38bf7ca051d39e6faa1ec8c80302eed0b1d` (cây làm việc có thay đổi chưa commit, xem `provenance.json` của run) |
| Run | `experiments/r1_loco/u1/runs/isaac/u1_seed42` — `scripts/loco/run.py train --config config/train/r1_u1.json` |
| Môi trường | Isaac Sim 5.1.0-rc.19, Isaac Lab v2.3.2, rsl-rl-lib 3.1.2, torch 2.7.0; seed 42; 10001 vòng; 4096 env |
| `checkpoint.pt` | `model_10000.pt`, sha256 `0ffa91c614102ceb908e859e6da1a632b877024999275f503edcc353bf61a86c` (khớp `artifact_manifest.json` của run) |
| `policy.onnx` | export của run (normalizer quan sát gói trong ONNX), sha256 `9cdf005ef7a7842b630744f298d992ad370ee2320f99d69691fc2db86b9e5fa5` |

## Hợp đồng (`r1_wb_legs_v1_blind`)

- Input `obs[1,71]`: `base_ang_vel(3) projected_gravity(3) command(3) phase(2) joint_pos(24) joint_vel(24) actions(12)`, không scale;
  `joint_pos` = q − default, `actions` = raw action bước trước; `phase` = sin/cos(2πt/0.6), bằng 0 khi ‖(vx,vy,yaw)‖ < 0.1.
- Output `actions[1,12]`: target chân = default + action × action_scale (0.15 hông/gối, 0.312 cổ chân).
- PD: metadata `joint_stiffness` / `joint_damping` (chân 100/2, cổ chân 40/2, tay 40/2 và 20/1).
- `default_joint_pos` chân khác policy walk `flat_plus_gait_0917_armfree` tối đa 0.048 rad (hip roll/yaw, ankle roll = 0 ở U1);
  tay và eo trùng. run_r1 cho phép lệch chân ≤ 0.10 rad và blend `blend_time_s` khi đổi chế độ.
- Runtime: `controller/src/policy/WholeBodyLegsController.hpp`.

## Bằng chứng hiện có (chỉ mô phỏng Isaac, một seed; nguồn `experiments/r1_loco/u1/experiment.md`)

- Đi bộ: sai số vận tốc trung bình 0.276 m/s (E1, tay chuyển động) — **không đạt** tiêu chí ≤ 0.15; gần W1 (0.253).
- Đẩy khi đứng (1024 env): ngã 0.2 / 0.7 / 10.8 % ở 0.5 / 1.0 / 1.5 m/s — tốt nhất trong các policy đã thử.
- Rung khi đứng (leg-target-rate RMS) 0.42–0.44 rad/s — **không đạt** (≤ 0.36).
- Tốc độ tiến dùng được ~1.1 m/s; lệnh 1.6 m/s chỉ đạt 0.74, lệnh 1.9 m/s robot đứng yên. `fast_vx: 1.5` trong
  `locomotion.yaml` nằm ngoài dải này.
- Chưa có: chẩn đoán rung (mục 4 của protocol), sim2sim MuJoCo, chạy trên robot, đánh giá với target tay từ Quest thật.
