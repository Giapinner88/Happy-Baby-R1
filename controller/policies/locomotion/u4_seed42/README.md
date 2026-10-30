# u4_seed42 — U4 whole-body legs policy với gait FSM + history 4 bước

Thêm vào 2026-10-28. Cùng ngày người vận hành chọn làm `teleop_policy` (R2+X, thay U3) và **duyệt `teleop_h4: true`**,
để thử trên robot; khi duyệt chưa có đánh giá mô phỏng nào (xem *Bằng chứng hành vi*). Policy điều khiển **12 khớp chân**; eo giữ
`default_joint_pos`, tay thuộc teleop H4 / default (contract blind: obs tay không bị mask).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco`, commit `3b6be38bf7ca051d39e6faa1ec8c80302eed0b1d` (cây làm việc có thay đổi chưa commit; `provenance.json` ghi sha256 từng file nguồn — `loco/r1/gait_fsm.py`, `isaac/gait.py`, `isaac/env_cfg.py` khớp cây hiện tại ngày 2026-10-28) |
| Run | `experiments/r1_loco/u4/runs/isaac/u4_seed42` — `scripts/loco/run.py train --config config/train/r1_u4.json` |
| Môi trường | Isaac Sim 5.1.0-rc.19, Isaac Lab v2.3.2, rsl-rl-lib 3.1.2, torch 2.7.0; seed 42; 5000 vòng; 4096 env; `vendor_hb_pose`; **không** random độ lỏng khớp |
| `model_4999.pt` | sha256 `e706ea4125f883e68fed957c11db96c741e247ca4102a307f0b62695b847f819` |
| `policy.onnx` | export của run, sha256 `23021a5fd4e636e6d5f0b17f900cd74c6bbbbe5fdd2523b6df4ccffbe1382cb9` |

Mọi file chép sang đã kiểm khớp `artifact_manifest.json` của run (2026-10-28).

## Hợp đồng (`r1_wb_legs_gait_h4_v1_blind`)

- Input `obs[1,287]` = history 4 bước của frame 71-D (`base_ang_vel, projected_gravity, command, phase,
  joint_pos[24], joint_vel[24], actions[12]` — giống U1/U3), **term-major**, cũ → mới, bước đầu lặp lại sau reset;
  rồi one-hot `STAND, WALK, W2S` (3) nối một lần, không nhiễu, không stack. Normalizer nằm trong ONNX.
- Output `actions[1,12]`, PD, action scale, default pose, phase: như U3 (`vendor_hb_pose`, `gait_period_s` 0.6).
- FSM (`loco/r1/gait_fsm.py`): stop khi ‖v‖ < 0.10 và |yaw| < 0.10, move khi > 0.15; W2S → STAND sau khi dừng và
  ổn định **1.5 s** (gyro lọc < 0.25 rad/s, RMS vận tốc **12 khớp chân** lọc < 0.6 rad/s, τ = 0.2 s, nghiêng ≤ 5°);
  STAND → W2S khi bị đẩy (nghiêng > 8° hoặc gyro > 0.75 ngay; > 6.5° hoặc > 0.5 trong 0.3 s).
  Runtime đọc mọi hằng số từ metadata ONNX, kể cả dwell 1.5 s (**không** dùng `gait_settle_dwell_s` của `locomotion.yaml`).
- Runtime: `controller/src/policy/WholeBodyLegsGaitController.hpp` (FSM `GaitModeScheduler`, legs12 + upright + push exit).
- Dải lệnh lúc train: vx −1.0…1.2, vy ±0.6, yaw ±1.0; lệnh trong vùng 0.10–0.15 bị đẩy ra ngoài lúc train (FSM giữ
  mode cũ trong vùng này). `slow_*` nằm trong dải; `fast_vx` 1.5, `fast_vy` 0.8, `fast_yaw` 1.5 nằm ngoài.

## Kiểm chứng runtime (không phải bằng chứng hành vi)

`wb_legs_gait_golden_test` (dữ liệu từ `controller/scripts/make_wb_legs_gait_golden.py`, chạy chính `gait_fsm.py`
của nhánh loco và một history Isaac Lab viết độc lập): 130 bước đi qua STAND → WALK → W2S → STAND (sau đúng
75 bước = 1.5 s) → W2S do đẩy → vùng hysteresis, rồi Reset; khớp mode, obs 287-D (1e-6; block action 1e-4),
action và 24 target. Test bắt được hai lỗi cố ý (đảo thứ tự history; dwell 0.5 s). Preflight đạt với
U4 = `walk_policy` + U3 = `teleop_policy`, và với `flat_plus_gait_0917_armfree` + U4 (cấu hình đang chọn).

Khác biệt đã biết so với lúc train: FSM lúc train đọc vận tốc góc/vận tốc khớp **thật** của simulator, runtime đọc
`gyro_raw` / `dq_raw` của cảm biến (có nhiễu); phase là đồng hồ episode như U3.

## Bằng chứng hành vi

- Log train (`events.out.tfevents`, bước 4999): STAND 6.8 % / WALK 84 % / W2S 8.8 % thời gian, 2.1 lần chuyển mode mỗi
  episode, 99.4 % episode hết giờ (không ngã); `error_vel_xy` 0.63 (U2a 0.628, U3 0.69 cùng metric). Đây là số liệu lúc train,
  không phải đánh giá.
- **Chưa có đánh giá nào.** `experiment.md` của U4 còn ghi "running"; run đã xong (`status.json` completed) nhưng chưa
  chạy E1/E2/E3, push, jitter, `gait_mode_fraction` khi đánh giá.
- Chưa có: sim2sim MuJoCo qua runtime C++, chạy trên robot.
