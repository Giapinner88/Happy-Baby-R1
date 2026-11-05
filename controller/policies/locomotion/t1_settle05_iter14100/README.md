# ArmFree0917 T1 — checkpoint 14100

Nguồn: task MJLab `Unitree-R1-ArmFree0917-T1-Settle05-4096`,
`model_14100.pt`, mới nhất tại thời điểm bắt đầu export ngày 05/11/2026.
Run có tổng đích 20000 iteration và vẫn tiếp tục train; bundle này là snapshot cố định.
SHA checkpoint, ONNX, metadata và sai số Torch–ONNX nằm trong `packaging.json`.

Contract `flat_plus_gait_h4_v1`: input 335, output 24, H4 term-major,
normalization chỉ áp cho prefix 332. FSM dùng RMS đủ 24 khớp, settle 0.5s.
Phase dùng episode clock T=1s, frame đầu t=0; lệnh norm <0.1 mask phase
nhưng clock vẫn chạy. Kp/Kd, action scale và default pose lấy đúng metadata nguồn.

```yaml
walk_policy: t1_settle05_iter14100
walk_gait_period_s: 1.0
teleop_policy: u4_seed42
teleop_gait_period_s: 0.6
```

`gait_period_s` là fallback khi thiếu override. Runtime kiểm period của từng route
và đổi clock khi chuyển policy. Rollback: chọn `flat_plus_gait_0917_armfree`
và đặt `walk_gait_period_s: 0.6`.

Miền train: vx [-1.0, 1.2], vy [-0.6, 0.6] m/s, yaw [-1.0, 1.0] rad/s.
Các giới hạn lệnh hiện có của HB vẫn có thể vượt miền này.
Chưa deploy, chưa nghiệm thu phần cứng hoặc H4 co-motion.

Build/preflight và 10/10 kiểm tra tích hợp đã đạt; xem `integration_validation.json`.
Phase T1 khớp Torch qua 1500 frame, sai số tối đa `5.96e-8`; đã kiểm tra chuyển
1s ↔ 0.6s và chặn metadata sai. Chưa có qualification closed-loop physics của HB.
