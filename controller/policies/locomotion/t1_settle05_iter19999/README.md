# ArmFree0917 T1 — checkpoint 19999 (run hoàn tất)

Nguồn: task MJLab `Unitree-R1-ArmFree0917-T1-Settle05-4096`, `model_19999.pt`.
Run đã chạy hết 20000 iteration / 480000 policy step ngày 05/11/2026 và ghi
`TRAINING_COMPLETE.json`; đây là checkpoint cuối, không còn là snapshot giữa chừng.
Bundle này thay cho `t1_settle05_iter14100` cùng run.

SHA checkpoint, ONNX, metadata và sai số Torch–ONNX nằm trong `packaging.json`.
Parity đo lại trên chính file được ship: play env num_envs=1 trên CPU, 128 bước
liên tiếp, sai số tuyệt đối tối đa `1.67e-06`.

Contract `flat_plus_gait_h4_v1`: input 335, output 24, H4 term-major,
normalization chỉ áp cho prefix 332. FSM dùng RMS đủ 24 khớp, settle 0.5s.
Phase dùng episode clock T=1s, frame đầu t=0; lệnh norm <0.1 mask phase
nhưng clock vẫn chạy. Kp/Kd, action scale và default pose lấy đúng metadata nguồn.

102 key metadata giống hệt bundle 14100 về mặt tập key; chỉ 6 giá trị đổi:
`source_checkpoint*` (provenance mới) và `run_path`. Năm key
`gait_phase_contract`, `gait_phase_tick_alignment`, `run_path`,
`source_checkpoint*` không do runner train ghi ra mà được đóng dấu lúc đóng gói,
đúng như bundle 14100; hai key phase chỉ được chép lại sau khi đã assert
`training_phase_clock`, `training_phase_zero_mask`, `gait_period_s` và
`gait_t_settle_s` không đổi giữa hai bản export.

```yaml
walk_policy: t1_settle05_iter19999
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

Build/preflight và kiểm tra tích hợp: xem `integration_validation.json`.
`quality_passed` của run train là `false` — hoàn tất số iteration không phải
nghiệm thu chất lượng. Chưa có qualification closed-loop physics của HB.
