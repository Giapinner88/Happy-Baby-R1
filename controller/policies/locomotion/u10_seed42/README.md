# u10_seed42 — U10: 12 chân + 10 residual tay quanh tham chiếu teleop, train từ đầu

Thêm vào 2026-11-05, đặt làm `teleop_policy` (R2+X) theo quyết định của người vận hành để thử trên robot.
**Mô phỏng: failed** (xem *Bằng chứng hành vi*). `teleop_h4: false`.

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u10/runs/isaac/u10_seed42` — `scripts/loco/run.py train --config config/train/r1_u10.json`; thiết kế, đánh giá, chẩn đoán trong `experiments/r1_loco/u10/experiment.md` |
| Môi trường | Isaac Sim 5.1.0-rc.19, Isaac Lab v2.3.2, rsl-rl-lib 3.1.2, torch 2.7.0; seed 42; 10000 vòng; 4096 env; `vendor_hb_pose_waist200` |
| `model_9999.pt`, `policy.onnx`, yaml, json | khớp `artifact_manifest.json` của run (kiểm 2026-11-05); sha256 ONNX trong `policy.yaml` |

## Hợp đồng `r1_wb_legs_armres_gait_h4_v1_ref`

- Input 408 = 4 bước term-major của frame 101 (`base_ang_vel, projected_gravity, command, phase, joint_pos[24],
  joint_vel[24], actions[22], arm_q_ref[10], arm_dq_ref[10]`) + one-hot `STAND,WALK,W2S` + `arm_authority` g (cả hai raw).
- Output 22: 12 chân (default + scale × a, như U4), rồi 10 tay. Target tay =
  clamp(q_ref + g × 0.2 × clip(a, −1, 1), soft limit = giữa ± 0.9 × nửa dải URDF).
- `arm_q_ref`/`arm_dq_ref`: tay mong muốn (teleop H4 hoặc gesture; default khi không ai cầm tay) qua `ArmReferenceFilter`
  ở 50 Hz, trần 4 rad/s và 14 rad/s² (trần lúc đánh giá của loco), cập nhật trước obs và dùng cùng q_ref cho target.
  Overlay teleop/gesture **không** ghi đè target tay nữa; nó chỉ di chuyển tham chiếu.
- g: `armres_authority_idle` (1.0) khi không ai cầm tay, `armres_authority_overlay` (0.3) khi teleop/gesture cầm tay
  (`controller/config/locomotion.yaml`). Lúc train: không teleop g = 1; teleop cử động g 0.3–1; giữ tư thế g 0–0.3.
  Bộ điều khiển không biết đang "cử động" hay "giữ", nên 0.3 là điểm chung của hai dải (AI chọn).
- PD từ metadata: eo kp 200 / kd 4 (giữ bằng PD, không phải action), chân/tay như U4. FSM như U4, dwell 1.5 s.
- Runtime: `WholeBodyLegsGaitController(22)` trong `controller/src/policy/WholeBodyLegsGaitController.hpp`,
  `controller/src/policy/ArmReferenceFilter.hpp`.

## Kiểm chứng runtime (không phải bằng chứng hành vi)

- `wb_legs_armres_gait_golden_test` (trace từ `controller/scripts/make_wb_legs_gait_golden.py`, dùng
  `reference_filter_step` của loco lúc train và soft limit đọc từ URDF): 130 bước, tay idle → tư thế lớn vượt soft limit
  (g 0.6, 17 bước bị clamp) → giữ (g 0) → idle; khớp mode, obs 408, 22 action và 24 target. Bắt được lỗi cố ý bỏ clamp
  và đảo q_ref/dq_ref.
- `application_loco_mode_test`: đổi R2+A → R2+X sang U10; tay gửi ra motor là target của policy, nằm trong g × 0.2
  quanh tham chiếu; không ai cầm tay thì tham chiếu đứng ở default với g = 1; gesture di chuyển tham chiếu với g = 0.3,
  thu về và handover xong. Bắt được lỗi cố ý để overlay ghi đè tay.
- Giới hạn tay trong USD train (`R1_train_collision.usda`) trùng URDF và `RobotSpec::kJointRange*`.

## Bằng chứng hành vi (mô phỏng, một seed — `experiment.md` của U10)

| | U4 | U10 g = 0 | U10 g = 1 |
|---|---|---|---|
| Lúc train: ngã, `error_vel_xy` | 0.6 %, 0.63 | 12.2 %, 1.32 | |
| Sai số vận tốc E1 (m/s) | 0.247 | 0.516 | 0.520 |
| Đẩy 1.0 m/s: ngã, hồi phục trong 2 s | 2.1 %, 85 % | 41.4 %, 25 % | 39.6 %, 26 % |

Kết luận của thí nghiệm: **failed**, mọi tiêu chí có dữ liệu đều trượt xa. Chẩn đoán: std của 10 action tay giữ ≈ 1.0
tới cuối nên residual tay gần như ngẫu nhiên ±0.2 g rad mỗi bước; g < 0.3 khi tay đang cử động chưa từng được train.
U10b (std tay 0.2, g cử động trong [0, 1]) được đề xuất, chưa train; dùng cùng contract nên chạy được với controller này.
**Chưa chạy trên robot.** Thử phải có E-stop, giàn đỡ và test log theo `docs/safety.md`.
