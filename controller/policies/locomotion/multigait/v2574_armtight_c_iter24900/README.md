# multigait/v2574_armtight_c_iter24900

Contract HB: `r1_armfree0917_multigait_h4_v2573` (`MultiGaitV2573Controller`).
Chưa nghiệm thu trên robot. Export tự khai `deployable=False`; bằng chứng hiện có chỉ là mô phỏng.

## Nguồn

| | |
|---|---|
| Repo train | `unitree_rl_mjlab_gait_pro` (máy `train`) |
| Experiment | `experiments/armfree0917_multigait_run25_v2574_armtight_c_23k_2048` |
| Checkpoint | `logs/armfree0917_multigait_run25_v2574_armtight_c_23k_2048/seed42_2048env_28k/model_24900` |
| task_id | `Unitree-R1-ArmFree0917-MultiGait-Run25-v2574-ArmTight-C-23k-2048` |
| ONNX sha256 | `ef5d3873216e20969003dc6e338b67b94d7447e5cf7fc0599999fcf0f2f01400` |
| Lineage | resume từ `v2573_push_4096/model_23000`; train tới 24900/28000 rồi dừng tay (SIGINT 12/11) |

Ghi chú train: `HANDOFF_ARMFREE0917_V23_V2574.md` trong repo train (mục v2574 armtight).

## Hợp đồng

- obs 355 = 332 (H4 term-major, chuẩn hoá trong ONNX) + 23 conditioning FSM thô.
- output 25 = 24 offset khớp (`default + scale * a`) + lệnh nhịp ở index 24.
- FSM 8 mode, đồng hồ do policy điều khiển, bề rộng đứng 0.190 m, urgency recovery dạng elip.
- obs dùng gyro và vận tốc khớp thô, như lúc train.
- Không có `gait_period_s`: policy tự phát chu kỳ.
- Không hỗ trợ gesture và teleop H4.

## Bằng chứng (mô phỏng)

1. Parity runtime C++ với env Python: 42000 tick, 30 env, đủ 8 mode. FSM, conditioning, phase, lệnh khớp.
   Test HB `multigait_v2573_runtime_parity_test` chạy lại trên 9800 tick.
2. Probe vòng kín `simulate_cpp` (`multigait_v257_native_probe`), 14 case
   (đứng, đi 0.3/0.6/1.0, lùi, ngang, quay, chéo, dừng sớm/muộn, đẩy 80 N trong 0.35 s theo 4 hướng):

   | Ma sát chân | Ngã | Về STAND sau khi dừng |
   |---|---|---|
   | 0.6 | 0/14 | 14/14 |
   | 1.0 | 0/14 | 14/14 |
   | 1.5 | 0/14 | 14/14 |

   Bám vận tốc (mu 1.0): tiến 0.3: 117%, 0.6: 105%, 1.0: 97%; lùi 0.6: 90%; ngang 0.3: 57%; quay 0.8: 92%.
   Dừng từ 0.6 m/s về STAND sau 1.88 s.
3. So sánh cùng probe: v2574_b và v2573_push không về được STAND (bước tại chỗ, khoảng 1 lần/s);
   v2573_turnabduct ngã khi bị đẩy ngang 80 N.

## Giới hạn đã biết

- Train dừng sớm; reward thấp hơn v2573_push ở cùng iteration (10.5 so với 18.1, handoff).
- Tay: eval ở 24200 cho thấy khuỷu trái gập khoảng 91° sau khi dừng.
- Đi ngang chỉ bám khoảng 57%. Trần tốc tiến 1.2 m/s (runtime).
- Chưa có bằng chứng phần cứng nào.

## Lần thử đầu trên robot

Giá treo, có người giữ E-stop, tốc độ `slow_*`. Log `[MultiGaitV2573] mode=...` in mỗi giây (mode, hold, chu kỳ, lệnh, ước lượng tốc độ, recovery).
