# u12_seed42 — U12: U4 với push lúc train mạnh hơn (một thay đổi duy nhất)

Thêm vào 2026-11-06; chưa chọn cho R2+A/R2+X. `teleop_h4: false` (duyệt H4 là quyết định của người vận hành).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u12/runs/isaac/u12_seed42` — `scripts/loco/run.py train --config config/train/r1_u12.json` (= `r1_u4.json` + `push_xy_max` 1.0, `push_xy_ramp_iterations` 2000); thiết kế trong `experiments/r1_loco/u12/experiment.md` |
| Train | Isaac Lab, seed 42, 4096 env, 5000 vòng, `vendor_hb_pose`; push x/y ±0.5 tăng tuyến tính tới ±1.0 m/s trong 2000 vòng đầu |
| `model_4999.pt`, `policy.onnx`, yaml, json | khớp `artifact_manifest.json` của run (kiểm 2026-11-06); sha256 ONNX trong `policy.yaml` |

## Hợp đồng

`r1_wb_legs_gait_h4_v1_blind` như U4, one-hot vẫn qua normalizer như U4 (`gait_one_hot_normalized: true`). Controller không đổi.

## Kiểm chứng runtime

`wb_legs_gait_u12_golden_test` (trace `controller/tests/data/wb_legs_gait_u12_golden_trace.txt`): PASS 130 bước + Reset.
Preflight đạt với armfree (R2+A) + U12 (R2+X).

## Bằng chứng hành vi (mô phỏng, một seed; số liệu `metrics.json` các run đánh giá, đọc 2026-11-06)

| | U4 | U12 |
|---|---|---|
| E1 sai số vận tốc xy (m/s) | 0.247 | 0.257 |
| Đẩy 1.0 m/s: ngã, hồi phục trong 2 s | 2.1 %, 85 % | 0.9 %, 90 % |
| Đẩy 1.5 m/s: ngã, hồi phục trong 2 s | 20.8 %, 64 % | 7.7 %, 81 % |
| Tốc độ target chân khi yên sau đẩy 1.0 (rad/s RMS) | 0.494 | 0.435 |

Tiêu chí của thí nghiệm: ngã khi đẩy thấp hơn U4 (đạt cả 1.0 và 1.5); E1 không tệ hơn U4 quá 5 % (0.257 so với 0.247,
+4 %, đạt sát); E2 chưa xong lúc chép; `experiment.md` chưa có kết luận. **Chưa chạy trên robot.**
