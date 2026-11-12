# u11_seed42 — U11: U4 với one-hot gait để raw (một thay đổi duy nhất)

Thêm vào 2026-11-06; chưa chọn cho R2+A/R2+X. `teleop_h4: false` (duyệt H4 là quyết định của người vận hành).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u11/runs/isaac/u11_seed42` — `scripts/loco/run.py train --config config/train/r1_u11.json` (= `r1_u4.json` + `raw_gait_onehot: true`); thiết kế trong `experiments/r1_loco/u11/experiment.md` |
| Train | Isaac Lab, seed 42, 4096 env, 5000 vòng, `vendor_hb_pose`, như U4 |
| `model_4999.pt`, `policy.onnx`, yaml, json | khớp `artifact_manifest.json` của run (kiểm 2026-11-06); sha256 ONNX trong `policy.yaml` |

## Hợp đồng

`r1_wb_legs_gait_h4_v1_blind` như U4 (287 → 12, FSM dwell 1.5 s, PD và default như U4). Khác duy nhất bên trong ONNX:
one-hot không qua normalizer (`gait_one_hot_normalized: false`); controller vẫn đưa one-hot 0/1, không đổi code.
Metadata ghi đủ chữ số (scale cổ chân 0.3125; U4 ghi 0.312).

## Kiểm chứng runtime

`wb_legs_gait_u11_golden_test` (trace `controller/tests/data/wb_legs_gait_u11_golden_trace.txt`): PASS 130 bước + Reset.
Preflight đạt với armfree (R2+A) + U11 (R2+X).

## Bằng chứng hành vi (mô phỏng, một seed; số liệu `metrics.json` các run đánh giá, đọc 2026-11-06)

| | U4 | U11 |
|---|---|---|
| E1 sai số vận tốc xy (m/s) | 0.247 | 0.246 |
| Đẩy 1.0 m/s: ngã, hồi phục trong 2 s | 2.1 %, 85 % | 0.6 %, 90 % |
| Đẩy 1.5 m/s: ngã, hồi phục trong 2 s | 20.8 %, 64 % | 14.2 %, 71 % |
| Tốc độ target chân khi yên sau đẩy 1.0 (rad/s RMS) | 0.494 | 0.395 |

E2 chưa xong lúc chép; `experiment.md` còn ghi "running", chưa có kết luận. Đẩy 1.5: số reset 174 so với U4 144 dù tỉ lệ
ngã thấp hơn — chưa đối chiếu cách đếm. Một seed: khác biệt với U4 có thể do one-hot hoặc do seed. **Chưa chạy trên robot.**
