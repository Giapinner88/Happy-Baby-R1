# u13a_seed42 — U13a: U12 với settle dwell 0.5 s (một thay đổi duy nhất)

Thêm vào 2026-11-10; chưa chọn cho R2+A/R2+X. `teleop_h4: false` (duyệt H4 là quyết định của người vận hành).

## Nguồn

| Mục | Giá trị |
|---|---|
| Nhánh / repo | `Happy-Baby-R1-loco` (cây làm việc chưa commit; `provenance.json` ghi commit và sha256 từng file nguồn) |
| Run | `experiments/r1_loco/u13a/runs/isaac/u13a_seed42` — `scripts/loco/run.py train --config config/train/r1_u13a.json` (= `r1_u12.json` + `gait_settle_dwell_s` 0.5); thiết kế và đánh giá trong `experiments/r1_loco/u13a/experiment.md` |
| Train | Isaac Lab, seed 42, 4096 env, 5000 vòng, `vendor_hb_pose`; push như U12 |
| `model_4999.pt`, `policy.onnx`, yaml, json | khớp `artifact_manifest.json` của run (kiểm 2026-11-10); sha256 ONNX trong `policy.yaml` |

## Hợp đồng

`r1_wb_legs_gait_h4_v1_blind` như U4/U12. Metadata ONNX khác U12 đúng một khoá: `gait_t_settle_s` 0.5 (U12 1.5),
controller đọc dwell từ đây nên W2S → STAND sau 0.5 s đứng yên. Controller không đổi.

## Kiểm chứng runtime

`wb_legs_gait_u13a_golden_test` (trace `controller/tests/data/wb_legs_gait_u13a_golden_trace.txt`, FSM loco với dwell
0.5 s): PASS 130 bước + Reset.

## Bằng chứng hành vi (mô phỏng, một seed, đánh giá tối thiểu 2026-11-10 — `experiment.md` của U13a)

Chỉ chạy `push1.0` (1024 env) và `stop_after_walk` (256); push 1.5, E1/E2, bias, jitter chưa chạy.

| | U4 | U12 | U13a |
|---|---|---|---|
| Dừng: tỉ lệ STAND trong thời gian dừng | 66 % | 22 % | 88 % |
| Dừng: median nghiêng W2S / STAND | 2.6 / 1.1° | 5.4 / 1.0° | 2.1 / 1.0° |
| Dừng: median lệch gối / hông pitch L-R | 0.029 / 0.042 rad | 0.051 / 0.059 | 0.028 / 0.045 |
| Đẩy 1.0: ngã, hồi phục trong 2 s | 2.1 %, 85 % | 0.9 %, 90 % | 1.7 %, 91 % |
| Đẩy 1.0: RMSE tuyến tính / yaw | 0.255 / 0.263 | 0.234 / 0.235 | 0.230 / 0.238 |

Đạt tiêu chí dừng (tỉ lệ STAND, nghiêng W2S), dáng đứng đối xứng như U4; trượt sát tiêu chí đẩy (1.7 % so với U12
0.9 %, 17/1024 so với 10/1024). Một phần tỉ lệ STAND tăng là cơ học (dwell ngắn hơn 1 s). **Chưa chạy trên robot.**
