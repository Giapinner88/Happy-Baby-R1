# Flat-Plus candidates — 2026-09-07

Đây là kho candidate tách khỏi `policy/locomotion/`. Các file ở đây không được
CMake tự động đưa vào `single_policy_*` và không được chọn bởi
`config/tuning.yaml`.

## Gait-conditioned MLP

`gait/policy_flat_plus_gait_h4_v1.onnx`

- Contract: `flat_plus_gait_h4_v1`
- Tensor: `[1,335] -> [1,24]`, một input/một output
- Layout: H4 `332-D` + gait one-hot hiện tại `[STAND,WALK,W2S]`
- Normalization: chỉ prefix 332-D; gait 3-D raw 0/1
- Trạng thái: bản này đã được promote nguyên vẹn vào
  `policy/locomotion/flat_plus/` sau khi C++ cập nhật validator và scheduler
  theo metadata FSM canonical; giữ lại ở đây làm provenance đối chiếu.

Không đổi tên contract. `gait_golden_trace.txt` là trace chuẩn để kiểm parity
khi thay đổi scheduler về sau.

## LSTM ablations

`lstm/policy_flat_plus_gait_lstm_paper_v1.onnx` và
`lstm/policy_flat_plus_gait_lstm_capacity_v1.onnx` đều là research ablation:

- input `obs[1,86]` + `h_in/c_in`;
- output `actions` + `h_out/c_out`;
- lần lượt hidden `64` và `247`;
- metadata `deployable=false` và không có route single-policy C++.

Do đó không đưa chúng vào `policy/locomotion/`; muốn test tiếp phải xây route
stateful riêng, gồm reset hidden state và parity nhiều bước.

Nguồn provenance đầy đủ (checkpoint, config, metrics, manifest) vẫn ở
`r1_flat_dev_20260907/`.
