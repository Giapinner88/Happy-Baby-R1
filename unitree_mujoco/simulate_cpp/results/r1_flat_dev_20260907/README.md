# R1 Flat — bàn giao máy dev, 2026-09-07

Năm policy, ba contract. Cái nào mang sang `simulate_cpp` được ngay và cái nào
chưa, nói rõ ở dưới.

## Mang sang được ngay

| | Contract | Tensor | Ghi chú |
|---|---|---|---|
| `H4_H5/DEPLOY/policy_flat_plus_h4_v1.onnx` | `flat_plus_h4_v1` | `[1,332] → [1,24]` | **arm được chọn** |
| `H4_H5/arms/h5_candidate/policy.onnx` | `flat_plus_h5_v1` | `[1,415] → [1,24]` | đối chứng, hoà với H4 |

H4/H5 chỉ cần packing observation — golden trace và metadata đã phủ đủ.

## Cần thêm việc trước khi deploy

| | Contract | Tensor |
|---|---|---|
| `GAIT/gait/policy.onnx` | `flat_plus_gait_h4_v1` | `[1,335] → [1,24]` |
| `GAIT/lstm_paper/policy.onnx` | `flat_plus_gait_lstm_v1` | stateful, h/c là I/O |
| `GAIT/lstm_capacity/policy.onnx` | `flat_plus_gait_lstm_v1` | stateful, h=247 |

Gait cần C++ cài thêm **gait FSM + bộ lọc stability**, là code có trạng thái.
`verify/gait_golden_trace.txt` tồn tại để đối chiếu đúng phần đó.

Hai bản LSTM là **ablation nghiên cứu, không phải đường deploy** — metadata ghi
`deployable=false` và chúng không có route trong runtime. Đừng cấp route cho
chúng.

## Kiểm trước khi tin

```bash
sha256sum -c SHA256SUMS

python verify/check_flat_plus.py --history-length 4 \
    --onnx H4_H5/DEPLOY/policy_flat_plus_h4_v1.onnx --strict     # kỳ vọng 0 failed 0 skipped
python verify/check_flat_plus_gait.py \
    --onnx GAIT/gait/policy.onnx --strict
```

## Hai điều quan trọng nhất khi ghép vào C++

**1. Dispatch bằng `policy_contract` trong metadata, KHÔNG bằng `input_dim`.**
Vector 332D term-major và 332D time-major load được như nhau, chạy được như
nhau, và cho action khác hẳn. Kích thước không phân biệt được.

**2. Đối chiếu golden trace trước khi tin packing của mình.**

| File | Phủ gì |
|---|---|
| `verify/golden_trace_h4.txt` | dựng frame 83D → gói 332D, 2 mốc reset kiểm backfill |
| `verify/golden_trace_h5.txt` | như trên, 415D |
| `verify/gait_golden_trace.txt` | **FSM + bộ lọc**: `α`, thứ tự lọc-rồi-norm, seed lúc reset, hysteresis, dwell, timeout |

Cách dùng: đọc `GYRO/GRAV/CMD/PHASE/Q/DQ/PREV` (hoặc `CMD/W/DQ` với trace gait),
chạy code C++ của mình, so với dòng kết quả cùng step. Lệch ở đâu biết ngay ở đó.

`gait_golden_trace.txt` có 640 step, phủ STAND 110 / WALK 75 / W2S 455, 2 mốc
reset (một cái đặt giữa lúc robot đang mất ổn định để kiểm quy tắc seed filter),
và 1 lần W2S timeout.

## Bốn lỗi C++ mà trace gait bắt được

Đã kiểm bằng negative control — mỗi lỗi được cài thử và xác nhận trace fail đúng chỗ:

1. `α` sai (dùng `τ/dt`, hoặc quên `dt`)
2. Lọc `|ω|` thay vì lọc `ω` rồi mới lấy norm
3. Reset bộ lọc về 0 thay vì seed bằng mẫu hiện tại
4. Dùng một ngưỡng thay vì hai (mất hysteresis)

Cả bốn đều **không gây lỗi gì** khi chạy — chỉ ra mode khác.

## Giới hạn

- **1 seed (42)** cho mọi policy. Plan đòi ≥3 seed mới được kết luận.
- **Chưa qua Gate 7** (closed-loop simulator) cho bất kỳ policy nào.
- **H1-control (83D) chưa retrain** cùng pipeline, nên chưa trả lời được
  "history 332D có hơn baseline 83D không".
- 5 ablation gait của plan 04 (no-gait-input, shuffled-ID…) chưa có.

`docs/` chứa toàn bộ kế hoạch, gồm các correction đo được trong đợt này.
