# Verify trước khi đưa vào simulate_cpp

`golden_trace_h4.txt` / `golden_trace_h5.txt` là trace chuẩn dùng chung cho ba
runtime (Python, HB, simulate_cpp). Mỗi dòng `PACKED` là một vector actor đã
đóng gói sẵn — nếu C++ dựng vector từ cùng các dòng `GYRO/GRAV/CMD/PHASE/Q/DQ/PREV`
mà ra khác `PACKED`, thì packing của C++ sai, và sai đó **không** phát hiện được
bằng kích thước tensor: term-major và time-major đều là 332 số.

Chạy checker (cần mjlab + onnxruntime; `check_flat_plus.py` cần repo training):

    python scripts/check_flat_plus.py --history-length 4 \
        --onnx <bundle>/DEPLOY/policy_flat_plus_h4_v1.onnx --strict

Kỳ vọng: `0 failed, 0 skipped`.

Metadata bắt buộc trong ONNX (21 khoá), quan trọng nhất khi đọc từ C++:

    policy_contract          = flat_plus_h4_v1     <- dispatch bằng khoá này
    actor_history_order      = term_major_oldest_to_newest
    actor_history_padding    = repeat_first
    actor_history_steps      = 4
    observation_layout_json  = offset từng term + slice hiện tại

**Không dispatch bằng `input_dim`.** Một vector 332D term-major và một vector
332D time-major load được như nhau và cho action khác hẳn.
