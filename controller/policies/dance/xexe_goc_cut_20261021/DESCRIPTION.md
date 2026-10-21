Xexe — cắt 1:12,80 đến 1:19,28

Giữ nguyên policy GOC và checkpoint model_19999.pt của run 20k ngày 13/10/2026, không train lại. NPZ runtime bỏ 324 frame (6,48 giây), chỉ làm mượt 24 frame quanh điểm nối, không chèn đoạn thay thế. Bản mới có 4683 frame, 50 Hz, 93,66 giây.

Giữ nguyên audio xexe.mp3 theo lựa chọn của người dùng. Checkpoint và metadata thuộc cùng run gốc. env.yaml là cấu hình train gốc; reference runtime đã chỉnh sau train và ghi riêng provenance trong MANIFEST. QA điểm nối đạt; chưa đánh giá tracking policy với reference mới.
