Doremon — kết thúc sớm và về HOME

Giữ nguyên policy và checkpoint model_19999.pt của run 4096 envs, friction 0.8–1.5, 20k iterations ngày 20/10/2026. Không train lại. Reference runtime giữ 3740 frame đầu (74,8 giây), bỏ toàn bộ phần nhảy sau đó, nối về HOME 1,5 giây và giữ yên 0,5 giây. NPZ mới: 3840 frame, 50 Hz, 76,8 giây.

Policy, checkpoint, metadata và MP3 giữ nguyên từ đúng run/gói gốc. env.yaml ghi cấu hình train gốc; MANIFEST ghi riêng reference thay đổi sau train. Các góc khớp trước điểm cắt giữ nguyên. Đoạn nối và kết thúc đạt QA archive/geometry; chưa đánh giá tracking policy với reference mới. Audio được giữ nguyên, không cắt.
