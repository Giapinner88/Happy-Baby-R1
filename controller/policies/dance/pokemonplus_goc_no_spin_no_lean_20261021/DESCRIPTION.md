Pokémon Plus — bỏ nhảy xoay và ngửa lưng

Giữ nguyên policy/checkpoint model_19999.pt từ run ngày 18/10/2026; không train lại. NPZ runtime đã thay vùng 42,5–52 giây của bản gốc để bỏ nhảy xoay, sau đó nối 50,3 → 55,1 giây của bản no-spin bằng chuyển tiếp 0,8 giây để bỏ động tác đưa hai tay ra trước/ngửa lưng.

Reference cuối có 3414 frame, 50 Hz, 68,28 giây. Giữ nguyên MP3 theo yêu cầu người dùng; audio không bị cắt theo 4 giây motion đã bỏ. Các vùng sửa đã kiểm tra giới hạn khớp, FK/vận tốc, xuyên sàn và tự va chạm. Policy chưa được đánh giá tracking với reference mới. env.yaml giữ provenance của quá trình train gốc.
