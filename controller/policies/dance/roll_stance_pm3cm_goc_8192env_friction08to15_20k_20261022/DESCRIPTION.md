Roll — random stance ±3 cm, final 20k

Train mới từ đầu với 8192 envs, friction 0.8–1.5, seed 42, 20.000 iterations. Task Unitree-R1-Tracking-Roll-Random-Stance: mỗi reset bắt đầu frame 0; chọn ngẫu nhiên trong 121 tư thế IK hợp lệ. Bề rộng giữa tâm hai chân 13,43–19,43 cm, quanh gốc 16,43 cm (±3 cm tổng bề rộng).

Checkpoint cuối model_19999.pt; ONNX actor 129→24 và metadata lấy đúng cùng run. Runtime reference giữ đúng NPZ roll_xexe_contact_fixed_50hz.npz đã train, 308 frame/50 Hz/6,16 giây. Ngân hàng s0 chỉ dùng khi train, lưu trong training_reset/ để truy xuất provenance; không cần để chạy HB policy. Không audio như các gói roll trước.
