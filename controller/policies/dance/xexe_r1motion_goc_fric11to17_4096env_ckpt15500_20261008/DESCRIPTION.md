# Xexe R1MotionLab — GOC snapshot checkpoint 15500

NPZ Xexe được xử lý bằng R1MotionLab; 5007 frame, 50 Hz, giữ timing. Chỉ 240/5007 frame có nghiệm dynamics local; các vùng chưa giải giữ pose gốc. Reference chưa được chứng nhận cân bằng toàn clip. Actor 129 obs → 24 actions, tracking ankle. Package inactive, chưa gán dance slot. MP3 giữ đúng cặp motion/audio từ package nguồn xexe_trimmed_20260921_meta_19999; không thêm cắt/retime.

Framework unitree_rl_mjlab_goc, 4096 envs, seed 42, friction 1.1–1.7. Checkpoint model_15500.pt có 15501 updates theo index bắt đầu từ 0, mục tiêu 20000. Log chạy tới index 15842 nhưng phần sau lần lưu 15500 không có trong package. Đây là snapshot chưa hoàn tất 20k. Metadata ONNX lấy từ đúng run. Chất lượng đang được kiểm tra riêng bằng rollout không reset.
