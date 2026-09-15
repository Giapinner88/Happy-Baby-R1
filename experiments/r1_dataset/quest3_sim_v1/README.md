# Thu dataset R1 bằng Quest 3 trong Isaac Sim

Đây là nơi chứa kế hoạch, đặc tả và bằng chứng cho việc thu dataset học bắt
chước cho R1. **Chưa có run nào và chưa có code nào chạy được đường này** — xem
PLAN.md để biết còn thiếu gì.

Đọc theo thứ tự:

1. **[PLAN.md](PLAN.md)** — sáu giai đoạn, mỗi giai đoạn một cổng đo được.
   Bắt đầu từ đây.
2. **[experiment.md](experiment.md)** — hồ sơ experiment, ranh giới với
   `r1_teleop/quest3_sim_v1`.
3. **[D001/D001.md](D001/D001.md)** — protocol đầu tiên: dataset "chỉ tay vào
   vật", không cần end-effector.
4. **[D001/config/](D001/config/)** — cấu hình khai báo: vật thể, camera, định
   dạng bản ghi, tiêu chí loại episode.

## Công cụ

`tools/inspect_episode.py` — đọc một thư mục `episode_XXXX/` theo định dạng
Unitree `EpisodeWriter` và báo cáo: số bước, nhãn task, bộ khớp, đối chiếu ảnh,
và RMS hiệu giữa `states` với `actions`. Chỉ dùng thư viện chuẩn.

```bash
python3 experiments/r1_dataset/quest3_sim_v1/tools/inspect_episode.py <episode_dir>
```

`tools/preview_scene.py` — dựng thử một khung cảnh quanh R1 và xem trong Isaac.
Không teleop, không ghi gì. Dùng để chọn bố cục trước khi chốt scene cho D001.

```bash
conda run --no-capture-output -n unitree_sim_env python \
  experiments/r1_dataset/quest3_sim_v1/tools/preview_scene.py \
  --environment simple_room --table --ycb 006_mustard_bottle --device cuda:0
```

Môi trường và vật lấy từ kho chuẩn NVIDIA (Isaac 5.1) qua HTTPS ở lần chạy đầu.
`--environment none` chạy hoàn toàn offline.

## Bộ cảnh

[D001/config/scenes/](D001/config/scenes/) chứa bảy biến thể cảnh dựng sẵn —
phòng, nhà kho, văn phòng, các bộ vật khác nhau — kèm danh mục đầy đủ những
khung cảnh và prop đã kiểm chứng tồn tại trong kho asset Isaac 5.1.

Việc tiếp theo của bạn nằm ở PLAN.md **Giai đoạn 1**.
