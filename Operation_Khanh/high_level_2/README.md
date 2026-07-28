# HB R1 High-Level Runner

Ứng dụng deploy các chính sách học tăng cường RL (di chuyển + nhảy múa) lên robot humanoid Unitree R1.

## Cấu trúc thư mục

```
high_level_2/
├── config/tuning.yaml        ← Các tham số cấu hình chạy runtime
├── policies/
│   ├── flat/   *.onnx  (policy di chuyển)
│   └── dance/  *.onnx + *.npz  (policy nhảy múa và dữ liệu chuyển động)
├── scripts/    Các script build và deploy
├── src/
│   ├── main.cpp
│   ├── app/         Quản lý trạng thái và vòng lặp điều khiển chính (500Hz)
│   ├── config/      Định nghĩa thông số robot và cấu hình tham số
│   ├── estimation/  Ước lượng trạng thái robot (IMU, lọc thông thấp)
│   ├── gait/        Bộ lập lịch dáng đi (gait scheduler)
│   ├── policy/      Nạp và thực thi mô hình ONNX
│   ├── motion/      Xử lý dữ liệu chuyển động từ file NPZ
│   ├── input/       Nhận tín hiệu điều khiển từ gamepad và bàn phím
│   ├── robot/       Gửi lệnh low-level xuống robot qua DDS
│   ├── safety/      Bộ phát hiện ngã (fall detector)
│   └── util/        Bộ lọc tín hiệu thông thấp
└── thirdparty/      Thư viện cnpy và onnxruntime
```

## Hướng dẫn Build & Deploy

```bash
# Trên PC (Kiểm tra biên dịch):
./scripts/build.sh

# Deploy sang robot (Unitree R1) và build từ xa:
./scripts/deploy_to_robot.sh

# Chạy trực tiếp trên robot:
ssh -Y unitree@192.168.12.2
cd ~/HB/high_level_2/build
./run_r1
```

Mặc định chương trình sẽ sử dụng card mạng được cấu hình trong `config/tuning.yaml`. Bạn có thể chỉ định card mạng thủ công bằng cách truyền tham số: `./run_r1 <interface>`.

## Điều khiển Robot

| Hành động | Bàn phím (cửa sổ X11) | Tay cầm R3-1 |
|---|---|---|
| Đứng gồng (STAND LOCK) | `0` | `L2 + Lên` |
| Bắt đầu đi bộ (LOCOMOTION) | `1` (khi đang STAND LOCK) | `R2 + A` |
| Nhảy DANCE (MIMIC) | `2`–`8` (khi đang LOCOMOTION) | `R1 + Lên/Phải/Xuống/Trái/A` (2–6) |
| Di chuyển | `W/S/A/D`, xoay `Q/E` | Stick trái + phải |
| Thay đổi tốc độ | `Tab` | `R2+Lên` (nhanh), `R2+Xuống` (chậm) |
| Reset policy | `R` | — |
| Ngồi xuống an toàn | `9` | `L2 + X` |
| DỪNG KHẨN CẤP | `ESC` | `L2 + B` |

Quy trình vận hành chuẩn: Treo robot an toàn → Chạy `./run_r1` → Nhấn `0` để đứng dậy → Đặt robot xuống đất → Nhấn `1` để đi bộ → Chọn các chế độ nhảy múa từ `2` đến `8`.

### Watchdog tay cầm R3-1

- Gói remote bằng 0 dưới `remote_timeout_ms` (`3000 ms`) là `SUSPECT`: ngừng nhận nút/stick nhưng chưa phát cảnh báo.
- Quá 3 giây là `LOST`; sau `safe_stop_debounce_ms`, vận tốc bị ép về 0 và phát cảnh báo an toàn một lần.
- Khi kết nối lại, R3-1 phải trung tính ổn định `remote_recover_ms` (`200 ms`) trước khi nhận lệnh.

## Hướng dẫn thay đổi chuyển động nhảy múa (Dance)

1. Sao chép file `.npz` chuyển động mới vào thư mục `policies/dance/`.
2. Khai báo đường dẫn file trong cấu hình `tuning.yaml`.
3. Cấu trúc file `.npz` yêu cầu các mảng: `joint_pos` / `joint_vel` kích thước `(frames, 24)`, `body_quat_w` kích thước `(frames, N, 4)` dạng wxyz ở tần số `50Hz`.
4. Tham số `dance_start_frame` để mặc định `-1` sẽ kích hoạt chế độ tự động tìm kiếm frame bắt đầu mượt mà nhất. Bạn cũng có thể thiết lập một giá trị cụ thể để khóa cứng frame bắt đầu mong muốn.
