# HB R1 High-Level Runner

## Cấu trúc

```
high_level/
├── config/tuning.yaml        ← Tham số cấu hình runtime (không cần build lại)
├── policies/
│   ├── flat/   *.onnx        (model đi bộ, chọn qua flat_model trong tuning.yaml)
│   └── dance/  *.onnx + *.npz (model nhảy, khai báo qua dance_model_N/dance_npz_N)
├── scripts/    build.sh, deploy_to_robot.sh
├── src/        Mã nguồn C++ High-Level Runner
└── thirdparty/ cnpy, onnxruntime (x86_64 + aarch64)
```

## Build & deploy

```bash
# Trên PC (kiểm tra compile):
./scripts/build.sh

# Đẩy sang robot + build từ xa (robot: unitree@192.168.12.2):
./scripts/deploy_to_robot.sh

# Chạy trên robot:
ssh -Y unitree@192.168.12.2
cd ~/HB/high_level_2/build 
./run_r1                                         # Mặc định lấy network_interface từ tuning.yaml
./run_r1 eth10                                   # Ghi đè interface (card mạng) 1 lần
```

## Điều khiển

| Hành động                    | Bàn phím (cửa sổ X11)       | Tay cầm R3-1                             |
| ------------------------------- | ------------------------------- | ----------------------------------------- |
| Gồng cứng (STAND LOCK)        | `0`                           | `L2 + Lên`                             |
| Bật đi bộ (LOCOMOTION)       | `1` (phải đang STAND LOCK)  | `R2 + A`                                |
| Nhảy DANCE (MIMIC)             | `2`–`8` (đang LOCOMOTION) | `R1 + Lên/Phải/Xuống/Trái/A` (2–6) |
| Di chuyển                      | `W/S/A/D`, xoay `Q/E`       | Stick trái + phải                       |
| Đổi tốc độ                 | `Tab` (toggle)                | `R2+Lên`=nhanh, `R2+Xuống`=chậm    |
| Reset policy                    | `R`                           | —                                        |
| **Ngồi xuống an toàn** | `9`                           | `L2 + X`                                |
| **DỪNG KHẨN CẤP**      | `ESC`                         | `L2 + B`                                |

Trình tự chuẩn: treo robot → `./run_r1 eth0` → `0` (đứng dậy) → đặt xuống đất → `1` (đi bộ) → `2` (dance, tự về đi bộ khi xong).

### Kết thúc chương trình

- **Ngồi xuống an toàn (`9` / `L2+X`)**: Robot hạ về tư thế squat (feet-flat) sau đó xả lực. Thích hợp để dừng khi robot đang trên sàn phẳng.

## Cấu hình hệ thống

- **Tham số cứng (`src/config/RobotSpec.hpp`)**: Thứ tự khớp, pose mặc định, tỷ lệ action, PD gains từ ONNX policy. KHÔNG thay đổi các giá trị này.
- **Tham số runtime (`config/tuning.yaml`)**: Gains lúc thả lỏng/ngồi, giới hạn lệnh, bộ lọc tín hiệu, cấu hình fall detector. Có thể chỉnh sửa và chạy lại mà không cần re-build.

## Hướng dẫn thêm điệu nhảy (Dance)

1. Chép file `.npz` và `.onnx` tương ứng vào `policies/dance/`.
2. Khai báo `dance_model` và `dance_npz` trong `tuning.yaml`.
3. Layout yêu cầu đối với file .npz: `joint_pos/joint_vel (frames,24)`, `body_quat_w (frames,N,4)` wxyz, `fps=50`; torso = body index 14.
4. Tham số `dance_start_frame` mặc định `-1` sẽ tự động tìm frame khớp êm nhất. Có thể tuỳ chỉnh nếu cần thiết.
