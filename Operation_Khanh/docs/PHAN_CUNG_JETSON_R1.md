# Phần cứng máy tính R1 (Jetson Orin NX) — tài liệu chi tiết

> **Liên quan:** [Sơ đồ mạng](SO_DO_MANG_R1.md) · [Dịch vụ &amp; deploy HB](DICH_VU_HB_STACK_R1.md)
>
> 🌐 **Bản web:** https://claude.ai/code/artifact/3ffa8259-0a7e-47d9-8b5e-dad37197a61b · ⚠️ File tài liệu — **không deploy lên robot**.

---

## 1. Tóm tắt nhanh

| Hạng mục                | Giá trị                                                                      |
| ------------------------- | ------------------------------------------------------------------------------ |
| **Máy**            | NVIDIA**Jetson Orin NX** Developer Kit (Tegra `t186ref`)               |
| **Hostname**        | `ubuntu`                                                                     |
| **OS**              | Ubuntu**20.04.5 LTS** (Focal), kernel **5.10.104-tegra**           |
| **JetPack / L4T**   | **L4T 35.3.1** (≈ JetPack 5.1.1), build 2023-03-19                      |
| **CUDA**            | **11.4** (V11.4.315); có `/usr/local/cuda-11.4`                       |
| **CPU**             | 8 nhân ARMv8 (Cortex-A78AE), tối đa**1984 MHz**, aarch64              |
| **RAM**             | **16 GB** (15 GiB khả dụng; lúc đo dùng ~1.5 GiB)                   |
| **Ổ cứng**        | NVMe`/dev/nvme0n1p1` **469 GB**, dùng **7%** (~32 GB)           |
| **Nhiệt độ**     | CPU 58°C, GPU 54°C, SoC ~56°C (mát, tải nhẹ) — quạt do`nvfancontrol` |
| **Uptime lúc đo** | ~50 phút, load 0.2–0.5                                                       |

Đây là **bộ não cấp cao** của R1 — nơi chạy policy điều khiển (ONNX), voice, và toàn bộ stack HB. Bo điều khiển cấp thấp (motor/IMU) nằm ở máy khác trên LAN R1 (`192.168.123.161`).

---

## 2. Tính toán & tăng tốc

- **GPU/CUDA:** CUDA 11.4, dùng cho ONNX Runtime (thư mục `high_level_2/thirdparty/onnxruntime_aarch64`). Policy chạy inference trên Orin NX.
- **Kiến trúc ARM64** → mọi binary phải build cho `aarch64`. Script build/deploy tự chọn `thirdparty/onnxruntime_aarch64/lib` khi `uname -m = aarch64`.
- **Bộ nhớ dư dả** (12 GiB trống) và **ổ NVMe 418 GB trống** → thoải mái cho model, log, backup.

---

## 3. Thiết bị USB

`lsusb`:

| Thiết bị                     | ID            | Vai trò                                                                  |
| ------------------------------ | ------------- | ------------------------------------------------------------------------- |
| **TP-Link 802.11ac NIC** | `2357:0138` | **WiFi dongle USB** — chính là `wlan0` (đường ra Internet!) |
| QinHeng CH340                  | `1a86:5395` | USB-serial (cầu nối bo/sensor R1)                                       |
| QinHeng CH340                  | `1a86:55ec` | USB-serial                                                                |
| QinHeng CH340                  | `1a86:55e7` | USB-serial                                                                |
| QinHeng USB2.0 HUB             | `1a86:809f` | Hub gom các cổng serial                                                 |

> **Lưu ý quan trọng:** WiFi của robot là **USB dongle TP-Link**, không phải WiFi tích hợp. Nếu dongle lỏng/rớt → mất Internet → voice chết (`transport_error`). Đây là điểm hỏng vật lý cần để ý.
>
> Các **CH340** là chip USB-to-serial (thường dùng nối vi điều khiển). Lúc đo **không thấy node `/dev/ttyUSB*`/`/dev/ttyACM*`** — nhiều khả năng đang bị các dịch vụ Unitree (`master_service`/`nxserver`) giữ hoặc bind qua driver khác. Không nên tự mở/độc chiếm các cổng này khi robot đang chạy.

---

## 4. Âm thanh (audio)

`/proc/asound/cards`:

| Card  | Tên                                     | Vai trò                                                        |
| ----- | ---------------------------------------- | --------------------------------------------------------------- |
| `0` | **HDA** — Tegra HD Audio          | Ngõ ra**HDMI** (device 3/7/8/9 = HDMI 0–3)              |
| `1` | **APE** — Audio Processing Engine | Khối xử lý audio nội bộ Tegra (nhiều virtual link ADMAIF) |

> **Không có mic/loa USB gắn vào Jetson.** Vì vậy **voice KHÔNG dùng ALSA cục bộ** — mic & loa nằm trên chính con R1, truyền qua mạng:
>
> - **Mic:** bo `.161` phát multicast `239.168.123.161:5555` → `r1_bridge` nhận (xem [Sơ đồ mạng §9.1](SO_DO_MANG_R1.md)).
> - Cấu hình `input.source: r1_multicast` trong `voice_r1/config/tuning.yaml` khớp với điều này.
> - (Có sẵn nhánh `alsa_usb` trong code phòng khi sau này cắm mic USB, nhưng hiện không dùng.)

---

## 5. Camera / thị giác

- `ls /dev/video*` → **trống**; `lsusb` không có RealSense/webcam.
- Tuy nhiên dịch vụ **`nvargus-daemon`** (ISP camera CSI của NVIDIA) **đang chạy** → phần cứng Orin NX **có khả năng** nhận camera CSI, chỉ là **hiện chưa gắn/enumerate** camera nào.
- Kết luận: hiện tại **không có luồng thị giác** trên máy này; điều khiển/voice thuần dựa trên IMU + khớp + audio.

---

## 6. Điểm hỏng vật lý cần để ý

| Bộ phận                           | Rủi ro                 | Triệu chứng                                                   |
| ----------------------------------- | ----------------------- | --------------------------------------------------------------- |
| **WiFi dongle USB (TP-Link)** | Lỏng/rớt/quá nhiệt  | Mất Internet → voice`transport_error`, `openai_ready=0`   |
| **Dây eth10 sang R1**        | Lỏng                   | Mất DDS → high-level không đọc được lowstate; mic chết |
| **CH340 USB-serial**          | Lỏng hub               | Mất kênh tới bo/sensor phụ (nếu có)                       |
| **Nhiệt**                    | Tải nặng + quạt lỗi | `tj-therm` tăng cao → throttle                              |

Lệnh kiểm tra nhanh phần cứng:

```bash
cat /proc/device-tree/model            # tên máy
free -h ; df -h /                       # RAM, đĩa
lsusb                                   # thiết bị USB (WiFi dongle, serial)
cat /proc/asound/cards                  # card âm thanh
for z in /sys/class/thermal/thermal_zone*/; do echo "$(cat $z/type)=$(($(cat $z/temp)/1000))C"; done
tegrastats                              # (Ctrl-C để thoát) tải CPU/GPU/RAM realtime
```

---

*Tài liệu tạo từ dữ liệu SSH thật ngày 2026-07-24.*
