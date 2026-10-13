# PLAN — Đưa 3 lớp bảo vệ TeleopReceiver của bundle vào controller

> Phát hiện 2026-08-19 khi gộp bundle: `hb-teleop/Happy-Baby-R1/hardware/high_level/src/input/TeleopReceiver.hpp`
> **an toàn hơn** bản đang chạy ở `controller/src/input/TeleopReceiver.hpp`. Cùng dòng code,
> nhưng máy kia đã vá 3 điểm mà bản này chưa có.

> **Đã hoàn tất 2026-08-19:** receiver thật + twin sim đã bind loopback,
> có `Valid()`, khóa ABI 60 byte và test UDP thật cho gói hợp lệ/NaN/range/
> watchdog. Robot active đã xác minh `127.0.0.1:5560`; bảng dưới đây là
> so sánh lịch sử trước khi port.

## 1. Ba khác biệt (bundle → controller)

| # | Bundle có | controller hiện tại | Hệ quả |
|---|---|---|---|
| 1 | `addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK)` | `INADDR_ANY` | Bản đang chạy **mở cổng 5560 ra toàn mạng, không xác thực**: ai trong cùng wifi gửi đúng 60 byte magic `UTL1` là điều khiển được tay/đầu. Đây đúng là rủi ro tôi đã cảnh báo, và máy kia đã vá rồi. |
| 2 | `Valid(p)` — từ chối gói có `arm_q` không hữu hạn hoặc \|q\| > 3.5 rad, `head_*` không hữu hạn hoặc vượt clamp | nhận mọi gói đúng magic | NaN/giá trị điên vào `tgt_arm_` → lọt qua bộ làm mượt rồi ra `BlendInto`. Clamp đầu vẫn có, nhưng **tay thì không được kiểm tra gì** trước khi trộn. |
| 3 | `ArmTarget()` accessor | không có | Chỉ là tiện ích cho log/telemetry, không phải an toàn. |

## 2. Quyết định cần bạn chốt: cổng 5560 nghe ở đâu

Chuỗi Quest đang dùng **SSH → sidecar chạy TRÊN robot → UDP loopback**, nên
`INADDR_LOOPBACK` là **đủ** và không mất chức năng gì. Nhưng nó **chặn** cách phát trực
tiếp từ laptop (`teleop_send_test.py --host <IP robot>`) mà tôi đã ghi trong quy trình
nghiệm thu — cái đó sẽ không còn gửi tới được.

| Phương án | Được | Mất |
|---|---|---|
| **A. Loopback (như bundle)** | đóng hẳn lỗ hổng LAN | test bằng sidecar hoặc chạy `teleop_send_test.py` ngay trên robot; SSH `-L` không chuyển tiếp UDP |
| **B. Giữ INADDR_ANY** | test từ laptop tiện | cổng mở ra LAN, chỉ dựa vào giữ START + watchdog |
| **C. Núm cấu hình** `teleop_bind_loopback` (mặc định `true`) | an toàn khi chạy thật, mở được khi cần dev | thêm một núm nữa phải nhớ |

**Đã chọn A cho Goal B:** bind cứng loopback, đúng với sidecar chạy trên robot và giảm
bề mặt cấu hình của đường motor-target. Test thủ công phải chạy local trên robot.

## 3. Việc

1. Port `Valid(p)` sang `controller/src/input/TeleopReceiver.hpp` (vô điều kiện — không có
   lý do gì để nhận gói NaN).
2. Bind theo phương án đã chốt ở mục 2 (+ `teleop_bind_loopback` trong `Tuning` nếu chọn C).
3. **Sync twin**: `unitree_mujoco/simulate_cpp/src/core/TeleopReceiver.hpp` — file này là bản
   sinh đôi, sửa một bên phải sửa bên kia (header của nó ghi rõ).
4. Test: gói NaN / `arm_q = 9.9` / `head_yaw` vượt clamp → phải bị **bỏ**, `weight_` không tăng.

## 4. KHÔNG lấy gì khác từ `hardware/high_level` của bundle

Bản C++ trong bundle phân kỳ với `controller`: config còn `tuning.yaml` monolithic (chưa
tách `audio/dance/gestures/...`), thiếu nhiều file audio, và **không có** phần vừa làm ở
`controller` (nút START, `teleop_lock_base`, rate-limit đầu riêng). `controller` là bản
đang deploy. Chỉ lấy 3 điểm ở mục 1, không rsync đè.

---

## Phụ lục — Mã nguyên văn (bundle đã xoá 2026-08-19)

Từ `hardware/high_level/src/input/TeleopReceiver.hpp` của bundle, 3 chỗ khác `controller`:

```cpp
        // Sidecar chạy ngay trên robot qua SSH. Chỉ loopback được phép gửi
        // target; không mở cổng motor-target ra LAN.
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
```

```cpp
    const std::array<float, kNumArm>& ArmTarget() const { return cur_arm_; }
```

```cpp
            // trong ThreadFn(): thêm && Valid(buf)
            if (n == static_cast<ssize_t>(sizeof(buf)) && buf.magic == kMagic && Valid(buf)) {
    ...
    bool Valid(const Packet& p) const {
        for (float q : p.arm_q)
            if (!std::isfinite(q) || std::fabs(q) > 3.5f) return false;
        return std::isfinite(p.head_yaw) && std::isfinite(p.head_pitch) &&
               std::fabs(p.head_yaw) <= head_yaw_max_ &&
               std::fabs(p.head_pitch) <= head_pitch_max_;   // dòng cuối suy ra theo đối xứng
    }
```

Dòng cuối của `Valid()` bị cắt khi đọc; đã suy ra theo đối xứng với `head_yaw`. Nếu cần bản
gốc: repo `Happy-Baby-R1` branch `develop` trên máy `ubuntu22-X670E-Taichi`.
