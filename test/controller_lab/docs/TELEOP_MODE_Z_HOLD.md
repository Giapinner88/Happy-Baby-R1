# Mode Z teleop: giữ chân và eo trên giá treo

`controller` vẫn là tiến trình duy nhất publish `rt/lowcmd`. Tính năng này
không thêm motor owner: nó chỉ thay các lệnh zero-torque của chân và eo bằng
PD giữ đúng encoder được chốt tại cạnh teleop chuyển từ inactive sang active.

## Phạm vi

- Giữ chân IDL 0–11 và eo IDL 12–13.
- Tay IDL 15–19/22–26 và đầu IDL 29/30 vẫn do teleop điều khiển.
- Các slot IDL không có mapping vẫn zero torque.
- Encoder không hữu hạn làm riêng khớp đó zero torque.
- Compile-time check chặn một joint vừa nằm trong tập giữ vừa thuộc teleop.

Latch được xóa ngay khi deadman/watchdog mất authority, teleop bị tắt, vào
`IDLE`, hoặc vào lại `ZERO TORQUE`. Vì vậy chân và eo không tiếp tục được cấp
dòng sau khi phiên teleop kết thúc.

## Cấu hình

Các khóa nằm trong `config/gestures.yaml`:

```yaml
teleop_lock_others_enabled: false
teleop_lock_kp: 20.0
teleop_lock_kd: 3.0
teleop_lock_max_rate_rad_s: 0.20
```

Mặc định `false` để giữ nguyên hành vi đang chạy. Chỉ bật khi robot đã treo hoặc
cố định trên giá và có người giữ E-stop. Bật khóa là cấp dòng cho chân và eo;
không dùng tính năng này để đỡ robot đứng trên sàn.

Nếu chân rung hoặc kêu, dừng phiên, nhả authority và đánh giá lại `kp/kd` trên
giá treo trước lần thử tiếp theo.

## Kiểm chứng local

`lowcmd_upper_body_zero_torque_test` kiểm cả hai nhánh:

- khóa tắt: chân/eo/slot không dùng vẫn zero torque;
- khóa bật: chỉ IDL 0–13 nhận target/gain đã chốt;
- encoder NaN: joint lỗi vẫn zero torque;
- nhả latch: toàn bộ IDL 0–13 trở lại zero torque;
- tay/head giữ nguyên `arm_valid`, joint mapping, slew limit và CRC.

Local test/preflight không thay thế nghiệm thu trên robot thật.
