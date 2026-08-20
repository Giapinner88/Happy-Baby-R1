# Cổng phần cứng cho teleop R1

Tài liệu này liệt kê những việc **phải hoàn thành** trước khi teleop được phép
ghi lệnh xuống motor thật. Chưa đóng đủ thì `HB_TELEOP_ALLOW_MOTOR_WRITE` phải
giữ nguyên `0`.

Không mục nào dưới đây được đánh dấu hoàn thành ở thời điểm tạo package này.

## 1. Bằng chứng mô phỏng

- [ ] Một run T007 đạt tiêu chí chấp nhận đã khai báo, với `scientific_outcome`
      khác `unassessed`.
- [ ] Sai số bám endpoint và tốc độ vòng điều khiển được đo và ghi lại, không
      phải suy ra.
- [ ] Chạy lại được: cùng config, cùng commit, ra cùng kết luận.

Hiện trạng: run mới nhất `t007_whole_upper_body_20260818T114338Z` vẫn là
`unassessed`, đạt 9.98 Hz so với 20 Hz yêu cầu, và phần lớn target được
*projected* chứ không hội tụ chính xác.

## 2. Đối chiếu mô hình với robot thật

- [ ] Xác nhận dấu và thứ tự khớp của FK so với pose đo được trên robot.
- [ ] Giải quyết xung đột chỉ số motor đầu giữa spec high-level nội bộ và
      interface R1-A5 của hãng (đã ghi trong method record).
- [ ] Kiểm tra `assets/R1.urdf` khớp với robot đang dùng.

## 3. Giới hạn an toàn

- [ ] Thay `max_joint_velocity_rad_s` / `max_joint_acceleration_rad_s2` bằng
      giới hạn phần cứng đã được duyệt, không dùng số tuning của mô phỏng.
- [ ] Thêm guard va chạm và moment xoắn.
- [ ] Xác định hành vi khi mất kết nối Quest: phải giữ nguyên vị trí, không rơi
      tay.
- [ ] `--disable-self-collisions` là workaround của simulator; phải xác định
      hành vi tương ứng trên phần cứng.

## 4. Quyền ghi khớp

- [ ] Chứng minh teleop và high-level không bao giờ cùng ghi một khớp.
      `hb_teleop.service` khai báo `Conflicts=hb_high_level.service`; cần kiểm
      tra thực tế chứ không chỉ dựa vào unit file.
- [ ] Xác định trạng thái robot khi chuyển qua lại giữa hai chế độ.

## 5. Quy trình vận hành

- [ ] Chạy dry-run với motor chưa enable, ghi lại lệnh sẽ phát.
- [ ] Có người giữ E-stop, không vận hành một mình.
- [ ] Ghi log theo `docs/templates/test_log_template.md` của workspace.
- [ ] Tuân thủ `docs/safety/safety_rules.md`.

## Ghi chú về IK

Bộ giải hiện dispatch cả nghiệm *projected* cho target ngoài tầm với. Trong mô
phỏng đó là lựa chọn đúng (đóng băng cả tay còn tệ hơn), nhưng trên phần cứng
nó có nghĩa là robot sẽ đi tới tư thế gần nhất thay vì từ chối — hành vi này
phải được duyệt riêng, không mặc nhiên mang từ mô phỏng sang.
