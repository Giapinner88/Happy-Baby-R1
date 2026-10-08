# An toàn vận hành R1

Áp dụng cho mọi buổi chạy robot thật. Nếu tài liệu khác mâu thuẫn với trang này, dừng lại và hỏi; không suy đoán quy trình an toàn.

## 1. Quy tắc bất biến

1. **Một chủ `rt/lowcmd` duy nhất:** `hb_high_level` (`run_r1`). Không chạy hai bản `run_r1` (`pgrep -a run_r1` phải ra đúng 1 dòng). Teleop, voice, integration không bao giờ publish lệnh động cơ.
2. **Có người giữ E-stop**, không vận hành một mình. Mỗi buổi xác định rõ người vận hành và người giám sát an toàn.
3. **Bàn giao từ controller của hãng** đúng thứ tự: `L2+R2` (một lần) → giữ `R1+R2` ~1 s. `run_r1` từ chối arm nếu không thấy controller hãng (chống arm mù khi DDS lỗi).
4. **Teleop chỉ khi robot treo/cố định trên giá.** Chưa có bằng chứng cho phép teleop khi robot đứng trên sàn (xem [teleop.md](teleop.md)).
5. **Không đặt secret trong repo.** API key và cấu hình riêng robot nằm ở `/etc/hb/stack.env`; deploy không ghi đè file này.
6. **Ghi log mỗi buổi** theo [test_log_template.md](test_log_template.md).

## 2. Trước khi chạm vào robot

- Robot đã ở trạng thái an toàn và cô lập khỏi lệnh điều khiển đang chạy.
- Không đứng trong vùng quét hoặc vùng rơi của robot khi chưa có tín hiệu an toàn.
- Không ép khớp bằng tay khi robot có điện; không giữ các khớp đang chuyển động.
- Pin: không dùng pin phồng/nóng/hư hỏng; không để pin, dây nguồn, cáp mạng nằm lỏng trên lối đi. Không cắm/rút cáp khi robot đang chạy nếu chưa được cho phép.

## 3. DISARMED trước khi dừng, restart hoặc calib

`IDLE` và `ZERO TORQUE` **vẫn phát gói lệnh động cơ** — không phải DISARMED.

1. Đỡ/treo robot trước khi vào ZERO TORQUE (Mode Z).
2. Nhả deadman Quest và mọi nút R3. Chờ teleop thu tay hết.
3. Robot đứng yên, giữ đúng `R1+R2` trong `arm_hold_s` (hiện 1 s).
4. Chờ `DISARMED` / `high_armed=0`, rồi mới `make stop-high` (trước auto-calib) hoặc `make restart-all`.

`make stop-all`, `make stop-high`, `make restart-all` tự từ chối khi chưa DISARMED. Không dùng `systemctl stop hb_high_level` trực tiếp khi robot còn armed.

Release bị chặn nếu: còn lệnh UDP mới trong hàng đợi, teleop đang active/thu tay, LowState hỏng/cũ, gyro > 0.15 rad/s, hoặc tốc độ khớp > 0.10 rad/s (kể cả khớp đầu). Phần mềm **không** biết robot có đang được đỡ hay không. Arm lại cần nhả hết nút, giữ lại từ đầu và thỏa lại cổng quan sát controller hãng.

E-stop (`L2+B`) là damping, vẫn armed. `L2+B+Y` đồng thời kết thúc ở ZERO TORQUE; vào trạng thái mềm hoàn toàn vẫn cần đỡ robot.

## 4. Giới hạn đầu (tạm thời)

| Trục | Min | Max |
|---|---:|---:|
| Pitch (IDL 29) | −0.14 rad | +0.64 rad |
| Yaw (IDL 30) | −0.65 rad | +0.44 rad |

Làm tròn vào trong từ 66 774 frame encoder đã ghi; **chưa** phải giới hạn cơ khí/nhiệt/tải đã chứng nhận. Mọi nguồn lệnh đầu (gesture, teleop, Mode Z) đi qua cùng bộ giới hạn trong `LowCmdSender`.

## 5. Dừng ngay khi

- Robot mất thăng bằng, có dấu hiệu ngã, va chạm cơ khí, tiếng rít, rung, mùi khét hoặc nhiệt bất thường.
- Pin tụt nhanh bất thường, các khớp phản ứng không đồng nhất.
- Mất ổn định mạng/DDS, hoặc log báo lỗi/timeout/restart liên tục.
- Target teleop lệch với chuyển động người vận hành.
- Có người đi vào vùng test.
- Phát hiện cấu hình hoặc binary không khớp phiên bản đã kiểm tra (`make preflight` báo lỗi).

## 6. Thay đổi phần mềm

- Mã mới phải qua `make test` và `make preflight` trước khi chạy trên robot.
- Không đổi đồng thời logic điều khiển và cấu hình mạng trong cùng một lần thử nếu chưa có kế hoạch rollback (`make rollback` từ máy dev — xem [deploy.md](deploy.md)).
- Đổi policy theo [policies.md](policies.md); không chạy policy chưa có checksum trong manifest.

## 7. Chưa được nghiệm thu trên phần cứng

Kiểm thử local (CTest, test hợp đồng) chỉ bao phủ hợp đồng phần mềm, không phải thăng bằng vật lý hay firmware động cơ. Còn mở: thời gian thực trên PC2, bàn giao khi robot được đỡ, tầm/tải khớp đầu, nghiệm thu E-stop phần cứng, hành vi mất tay cầm trong Mimic/đứng dậy/nằm xuống.
