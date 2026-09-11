# Quy tắc an toàn R1 teleop

Áp dụng cho mọi phiên dùng Quest 3 với R1-A5.

- Mô phỏng/dry-run và evidence không thay thế hardware gate.
- Hardware chỉ chạy khi robot treo/cố định, có hai người: một vận hành và một
  giữ R3 E-stop.
- Xác nhận đúng robot, interface, config và source revision trước khi chạy.
- Chỉ một tiến trình được sở hữu `rt/lowcmd`; không chạy hai high-level owner
  đồng thời.
- `hb_teleop.service` phải inactive trong phiên foreground theo runbook.
- Dừng ngay khi mất mạng/state, timeout lặp lại, output vượt envelope, có rung,
  tiếng động, nhiệt, pin hoặc chuyển động bất thường.
- Không chạy trên sàn: trạng thái đó chưa được thiết lập.

Thao tác đầy đủ: [runbook hardware](../teleop/r1_quest3_teleop_hardware.md).
Các điều kiện còn thiếu: [hardware gate](../../hardware/teleop/docs/hardware_gate.md).

Xem thêm [an toàn phần cứng](hardware_safety_rules.md) và
[an toàn phần mềm](software_safety_rules.md).
