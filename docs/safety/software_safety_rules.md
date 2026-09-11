# An toàn phần mềm

- Chạy `make teleop-dry-run` và test code trước hardware pilot.
- Xác nhận branch, environment, `HOST_IP`, `ROBOT`, config và certificate; không
  đổi chúng giữa phiên.
- Sidecar không được tạo DDS command publisher. `hb_high_level` là owner
  `rt/lowcmd` duy nhất và R3 E-stop luôn ưu tiên.
- Không replay dữ liệu chưa kiểm tra vào hardware; input tổng hợp chỉ là smoke.
- Không trộn evidence upstream với coupled/differential hoặc thay đổi tiêu chí
  sau khi xem kết quả.
- Giữ log, resolved config và status của phiên hardware; không coi exit code 0
  là scientific pass.
- Dừng khi state mất freshness, sequence lỗi, process restart, output không hữu
  hạn hoặc vượt envelope.

Xem [kiến trúc hardware](../teleop/07_hardware_boundary.md) và
[hardware gate](../../hardware/teleop/docs/hardware_gate.md).
