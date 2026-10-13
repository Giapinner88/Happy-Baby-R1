# Safety Rules
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SAF-000
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này là trang tổng quan cho toàn bộ quy tắc an toàn của dự án. Quy tắc bên dưới áp dụng cho mọi buổi test, mọi cấu hình robot, và mọi môi trường làm việc có liên quan đến R1.

## 1. Cách sử dụng

* [SOP_v0.md](../operations/SOP_v0.md) mô tả quy trình vận hành.
* [hardware_safety_rules.md](hardware_safety_rules.md) mô tả an toàn phần cứng.
* [software_safety_rules.md](software_safety_rules.md) mô tả an toàn phần mềm.
* [network_setup_checklist.md](../operations/network_setup_checklist.md) mô tả thiết lập mạng/DDS.
* [test_log_template.md](../templates/test_log_template.md) mô tả cách ghi nhận kết quả test.

## 2. Nguyên tắc chung

* Không suy đoán quy trình an toàn nếu chưa có tài liệu tham chiếu.
* Nếu có mâu thuẫn giữa tài liệu, ưu tiên tài liệu chuyên biệt hơn.
* Mỗi buổi test phải xác định rõ người vận hành, người giám sát an toàn, và phạm vi môi trường.
* Mọi thay đổi có ảnh hưởng đến robot, mạng hoặc dữ liệu đều phải được kiểm tra và ghi log.

## 3. Khi nào phải dừng test ngay

* Robot mất thăng bằng hoặc có dấu hiệu ngã.
* Nhiệt độ, dòng điện hoặc trạng thái pin có dấu hiệu bất thường.
* Kết nối mạng hoặc DDS bị mất ổn định.
* Có va chạm cơ khí, tiếng động lạ hoặc rung bất thường.
* Phát hiện code điều khiển đang chạy sai kịch bản hoặc sai môi trường.

## 4. Tài liệu liên quan

* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)
* Bố cục phòng lab: [../Lab%20setup/laboratory_layout.md](../Lab%20setup/laboratory_layout.md)
* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
* Mẫu log kiểm thử: [../templates/test_log_template.md](../templates/test_log_template.md)
