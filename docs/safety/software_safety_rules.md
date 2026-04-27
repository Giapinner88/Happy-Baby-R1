# Software Safety Rules
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SAF-002
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này chỉ quy định các nguyên tắc an toàn phần mềm cho các script điều khiển, pipeline ROS 2, ghi log, mô phỏng và triển khai lên robot R1.

## 1. Nguyên tắc bắt buộc

* Mã mới phải chạy thành công trên simulator trước khi thử trên hardware thật.
* Không chạy code điều khiển nếu chưa xác nhận đúng branch, đúng môi trường và đúng file cấu hình.
* Không sửa đồng thời cả logic điều khiển lẫn cấu hình mạng trong cùng một lần test nếu chưa có kế hoạch rollback rõ ràng.
* Mọi buổi chạy thử phải được ghi nhận trong [test_log_template.md](../templates/test_log_template.md).

## 2. An toàn khi làm việc với ROS 2 và DDS

* Chỉ dùng cấu hình mạng đã kiểm tra trong [network_setup_checklist.md](../operations/network_setup_checklist.md).
* Không tự ý đổi `RMW_IMPLEMENTATION` hoặc `CYCLONEDDS_URI` trong lúc đang chạy test nếu chưa xác nhận tác động.
* Nếu topic không xuất hiện hoặc replay lệch, dừng lại để kiểm tra QoS, timestamp và clock sync trước khi chạy tiếp.
* Không ghi rosbag toàn bộ topic nếu không có lý do rõ ràng; ưu tiên theo đúng quy ước trong [rosbag2_operation.md](../operations/rosbag2_operation.md).

## 3. An toàn khi chạy script điều khiển

* Kiểm tra file cấu hình, IP robot và môi trường Python trước khi kích hoạt node điều khiển.
* Đảm bảo không có môi trường xung đột giữa Conda, ROS system packages và Python dự án.
* Không gộp thay đổi về control, logging và UI trong cùng một test mà không có cách xác định lỗi riêng lẻ.
* Nếu script xuất hiện hành vi bất thường, phải dừng ngay và chuyển robot về trạng thái an toàn.

## 4. An toàn khi ghi log và replay

* Dữ liệu log phải được đặt tên theo [naming_convention.md](../operations/naming_convention.md).
* Không replay dữ liệu chưa kiểm tra trong môi trường thật.
* Không dùng file log không rõ nguồn gốc để benchmark hoặc so sánh kết quả.
* Khi extract dataset từ rosbag, phải kiểm tra lại timestamp và sự đồng bộ sensor trước khi đưa vào pipeline AI.

## 5. Khi nào phải dừng phần mềm ngay

* Node ROS 2 bị treo, phát topic sai hoặc mất liên lạc với robot.
* Có mismatch giữa thời gian thực và simulation time.
* Script liên tục ghi lỗi, timeout hoặc restart ngoài ý muốn.
* Control output vượt ngoài ngưỡng an toàn đã đặt ra.
* Phát hiện file cấu hình hoặc binary không khớp với phiên bản đã kiểm tra.

## 6. Tài liệu liên quan

* Trang chỉ mục an toàn: [safety_rules.md](safety_rules.md)
* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)
* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
* Quy ước đặt tên file: [../operations/naming_convention.md](../operations/naming_convention.md)
* Hướng dẫn rosbag2: [../operations/rosbag2_operation.md](../operations/rosbag2_operation.md)
* Mẫu log kiểm thử: [../templates/test_log_template.md](../templates/test_log_template.md)
