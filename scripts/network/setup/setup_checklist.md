 Checklist Thiết lập Mạng & DDS (Unitree R1)

Tài liệu này hướng dẫn các bước thiết lập mạng để máy tính (Dev Machine) giao tiếp được với Robot Unitree R1 thông qua giao thức DDS.

1. Kết nối Vật lý (Physical Connection)

[ ] Sử dụng cáp Ethernet (khuyên dùng Cat6 trở lên) kết nối từ cổng LAN của máy tính đến cổng Ethernet trên Robot R1.

[ ] Kiểm tra đèn tín hiệu tại cổng mạng đã sáng (Link/Act).

2. Cấu hình IP tĩnh (Static IP Configuration)

DDS yêu cầu các thiết bị trong mạng phải nằm cùng một dải IP để "thấy" nhau.

[ ] Mở phần cài đặt mạng (Network Settings) trên Ubuntu.

[ ] Chọn interface Ethernet tương ứng và chỉnh sang chế độ Manual (Static).

[ ] Thiết lập các thông số chuẩn cho hệ sinh thái Unitree:

Address: 192.168.123.x (Trong đó x là số định danh máy của bạn, ví dụ: 161, tránh trùng với IP mặc định của Robot là 161 hoặc 1).

Netmask: 255.255.255.0

Gateway: Thường để trống hoặc để 192.168.123.1.

[ ] Nhấn Apply và thực hiện tắt/bật lại interface mạng để nhận cấu hình mới.

3. Cấu hình CycloneDDS cho R1

[ ] Cài đặt package: sudo apt install ros-humble-rmw-cyclonedds-cpp.

[ ] Thiết lập biến môi trường trong file .bashrc hoặc .zshrc:

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=<đường_dẫn_đến_file_config.xml_trong_repo>


[ ] Kiểm tra file cấu hình XML (thường nằm trong scripts/network/) đảm bảo đã chọn đúng interface mạng (ví dụ: enp...).

4. Kiểm tra Kết nối (Verification)

[ ] Ping test: Chạy lệnh ping 192.168.123.1 (hoặc IP mặc định của R1) để kiểm tra thông suốt.

[ ] ROS2 Pub/Sub Demo:

Chạy ros2 run demo_nodes_cpp talker trên máy 1.

Chạy ros2 run demo_nodes_cpp listener trên máy 2.

Xác nhận hai máy nhận được tin nhắn của nhau qua mạng nội bộ.

5. Chạy thử Unitree SDK2 (SDK Verification)

[ ] Di chuyển vào thư mục unitree_sdk2_python.

[ ] Chạy bản demo HelloWorld: python3 examples/helloworld.py.

[ ] Đọc State R1: Chạy script đọc trạng thái robot (giả lập hoặc thật) để xác nhận dữ liệu IMU/Joint đổ về máy thành công.

