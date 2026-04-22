Thiết lập mạng trong dự án robot humanoid (cụ thể là Unitree G1) không đơn thuần là việc "kết nối Internet", mà nó đóng vai trò là **hệ thần kinh** truyền dẫn tín hiệu giữa bộ não (máy tính của bạn) và cơ bắp (các mô tơ của robot).

Dưới đây là 4 tác dụng cốt lõi của việc thiết lập mạng mà bạn cần nắm vững trong Sprint 2:

### 1. Đảm bảo tính ổn định của đường truyền (Static IP)
Trong môi trường phát triển, việc để IP động (DHCP) rất nguy hiểm vì địa chỉ robot có thể thay đổi mỗi khi khởi động lại.
* **Tác dụng:** Thiết lập **Static IP** giúp máy tính của bạn và robot luôn "tìm thấy nhau" tại một địa chỉ cố định[cite: 72]. 
* **Ví dụ:** Nếu không có IP tĩnh, script điều khiển của bạn sẽ báo lỗi "Robot not found" chỉ vì cục router vừa cấp cho robot một địa chỉ mới, khiến toàn bộ quy trình vận hành bị gián đoạn.

### 2. Truyền tải dữ liệu thời gian thực (CycloneDDS)
Robot G1 sử dụng chuẩn giao tiếp DDS (Data Distribution Service) chạy trên nền Ethernet[cite: 221].
* **Tác dụng:** Cấu hình **CycloneDDS** giúp tối ưu hóa việc gửi nhận gói tin với độ trễ thấp nhất có thể (low latency)[cite: 72]. 
* **Ví dụ:** Để robot giữ thăng bằng, lệnh điều khiển cần được gửi đi ở tần số cao (khoảng $500Hz$ đến $1000Hz$)[cite: 228]. Nếu mạng không được cấu hình chuẩn (ví dụ bị nghẽn hoặc mất gói tin), lệnh "giữ thăng bằng" sẽ đến chậm, dẫn đến việc robot bị ngã ngay lập tức.



### 3. Phục vụ việc ghi nhật ký dữ liệu (Data Logging)
Đây là nhiệm vụ chính của bạn (Việt Anh) khi sử dụng `rosbag2`.
* *Tác dụng:** Mạng ổn định cho phép bạn "nghe lén" toàn bộ các Topic (dữ liệu cảm biến, vị trí khớp, IMU) đang chạy trên robot để ghi lại thành file log[cite: 98].
* **Ví dụ:** Khi Giáp chạy code điều khiển, nhờ mạng đã thiết lập, bạn có thể đứng từ xa dùng máy tính của mình chạy lệnh `rosbag2 record` để thu thập toàn bộ dữ liệu đó mà không làm ảnh hưởng đến hiệu suất của robot.

### 4. Kích hoạt các chế độ an toàn (Safety & Emergency Stop)
Mạng là con đường duy nhất để bạn gửi lệnh dừng khẩn cấp từ xa.
* **Tác dụng:** Thông qua SDK chạy trên nền mạng, nhóm có thể kích hoạt các chế độ như **Damping mode** (chế độ giảm chấn) hoặc **Zero torque mode** (chế độ không mô-men xoắn) khi thấy robot có dấu hiệu mất kiểm soát[cite: 80, 227].
* **Ví dụ:** Nếu robot gặp sự cố phần mềm và bắt đầu quay cuồng, một kết nối mạng chuẩn cho phép bạn ấn nút E-Stop trên máy tính để ngay lập tức ngắt lực các mô tơ, bảo vệ phần cứng robot.

---

**Tóm lại:** Nếu thiết lập mạng sai, robot sẽ giống như bị "đơ" hoặc "phản ứng chậm" – điều cực kỳ nguy hiểm đối với một robot nặng và phức tạp như G1. [cite_start]Việc bạn viết **Checklist thiết lập mạng** [cite: 101] chính là để đảm bảo bất kỳ ai trong nhóm khi kết nối vào robot cũng đều có một đường truyền "sạch" và an toàn.

Bạn đã cài đặt thử công cụ `ping` hoặc `ros2 topic list` để kiểm tra kết nối giữa hai máy tính trong mạng nội bộ chưa?