# Báo cáo tổng hợp công việc đã hoàn thành
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-REP-001
**Author:** Integration & Operation Team
**Status:** Draft / Internal Use
**Ngày báo cáo:** 2026-05-05

## 1. Mục đích

Tài liệu này tổng hợp các công việc đã hoàn thành, các note kỹ thuật đã ghi nhận, và các hạng mục đã có nền tảng để tiếp tục triển khai cho hệ thống Unitree R1. Báo cáo được viết theo checklist làm việc hiện tại của nhóm để tiện theo dõi, bàn giao và đối chiếu.

## 2. Tóm tắt kết quả

Report này chỉ ghi nhận 11 task đầu tiên trong checklist đã được bạn nêu, tập trung vào nền tảng ROS 2, network/DDS, `unitree_sdk2_python`, demo cơ bản, và ba đầu việc nghiên cứu hardware communication ban đầu.

## 3. Các công việc đã hoàn thành

### 3.1. Nền tảng hệ thống và môi trường

1. Cài ROS 2 Foxy trên golden machine Ubuntu 20.04.
2. Build `unitree_ros2` workspace thành công.
3. Cấu hình Ethernet static IP và kiểm tra interface mạng.
4. Cấu hình CycloneDDS và xác nhận DDS hoạt động.
5. Cài `unitree_sdk2_python` và chạy demo DDS helloworld.
6. Chạy example đọc robot state ở chế độ giả lập.
7. Cài ROS 2 Foxy theo guide nội bộ trên máy cá nhân Ubuntu 20.04.
8. Đọc và hệ thống lại ROS 2 concepts: topic, service, action, node, lifecycle node.
9. Nghiên cứu DDS, bao gồm CycloneDDS vs FastDDS và các khái niệm QoS cơ bản.

### 3.2. Demo và kiểm thử cơ bản

1. Chạy pub/sub demo cơ bản `talker/listener` để xác nhận đường truyền ROS 2 hoạt động.
2. Xác nhận các bước network/DDS theo checklist đã được document lại để có thể replicate.
3. Chuẩn bị nền tảng cho các test case tiếp theo trên MuJoCo và hardware thật.

### 3.3. Note nghiên cứu hardware communication

1. Viết note giải thích kiến trúc giao tiếp SDK2 -> hardware thật: PC -> onboard computer -> motor controllers.
2. Tìm hiểu control modes của G1: position control, velocity control, torque control.
3. Tìm hiểu safety modes: damping mode, zero torque mode, và cách kích hoạt emergency stop qua SDK.

## 4. Checklist 11 task đã hoàn thành

Task chỉ nên được xem là hoàn thành khi có ít nhất một dẫn chứng rõ ràng: log chạy lệnh, file test, file cấu hình, hoặc note đã lưu trong repo.

| Vị trí | Task | Cách thực hiện chính | Dẫn chứng / file test | Kết quả |
| --- | --- | --- | --- | --- |
| 1 | Cài ROS 2 Foxy trên golden machine | Cài ROS 2 Foxy theo guide nội bộ trên máy Ubuntu 20.04, bảo đảm môi trường chuẩn cho nhóm | Log cài đặt ROS 2, terminal output `ros2 --version`, hoặc ảnh chụp môi trường sau khi source ROS | Hoàn tất, máy golden sẵn sàng cho các bước build và test tiếp theo |
| 2 | Build `unitree_ros2` workspace thành công | Đồng bộ source `unitree_ros2`, chạy build workspace và xác nhận package ROS 2 sinh ra đúng | Log `colcon build`, thư mục `install/` được sinh ra, hoặc output build không lỗi | Build thành công |
| 3 | Cấu hình Ethernet static IP, kiểm tra interface | Gán IP tĩnh cho interface Ethernet, kiểm tra interface đang dùng bằng lệnh hệ thống | Output `ip a` / `nmcli con show`, ảnh chụp interface, hoặc note cấu hình IP trong `network_configuration_static_ethernet.md` | Interface mạng hoạt động đúng dải IP của robot |
| 4 | Cấu hình CycloneDDS, verify DDS hoạt động | Set `RMW_IMPLEMENTATION`, trỏ `CYCLONEDDS_URI` tới file XML trong repo, kiểm tra DDS discovery | File `config/cyclonedds_config.xml`, output `ros2 doctor --report`, hoặc log discovery/pub-sub | DDS hoạt động, middleware nhận đúng cấu hình interface |
| 5 | Chạy pub/sub demo cơ bản (`talker/listener`) | Chạy demo ROS 2 chuẩn để kiểm tra publish/subscribe giữa 2 process | Terminal log của `ros2 run demo_nodes_cpp talker` và `listener`, hoặc video/screenshot demo | Pub/sub thông suốt |
| 6 | Cài `unitree_sdk2_python`, chạy DDS helloworld | Cài binding Python vào môi trường `r1_env`, chạy example helloworld để xác nhận kết nối DDS cơ bản | Test script `test/test_unitree_dds_helloworld.py`, log `RESULT: PASS`, hoặc output example helloworld | Python binding hoạt động |
| 7 | Chạy example đọc robot state (giả lập) | Chạy script/example đọc state trong môi trường sim để kiểm tra luồng dữ liệu state | Log chạy example đọc state, file script example trong `third_party/unitree_sdk2_python/`, hoặc output state nhận được | Script đọc state giả lập chạy được |
| 8 | Viết note đầy đủ giải thích network/DDS setup | Ghi lại cấu hình IP tĩnh, file CycloneDDS XML, biến môi trường, và luồng verify | File note mạng/DDS trong repo, đặc biệt `network_setup_checklist.md`, `network_configuration_static_ethernet.md`, `dds_implementation.md` | Có note giải thích setup để replicate |
| 9 | [HW Comm] Nghiên cứu cấu trúc giao tiếp SDK2 -> hardware thật: PC -> onboard computer -> motor controllers | Đọc tài liệu SDK2, ráp lại chuỗi giao tiếp từ host sang onboard computer rồi xuống motor controller | Note kiến trúc giao tiếp nội bộ hoặc file tài liệu nghiên cứu HW Comm | Có note kiến trúc giao tiếp tổng thể |
| 10 | [HW Comm] Tìm hiểu các control modes của G1: position control, velocity control, torque control | Đọc API và note cách mỗi mode tác động lên tầng điều khiển | Note phân tích control modes, bảng so sánh mode, hoặc tài liệu nghiên cứu HW Comm | Đã nhận diện được 3 mode điều khiển chính |
| 11 | [HW Comm] Tìm hiểu safety modes: damping mode, zero torque mode và cách kích hoạt emergency stop qua SDK | Rà lại các mode an toàn, phân biệt cách đưa robot về trạng thái an toàn và cơ chế emergency stop | Note safety modes, phần mô tả emergency stop qua SDK, hoặc tài liệu nghiên cứu HW Comm | Có note sơ bộ về safety mode và thao tác dừng khẩn |

### 4.1. Diễn giải ngắn theo nhóm task

1. Task 1 đến Task 4 là nhóm nền tảng hệ thống. Trình tự thực hiện là cài ROS 2, build workspace, cấu hình mạng tĩnh, rồi mới khóa CycloneDDS vào đúng interface để tránh DDS đi sai card mạng.
2. Task 5 và Task 6 là nhóm kiểm tra đường truyền tối thiểu. `talker/listener` xác nhận ROS 2 pub/sub nội bộ, còn `unitree_sdk2_python` helloworld xác nhận binding Python và DDS cùng hoạt động.
3. Task 7 và Task 8 là nhóm xác minh và ghi nhận. Một bên kiểm tra đọc state giả lập, một bên chuẩn hóa note để người khác có thể làm lại đúng các bước đã test.
4. Task 9 đến Task 11 là nhóm nghiên cứu hardware communication. Mục tiêu là hiểu luồng giao tiếp thật, các control mode khả dụng, và các mode an toàn cần nắm trước khi đi vào hardware.

### 4.2. Chi tiết thực hiện từng task

1. Task 1 - Cài ROS 2 Foxy trên golden machine
	- Cài ROS 2 Foxy theo guide nội bộ đúng phiên bản Ubuntu 20.04.
	- Kiểm tra source environment sau khi cài để bảo đảm shell có thể load ROS bình thường.
	- Dùng máy golden làm máy chuẩn để các bước build và demo sau đó có cùng môi trường.

2. Task 2 - Build `unitree_ros2` workspace thành công
	- Đưa source `unitree_ros2` vào workspace và chạy build theo quy trình ROS 2/colcon của repo.
	- Kiểm tra package sinh ra sau build để xác nhận dependency và message generation không lỗi.
	- Xác nhận workspace có thể source lại sau build để dùng cho các demo tiếp theo.

3. Task 3 - Cấu hình Ethernet static IP, kiểm tra interface
	- Chọn đúng interface Ethernet đang nối trực tiếp với robot hoặc thiết bị mạng mục tiêu.
	- Gán IP tĩnh cùng subnet với dải Unitree để tránh phụ thuộc DHCP.
	- Kiểm tra interface sau khi áp cấu hình để bảo đảm card mạng đúng trạng thái và đúng địa chỉ.

4. Task 4 - Cấu hình CycloneDDS, verify DDS hoạt động
	- Thiết lập `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` để ROS 2 dùng CycloneDDS làm middleware.
	- Trỏ `CYCLONEDDS_URI` đến file XML trong repo để ép DDS dùng đúng interface mạng.
	- Verify lại bằng các kiểm tra discovery/pub-sub nội bộ để xác nhận DDS chạy đúng lớp mạng đã chọn.

5. Task 5 - Chạy pub/sub demo cơ bản (`talker/listener`)
	- Mở 2 terminal ROS 2 chuẩn.
	- Chạy `talker` ở terminal thứ nhất và `listener` ở terminal thứ hai để kiểm tra publish/subscribe.
	- Quan sát message đi qua lại liên tục để xác nhận ROS 2 message pipeline hoạt động.

6. Task 6 - Cài `unitree_sdk2_python`, chạy DDS helloworld
	- Cài Python binding vào môi trường `r1_env` để tách biệt với Python hệ thống.
	- Chạy example helloworld của SDK2 Python để xác nhận binding import được và giao tiếp DDS cơ bản thông.
	- Kiểm tra output chạy xong không lỗi import, không lỗi tìm library, và có phản hồi từ demo.

7. Task 7 - Chạy example đọc robot state (giả lập)
	- Chạy script/example đọc state trong môi trường giả lập thay vì hardware thật.
	- Kiểm tra các trường state cơ bản có thể nhận được như thông tin IMU hoặc joint state theo example.
	- Dùng bước này để xác nhận logic đọc state đã hoạt động trước khi đi sâu vào robot thật.

8. Task 8 - Viết note đầy đủ giải thích network/DDS setup
	- Ghi lại IP tĩnh, interface đang dùng, biến môi trường ROS 2/DDS, và file XML CycloneDDS.
	- Mô tả thứ tự setup từ mạng vật lý đến middleware để người khác có thể làm lại từ đầu.
	- Ghi thêm checklist verify để khi cần hỗ trợ network setup có thể đối chiếu nhanh.

9. Task 9 - [HW Comm] Nghiên cứu cấu trúc giao tiếp SDK2 -> hardware thật
	- Đọc SDK2 và note lại tuyến giao tiếp từ PC sang onboard computer rồi xuống motor controllers.
	- Tách lớp trách nhiệm: host side, onboard compute, và actuator side để hiểu dữ liệu đi qua đâu.
	- Ghi chú đây là kiến trúc nền để về sau đối chiếu khi debug độ trễ hoặc sai lệch lệnh.

10. Task 10 - [HW Comm] Tìm hiểu các control modes của G1
	- Rà tài liệu/API để phân biệt position control, velocity control, và torque control.
	- Note ngắn tác động của từng mode lên robot để tránh chọn nhầm mode khi viết code điều khiển.
	- Lưu ý mode nào phù hợp cho test an toàn, mode nào chỉ dùng khi đã kiểm soát được feedback loop.

11. Task 11 - [HW Comm] Tìm hiểu safety modes và emergency stop qua SDK
	- Đọc các trạng thái an toàn như damping mode và zero torque mode.
	- Note cách đưa robot về trạng thái ít rủi ro nhất trước khi ngắt hoặc dừng hệ thống.
	- Xác định cách kích hoạt emergency stop qua SDK để dùng trong tình huống cần dừng khẩn.

## 5. Evidences

Mục này gom các mốc kiểm chứng tối thiểu cho từng task. Nếu thiếu một trong các evidence tương ứng, task đó chưa nên xem là hoàn thành hoàn toàn.

1. Task 1 - Cài ROS 2 Foxy trên golden machine
	- Evidence gợi ý: terminal output `ros2 --version`, log cài ROS 2, hoặc ảnh chụp môi trường sau khi source ROS.
	- File liên quan: `docs/operations/ubuntu_20_04_lts_setup_guide.md`.

2. Task 2 - Build `unitree_ros2` workspace thành công
	- Evidence gợi ý: log `colcon build`, thư mục `install/` sinh ra, hoặc output build không lỗi.
	- File liên quan: `src/unitree_ros2/README.md` và log build trong `build/` hoặc `log/`.

3. Task 3 - Cấu hình Ethernet static IP, kiểm tra interface
	- Evidence gợi ý: output `ip a`, `nmcli con show`, `ip link show` hoặc ảnh chụp interface đang dùng.
	- File liên quan: `docs/operations/network_configuration_static_ethernet.md` và `docs/operations/network_setup_checklist.md`.

4. Task 4 - Cấu hình CycloneDDS, verify DDS hoạt động
	- Evidence gợi ý: file `config/cyclonedds_config.xml`, output `ros2 doctor --report`, hoặc log discovery/pub-sub.
	- File liên quan: `config/cyclonedds_config.xml` và `docs/operations/dds_implementation.md`.

5. Task 5 - Chạy pub/sub demo cơ bản (`talker/listener`)
	- Evidence gợi ý: terminal log của `ros2 run demo_nodes_cpp talker` và `listener`, hoặc video/screenshot demo.
	- File liên quan: note/test log nếu đã lưu trong `docs/templates/test_log_template.md` theo format dự án.

6. Task 6 - Cài `unitree_sdk2_python`, chạy DDS helloworld
	- Evidence gợi ý: test script `test/test_unitree_dds_helloworld.py`, log `RESULT: PASS`, hoặc output helloworld.
	- File liên quan: `test/test_unitree_dds_helloworld.py`.

7. Task 7 - Chạy example đọc robot state (giả lập)
	- Evidence gợi ý: log chạy example đọc state, hoặc output state nhận được từ script example.
	- File liên quan: script/example trong `third_party/unitree_sdk2_python/` và các note sim liên quan nếu đã lưu.

8. Task 8 - Viết note đầy đủ giải thích network/DDS setup
	- Evidence gợi ý: tài liệu note đã lưu trong repo, đặc biệt file checklist và guide cấu hình mạng/DDS.
	- File liên quan: `docs/operations/network_setup_checklist.md`, `docs/operations/network_configuration_static_ethernet.md`, `docs/operations/dds_implementation.md`.

9. Task 9 - [HW Comm] Nghiên cứu cấu trúc giao tiếp SDK2 -> hardware thật
	- Evidence gợi ý: note kiến trúc giao tiếp nội bộ hoặc file tài liệu nghiên cứu HW Comm.
	- File liên quan: note HW Comm đã lưu trong `docs/operations/` hoặc tài liệu tương đương.

10. Task 10 - [HW Comm] Tìm hiểu các control modes của G1
	 - Evidence gợi ý: note phân tích control modes, bảng so sánh mode, hoặc file tài liệu HW Comm có phần control mode.
	 - File liên quan: note HW Comm đã lưu trong `docs/operations/` hoặc tài liệu tương đương.

11. Task 11 - [HW Comm] Tìm hiểu safety modes và emergency stop qua SDK
	 - Evidence gợi ý: note safety modes, phần mô tả emergency stop qua SDK, hoặc file tài liệu HW Comm có phần safety.
	 - File liên quan: note HW Comm đã lưu trong `docs/operations/` hoặc tài liệu tương đương.
