# Tích hợp Vẫy Tay (Waving Animation) vào RL Policy cho Unitree G1

Tài liệu này tổng hợp lại toàn bộ luồng thiết kế và kiến trúc hệ thống để tích hợp tính năng "Vừa giữ thăng bằng (RL) vừa vẫy tay (Animation)" cho robot Unitree G1. 

Cốt lõi của giải pháp là sự kết hợp giữa kỹ thuật **Keyframe Animation (Nội suy động tác tĩnh)** và **Observation Masking (Che giấu dữ liệu quan sát của mạng Neural)** nhằm ép cánh tay di chuyển theo ý muốn mà robot không bị mất thăng bằng.

---

## 1. Kiến trúc hệ thống và Chức năng từng file

Để đảm bảo tính module hóa và không làm ảnh hưởng đến mã nguồn gốc, toàn bộ logic vẫy tay được đóng gói vào thư mục `wave_animation/` và chỉ thực hiện các "điểm móc" (hooks) cần thiết vào file chạy chính `run98_3.py`.

### 🛠️ `wave_animation/pose_editor.py` (Môi trường Studio tạo dáng)
Việc kéo thả cánh tay trong lúc robot đang chạy mô phỏng vật lý là bất khả thi do sự can thiệp của trọng lực và các bộ điều khiển. Do đó, một môi trường độc lập chuyên dụng đã được thiết kế để nắn khớp:
* **Môi trường phi trọng lực (`gravity = 0`):** Giúp cánh tay không bị rớt xuống khi ngừng tương tác.
* **Khóa cứng (Pinning) thân robot (`d.qpos[0:7]` và `d.qvel[0:6]`):** Cố định tuyệt đối phần thân (Floating Base) lơ lửng giữa không gian. Lực tác động lên cánh tay sẽ không làm di chuyển trọng tâm của thân.
* **Phân bổ Ma sát (Damping) có chọn lọc:** Đóng băng toàn bộ các khớp không liên quan (Damping = 100), và chỉ nới lỏng đúng 7 khớp của Cánh tay phải (Damping = 0.5) để dễ dàng tinh chỉnh góc độ.
* **Ghi hình (Capture):** Lắng nghe sự kiện phím `Enter` để lưu lại tọa độ 7 khớp tay phải thành các khung hình (Keyframes) và xuất ra file `wave_keyframes.json`.

### 🧠 `wave_animation/controller.py` (Lõi xử lý chuyển động)
Chứa class `WaveController`. Đây là module được viết ra nhằm tách biệt các phép toán ma trận phức tạp ra khỏi vòng lặp RL chính.
* **Tải dữ liệu:** Tự động đọc và parse file `wave_keyframes.json`.
* **Làm mượt (Blend Transition):** Để tránh việc robot bị thay đổi trọng lượng đột ngột gây ngã, tham số `blend_duration = 1.5` giây được thiết lập. Khi kích hoạt, cánh tay sẽ mất 1.5 giây để chuyển dần từ quyền điều khiển của RL sang quỹ đạo của Animation.
* **Nội suy tuyến tính (Linear Interpolation):** Tính toán liên tục các góc khớp trung gian giữa các Keyframes (với tốc độ `0.8` giây/khung hình) để tạo ra chuyển động vẫy tay (ping-pong) mượt mà.
* **Ghi đè (Override):** Tính toán và trả về mảng `target_q` mới, trong đó 7 giá trị của cánh tay phải (index 22 đến 28) đã được thay thế.

### 🚀 `run98_3.py` (Vòng lặp điều khiển chính)
Đây là nơi hệ thống vẫy tay được tích hợp vào vòng lặp Reinforcement Learning. Điểm mấu chốt nằm ở thuật toán Masking:
1. **Lắng nghe tín hiệu:** Bắt sự kiện phím `T` để bật/tắt (Tín hiệu Gamepad đã được gỡ bỏ để tránh tình trạng "cò" analog bị kẹt gây nhiễu).
2. **Observation Masking (Kỹ thuật Đánh lừa AI - Cốt lõi chống ngã):**
   * **Vấn đề:** Mạng Neural (RL Policy) hiện tại được huấn luyện với tư thế hai tay để thõng. Nếu cánh tay bị ép giơ lên cao, mạng Neural sẽ nhận được độ lệch góc (`q_rel`) cực kỳ lớn. Sự khác biệt quá xa so với phân phối dữ liệu huấn luyện (out-of-distribution) này sẽ khiến AI xuất ra các lệnh torque sai lệch cho đôi chân, làm robot ngã sập.
   * **Giải pháp:** Khi chế độ vẫy tay kích hoạt, thao tác "đắp mặt nạ" (mask) sẽ được áp dụng lên mảng quan sát (Observation). Các giá trị góc hiện tại (`q`), vận tốc (`dq`) và lệnh trước đó (`last_action`) của riêng cánh tay phải sẽ bị gán ép về **0**. 
   * **Kết quả:** Mạng Neural tạm thời không nhận thức được chuyển động của cánh tay phải và xử lý như thể cánh tay vẫn đang ép sát thân. Tuy nhiên, sức nặng của cánh tay giơ lên sẽ làm thân robot hơi chúi về trước. Cảm biến IMU ghi nhận độ nghiêng thực tế này, và mạng Neural sẽ thuần túy dựa vào IMU để điều chỉnh các khớp chân nhằm bảo vệ trọng tâm. Robot giữ thăng bằng hoàn hảo!
3. **Cập nhật Lệnh Motor:** Ghi đè `target_q_arr` trước khi gửi xuống bộ PD controller cấp thấp.

---

## 2. Chiến lược triển khai lên G1 Thực tế (Sim-to-Real)

Phương pháp thiết kế này hoàn toàn có thể áp dụng 100% lên robot G1 vật lý. Quy trình triển khai thực tế như sau:

### Tái sử dụng dữ liệu Keyframe
* Các giá trị góc khớp (`q`) trong MuJoCo đã được Unitree ánh xạ (map) tỷ lệ 1:1 với encoder của động cơ thực.
* Không cần thiết phải đặt robot thật ở chế độ Zero-Torque để kéo tay và record (việc này tốn sức và thiếu an toàn). Thay vào đó, có thể dùng `pose_editor.py` trong Simulator để thiết kế các tư thế. File `wave_keyframes.json` sinh ra từ máy tính sẽ được sử dụng trực tiếp trên bộ nhớ của robot G1 thật.

### Vận hành với Low-Level SDK
* Thuật toán `WaveController` với cơ chế nội suy tuyến tính (Interpolation) sẽ chạy bình thường trên máy tính nhúng của robot. Mảng `target_q_arr` sau khi override sẽ được gửi xuống các motor thông qua Low-level SDK.
* **Bắt buộc áp dụng Observation Masking:** Khi deploy file `.onnx` (RL Policy) này lên robot thật, đoạn code "đắp mặt nạ" (che giấu `q`, `dq` của tay phải) phải được giữ nguyên. Trên thực tế, IMU vật lý ở bụng robot sẽ đo lường sự thay đổi trọng tâm thật khi cánh tay kim loại dơ lên, và Policy sẽ điều khiển đôi chân vật lý để trụ vững y hệt như những gì diễn ra trong môi trường giả lập.
