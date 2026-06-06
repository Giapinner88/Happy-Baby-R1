# Tích hợp Vẫy Tay (Waving Animation) vào RL Policy cho Unitree G1

Tài liệu này trình bày phương án kỹ thuật chuẩn xác và kiến trúc hệ thống để thực hiện tính năng "Vừa giữ thăng bằng (RL) vừa vẫy tay (Animation)" cho robot Unitree G1.

Giải pháp cốt lõi là sự kết hợp giữa **Keyframe Animation (Nội suy động tác tĩnh)** và **Observation Masking (Che giấu dữ liệu quan sát của AI)** nhằm đảm bảo cánh tay di chuyển mượt mà theo kịch bản nhưng không làm sập bộ điều khiển thăng bằng của robot.

---

## 1. Kiến trúc hệ thống

Hệ thống được chia làm 2 giai đoạn tách biệt để đảm bảo an toàn và dễ tinh chỉnh:

### Giai đoạn 1: Thu thập động tác (Pose Editor)
Thay vì cố gắng nắn khớp robot trong lúc mô phỏng chính đang chạy (bị nhiễu bởi trọng lực và RL Policy), một môi trường Studio giả lập tĩnh (`wave_animation/pose_editor.py`) được thiết kế riêng:
* **Môi trường phi trọng lực (`gravity = 0`):** Giữ cho cánh tay lơ lửng, không bị rớt xuống khi thả chuột.
* **Cố định trọng tâm (Pinning Base):** Khóa cứng toàn bộ phần thân (Floating Base), loại bỏ hoàn toàn các chuyển động rơi hoặc trượt.
* **Phân bổ Damping:** Đóng băng toàn bộ các khớp không liên quan (Cổ, Thân, Chân, Tay trái) với Damping cực lớn (100.0). Chỉ nới lỏng 7 khớp của Cánh tay phải (Damping = 0.5) để tự do tương tác nắn chỉnh.
* **Ghi hình (Capture):** Lưu tọa độ 7 khớp tay phải thành các khung hình tĩnh (Keyframes) và xuất ra file `wave_keyframes.json`.

### Giai đoạn 2: Tích hợp và Vận hành (Controller & Main Loop)
Bộ điều khiển vẫy tay (`wave_animation/controller.py`) được nhúng vào vòng lặp RL chính (`run98_3.py`). Khi kích hoạt (nhấn phím `T`), luồng xử lý diễn ra như sau:
1. **Nội suy chuyển động (Interpolation):** Thuật toán tính toán các góc khớp trung gian giữa các Keyframes để tạo ra quỹ đạo quét tay qua lại liên tục. Cơ chế "Blend" 1.5 giây được áp dụng ở đầu/cuối chu kỳ để cánh tay nâng lên và hạ xuống từ từ, tránh gây shock cho hệ thống vật lý.
2. **Observation Masking (Kỹ thuật Đánh lừa AI):** 
   * **Lý do:** RL Policy được huấn luyện với tư thế tay để thõng. Nếu tay bị ép giơ lên cao, Policy sẽ nhận một lượng sai số góc (`q_rel`) khổng lồ, khiến nó tính toán sai lệch lực ở chân và gây ngã.
   * **Thực thi:** Trước khi đưa trạng thái vào mạng Neural, toàn bộ dữ liệu vị trí (`q`), vận tốc (`dq`) và lực trước đó (`last_action`) của riêng 7 khớp tay phải sẽ bị gán ép về **0**. 
   * **Hiệu ứng:** Mạng Neural bị "mù" phần cánh tay và lầm tưởng cánh tay vẫn đang ép sát thân. Tuy nhiên, sức nặng của cánh tay giơ lên làm thay đổi trọng tâm thực tế, được cảm biến IMU ghi nhận. Mạng Neural thuần túy sử dụng dữ liệu IMU này để điều chỉnh đôi chân, giúp robot giữ thăng bằng hoàn hảo.
3. **Ghi đè (Override):** Mục tiêu góc (`target_q`) của mạng Neural xuất ra cho tay phải sẽ bị ghi đè hoàn toàn bằng quỹ đạo từ `WaveController`, sau đó truyền xuống Low-Level PD Controller.

---

## 2. Quy trình thao tác thực tế

**Bước 1: Đạo diễn tư thế**
1. Mở Terminal chạy: `python wave_animation/pose_editor.py`
2. Sử dụng chuột (nháy đúp vào bắp tay phải để mở thanh trượt Joint Control) để kéo tay robot lên thành các dáng vẫy.
3. Chuyển sang Terminal, nhấn `Enter` để lưu từng dáng.
4. Gõ `s` và `Enter` để lưu xuất ra file `wave_keyframes.json`.

**Bước 2: Diễn xuất cùng AI**
1. Mở Terminal 1 chạy bộ mô phỏng: `python unitree_mujoco2.py`
2. Mở Terminal 2 chạy bộ điều khiển: `python run98_3.py`
3. Nhấn phím **T** trên bàn phím để bật chế độ vẫy tay. Nhấn lần nữa để tắt. Cánh tay sẽ hoạt động mượt mà trong khi đôi chân tự động nhún nhảy giữ thăng bằng.

---

## 3. Khả năng ứng dụng thực tế (Sim-to-Real)

Phương pháp này được thiết kế tương thích 100% để triển khai lên robot G1 vật lý:
* Cấu trúc `wave_keyframes.json` ánh xạ tỷ lệ 1:1 với encoder của motor thực.
* Quy trình tạo dáng bằng giả lập (`pose_editor.py`) loại bỏ rủi ro và khó khăn khi phải nắn khớp một con robot kim loại nặng nề ở chế độ Zero-Torque.
* Kỹ thuật **Observation Masking** là bắt buộc và vẫn sẽ hoạt động chính xác khi triển khai file ONNX RL Policy lên phần cứng thực thông qua Unitree Low-Level SDK. Trọng tâm thực tế thay đổi sẽ được IMU vật lý bắt sóng và tự động xử lý.
