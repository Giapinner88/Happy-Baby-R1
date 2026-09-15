# Đề xuất nghiên cứu — Phối hợp hai tay khi teleop gần giới hạn khả thi

**One-page · 2026-09-15 · Trạng thái: hướng nghiên cứu đề xuất; chưa triển khai hoặc kiểm chứng.**

## Bối cảnh và vấn đề

Pipeline Quest 3 → upstream R1-A5 IK → Isaac đã cho kết quả mô phỏng tốt về mặt biểu diễn chuyển động tay–đầu, tiêu biểu ở [run ngày 09-09-2026](../../experiments/r1_teleop/quest3_sim_v1/baseline/runs/t007_whole_upper_body_20260909T072030Z/metrics.json): control thực tế 27,03 Hz, mô phỏng gần thời gian thực và không có mẫu bị clamp joint limit. Run này chưa đánh giá định lượng phối hợp hai tay. Mỗi tay có 5 DoF; thân và các khớp ngoài tay–đầu được giữ cố định.

[Upstream IK](03_r1_a5_ik.md) tối ưu sai số pose từng bàn tay cùng regularization và smoothness, chưa có mục tiêu giữ quan hệ giữa hai tay. Khi cùng thực hiện một nhiệm vụ, sai số từng tay nhỏ vẫn có thể làm hỏng phối hợp: hai tay cùng lệch 2 cm giữ nguyên khoảng cách, nhưng mỗi tay lệch ra ngoài 2 cm làm khoảng cách tăng 4 cm. Hiện tượng này gần giới hạn workspace là **giả thuyết cần kiểm tra**, chưa phải lỗi đã xác nhận của baseline.

## Câu hỏi và giả thuyết nghiên cứu

**Khi hai tay không thể đồng thời đạt pose mong muốn, phân bổ sai số giữa chuyển động chung và quan hệ tương đối như thế nào để bảo toàn nhiệm vụ phối hợp mà vẫn đáp ứng thao tác của người?**

Giả thuyết: ưu tiên quan hệ hai tay theo nhiệm vụ trong vùng khó đạt target có thể giảm biến dạng phối hợp, với mức tăng sai số chuyển động chung và độ trễ nằm trong giới hạn được khai báo trước.

## Hướng phương pháp

Biểu diễn nhiệm vụ bằng frame chung của vật ảo $T_c$ và quan hệ hai bàn tay $T_{\mathrm{rel}}=T_L^{-1}T_R$, với $T_L,T_R$ cùng hệ quy chiếu. Với phép thử giữ thanh ảo, khai báo hai frame gắn tay cố định $G_L,G_R$ rồi sinh target $T_{L,d}=T_{c,d}G_L$, $T_{R,d}=T_{c,d}G_R$ Định nghĩa cách suy ra frame chung đạt được khi hai tay lệch phải được chốt trước phép đo.

Khảo sát cơ chế điều chỉnh cặp target theo khả năng robot: giữ chuyển động chung khi khả thi; cho phép sai lệch có kiểm soát khi cần bảo toàn quan hệ hai tay. Các giới hạn khớp và tính liên tục vẫn phải được thỏa mãn. Kiến trúc thuật toán chưa chốt; baseline upstream là đối chứng. Đóng góp tiềm năng nằm ở **phân bổ sai số dưới giới hạn 5 DoF và chuyển tiếp liên tục vào/ra vùng giới hạn**, không chỉ thêm một relative-pose cost.

## Thí nghiệm phân biệt đầu tiên

**Nhiệm vụ:** hai controller điều khiển một thanh ảo có quan hệ gắn tay cố định; dịch chuyển và xoay từ vùng dễ tới vùng một tay gần giới hạn, rồi quay lại. Bước đầu là phối hợp hình học trong Isaac, không mô phỏng giữ vật bằng lực. Ghi và tái sử dụng cùng trace, timestamp, calibration, điều kiện khởi tạo và cấu hình simulator.

1. Chạy baseline, xác định có sai lệch phối hợp đáng kể và lặp lại được hay không.
2. Chỉ khi có bằng chứng, xây dựng biến thể và so sánh trên cùng đầu vào; giữ đối chứng cả ở vùng baseline hoạt động tốt.
3. Đo sai số vị trí/hướng tương đối hai tay (chỉ số chính); sai số pose chung, mức sửa lệnh người, bước khớp, vận tốc/gia tốc, thời gian solve p95/p99 và tỷ lệ quá hạn (chỉ số đánh đổi). Tách target, lệnh áp dụng và trạng thái sau physics; lưu video, trace và cấu hình đã phân giải.

**Quyết định:** tiếp tục nếu giảm được sai số phối hợp mà không vượt ngân sách sai số chung, độ trễ và giới hạn chuyển động. Nếu baseline đã đạt yêu cầu, hoặc trợ giúp chỉ giữ quan hệ bằng cách gần như dừng robot, giả thuyết chưa được hỗ trợ. Ngưỡng, đoạn đánh giá và điều kiện loại run phải được thống nhất trước so sánh; tài liệu này chưa phải protocol thực thi.

## Phạm vi, nền tảng và kết quả kỳ vọng

Phạm vi là teleop hai tay với thân cố định; chưa mở rộng sang WBC, learning hoặc tối ưu cơ năng. Thanh ảo không chứng minh khả năng giữ vật thật: lực nội, trượt và compliance cần một nghiên cứu tiếp theo. Các run cũ chỉ là evidence của phương pháp/cấu hình đã chạy.

Nền tảng liên quan: [*A constrained optimization approach to virtual fixtures for multi-robot collaborative teleoperation*, IROS 2011](https://doi.org/10.1109/IROS.2011.6095056), đề xuất ràng buộc nhiều robot để hỗ trợ teleoperation. Tính mới của hướng R1 cần được đối chiếu sâu với cooperative manipulation và shared control trước khi khẳng định. Kết quả kỳ vọng đầu tiên là xác định **khi nào bám từng tay không đủ cho nhiệm vụ hai tay**, cùng bằng chứng về đánh đổi phối hợp–trung thực với lệnh người–thời gian tính toán.
