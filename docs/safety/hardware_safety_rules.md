# Hardware Safety Rules
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SAF-001
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này chỉ quy định các nguyên tắc an toàn phần cứng khi thao tác với robot R1, nguồn điện, dây cáp, pin và môi trường vật lý xung quanh robot.

## 1. Trước khi chạm vào robot

* Chỉ tiếp cận robot khi robot đã ở trạng thái an toàn theo quy trình vận hành.
* Xác nhận robot đã được cô lập khỏi các lệnh điều khiển đang chạy.
* Không đứng trong vùng quét hoặc vùng rơi của robot khi chưa có tín hiệu an toàn.
* Kiểm tra khu vực sàn, vật cản, bàn ghế và khoảng cách an toàn quanh robot.

## 2. Quy tắc với cơ cấu cơ khí

* Không ép khớp bằng tay khi robot đang có điện.
* Không thử bẻ, xoay hoặc giữ lại các khớp đang di chuyển.
* Không vận hành các động tác mạnh khi robot đang gắn tay dexterous hoặc các phụ kiện có nguy cơ va chạm.
* Khi thay đổi pose, phải giữ khoảng cách an toàn với tay, vai, hông và đầu robot.

## 3. Quy tắc về pin và nguồn

* Chỉ sạc pin ở khu vực sạc được chỉ định trong [laboratory_layout.md](../Lab%20setup/laboratory_layout.md).
* Không để pin, dây nguồn hoặc adapter nằm lỏng trên lối đi.
* Không dùng pin có dấu hiệu phồng, nóng bất thường hoặc hư hỏng vật lý.
* Trước khi đấu nối hoặc tháo nguồn, phải xác nhận trạng thái robot và quyền kiểm soát của Safety Lead.

## 4. Quy tắc về cáp và mạng

* Cáp Ethernet phải được cố định gọn, tránh vướng vào bánh xe, chân robot hoặc người vận hành.
* Không cắm/rút cáp mạng trong lúc robot đang chạy thử nếu chưa được cho phép.
* Khi kiểm tra thiết bị mạng, ưu tiên xác minh trên máy trạm trước, không tiếp cận robot khi chưa cần thiết.

## 5. Dấu hiệu phải dừng ngay

* Có mùi khét, tiếng rít, rung động hoặc nhiệt độ bất thường.
* Robot mất ổn định cơ học hoặc đứng sai tư thế nguy hiểm.
* Pin tụt nhanh bất thường hoặc robot phản ứng không đồng nhất giữa các khớp.
* Có người đi vào Test Zone hoặc vùng an toàn bị xâm phạm.

## 6. Tài liệu liên quan

* Trang chỉ mục an toàn: [safety_rules.md](safety_rules.md)
* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)
* Bố cục phòng lab: [../Lab%20setup/laboratory_layout.md](../Lab%20setup/laboratory_layout.md)
* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
