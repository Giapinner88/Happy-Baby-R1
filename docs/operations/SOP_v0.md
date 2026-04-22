*** Mẫu SOP (Standard Operating Procedure) - SOP_v0.md  
Tài liệu này tập trung vào tính an toàn và chuẩn hóa vận hành.  
I. Mục đích  
• Thiết lập quy trình chuẩn để vận hành robot Unitree G1/R1 an toàn.  
• Đảm bảo tính nhất quán trong việc thu thập dữ liệu phục vụ Imitation Learning.  
II. Quy tắc An toàn bắt buộc (Safety First)  
• Nhân sự: Tuyệt đối không vận hành robot một mình. Phải có 1 Operator (điều khiển chính) và 1 Safety Lead (giữ nút E-Stop).  
• Môi trường: Robot chỉ được hoạt động trong Test Zone đã quy định trong sơ đồ Lab.  
• Thử nghiệm: Mọi code mới phải chạy thành công trên Simulator (MuJoCo/IsaacLab) trước khi nạp vào robot thật.  
III. Quy trình vận hành  
1. Trước khi khởi động (Pre-flight Checklist):  
o Kiểm tra ngoại quan: Khớp, pin, dây cáp.  
o Kiểm tra kết nối mạng (Ethernet/DDS) theo hướng dẫn của Integration Lead.  
2. Trong khi vận hành:  
o Tuân thủ đúng kịch bản thử nghiệm đã đề ra.  
o Safety Lead luôn quan sát sát sao để kích hoạt Emergency Stop nếu có hiện  
tượng robot ngã hoặc quá nhiệt.  
o  Khoảng cách an toàn: Do robot có cấu trúc phức tạp và sức mạnh cực lớn,  
mọi thành viên phải duy trì khoảng cách an toàn đủ xa trong suốt quá trình  
vận hành.  
o  Giới hạn tay Dexterous: * Tuyệt đối không thực hiện các cử động mạnh
như chạy hoặc test thăng bằng khi robot đang gắn tay dexterous để tránh hư
hại.  
o Khi lập trình, cần kiểm tra kỹ và tăng độ lệch vai (outward offset) để tránh tay
va chạm với thân robot.  
o  Tải trọng tay: Lưu ý rằng tải trọng tối đa của cánh tay sẽ thay đổi đáng kể
tùy thuộc vào tư thế duỗi của tay.  

3. Ghi hình

o Tư thế chân tự nhiên: Khi phát triển chương trình di chuyển, ưu tiên
giữ khớp gối thẳng hoặc gần thẳng để robot trông giống người hơn.  
o Cách thức di chuyển: * Giảm tần suất bước chân (step frequency)
xuống mức thấp nhất có thể.  
o Tránh để robot dậm chân tại chỗ khi quay phim.  
o Giữ hai bàn chân gần nhau và tránh để bàn chân bị xòe ra (splaying)
khi đi bộ.  

4. Sau khi vận hành:  
o Đưa robot về trạng thái nghỉ (Damping/Zero torque mode).  
o Lưu trữ dữ liệu log và video vào đúng cấu trúc thư mục quy định.  