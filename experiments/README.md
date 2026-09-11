# Experiments

Workspace active chỉ giữ một experiment: [baseline Quest 3 teleop tay/đầu](r1_teleop/quest3_sim_v1/experiment.md).
Các protocol sàng lọc T001–T006 và placeholder T008 đã hoàn thành vai trò phát
triển hoặc không còn có implementation active; chúng được bỏ khỏi working tree
và vẫn truy được trong Git history.

Baseline dùng `runs/<run-id>/` bất biến. Mỗi run canonical phải đủ data,
metrics, figures, video và `artifact_manifest.json`; artifact gate trả lỗi nếu
thiếu. Không sửa hoặc xoá run hoàn chỉnh chỉ để giảm dung lượng; chỉ dọn
`.staging`, file `*.tmp` và output smoke có thể tái tạo khi đã xác nhận không
phải evidence.

Schema record dùng chung thuộc `evidence/`. Registry chỉ là chỉ mục của
experiment active, không tự suy diễn trạng thái khoa học từ exit code.
