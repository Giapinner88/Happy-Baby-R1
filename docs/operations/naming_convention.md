# Quy ước đặt tên file
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-DOC-001
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Approved / Final

Bạn có thể hình dung cấu trúc tên file như một chuỗi các "mô-đun" thông tin nối liền nhau:

| Thành phần | Định dạng | Ví dụ | Ý nghĩa |
| :--- | :--- | :--- | :--- |
| **Thời gian** | `YYYYMMDD` | `20260425` | Ngày thực hiện test |
| **Thiết bị** | `RobotID` | `R1` | Định danh robot (Unitree R1) |
| **Môi trường** | `Env` | `SIM` hoặc `REAL` | Phân biệt mô phỏng hay thực tế |
| **Nhiệm vụ** | `TaskName` | `WalkForward` | Tên kịch bản (viết liền, chữ cái đầu viết hoa) |
| **Lần thử** | `Attempt` | `001` | Số thứ tự lần thử trong ngày (luôn có 3 chữ số) |
| **Trạng thái** | `Status` | `Success` | Kết quả: Success, Fail, hoặc Abort (hủy) |

**Cấu trúc tổng quát:** `YYYYMMDD_RobotID_Env_TaskName_Attempt_Status.ext`

## Phạm vi áp dụng

Quy ước này áp dụng cho tên file log, rosbag, video test và các file ghi nhận kết quả vận hành trong toàn bộ dự án.

## Ví dụ chuẩn

* `20260425_R1_REAL_WalkForward_001_Success.mcap`
* `20260425_R1_SIM_StandUp_002_Fail.mp4`
* `20260425_R1_REAL_NetworkCheck_003_Abort.md`

## Tài liệu liên quan

* Quy trình rosbag2: [rosbag2_operation.md](rosbag2_operation.md)
* Mẫu log kiểm thử: [../templates/test_log_template.md](../templates/test_log_template.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)