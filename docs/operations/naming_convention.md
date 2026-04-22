### Quy ước đặt tên File (Naming Convention)

Bạn có thể hình dung cấu trúc tên file như một chuỗi các "mô-đun" thông tin nối liền nhau:

| Thành phần | Định dạng | Ví dụ | Ý nghĩa |
| :--- | :--- | :--- | :--- |
| **Thời gian** | `YYYYMMDD` | `20260425` | Ngày thực hiện test |
| **Thiết bị** | `RobotID` | `G1` | Định danh robot (Unitree G1) |
| **Môi trường** | `Env` | `SIM` hoặc `REAL` | Phân biệt mô phỏng hay thực tế |
| **Nhiệm vụ** | `TaskName` | `WalkForward` | Tên kịch bản (viết liền, chữ cái đầu viết hoa) |
| **Lần thử** | `Attempt` | `001` | Số thứ tự lần thử trong ngày (luôn có 3 chữ số) |
| **Trạng thái** | `Status` | `Success` | Kết quả: Success, Fail, hoặc Abort (hủy) |

**Cấu trúc tổng quát:** `YYYYMMDD_RobotID_Env_TaskName_Attempt_Status.ext`