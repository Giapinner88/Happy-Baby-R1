# Thực hành 04 - Ghi log một buổi test
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-004
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này giúp bạn luyện cách ghi log sao cho một người khác có thể đọc lại buổi test và hiểu chính xác điều gì đã xảy ra.

## 2. Mẫu ghi nhanh

```text
Test ID: 20260427_R1_SIM_NetworkCheck_001_Success
Environment: SIM
Branch: feature/demo-run
Goal: Verify DDS connection and node startup
Result: Success
Issues: None
Artifacts: rosbag2, screenshot, short note
Next step: Increase duration to 5 minutes
```

## 3. Những gì bắt buộc phải ghi

1. Mục tiêu buổi test.
2. Môi trường test.
3. Nhánh code hoặc commit đang dùng.
4. Kết quả.
5. Bất thường hoặc lỗi phát sinh.
6. Đường dẫn tới data liên quan.

## 4. Bài tập

Hãy tự viết một log ngắn theo mẫu dưới đây:

```text
Test ID:
Date:
Operator:
Safety Lead:
Environment:
Branch/Commit:
Goal:
Steps:
Result:
Issues:
Data Links:
Next Step:
```

Sau đó đối chiếu với [../../templates/test_log_template.md](../../templates/test_log_template.md) để xem mình đã bỏ sót mục nào chưa.

## 5. Tiêu chí đạt

Một log tốt phải trả lời được các câu hỏi sau:

* Ai đã chạy test?
* Chạy ở đâu?
* Chạy cái gì?
* Có thành công không?
* Nếu lỗi, lỗi nằm ở đâu?
* Muốn lặp lại test thì cần thông tin nào?

## 6. Liên kết

* Mẫu chính thức: [../../templates/test_log_template.md](../../templates/test_log_template.md)
* Quy trình vận hành: [../SOP_v0.md](../SOP_v0.md)
* Quy trình rosbag2: [../rosbag2_operation.md](../rosbag2_operation.md)
