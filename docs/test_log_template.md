# Mẫu log buổi chạy robot

Mỗi buổi chạy robot thật ghi một bản theo mẫu này (cùng thư mục với video/log của buổi đó).

## 1. Thông tin chung

- ID buổi: `YYYYMMDD_R1_<TaskName>_<Attempt>_<Success|Fail|Abort>`
- Ngày:
- Người vận hành: ________ · Người giữ E-stop: ________
- Robot treo trên giá / đứng trên sàn:

## 2. Phiên bản

- Commit: `git rev-parse --short HEAD` (ghi rõ nếu có thay đổi chưa commit)
- Policy đi bộ: `flat_model` + SHA (`integration/config/model_manifest.conf`)
- Slot nhảy dùng trong buổi (`controller/config/dance.yaml`):
- Kết quả `make preflight`: đạt / cảnh báo (chép cảnh báo)

## 3. Nội dung

1. Mục tiêu (ví dụ: đi bộ 2 phút trên sàn phẳng, điệu slot 3, teleop trên giá):
2. Các bước đã làm:
3. Kết quả: `Success` / `Fail` / `Abort`
4. Sự cố (phần mềm, mạng/DDS, cơ khí, an toàn) và thời điểm:
5. Bước tiếp theo:

## 4. Dữ liệu

- Log: `journalctl -u hb_high_level --since "<giờ bắt đầu>"` (lưu ra file), teleop: `teleop/logs/robot_runtime/<run>/`
- Video:

Quy tắc an toàn: [safety.md](safety.md).
