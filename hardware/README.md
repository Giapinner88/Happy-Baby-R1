# Hardware integration boundary

Mã/config chỉ dành cho robot thật, calibration hoặc deployment được đặt ở đây
khi cần. Không đưa secret, key hoặc log buổi chạy vào Git. Trước hardware run:
policy phải qua direct-eval và DDS bridge parity, sau đó theo `docs/safety/`.
