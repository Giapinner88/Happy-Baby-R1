# Báo cáo VA-M10 — Tìm và chỉ vào đối tượng được yêu cầu

**Ngày lập:** 28/11/2026. **Phạm vi:** đối chiếu mã và tài liệu trên máy trạm, nhánh `develop`, HEAD `dfb68e652030b792d5289bea5ee26c71df8eb01c`. Checkout có thay đổi chưa commit; liên kết GitHub dùng nhánh `develop`, có thể khác bản cục bộ. Chưa xác minh trạng thái đã push, chưa SSH/deploy hoặc thử robot trong lần lập báo cáo này.

**Tài liệu vận hành:** [docs/assistant.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/assistant.md), [docs/voice.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/voice.md), [docs/controls.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/controls.md), [docs/safety.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/safety.md). Đây là báo cáo chức năng và hướng dẫn kiểm chứng, chưa phải biên bản nghiệm thu.

## 1. Mục tiêu và mức đáp ứng

**Language grounding:** hiểu mô tả mục tiêu, tìm đúng vật và chỉ tay vào vật đó. Ngôn ngữ được nối với nhận diện thị giác và hành động chỉ.

Luồng chính: `find_and_point` → assistant → P4/P5 tìm kiếm → grounding ảnh → chọn mục tiêu → điều khiển đầu/tay qua owner. Theo tài liệu, robot quét đầu ±25°; nếu chưa thấy, có thể quay thân một lượt 180° rồi quét lại. Vì vậy tác vụ có thể làm thân robot chuyển động, không chỉ nâng tay.

Khi có nhiều vật phù hợp, hệ thống có nhánh hỏi lại. Chưa bảo đảm hợp nhất danh tính một vật qua nhiều ảnh, chọn vật gần nhất hoặc đạt độ chính xác chỉ tay định lượng. Backend ghi `pointing_accuracy_verified: false` ngay cả khi kết quả tìm là `found`.

## 2. Cách sử dụng

Trước khi dùng, người vận hành phải hoàn tất quy trình cài và kiểm tra tại mục 4 của `docs/assistant.md`: có Internet, `OPENAI_API_KEY` trong `/etc/hb/stack.env`, camera/mic/loa hoạt động và các dịch vụ đúng phiên bản. Trên robot, có thể kiểm tra chỉ đọc:

```bash
systemctl is-active hb_high_level hb_voice r1-p5-agent hb_assistant
journalctl -u hb_assistant -u hb_voice -n 100 --no-pager
```

Không bật chế độ thực thi chỉ để vượt lỗi. Cấu hình đóng gói mặc định `shadow`; việc kích hoạt phải theo tài liệu vận hành, khi robot được đỡ và DISARMED. Khi chạy chuyển động, cần preflight, người giữ E-stop và nhật ký thử. `hb_high_level` là tiến trình duy nhất ghi `rt/lowcmd`.

1. Hoàn tất cài đặt search/assistant theo `docs/assistant.md`, dùng `walk_policy` đã được kiểm tra và ghim cấu hình đúng policy. Chọn R2+A; cấp quyền đầu/tay bằng START theo tài liệu.
2. Bố trí vật mục tiêu có tên/đặc điểm rõ, ví dụ thẻ số 9 hoặc bình cứu hỏa; chừa trống vùng quay thân và vùng vung tay.
3. Giữ `SELECT`, nói “Tìm và chỉ vào số 9”, rồi nhả.
4. Nếu robot hỏi phân biệt, trả lời “bên trái”, “bên phải” hoặc đặc điểm phân biệt. Không đưa lệnh thứ hai trong khi robot đang bận.
5. Quan sát vật robot chọn và hướng tay; khi cần nói “dừng lại”, đồng thời sẵn sàng lấy quyền bằng tay cầm/E-stop. Yêu cầu hủy chưa phải xác nhận robot đã dừng.

## 3. Nguồn code và vị trí GitHub

| Thành phần | Vị trí và vai trò |
| --- | --- |
| Tool lời nói | [assistant/runtime/r1_assistant/voice.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/runtime/r1_assistant/voice.py) |
| Điều phối tìm/chỉ | [assistant/runtime/r1_assistant/backends.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/runtime/r1_assistant/backends.py) |
| Search server | [assistant/app/experiments/r1_object_grounding/p5_voice_search/server.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/experiments/r1_object_grounding/p5_voice_search/server.py) |
| Chiến lược quét | [assistant/app/experiments/r1_object_grounding/p4_search/search.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/experiments/r1_object_grounding/p4_search/search.py) |
| Grounding ảnh | [assistant/app/ai_modules/vision/openai_grounding.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/ai_modules/vision/openai_grounding.py) |
| Điều khiển mục tiêu | [assistant/app/agent/target_control/control.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/agent/target_control/control.py) |
| Cấu hình tìm kiếm | [assistant/config/robot.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/config/robot.json) |

Phần assistant được xuất từ dự án VLA, nhánh `vla`, commit nguồn `d422befb4d8cd5d566e4f4f8a588e3a644b67669`; [assistant/EXPORT.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/EXPORT.json) ghi `source_has_uncommitted_changes: true`. Vì vậy commit nguồn một mình không tái dựng đủ gói: cần giữ snapshot và hash từng file trong manifest. Sửa tại nguồn VLA rồi xuất lại bằng `python3 scripts/interaction/export_r1_assistant_hb.py --hb-root <HB> --force`; không sửa tay gói `assistant/`.

[object_search/README.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/object_search/README.md) mô tả gói search cũ và agent. Theo tài liệu assistant, khi assistant bật, mã P4 trong `assistant/app` mới là đường tác vụ giọng nói; không lấy bản `object_search/` cũ làm đại diện cho toàn bộ luồng mới.

## 4. Kiểm chứng cần ghi

Định trước tập vật và tiêu chí đúng. Thử lần lượt vật phía trước, lệch góc, phía sau, vật vắng mặt và hai vật giống nhau. Cố định ánh sáng/khoảng cách trước khi mở rộng điều kiện. Lưu ảnh, transcript, kết quả grounding, `search/<lượt>/events.jsonl`, `result.json` và video hướng tay. Tính riêng tỷ lệ tìm đúng, chọn đúng sau hỏi lại, chỉ đúng và độ trễ. Sai số chỉ tay phải đo với quy ước hình học và ngưỡng đã chốt, không chỉ suy từ trạng thái `found`.

## 5. Lưu ý và kết luận

Chỉ một owner phát lệnh motor; không chạy Quest và search sender xung đột. Theo tài liệu ngày 27/11, tính năng đã chạy trên stack thử nghiệm ngày 21/11 nhưng bản tích hợp HB chưa được kiểm chứng trên robot. **Kết luận: có chuỗi hiểu–tìm–chỉ và xử lý hỏi lại; độ chính xác chỉ và kết quả triển khai HB cần được đo, chưa kết luận đạt nghiệm thu.**
