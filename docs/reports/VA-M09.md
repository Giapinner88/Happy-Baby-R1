# Báo cáo VA-M09 — Hiểu lời nói và thực hiện cử chỉ

**Ngày lập:** 28/11/2026. **Phạm vi:** đối chiếu mã và tài liệu trên máy trạm, nhánh `develop`, HEAD `dfb68e652030b792d5289bea5ee26c71df8eb01c`. Checkout có thay đổi chưa commit; liên kết GitHub dùng nhánh `develop`, có thể khác bản cục bộ. Chưa xác minh trạng thái đã push, chưa SSH/deploy hoặc thử robot trong lần lập báo cáo này.

**Tài liệu vận hành:** [docs/assistant.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/assistant.md), [docs/voice.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/voice.md), [docs/controls.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/controls.md), [docs/safety.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/safety.md). Đây là báo cáo chức năng và hướng dẫn kiểm chứng, chưa phải biên bản nghiệm thu.

## 1. Mục tiêu và mức đáp ứng

**Language grounding:** hiểu câu hỏi/yêu cầu đơn giản liên quan đến mục tiêu đã xác định. Ở đây mục tiêu là hành động dựng sẵn: bắt tay, tặng quà và các cử chỉ trong danh sách cho phép.

Luồng xử lý: mic → OpenAI Realtime → `perform_gesture` với ID hành động → socket owner → controller kiểm tra điều kiện → phát clip và thu tay. `handshake` được ánh xạ tới slot 2, `give_gift` tới slot 4 trong cấu hình cục bộ tại thời điểm lập báo cáo; slot có thể thay đổi theo cấu hình.

“Tặng quà” là cử chỉ đưa/tặng quà. Chưa có bằng chứng trong luồng này về nhận diện món quà, gắp vật, điều khiển lực hay xác nhận người nhận đã lấy quà. “Bắt tay” cũng chưa chứng minh tương tác tiếp xúc có điều khiển lực.

## 2. Cách sử dụng

1. Kiểm tra mic/loa, phiên OpenAI và `./hb status`; đọc quy trình ARM trong `docs/controls.md`. Chỉ thử cử chỉ khi đã qua điều kiện an toàn.
2. Vào Dev locomotion theo tài liệu vận hành, đứng yên đủ thời gian ổn định, tay cầm còn kết nối và teleop không giữ thân trên. Voice tự quảng bá công cụ khi đủ điều kiện; giữ `gesture_control.mode: off` theo hướng dẫn, không bật cưỡng bức.
3. Giữ `SELECT`, nói “Bắt tay với mình được không?” hoặc “Tặng quà cho mình nhé”, rồi nhả. Thử từng yêu cầu riêng và chờ kết thúc/thu tay.
4. Nếu không chạy, đọc log thay vì lặp lệnh liên tục:

```bash
journalctl -u hb_voice.service -n 250 --no-pager
ls -l /run/hb/gesture_owner.sock
```

Nếu assistant đã được bật, giữ đúng cấu hình điều phối hiện hành. Không kích hoạt thêm stack thử nghiệm song song. Khi có lỗi hoặc cần dừng, người vận hành dùng điều khiển và E-stop theo SOP.

## 3. Nguồn code và vị trí GitHub

| Thành phần | Vị trí và vai trò |
| --- | --- |
| Hiểu yêu cầu, enum sáu hành động | [voice/hb_voice/gesture_tools.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/voice/hb_voice/gesture_tools.py) |
| Phiên hội thoại và công cụ | [voice/hb_voice/app.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/voice/hb_voice/app.py) |
| Owner socket | [controller/src/input/VoiceGestureOwner.hpp](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/controller/src/input/VoiceGestureOwner.hpp) |
| Gate và thực hiện cử chỉ | [controller/src/app/ApplicationUpperBody.cpp](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/controller/src/app/ApplicationUpperBody.cpp) |
| Player cử chỉ | [controller/src/motion/ArmGesturePlayer.hpp](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/controller/src/motion/ArmGesturePlayer.hpp) |
| Slot, đường dẫn asset, tham số | [controller/config/gestures.yaml](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/controller/config/gestures.yaml) |
| Backend assistant nếu sử dụng | [assistant/runtime/r1_assistant/backends.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/runtime/r1_assistant/backends.py) |
| Kiểm thử tool | [voice/tests/test_gesture_tools.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/voice/tests/test_gesture_tools.py) |

Đường native là mã HB voice/controller trong repository hiện tại. Chưa xác định tác giả hoặc commit ban đầu của từng clip bằng hồ sơ này; không gán nguồn huấn luyện không có bằng chứng. Phần assistant dùng quy trình xuất VLA nêu tại [assistant/EXPORT.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/EXPORT.json).

## 4. Kiểm chứng cần ghi

Thử yêu cầu trực tiếp, lịch sự và diễn đạt tương đương; thêm câu phủ định, trích dẫn, câu mơ hồ và hội thoại không yêu cầu hành động. Ghi transcript, ID được chọn, kết quả owner, video động tác và thu tay. Tách tỷ lệ hiểu đúng, tỷ lệ khởi chạy đúng và tỷ lệ hoàn tất. Kiểm tra từ chối khi chưa ARM, đang di chuyển hoặc teleop giữ tay.

## 5. Lưu ý và kết luận

`accepted` chỉ chứng minh owner nhận khởi chạy; lời nói “được rồi” không chứng minh robot đã làm. Không bỏ gate để ép động tác, không đặt tay/người trong vùng chuyển động khi chưa đánh giá tiếp xúc. `gestures.yaml` có thay đổi chưa commit nên phải lưu cấu hình và hash asset của buổi thử. **Kết luận: có ánh xạ lời nói tới cử chỉ bắt tay/tặng quà; chưa đủ bằng chứng nghiệm thu tương tác vật lý hoặc trao vật.**
