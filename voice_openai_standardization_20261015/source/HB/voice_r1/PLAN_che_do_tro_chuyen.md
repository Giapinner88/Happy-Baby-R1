# Chế độ trò chuyện rảnh tay (F1)

## Hành vi vận hành

- **Double-click F1** (hai cú nhấn-nhả ngắn trong 0,8 giây) bật/tắt `conv_mode`.
- Mặc định sau restart là tắt: chỉ giữ **Select** mới mở mic (PTT).
- Khi bật, mic mở liên tục nếu high-level đang rảnh và remote R3-1 còn kết nối. Khi robot phát tiếng, `voice_speaking` vẫn đóng mic để chống vọng.
- Remote mất kết nối sẽ lập tức tắt PTT **và** `conv_mode`; remote kết nối lại không tự mở mic.
- **Không có âm/loa báo khi bật/tắt.** Trạng thái chỉ hiển thị qua log gate và `/run/hb/status.env` (`conv_mode=0|1`) — tránh đụng quyền ưu tiên loa của high-level.

## Voice và hội thoại chủ động

- PTT giữ cơ chế commit thủ công khi nhả Select (`turn_detection=False`).
- Hands-free chỉ được mở sau khi phiên OpenAI Realtime chuyển sang `SemanticTurnDetection(eagerness="medium")`; VAD server là nguồn duy nhất chốt lượt trong mode này, tránh hai response cho một câu.
- Khi tắt hands-free, service xóa audio đang chờ rồi trả lại session về PTT. Double-click Select vẫn là thao tác ngắt response hiện tại.
- Robot chỉ trả lời sau khi nhận được lượt nói của khách; không tự phát câu nói khi khách im lặng.
- Tính cách chỉ là pha trò/chọc yêu nhẹ nhàng, phù hợp gia đình và mẹ & bé. Không được chế giễu ngoại hình, năng lực, trẻ em, ba mẹ hay nhãn hàng; không ép khách trả lời hoặc mua hàng.

## Giao thức và theo dõi

- `hb_integration` gửi thêm `conv=0|1` trong gate datagram; voice gate parse token thiếu như `false` để tương thích sender cũ.
- `/run/hb/status.env` có thêm `conv_mode=0|1`. Dùng `health_check.sh` để xem cùng `mic_allowed` và `speaker_allowed`.
- Tuning nằm trong `config/tuning.yaml`:

  ```yaml
  conversation:
    vad_eagerness: "medium"
  ```

## Nghiệm thu trước khi deploy

1. Chạy unit tests và `hb_integration --self-test`.
2. Khi robot không armed, xác minh firmware `.161` thật sự gửi bit F1.
3. Kiểm tra F1/status/mic cùng đổi trạng thái, và nghe âm báo hai nốt đi lên khi vào chế độ auto / đi xuống khi về chế độ giữ nút (`conversation.mode_cue`); nói một câu ở chế độ auto chỉ nhận một response.
4. Kiểm tra PTT nhả Select vẫn có một response, Select double-click vẫn cắt lời, và high-level vẫn ưu tiên loa/mic.

Deploy chỉ rebuild `hb_integration`, sync `voice_r1`, rồi restart `hb_integration` và `hb_voice`; không chạm dịch vụ motor.
