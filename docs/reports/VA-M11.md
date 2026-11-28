# Báo cáo VA-M11 — Đi tới vật được chọn khi vật đổi vị trí

**Ngày lập:** 28/11/2026. **Phạm vi:** đối chiếu mã và tài liệu trên máy trạm, nhánh `develop`, HEAD `dfb68e652030b792d5289bea5ee26c71df8eb01c`. Checkout có thay đổi chưa commit; liên kết GitHub dùng nhánh `develop`, có thể khác bản cục bộ. Chưa xác minh trạng thái đã push, chưa SSH/deploy hoặc thử robot trong lần lập báo cáo này.

**Tài liệu vận hành:** [docs/assistant.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/assistant.md), [docs/voice.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/voice.md), [docs/controls.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/controls.md), [docs/safety.md](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/docs/safety.md). Đây là báo cáo chức năng và hướng dẫn kiểm chứng, chưa phải biên bản nghiệm thu.

## 1. Mục tiêu và mức đáp ứng

**End-to-end interaction:** nghe/hiểu → quan sát → chọn mục tiêu → trả lời → hành động, có kiểm tra lại khi vật thay đổi vị trí.

Luồng chính: `approach_object` → tìm và hướng về vật → grounding/depth DAC → tiếp cận theo từng đoạn → quan sát lại → tìm lại nếu cần. Lệnh tiếp cận gửi qua APV tới `hb_high_level`; assistant không phát motor trực tiếp.

Mã có nhánh phục hồi khi mất mục tiêu hoặc khi vòng đi kết thúc bằng ước lượng quãng đi: quan sát mới, có thể tìm lại rồi tiếp cận chỗ mới. Đây chưa phải khả năng bám vật chuyển động liên tục. `object_moved` là cờ của luồng phục hồi; bản thân cờ không chứng minh đo được chuyển động thật của vật.

## 2. Cách sử dụng

Trước khi dùng, người vận hành phải hoàn tất quy trình cài và kiểm tra tại mục 4 của `docs/assistant.md`: có Internet, `OPENAI_API_KEY` trong `/etc/hb/stack.env`, camera/mic/loa hoạt động và các dịch vụ đúng phiên bản. Trên robot, có thể kiểm tra chỉ đọc:

```bash
systemctl is-active hb_high_level hb_voice r1-p5-agent hb_assistant
journalctl -u hb_assistant -u hb_voice -n 100 --no-pager
```

Không bật chế độ thực thi chỉ để vượt lỗi. Cấu hình đóng gói mặc định `shadow`; việc kích hoạt phải theo tài liệu vận hành, khi robot được đỡ và DISARMED. Khi chạy chuyển động, cần preflight, người giữ E-stop và nhật ký thử. `hb_high_level` là tiến trình duy nhất ghi `rt/lowcmd`.

Robot cần thêm mô hình DAC/TensorRT phù hợp ARM64 trong `assistant/models/dac01`, calibration và controller có approach receiver. Dùng quy trình `prepare/check/enable` ở `docs/assistant.md`; không sao chép engine x86 sang Jetson và không tự sửa `execute` để bỏ gate.

1. Chuẩn bị khu vực thử trống, người giữ E-stop và vật rõ ràng. Chọn R2+A, quyền đầu/tay và quyền thân đúng theo tài liệu.
2. Giữ `SELECT`, nói “Đi tới bình cứu hỏa”, rồi nhả. Nếu có nhiều vật, trả lời câu hỏi phân biệt.
3. Chạy ca vật đứng yên trước. Chỉ sau khi ca này được kiểm tra mới làm ca dời vật có giám sát; không bước vào đường robot. Quy định trước thời điểm và vị trí dời.
4. Theo dõi robot dừng/quan sát/tìm lại. Không suy từ câu “Tới nơi rồi” rằng khoảng cách thật đã được đo đúng.
5. Nói “dừng lại” khi cần; tay cầm/E-stop là phương tiện can thiệp của người vận hành. Owner thu hồi quyền thì không để tác vụ tự mở lại phiên đi.

## 3. Nguồn code và vị trí GitHub

| Thành phần | Vị trí và vai trò |
| --- | --- |
| Tool và câu trả lời | [assistant/runtime/r1_assistant/voice.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/runtime/r1_assistant/voice.py) |
| Chuỗi tìm–đi–tìm lại | [assistant/runtime/r1_assistant/backends.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/runtime/r1_assistant/backends.py) — `_approach`, `_face`, `_approach_round` |
| Runtime camera/depth/odom | [assistant/app/experiments/r1_object_grounding/p6_robot_local/runtime.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/experiments/r1_object_grounding/p6_robot_local/runtime.py) |
| Điều khiển tiếp cận | [assistant/app/ai_modules/navigation/gait_approach.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/ai_modules/navigation/gait_approach.py) |
| Ước lượng độ sâu | [assistant/app/ai_modules/vision/dac_depth.py](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/app/ai_modules/vision/dac_depth.py) |
| Owner nhận lệnh | [controller/src/input/ApproachReceiver.hpp](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/controller/src/input/ApproachReceiver.hpp) |
| Giới hạn và chế độ | [assistant/config/approach.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/config/approach.json), [assistant/config/robot.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/config/robot.json) |

Phần assistant được xuất từ dự án VLA, nhánh `vla`, commit nguồn `d422befb4d8cd5d566e4f4f8a588e3a644b67669`; [assistant/EXPORT.json](https://github.com/Giapinner88/Happy-Baby-R1/blob/develop/assistant/EXPORT.json) ghi `source_has_uncommitted_changes: true`. Vì vậy commit nguồn một mình không tái dựng đủ gói: cần giữ snapshot và hash từng file trong manifest. Sửa tại nguồn VLA rồi xuất lại bằng `python3 scripts/interaction/export_r1_assistant_hb.py --hb-root <HB> --force`; không sửa tay gói `assistant/`.

Mô hình DAC là phụ thuộc riêng; báo cáo chưa xác minh nguồn checkpoint/giấy phép hoặc nội dung model trên robot. Cần giữ signature/hash, calibration và engine của đúng lượt thử.

## 4. Kiểm chứng cần ghi

So sánh cặp ca cùng vật/cùng điểm xuất phát: vật đứng yên và vật được dời một lần tới vị trí đã đo. Giữ nguyên policy, cấu hình, sàn và ánh sáng. Lưu video, tọa độ/vị trí vật trước–sau, khoảng cách cuối đo độc lập, transcript và toàn bộ thư mục tác vụ ở `~/hb_assistant/runs/`: `task.json`, search events/results, approach `control.jsonl`, `observations.jsonl`, `result.json`, tracker-lost và owner trace nếu có.

Đánh giá riêng: chọn đúng vật, phát hiện mất mục tiêu, tìm lại đúng vật, đến vị trí mới, sai số dừng, số lần tìm lại, thời gian và can thiệp. Chốt ngưỡng nghiệm thu trước chạy; ghi đầy đủ lần thất bại. `camera_confirmed` chỉ là điều kiện thị giác trong phần mềm, và backend vẫn ghi `physical_arrival_verified: false`.

## 5. Giới hạn và kết luận

Theo cấu hình/tài liệu hiện tại: tốc độ tiến tối đa 0,16 m/s; đích tâm thân khoảng 0,55 m tới vật; mỗi đoạn tối đa 0,5 m; tổng quãng lệnh 3 m; tối đa 2 lần tìm lại với giới hạn thời gian 150 s được kiểm tra giữa các vòng. Đây là giới hạn theo lệnh/mô hình, không bảo đảm khoảng cách thật hoặc thời gian dừng vật lý.

**Robot không phát hiện người/vật cản trong lúc đi.** Độ sâu từng đọc gần phẳng 1,3–1,4 m khi vật cách 1,5–2,0 m; quay tại chỗ từng gây trôi; có lần owner thu hồi quyền chưa rõ nguyên nhân. Không coi bản đồ vùng trống do người vận hành khai báo là cảm biến tránh vật cản. Không dùng gần trẻ em khi chưa kiểm chứng riêng.

Theo `docs/assistant.md`, stack thử nghiệm đã chạy ngày 21/11, nhưng bản HB tích hợp chưa chạy trên robot. **Kết luận: có chuỗi end-to-end và cơ chế tìm lại có giới hạn; chưa đủ dữ liệu để khẳng định đi tới chính xác/an toàn khi vật đổi vị trí.**
