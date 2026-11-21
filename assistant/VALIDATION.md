# Kiểm chứng workstation — 2026-11-15

Phạm vi: tích hợp phần mềm và gói triển khai; không SSH, không camera/cloud
thật, không thử chuyển động robot trong lượt làm này.

| Kiểm tra | Kết quả |
| --- | --- |
| Assistant: điều phối, HTTP loopback, voice, camera lease, hủy/race, bundle/owner staging | 22/22 Python tests qua |
| P5 tìm/chỉ, gồm search Python riêng và cancel trong admission | 9/9 qua |
| P6 robot-local, tracking/odom, freshness, startup, gait/turn | 42/42 qua |
| Navigation/depth-grid offline | 23/23 qua |
| APV1 Lease: velocity/nonce/sequence/expiry/manual/emergency | qua, C++ -Werror |
| APV1 STOP/Release handoff: pure lease và UDP loopback | qua, C++ -Werror |
| HB controller copy build x86_64 + full CTest | build qua; 49/50 CTest qua |
| Extension với HB voice venv/Pipecat1.5 hiện có | nạp được đủ 5 tool camera/tìm-chỉ/tiếp cận/hủy/trạng thái; gesture giữ registry/gate native |
| Hash source/config HB dùng chung | 166 file đối chiếu, không đổi |
| Python compatibility của release | tất cả .py parse theo Python3.8 |

Full CTest còn `t1_policy_period_test`: script đặt `walk_gait_period_s=0.6`
và mong nó khác metadata, nhưng policy R2+A hiện tại cũng có period0.6.
Controller chấp nhận hai giá trị bằng nhau là đúng. Đối chứng: chỉ thay
giá trị thử trên **bản sao script ở /tmp** thành0.123456 thì toàn bộ bài
test đó qua. Không sửa test, policy hoặc config trong cây HB dùng chung.
Binary có sẵn của HB cũng không hoàn thành bài test này; kết quả đó không
được dùng thay cho build từ source hiện tại.

Gate installer dùng tập10 CTest được nêu rõ trong `deploy.REQUIRED_CTEST`
(motion safety/input/remote/teleop/gait/config/preflight), cộng APV1 handoff.
Không mô tả tập gate này là full suite HB. Cấu hình CMake full suite trên
workstation dùng `HB_TEST_PYTHON` trỏ tới r1_env có onnx; installer không
yêu cầu cài thêm onnx Python vào môi trường hệ thống robot chỉ để chạy
test sinh model. Binary preflight vẫn kiểm model/config của chính owner.

Socket/asyncio/multiprocessing tests cần chạy ngoài filesystem sandbox
chặn socket. Lần đầu sandbox từ chối HTTP socket và khiến một lượt test
multiprocessing bị dừng; các lượt chạy lại với quyền loopback đã qua.
Đây là giới hạn môi trường test, không phải bằng chứng về kết nối robot.

```bash
python3 -m unittest discover -s integrations/r1_assistant/tests -v
python3 -m unittest experiments/r1_object_grounding/p5_voice_search/test_server.py
python3 -m unittest discover -s experiments/r1_object_grounding/p6_robot_local -p 'test_*.py'
python3 -m unittest discover -s experiments/r1_object_grounding/p6_approach_offline -p 'test_*.py'
c++ -std=c++17 -Wall -Wextra -Werror -O2 \
  experiments/r1_object_grounding/p6_owner_adapter/test_release.cpp -o /tmp/r1_apv_handoff
/tmp/r1_apv_handoff
c++ -std=c++17 -Wall -Wextra -Werror -O2 -pthread \
  experiments/r1_object_grounding/p6_owner_adapter/test_network.cpp -o /tmp/r1_apv_network
/tmp/r1_apv_network
```

Giới hạn còn lại: chưa cài/khởi động bản tích hợp này trên ARM64 robot;
model TensorRT/SDK/mic/network và quyền sudo phải qua check tại đích.
Chưa xác nhận độ chính xác depth/camera mount/odom, khoảng dừng thực,
pointing accuracy, né vật cản hoặc độ trễ voice cancel ngoài thực tế.
Sự cố v26 được giữ nguyên trong hồ sơ, không đổi thành success.

Manifest release lưu hash code/calibration; owner build lưu hash binary,
source và phạm vi CTest. Task lưu config, provenance và result; P4/P6 giữ
raw observations/control/ảnh/STOP ACK. Không dùng accepted, ACK hay đích
mô hình làm kết luận đã tới đúng khoảng cách thực.
