# Cổng phần cứng cho teleop R1

Tài liệu này liệt kê những việc **phải hoàn thành** trước khi teleop được phép
gửi target tới sole-owner thật. Chưa đóng đủ thì
`HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP` phải giữ nguyên `0`.

Không mục nào dưới đây được đánh dấu hoàn thành ở thời điểm tạo package này.

## 0. Cổng tốc độ vòng điều khiển — CHƯA ĐÓNG

- [ ] Bộ giải trên đường phần cứng chạy được ở nhịp điều khiển đã khai báo.

Đo ngày 2026-08-23 (`t007_simonline_seg2_20260823`): `WholeUpperBodyIsaacLabSink`
— đúng bộ giải mà `run_r1_quest3_hardware_targets.py` dùng — đạt **3.46 Hz so
với 30 Hz yêu cầu**, mỗi bước có IK tốn **291 ms so với ngân sách 33 ms**. Chỉ
3.9% lệnh nhận được dispatch.

Sau khi thay Jacobian sai phân bằng giải tích (`t007_simonline_analytic_seg2_20260823`):
**19.99 Hz, 50.1 ms mỗi bước, dispatch 818/3666**.

**Cập nhật cuối ngày — cổng này đo sai đối tượng.** Tách chi phí cho thấy trong
37.1 ms mỗi bước, **chỉ 6.3 ms là bộ giải**; 30.8 ms còn lại là PhysX + render,
**không tồn tại trên phần cứng**. Trần nhịp do bộ giải là **159 Hz** ở 20 vòng
lặp và **37 Hz** ở 100 vòng — cả hai đều trên 30 Hz.

Cổng tốc độ vì thế **coi như đạt về mặt tính toán** (26.8 ms so với ngân sách
33.3 ms ở cấu hình chất lượng cao nhất), nhưng **chưa được xác nhận trên phần
cứng thật**, nơi có thêm chi phí DDS, mạng và bộ điều khiển motor mà mô phỏng
không có.

Cổng chặn còn lại là **chất lượng bám**, không phải tốc độ: p95 trái 143.9 mm
đạt ngưỡng 150 mm nhưng phải 165.3 mm chưa; max trái 204.0 mm đạt ngưỡng 250 mm
nhưng phải 418.4 mm chưa.

Đây là cổng chặn: teleop ở 3.5 Hz nghĩa là tay robot cập nhật chưa tới 4 lần mỗi
giây trong khi người vận hành cử động liên tục. Không được đưa xuống phần cứng
cho tới khi đóng.

### Cập nhật 2026-08-24 — đường phần cứng đã đổi bộ giải

`run_r1_quest3_hardware_targets.py` mặc định **không còn tự giải**. Joint tới nó
đã được `xr_teleoperate` R1_A5_ArmIK giải sẵn ở tiến trình phía trên, và nó chỉ
áp giới hạn khớp theo asset cùng trần vận tốc/gia tốc.

Đo lại trên cùng một đoạn 10 s của `t007_whole_upper_body_20260824T071530Z`,
chạy đúng đường ống phần cứng (`replay → vendor IK → producer`), ở 10 Hz:

| | coupled (cũ) | **upstream (mới)** |
|---|---|---|
| target phát ra / mẫu | 19 / 100 | **90 / 90** |
| chi phí giải | 49.3 ms/bước | **~1.2 ms/bước** |
| `solver_solution_kind` | 81 mẫu không có nghiệm nào được chấp nhận | không áp dụng — vendor luôn trả nghiệm |

Đường cũ **từ chối 81% mẫu**: đường phần cứng đặt
`allow_nonconverged_solution=False` và `allow_projected_position_solution=False`,
nên mọi mẫu không hội tụ chính xác đều không phát ra target. Đây mới là lý do
thật khiến đường cũ không dùng được, chứ không chỉ là chậm.

Cổng này vẫn **CHƯA ĐÓNG**: số trên đo trên workstation với trace đã ghi, chưa
đo trên robot có DDS, mạng và bộ điều khiển motor thật.

Lưu ý: bộ giải nhân quả `solve_r1_t007_causal_tracking.py` đạt 18–22 Hz nhưng
**không nằm trên đường phần cứng**. Ba bộ giải đang song song; chỉ một được
validate.

## 1. Bằng chứng mô phỏng

- [ ] Một run T007 đạt tiêu chí chấp nhận đã khai báo, với `scientific_outcome`
      khác `unassessed`.
- [ ] Sai số bám endpoint và tốc độ vòng điều khiển được đo và ghi lại, không
      phải suy ra.
- [ ] Chạy lại được: cùng config, cùng commit, ra cùng kết luận.

Hiện trạng: run mới nhất `t007_whole_upper_body_20260818T114338Z` vẫn là
`unassessed`, đạt 9.98 Hz so với 20 Hz yêu cầu, và phần lớn target được
*projected* chứ không hội tụ chính xác.

## 2. Đối chiếu mô hình với robot thật

- [ ] Xác nhận dấu và thứ tự khớp của FK so với pose đo được trên robot.
- [ ] Giải quyết xung đột chỉ số motor đầu giữa spec high-level nội bộ và
      interface R1-A5 của hãng (đã ghi trong method record).
- [ ] Kiểm tra `assets/R1.urdf` khớp với robot đang dùng.

## 3. Giới hạn an toàn

- [ ] Thay `max_joint_velocity_rad_s` / `max_joint_acceleration_rad_s2` bằng
      giới hạn phần cứng đã được duyệt, không dùng số tuning của mô phỏng.

      Hiện đang dùng 0.5 rad/s và 1.0 rad/s², áp trong
      `run_r1_quest3_hardware_targets.py` cho cả hai chế độ giải. **Đây vẫn là
      số chưa được duyệt.** Hai điều đã đo, cần biết trước khi duyệt:

      1. **0.5 rad/s là chặn thật, không phải chặn dự phòng.** Trên đoạn thử,
         limiter bão hoà ở đúng 0.5 rad/s gần như mọi bước, trong khi bản mô
         phỏng của cùng đường này chạy tới p95 2.33 và max 5.94 rad/s. Nghĩa là
         trên phần cứng tay sẽ **bám trễ rõ rệt** so với người vận hành. Nâng
         trần là quyết định của cổng này, không phải của người vận hành lúc chạy.
      2. **Trần gia tốc không được đảm bảo khi dừng.** `OnlineJointLimiter` cố ý
         zero vận tốc khi tới đích hoặc khi chạm giới hạn khớp, nên gia tốc đo
         được đạt **5.6 rad/s² so với trần 1.0**. Đều là giảm tốc, và hành vi này
         có sẵn từ trước chứ không do đổi bộ giải, nhưng phải được duyệt như một
         đặc tính chứ không được coi là đã bị chặn.
- [ ] Thêm guard va chạm và moment xoắn.
- [ ] Xác định hành vi khi mất kết nối Quest: phải giữ nguyên vị trí, không rơi
      tay.
- [ ] `--disable-self-collisions` là workaround của simulator; phải xác định
      hành vi tương ứng trên phần cứng.

## 3b. Khác biệt mang về từ sidecar của robot — CHƯA DUYỆT

Bản `high_level_sidecar.py` kéo từ robot về ngày 2026-08-24 mang ba thay đổi
hành vi mà cổng này chưa xét:

- [ ] **`schema_version` trở thành tuỳ chọn.** Bản cũ từ chối line không có
      `schema_version`; bản này đọc mặc định là 1. Một producer sai phiên bản
      giờ được nhận thay vì bị loại.
- [ ] **Chấp nhận stream chỉ 10 khớp tay.** Khi đó `head_valid=0` và đầu thả
      limp. Phải xác định đây là hành vi mong muốn trên phần cứng.
- [ ] **`target_mode: absolute_robot`** cùng tolerance 0.002 rad khi so với
      envelope. Dùng chung envelope với LeRobot; chưa đo trên đường Quest.

Cả ba đã được ghim bằng test trong `tests/test_high_level_sidecar.py` để chúng
là quyết định chứ không phải bất ngờ.

## 3c. Khoá cứng khớp ngoài teleop — CHƯA DUYỆT, CHƯA CHẠY

- [ ] `hardware/high_level_lock/` giữ chân (IDL 0-11) và eo (12-13) tại encoder
      chốt lúc teleop active, thay vì để limp như bản đang chạy. Chưa build,
      chưa link, chưa chạy trên robot; `teleop_lock_kp/kd = 20/3` là số khởi
      điểm chưa đo. Chỉ có nghĩa khi robot treo trên giá — khoá giữ tư thế, nó
      không đỡ trọng lượng.

## 4. Quyền ghi khớp

- [ ] Chứng minh `hb_high_level` là DDS motor writer duy nhất; `run_teleop.py`
      và `high_level_sidecar.py` không được chứa/tạo `ChannelPublisher`.
- [ ] Chứng minh UTL1 chỉ bind loopback `127.0.0.1:5560` và stale target làm
      high-level release về ZERO TORQUE.
- [ ] Xác định trạng thái robot khi chuyển qua lại giữa hai chế độ.

## 5. Quy trình vận hành

- [ ] Chạy dry-run với motor chưa enable, ghi lại lệnh sẽ phát.
- [ ] Có người giữ E-stop, không vận hành một mình.
- [ ] Ghi log theo `docs/templates/test_log_template.md` của workspace.
- [ ] Tuân thủ `docs/safety/safety_rules.md`.

## Ghi chú về IK

Bộ giải hiện dispatch cả nghiệm *projected* cho target ngoài tầm với. Trong mô
phỏng đó là lựa chọn đúng (đóng băng cả tay còn tệ hơn), nhưng trên phần cứng
nó có nghĩa là robot sẽ đi tới tư thế gần nhất thay vì từ chối — hành vi này
phải được duyệt riêng, không mặc nhiên mang từ mô phỏng sang.
