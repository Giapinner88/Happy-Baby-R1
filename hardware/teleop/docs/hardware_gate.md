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

**Cập nhật 2026-09-09.** Researcher chọn video của
`t007_whole_upper_body_20260909T072030Z` làm tham chiếu chuyển sang hardware.
Run upstream live hoàn tất ở 27.03 Hz, `sim_to_wall_ratio=0.99998`, không có
joint-limit clamp trong simulator; vận tốc q upstream p95 4.008 rad/s, max
12.149 rad/s. Đây là đánh giá hình ảnh tốt, nhưng `status.json` vẫn ghi
`scientific_outcome: unassessed`, nên ba checkbox trên chưa được tự động tick và
run này chưa phải bằng chứng hardware.

Phân tích `raw_commands.jsonl` của chính run cho thấy q vendor đầu tiên có
`max(|q|)=0.2703 rad`, phù hợp làm candidate source-alignment. Trong toàn phiên,
delta lớn nhất của nhóm không phải vai là 1.7755 rad, vượt envelope mặc định
1.0 rad. Head pitch nằm trong [-0.5421, 0.2244] và yaw trong
[-1.2986, 1.2965] rad, cũng vượt gate hardware 0.35/0.60 rad. Bởi vậy không được
tuyên bố toàn bộ video sẽ tái tạo không clamp trên robot. Sidecar mới ghi rõ số
lần/tên khớp chạm envelope và head gate; limiter producer vẫn là một khác biệt
thời gian riêng.

## 2. Đối chiếu mô hình với robot thật

- [ ] Xác nhận dấu và thứ tự khớp của FK so với pose đo được trên robot.
- [x] Giải quyết xung đột chỉ số motor đầu giữa spec high-level nội bộ và
      interface R1-A5 của hãng (đã ghi trong method record).

      Đo trên robot 2026-08-29: `rt/lowstate` cho IDL 30 = **2.0068 rad**, đúng
      bằng giới hạn ±2.0071 của `head_yaw_joint` trong `assets/R1.urdf`, trong
      khi IDL 29 = −0.001. Nếu 30 là pitch thì giá trị đó đã vượt xa giới hạn
      ±0.6283 của pitch nên không thể tồn tại. **IDL 29 = pitch, IDL 30 = yaw**,
      khớp với `spec::kHeadPitchIdl`/`kHeadYawIdl` và với thứ tự UTL1 của
      sidecar. Đây là phép đo trên phần cứng, không phải suy từ tài liệu.
- [ ] Kiểm tra `assets/R1.urdf` khớp với robot đang dùng.

## 3. Giới hạn an toàn

- [ ] Thay `max_joint_velocity_rad_s` / `max_joint_acceleration_rad_s2` bằng
      giới hạn phần cứng đã được duyệt, không dùng số tuning của mô phỏng.

      Hiện đang dùng 1.0 rad/s và 2.0 rad/s², áp trong
      `run_r1_quest3_hardware_targets.py` cho cả hai chế độ giải. **Đây vẫn là
      số chưa được duyệt.** Phép đo cũ ở cấu hình 0.5/1.0 cho thấy hai điều cần
      kiểm lại ở cấu hình hiện tại:

      1. **0.5 rad/s từng là chặn thật, không phải chặn dự phòng.** Trên đoạn thử,
         limiter bão hoà ở đúng 0.5 rad/s gần như mọi bước, trong khi bản mô
         phỏng của cùng đường này chạy tới p95 2.33 và max 5.94 rad/s. Nghĩa là
         trên phần cứng tay sẽ **bám trễ rõ rệt** so với người vận hành. Nâng
         trần là quyết định của cổng này, không phải của người vận hành lúc chạy.
      2. **Trần gia tốc cũ không được đảm bảo khi dừng.** `OnlineJointLimiter` cố ý
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

## 3d. Chuyển target upstream từ sim sang hardware — HARDWARE ATTEMPT FAILED

- [x] Producer khai báo tường minh `target_mode: relative_source`.
- [x] Upstream launcher chọn `--home-to-source`; coupled legacy vẫn chọn
      `--home-to-nominal`.
- [x] Sidecar đóng băng q vendor đầu tiên, kiểm nó trong zero-centred per-joint
      bound và ramp tới đó ở tốc độ homing với command tolerance 0.02 rad.
- [x] Sau alignment, `start_q = source_zero = q_source_initial`; test chứng minh
      công thức relative không cộng posture offset vào q upstream đã rate-limit.
- [x] Metadata ghi `home_mode`, goal/sai số encoder, envelope và head-gate clamp counters.
- [x] Package deploy-only đã được copy tới robot `10.42.0.33`; SHA-256 sidecar
      local/remote cùng là
      `6fd63324abe22cfac99a945a77d07c8d2cdb16a005f1def7326e88c50ec26ee8`.
      Sau deploy, `hb_teleop.service` vẫn inactive và không có Python sidecar.
- [ ] Chạy một bounded suspended pilot; đối chiếu `target_q`/`observed_q` và xác
      nhận đầu-only motion không kéo tay thật.

Thay đổi này sửa semantic mismatch trước đó: simulator áp q vendor tuyệt đối,
trong khi hardware home về nominal rồi re-anchor source mới nhất, nên hai bên
không thể có cùng posture. Nó không nới vận tốc, gia tốc, head gate, envelope,
watchdog hay quyền ghi `rt/lowcmd`. Hardware vẫn không phải temporal parity:
producer giới hạn 1.0 rad/s và 2.0 rad/s², trong khi run reference có q-speed
p95 4.008 và max 12.149 rad/s.

Ba pilot ngày 2026-09-10 (`030102Z`, `030330Z`, `030448Z`) đều được người vận
hành mô tả về goal nhưng không vào teleop; metadata lặp lại
`stop_reason=home_aborted_input`. Encoder tolerance 0.02 rad từng được nghi là
điều kiện giữ homing quá lâu, nhưng artifact không đủ để khẳng định đó là nguyên
nhân. Điều kiện này đã bỏ: command ramp quyết định khi home hoàn tất, encoder
residual vẫn được ghi và head vẫn qua gate riêng.

Pilot `031455Z` xác nhận một lỗi thứ hai: bridge ghi cò phải chuyển sang `true`
và không có transition về `false`, nhưng sidecar dừng lúc homing với
`home_aborted_input`; 120 giây sau owner ghi `Giu qua 120s ... -> ZERO TORQUE`.
Vì vậy cú sụp là hậu quả của mất target rồi hết hạn hold, không phải người vận
hành nhả cò. Artifact cũ không phân biệt EOF với watchdog và không lưu exit code
của từng process, nên tầng đầu tiên đóng vẫn **chưa xác định**. Launcher mới lưu
stderr/exit code riêng cho bridge, upstream solver, hardware target producer và
SSH; solver luôn ghi timing/status kể cả khi exception; sidecar tách
`home_aborted_stream_closed` khỏi `home_aborted_input_watchdog`. Owner cũng log
cạnh `stream ACTIVE/INACTIVE` với nguyên nhân. Các thay đổi này chỉ tăng khả năng
quan sát, không đổi timeout hoặc fallback.

Bản `high_level_lock` có log transition đã build thành công trên x86_64 và
ARM64 tại `/home/unitree/HB/high_level_lock/build/run_r1`; binary ARM64 chứa các
chuỗi diagnostic mới. Build/deploy không restart owner: `hb_high_level` vẫn
active bằng `/home/unitree/HB/high_level_2/build/run_r1`, còn `hb_teleop`
inactive.

Sửa tiếp sau run `035042Z`: lowstate chuyển từ `Read()` chặn sang callback lấy
mẫu mới nhất với timestamp nhận; rehome chạy từng tick ngoài để giữ watchdog
input/state/mode. Test vòng sidecar với transport giả lập: home thành công và
EOF/input timeout/state timeout (kể cả rehome) đều đạt. Tổng bộ liên quan:
90 passed, 1 skipped; trực tiếp Python 3.8 robot: 42 passed, 4 skipped. Replay
150 target qua IK/limiter/SSH không phát motor có gap lớn nhất 0.111 s. Đây là
code/transport evidence, chưa chứng minh giải quyết gap 0.756 s của live run;
run đó vẫn thực hiện ~109 bước trong 1.099 s, không chứng minh DDS bị chặn.
Chi tiết freshness và compatibility ở runbook `docs/teleop/r1_quest3_teleop_hardware.md`.

## 3c. Khoá cứng khớp ngoài teleop — CHƯA DUYỆT, CHƯA CHẠY

- [ ] `hardware/high_level_lock/` giữ chân (IDL 0-11) và eo (12-13) tại encoder
      chốt lúc teleop active, thay vì để limp như bản đang chạy. Chưa build,
      đã build và preflight trên robot, **chưa chạy `Run()`**. Chỉ có nghĩa khi
      robot treo trên giá — khoá giữ tư thế, nó không đỡ trọng lượng.
      `teleop_lock_kp/kd = 20/3` chưa đo, và **không phải điều kiện chặn**: chân
      và eo không phải đối tượng đo của pilot này, chỉ cần đứng yên. Encoder
      ngoài teleop đọc lỗi thì riêng khớp đó thả limp, phiên vẫn chạy.
- [ ] **Eo bị khoá (chốt 2026-08-24).** Teleop chỉ lái tay và đầu, nên eo tự do
      là nhiễu lẫn vào chính phép đo đang cần làm. Điều kiện kèm theo: tay và
      đầu phải chứng minh được trước; muốn cho teleop lái `waist_yaw` thì phải
      bỏ IDL 13 khỏi danh sách khoá TRƯỚC. Ràng buộc này được `static_assert`
      chặn lúc biên dịch chứ không dựa vào comment.

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
