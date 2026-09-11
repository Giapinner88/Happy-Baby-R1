# R1 Quest 3 teleop trên robot thật — Dev Mode tay/đầu

Quy trình chạy Quest teleop cho R1-A5 trong phạm vi **10 khớp tay + 2 khớp đầu**.
`hb_high_level` là publisher `rt/lowcmd` duy nhất; sidecar teleop chỉ gửi target
qua UDP loopback và không target eo hoặc chân.

> **Chưa qua hardware gate.** Trạng thái hiện tại chỉ được xác nhận khi robot
> treo/cố định trên giá. Không có gì trong tài liệu này cho phép chạy trên sàn.
> Các mục chưa đóng: [`hardware/teleop/docs/hardware_gate.md`](../../hardware/teleop/docs/hardware_gate.md).

Cập nhật 2026-09-11: đường phần cứng chỉ còn bộ giải vendor
`xr_teleoperate` chạy nguyên xi; fallback coupled đã được loại khỏi launcher.

---

## 1. Điều kiện bắt buộc

Không đủ một trong các điều dưới đây thì không chạy.

- Robot **treo và cố định trên giá**.
- **Hai người**: một giữ E-stop, một vận hành Quest/R3. Không chạy một mình.
- Robot đã vào Dev Mode và high-level đã tiếp quản (mục 4).
- `hb_teleop.service` **inactive**.
- Đã đọc [`docs/safety/safety_rules.md`](../safety/safety_rules.md).

## 2. Topology

> **Địa chỉ robot là động.** `wlan0` lấy IP qua DHCP, nên số IP đổi giữa các lần
> bật. Đừng chép IP vào lệnh. Mọi script lấy địa chỉ theo thứ tự: biến `ROBOT=`
> trên dòng lệnh → `~/.config/hb/robot.env` → dò tự động; rồi **xác minh đúng
> máy** trước khi ghi bất cứ thứ gì.
>
> Mất liên lạc thì tìm robot theo MAC cố định `c0:3a:55:f1:41:84`:
>
> ```bash
> for i in $(seq 2 254); do (ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 &); done; sleep 5
> ip neigh | grep -i c0:3a:55:f1:41:84
> ```
>
> rồi sửa đúng một dòng trong `~/.config/hb/robot.env`.

| Thành phần             | Địa chỉ / interface                                |
| ------------------------ | ----------------------------------------------------- |
| Workstation (Quest + IK) | `192.168.1.106`, `wlp77s0`                        |
| Robot SSH                | `$ROBOT` — IP động, xem cảnh báo trên         |
| DDS trên robot          | `eth10` — `rt/lowstate`, `rt/lowcmd`           |
| UTL1 loopback            | `127.0.0.1:5560` (chỉ loopback)                    |
| Quest                    | cùng Wi-Fi`HappyBaby`                              |
| Cert                     | `~/.config/xr_teleoperate/happybaby_192_168_1_106/` |

## 3. Đường ống

Bốn tiến trình trên workstation, một trên robot:

```text
quest_bridge.py                     (env tv)              đọc headset, phát R1TeleopCommand 30 Hz
  → run_r1_upstream_ik_stream.py --passthrough   (env tv) giải bằng R1_A5_ArmIK nguyên xi
  → run_r1_quest3_hardware_targets.py            (unitree_sim_env) áp envelope, phát 12 góc khớp 10 Hz
  → ssh → teleop.hardware.high_level_sidecar     (robot) chốt phiên tương đối, gửi UTL1 loopback
  → hb_high_level                                (robot) chủ rt/lowcmd duy nhất
```

Không tiến trình nào ngoài `hb_high_level` được ghi `rt/lowcmd`; contract này
được mô tả tại [hardware boundary](07_hardware_boundary.md).

## 4. Chọn bản chạy phía robot

Hai lựa chọn, **không bao giờ chạy cùng lúc**.

### 4a. Bản service thường — chân và eo thả limp

```bash
ssh $ROBOT 'systemctl is-active hb_high_level'   # phải: active
```

### 4b. Bản cô lập khoá khớp — chân và eo giữ cứng tại tư thế lúc bóp cò

Dùng khi muốn thân dưới đứng yên để không lẫn vào phép đo tay/đầu.
**Chỉ đúng khi robot đang treo** — khoá giữ tư thế, nó không đỡ trọng lượng.

```bash
# trên robot, terminal riêng, giữ nguyên suốt phiên
cd ~/HB/high_level_lock && ./scripts/run_lock_foreground.sh
```

Script dừng `hb_high_level`, chạy bản cô lập foreground, và **bật lại service khi
thoát** — kể cả Ctrl+C hay crash. Nó từ chối khởi động nếu `high_level_2` còn
sống. Cần mật khẩu sudo của robot, nên phải do người ở cạnh giá chạy.

Chi tiết: [`hardware/high_level_lock/README_LOCK.md`](../../hardware/high_level_lock/README_LOCK.md).

## 5. Chuẩn bị (làm một lần mỗi khi đổi code)

```bash
# workstation, từ root repo
make teleop-hardware-prepare
```

Lệnh này sync source, kiểm đường Quest và copy package. Nó **không** install,
start, enable service, không arm motor, không tạo publisher.

Xem trước rồi mới đẩy:

```bash
./hardware/teleop/scripts/deploy_teleop.sh diff   # dry-run
```

Kiểm read-only trên robot, không tạo publisher:

```bash
ssh $ROBOT   # menu đăng nhập: chọn foxy (1)
cd /tmp && PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.run_teleop --interface eth10
```

Mong đợi: `rt/lowstate`, `mode_machine=1`, `motors=35`, và `no publisher was created`.

## 6. Trình tự chạy

**Bước 1 — khởi động chủ `rt/lowcmd`.** Chọn 4a hoặc 4b.

**Bước 2 — tay cầm R3.**

```
L2+R2            → Dev Mode (built-in)
giữ R1+R2 3 giây → high-level tiếp quản
L2+Y             → ZERO TORQUE
```

Với bản cô lập, sau banner phải thấy:

```
[Application] Không thấy built-in thì tự bypass sau 15s.
[Application] ARM — bypass Dev Mode (không thấy built-in sau 15s), ý định người OK.
```

Nếu đứng mãi ở `[kDisarmed]` thì đọc mục 9.

**Bước 3 — workstation.**

```bash
make teleop-hardware \
  HOST_IP=192.168.1.106 \
  DURATION_S=180 \
  CERT_FILE=$HOME/.config/xr_teleoperate/happybaby_192_168_1_106/cert.pem \
  KEY_FILE=$HOME/.config/xr_teleoperate/happybaby_192_168_1_106/key.pem
```

Đây là bộ giải vendor upstream, tay + đầu; launcher không còn chế độ coupled.

Entrypoint kiểm tra cert trước mỗi lần chạy. Pair còn hạn và có SAN khớp
`HOST_IP` được reuse; pair hết hạn, hỏng hoặc sai SAN được tạo lại atomically.
Nếu chỉ còn một trong `cert.pem`/`key.pem`, lệnh dừng để người vận hành kiểm tra
thay vì tự ghi đè credential còn lại.

**Bước 4 — Quest.**

1. Wi-Fi `HappyBaby`, mở `https://192.168.1.106:8012/?ws=wss://192.168.1.106:8012`.
2. Chấp nhận cert nếu hỏi. Chọn **Enter VR**.
3. **Chưa bóp cò phải.** Giữ đầu và hai controller ở đúng neutral thoải mái đã
   dùng trong run sim chuẩn; dọn khoảng trống quanh cả hai tay robot và đầu.
4. Với upstream mặc định, không cần nắn robot về đúng dáng sim bằng tay. Sidecar
   sẽ kiểm và ramp tới target vendor đầu tiên. Khi tắt alignment, đầu robot phải
   gần thẳng: sidecar từ chối nếu `|yaw| > 0.60` hoặc `|pitch| > 0.35` rad.
5. Kiểm lần cuối: robot không còn chuyển động chuyển tiếp nào.
6. **Giữ cò index bên phải và GIỮ YÊN đầu/controller trong dòng `[HOME]`.** Với
   upstream mặc định, target hợp lệ đầu tiên được đóng băng làm goal. Robot ramp
   tới đúng vector 12 khớp đó ở 0.15 rad/s; sidecar vào teleop khi ramp command
   còn sai không quá 0.02 rad và ghi riêng residual encoder. Nhả cò, mất lowstate, đổi mode,
   quá 45 giây, hoặc target ban đầu vượt bound đều làm alignment fail-closed.
7. Khi alignment đạt, `start_q = source_zero = q_source_initial`. Vì vậy công
   thức tương đối của sidecar rút gọn thành
   `q_sidecar = q_source_initial + (q_source - q_source_initial) = q_source`
   trong phần chưa chạm envelope/head gate. `q_source` là q vendor đã qua limiter
   hardware 1.0 rad/s và 2.0 rad/s²; do đó posture không còn offset, nhưng đáp
   ứng thời gian vẫn chậm hơn simulator. Với bản cô lập, chân và eo cũng bị khoá.
8. Di chuyển **chậm**.

Producer upstream còn dùng ba mẫu deadman liên tiếp đầu tiên để chốt head pose
làm spatial anchor. Hai wrist được biểu diễn theo position/yaw của anchor này
trước khi giải IK, nên xoay hoặc dịch đầu sau đó không kéo tay theo nếu
controller đứng yên. Nhả/bóp lại cò phải giữ nguyên anchor; chạy lại pipeline
mới tạo anchor mới. Đây là mốc không gian phía Quest, khác với `source_zero` và
`start_q` của sidecar dùng cho ánh xạ tương đối sang encoder robot. Ở upstream,
hai mốc sidecar cố ý bằng nhau sau source alignment để không thêm posture offset
vào q upstream đã qua limiter của producer.

Tắt alignment: `HB_TELEOP_HOME=0`. Khi tắt, robot giữ encoder hiện tại làm mốc;
mapping trở lại dạng offset và không còn đảm bảo giống dáng sim.

**Cò trái = căn lại.** Goal mới là q source tại thời điểm bấm. Phiên tiếp tục
sau khi ramp và chốt mốc.

**Nhả cò phải thì tay giữ nguyên tư thế**, không sụp như trước. Bóp lại là đi
tiếp từ đúng chỗ đó. Quá 120 giây không ai lái thì owner trả về ZERO TORQUE.

Bóp cò phải khi chưa ở neutral: nhả cò ngay, chờ pipeline release, chạy lại từ
đầu. Không vặn tay/đầu sang tư thế bù trong khi controller còn active.

## 7. Giới hạn đang áp

| Chặn ở đâu                                        | Giá trị                                                                  |
| ----------------------------------------------------- | -------------------------------------------------------------------------- |
| Producer — vận tốc / gia tốc khớp                | 1.0 rad/s, 2.0 rad/s² (để owner là thứ chặn thật)                   |
| Producer — giới hạn khớp                          | theo asset`R1.urdf`                                                      |
| Producer — nhịp phát                               | 10 Hz                                                                      |
| Sidecar — envelope mỗi khớp so với`source_zero` | **±1.0 rad** (`HB_TELEOP_MAX_OFFSET_RAD`)                         |
| Sidecar — envelope 6 khớp VAI                       | **±3.2 rad = hết tầm** (`HB_TELEOP_MAX_OFFSET_SHOULDER_RAD`)    |
| Sidecar — watchdog lệnh vào /`rt/lowstate`       | 0.75 s / 0.20 s                                                            |
| Sidecar — nhịp gửi UTL1                            | 100 Hz                                                                     |
| Sidecar — source alignment                           | 0.15 rad/s, timeout 45 s, command tolerance 0.02 rad; ghi residual encoder |
| Owner — slew                                         | **0.6 rad/s** (`HB_TELEOP_RATE`, trần 1.50)                       |
| Owner — PD tay                                       | kp 40, kd 2                                                                |
| Owner — giới hạn đầu                             | yaw 1.0 rad, pitch 0.62 rad                                                |
| Owner — timeout UTL1                                 | 300 ms                                                                     |
| Bản cô lập — khoá chân/eo                       | kp 20, kd 3, slew 0.20 rad/s                                               |
| Bản cô lập — giữ tay khi nhả cò                | bật, hết hạn sau 120 s                                                  |

Không nới bất kỳ giá trị nào trong bảng này mà chưa qua hardware gate.

### Chỉnh lúc chạy

Tốc độ bám và gain đặt bằng biến môi trường khi khởi động bản cô lập; không sửa
file, không build lại. Có tác dụng từ lần khởi động sau vì owner đọc config một
lần lúc start.

```bash
HB_TELEOP_RATE=0.9 ./scripts/run_lock_foreground.sh      # trần 1.50
```

| biến                          | mặc định | tác dụng                            |
| ------------------------------ | ----------- | ------------------------------------- |
| `HB_TELEOP_RATE`             | 0.6         | slew của owner, rad/s                |
| `HB_TELEOP_ARM_KP` / `_KD` | 40 / 2      | PD tay: cao hơn thì bám cứng hơn |
| `HB_TELEOP_LOCK_KP`          | 20          | độ cứng khoá chân/eo             |
| `HB_TELEOP_HOLD_TIMEOUT_S`   | 120         | giữ tay bao lâu sau khi nhả cò    |

Script sinh `config/teleop_runtime.yaml` và cho `tuning.yaml` include nó sau
cùng, nên nó ghi đè mọi tầng bên dưới. File đó sinh tự động, đừng sửa tay.

Envelope thì đặt ở phía workstation, lúc chạy `make teleop-hardware`:

```bash
HB_TELEOP_MAX_OFFSET_SHOULDER_RAD=2.0 make teleop-hardware ...
```

## 8. Dừng

**Bình thường**

1. Giữ nguyên tư thế.
2. Nhả cò phải. Stream gửi STOP; đầu nhả ngay, tay giảm quyền trong 0.5 giây rồi
   về `ZERO TORQUE`.
3. Terminal chưa thoát thì `Ctrl+C` một lần.
4. Bản cô lập: `Ctrl+C` ở terminal robot; script tự bật lại `hb_high_level`.

**Bất thường — dùng E-stop ngay**, không chờ watchdog, khi có: rung, sai chiều,
va chạm, tiếng lạ, mất mạng, hoặc target không tương ứng chuyển động người
vận hành.

**Xác minh đã dừng**

```bash
ssh $ROBOT \
  'ps -eo pid,args | grep -E "high_level_sidecar|run_r1" | grep -v grep; systemctl is-active hb_high_level hb_teleop'
```

Không được còn `high_level_sidecar`. `hb_high_level` phải active. Chân/eo về
`ZERO TORQUE`.

## 9. Sự cố thường gặp

**Kẹt ở `[kDisarmed]`, giữ R1+R2 mãi không arm.** Đọc phần cuối dòng log:

- `CHAN: chua chot y dinh` → nút chưa được giữ đủ `arm_hold_s`.
- `CHAN: chua nghe built-in va khong co bypass` → built-in không phát gói nào
  (`gói built-in đã thấy: 0`) mà `arm_no_builtin_timeout_s = 0`, nên **không có
  đường nào arm được**; giữ nút lâu hơn vô ích. Profile phải đặt
  `arm_no_builtin_timeout_s: 15.0`. Đây là lỗi đã gặp thật ngày 2026-08-24.
- `CHAN: built-in chua im du ...ms` → built-in còn đang phát; nhả rồi thử lại.

Dòng `[kDisarmed]` in đúng 1 giây một lần — đó là nhịp thiết kế, không phải treo.

**`make teleop-hardware` thoát ngay với `Error 1` sau khi check_vuer xanh.** Đọc
dòng `[FAIL]` ngay trên nó. Bước kiểm điều kiện đòi **đúng một** chủ `rt/lowcmd`
đang chạy và giữ 5560 — không quan tâm đó là service hay bản cô lập foreground.
Hai tiến trình `run_r1` cùng chạy cũng bị chặn ở đây (vi phạm D003).

**`[SAFE] no valid target`** — sidecar không nhận được line hợp lệ nào. Hầu như
luôn là sai thứ tự tên khớp hoặc sai định dạng; kiểm producer có phát
`joint_names` kết thúc bằng `head_yaw_joint, head_pitch_joint` không.

**`[SAFE] head not neutral`** — chỉ xảy ra khi **alignment tắt**. Khi alignment
bật, gate chuyển xuống sau pha ramp và kiểm kết quả
(`head_not_neutral_after_home`) bằng **encoder đo được**, không phải giá trị vừa
ra lệnh. Với upstream, goal đầu bình thường bằng 0 vì output đầu được đo tương
đối từ headset anchor ban đầu.

**`source_home_goal_outside_envelope`** — target vendor đầu tiên không phải một
neutral an toàn theo bound đã khai báo. Không clamp im lặng và không nới bound;
nhả cò, đặt lại đầu/controller về neutral rồi chạy lại pipeline.

`final_measured_error_rad` trong metadata là sai số bám được quan sát lúc ramp
command hoàn tất; nó không chặn chuyển sang teleop. Head vẫn phải qua gate riêng
sau home. Sai số tay lớn cần được đọc cùng `target_q`/`observed_q` để kiểm
owner/mode/gain hoặc cơ khí.

Với homing tắt: đầu đang ZERO TORQUE nên xoay tay được. Vừa xoay vừa nhìn số:

```bash
./scripts/teleop/watch_r1_head_angle.sh      # read-only, Ctrl+C để thoát
```

Ghi nhận 2026-08-29: đọc được `yaw = +2.007 rad`, đúng bằng giới hạn ±2.0071 của
`head_yaw_joint` trong asset — đầu nằm sát chặn cơ khí, xoay hết cỡ 115°. Con số
này cũng xác nhận độc lập rằng **IDL 29 là pitch và IDL 30 là yaw**: nếu ngược
lại thì 2.007 đã vượt xa giới hạn ±0.628 của pitch, không thể tồn tại.

**`connect_count=0`** — Quest chưa cùng mạng, chưa chấp nhận cert, hoặc chưa Enter VR.

**`deadman_enabled=false`** — đang nhả cò, hoặc bấm nhầm cò trái.

**`home_aborted_stream_closed`** — một tầng upstream đã đóng pipe trong lúc
homing. Đọc `pipeline_status.json` rồi stderr của tầng có exit code khác 0.

**`home_aborted_input_watchdog` / `input_watchdog`** — pipe còn mở nhưng sidecar
không nhận target hợp lệ mới trong 0.75 s. Đối chiếu solver timing và stderr của
target producer; không mặc định coi đây là nhả cò.

**Thiếu listener `127.0.0.1:5560`** — high-level chưa chạy hoặc sai config. Không
chạy direct-lowcmd để lách kiểm tra này; đường đó đã bị loại.

## 10. Evidence

```text
workstation : results/smoke/<UTC>_r1_quest3_hardware/
robot       : /home/unitree/HB/teleop/logs/<UTC>_r1_high_level_teleop/
bản cô lập  : ~/HB/high_level_lock/logs/<UTC>_run_lock.log
```

Mỗi run workstation còn ghi `pipeline_status.json`, `bridge.stderr.log`,
`upstream_solver.stderr.log`, `upstream_solver_stats.json`,
`hardware_targets.stderr.log` và `ssh.stderr.log`. Các file này xác định tầng
đóng đầu tiên; `bridge_stop=downstream_closed` chỉ là hậu quả lan ngược, không
tự nó chứng minh bridge lỗi.

Khi sidecar thoát, launcher tự tải `metadata.json` và `samples.jsonl` từ path
robot nói trên vào `robot/`, tính `metrics.json`, sinh
`figures/hardware_joint_tracking.png`, `telemetry_tracking.mp4` và
`artifact_manifest.json`. MP4 là animation của target/encoder, không phải video
camera robot. Thiếu data, figure hoặc video làm artifact gate và toàn lệnh fail;
artifact chẩn đoán đã có vẫn được giữ lại.

Sửa vòng nhận lowstate 2026-09-10: sidecar dùng callback `LatestLowState.receive`
và đọc snapshot không chặn, thay `ChannelSubscriber.Read()` vốn có thể chờ
vô hạn. Tuổi lowstate tính từ callback nhận mẫu, đọc lại snapshot không gia hạn
watchdog 0.20 s. Cả homing và căn lại bằng cò trái đều kiểm input/lowstate/mode
mỗi tick 100 Hz. Đây là thay đổi cách nhận và đo freshness, không đổi IK, gain,
ngưỡng watchdog hay hold 120 s của owner. Callback/latest-sample là lựa chọn
triển khai; mẫu trung gian được bỏ, encoder mới nhất được dùng.

`hardware_targets.jsonl` lưu sequence và thời điểm phát ở workstation để đối
chiếu với sequence cuối receiver nhận; không trừ trực tiếp monotonic của hai máy.
Run `035042Z` dừng sau 1.099 s homing, command còn lệch 0.551 rad và loop đã
thực hiện khoảng 109 bước: log này chưa chứng minh DDS đã block trong run đó.
Watchdog input 0.756 s là xác nhận; nguồn gây gap cụ thể vẫn chưa tái hiện.
Test vòng thật với DDS/UDP giả lập kiểm homing, EOF, mất input, mất lowstate và
rehome; replay qua IK/limiter/SSH chỉ thu dữ liệu kiểm đường truyền, không phải
bằng chứng bám quỹ đạo trên robot. Cần pilot mới để xác nhận sửa được sự cố thực tế.

Trên robot, `metadata.json` ghi `joint_names`, `motor_indices`, `target_mode`,
`home_mode`, goal/sai số alignment, số lần envelope clamp, các khớp từng clamp,
`head_gate_clamp_count`, `head_valid` và envelope của phiên; `samples.jsonl` ghi
`target_q` và `observed_q` ở 10 Hz. Clamp count lớn hơn 0 nghĩa là đoạn đó bị
thêm giới hạn ở sidecar và phải được nêu khi đọc evidence. Ngay cả khi count
bằng 0, limiter phía producer vẫn làm hardware chậm hơn q thô trong simulator.

## 11. Chuyển Dev Mode / Regular mode

- Chỉ chuyển sau khi sidecar đã thoát và tay/đầu đã về `ZERO TORQUE`.
- Workflow yêu cầu Dev Mode; chỉ high-level được dùng `rt/lowcmd`.
- Theo vendor `xr_teleoperate_v1_6`, `R1 + X` chuyển sang Regular mode. Lần kiểm
  `rt/arm_sdk` trong Dev Mode đã chạy hết command nhưng encoder gần như không
  đổi, nên không dùng transport đó trong workflow này.
- Không đổi mode khi lệnh foreground còn chạy.

## 12. Ghi chú lịch sử

Phiên direct-lowcmd đầu tiên ngày 2026-08-18 đã kết nối Quest, nhận tới sequence
351 và làm đủ 12 encoder chuyển động. Đó là bằng chứng plumbing lịch sử; kiến
trúc sole-owner D003 phải tạo một bounded run mới trước khi tuyên bố đã được xác
nhận trên phần cứng. Đường direct-lowcmd đã bị loại; `teleop.hardware.run_teleop`
nay chỉ đọc `rt/lowstate`.
