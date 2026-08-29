# R1 Quest 3 teleop trên robot thật — Dev Mode tay/đầu

Quy trình chạy Quest teleop cho R1-A5 trong phạm vi **10 khớp tay + 2 khớp đầu**.
`hb_high_level` là publisher `rt/lowcmd` duy nhất; sidecar teleop chỉ gửi target
qua UDP loopback và không target eo hoặc chân.

> **Chưa qua hardware gate.** Trạng thái hiện tại chỉ được xác nhận khi robot
> treo/cố định trên giá. Không có gì trong tài liệu này cho phép chạy trên sàn.
> Các mục chưa đóng: [`hardware/teleop/docs/hardware_gate.md`](../../hardware/teleop/docs/hardware_gate.md).

Cập nhật 2026-08-24: bộ giải trên đường phần cứng đã đổi sang bộ giải vendor
`xr_teleoperate` chạy nguyên xi. Đường cũ vẫn gọi được bằng `HB_TELEOP_SOLVER=coupled`.

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
| Robot SSH                | `$ROBOT` — IP động, xem cảnh báo trên               |
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

Không tiến trình nào ngoài `hb_high_level` được ghi `rt/lowcmd`
([D003](../../decisions/r1_teleop/D003_single_lowcmd_owner.md)).

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

Mặc định là bộ giải vendor, tay + đầu. Đường cũ: thêm `HB_TELEOP_SOLVER=coupled`.

**Bước 4 — Quest.**

1. Wi-Fi `HappyBaby`, mở `https://192.168.1.106:8012/?ws=wss://192.168.1.106:8012`.
2. Chấp nhận cert nếu hỏi. Chọn **Enter VR**.
3. **Chưa bóp cò phải.** Đưa tay/đầu robot và tư thế người vận hành về neutral.
4. Đầu robot phải gần thẳng: sidecar từ chối khởi động nếu `|yaw| > 0.60` hoặc
   `|pitch| > 0.35` rad. Đầu đang limp thì nắn tay cho thẳng trước.
5. Kiểm lần cuối: robot không còn chuyển động chuyển tiếp nào.
6. **Giữ cò index bên phải.** Nếu homing đang bật (mặc định), robot **tự gập
   khuỷu** đưa cẳng tay từ buông thõng lên ngang hướng ra trước, khoảng 9 giây
   ở 0.15 rad/s, **và tự xoay đầu về giữa** nếu đầu đang lệch (từ chặn cơ khí
   2.007 rad về 0 mất ~13 giây), rồi mới bám theo tay bạn. **Giữ nguyên cò suốt lúc đó** — nhả
   giữa chừng là hủy phiên, không để tay ở lưng chừng. Chỉ cẳng tay quét, quanh
   khuỷu bán kính ~0.16 m; vai và cánh tay trên gần như đứng yên.
   Frame hợp lệ đầu tiên được chốt làm `source_zero`,
   encoder tay/đầu hiện tại làm `start_q`. Mọi target sau đó là độ lệch giữa hai
   mốc. Với bản cô lập, chân và eo cũng bị chốt và khoá tại đúng thời điểm này.
7. Homing xong, sidecar **chốt lại cả hai mốc**: `start_q` là tư thế vừa tới,
   `source_zero` là mẫu Quest mới nhất. Nên tay bạn cử động trong lúc homing
   không bị tính thành lệch.
8. Di chuyển **chậm**.

Tắt homing: `HB_TELEOP_HOME=0`. Khi tắt, robot giữ nguyên tư thế tay đang buông
làm mốc và **sẽ không bao giờ giống dáng sim** — envelope ±0.15 rad chỉ cho
dịch 8.6°/khớp, trong khi khuỷu treo tự do lệch tới 78°.

Cò trái **không** dùng để điều khiển. Bấm cò trái giữa phiên thì pipeline dừng và
phải chạy lại từ đầu để lập neutral mới.

Bóp cò phải khi chưa ở neutral: nhả cò ngay, chờ pipeline release, chạy lại từ
đầu. Không vặn tay/đầu sang tư thế bù trong khi controller còn active.

## 7. Giới hạn đang áp

| Chặn ở đâu                                        | Giá trị                    |
| ----------------------------------------------------- | ---------------------------- |
| Producer — vận tốc / gia tốc khớp                | 0.5 rad/s, 1.0 rad/s²       |
| Producer — giới hạn khớp                          | theo asset`R1.urdf`        |
| Producer — nhịp phát                               | 10 Hz                        |
| Sidecar — envelope mỗi khớp so với`source_zero` | ±0.15 rad                   |
| Sidecar — watchdog lệnh vào /`rt/lowstate`       | 0.75 s / 0.20 s              |
| Sidecar — nhịp gửi UTL1                            | 100 Hz                       |
| Owner — slew                                         | 0.30 rad/s                   |
| Owner — PD tay                                       | kp 40, kd 2                  |
| Owner — giới hạn đầu                             | yaw 1.0 rad, pitch 0.62 rad  |
| Owner — timeout UTL1                                 | 300 ms                       |
| Bản cô lập — khoá chân/eo                       | kp 20, kd 3, slew 0.20 rad/s |

Không nới bất kỳ giá trị nào trong bảng này mà chưa qua hardware gate.

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

**`[SAFE] head not neutral`** — chỉ xảy ra khi **homing tắt**. Khi homing bật,
đầu lệch được chính homing đưa về nominal, nên gate chuyển xuống chạy sau đó và
kiểm kết quả (`head_not_neutral_after_home`) bằng **encoder đo được**, không phải
giá trị vừa ra lệnh.

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

**`input_watchdog`** — producer không đưa target mới trong 0.75 s; owner đã release.

**Thiếu listener `127.0.0.1:5560`** — high-level chưa chạy hoặc sai config. Không
chạy direct-lowcmd để lách kiểm tra này; đường đó đã bị loại.

## 10. Evidence

```text
workstation : results/smoke/<UTC>_r1_quest3_hardware/
robot       : /home/unitree/HB/teleop/logs/<UTC>_r1_high_level_teleop/
bản cô lập  : ~/HB/high_level_lock/logs/<UTC>_run_lock.log
```

Trên robot, `metadata.json` ghi `joint_names`, `motor_indices`, `target_mode`,
`head_valid` và envelope của phiên; `samples.jsonl` ghi `target_q` và `observed_q`
ở 10 Hz. Đây là hai file để đối chiếu tay/đầu có bám không.

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
