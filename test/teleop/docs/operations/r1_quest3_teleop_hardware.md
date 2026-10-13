# R1 Quest 3 teleop trên robot thật — Dev Mode arms/head

Tài liệu này ghi lại đường teleop **chạy trên laptop**. Với runtime WebXR/vendor
IK hiện chạy trực tiếp trên robot, dùng `./deploy.sh` (toàn bộ HB) hoặc
`./deploy_teleop.sh runtime` (chỉ teleop) từ thư mục gốc HB; xem
`HB/docs/DEPLOY.md`.

Quy trình này chạy Quest teleop cho R1-A5 trong phạm vi **10 khớp tay + 2
khớp đầu**. `hb_high_level` vẫn là publisher `rt/lowcmd` duy nhất; sidecar
teleop chỉ gửi target qua UDP loopback và không target eo hoặc chân.
Trạng thái hiện tại chỉ được xác nhận khi robot treo/cố định trên giá, không
phải bằng chứng an toàn để chạy trên sàn.

## 1. Topology đã kiểm tra

| Thành phần | Địa chỉ / interface |
| --- | --- |
| Workstation chạy Quest + IK | `192.168.1.105`, `wlp111s0` |
| Robot SSH | `unitree@192.168.1.104` |
| DDS nội bộ robot | `eth10`, `rt/lowstate`, `rt/lowcmd` |
| Quest | cùng Wi-Fi `HappyBaby`; phiên đã thấy từ `192.168.1.108` |

Luồng dữ liệu:

```text
Quest Browser/Vuer -> quest_bridge.py -> arms_head IK (workstation)
                   -> JSONL qua SSH -> sidecar UDP 127.0.0.1:5560 (robot)
                   -> hb_high_level ZERO TORQUE arbitration -> rt/lowcmd
```

## 2. Điều kiện bắt buộc

- Robot treo và cố định trên giá.
- Có một người giữ E-stop và một người vận hành Quest/R3.
- Robot đã vào **Dev Mode** và high-level đã tiếp quản: `L2+R2` vào
  Dev Mode, giữ `R1+R2` 3 giây để arm high-level, sau đó bấm `L2+Y`
  một lần để vào `ZERO TORQUE`.
- `hb_high_level.service` phải `active`, ở `ZERO TORQUE`, và chỉ lắng nghe
  `127.0.0.1:5560`; đây là publisher motor duy nhất.
- `hb_teleop.service` phải `inactive` và không được có direct-lowcmd receiver.

## 3. Deploy và kiểm tra kết nối

Từ `HB/teleop` trên workstation:

```bash
./scripts/preflight.sh
./scripts/check_vuer.sh
./scripts/deploy_teleop.sh diff
./scripts/deploy_teleop.sh deploy
```

Checkout này không có target `make teleop-hardware-*`. `deploy_teleop.sh`
chỉ copy file, không restart `hb_high_level`, không start sidecar và không arm.

Đăng nhập robot khi cần kiểm tra thủ công; chọn `foxy (1)` ở menu đăng nhập:

```bash
ssh unitree@192.168.1.104
# ros:foxy(1) noetic(2) ?  -> nhập 1
cd ~/HB/teleop
./scripts/preflight.sh
```

Kiểm tra read-only, không tạo publisher:

```bash
cd /tmp
PYTHONPATH=/home/unitree/HB/teleop/src \
python3 -m teleop.hardware.run_teleop --interface eth10
```

Kết quả mong đợi có `rt/lowstate`, `mode_machine=1`, `motors=35` và dòng
`no publisher was created`.

Xác minh sole-owner IPC sau khi deploy high-level:

```bash
ssh unitree@192.168.1.104 \
  'systemctl is-active hb_high_level.service; ss -H -lun "sport = :5560"'
```

## 4. Chạy teleop foreground

### Nhánh vendor R1-A5 IK (chọn riêng)

`vendor_r1_a5` là backend mặc định và chọn solver `R1_A5_ArmIK` nguyên bản của
Unitree, pin tại commit `845b25a32f7febedf220e830952a7134897adb9d`.
`vendor_r1_a5_845b25b.json` kiểm hash mã solver, hai URDF, meshes và bộ lọc trước khi
khởi tạo. `coupled_dls` chỉ còn là override rollback tường minh qua
`HB_TELEOP_IK_BACKEND=coupled_dls`.
FK của các khớp tay/đầu trùng với asset HB khi biểu diễn tương đối với
`waist_yaw_link`; tọa độ world khác một offset cố định khoảng 5.9 cm do HB có
`waist_roll_link` phía dưới, còn URDF vendor lược phần đó. Không dùng tọa độ
world của hai URDF thay thế trực tiếp cho nhau.
Replay MuJoCo sơ bộ 4 giây với pelvis/waist cố định, collision tắt, 40 target
và PD gần cấu hình HB đã cho `max_command_delta=0.15 rad` nhưng đo
`max_observed_delta=0.291 rad`, `max_tracking_error=0.141 rad` và
`max_joint_speed=15.0 rad/s`. **Gate mô phỏng chưa đạt**; đây chưa phải mô
phỏng đầy đủ bộ điều khiển high-level và chưa xác nhận an toàn phần cứng.

Môi trường solver nằm trên workstation, tách khỏi `tv` và `unitree_sim_env`:

```bash
./scripts/setup_vendor_ik_env.sh   # chỉ chạy khi chưa có môi trường
```

Pipeline vendor: bridge -> `run_r1_vendor_ik_stream.py` ->
`run_r1_vendor_targets.py` -> sidecar. Đầu ra chỉ có 10 khớp tay và hai khớp
đầu; nhả cò phải liên tục 5 giây mới kết thúc phiên. Trong cửa sổ 5 giây H4
phát STOP và cho phép bóp lại để tiếp tục, chốt lại `source_zero` theo encoder
của robot.

Sau khi gate mô phỏng và an toàn được xử lý, chọn nhánh này trong lần thử robot
**đang treo, có người giữ E-stop**:

```bash
./run_teleop.sh
```

Backend, contract, limits and timing are configured in
`config/teleop.yaml` under `ik`, `limits`, `bridge` and `sidecar`.

Chờ dòng `[VENDOR IK READY]` trước khi bóp cò phải. Thiết lập mặc định mới
giới hạn vận tốc target tại `0.5 rad/s`, gia tốc `1.0 rad/s²`; sidecar vẫn giữ
bao lệch phiên `0.15 rad` và watchdog riêng.

Trước lần thử robot đầu tiên, cần xác nhận thứ tự và chiều khớp bằng replay và
mô phỏng. Dữ liệu offline không xác nhận an toàn robot thật. Phiên robot phải
dừng nếu target lệch với chuyển động người vận hành hoặc có rung/va chạm.

Log mới tại `results/smoke/<UTC>_r1_quest3_hardware/` gồm
`quest_commands.jsonl`, `vendor_commands.jsonl`, `targets.jsonl`,
`vendor_ik_stats.json`, `vendor_targets_diagnostics.jsonl`, `ssh.log`,
`robot_receiver.log` và `pipeline_exit_codes.txt`. Đối chiếu `sequence_id`
tại từng đoạn với `last_target_seq`, `valid` và `rejected` trong
`robot_receiver.log` (đồng thời có trong `metadata.json` trên robot) khi gặp
`input_watchdog`. Không tăng watchdog chỉ vì đổi solver.

### Nhánh IK mặc định

Trên workstation, từ `HB/teleop`:

```bash
./run_teleop.sh
```

Địa chỉ robot, IP/cert/key, conda env và các núm điều chỉnh đều đọc từ
`config/teleop.yaml` trong HB. Router đổi lease thì sửa `network.robot` hoặc
để `network.host_ip: auto`; không cần tìm và sửa shell script.

Đặt `safety.confirm_suspended_with_estop: false` trong YAML để launcher hỏi gõ
`YES` trực tiếp. Chỉ để `true` khi robot đã treo/cố định và người riêng giữ
E-stop.

Các núm điều chỉnh:

| YAML                              | Mặc định | Ý nghĩa                                                        |
| --------------------- | -------- | -------------------------------------------------------------- |
| `limits.max_offset_rad`          | `0.15`   | Bao lệch tối đa mỗi khớp so với neutral. Nới dần: 0.15 → 0.35 → 0.60 → 1.5. |
| `limits.head_pitch_max_rad`      | `0.70`   | Ngưỡng đầu và clamp tầm cúi/ngửa.                              |
| `limits.head_yaw_max_rad`        | `0.60`   | Ngưỡng và clamp trục yaw.                                      |
| `bridge.duration_s`              | `1200`   | Độ dài phiên.                                                  |

Bao lệch **không** phải giới hạn tốc độ. Tốc độ do
`teleop_max_rate_rad_s` trong `controller/config/gestures.yaml` quyết định
(hiện `2.0` rad/s ≈ 115°/s — **trần cứng** của validator `Tuning.cpp:333`;
đặt cao hơn thì `run_r1` từ chối khởi động) và chỉ đổi được bằng sửa config rồi
`systemctl restart hb_high_level`.

Trên Quest:

1. Kết nối Wi-Fi `HappyBaby`.
2. Mở URL mà script in ra ở dòng `[READY] Quest URL:` — nó theo IP hiện
   tại của workstation nên **đổi sau mỗi lần router cấp lease mới**.
   Nếu tab từ phiên trước còn mở thì phải **reload**, websocket cũ đã chết.
3. Chấp nhận chứng chỉ cục bộ nếu trình duyệt hỏi.
4. Chọn **Enter VR**; chỉ mở trang chưa tạo dữ liệu pose.
5. **Chưa bóp cò phải.** Đưa robot arms/head về trạng thái ban đầu và đưa
   người vận hành Quest về đúng tư thế neutral đã dùng trong sim.
6. Kiểm tra lần cuối: đầu nhìn thẳng và hai tay đang ở vị trí bắt đầu mong
   muốn; không còn chuyển động chuyển tiếp trên robot. **Đầu phải được kê/đỡ
   ngang ngay lúc này** — gate head-neutral đọc góc ngay sau target đầu tiên
   (tức khi vừa Enter VR), chứ không phải lúc bóp cò; ở ZERO TORQUE đầu limp sẽ
   tự rũ xuống quá ngưỡng nếu buông tay.
7. Giữ **cò index bên phải**. Frame hợp lệ đầu tiên tại thời điểm này được
   high-level chốt làm `source_zero`; encoder arms/head hiện tại của robot được
   chốt làm `start_q`. Toàn bộ target sau đó là độ lệch tương đối giữa hai mốc.
8. Di chuyển chậm. High-level giới hạn mỗi khớp trong `MAX_OFFSET_RAD` so với
   neutral robot, và giới hạn tốc độ theo `teleop_max_rate_rad_s`.

Cò trái không dùng để điều khiển. Nếu bấm cò trái trong phiên đang chạy,
pipeline dừng và phải khởi động lại để lập neutral mới.

Nếu bóp cò phải khi người vận hành hoặc robot chưa ở neutral, nhả cò ngay và
bóp lại trong 5 giây sau khi đưa về neutral. Sau 5 giây pipeline release và
phải chạy lại từ đầu. Không cố sửa offset bằng cách vặn tay hoặc đầu sang tư
thế bù trong khi controller còn active.

## 5. Dừng và E-stop

Dừng bình thường:

1. Giữ nguyên tư thế.
2. Nhả cò phải. Target stream gửi STOP; head nhả ngay, tay giảm quyền
   trong 0.5 giây rồi trở về `ZERO TORQUE`, service vẫn active.
3. Nếu terminal chưa thoát, nhấn `Ctrl+C` một lần.

Dừng bất thường: dùng E-stop ngay nếu có rung, sai chiều, va chạm, tiếng lạ,
mất mạng hoặc target không tương ứng chuyển động người vận hành. Không chờ
watchdog trong tình huống cơ khí bất thường.

Xác minh đã dừng:

```bash
ssh unitree@192.168.1.104 \
  'ps -eo pid,args | grep -E "teleop.hardware.high_level_sidecar|hb_teleop" | grep -v grep || true; systemctl is-active hb_high_level.service hb_teleop.service || true'
```

Không được còn tiến trình `teleop.hardware.high_level_sidecar`.
`hb_high_level.service` phải còn active và chân/eo phải tiếp tục `ZERO TORQUE`.

## 6. Chuyển Dev Mode / Regular mode

- Chỉ chuyển mode sau khi sidecar đã thoát và tay/đầu đã nhả về
  `ZERO TORQUE`.
- Workflow yêu cầu Dev Mode; chỉ high-level được dùng `rt/lowcmd`.
- Theo vendor `xr_teleoperate_v1_6`, `R1 + X` chuyển sang **Regular mode**.
  Lần kiểm tra `rt/arm_sdk` trong Dev Mode đã chạy hết command nhưng encoder
  gần như không đổi, nên không dùng transport đó trong workflow này.
- Sau restart: `L2+R2` -> Dev Mode, giữ `R1+R2` 3 giây -> high-level
  armed, `L2+Y` -> `ZERO TORQUE`.
- Không đổi Dev/Regular mode khi lệnh foreground còn chạy.

## 7. Evidence và xử lý lỗi

Log workstation nằm tại:

```text
results/smoke/<UTC>_r1_quest3_hardware/
```

Log encoder/receiver trên robot:

```text
/home/unitree/HB/teleop/logs/<UTC>_r1_high_level_teleop/
```

Các dấu hiệu thường gặp:

- `connect_count=0`: Quest chưa cùng mạng, chưa chấp nhận cert, chưa Enter VR —
  hoặc **tab cũ còn mở từ phiên trước**. Mỗi lần chạy script là một server vuer
  mới; websocket của tab cũ đã chết và KHÔNG tự nối lại. Phải **reload trang**
  mỗi phiên.
- `[SAFE] head not neutral` / `stop_reason=head_not_neutral`: đầu lệch quá ngưỡng
  tại thời điểm target đầu tiên. Ở ZERO TORQUE đầu limp nên nó rũ xuống theo
  trọng lực — đo thực tế 2026-08-26 là pitch 0.623 rad (35.7°) so với mặc định
  0.35. Kê/đỡ đầu ngang, hoặc nới ngưỡng:

  ```bash
  # sửa limits.head_pitch_max_rad và limits.max_offset_rad trong config/teleop.yaml
  ./run_teleop.sh
  ```

  Gate đọc góc **ngay sau target đầu tiên**, không phải lúc chạy script
  (`src/teleop/hardware/high_level_sidecar.py:213`) — tức đầu phải đúng vị trí
  vào lúc bạn vừa Enter VR. Sidecar thoát mã 3 và kéo cả pipeline chết theo, nên
  triệu chứng nhìn thấy là "robot không cử động".
- `stop_reason=first_input_timeout` ở robot kèm `connect_count=0` ở workstation:
  không phải lỗi robot. Sidecar chờ đủ 120 s mà máy dev không gửi gì vì Quest
  chưa nối. Đọc `bridge_connection.jsonl` trước khi nghi ngờ phía robot.
- `deadman_enabled=false`: đang nhả cò hoặc bấm nhầm cò trái.
- `input_watchdog`: robot không nhận target mới trong 0.75 s và đã release. Nếu
  `bridge_connection.jsonl` vẫn báo `connected` và cò phải vẫn giữ, xem
  `ik_diagnostics.jsonl` để tìm frame IK giải chậm hoặc bị từ chối. Launcher
  hardware giới hạn 6 vòng IK/frame và giữ đúng thứ tự sequence khi luồng
  keepalive gửi xen kẽ. Nhả cò phải liên tục 5 s sẽ kết thúc phiên; trước đó có
  thể bóp lại để resume, sau đó mới cần chạy lại launcher.
- Thiếu listener `127.0.0.1:5560`: high-level binary/config chưa được deploy
  hoặc service chưa active; không chạy direct-lowcmd để lách kiểm tra này.

Phiên direct-lowcmd đầu tiên ngày 2026-08-18 đã kết nối Quest, nhận tới sequence
351 và làm đủ 12 encoder chuyển động. Đây là bằng chứng plumbing lịch sử;
kiến trúc sole-owner D003 phải tạo một bounded run mới trước khi tuyên bố đã
được xác nhận trên hardware. Không kết quả nào cho phép chạy trên sàn.
