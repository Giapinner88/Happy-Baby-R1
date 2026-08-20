# R1 Quest 3 teleop trên robot thật — Dev Mode arms/head

Quy trình này chạy Quest teleop cho R1-A5 trong phạm vi **10 khớp tay + 2
khớp đầu**. `hb_high_level` vẫn là publisher `rt/lowcmd` duy nhất; sidecar
teleop chỉ gửi target qua UDP loopback và không target eo hoặc chân.
Trạng thái hiện tại chỉ được xác nhận khi robot treo/cố định trên giá, không
phải bằng chứng an toàn để chạy trên sàn.

## 1. Topology đã kiểm tra

| Thành phần | Địa chỉ / interface |
| --- | --- |
| Workstation chạy Quest + IK | `192.168.1.106`, `wlp77s0` |
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

Từ root repo trên workstation:

```bash
make teleop-hardware-prepare ROBOT=unitree@192.168.1.104
```

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

Trên workstation, từ root repo:

```bash
make teleop-hardware \
  ROBOT=unitree@192.168.1.104 \
  HOST_IP=192.168.1.106 \
  DURATION_S=180 \
  CERT_FILE=$HOME/.config/xr_teleoperate/happybaby_192_168_1_106/cert.pem \
  KEY_FILE=$HOME/.config/xr_teleoperate/happybaby_192_168_1_106/key.pem \
  CONFIRM_SUSPENDED_WITH_ESTOP=1
```

Trên Quest:

1. Kết nối Wi-Fi `HappyBaby`.
2. Mở `https://192.168.1.106:8012/?ws=wss://192.168.1.106:8012`.
3. Chấp nhận chứng chỉ cục bộ nếu trình duyệt hỏi.
4. Chọn **Enter VR**; chỉ mở trang chưa tạo dữ liệu pose.
5. **Chưa bóp cò phải.** Đưa robot arms/head về trạng thái ban đầu và đưa
   người vận hành Quest về đúng tư thế neutral đã dùng trong sim.
6. Kiểm tra lần cuối: đầu nhìn thẳng và hai tay đang ở vị trí bắt đầu mong
   muốn; không còn chuyển động chuyển tiếp trên robot.
7. Giữ **cò index bên phải**. Frame hợp lệ đầu tiên tại thời điểm này được
   high-level chốt làm `source_zero`; encoder arms/head hiện tại của robot được
   chốt làm `start_q`. Toàn bộ target sau đó là độ lệch tương đối giữa hai mốc.
8. Di chuyển chậm. High-level giới hạn mỗi khớp trong ±0.15 rad so với neutral
   robot và giới hạn tốc độ 0.30 rad/s.

Cò trái không dùng để điều khiển. Nếu bấm cò trái trong phiên đang chạy,
pipeline dừng và phải khởi động lại để lập neutral mới.

Nếu bóp cò phải khi người vận hành hoặc robot chưa ở neutral, nhả cò ngay,
chờ pipeline release và chạy lại từ đầu. Không cố sửa offset bằng cách vặn tay
hoặc đầu sang tư thế bù trong khi controller còn active.

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

- `connect_count=0`: Quest chưa cùng mạng, chưa chấp nhận cert hoặc chưa Enter VR.
- `deadman_enabled=false`: đang nhả cò hoặc bấm nhầm cò trái.
- `input_watchdog`: IK không đưa target mới trong timeout; high-level đã release.
- Thiếu listener `127.0.0.1:5560`: high-level binary/config chưa được deploy
  hoặc service chưa active; không chạy direct-lowcmd để lách kiểm tra này.

Phiên direct-lowcmd đầu tiên ngày 2026-08-18 đã kết nối Quest, nhận tới sequence
351 và làm đủ 12 encoder chuyển động. Đây là bằng chứng plumbing lịch sử;
kiến trúc sole-owner D003 phải tạo một bounded run mới trước khi tuyên bố đã
được xác nhận trên hardware. Không kết quả nào cho phép chạy trên sàn.
