# HB R1 Teleop

Quest 3 teleoperation cho R1: bridge nhận pose từ headset, giải IK phần thân
trên, và ghi joint target xuống robot.

> **Trạng thái: CHƯA ĐƯỢC PHÉP CHẠY TRÊN ROBOT THẬT.**
> Bộ giải IK có bằng chứng mô phỏng trong `experiments/r1_teleop/quest3_sim_v1/`
> và một smoke test phần cứng giới hạn khi robot treo; chưa có bằng chứng cho
> robot đứng trên sàn. `hb_teleop.service` mặc định **không** được enable và
> hardware gate trong `docs/hardware_gate.md` vẫn chưa đóng đủ.

Cập nhật 2026-08-18: direct-lowcmd smoke ban đầu đã xác nhận 12 encoder có thể
chuyển động. Kiến trúc active sau D003 dùng sidecar không có DDS publisher và
giữ `hb_high_level` làm chủ `rt/lowcmd` duy nhất; kiến trúc này còn cần bounded
hardware validation. Kết quả không cho phép chạy robot trên sàn. Quy trình nằm tại
`../../docs/operations/r1_quest3_teleop_hardware.md`.

## Mốc tay và reset trong runtime trên R1

Runtime dùng `body_relative`: vị trí mốc đi theo vị trí đầu người, còn hướng
phía trước lấy lúc hiệu chuẩn. Đi lại hoặc đứng lên không cộng thêm độ dịch
chuyển cả người vào mục tiêu tay. Đây vẫn là phép xấp xỉ thân người bằng pose
đầu; cúi đầu/tịnh tiến đầu riêng với thân có thể ảnh hưởng mục tiêu tay.

Sau khi mở phiên, nhả cò phải rồi mới bóp để điều khiển. Cò trái reset mốc:
pipeline phát STOP, chờ nhả cò phải, rồi lấy mốc mới ở lần bóp tiếp theo.
Reset không đóng tiến trình. Nhả cò phải bình thường không đổi hướng mốc.

Target được xử lý tối đa 30 Hz, dùng lọc hai mẫu 0.7/0.3. Nhịp giải IK thực tế
vẫn phụ thuộc CPU và độ khó của pose; 30 Hz không phải cam kết tốc độ solver.
Thay đổi này cần được xác nhận bằng thao tác Quest sau khi restart runtime.

## Nguồn gốc

Logic IK/mapping xuất phát từ repo `Happy-Baby-R1` (máy khác). Từ 2026-08-19 thư mục
này là **bản làm việc duy nhất trên máy này** — repo nguồn không có ở đây, nên
`scripts/sync_from_workspace.sh` **không chạy được** (nó fail-closed vì không thấy
`$WORKSPACE/teleop`). Sửa trực tiếp ở đây, và mang thay đổi về repo nguồn khi có dịp;
`SOURCE_BUNDLE.md` ghi các chỗ đã phân kỳ.

Method record: `Happy-Baby-R1/docs/teleop/r1_upper_body_ik.md`

## Bố cục

Gộp 2026-08-19: phần operator (Quest) đã nằm cùng chỗ với phần robot. Xem
`SOURCE_BUNDLE.md` để biết mảnh nào đến từ đâu.

```text
scripts/
  run_quest_teleop.sh    lớp bọc: dựng PYTHONPATH + ROBOT rồi gọi chuỗi upstream
  teleop/                20 script operator (quest_bridge, run_r1_quest3_*, plot, T00x)
  sync, check_vuer, preflight, deploy, install_service
src/
  teleop/r1/             IK + mapping (nguồn sự thật, bị sync_from_workspace.sh ghi đè)
  teleop/hardware/       sidecar + pilot (HB sở hữu, sync KHÔNG đụng)
  assets/R1.urdf         mô hình cho IK (kinematics.DEFAULT_URDF trỏ vào đây)
assets -> src/assets     symlink cho đường dẫn "assets/R1.urdf" trong profile
experiments/r1_teleop/   profile IK T001 (mapping) + T007 (arm_head, whole_upper_body)
third_party/xr_teleoperate/teleop/televuer/   wrapper WebXR mà quest_bridge.py import
env/                     conda spec: tv (bridge Quest), unitree_sim_env (IK)
config/                  teleop.yaml (single editable runtime config)
systemd/                 hb_teleop.service.in (KHÔNG tự enable)
tests/                   kiểm tra tĩnh: an toàn, ownership, không DDS ngoài ý muốn
docs/                    hardware_gate, operations/, safety/, decisions/, r1_upper_body_ik
```

## Đường dữ liệu

```text
Quest 3 --WebXR/HTTPS:8012--> quest_bridge.py (env tv)
   | JSONL pose
   v
run_r1_quest3_hardware_targets.py (mặc định, env unitree_sim_env, IK 10 Hz)
  hoặc
run_r1_vendor_ik_stream.py (vendor Python, R1_A5_ArmIK)
  -> run_r1_vendor_targets.py (env unitree_sim_env, target 10 Hz)
   | JSONL 12 khớp: tay[10] + head_yaw + head_pitch   <- thứ tự theo RobotSpec.hpp
   v  (SSH)
robot: teleop.hardware.high_level_sidecar  --UDP 127.0.0.1:5560 @100Hz-->  run_r1
```

Sidecar phát lại target ở 100 Hz nên nguồn 10 Hz vẫn không chạm watchdog 300 ms của
`run_r1`. Nó không tạo DDS publisher: `run_r1` vẫn là chủ `rt/lowcmd` duy nhất.
`vendor_r1_a5` là backend mặc định; `HB_TELEOP_IK_BACKEND=coupled_dls` chỉ dùng
để rollback tường minh. Hướng dẫn và
giới hạn thử nghiệm nằm trong `docs/operations/r1_quest3_teleop_hardware.md`.

## Chuẩn bị (máy dev)

```bash
# chỉnh trực tiếp config/teleop.yaml trong HB
conda env create -f env/tv.quest.yml
conda env create -f env/unitree_ik.quest.yml
./scripts/preflight.sh                               # kiểm tra tĩnh, chạy được ở máy dev
```

### Cấu hình mạng theo từng Wi-Fi

Các địa chỉ và thông số runtime nằm trong **một file YAML bên trong HB**, để
đổi mạng hoặc tinh chỉnh teleop không cần sửa launcher:

```bash
editor config/teleop.yaml
```

Trong file này chỉnh `network.robot`, `network.host_ip` (đặt `auto` để lấy IP
trên default route), `network.host_interface` nếu máy có nhiều card mạng,
`network.robot_interface`, cùng các mục `bridge`, `ik`, `limits`, `sidecar` và
`safety`. Với runtime chạy trực tiếp trên robot, chỉnh thêm `robot_runtime.*`;
địa chỉ Wi-Fi robot được tự lấy từ `robot_runtime.host_interface`.

Đổi YAML cho phiên laptop:

```bash
./run_teleop.sh
```

Đổi IP/hostname/cổng của runtime robot và cập nhật ngay:

```bash
./scripts/deploy_teleop.sh runtime   # sync YAML, refresh cert, restart service (sudo)
```

### Dùng trực tiếp với Quest, không cần laptop

`robot_runtime` là luồng vận hành thường ngày. Sau một lần deploy runtime,
R1 tự chạy WebXR, vendor IK và camera R1; laptop không tham gia phiên Quest.
Quest và R1 phải cùng LAN/VLAN, Wi-Fi không bật client/AP isolation, và R1 có
IPv4 trên `robot_runtime.host_interface`.

1. Kết nối Quest vào Wi-Fi robot `HB`; lưu bookmark
   `https://hb-r1.local:8012/?ws=wss://hb-r1.local:8012`.
2. Mở bookmark, chấp nhận certificate tự ký lần đầu, rồi chọn **ENTER VR**.
3. Bấm **START** trên remote R1 để bật/tắt quyền teleop ở high-level; bóp cò
   phải trên Quest để gửi pose.
4. Bấm **A phải** trên controller Quest để đổi giữa passthrough và camera R1.
   Nếu khung hình camera cũ quá một giây, view tự về passthrough và không tác
   động quyền điều khiển.

`./scripts/deploy_teleop.sh runtime` cài/bật Avahi mDNS chỉ trên
`robot_runtime.host_interface`, mở WebXR và mDNS khi UFW đang active, tạo lại
certificate có SAN cho `hb-r1.local` và IP
Wi-Fi hiện tại, kiểm tra mDNS/certificate/service/cổng/camera trước khi báo
thành công. Nó không thể kiểm tra Quest Browser hoặc cấu hình isolation của
access point; nếu Quest không resolve `.local`, dùng IP Wi-Fi in ra ở cuối
deploy rồi chạy lại `runtime` sau khi R1 đổi Wi-Fi để certificate có IP mới.

Đường **deploy** độc lập với mạng Quest: `deploy_teleop.sh` thử SSH tới địa chỉ
cấu hình trước, rồi quét các đường Ethernet, Wi-Fi và Tailscale đã khai báo;
chỉ chọn host có `/home/unitree/Foundation_Unitree` và `/home/unitree/HB`.
Đặt `HB_TELEOP_ROBOT_OVERRIDE=unitree@<host>` cho một lượt, hoặc đặt
`HB_ROBOT_CANDIDATES="host1 host2 ..."` để thay danh sách. Tailscale SSH phải
được đăng nhập/xác thực trước. Quest vẫn chỉ cần cùng LAN/VLAN với Wi-Fi R1.

Khi cần dọn mọi phiên runtime teleop HB cũ trên laptop và robot trước khi
chạy, dùng `./run_teleop_all.sh`. Lệnh dừng các pipeline Quest/IK/sidecar
nhận diện được, giữ nguyên `run_r1`/`hb_high_level`; dùng
`./run_teleop_all.sh --dry-run` để quét trước.
Muốn chỉ dừng phiên hiện tại mà không mở phiên mới, dùng
`./run_teleop_all.sh --stop`.

Launcher mặc định chờ `high_armed=1` và chấp nhận `high_state=ZERO_TORQUE`
hoặc `LOCOMOTION` tối đa 30 s. Ở locomotion, cần vận tốc bằng 0 vẫn hợp lệ.
Đặt `safety.require_locomotion: true` nếu muốn chỉ cho phép locomotion.

Nhấn START một lần để bật/tắt teleop; khi bật phát hai tiếng bíp ngắn, khi tắt
phát một tiếng bíp. Terminal/journal cũng ghi `[Teleop] BẬT/TẮT (START)`. Cò phải trên Quest vẫn là
deadman riêng của phiên.

Muốn thử tạm một địa chỉ đã xác minh mà không sửa YAML, dùng biến namespaced,
ví dụ: `HB_TELEOP_ROBOT_OVERRIDE=unitree@192.168.123.164 ./run_teleop.sh`.

Launcher và `check_vuer.sh` dùng cùng YAML, tự kiểm tra IP laptop, cert, cổng
WebXR, SSH robot và trạng thái sidecar trước khi in URL Quest.
Nếu còn sidecar teleop cũ trên robot, launcher sẽ dừng an toàn và in PID/lệnh;
hãy kết thúc phiên cũ trong trạng thái an toàn rồi chạy lại.

Thay đổi `config/teleop.yaml` chỉ có hiệu lực ở phiên mới: nhả cò/dừng phiên
cũ rồi chạy lại `./run_teleop.sh`. Không cần build hoặc restart
`hb_high_level.service`. Nếu sửa Python trong `teleop/`, chạy
`./scripts/deploy_teleop.sh deploy` trước phiên mới; thao tác này cũng không
build hoặc restart service.

`run_quest_teleop.sh` tự gọi `scripts/cleanup_teleop.sh` trước preflight để
dừng bridge/pipeline HB cũ và giải phóng cổng WebXR. Tiến trình lạ chiếm cổng
không bị kill nhầm; script sẽ dừng và báo PID.

Chỉ khi sửa C++/config của `controller` (ví dụ `gestures.yaml` làm đổi trần
tốc độ) mới cần deploy/build và restart high-level theo quy trình HB.

Hai file `*.quest.yml` là môi trường tối thiểu cho đúng chuỗi Goal B. Các file
`*.environment.yml` là snapshot đầy đủ của máy nguồn (có cả Isaac Sim/CUDA),
giữ để truy vết nhưng không cần và không nên dùng cho launcher Quest này.

## Kiểm tra đường Quest trước khi đeo kính

```bash
./scripts/check_vuer.sh
```

Kiểm tra host IP, chứng chỉ (SAN + hạn), cổng 8012 và môi trường `tv`. Chạy cái
này trước; đeo kính lên rồi mới phát hiện hỏng thì mất thời gian hơn nhiều.

## Xem trước rồi mới đẩy

```bash
./scripts/deploy_teleop.sh diff     # rsync --dry-run, KHÔNG ghi gì lên robot
./scripts/deploy_teleop.sh deploy   # chỉ copy file; không restart, không arm
./scripts/deploy_teleop.sh runtime  # sync + restart robot-local vendor IK runtime
```

`deploy` chỉ đồng bộ file. Nó không enable service, không khởi động motor và
không tự arm. Bật service là thao tác thủ công, sau khi đã đọc
`docs/hardware_gate.md`.

Checkout này không có `Makefile`; quy trình chuẩn bị thật là ba lệnh sau
từ `HB/teleop`:

```bash
./scripts/preflight.sh
./scripts/check_vuer.sh
./scripts/deploy_teleop.sh deploy
```

Ba lệnh này **không chạy teleop trên robot**; lệnh cuối chỉ copy
sidecar và asset. Chỉ chạy `./run_teleop.sh` sau khi đóng các
gate phần cứng.

## An toàn

Không vận hành robot thật khi chưa thỏa điều kiện trong
`Happy-Baby-R1/docs/safety/safety_rules.md`: có người giữ E-stop, không chạy
một mình, đã chạy dry-run, và có ghi log kết quả.

SOP smoke test hardware treo trên giá:
[`r1_quest3_teleop_hardware.md`](../../docs/operations/r1_quest3_teleop_hardware.md).

## Video WebRTC và nâng giới hạn teleop

Xem [hướng dẫn kích hoạt và kết quả kiểm tra](docs/operations/quest_webrtc_upgrade.md).
`../deploy_teleop.sh upgrade` apply bản 4 rad/s và runtime khi DISARMED;
`runtime` chỉ cập nhật teleop. Camera mặc định dùng FPS tự điều chỉnh.
