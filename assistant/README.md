# R1: hội thoại và tác vụ trên robot

Code tích hợp nằm ngoài `third_party/Happy-Baby-R1-develop`. Dùng HB voice hiện
có để nghe/nói, gesture socket cho cử chỉ, P4/P6 cho tìm/chỉ và APV1 cho tiếp
cận. Controller HB vẫn là nguồn duy nhất ghi `rt/lowcmd`. Runtime không cần
workstation; hội thoại và grounding cần Internet/OpenAI như bản hiện có.

Một tác vụ chuyển động tại một thời điểm, không xếp hàng hoặc tự thử lại.
Quan sát camera, gesture, tìm/chỉ và tiếp cận cùng đi qua bộ điều phối cổng
loopback5595. Mã tác vụ phân biệt admission, tiến độ, kết quả và hủy. Câu nói
thông thường tiếp tục dùng persona/audio của HB. Tool tìm/chỉ và tiếp cận
chờ kết quả thật trước khi trả lời; không báo hoàn thành từ ACK.

## Đóng gói trên workstation

Để test riêng theo develop nhóm mà không đồng bộ vào HB dùng chung, dùng
`--isolated-hb`:

```bash
python3 scripts/interaction/run_r1_assistant.py pack \
  --hb-source-root /home/ubuntu22/Projects/Happy-Baby-R1-develop \
  --isolated-hb \
  --walk-policy flat_plus_gait_0917_armfree \
  --output-dir data/packages/r1_assistant_private_v1 \
  --install-root /home/unitree/r1_assistant/releases/private_v1
```

Gói này chứa snapshot source HB đã loại build, môi trường Python và model.
`--walk-policy` chỉ sửa `locomotion.yaml` trong snapshot riêng, đồng thời pin
target-control theo đúng policy/contract/hash đó; cấu hình nhóm không đổi.
Sau khi chép gói lên robot, chạy `python3 bin/r1-assistant prepare-baseline
--root "$PWD"` từ release. Source/config riêng ở
`/home/unitree/r1_assistant/baselines/private_v1`, owner build riêng ở
`/home/unitree/r1_assistant/owners/private_v1`. Model, SDK và môi trường voice
có sẵn được liên kết để đọc; hai policy được kiểm hash. Không đồng bộ hoặc
sửa source/config trong `/home/unitree/HB`. Doctor kiểm baseline riêng này.
Installer cũng tự chuẩn bị baseline nếu chưa có.

Khi kích hoạt test, cùng service `hb_high_level` tạm dùng binary riêng;
chỉ có một owner điều khiển phần cứng. Rollback trả service về bản nhóm.
Không chạy `./hb deploy` song song trong lúc test. Mặc định install là shadow.

Sửa preset tại `config/robot.json` và `config/approach.patch.json` nếu cần đổi
đường dẫn, phạm vi quét hoặc tham số. Pack tạo thư mục mới và snapshot đầy đủ;
không ghi đè release cũ. Ví dụ từ root workspace:

```bash
python3 scripts/interaction/run_r1_assistant.py pack \
  --hb-source-root /home/ubuntu22/Projects/Happy-Baby-R1-develop \
  --output-dir data/packages/r1_assistant_v1 \
  --install-root /home/unitree/r1_assistant/releases/v1
python3 scripts/interaction/run_r1_assistant.py check --root data/packages/r1_assistant_v1
```

Gói chứa code dự án, calibration, cấu hình, manifest SHA256 và launcher.
`--hb-source-root` chỉ đọc checkout develop của nhóm: lấy `walk_policy` từ
file hiện tại (kể cả thay đổi chưa commit), kiểm ONNX hash/contract và ghi
`app/hb_baseline.json` với hash controller/config/voice/integration. Hai đường
import target-control trong gói nhận cùng policy. Không tự chọn policy theo
tên commit hoặc copy đè vào `third_party`. Bỏ tùy chọn này chỉ phù hợp pack
test baseline cũ; gói triển khai cho nhóm nên luôn truyền đường dẫn trên.

Trước dựng owner và trong doctor, nếu `/home/unitree/HB` lệch baseline đã pin,
installer dừng. Đồng bộ baseline bằng `./hb deploy` của nhóm rồi đóng gói lại;
không bỏ gate hay đổi policy trên robot để ép cài. Receiver APV1 chỉ được
overlay nếu đúng phiên bản đã review; receiver mới của nhóm cần review lại.
Sau khi chạy assistant, muốn deploy controller/voice của nhóm thì rollback
assistant khi DISARMED trước, deploy HB và tạo release mới theo baseline mới.
Hai cơ chế cùng quản lý service HB nên không chạy deploy song song.

Không có key, policy weights hoặc DAC weights. Gói thông thường không chứa
source HB; gói `--isolated-hb` có snapshot source HB để test riêng. Cần giữ model TensorRT đã
build trên robot ở `/home/unitree/r1_local_approach/models/dac01` và môi trường
`/home/unitree/r1_approach_local_env`; model/calibration/engine được kiểm hash.
Không dùng engine x86 thay engine Jetson. API key dùng `/etc/hb/stack.env`.

## Cài và chạy trên robot

Chép **nội dung** gói vào đúng install root, ví dụ qua rsync/scp. Trong các
lệnh dưới, đường dẫn chỉ tới release đã được chép, không phải checkout HB:

```bash
cd /home/unitree/r1_assistant/releases/v1
python3 bin/r1-assistant setup-env --root "$PWD"
# Treo/đỡ robot, DISARMED theo SOP trước thao tác service/controller.
sudo python3 bin/r1-assistant install --root "$PWD"
```

Install mặc định shadow: hội thoại/ảnh camera hoạt động; gesture/tìm-chỉ
không gửi chuyển động; tiếp cận tạo preview. Installer dựng và CTest một
controller riêng ở `/home/unitree/r1_assistant/owners/v1`, chỉ thêm receiver
APV1 nếu thiếu. Policy/config/gains được giữ từ HB; thư viện/policy dùng
symlink chỉ đọc về cây HB. Không sửa source hay binary bên trong HB.
Gate cài đặt gồm 10 bài CTest cho safety/input/teleop/gait/preflight và bài
APV1 handoff; đây là tập kiểm tra liên quan, không phải toàn bộ suite HB.
Không có controller thứ hai: **cùng** service `hb_high_level` chuyển sang
binary ở cây riêng sau khi owner cũ đã dừng. Giữ nguyên preflight HB,
manual priority, mode gate, START lock, watchdog và E-stop. APV1 mới hỗ trợ
Release5 sau STOP trong cùng session để tác vụ tìm/chỉ kế tiếp có thể nhận
lại yaw; timeout đơn lẻ vẫn giữ zero latch. Không bật lại sender cũ.

Để cài trực tiếp chế độ thực thi, dùng lệnh dưới **thay** lệnh install
shadow, sau khi đã chuẩn bị khu vực và người giữ E-stop theo SOP:

```bash
sudo python3 bin/r1-assistant install --root "$PWD" --execute --operator-ready
```

Cờ này tạo snapshot execute riêng, mở APV1 nonzero trong môi trường owner
và cho phép các tác vụ. Không tự arm, không tự chuyển R2+A hoặc phát tác vụ.
Sau cài, người vận hành dùng quy trình tay cầm HB, chọn R2+A/walk và cấp
quyền tay/đầu theo baseline. START khóa base thì tiếp cận vẫn có thể bị
owner từ chối; không bỏ khóa để ép chạy. Chỉ gọi tác vụ khi trạng thái và
quyền tương ứng đã đúng. Native gesture vẫn cần Dev gate của voice HB.

Installer lưu cấu hình hiệu lực và `rollback.json` trong thư mục activation
được in ra cuối lệnh. Gói release giữ nguyên. Có bản đang cài thì rollback
trước khi cài release/chế độ khác; không đổi mode khi robot đang chạy.
Installer chặn high-level drop-in cũ, task marker đang tồn tại, model sai
hash, policy tìm/chỉ lệch walk_policy hoặc voice không nạp được extension.

Các service search/approach cũ và Quest được dừng, có systemd condition
ngăn tự chạy lại trong lúc assistant được bật. Voice được thêm drop-in
ngoài source để nạp extension; persona/mic/loa vẫn dùng HB. Chạy lại deploy
HB của nhóm có thể thay unit/config; hãy check và cài lại tích hợp phù hợp,
không vận hành hai stack cùng lúc.

```bash
systemctl status hb_r1_assistant hb_r1_assistant_agent hb_voice hb_high_level
journalctl -u hb_r1_assistant -u hb_r1_assistant_agent -n 80 --no-pager
python3 bin/r1-assistant request status
python3 bin/r1-assistant request gesture --gesture-id wave
python3 bin/r1-assistant request find_and_point --query "chiếc ghế đỏ"
python3 bin/r1-assistant request approach --query "bình cứu hỏa"
python3 bin/r1-assistant request cancel
```

Qua mic: “Vẫy tay chào mình”, “Tìm và chỉ chiếc ghế đỏ”, “Tiến tới bình
cứu hỏa”, “Dừng lại”. Trả lời câu hỏi trái/phải của tìm/chỉ sẽ dùng cache
có hạn của pipeline hiện hữu. Tiếp cận hiện chỉ grounding vật nhìn thấy
trong ảnh đầu; nếu chưa thấy, yêu cầu tìm/chỉ trước rồi yêu cầu tiến tới.
Chưa có chuỗi tự quét toàn phòng rồi đi hoặc tránh người/vật cản động.

Tìm/chỉ cần bật quyền tay/đầu bằng START trong R2+A. Dev gate cho cử chỉ
và START cho luồng UTL1 là hai quyền riêng. Nếu log báo `[Teleop] TẮT (START)`,
bấm START một lần để bật trước khi yêu cầu tìm/chỉ; xác nhận log `[Teleop] BẬT`.
Đầu không bám góc đích sẽ dừng tác vụ trước bước chụp ảnh; đó không phải lỗi
người dùng giữ vật không ổn định.

Cancel bằng lời cần voice nhận được lượt mới; phụ thuộc mic gate và trạng
thái Realtime. Lệnh HTTP/CLI cancel độc lập với lượt nói. E-stop/tay cầm
luôn là đường dừng phần cứng theo SOP, không bị thay bởi voice.

## Tiếp cận: hành vi và giới hạn hiện tại

Preset assistant hiện bật `visual_segments` trong `config/approach.patch.json`.
Luồng này dùng điểm vật đo trực tiếp trong `pelvis_link`, không dùng vendor
odometry để xác định còn cách vật bao xa. Runtime phát lệnh qua APV1 hiện có:
`ACQUIRE → ADVANCE → BRAKE → RECHECK → ADVANCE`.
Khi bắt đầu BRAKE, control báo sensor chụp ngay ở cơ hội RPC kế tiếp,
không đợi lệnh về 0 hoặc đợi hết chu kỳ thu ảnh. Request đang chạy được hoàn
thành trước, luôn chỉ có một RPC. Sensor vẫn thu liên tục; bộ đệm giữ ảnh mới
nhất, tracking và depth chạy song song trên cùng frame như trước.
Luồng này bỏ subscriber/ghép vendor odometry không cần thiết; giữ LowState
để biến đổi từ camera sang thân. Phép đo depth và hiệu chuẩn không thay đổi.

Ảnh đầu tiên có capture bắt đầu sau BRAKE được giữ thành
`transition_measurement` với timestamp/sequence gốc và xử lý khi đang giảm tốc.
Một phép đo mới sau khi lệnh về 0 cho phép đi đoạn tiếp theo. Ảnh chụp trong
lúc giảm tốc không được coi là vị trí cuối; chưa có bộ đo chuyển động chân
tin cậy để bù quãng còn đi. `stationary_verified=false`: lệnh 0 không chứng
minh thân đã đứng yên. Nếu camera quá hạn khi đi, lệnh về 0 ngay và chuyển
RECHECK; chỉ trạng thái chờ tại lệnh 0 được phép đợi tối đa 2 giây. Không
gia hạn tuổi ảnh 600 ms để tiếp tục bước. Mất identity, cancel, quá ngân sách
hoặc quá hạn đo lại đều kết thúc tác vụ, đường đóng APV vẫn gửi STOP/release.

Tham số `segment_commanded_m=0.2` là tích phân vận tốc lệnh để kích hoạt
BRAKE, **không phải robot đã đi đúng 20 cm hoặc số bước đo được**. Quãng giảm
tốc còn nằm sau mốc đó. `max_drive_s=4` chặn một pha phát lệnh kéo dài;
`max_total_commanded_m=3` và `max_duration_s=95` chặn toàn tác vụ. Bộ điều khiển
giữ giới hạn vận tốc/gia tốc từ gait config; mỗi đoạn tăng tốc lại từ 0.
Điều kiện đích dùng range trực tiếp và standoff/tolerance đang cấu hình,
chỉ trả `estimated_visual_goal`, `physical_arrival_verified=false`.
Đây là triển khai segment theo lệnh + đo lại bằng ảnh, chưa phải leg odometry.

`segment_basis` (tùy chọn, mặc định `all_commands`) chọn phần tích phân lệnh
dùng để kết thúc đoạn: `all_commands` cộng cả ramp dưới ngưỡng gait;
`above_gait_threshold` chỉ cộng phần lệnh có `vx > gait_threshold_mps`
(mặc định 0.15). Status mỗi tick có `segment_command_profile` tách
ramp / trên ngưỡng gait / BRAKE (thời gian và tích phân lệnh). Replay xác định
với preset hiện tại cho 0.55 s (11 tick) trên ngưỡng, khớp lượt thật
`4e1b0dc0…` (~0.55 s, 12 tick); với `above_gait_threshold` là 1.30 s. Cả hai vẫn
là tích phân lệnh, không phải quãng đo được.
Preset từ v10 bật `look_once`: đứng yên lấy trung vị `look_once_frames` (5) khung,
nếu |bearing| > `look_once_align_tolerance_rad` (0.08) thì quay tại chỗ (pha
`ALIGN`, tối đa 3 lần) rồi nhìn lại; sau đó đi mù một đoạn
`plan = min(look_once_max_m, khoảng cách − standoff − look_once_overhead_m)`, dừng,
nhìn lại, lặp tới standoff (tối đa `max_segments`, tổng `max_total_commanded_m`).
Trong `ALIGN/ADVANCE/BRAKE` không xét ảnh, tracker hay đích. Bằng chứng (E003):
quãng thước/tích phân lệnh 0.875–1.01 ở 5 đoạn thẳng 0.25–0.55 m; độ sâu gần như
phẳng 1.3–1.4 m khi thước 1.5–2.0 m nên từ xa robot đi từng đoạn ngắn. Góc quay
`ALIGN` chưa được đo trên phần cứng.
Từ v11, sau lần nhìn hợp lệ đầu tiên, look-once không hủy vì nhận thức: tracker
`lost_latched`, lỗi perception/sensor, worker chết hoặc hết `recheck_timeout_s` chỉ
chuyển sang ước lượng quãng (`còn lại = khoảng cách lần nhìn cuối − tích phân lệnh
từ đó`) và đi tiếp tới standoff (`estimated_goal_dead_reckoned`, `vision_lost` ghi lý
do). Chỉ hủy khi chưa từng có lần nhìn hợp lệ. Vẫn dừng bởi cancel/E-stop, ACK owner,
ngân sách quãng/thời gian và giới hạn tốc độ/gia tốc.
Giọng nói (v22): công cụ chỉ trả cho mô hình `status`, `answer`, `task_id` (chi tiết vẫn ở log dịch vụ), câu
`answer` ngắn tự nhiên ("Tới nơi rồi nè.", "Mình tới gần rồi.", "Mình không thấy nó nữa…"), và lời dặn cấm giải thích
kỹ thuật khi kết thúc. ElevenLabs: `pack --isolated-hb --tts-provider elevenlabs` đặt `tts.provider` trong bản sao HB;
key/voice id lấy từ `/etc/hb/stack.env` (`ELEVENLABS_API_KEY`, `ELEVENLABS_VOICE_ID`). Thiếu key thì lớp mở rộng
tự dùng giọng OpenAI và ghi lý do vào log `hb_voice`; thêm key rồi restart `hb_voice` là đổi sang ElevenLabs.
Xuất sang HB: `python3 scripts/interaction/export_r1_assistant_hb.py --hb-root <HB> [--force]` tạo `assistant/`
(bản mirror có `EXPORT.json` ghi hash và commit nguồn), giống quy ước `object_search/`.
Tìm lại vật bị dời (v19, `search.relocate_rounds` 2, `relocate_time_s` 150): mỗi lượt đi tới kết thúc bằng
camera (`estimated_visual_goal`) là kết quả cuối. Kết thúc bằng ước lượng quãng (tracker mất) thì chạy thêm một
lượt: định vị mới hoặc xác nhận đích mà không đi, hoặc đi theo chỗ mới của vật. Lượt không thấy vật
(`NOT_FOUND`, căn thẳng không hội tụ, hết giờ nhìn) thì chạy lại tìm + quay thân (quét đầu, quay 180°) rồi đi tiếp.
Các lượt dùng chung một ngân sách quãng (`max_total_commanded_m`). Kết quả có `approach_rounds`,
`camera_confirmed`, `object_moved`. Chưa chạy trên robot.
Tracker (v15): `template_refresh_score` 0.9 làm mới mẫu khi khớp mẫu hiện tại ≥ 0.9 (ít nhất mỗi 3
khung), còn danh tính bị neo bởi `seed_floor_score` 0.3 so với mẫu gốc; `recover_geometry` cho phép
bỏ qua một khung `geometry_gate` hỏng (giới hạn `recovery_max_frames`/`recovery_max_s`). Khi tracker
mất, 8 khung JPEG gần nhất và chẩn đoán của khung hỏng được lưu ở `<run>/tracker_lost/`. Mô phỏng:
mẫu cố định mất sau 19 khung, mẫu làm mới giữ 26 khung và dừng đúng khi nội dung biến thành vật khác.
Chưa chạy trên robot; làm mới mẫu không phải nhận diện lại theo ngữ nghĩa.
Cổng đồng ý camera (v14): `inspect_camera`, `find_and_point` và `approach_object` chỉ chạy
sau một transcript người dùng mới, chưa dùng, trong 60 s (một câu nói mở đúng một lần chụp;
một chữ lửng như "cái", tiếng động hoặc lần gọi do mô hình tự quyết bị từ chối với
`not_requested`; câu trả lời ngắn "có/ừ/ok" được chấp nhận). Transcript lấy bằng cách
bọc `LatencyTracker.set_transcript` của hb_voice; không có nguồn transcript thì công cụ từ
chối (fail closed). Lý do: log 21/11 11:37 cho thấy mô hình tự gọi `inspect_camera` sau một
sự kiện ngắt âm thanh mà không có lời yêu cầu nào được ghi.
`approach_object` (v11, chế độ execute, `search.approach_search: true`): chạy P4
`--face-only` (quét đầu/quay thân như tìm vật; khi thấy thì thu tay, đầu về 0°, quay
thân tới `target_heading_deg` bằng vòng yaw đo được, không chỉ tay), release agent,
rồi chạy approach look-once ở trên. Không thấy vật thì dừng ở bước tìm. P4 gọi agent
qua socket trong cùng tiến trình (`AgentBackend`) thay cho một tiến trình
`local_bridge.py` mỗi RPC. Preset nhìn lại dùng 3 khung. Chuỗi tìm → quay → đi chưa chạy
trên robot; bàn giao owner giữa cổng yaw P4 và APV approach chưa được kiểm chứng.
Chưa có cảm nhận chướng ngại vật trong luồng này; chạy thực vẫn yêu cầu
`clear_corridor_confirmed`, `planner.corridor.operator_confirmed` và
`gait_approach.execute_enabled` cùng quyền owner hiện có. Preset không tự bật execute.

Để quay về controller cũ đặt `visual_segments.enabled=false`.
Preset `approach.initial_odom.patch.json` tắt rõ tính năng này để không ghép
hai chế độ không tương thích. Đóng gói bằng workflow `pack` hiện có; bản nạp
cũ trên robot không tự cập nhật khi sửa preset workstation.
Kiểm thử replay không motor, gồm nhiều đoạn và một lần camera mất 0.9 giây:
`PYTHONPATH=experiments/r1_object_grounding/p6_robot_local:. python3 -m unittest test_visual_segments test_sensor_latency -v`.
Replay dùng mô hình động học lý tưởng, không chứng minh khoảng cách bước thật.

Tracker có `recovery_max_frames=3`, `recovery_max_s=0.6` trong preset.
Các lỗi flow/thiếu feature được thử lại từ ảnh và hình học tốt cuối cùng;
frame lỗi không cập nhật reference, corners hoặc feature points. Trạng thái
`recovering` không có depth mask hợp lệ và buộc visual_segments đưa lệnh về 0,
chuyển RECHECK. Chỉ ảnh mới vượt lại tất cả kiểm tra LK, RANSAC và mẫu cố định
mới được đo depth và tiếp tục. Quá ngân sách/gap hoặc sai appearance/geometry,
timestamp, identity vẫn mất bám và dừng; không tự chọn vật khác.
Giá trị mặc định khi không cấu hình recovery là 0 để các luồng cũ giữ hành vi.
Nguyên nhân dừng run `d65cdd5049b244cf8de6b53502a59125`: một frame
`insufficient_consistent_features` lập tức khóa `lost_latched` ở bản trước;
log không lưu đủ ảnh lỗi để xác định đó là nhòe, che khuất hay thay đổi cảnh.
Tracker mới ghi thêm số feature nhất quán, số tối thiểu và số feature đầu vào.
24 test tracker/segment/camera chạy đạt trên robot. Backup cập nhật:
`/home/unitree/r1_assistant/backups/tracking_recovery_20261119T061955440494Z`.
Code đã nạp; config recovery nằm trong staged `approach.visual_segments.json`,
cần chạy lại lệnh activation shadow để restart và nạp đồng thời controller/tracker.
Camera thật shadow run `070b08cee51f4b818a2cae4cd1ad0272` xử lý 99 phép đo
tracked, một lần `recovering` (5 feature nhất quán/45 đầu vào, yêu cầu 8).
Control xác nhận vx=wz=0 trong RECHECK tại lần lỗi đó, sau đó tracked và đi
tiếp trong mô phỏng lệnh tới segment 8; kết thúc theo ngân sách 20 giây, không
lost_latched. Không có request motor. Decoder có cảnh báo JPEG bất thường
trong lượt này; chưa đủ chứng cứ gắn nguyên nhân đó với frame lỗi của lượt cũ.
Raw log/config nằm tại `/home/unitree/r1_assistant/runs/segments_shadow/070b08cee51f4b818a2cae4cd1ad0272/`.

Lượt hardware một đoạn ngày 2026-11-19 (operator xác nhận hành lang/E-stop):
run `47b50a52511648859d4523ca08f85d85` dừng ngay tại ACK đầu không khớp,
STOP/release đều ACK. Đã sửa handoff: sau APV Hello luôn gửi command 0,
reset ramp controller rồi mới tăng tốc ở tick sau; không nới slew của owner.
ACK lỗi hiện ghi status và vận tốc request/echo để chẩn đoán.
Run sau sửa `4e1b0dc0284b40939efb164bbd870fa5` có 42/42 lệnh chuyển động
được ACK, max vx=0.16 m/s, kết thúc `segment_trial_complete`, STOP/release ACK,
controller sau đó báo vx=vy=yaw=0. Tích phân lệnh 0.212 m; range camera
1.542→1.215 m, chưa có đo thước xác nhận quãng đường thực. Không suy ra
độ chính xác chân/arrival từ hai con số này. Lượt thử giới hạn `max_segments=1`,
`max_duration_s=6`, `max_total_commanded_m=0.5`; không tự đi đoạn thứ hai.
Raw log/resolved config nằm dưới `/home/unitree/r1_assistant/runs/single_segment_trial/`.
Runner tại robot: `/home/unitree/r1_assistant/r1_single_segment_trial.py`
(có chuyển động thật; không tự replay). Voice service vẫn shadow; runner
chỉ bật execute trong cấu hình riêng của lượt thử. Bản runtime mới cần restart
assistant khi muốn dịch vụ nạp handoff fix, không cần restart controller.
Operator sau lượt thử báo chỉ dịch chuyển khoảng 5–7 cm (quan sát, chưa xác
nhận đo thước). Lệnh span 2.145 s nhưng chỉ 12 tick vượt vx>0.15 m/s,
span 0.550 s; tích phân 0.212 m gồm cả ramp dưới ngưỡng gait. Cuối lượt,
ảnh nguồn già từ 0.570 tới 0.620 s nên BRAKE chuyển lệnh về 0 vì quá hạn.
Do đó không dùng giảm range 0.326 m làm quãng đi, không fit hệ số bước từ
lượt này. Cần tách thời gian/tích phân lệnh vượt ngưỡng gait khỏi ramp và
khắc phục gián đoạn ảnh trước khi thử đoạn dài hơn.

Đã nạp code vào private release `20261118_private_flat_v3`, backup
`/home/unitree/r1_assistant/backups/visual_segments_20261119T060052316313Z`.
15 test segment/camera chạy đạt trong Python của robot. Lượt camera thật
shadow `659030786a2947a58ee7854faaca72be` nằm tại
`/home/unitree/r1_assistant/runs/segments_shadow/`: control chạy 20 giây,
xử lý 109 ảnh, 24 timeout, tổng trễ trung vị 165 ms (max 283 ms), bắt đầu
6 đoạn mô phỏng lệnh. Trong 4/5 pha BRAKE hoàn thành có ảnh hợp lệ được
yêu cầu sau chuyển trạng thái 0.3–8.6 ms; pha còn lại mất ảnh trước lệnh 0.
Không có đề xuất chuyển động dùng ảnh quá 600 ms, không có request động cơ.
Tác vụ kết thúc đúng ngân sách thử `task_timeout`, không phải xác nhận tới vật.
Đây là một lượt shadow trên camera thật, chưa chứng minh chuyển động vật lý.
Raw log, resolved config và `segments_assessment.json` được giữ cùng run.
Tái chạy bằng `/home/unitree/r1_approach_local_env/bin/python /home/unitree/r1_assistant/r1_run_segments_shadow.py`
(tạo run mới, giữ khóa camera chung). Cấu hình mới được staging ở
`config/approach.visual_segments.json` trong private release; dịch vụ đang
chạy chưa được đổi cấu hình cho tới khi chạy trên robot:
`sudo python3 /home/unitree/r1_assistant/r1_activate_segments_shadow.py`.
Lệnh này kiểm tra idle/shadow, sao lưu cấu hình và chỉ restart assistant,
không bật execute hoặc restart controller.

Luồng camera và perception đã chạy độc lập qua bộ đệm chỉ giữ ảnh mới nhất.
Preset assistant đặt `min_depth_interval_s=0.1` để tránh nhịp depth 200 ms
làm ảnh chờ trong khi camera thu mỗi 167 ms. Đây là giới hạn khoảng cách
giữa hai lần bắt đầu xử lý, không bảo đảm inference đạt 10 Hz.
Đo shadow ngày 2026-11-19 trên robot: run `f6a2a5f1facd434b9548451546187635`
(200 ms) có tuổi ảnh đầu vào trung vị 117 ms, tổng trễ 276 ms; run
`66c515884d5040b5b2d7b2dbb1473e89` (100 ms) tương ứng 18 ms và 161 ms.
Đây là hai lượt liên tiếp, không phải benchmark kiểm soát đầy đủ. Cả hai
vẫn dừng vì ảnh quá hạn sau timeout. Thử RPC timeout 200 ms trong run
`23490129021749f1a85f277759cfb507` không giải quyết lỗi, đã trả về 100 ms.
Phép đo chỉ camera `runs/camera_only_20261119.jsonl` trên robot có 112 ảnh
hợp lệ và 11 timeout trong 20 giây, khoảng trống ảnh tối đa 601 ms:
lỗi nhận ảnh còn tồn tại khi không chạy depth/tracking. Các run và cấu hình
gốc nằm dưới `/home/unitree/r1_assistant/runs/`; chưa chứng minh đã hết mất ảnh.
Thử camera 4 Hz (`camera_only_4hz_20261119.jsonl`) có 68 ảnh hợp lệ và
16 timeout/20 giây; không cải thiện nên đã trả nhịp camera về cấu hình gốc.
Điều tra tiếp ngày 2026-11-19: SDK tối giản tại 6 Hz nhận 120/120 phản hồi
trong 20 giây (`dds_probe_20261119T121434.json`, phản hồi tối đa 28 ms).
Thêm subscriber LowState/odom với callback rỗng vẫn 120/120
(`dds_probe_20261119T121517.json`, tối đa 20 ms), nhưng bộ đếm UDP toàn máy
tăng 18 RcvbufErrors; chưa xác định socket bị mất gói. Worker đầy đủ không
chạy depth/tracking nhận 106 ảnh và 19 timeout/20 giây
(`camera_worker_20261119T121534.jsonl`). Các phép đo liên tiếp chưa loại trừ
biến động theo thời gian; cần phân biệt callback/buffering/transport của worker
với dịch vụ camera. Card eth10 không báo errors/dropped ở lần kiểm tra.
SDK robot có lỗi `RequestFutureQueue.Remove` kiểm `id` thay vì `requestId`;
đây là lỗi dọn yêu cầu hết hạn, chưa chứng minh là nguyên nhân timeout.
Chưa sửa SDK/vendor hoặc thông số mạng dùng chung.
`capture_retry_on_timeout=true` trong `config/approach.patch.json` cho phép
gửi lại ngay sau lần SDK timeout `3104` đầu tiên của mỗi đợt lỗi; nếu tiếp tục
timeout thì trở lại nhịp camera cấu hình. Chỉ một RPC đang chờ tại client,
không tạo client hoặc request song song; timeout không bảo đảm server đã hủy
request trước. Có thể đặt `false` để quay lại lịch thu cũ. Không tăng RPC timeout,
giới hạn tuổi dữ liệu, vận tốc hoặc thay phép đo depth.

Khởi tạo tác vụ chờ ảnh hợp lệ tối đa 2 giây, bỏ ảnh quá `max_input_age_s`,
thay vì thất bại ngay sau một timeout. Các lỗi khác vẫn kết thúc khởi tạo.
Trong `observations.jsonl`, trường `capture` có `request_to_receive_s`,
`image_receive_gap_s`, `camera_timeout_streak`, `camera_timeouts_total` và
`metadata_wait_s` khi ghép được pose. Đây là thời gian RPC/nhận dữ liệu,
không phải timestamp phơi sáng đã được xác minh. Đối chiếu với `input_age_s`,
`inference_total_s`, `request_to_publish_s` để phân biệt trễ thu ảnh và xử lý.
Counter timeout tính cả packet bị bộ đệm thay thế; observations không lưu mọi ảnh.

Kiểm thử không dùng phần cứng:
`python3 -m unittest discover -s experiments/r1_object_grounding/p6_robot_local -p test_sensor_latency.py -v`.
Thay đổi lịch retry cần đo lại trên robot để xác nhận mức giảm gián đoạn;
kiểm thử giả lập không chứng minh SDK/camera đã hết mất ảnh.

Preset tùy chọn `config/approach.initial_odom.patch.json` được merge lên cấu
hình approach đầy đủ: 3 mẫu depth ban đầu/median để ghim đích, sau đó chỉ
odometry 20Hz, không tiếp tục chụp ảnh. Preset để `execute=false` và không
tự kích hoạt trên robot. Đây là phương án đã có sự cố v26 nêu dưới; cần
đối chiếu khoảng cách thực và odometry trước khi cho phép chuyển động.

Tái sử dụng gait-aware arc: vx≤0.16m/s, |yaw|≤0.2rad/s, standoff danh nghĩa
1m, tolerance đích mô hình0.08m, source age≤600ms, APV1 deadline100ms;
task/path budget từ preset gốc95s/3m. Không tăng gain hoặc giới hạn tốc độ.
Overlay mới giữ camera/tracking/DAC trong pha đi và bật kiểm range jump;
không mặc định dùng `initial_target_then_odom` đã bỏ camera sau pin.
Sai depth, mất tracking hoặc pose/camera quá hạn vẫn có thể làm dừng sớm.

Lượt v26 trước đây có chuyển động thật nhưng operator báo đi vào bình dù
planner báo DONE: [hồ sơ sự cố](../../data/runs/r1_object_grounding/initial_target_odom_20261106_execute02/assessment.md).
Camera extrinsic, depth scale, vendor odom và quãng phanh chưa nghiệm thu;
giữ camera không tự giải quyết các sai số này. Bản này là tích hợp thực thi
thử nghiệm, chưa chứng minh khoảng dừng thực hay phù hợp vận hành gần trẻ.
`estimated_goal` chỉ là đạt đích theo ước lượng; `physical_arrival_verified`
luôn false. Hành lang planner là vùng operator khai báo, không phải bản đồ
vật cản được sensor xác minh. Không dùng khi thiếu vùng trống theo SOP.

## Quay lại stack trước

Đỡ robot và DISARMED, dùng đúng activation được in lúc install:

```bash
sudo python3 bin/r1-assistant rollback \
  --activation /home/unitree/r1_assistant/activations/ACTIVATION_ID
```

Khôi phục unit, marker và trạng thái enable/start trước cài; không xóa log,
model hoặc source. Không restart controller khi ARMED. Không chạy thêm
`install_zero.sh`/`apply_canonical.py` của thử nghiệm cũ vì chúng sửa cây HB.

## Kiểm chứng và bằng chứng

```bash
python3 -m unittest discover -s integrations/r1_assistant/tests -v
python3 -m unittest experiments/r1_object_grounding/p5_voice_search/test_server.py
```

Mỗi task lưu request/trạng thái, cấu hình và kết quả dưới `runs/`; P4/P6 lưu
thêm ảnh/grounding, control trace, observations và STOP ACK. Manifest release
pin source + calibration; manifest owner pin source/config/binary + CTest.
Kiểm phần mềm và loopback mock không xác nhận thăng bằng, đúng hướng chỉ,
khoảng cách thực hoặc hardware readiness. Xem `VALIDATION.md` cho lần kiểm
gói này trên workstation.
