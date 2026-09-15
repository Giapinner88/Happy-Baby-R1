# Kế hoạch thu dataset R1 bằng Quest 3 trong Isaac Sim

Tài liệu này là **kế hoạch**, không phải mô tả trạng thái. Mỗi giai đoạn có một
cổng (gate) đo được; không mở giai đoạn sau khi cổng của giai đoạn trước chưa
đạt. Bằng chứng của mỗi giai đoạn nằm trong `D00x/runs/<run-id>/`.

Đường tham chiếu là stack chính thức của Unitree, đã có sẵn trên máy trạm này:

- `third_party/xr_teleoperate_v1_6/` — **bản có hỗ trợ R1 chính thức** (`--arm R1_A5` /
  `R1_A7`, thêm ở v1.6 ngày 2026-07-29). Repo này đã dùng `R1_A5_ArmIK` của nó cho
  `make teleop-arms`.
- `third_party/xr_teleoperate/` — bản v1.5, nơi `televuer` mà `quest_bridge.py` đang import
- `/home/ubuntu22/train_mujoco/unitree_sim_isaaclab/` — phía Isaac Sim, DDS, image server

Mọi thông số camera và định dạng dữ liệu trong kế hoạch này được đọc ra từ hai
repo đó, không phải tự nghĩ. Nguồn cụ thể được ghi ngay tại chỗ dùng.

---

## Ràng buộc đã biết trước khi bắt đầu

| Ràng buộc | Bằng chứng | Hệ quả |
|---|---|---|
| R1 không có bàn tay / gripper | `assets/R1.urdf` — chuỗi khớp kết thúc ở `wrist_roll_joint` | Không thu được dữ liệu thao tác cho tới Giai đoạn 5 |
| Scene teleop hiện tại trống | `scripts/teleop/run_r1_quest3_live.py` chỉ spawn ground + light + robot | Phải thêm vật thể trước khi có task |
| Không có camera gắn robot | Hai camera hiện có là camera thế giới ở `(2.2, ±1.6, 1.5)` | Không có observation ego |
| Kính đang ở chế độ pass-through | `scripts/teleop/quest_bridge.py` đặt cứng `display_mode="pass-through", zmq=False` | Người vận hành đang điều khiển mù |
| Bản sao `unitree_sim_isaaclab` trên máy (17-06-2026) không có task R1 | Task chỉ có G1-29 và H1-2 | Không chạy được đường DDS của Unitree cho R1; xem Giai đoạn 1 |
| Model R1_A5 của chính Unitree cũng **không có bàn tay** | `third_party/xr_teleoperate_v1_6/assets/r1/r1_a5.urdf` — 13 khớp, kết thúc ở `wrist_roll_joint` | Kết luận "không có end-effector" đúng cả với bản chính thức, không phải thiếu sót của asset trong repo này |
| Tầm với vai→cổ tay ≈ 0.267 m (tư thế zero) | FK từ `assets/R1.urdf` | Vật thể phải đặt trong/ngoài tầm một cách có chủ ý |

---

## Phát triển không cần đeo kính

`scripts/teleop/run_r1_quest3_live.py --replay-command-file <raw_commands.jsonl>`
phát lại một phiên teleop đã ghi qua **đúng** đường code của phiên live: cùng
mapper, cùng bộ giải, cùng vòng lặp điều khiển, cùng đường ghi bằng chứng. Chỉ
khác nguồn lệnh là file thay vì stdin.

Hệ quả: mọi giai đoạn dưới đây **trừ Giai đoạn 3** đều phát triển và kiểm chứng
được mà không cần đeo Quest. Chỉ khi cần xác nhận ảnh thật sự tới được kính mới
phải đeo. Kho `T007/runs/` có sẵn hơn 30 phiên đã ghi; những phiên có deadman
bật 100% (ví dụ `t007_waist_held_isaac_20260823_v25`, 46.8 s) là nguồn thay thế
tốt nhất cho một người vận hành.

---

## Giai đoạn 0 — Nền tảng chạy được

**Mục tiêu.** Một phiên teleop có GUI, ổn định nhịp, tái lập được.

**Việc.**
- Chạy `make teleop HOST_IP=<ip> DEVICE=cuda:0 TELEOP_ARGS="--single-view"`.
- Nâng giới hạn inotify để log không ngập lỗi `errno=28`.

**Cổng ra.** Một run trong `experiments/r1_teleop/.../T007/runs/` có
`achieved_control_hz >= 29.0`, `stop_reason` không phải lỗi, và cửa sổ Isaac hiện
hình. Không cần thêm gì mới cho giai đoạn này — chỉ cần xác nhận.

**Ước lượng.** Đã gần xong.

---

## Giai đoạn 1 — Học định dạng dataset

**Mục tiêu.** Hiểu chính xác một dataset đúng chuẩn gồm những gì, trước khi tự
xây. Cổng ra là hiểu biết, không phải số episode.

**Bối cảnh.** `xr_teleoperate` v1.6 đã hỗ trợ R1 chính thức (`--arm R1_A5`), và
`EpisodeWriter` của nó tự chia state/action theo số bậc tự do của tay nên hoạt
động với R1 (5 khớp mỗi tay) mà không cần sửa. Nhưng đường `--sim` của Unitree
đi qua DDS và cần simulator publish `LowState` cho R1 — bản sao
`unitree_sim_isaaclab` trên máy này không có task R1 nào.

**Chọn một trong hai đường.**

*1a — Chạy stack Unitree với G1.* Học đúng thứ cần học, nhưng trên robot khác.
Cần làm trước: cài `unitree_sdk2py` và `teleimager` vào `unitree_sim_env` (đang
thiếu cả hai), tải asset bằng `fetch_assets.sh`, sửa `cam_config_server.yaml` cho
camera đơn của sim, và trỏ `XR_TELEOP_CERT`/`XR_TELEOP_KEY` sang cert có SAN
đúng IP. Khoảng nửa ngày.

*1b — Đọc định dạng từ code và một episode mẫu, rồi đi thẳng sang Giai đoạn 2.*
Repo này đã có đường teleop R1 chạy được với chính bộ giải `R1_A5_ArmIK` của
Unitree (`make teleop-arms`), nên việc chạy lại stack của họ chỉ để xem định dạng
là đường vòng. Đọc `EpisodeWriter` (~230 dòng) và
`teleop_hand_and_arm.py:428-520` là đủ.

**Đường được khuyến nghị: 1b**, trừ khi bạn muốn tận mắt thấy vòng lặp kín
"mắt robot vào kính" hoạt động trước khi tự xây nó — khi đó 1a đáng nửa ngày.

**Công cụ kèm theo.** `tools/inspect_episode.py` đọc một thư mục
`episode_XXXX/` và in ra: số bước, nhãn task, bộ khớp, đối chiếu ảnh tham chiếu
với ảnh trên đĩa, và RMS hiệu giữa `states` và `actions` từng nhóm khớp. Nó cảnh
báo nếu hai thứ đó giống hệt nhau — lỗi âm thầm nguy hiểm nhất của dataset học
bắt chước.

**Cổng ra.** Trả lời được bằng chữ, không phải bằng cảm giác:
1. `states` khác `actions` ở chỗ nào, và vì sao học bắt chước cần cả hai.
2. Ảnh được lưu ở đâu và `data.json` tham chiếu tới nó thế nào.
3. Ranh giới episode do cái gì quyết định.

**Ước lượng.** 1b: một buổi đọc. 1a: thêm nửa ngày dựng môi trường.

## Giai đoạn 2 — Mắt cho robot R1 — ✅ XONG (2026-09-04)

**Đã làm.**
- `teleop/r1/dataset_scene.py`: đọc profile cảnh, dựng bàn + vật, tạo camera đầu,
  ghi ảnh và manifest. Toàn bộ import Isaac Lab nằm trong thân hàm vì module bị
  import trước khi `AppLauncher` chạy.
- `scripts/teleop/run_r1_quest3_live.py`: thêm cờ `--dataset-scene-config`.
  **Không truyền cờ thì đường T007 không đổi một dòng nào** — đó là ràng buộc
  thiết kế, không phải may mắn.
- `tests/teleop/test_r1_dataset_scene.py`: 8 test cho phần đọc cấu hình.

**Cách chạy, không cần Quest:**

```bash
conda run --no-capture-output -n unitree_sim_env python scripts/teleop/run_r1_quest3_live.py \
  --output-dir /tmp/g2_thu \
  --replay-command-file experiments/r1_teleop/quest3_sim_v1/T007/runs/t007_whole_upper_body_20260820T140045Z/raw_commands.jsonl \
  --whole-upper-body-config experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_whole_upper_body_live.json \
  --dataset-scene-config experiments/r1_dataset/quest3_sim_v1/D001/config/r1_d001_reach_point_dataset.json \
  --duration-s 25 --control-hz 30 --physics-hz 200 \
  --device cuda:0 --disable-self-collisions --headless --no-video
```

`--control-hz 30` là bắt buộc: mặc định của script là **50**, còn profile T007 chỉ
được kiểm chứng ở 30. Bỏ quên nó thì `requested_control_hz` ghi 50 trong bằng
chứng và `sim_to_wall_ratio` tụt xuống dưới 1, tức mô phỏng chạy chậm hơn đồng hồ
thật. Mỗi lần chạy phải đổi `--output-dir`: script từ chối ghi đè một thư mục
bằng chứng đã tồn tại, và đó là chủ đích.

**Cổng ra — đạt.** Run 25 s sinh 220 ảnh, `head_camera_frames.json` khớp đúng 220
file trên đĩa, ảnh 640×480 có hình (không đen), và khung hình cho thấy mặt bàn,
hai vật YCB **và cả hai tay robot** — tức chính sách học sẽ nhìn thấy tay mình.

**Điều đo được, không đoán.**

| Cấu hình | achieved_control_hz |
|---|---|
| Cảnh trống, giải liên tục | 15.7 và 19.6 (hai lần chạy cùng đầu vào) |
| Thêm bàn + 2 vật + camera đầu | 15.9 và 12.5 |

**Đừng đọc bảng này thành 'cảnh tốn 3 Hz'.** Cùng một đầu vào, cùng một cấu hình
cho ra 15.7 rồi 19.6 — nhiễu giữa các lần chạy lớn ngang mức chênh lệch cần đo,
nên chi phí của cảnh chưa tách ra được. Điều duy nhất chắc chắn: **mọi lần đo đều
dưới 30 Hz**, nên `fps` của dataset phải chốt theo số đo lặp lại nhiều lần trên
một máy rảnh, không theo mong muốn. Máy này đang ở CPU governor `powersave`, cũng
là một biến chưa kiểm soát.

**Hai sai lầm đã phạm và cách phát hiện.**

1. `--no-video` đặt `enable_cameras=False`, nên camera đầu làm Isaac ném
   `RuntimeError` ngay khi `sim.reset()`. Sửa: có camera đầu thì bật rendering
   bất kể `--no-video`. Chỉ lộ ra khi chạy thật, không lộ khi đọc code.
2. Phiên replay đầu tiên tôi chọn (`t007_waist_held_isaac_20260823_v25`) là phiên
   người vận hành **giơ thẳng hai tay lên trời** suốt 47 giây. Camera đầu không
   thấy tay, và tôi suýt kết luận camera đặt sai. Chỉ sau khi mở video góc thứ ba
   của chính phiên đó mới thấy nguyên nhân thật. Bài học: khi ảnh không có thứ
   bạn mong đợi, kiểm tra dữ liệu đầu vào trước khi sửa mã.

Phiên nên dùng để kiểm chứng: `t007_whole_upper_body_20260820T140045Z` — 67.7% số
bước có cổ tay vươn ra trước ở tầm mặt bàn.

**Còn treo.** Vật YCB nằm ngang chứ không đứng thẳng; đó là tư thế gốc của bản
`Axis_Aligned` chứ không phải do rơi đổ. Muốn chúng đứng thì phải khai thêm
`rot` trong `init_state`, chưa làm.

## Giai đoạn 3 — Đưa góc nhìn robot vào kính — ✅ XONG (2026-09-05)

**Vì sao phải làm trước khi thu dữ liệu.** Tầm với tối đa của tay R1 là
**0.383 m** (đo từ `assets/R1.urdf`: 0.0638 + 0.1138 + 0.0885 + 0.1167), vai
phải ở world (0.033, −0.086, 1.006), chân robot chìa tới x = 0.304 nên bàn không
đặt gần hơn được. Dải x dùng được cho vật vì thế chỉ rộng **9 cm: [0.31, 0.40]**.
Không ai vươn tay trúng dải đó khi kính còn ở pass-through.

**Đã làm.**
- `teleop/r1/head_view_stream.py`: ZMQ PUB/SUB, khung nén JPEG, giữ đúng một
  khung mới nhất ở cả hai đầu. Mỗi khung mang dấu thời gian `time.monotonic()`
  lúc publish — trên Linux đây là CLOCK_MONOTONIC toàn hệ thống nên so được
  giữa hai tiến trình cùng máy, đó là cách đo độ trễ mà không cần đồng bộ đồng hồ.
- `run_r1_quest3_live.py --head-view-port N`: phát khung camera đầu, dùng lại
  đúng lần render đã có chứ không render thêm.
- `quest_bridge.py --head-view-port N`: nhận, và chỉ khi đó mới chuyển
  `display_mode` từ `pass-through` sang `ego`. Không có nguồn ảnh thì kính phải
  cho người vận hành thấy phòng thật.
- `launcher.py`: một cổng khai một chỗ, chảy tới cả hai đầu nên không lệch nhau.
- `tests/teleop/test_r1_head_view_stream.py`: 7 test, dựng cả hai đầu trên một
  cổng thật, không cần Isaac và không cần kính.

**Cổng ra — đạt.** Đo thật giữa hai môi trường conda (`unitree_sim_env` phát,
`tv` nhận):

```
phát 69 khung, nhận 69, rơi 0
độ trễ: nhỏ nhất 0.9 ms | trung vị 3.6 ms | lớn nhất 6.0 ms
khung ghép đôi: (480, 1280, 3) — đúng img_shape TeleVuer chờ
```

Độ trễ ~4 ms nhỏ hơn một bậc so với chu kỳ điều khiển 33 ms, nên nó không phải
nút cổ chai. Con số này đo **tới lúc bridge nhận được**; phần Vuer đẩy qua
WebSocket vào kính chưa đo được nếu không đeo kính.

**Một lỗi đã phạm.** Thống kê phía phát bị `head_camera_evidence = ...` gán đè
mất vì bộ ghi episode chạy sau. Chỉ lộ ra khi con số không xuất hiện trong
metrics; đổi thành `.update()` là xong.

**Cổng ra phiên trực tiếp: KHÔNG đạt — và bật mặc định đã làm hỏng đường live.**

Phiên thật đầu tiên (`d001_20260906T044656Z`) chạy 3 phút 31 giây và **phát ra 0
lệnh**: toàn bộ 6299 mẫu bị rơi, kèm `AssertionError` trong
`aiohttp/base_protocol.py resume_writing` trên luồng ghi SSL.

Nguyên nhân là một lỗi thiết kế tôi đưa vào. Ở chế độ `ego`, TeleVuer upsert
`ImageBackground` lên kính ở `display_fps` **ngay từ khi vào phiên**, đọc từ một
vùng nhớ chia sẻ khởi tạo bằng số không. Nhưng launcher **chờ Enter VR rồi mới
khởi động Isaac** — nên đúng trong cửa sổ chờ đó, bridge đang bơm JPEG
480×1280 rỗng lên WebSocket ở 30 fps mà không có gì để hiện. Luồng đó làm vỡ
điều tiết ghi SSL của aiohttp, phiên WebXR hỏng theo, `motion_data_ready` không
bao giờ bật, và không mẫu nào qua được.

Kênh truyền thì vẫn đúng — 69/69 khung, độ trễ trung vị 3.6 ms. Hỏng nằm ở chỗ
**bật nó trước khi có khung đầu tiên**.

Vì vậy `--head-view-port` giờ **mặc định TẮT** ở `run_d001_dataset.py`, và
`make dataset-d001-live` quay lại đúng đường T007 đã chạy được. Muốn thu dataset
thì dùng mặc định; muốn gỡ tiếp đường ảnh thì bật tay.

Cách sửa còn phải làm: hoặc khởi động Isaac trước rồi mới cho bridge vào chế độ
`ego`, hoặc hạ `display_fps` của vòng ảnh, hoặc chỉ upsert khi đã có khung thật.
Cả ba đều chưa thử.

**Mặc định `ego`, không phải `immersive`.** `ego` giữ lại hình ảnh phòng thật
xung quanh một ô nhỏ ở giữa. `immersive` che kín tầm nhìn trong khi tay người
vẫn đang vung — chọn nó là quyết định về an toàn, không phải về thẩm mỹ, nên nó
phải được gõ ra tay bằng `--head-view-mode immersive`.

## Giai đoạn 4 — D001: dataset "chỉ tay vào vật" — cơ chế XONG (2026-09-04)

**Đã làm.**
- `teleop/r1/dataset_episode.py`: cắt luồng điều khiển thành episode và ghi ra
  đúng lược đồ `EpisodeWriter` của Unitree.
- `tests/teleop/test_r1_dataset_episode.py`: 10 test, trong đó hai test **đọc mã
  nguồn của vendor** và kiểm tra lược đồ không trôi khỏi nhau.
- Ranh giới episode: cò trái (`reset_requested`) **và** mỗi lần nhả deadman.

**Định dạng: tái sử dụng, nhưng không import được.** Package `teleop` của repo
này che mất package `teleop` của vendor, nên
`from teleop.utils.episode_writer import EpisodeWriter` hỏng ở
`ModuleNotFoundError`. Bẻ `sys.path` sẽ làm hỏng chính các import của repo, nên
module sinh ra cùng lược đồ và test canh cho hai bên khỏi lệch.

**Kết quả kiểm chứng** (replay `t007_whole_upper_body_20260820T140045Z`, 130 s):

```
episode_count       : 2
episode_item_counts : [232, 378]
ảnh mỗi episode     : khớp đúng số item
states != actions   : RMS 0.067 rad (tay phải), 0.132 (tay trái)
left_ee / right_ee  : [] — R1 không có bàn tay, và [] khác 0.0
```

`tools/inspect_episode.py` viết ở Giai đoạn 1 đọc được kết quả này không cần sửa
gì. Vòng lặp khép kín.

**Phát hiện quan trọng: nhịp thật của dataset là ~9-10 Hz, không phải 30.**

`achieved_control_hz` của cả run là 27.4, nhưng đó là con số đánh lừa. Trong một
episode, các item là những bước điều khiển **liên tiếp** — `Δcontrol_step = 1` ở
mọi cặp, không có lỗ thời gian — nhưng `Δt` trung vị là 0.083 s. Con số 27.4 bị
thổi lên bởi những đoạn robot đứng yên: lúc đó không giải IK và không render ảnh
nên vòng lặp chạy rất nhanh.

Vì vậy `info.image.fps` của mỗi episode giờ là **nhịp đo được của chính episode
đó**, còn nhịp yêu cầu nằm cạnh dưới tên `fps_requested`. Ghi con số mong muốn
vào chỗ đó là nói dối về thời gian, và một chính sách học từ nó sẽ giả định các
bước cách đều nhau trong khi thực tế không phải.

**Cổng ra: CHƯA đạt.** Cơ chế chạy được, nhưng cổng đòi 20 episode hợp lệ. Hai
episode sinh từ replay là bằng chứng cơ chế đúng, không phải một dataset. Muốn
đủ 20 thì hoặc replay nhiều phiên đã ghi, hoặc thu phiên mới — và lúc đó Giai
đoạn 3 (ảnh vào kính) mới thật sự đáng làm, vì người vận hành cần nhìn thấy vật
để chỉ vào nó.

## Giai đoạn 5 — End-effector, rồi mới đến thao tác

**Cổng vào (bắt buộc).** R1 phải có bàn tay hoặc gripper trong URDF/USD. Không
có thì mọi task thao tác đều vô nghĩa và giai đoạn này đóng.

**Việc.** Chọn end-effector, chuyển sang USD bằng
`third_party/IsaacLab/scripts/tools/convert_urdf.py`, mở rộng bộ khớp điều khiển,
ánh xạ cò/nút bóp của Quest sang lệnh đóng/mở. Xem cách
`Isaac-PickPlace-RedBlock-G129-Dex3-Joint` gắn Dex3 vào G1 để làm mẫu.

**Cổng ra.** Nắm và nhấc được vật trong sim, lặp lại 10 lần liên tiếp.

---

## Giai đoạn 6 — Chuyển đổi và huấn luyện

Chuyển sang LeRobot bằng `unitree_IL_lerobot`, hoặc sang HDF5 để dùng
`isaaclab_mimic` + `robomimic` (đã cài sẵn trong `unitree_sim_env`).
`unitree_sim_isaaclab/sim_main.py` còn có `--replay --generate_data` để phát lại
episode đã ghi và sinh thêm biến thể.

Với D002 đích 30 FPS, chia giai đoạn này thành các cổng tuần tự:

1. Thu pilot 3–5 episode với `record.fps=30`; xác nhận
   `info.image.fps_requested=30`, FPS đo được từng episode nằm trong `[27,33]`,
   control đạt ít nhất 20 Hz, `source_camera_frame` tăng nghiêm ngặt và writer
   không drop frame. Launcher dùng RTX `performance` và strict gate mặc định;
   nếu 60 frame đầu không đạt nhịp thì dừng sớm thay vì tiếp tục thu dữ liệu
   không hợp lệ.
2. Thu pilot QA 10 episode; kiểm tra đúng tay/đúng số và cue không lọt vào ảnh.
   Contract head đã chốt là `arms_head`: 10 góc tay + head pitch/yaw trong cả
   state và action LeRobot.
3. Thu đủ dữ liệu qua cổng của task. Không gộp source 8 FPS rồi chỉ đổi metadata
   thành 30 FPS.
4. Chuyển sang LeRobot arms+head với `--fps 30`, validate đủ 12 chiều ở local,
   và lưu báo cáo validation.
5. Upload dataset private lên Hugging Face, tải lại đúng revision và validate
   round-trip trước khi dùng artefact từ Hub.
6. Chia train/val/test theo protocol của task, rồi mới huấn luyện và đánh giá.

Lệnh D002 cho bước 4–5 được giữ trong `D002/D002.md` và Makefile dưới target
`dataset-d002-lerobot-30fps`, `dataset-d002-upload`, và
`dataset-d002-publish-30fps`.

**Không mở giai đoạn này trước khi có ít nhất một dataset qua cổng Giai đoạn 4.**

---

## Cái gì KHÔNG nằm trong kế hoạch này

- Không có claim nào về hardware. Toàn bộ là `simulation_only`.
- Không sinh dữ liệu tổng hợp trước khi có dữ liệu người thật đạt cổng.
- Không tự định nghĩa định dạng dataset mới khi định dạng của Unitree đã đủ và
  đã có đường chuyển sang LeRobot.
