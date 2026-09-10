# T007 — Chạy nguyên xi bộ giải xr_teleoperate (2026-08-23)

## Quyết định

Sau khi so sánh A/B ba bộ giải tự viết trên trace `t007_whole_upper_body_20260823T122433Z`,
quyết định là **dùng nguyên xi `R1_A5_ArmIK` của `xr_teleoperate`**, không sửa,
để xuống phần cứng nhanh nhất rồi phát triển thêm sau. `third_party/` không bị chỉnh sửa.

## Kết quả đo

Cùng một trace, cùng một metric (residual của bộ điều khiển: target so với FK của nghiệm):

| | differential (tự viết) | **upstream** |
|---|---|---|
| trái p95 / max | 155.6 / 310.3 mm | **141.9 / 165.3 mm** |
| phải p95 / max | 221.3 / 306.7 mm | **180.6 / 196.7 mm** |
| chi phí giải | 9.49 ms trung bình | **1.25 ms** trung bình |
| trần nhịp độ | ~105 Hz | **~800 Hz** |

Upstream giảm gần một nửa sai số xấu nhất và rẻ hơn 7.6×. Trung bình thì hai bên xấp xỉ nhau.

**Giới hạn của so sánh này:** số của differential đo *bên trong* Isaac, số của upstream là
giải thuần. Hai bên chỉ so được trên metric residual chung; run differential không ghi
metric PhysX-FK nên phần trễ mô phỏng không so trực tiếp được.

### Replay Isaac (`t007_upstream_isaac_20260823`)

- 30.03 Hz, **0 hold event**, hông giữ ở stiffness 10000.
- Sai số đầu tối đa **1.4°** (0.0246 rad pitch), so với 6.8° của coupled.
- End-to-end (gồm cả retarget + PhysX): trái p95 190.5 / max 387.6 mm; phải p95 215.0 / max 437.6 mm.
- Riêng trễ mô phỏng (`command_fk_to_physx_fk`): trái p95 102.1 / max 223.6 mm.

### Đường online (`run_r1_upstream_ik_stream.py`)

Chạy trên toàn bộ trace, 1816 mẫu giải + 389 mẫu hold khi nhả deadman:

- 1.22 ms trung bình, p95 1.77 ms, max 13.9 ms → trần **821 Hz**, thừa sức trong ngân sách 33.3 ms.
- **0 vector khớp nằm ngoài giới hạn asset** — tính chất quan trọng nhất trước khi xuống phần cứng.
- Bước khớp p95 9.03° / max 16.98° ở 30 Hz, tức 8.9 rad/s, dưới trần 9.0 rad/s.
- Trái p95 159.9 / max 227.6 mm; phải p95 190.4 / max 245.4 mm (toàn trace, khó hơn đoạn đơn lẻ).

## Hai chi tiết khung toạ độ quyết định tính đúng đắn

1. **`r1_a5.urdf` của vendor cho `waist_yaw_joint` origin bằng `0 0 0`**, nên gốc mô hình của
   nó trùng `waist_yaw_link`. `R1.urdf` của dự án có `waist_roll_link` thật ở giữa và mang
   offset `[0.0325, 0, 0.049]` (58.8 mm). Lần chạy đầu tiên tôi áp offset đó lên target đưa
   cho vendor; sai số hiện ra 59-68 mm trung bình và **trông giống bám kém chứ không giống lỗi
   khung toạ độ**. Dấu hiệu nhận ra: `model_disagreement` là hằng số 58.8 mm trên cả 1181 mẫu —
   một lệch thuần tuý về khung, không thể là sai số giải. Bỏ phép đổi khung đi thì
   `model_disagreement` về đúng **0.0 mm**, đồng thời xác nhận độc lập rằng FK của dự án
   khớp với Pinocchio của vendor tới dưới milimet.
2. **Upstream điều khiển `L_ee`/`R_ee`**, khung ảo cách khớp wrist roll 0.20 m. Chuỗi tay của
   dự án đã mang đúng offset vendor đó, nên hai bên chấm điểm trên cùng một điểm vật lý.

Cả hai được ghim bằng test trong `tests/teleop/test_r1_upstream_ik_bridge.py`, chạy không cần CasADi.

## Kiến trúc

IK chạy trong env `tv` cạnh bộ đọc headset, phía robot chỉ nhận góc khớp để áp. Đây là
kiến trúc của chính upstream, và cũng là thứ hardware sidecar cần — phía robot không cần
CasADi lẫn mô hình động học. Nó cũng là bắt buộc trên thực tế: `tv` có CasADi 3.6.7 và
Pinocchio 3.1.0 kèm binding CasADi, còn `unitree_sim_env` chỉ có Pinocchio 2.7.0 không
binding. Nâng cấp một env mô phỏng đang chạy tốt để chiều bộ giải là đánh đổi tệ hơn việc
truyền góc khớp giữa hai tiến trình.

Upstream ship một package cũng tên `teleop`, đụng tên với package `teleop` của repo này.
Vì thế bộ giải luôn nằm trong tiến trình riêng, và tiến trình đó đọc wire format bằng
`json` thuần thay vì import schema của repo.

## Lệnh tái lập

```
make teleop-upstream-solve    # giải offline một trace đã ghi, sinh evidence T007
make teleop-upstream-stream   # đường online: stdin lệnh, stdout góc khớp
```

- Offline: `runs/t007_upstream_ik_seg_20260823/experiment_runner_command.txt`
- Isaac: `runs/t007_upstream_isaac_20260823/experiment_runner_command.txt`

## Chưa làm

1. **Chưa chạy live thật với Quest qua đường upstream.** Mới validate bằng replay trace đã ghi.
   Cần một phiên Quest trực tiếp trước khi coi đường online là đã chứng minh.
2. **Chưa có consumer phía Isaac cho luồng góc khớp trực tuyến.** Replay hiện tra cứu theo
   `sequence_id` từ npz dựng sẵn; bản live cần một sink đọc thẳng luồng.
3. Sai số vẫn trên ngưỡng dự án (p95 150 mm / max 250 mm) ở tay phải. Chấp nhận theo chủ
   trương "chạy được trước, hoàn thiện sau", nhưng ngưỡng chưa đạt thì chưa được coi là đạt.
4. Báo cáo này **không** cho phép actuation R1. Hardware gate chưa tick mục nào.

---

# Phần 2 — Đường end-to-end: `make teleop-arms` chạy thẳng upstream

## Quyết định

`make teleop-arms` từ nay **chạy nguyên xi bộ giải vendor, không lẫn gì của repo này**.
Đường differential cũ được giữ lại dưới tên `make teleop-arms-differential` để còn so sánh
được trên cùng một trace, chứ không bị xoá đi.

## Đường ống ba tiến trình

```
quest_bridge.py            (env tv)              đọc Quest, phát R1TeleopCommand
  → run_r1_upstream_ik_stream.py --passthrough   (env tv) giải bằng R1_A5_ArmIK
  → run_r1_quest3_live.py --upstream-joint-stream-config  (env unitree_sim_env) chỉ áp khớp
```

Trước đây launcher chỉ nối hai tiến trình. Nay `PilotLaunchSpec` có thêm `solver_args`
tuỳ chọn để chèn một tầng giải vào giữa; pilot nào tự giải bên trong simulator thì không
bị ảnh hưởng.

**Định dạng passthrough.** Tầng giải phát lại *nguyên văn* command gốc kèm thêm
`upstream_joint_position_rad` / `upstream_joint_names`. `R1TeleopCommand.from_dict` bỏ qua
khoá lạ, nên `raw_commands.jsonl` mà runner ghi ra vẫn là một command stream hợp lệ —
kỷ luật evidence không mất gì. Đây là lý do chọn passthrough thay vì phát riêng góc khớp.

## Sink mới: `teleop/r1/upstream_joint_stream.py`

Cố tình mỏng — chạy vendor nguyên xi mà đầu này lại nắn lại output thì mất hết ý nghĩa.
Nó chỉ làm ba việc không ai khác làm được:

- **Clamp theo giới hạn asset.** Vendor ràng buộc theo `r1_a5.urdf` của nó, không phải asset
  simulator nạp. Hai file không giống nhau nên phải kiểm chứ không được giả định.
- **Hold khi thiếu nghiệm** hoặc khi nhả deadman. Không có bộ giải dự phòng tại chỗ, nên
  giữ nguyên tư thế cuối là phản ứng an toàn duy nhất.
- **Từ chối vector sai tên khớp**, thay vì sắp xếp lại. Áp đúng số vào sai khớp là lỗi duy
  nhất đường này tuyệt đối không được phép tạo ra.

**Không có rate limiter.** Vendor đã có `WeightedMovingFilter`; thêm một bộ giới hạn nữa
nghĩa là đường này không còn là hành vi của vendor. Vận tốc khớp thực tế được **đo và ghi**
vào `metrics.upstream_joint_stream` để cái giá của lựa chọn đó nhìn thấy được trong evidence
chứ không bị giấu đi.

## Hai lỗi tìm ra khi dựng đường này

1. **Tra cứu kiểu tiêu thụ (`pop`) tạo hold giả.** Vòng điều khiển tái áp dụng command cũ khi
   chưa có command mới — hành vi bình thường. Nhưng sink của tôi `pop` vector ra khỏi hàng
   chờ, nên lần áp thứ hai của cùng `sequence_id` không tìm thấy gì và biến thành hold.
   Run đầu tiên có **12 hold `no_upstream_solution_for_sequence`** vì lý do này, và trong
   evidence nó trông y hệt như luồng bị mất mẫu thật. Sink offline tra cứu mà không tiêu thụ;
   tôi sửa thành `get` và thêm test hồi quy.
2. **Hằng số test chọn sai, không phải sink sai.** Test dùng 0.25 rad cho mọi khớp, nhưng
   `right_shoulder_roll_joint` chỉ tới 0.2268 rad nên clamp kích hoạt đúng. Đã đổi sang lấy
   trung điểm giới hạn từ chính asset thay vì hằng số cứng.

## `replay_commands_as_live.py`

Không thể pipe thẳng `raw_commands.jsonl` vào đường live: các dòng mang timestamp monotonic
của phiên đã ghi, nên mọi command đều cũ hàng nghìn giây và simulator hold toàn bộ — đúng
như nó phải làm. Lần chạy end-to-end đầu tiên cho `enabled_target_count: 0` và
`fresh_command_latency` trung bình **3415 s** vì lý do này. Script mới đóng vai bridge:
đóng dấu lại timestamp theo đồng hồ hiện tại và nhả theo đúng nhịp đã ghi, không đụng gì
tới pose/sequence/deadman. Nó là dụng cụ kiểm thử, và resolved config vẫn ghi rõ nguồn
command nên không thể nhầm một replay thành một phiên live.


## Kết quả chạy end-to-end (`t007_upstream_e2e_20260823`)

Chạy qua đúng ba tiến trình và đúng các pipe thật, chỉ thay headset bằng
`replay_commands_as_live.py`:

| | |
|---|---|
| nhịp điều khiển | **30.014 Hz** |
| target áp dụng | 1488 / 2207 bước |
| hold | 716 — 377 `deadman_released`, 339 `command_timeout` |
| `no_upstream_solution_for_sequence` | **0** (lỗi `pop` đã sửa) |
| dòng lệnh không hợp lệ | **0** — định dạng passthrough đúng |
| clamp theo giới hạn asset | **0** — hai asset khớp nhau trên mọi nghiệm |
| trễ lệnh | trung bình 18.2 ms, max 98.0 ms |
| sai số đầu | max pitch 7.15°, max yaw 6.00° |
| `resolved_config.t007_runtime.controller_type` | `upstream_xr_teleoperate_R1_A5_ArmIK` |

### Một phát hiện cần theo dõi trước khi xuống phần cứng

Không có rate limiter, nên vận tốc khớp áp dụng là thứ phải đo. Kết quả:

- p95 mỗi khớp nằm trong khoảng 0.16–4.28 rad/s, rất thoáng so với trần asset 18.8 rad/s (tay)
  và 33.4 rad/s (đầu).
- **Đúng 1 mẫu trên 1486 vượt trần**: `left_shoulder_pitch_joint` đạt 20.31 rad/s so với
  giới hạn 18.8 rad/s. Một xung đơn lẻ, không mang tính hệ thống.

Đây chính là cái giá cụ thể của quyết định "chạy nguyên xi, không thêm limiter". Trong mô
phỏng nó vô hại. Trên phần cứng, một xung vượt trần vận tốc ở khớp vai là thứ phải xử lý —
hoặc bằng limiter phía sidecar, hoặc bằng cách xác nhận driver của robot tự kẹp. **Chưa làm.**

## Chưa làm — chuyển tiếp sang lần sau

1. **Chưa có phiên Quest trực tiếp nào chạy qua đường upstream.** Toàn bộ xác nhận cho tới
   giờ dùng trace đã ghi phát lại qua `replay_commands_as_live.py`. Đường ống, định dạng,
   sink và evidence đã chứng minh; cái chưa chứng minh là headset thật với timing thật.
2. **Xung vận tốc vượt trần asset** (1/1486 mẫu, 20.31 vs 18.8 rad/s) chưa được xử lý.
3. Sai số vẫn trên ngưỡng dự án ở tay phải (p95 180.6 mm so với ngưỡng 150 mm; max 196.7 mm
   so với 250 mm). Chấp nhận theo chủ trương "chạy được trước", nhưng chưa đạt thì chưa được
   ghi là đạt.
4. Chưa đo latency end-to-end tách bạch cho đường ba tiến trình: thêm một pipe nghĩa là thêm
   một chặng, và chưa có số cho chặng đó.
5. `make teleop` (coupled, waist_yaw) và `make teleop-arms-differential` vẫn dùng bộ giải của
   repo. Chỉ `teleop-arms` được chuyển sang vendor.
6. **Khoá API OpenAI trong `Operation_Khanh/sk.txt` đã bị đẩy lên GitHub** ở commit `d0a22a4`.
   Cần **xoay khoá**, xoá file là không đủ. Chưa xử lý.
7. Xung đột nhánh develop với `d0a22a4` của đồng đội chưa giải quyết; chưa force-push.
8. Hardware gate chưa tick mục nào. Báo cáo này **không** cho phép actuation R1.

---

# Phần 3 — Chuẩn hoá upstream và contract độc lập đầu/tay (2026-09-09)

## Yêu cầu và hành vi thực thi

Theo quyết định của researcher, vendor `R1_A5_ArmIK` cùng initial-head anchor là
đường live chuẩn cho cả simulator và hardware. `make teleop` và launcher T007
chọn upstream khi không có flag; `--upstream-solver` vẫn được nhận để lệnh cũ
không hỏng. Bộ giải của repo chỉ còn là đối chứng qua `--coupled-solver`.

Contract hành vi của chuẩn này là **đầu và hai tay độc lập**: chuyển động chỉ của
headset thay đổi head pitch/yaw nhưng không thay đổi wrist target đưa vào IK;
chuyển động controller vẫn thay đổi wrist target. Producer tháo wrist khỏi
current-head position/yaw của TeleVuer rồi biểu diễn lại trong position/yaw của
head anchor chốt ở mẫu deadman thứ ba. Pitch/roll không nằm trong arm reference
frame của vendor; chúng chỉ đi vào nhánh head tương đối.

```text
Quest head + wrists
→ TeleVuer head-yaw-relative wrists
→ initial-head re-anchor
→ unmodified vendor R1_A5_ArmIK
→ absolute joint targets in Isaac / bounded hardware transport
```

## Lựa chọn và compatibility

| Lựa chọn | Trạng thái | Nguồn |
|---|---|---|
| upstream là default | specified | quyết định researcher 2026-09-09 |
| giữ `--upstream-solver` | inherited compatibility | lệnh/runbook cũ |
| coupled chỉ qua `--coupled-solver` | AI-selected interface | giữ khả năng A/B mà không làm mơ hồ default |
| ba mẫu deadman để chốt anchor | inherited | implementation đã accepted ở commit `7fe635a` |

Observation, data schema và solver numerics không đổi. Baseline selection đổi;
run coupled cũ vẫn là evidence lịch sử của coupled, không phải replicate của
default mới. Mọi run upstream trước initial-head anchor, gồm
`t007_whole_upper_body_20260908T073844Z`, **requires reproduction** trước khi
dùng làm evidence cho tính độc lập đầu/tay. Thay đổi default này mới ở mức code
verified cho tới khi có một live sim run hậu thay đổi.

## Review surface

- `scripts/teleop/run_t007_upper_body_pilot.py`: default/opt-in solver selection.
- `scripts/teleop/run_r1_upstream_ik_stream.py::reanchor_wrist_matrix`: contract độc lập.
- `tests/teleop/test_r1_upstream_ik_bridge.py`: bất biến wrist dưới head-only motion.
- `tests/teleop/test_r1_upstream_pilot_wiring.py`: pipeline mặc định và coupled opt-in.

---

# Phần 4 — Chuyển cùng target upstream sang hardware (2026-09-09)

## Reference run và giới hạn claim

Researcher chọn video của
`runs/t007_whole_upper_body_20260909T072030Z` làm reference để chuyển sang
hardware. Run nhận 2879 command, có 1678 target enabled, đạt 27.030 Hz và
`sim_to_wall_ratio=0.999978`; simulator không clamp joint limit. Vận tốc q
upstream có p95 4.008 rad/s và max 12.149 rad/s. Đây là run được đánh giá tốt
bằng hình ảnh; `status.json` vẫn ghi `scientific_outcome: unassessed`, nên phần
này không nâng nó thành hardware evidence hay scientific reproduction.

Q vendor đầu tiên có `max(|q|)=0.2703 rad`. Trong cả run, delta lớn nhất so với
q đầu ở một khớp không phải vai là 1.7755 rad, lớn hơn envelope hardware mặc
định 1.0 rad. Head pitch/yaw cũng vượt gate hardware. Do đó transfer dưới đây
loại affine posture offset; nó không hứa toàn bộ chuyển động hay timing của
video sẽ được tái tạo nguyên vẹn.

## Semantic mismatch đã sửa

Trước thay đổi:

```text
sim:      q_sim = q_vendor
hardware: q_hw  = q_nominal + (q_source - q_source_at_end_of_home)
```

Homing nominal rồi re-anchor source mới nhất làm posture hardware khác sim dù
cùng solver. `q_source` là output vendor sau limiter hardware. Upstream hardware
giờ dùng source alignment:

```text
q_source_initial = q_vendor_initial  # limiter khởi tạo từ mẫu đầu
goal = q_source_initial
ramp robot chậm tới goal và xác nhận encoder
start_q = source_zero = q_source_initial
q_sidecar = start_q + clamp(q_source - source_zero, envelope)
```

Vì vậy, khi chưa clamp, `q_sidecar = q_source`: sidecar không còn cộng offset
làm sai posture. Đây không phải temporal parity với Isaac vì producer vẫn giới
hạn 1.0 rad/s và 2.0 rad/s², còn sidecar vẫn có head/session clamp. Sole-owner
UTL1, target mode tương đối, watchdog, limiter, head gate và envelope không đổi.

## Implementation disclosure

| Lựa chọn | Phân loại | Hành vi |
|---|---|---|
| upstream hardware source-align theo q vendor đầu tiên | specified | chuyển reference posture sim được researcher chọn sang hardware |
| coupled legacy vẫn home nominal | inherited | không đổi semantics của đối chứng |
| dùng per-joint session envelope làm zero-centred acceptance bound cho initial q | AI-selected | target đầu vượt bound bị từ chối, không clamp im lặng |
| command cách goal ≤ 0.02 rad; encoder residual chỉ ghi nhận | inherited completion, measured observability | tránh khóa vĩnh viễn do sai số bám tĩnh trên hardware |
| giữ source goal đóng băng trong alignment | AI-selected | robot không đuổi theo controller đang di chuyển trong pha tự chạy |
| ghi envelope/head clamp count, joint và max offset | AI-selected observability | cho biết khác biệt nào được thêm ở sidecar |

Đổi actuation semantics ở bước khởi tạo và vì thế evidence hardware cũ trước
source alignment không comparable về absolute posture. Observation contract,
vendor IK numerics, initial-head wrist re-anchor, data schema version, sole
lowcmd ownership và success metric không đổi.

## Review và validation surface

- `scripts/teleop/run_r1_quest3_hardware.sh`: upstream chọn
  `--home-to-source`, coupled chọn `--home-to-nominal`.
- `scripts/teleop/run_r1_quest3_hardware_targets.py`: khai báo tường minh
  `target_mode: relative_source`.
- `hardware/teleop/src/teleop/hardware/high_level_sidecar.py`:
  `validate_source_home_goal`, `relative_session_target`, `constrain_head_target`,
  command-complete home và encoder residual.
- `hardware/teleop/tests/test_high_level_sidecar.py`: identity q và clamp/bound.
- `tests/teleop/test_r1_hardware_upstream_targets.py`: contract producer/sidecar.

Trạng thái hiện tại: **code verified và deploy-only verified**, chưa chạy motor
trong thay đổi này. Package đã copy tới robot `10.42.0.33`; SHA-256 sidecar
local/remote cùng là
`54d658fcd4eb7c2588dd6c63810f44e611a2a13ecd7bb87fdd7e1396c371cf54`, service
teleop vẫn inactive và không có Python sidecar sau deploy. Vì worktree dirty,
`SOURCE.txt` ghi commit nền `7fe635a` cùng trạng thái `dirty`; diff hiện tại là
phần provenance bắt buộc để tái tạo.

Một suspended bounded run mới là bước phân biệt còn thiếu; phải đọc metadata
clamp cùng `target_q`/`observed_q` trước khi tuyên bố hardware tracking; temporal
parity với sim không được kỳ vọng khi giữ hardware limiter hiện tại.
