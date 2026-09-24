# V1 — `flat_plus_h4_v1`: observation history H=4

> Nguồn observation duy nhất: canonical Flat cơ bản 83D; 332D chỉ là bốn frame
> 83D đóng gói term-major, không phải một base observation mới.

| | |
|---|---|
| **Contract** | `flat_plus_h4_v1` |
| **Task** | `Unitree-R1-Flat-Plus` |
| **Actor / critic / action** | 332 / 98 / 24 |
| **Cha** | `Unitree-R1-Flat-Meta` (common-PD, 83D) |
| **Trạng thái** | **ĐÃ TRIỂN KHAI LÕI** — chưa release-ready, còn hardening + train |

## Đã xong (2026-09-05..06)

| Gate | | Bằng chứng |
|---|---|---|
| 0 baseline + dọn test | ✅ | sim 67/67 · HB 14/14 · FlatPlus python 26/26 |
| 1 task tách biệt | ✅ | `tests/test_flat_plus_contract.py` |
| 2 semantics history | ✅ | đo trên tensor thật, không suy từ source |
| 3a converter warm-start | ✅ | `tests/test_flat_plus_warm_start.py` |
| 3b train smoke + save/reload/export | ❌ | chưa có trained/exported H4 artifact |
| 6 packing parity 3 runtime | ✅ | `golden_trace.txt` + negative control |
| 4 train · 5 export · 7 sim · 8 robot | ❌ | cần máy train và các gate sau đó |

Code: `src/tasks/velocity/config/r1/history_contract.py`,
`env_cfgs.py::unitree_r1_flat_plus_env_cfg`, `rl/flat_plus_runner.py`,
`scripts/init_flat_plus_checkpoint.py`, `scripts/check_flat_plus.py`,
`scripts/make_flat_plus_golden_trace.py`, `scripts/collect_flat_plus_bundle.py`.
Runtime: `HB/high_level_2/src/policy/{BaseObservation83,BaseObservationHistory,FlatPlusController}.hpp`,
`unitree_mujoco/simulate_cpp/src/controllers/locomotion/{BaseObservationHistory,FlatPlusController}.*`.

Bàn giao máy train: `TRAIN_MACHINE_HANDOFF.md`.

## Còn phải làm sau audit 2026-09-06

- Bổ sung validation metadata H4 đầy đủ ở cả HB và simulator: stride,
  `includes_current`, append stage, previous-action semantics, policy dt/rate,
  gait period, term dimensions và layout/schema chính xác.
- Khóa tensor contract ONNX theo rank/batch/dtype/output count và chặn NaN/Inf
  trước khi action đi tới target joint/LowCmd.
- Đổi các `assert` có thể biến mất dưới `NDEBUG` trong history test thành
  `Require/Check` luôn chạy ở Release.
- Mở rộng golden regression từ raw state/command/previous action qua chính
  builder Flat 83D; test hiện tại mới khóa phần history packing.
- Train H1/H4/H5, export `policy_flat_plus_h4_v1.onnx`, chạy PyTorch↔ONNX↔C++
  parity, closed-loop simulator và cuối cùng hardware acceptance.

`scripts/check_flat_plus.py --strict` hiện báo **0 failed, 1 skipped** cho cả
H4 và H5: phần bị skip duy nhất là ONNX đã train/export chưa tồn tại. Vì strict
mode không chấp nhận skip nên release gate vẫn FAIL đúng thiết kế. Test suite:
**41 passed, 1 skipped**.

## Các correction chính so với bản kế hoạch gốc bên dưới

1. **§5.3 `atol <= 1e-6`** — không đạt được trong float32. Lớp đầu mới cộng qua
   332 số hạng thay vì 83; 249 số hạng thêm vào nhân với 0 nên đóng góp bằng 0,
   nhưng chúng đổi cách chia khối phép tổng, mà cộng dấu phẩy động không kết
   hợp. Đo được: float64 lệch 1.4e-14, float32 lệch tối đa 4.8e-6 tuyệt đối.
   Test tách làm hai: float64 khẳng định tính đúng (`max_abs ≤ 1e-10`), float32
   chỉ canh rò rỉ từ slot cũ với `|Δ| ≤ atol + rtol·|want|`, `atol=1e-4`,
   `rtol=1e-3`.

   **Sửa tiếp 2026-09-06:** bản trước dùng max-relative trần `≤ 1e-3` và nó
   fail trên torch 2.7.0/CPU với `1.57e-3`. Không phải mapping sai: relative bị
   chi phối bởi component action gần 0 — `want=3.0e-4` lệch `4.7e-7` đọc thành
   1.6e-3 "relative". Ngưỡng tuyệt đối giữ nguyên sức phát hiện vì rò rỉ thật
   từ slot cũ là O(1).
2. **`FlatPlusPolicyProfile.hpp` không được tạo** — hằng số contract nằm chung
   với packer trong `BaseObservationHistory.hpp` để không có hai nguồn sự thật.
3. **Gate 3 được tách thành 3a/3b** — converter warm-start đã pass, còn train
   smoke/save-reload/export chưa có artifact nên không được ghi là Gate 3 đầy đủ.
4. **§5.2 from-scratch nay là mặc định của code** (sửa 2026-09-06). Trước đó
   `scripts/train.py` vẫn default warm-start cho `Unitree-R1-Flat-Meta`, nên
   lệnh trần của arm H1 âm thầm kế thừa `model_flat_common_pd.pt` trong khi
   H4/H5 chạy from-scratch — đúng cái §5.2 cấm. Default đã bị bỏ, và
   `test_ablation_arms_train_from_scratch` gọi `TrainConfig.from_task` để khoá.
   Test cũ `test_ablation_arms_share_one_parent` chỉ so hai hằng số đường dẫn
   nên vẫn pass suốt thời gian runtime vi phạm; nó đã bị thay.

## Quy ước đọc phần kế hoạch gốc

Phần bên dưới được giữ nguyên để đối chiếu lịch sử, không phải trạng thái hiện
tại. Vì vậy các dòng `PLAN ONLY`, `atol <= 1e-6` và bảng file có
`FlatPlusPolicyProfile.hpp` là yêu cầu cũ đã bị supersede. Source hiện tại dùng
`BaseObservationHistory.hpp` cho contract constants; tiêu chí tolerance hiện tại
là float64 mapping chính xác và float32 `max_rel <= 1e-3`.

---

## Kế hoạch gốc (2026-09-05) — nguyên văn

> Ngày lập kế hoạch: 2026-09-05  
> Trạng thái: **PLAN ONLY — chưa triển khai mã nguồn, chưa train policy mới**  
> Checkout training mục tiêu: `unitree_rl_mjlab_meta`  
> Baseline phải bảo toàn: `Unitree-R1-Flat-Meta` / common-PD / actor 83D

> **Khóa phạm vi:** toàn bộ task mới trong tài liệu này chỉ lấy observation từ
> **Flat cơ bản 83D** nêu ở mục 3.1. H4 332D là bốn frame 83D được đóng gói, còn
> các cột gait/latent ở các phase sau chỉ là conditioning bổ sung trên cùng nền
> Flat 83D. Không dùng bất kỳ policy locomotion, camera/height scan hay
> observation của meta-policy nào khác làm base frame.
> Các route ngoài Flat chỉ được giữ nguyên để không gây regression, không thuộc
> phạm vi train hay nghiệm thu của `flat_plus`.

## 1. Mục tiêu

Thêm một cửa sổ lịch sử ngắn vào observation của **actor locomotion Flat**, để policy có thể suy ra xu hướng động học từ nhiều policy step liên tiếp mà vẫn dùng toàn bộ tín hiệu có thể tái tạo trên robot thật. Sau khi có baseline history, tạo thêm một task gait-conditioned độc lập lấy các ý tưởng cốt lõi từ bài báo G1: một policy chung, gait ID tường minh, reward routing và curriculum chuyển gait.

Kết quả cần đạt:

- Tạo một task train mới, opt-in, kế thừa đúng `Unitree-R1-Flat-Meta`.
- Actor mới nhận history đã flatten; critic tiếp tục nhận observation hiện tại.
- Với `flat_plus`, giữ nguyên action 24D, joint order, default joint position, action scale, stiffness, damping, reward, command distribution và domain randomization của common-PD baseline. `flat_plus_gait` chỉ thay reward/command bằng factory riêng được mô tả ở mục 13.
- Export ONNX có metadata đủ để C++ từ chối model sai contract.
- Cả HB runtime và MuJoCo C++ đều tạo history giống hệt training.
- Không thay thế `policy_goc_2.onnx`, không đổi cấu hình locomotion đang chạy, không làm đổi đường chạy legacy.
- Tạo task opt-in `flat_plus_gait` cho `STAND`, `WALK`, `W2S`; không biến Stand/Walk thành các expert hoặc policy riêng.
- Không đưa `RUN/R2W`, LSTM hoặc A-RMA vào artifact đầu tiên của `flat_plus_gait`; các phần này phải là ablation/release riêng.

## 2. Kết luận thiết kế đề xuất

### 2.1. Candidate đầu tiên

Đề xuất triển khai candidate đầu tiên với:

```text
Tên gọi ngắn:     flat_plus
Task ID:          Unitree-R1-Flat-Plus
Experiment name:  r1_flat_plus
Policy contract:  flat_plus_h4_v1
Base frame:       83 float32
History length:   H = 4
Actor input:      4 x 83 = 332 float32
Critic input:     98 float32, không stack
Action output:    24 float32
Policy rate:      50 Hz
History order:    term-major, oldest -> newest trong từng term
Reset padding:    lặp lại frame hợp lệ đầu tiên, không zero-pad
```

Quy ước gọi hằng ngày chỉ dùng **`flat_plus`**. `h4_v1` chỉ xuất hiện trong contract/model/manifest vì đó là thông tin bắt buộc để phân biệt dimension và version; không kéo chuỗi kỹ thuật dài vào Task ID.

H=4 là **điểm khởi đầu kỹ thuật**, không phải kết luận rằng H=4 tối ưu. Ở 50 Hz, bốn mẫu tương ứng `t-60 ms`, `t-40 ms`, `t-20 ms`, `t`; span từ mẫu cũ nhất tới hiện tại là 60 ms.

Trước khi chọn model cuối cùng, phải ablation ít nhất H=1, H=4 và H=5 với cùng seed/config. Nếu H=5 thắng rõ và đáp ứng budget runtime, nó phải dùng task/contract/artifact mới có `h5` trong tên; không được âm thầm thay dimension của contract H4.

### 2.2. Vì sao chọn MLP frame stack cho v1

- `mjlab==1.2.0` đã có native observation history; không cần tự viết buffer trong môi trường train.
- ONNX vẫn giữ một input tensor phẳng `[1, N]`, phù hợp với các runner C++ hiện tại.
- Không phát sinh hidden state input/output, reset mask và recurrent rollout storage như GRU/LSTM.
- Dễ viết golden test Python/C++ và dễ rollback về policy 83D.

GRU/RMA recurrent locomotion nên là một contract v2 riêng. Các buffer 50x83 đang dùng cho meta selector/RMA không được tái sử dụng vì chúng phục vụ policy chọn expert và có startup semantics khác.

### 2.3. Task gait-conditioned kế tiếp

Sau khi `flat_plus` vượt các gate contract và closed-loop simulator, tạo task mới:

```text
Tên gọi ngắn:     flat_plus_gait
Task ID:          Unitree-R1-Flat-Plus-Gait
Experiment name:  r1_flat_plus_gait
Policy contract:  flat_plus_gait_h4_v1
Parent task:      Unitree-R1-Flat-Plus
Gait modes:       STAND, WALK, W2S
Actor input:      H4 332D + current gait one-hot 3D = 335D
Critic input:     current critic 98D + current gait one-hot 3D = 101D
Action output:    24D
Actor topology:   MLP (512, 256, 128), ELU
```

Đây vẫn là **một continuous locomotion policy duy nhất**, không phải ba policy và không phải selector chọn expert. `gait_id` mô tả hành vi mong muốn; actor dùng cùng một bộ trọng số để tạo action cho cả ba mode. Reward routing chỉ tồn tại khi train; runtime chỉ cần gait FSM, observation packing và actor.

`flat_plus_gait` dùng H4-MLP làm candidate đầu tiên để có thể đo riêng đóng góp của gait conditioning so với `flat_plus`. Một nhánh LSTM bám sát bài báo phải được chạy như ablation riêng, không được thay kiến trúc mặc định rồi quy toàn bộ improvement cho gait ID/reward routing.

## 3. Baseline đã xác minh

### 3.1. Training observation hiện tại

Nguồn: `src/tasks/velocity/velocity_env_cfg.py`.

| Term actor | Dimension | Ý nghĩa |
|---|---:|---|
| `base_ang_vel` | 3 | Gyroscope trong body frame |
| `projected_gravity` | 3 | Vector gravity chiếu vào body frame |
| `command` | 3 | `vx`, `vy`, `yaw_rate` |
| `phase` | 2 | `sin(phase)`, `cos(phase)` với period 0.6 s |
| `joint_pos` | 24 | Joint position tương đối default pose |
| `joint_vel` | 24 | Joint velocity |
| `actions` | 24 | Raw action của policy step trước |
| **Tổng** | **83** | Actor observation hiện tại |

Actor và critic hiện đều khai báo `history_length=1`. Trên Flat, critic có thêm privileged terms và có tổng dimension 98.

Simulation timestep là 0.005 s và decimation là 4, do đó policy step là:

```text
policy_dt = 0.005 x 4 = 0.02 s = 50 Hz
```

### 3.2. Task phải được kế thừa

Task history phải gọi:

```python
cfg = unitree_r1_flat_meta_env_cfg(play=play)
```

Không được kế thừa trực tiếp `unitree_r1_flat_env_cfg()`. `Flat-Meta` là task được tách riêng để khóa common-PD contract và command/friction distribution đang dùng cùng hệ slope expert.

### 3.3. Artifact baseline đang deploy

| Artifact | Contract đã xác minh |
|---|---|
| `checkpoints/r1_flat/model_flat_common_pd.pt` | actor 83D, critic 98D, action 24D |
| `checkpoints/r1_flat/policy_flat_common_pd.onnx` | `[1,83] -> [1,24]` |
| `HB/high_level_2/policies/flat/policy_goc_2.onnx` | cùng binary với ONNX common-PD |

SHA256 ONNX baseline:

```text
ee1cd73469ba0f16eff7542fc4758396a7301398ebdfb2e0926713b3c1954500
```

SHA256 checkpoint baseline:

```text
b6090c47723a625859fb20f4d288b0149712ef537bb8c83025136bad3f40c1f7
```

`HB/high_level_2/config/locomotion.yaml` hiện đang chọn:

```yaml
flat_policy_contract: legacy_83
flat_model: policy_goc_2.onnx
```

Hai dòng này phải giữ nguyên trong toàn bộ giai đoạn phát triển và nghiệm thu simulator.

## 4. Contract observation H4 chính xác

Native history của mJLab lưu history riêng cho từng term, rồi concatenate các term. Vì vậy layout là **term-major**, không phải time-major.

### 4.1. Layout `[1,332]`

| Offset half-open | Số phần tử | Nội dung |
|---|---:|---|
| `[0, 12)` | 4 x 3 | `base_ang_vel[t-3:t+1]` |
| `[12, 24)` | 4 x 3 | `projected_gravity[t-3:t+1]` |
| `[24, 36)` | 4 x 3 | `command[t-3:t+1]` |
| `[36, 44)` | 4 x 2 | `phase[t-3:t+1]` |
| `[44, 140)` | 4 x 24 | `joint_pos[t-3:t+1]` |
| `[140, 236)` | 4 x 24 | `joint_vel[t-3:t+1]` |
| `[236, 332)` | 4 x 24 | `actions[t-3:t+1]` |

Trong mỗi block term, dữ liệu đi từ **oldest tới newest**. Ví dụ newest/current slices là:

| Term | Slice current `t` trong H4 |
|---|---|
| `base_ang_vel` | `[9, 12)` |
| `projected_gravity` | `[21, 24)` |
| `command` | `[33, 36)` |
| `phase` | `[42, 44)` |
| `joint_pos` | `[116, 140)` |
| `joint_vel` | `[212, 236)` |
| `actions` | `[308, 332)` |

Không được đóng gói theo dạng:

```text
[full_frame(t-3), full_frame(t-2), full_frame(t-1), full_frame(t)]
```

Hai layout đều có 332 phần tử nhưng cho action hoàn toàn khác; kiểm tra dimension đơn thuần không phát hiện được lỗi này.

### 4.2. Trình tự update trong một policy step

Thứ tự bắt buộc ở training, Python replay và C++:

1. Đọc sensor/state hiện tại.
2. Tạo base frame 83D; trường `actions` phải là raw action của policy step trước.
3. Append từng term của frame hiện tại vào history đúng một lần.
4. Lấy buffer theo thứ tự oldest -> newest và flatten term-major thành 332D.
5. Chạy actor ONNX/PyTorch.
6. Chuyển action 24D thành target joint theo common-PD contract hiện tại.
7. Sau inference mới ghi action vừa sinh thành `last_action` cho policy step tiếp theo.

History phải update ở **50 Hz policy loop**, không update ở 500 Hz hardware control loop. Nếu append 10 lần cho mỗi action, history sẽ chứa các frame gần như lặp lại và không còn tương đương training.

### 4.3. Reset và activation

Khi reset episode hoặc chuyển vào policy history:

1. Clear valid-length/history state.
2. Reset `last_action` về zero như controller Flat hiện tại.
3. Tạo frame 83D hợp lệ đầu tiên từ state hiện tại.
4. Backfill cả bốn slot bằng frame đầu tiên đó.
5. Chạy inference đầu tiên trên history đã backfill.

Không zero-pad các slot cũ. Không giữ lại frame từ lần chạy policy trước. Khi re-activate policy sau Stand/Dance/Fall recovery, buffer phải được reset lại.

Gait clock hiện tại của HB không tự reset mỗi lần activate policy. V1 nên giữ hành vi phase hiện có để không tạo thêm một thay đổi ngoài history; history sẽ backfill bằng phase hiện tại tại thời điểm activate. Nếu sau này muốn reset phase, đó phải là một thí nghiệm/contract khác.

### 4.4. Tín hiệu bị cấm trong actor

Không thêm các tín hiệu chỉ có trong simulator:

- camera/depth;
- height scan/raycast;
- slope angle ground truth;
- contact state hoặc contact force lý tưởng;
- base linear velocity ground truth;
- terrain class/oracle expert label.

Critic vẫn có thể dùng privileged 98D hiện tại trong training; actor deploy chỉ dùng các tín hiệu tái tạo được trên robot.

## 5. Ma trận file cần chỉnh sửa

### 5.1. Training repo: `unitree_rl_mjlab_meta`

| File | Thay đổi dự kiến | Cách cô lập |
|---|---|---|
| `src/tasks/velocity/config/r1/history_contract.py` | File mới tách `BaseObservationSchema83` khỏi `ActorHistoryViewH4`; chứa contract ID, term order/dim, expected actor/critic/action dim và metadata builder | Giữ frame 83D nguyên tử để A-RMA dùng lại mà không coi 332D là observation gốc |
| `src/tasks/velocity/config/r1/env_cfgs.py` | Thêm `unitree_r1_flat_plus_env_cfg()`; gọi Flat-Meta rồi chỉ override actor history | Không sửa factory Flat/Flat-Meta hiện có |
| `src/tasks/velocity/config/r1/rl_cfg.py` | Thêm `unitree_r1_flat_plus_ppo_runner_cfg()` với `experiment_name="r1_flat_plus"` | Giữ nguyên PPO/MLP baseline |
| `src/tasks/velocity/config/r1/__init__.py` | Register `Unitree-R1-Flat-Plus` | Không đổi registration cũ |
| `src/tasks/velocity/rl/flat_plus_runner.py` | Runner mới kế thừa `VelocityOnPolicyRunner`, gắn/validate history metadata khi export | `VelocityOnPolicyRunner` legacy giữ nguyên |
| `src/tasks/velocity/rl/__init__.py` | Export runner mới | Chỉ `Flat-Plus` dùng runner này |
| `scripts/init_flat_plus_checkpoint.py` | Converter actor 83D -> actor H4 332D có parity tại thời điểm khởi tạo | Output vào thư mục mới, không sửa checkpoint gốc |
| `scripts/check_flat_plus.py` | Check task, checkpoint, ONNX, metadata, tensor shape, finite inference và parity | Không sửa checker common-PD cũ |
| `tests/test_flat_plus_contract.py` | Unit tests layout/reset/metadata/migration | Không thay expectation legacy |
| `checkpoints/r1_flat_plus/h4_v1/README.md` | Provenance, hashes, seed, command, metric và acceptance state | Artifact mới có namespace riêng |

Override môi trường dự kiến:

```python
def unitree_r1_flat_plus_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  cfg = unitree_r1_flat_meta_env_cfg(play=play)
  cfg.observations["actor"].history_length = 4
  cfg.observations["actor"].flatten_history_dim = True
  cfg.observations["critic"].history_length = 1
  return cfg
```

Không đổi `history_length` trong `velocity_env_cfg.py`, vì file đó là shared base của Rough, Flat, Slope và các task khác.

`self_collision_cfg.history_length=4` nếu gặp trong R1 config là history của contact sensor/substep phục vụ self-collision, không phải policy observation history.

### 5.2. `scripts/train.py`: thay đổi có điều kiện

Không được trỏ `Flat-Plus` trực tiếp tới checkpoint actor 83D; strict load sẽ lỗi shape `[512,83]` so với `[512,332]`. `strict=False` cũng không giải quyết tensor cùng tên nhưng khác shape.

Hai đường hợp lệ:

- **From scratch — đường chính đã chốt:** không truyền `--warm-start-checkpoint`
  cho H1, H4 hoặc H5. Cả ba arm phải dùng cùng initialization protocol, seed,
  budget và PPO để kết quả chỉ đo ảnh hưởng của cửa sổ history.
- **Mapped warm start — đường phụ:** chạy converter tạo checkpoint 332D tương
  thích, rồi truyền rõ `--warm-start-checkpoint <converted.pt>`. Đây chỉ là
  smoke/diagnostic hoặc ablation riêng; không trộn vào kết quả chính và không
  dùng để thay thế arm from-scratch.

Khuyến nghị smoke `H4-scratch` trước; `H4-mapped` chỉ chạy thêm nếu cần chẩn
đoán warm-start. Không dùng checkpoint converted để chọn kết quả chính.

### 5.3. Warm-start mapping 83D -> 332D

Nếu dùng mapped warm start:

1. Khởi tạo first actor layer mới `[512,332]` bằng zero.
2. Copy weight của mỗi term cũ vào **newest slice** tương ứng.
3. Để weight của ba slot history cũ hơn bằng zero.
4. Copy first-layer bias, tất cả layer sau và action distribution std.
5. Duplicate normalizer statistics của từng base term vào bốn history slot tương ứng; copy sample count.
6. Không load optimizer/iteration.
7. Critic 98D có thể khởi tạo mới; mặc định không warm-start critic để tách rủi ro.

Mapping first-layer cụ thể:

| Weight cũ 83D | Weight mới H4 current slice |
|---|---|
| `[0,3)` | `[9,12)` |
| `[3,6)` | `[21,24)` |
| `[6,9)` | `[33,36)` |
| `[9,11)` | `[42,44)` |
| `[11,35)` | `[116,140)` |
| `[35,59)` | `[212,236)` |
| `[59,83)` | `[308,332)` |

Acceptance của converter: với bất kỳ current base frame hợp lệ nào và arbitrary older frames, actor H4 vừa convert phải cho output bằng actor 83D baseline trong tolerance `atol <= 1e-6`, vì toàn bộ weight history cũ hơn đang bằng zero.

### 5.4. ONNX metadata bắt buộc

Exporter chuẩn của mJLab hiện ghi tên observation nhưng chưa mô tả history layout đầy đủ. Runner H4 phải bổ sung tối thiểu:

```text
policy_contract=flat_plus_h4_v1
observation_schema=flat_plus_h4_v1
observation_dim=332
base_observation_schema=r1_common_pd_base83_v1
base_observation_dim=83
actor_input_schema=flat_plus_h4_v1
actor_input_dim=332
action_dim=24
actor_history_steps=4
actor_history_stride=1
actor_history_order=term_major_oldest_to_newest
actor_history_padding=repeat_first
actor_history_includes_current=true
actor_history_append_stage=pre_inference
last_action_semantics=previous_raw_policy_action
policy_dt_s=0.02
policy_hz=50.0
gait_period_s=0.6
observation_terms=base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions
observation_term_dims=3,3,3,2,24,24,24
```

Nên thêm một `observation_layout_json` canonical chứa offset của từng term/current slice. C++ phải validate metadata và tensor shape; không suy đoán contract chỉ từ `input_dim == 332`.

Metadata common-PD hiện có về joint names, default joint positions, action scale, stiffness và damping phải tiếp tục được ghi và đối chiếu.

### 5.5. HB runtime: `HB/high_level_2`

| File/khu vực | Thay đổi dự kiến |
|---|---|
| `src/policy/BaseObservationHistory.hpp` | Primitive mới lưu full frame 83D theo policy step, track `valid_steps`, tạo view theo layout/padding đã khai báo |
| `src/policy/FlatPlusController.hpp` | Controller mới; dùng pure builder 83D và lấy H4 term-major view từ history bank |
| `src/policy/FlatPlusPolicyProfile.hpp` | Constants/profile mới chứa contract ID, input 332, output 24, gait 0.6 và capability flags |
| `src/policy/FlatPolicyProfile.hpp` | Thêm enum/parser cho exact history contract vào parser đang được `Application` dùng |
| `src/config/Tuning.cpp` | Thêm history contract vào whitelist và validate cặp contract/model; legacy values giữ nguyên |
| `src/policy/PolicyController.hpp` | Validate đầy đủ history metadata tại nơi contract metadata hiện đang được kiểm tra; exact input/output rank, dimension, dtype |
| `src/policy/OnnxPolicy.cpp` | Giữ kiểm tra tensor-level, reject shape/dtype sai và NaN/Inf cho input/output |
| `src/app/Application.cpp` | Chọn controller theo exact contract; gọi Reset trên mọi activation path; append ở policy loop 50 Hz; dùng capability overlay rõ ràng |
| `config/locomotion_flat_plus.example.yaml` | File opt-in mẫu; không đổi `config/locomotion.yaml` đang chạy |
| `policies/flat/policy_flat_plus_h4_v1.onnx` | Artifact mới, filename riêng; không overwrite `policy_goc_2.onnx` |
| `tests/flat_policy_profile_test.cpp` và config tests | Test parser/whitelist/profile mới và regression legacy |
| `HB/r1_integration/config/model_manifest.conf` | Manifest schema mới phải chứa hash/dim/history/contract |
| `HB/r1_integration/scripts/update_model_manifest.sh` | Bỏ hard-code input 83 cho route history, nhưng giữ default legacy |
| `HB/r1_integration/scripts/preflight.sh` | Probe exact shape, metadata và sequence inference trước deploy |
| `HB/r1_integration/scripts/deploy_stack.sh` | Cho phép filename/manifest history theo đường opt-in, không đổi active default |

Chuỗi xử lý 83D hiện tại nằm trong `LocomotionController.hpp`; nên tách thành pure `BuildBaseObservation83()` có golden regression, tránh viết lại công thức gravity/joint order/action offset lần thứ hai. History bank phải lưu full frame 83D và tách logic storage khỏi H4 packer; không hard-code storage thành vector 332D.

Contract cấu hình opt-in sau khi đã nghiệm thu simulator:

```yaml
flat_policy_contract: flat_plus_h4_v1
flat_model: policy_flat_plus_h4_v1.onnx
```

V1 phải khai rõ `SupportsArmOverlay() == false`, và `Application` phải dùng capability đó cho cả observation masking lẫn motor-target overlay. Chỉ `legacy_83` được opt-in overlay theo offset 83D. Không chèn arm gesture/command trực tiếp vào vector 332D bằng các offset 83D cũ. Nếu muốn policy history hỗ trợ overlay, phải train/test contract riêng.

### 5.6. MuJoCo C++ simulator: `unitree_mujoco/simulate_cpp`

| File/khu vực | Thay đổi dự kiến |
|---|---|
| `src/controllers/locomotion/BaseObservationHistory.{hpp,cpp}` | History bank full-frame 83D, track valid steps và hỗ trợ view-specific layout/padding |
| `src/controllers/locomotion/FlatPlusController.{hpp,cpp}` | Controller H4 mới, reuse pure base-frame builder và H4 term-major view |
| `src/app/PolicyApplication.cpp` | Dispatch bằng exact `policy_contract`, không chỉ bằng input dimension |
| `src/onnx/OnnxModel.{hpp,cpp}`, `src/onnx/PolicyContract.{hpp,cpp}` và `src/runtime/PolicyRunner.hpp` | Đọc tensor/metadata và fail-closed trên exact `policy_contract`, history schema, input `[1,332]` và output `[1,24]`; không nhận 332D chỉ từ dimension |
| `src/runtime/Tuning.{hpp,cpp}` | Mapping fail-closed hai nhánh: model legacy KHÔNG có `policy_contract` chỉ giữ các route hiện hữu an toàn `{83,270}`; model FlatPlus CÓ contract bắt buộc khớp exact `(flat_plus_h4_v1,332)`. Reject mọi trường hợp còn lại trước khi chọn profile; task này chỉ train/evaluate nhánh Flat 83D/H4 |
| `config/` | Profile H4 opt-in với gait period 0.6 s; `flat_gait_period_s` đã là 0.6 nên baseline H1 dùng thẳng Flat profile hiện có, không cần profile riêng |
| `policy/locomotion/history/` | Artifact/manifest riêng cho simulator |
| `CMakeLists.txt` | Chỉ thêm source/test mới |
| `tests/contracts/single_policy_smoke_test.cpp` | Cho 332D đi qua chỉ khi exact history metadata hợp lệ; giữ hard rejection với 332D không rõ schema |
| `tests/` | Buffer/order/reset/golden trace/ONNX smoke tests |

Default simulator hiện vẫn là Rough 270D; phải giữ nguyên vì đó là cấu hình ngoài phạm vi. Task này chỉ thêm route FlatPlus bắt nguồn từ Flat 83D, không train hay sửa hành vi Rough.

`Tuning::ResolveLocomotionProfile()` phải đưa validation exact contract/dimension lên trước profile resolution: legacy không metadata chỉ nhận 83/270, còn FlatPlus chỉ nhận exact contract H4 332D. Không được chỉ "mở rộng danh sách dimension" hoặc suy FlatPlus từ 332D trần.

Simulator hiện có logic overlay dùng offset cố định của frame 83D sau khi observation đã được tạo. Với 332D, các offset đó sẽ ghi vào sai thời điểm/term. V1 phải disable overlay cho policy history thay vì áp dụng các index cũ.

Gait period của simulator đã được đưa về 0.6 s (2026-09-05) ở cả `config/tuning.yaml`, default `LocomotionProfile` và init `flat_profile`, khớp Flat-Meta/common-PD và HB; `tests/unit/tuning_profile_test.cpp` đã đổi expectation tương ứng và toàn bộ 67 CTest PASS. Baseline H1 và candidate history vì vậy chạy chung Flat profile 0.6 s, không cần profile đối chứng riêng. Vẫn phải validate gait period đọc từ metadata khi nạp model mới, và mọi trần tốc độ `flat_speed_*`/`flat_fast_speed_*` đều được tune dưới nhịp 0.8 cũ nên phải đo lại bằng autotest trước khi dùng làm giới hạn evaluation.

## 6. Trình tự triển khai đề xuất

### Phase 0 — Khóa baseline và làm sạch cổng test

- Ghi lại SHA256 baseline PT/ONNX và current config.
- Tạo snapshot/branch có tracking trước khi sửa; checkout hiện nằm trong một worktree cha có rất nhiều thay đổi/untracked files.
- Chạy checker common-PD và CTest legacy trước thay đổi.
- Sửa riêng baseline test drift: `tuning_include_test.cpp` và `config_reader_test.sh` còn kỳ vọng `policy_goc.onnx`, trong khi config hiện chọn `policy_goc_2.onnx`.
- Không trộn baseline-test cleanup và history implementation trong cùng commit.

**Gate 0:** có log baseline xanh hoặc danh sách test đỏ đã được xác nhận là lỗi có sẵn.

### Phase 1 — Định nghĩa contract và task `Flat-Plus`

- Thêm constants/layout module.
- Thêm env factory kế thừa Flat-Meta.
- Register task/runner riêng.
- Unit-test actor dim 332, critic dim 98, action dim 24.
- Test Flat/Flat-Meta legacy vẫn 83/98/24.

**Gate 1:** task registry tách biệt và không có shared-config mutation.

### Phase 2 — Test native history semantics

- Dùng trace có số dễ nhận biết cho từng term/timestep.
- Xác nhận term-major, oldest->newest.
- Xác nhận reset backfill first frame.
- Xác nhận history append đúng một lần mỗi environment/policy step.
- Xác nhận actor corruption/noise xảy ra trước khi frame được đưa vào history như mJLab hiện hành.

**Gate 2:** expected vector 332D bằng tensor do environment trả về, element-by-element.

### Phase 3 — Warm-start converter và train smoke

- Tạo converted checkpoint trong namespace mới.
- Chạy actor parity test với nhiều random batches.
- Chạy 2-5 iterations, ít env trên CPU/GPU để phát hiện shape/NaN/reset lỗi.
- Chạy smoke H4 from-scratch và H4 mapped.
- Sau đó chạy khoảng 100 iterations với số env vừa phải trước full-scale.

**Gate 3:** không NaN/Inf, loss/normalizer hợp lệ, checkpoint save/reload/export được.

### Phase 4 — Full training và ablation

Giữ cố định giữa các arm:

- reward terms/weights;
- command ranges và standing fraction;
- friction/domain randomization;
- PPO hyperparameters, network hidden dims và rollout length;
- train budget;
- evaluation scenarios;
- seed set;
- parent checkpoint/initialization strategy;
- optimizer reset semantics và số iteration tính từ sau initialization.

Không được so H1 warm-start từ checkpoint cũ với H4/H5 from-scratch rồi quy mọi
chênh lệch cho history. **Kết quả chính của roadmap là cả H1/H4/H5 train from
scratch với cùng initialization protocol.** Nếu chạy thêm mapped warm-start,
đó là ablation phụ, báo cáo riêng và không dùng để chọn schema thắng.

So sánh tối thiểu:

| Arm | Input | Mục đích |
|---|---:|---|
| H1 control | 83 | Đối chứng retrain cùng pipeline |
| H4 candidate | 332 | Candidate mặc định |
| H5 candidate | 415 | Kiểm tra lợi ích cửa sổ 80 ms |

Chạy tối thiểu 3 seed cho quyết định; không chọn bằng một run đẹp nhất.

**Gate 4:** chọn schema thắng dựa trên tracking, fall/recovery, smoothness, inference latency và consistency giữa seed. H5 chỉ thay thế H4 bằng một task/contract/artifact có version mới.

### Phase 5 — Export và parity

- Export ONNX từ checkpoint đã chọn.
- Checker xác nhận tensor types/shapes và metadata.
- So sánh PyTorch với ONNX Runtime trên random inputs và recorded traces.
- Lưu Git revision, config dump, dependency versions, checkpoint hash, ONNX hash và train command.

**Gate 5:** max action error nằm trong tolerance đã định, metadata đầy đủ, inference finite.

### Phase 6 — C++ packing parity

- Tạo một trace canonical gồm state, command, phase và previous action qua nhiều step/reset.
- Python xuất expected base 83D, history 332D và action.
- HB C++ và simulator C++ đọc cùng trace.
- So sánh element-by-element, đặc biệt tại step đầu, wrap-around, reset và re-activation.

**Gate 6:** vector packing phải exact hoặc trong float tolerance rất nhỏ; dimension-only PASS là chưa đủ.

### Phase 7 — Closed-loop simulator acceptance

Chạy baseline `policy_goc_2` và history candidate trên cùng điều kiện:

- đứng yên command zero;
- đi thẳng tiến/lùi;
- đi ngang trái/phải;
- quay trái/phải;
- command ramp và command step;
- stop -> stand -> restart;
- reset giữa episode;
- handover Stand/Dance -> Locomotion;
- nhiều seed và thời lượng đủ dài để quan sát drift/fall.

Ma trận chính phải nằm trong command envelope của Flat-Meta: `vx, vy` trong `[-0.6, 0.6]` m/s và yaw rate trong `[-1.0, 1.0]` rad/s. Các lệnh ngoài dải này phải được báo cáo thành extrapolation/stress test riêng, không trộn vào kết quả in-distribution.

Thu thập tối thiểu:

- pass/fall count và time-to-fall;
- velocity tracking RMSE theo trục;
- zero-command displacement/root velocity;
- action rate, target-q rate, torque/energy proxy;
- joint-limit/safety clamp events;
- ONNX inference p50/p95/p99 và toàn bộ policy-step latency.

**Gate 7 đề xuất:**

- Không tăng số fall so với baseline trên ma trận test cố định.
- Zero-command drift không xấu hơn baseline.
- Tracking RMSE không xấu hơn quá 10% ở bất kỳ trục chính nào; improvement phải lặp lại giữa seed nếu dùng để tuyên bố tốt hơn.
- Không tăng bất thường action/target-q rate hoặc clamp events.
- Inference p99 nhỏ hơn 20 ms và còn margin cho control loop.
- Tất cả legacy CTest/smoke test vẫn PASS.

### Phase 8 — HB preflight và hardware rollout

Chỉ bắt đầu sau Gate 7:

1. Copy artifact mới bằng filename riêng.
2. Verify SHA256, exact contract metadata, input/output shape và sequence inference smoke.
3. Build/test trên target architecture.
4. Test robot treo/chân không tải ở command zero.
5. Test stand tại chỗ có dây giữ và emergency stop sẵn sàng.
6. Test tốc độ thấp tiến/lùi/ngang/yaw.
7. Test stop/restart và policy re-activation để xác minh history reset.
8. Chỉ sau đó mới mở dần command envelope.

Preflight/hash/latency chỉ chứng minh contract/runtime, không thay thế nghiệm thu locomotion trên simulator và robot.

## 7. Test bắt buộc

### 7.1. Python/training

- `test_legacy_flat_dims_unchanged`
- `test_history_h4_actor_332_critic_98_action_24`
- `test_history_layout_term_major_oldest_to_newest`
- `test_history_reset_repeats_first_frame`
- `test_history_roll_and_wraparound`
- `test_last_action_is_previous_policy_action`
- `test_history_task_inherits_flat_meta_contract`
- `test_history_warm_start_output_matches_legacy`
- `test_onnx_metadata_complete`
- `test_torch_onnx_trace_parity`

### 7.2. HB/C++

- Base-frame 83D golden regression so refactor không đổi legacy values.
- H4 offsets/order golden test.
- Reset/backfill/reactivation test.
- 500 Hz outer loop nhưng chỉ 50 Hz append test.
- ONNX wrong input dim rejection.
- Missing/wrong `policy_contract` rejection.
- Resolver chỉ nhận `(flat_plus_h4_v1,332)`; 332D thiếu/sai contract phải fail trước profile selection.
- Dimension lạ như 84/271/999 phải fail ngay trong resolver, không âm thầm nhận Flat/Slope profile.
- Regression route Flat legacy 83D giữ nguyên; smoke route Rough 270D chỉ xác nhận không bị ảnh hưởng.
- Missing/wrong history order/padding rejection.
- NaN/Inf observation/action rejection.
- New profile/config parsing test.
- Legacy `policy_goc_2` load/inference test.

### 7.3. Cross-runtime

Cùng một trace phải thỏa:

```text
Python base frame == HB base frame == simulator base frame
Python H4 vector   == HB H4 vector   == simulator H4 vector
PyTorch action     ~= ONNX Runtime action ~= C++ ONNX action
```

## 8. Artifact và rollback

### 8.1. Namespace mới

Không ghi đè bất kỳ file baseline nào. Dùng ví dụ:

```text
checkpoints/r1_flat_plus/h4_v1/
  README.md
  init_from_common_pd.pt
  model_<iteration>.pt
  policy_flat_plus_h4_v1.onnx
  manifest.json
  env.yaml
  agent.yaml
  train.yaml
  evaluation.json
```

HB artifact phải nằm trực tiếp dưới `policies/flat/` nếu deploy script hiện không cho subdirectory, nhưng vẫn có filename riêng:

```text
HB/high_level_2/policies/flat/policy_flat_plus_h4_v1.onnx
```

### 8.2. Rollback

Rollback không cần xóa artifact mới. Chỉ khôi phục/chọn:

```yaml
flat_policy_contract: legacy_83
flat_model: policy_goc_2.onnx
```

Sau đó restart runtime và chạy legacy preflight. Không đổi symlink/model file theo cách làm mất hash/provenance.

## 9. Các rủi ro chính và biện pháp

| Rủi ro | Hậu quả | Biện pháp |
|---|---|---|
| Nhầm term-major với time-major | Model chạy nhưng action sai | Offset table + golden cross-runtime trace |
| Append history ở 500 Hz | History gần như lặp | Counter test, append chỉ trong policy step 50 Hz |
| Zero-pad khi reset | Startup khác training | Backfill first frame và reset test |
| Warm-load trực tiếp 83D | Shape mismatch hoặc init sai | Converter explicit + parity test; from-scratch fallback |
| Vô tình stack critic | Critic 392D, tốn memory và đổi bài toán | Chỉ override actor group; assert critic 98D |
| Sửa shared base config | Rough/Slope/Flat cũ cùng đổi | Factory/task mới kế thừa Flat-Meta |
| Dispatch theo dimension | Nhận nhầm schema 332D khác | Exact metadata contract fail-closed |
| Overlay dùng offset 83D | Ghi sai term/timestep | Disable overlay capability cho v1 |
| H quá dài | Tăng memory/latency, học chậm | H1/H4/H5 ablation, đo p99 và GPU memory |
| Chỉ nhìn checker/hash | Tưởng policy đã sẵn sàng | Tách contract, closed-loop sim và hardware gates |

## 10. Những phần không nằm trong thay đổi v1

- Không thêm camera hoặc terrain truth.
- Không đổi reward tuning nếu chưa có bằng chứng history gây regression; nếu cần đổi reward, chạy thành experiment riêng.
- Không đổi common-PD gains/action scale/default joint pose.
- Không đổi gait period 0.6 s.
- Không đổi policy frequency 50 Hz.
- Không chuyển sang GRU/LSTM/RMA trong cùng release H4; gait-conditioned task và A-RMA được thiết kế thành các release riêng ở mục 13 và 14.
- Không sửa/overwrite Flat, Rough, Slope, Meta/RMA artifacts hiện có.
- Không bật policy mới trong `HB/high_level_2/config/locomotion.yaml` trước acceptance.

## 11. Definition of Done

Chỉ đánh dấu hoàn tất khi tất cả điều kiện sau đều có evidence:

- [ ] Task `Unitree-R1-Flat-Plus` kế thừa Flat-Meta và legacy task dimensions không đổi.
- [ ] Actor/critic/action lần lượt là 332/98/24.
- [ ] Reset/backfill/order/update-rate được unit-test.
- [ ] `BaseObservationSchema83` và `ActorHistoryViewH4` là hai schema riêng; canonical trace lưu full frame 83D.
- [ ] Warm-start converter parity PASS hoặc quyết định train from scratch được ghi rõ.
- [ ] Full train hoàn tất với config/hash/seed provenance.
- [ ] ONNX metadata đầy đủ và checker fail-closed PASS.
- [ ] PyTorch/ONNX parity PASS.
- [ ] Python/HB/simulator history packing parity PASS.
- [ ] Legacy CTest/smoke test PASS.
- [ ] Closed-loop comparison với `policy_goc_2` PASS.
- [ ] Hardware preflight PASS.
- [ ] Tethered low-speed validation PASS.
- [ ] Chỉ sau đó mới đổi active config bằng một thay đổi riêng, có rollback đã thử.

## 12. Giới hạn của audit hiện tại

Audit hiện đã instantiate được `ManagerBasedRlEnv` trên CPU và chạy bộ test
`tests/test_flat_plus_contract.py`/`tests/test_flat_plus_warm_start.py` (26 pass),
nhưng chưa có full-scale training, trained ONNX hoặc closed-loop acceptance.
Dimension/history runtime đã được xác nhận cho task hiện có; các tiêu chí export,
parity, simulator và hardware vẫn phải qua các gate tương ứng.

Repo/workspace hiện có nhiều thay đổi sẵn có của người dùng. Khi triển khai phải chỉ chạm đúng các file đã liệt kê, tạo snapshot trước, và kiểm tra diff theo từng phase.
