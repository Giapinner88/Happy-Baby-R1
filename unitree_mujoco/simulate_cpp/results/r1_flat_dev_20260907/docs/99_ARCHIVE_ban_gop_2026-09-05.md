# Kế hoạch R1 Flat-Plus: observation history và gait-conditioned locomotion

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

- **From scratch:** không cần sửa `scripts/train.py`; `Flat-Plus` mặc định `warm_start_checkpoint=None`.
- **Mapped warm start:** chạy converter tạo checkpoint 332D tương thích, rồi truyền rõ `--warm-start-checkpoint <converted.pt>`; chỉ thêm task map vào `train.py` sau khi converter parity test đã PASS.

Khuyến nghị chạy smoke cả hai nhánh `H4-scratch` và `H4-mapped`, rồi chọn nhánh full training dựa trên stability/learning curve. Không mặc định dùng checkpoint converted trước khi checker xác nhận.

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

Không được so H1 mặc định warm-start từ `model_flat_goc.pt` với H4/H5 scratch hoặc mapped từ một parent khác rồi quy mọi chênh lệch cho history. Có hai ablation hợp lệ:

- tất cả H1/H4/H5 train from scratch với cùng initialization protocol; hoặc
- tất cả dùng cùng `model_flat_common_pd.pt` làm parent, trong đó H4/H5 dùng newest-slice mapping, optimizer/critic/iteration cùng reset theo một quy tắc.

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

Audit này đã xác minh source, checkpoint, ONNX, hashes và các đường C++ liên quan. Python environment hiện có trên máy chưa cài đúng `mjlab`/`rsl_rl` theo pin của checkout, nên chưa instantiate full training environment trong audit này. Vì vậy dimension/history runtime của task mới vẫn phải được xác nhận lại ở Gate 1-3 sau khi triển khai trong environment đúng dependency.

Repo/workspace hiện có nhiều thay đổi sẵn có của người dùng. Khi triển khai phải chỉ chạm đúng các file đã liệt kê, tạo snapshot trước, và kiểm tra diff theo từng phase.

## 13. Task `flat_plus_gait` lấy tinh túy từ gait-conditioned G1

Nguồn tham khảo chính: [Gait-Conditioned Reinforcement Learning with Multi-Phase Curriculum for Humanoid Locomotion, arXiv:2505.20619v3](https://arxiv.org/abs/2505.20619v3).

### 13.1. Phạm vi và quyết định kiến trúc

Task mới chỉ lấy bốn ý tưởng cốt lõi của bài báo:

1. Một policy chung học nhiều mode, không train một policy Stand và một policy Walk riêng.
2. Gait mode được biểu diễn tường minh bằng one-hot và đưa vào actor lẫn critic.
3. Reward dùng chung và reward theo mode được route từ cùng một gait state.
4. Training tăng dần độ khó bằng curriculum, không bật mọi mode và mọi transition ngay từ đầu.

Candidate đầu tiên cho R1 được khóa như sau:

```text
Tên ngắn:               flat_plus_gait
Task ID:                Unitree-R1-Flat-Plus-Gait
Env factory:            unitree_r1_flat_plus_gait_env_cfg()
Runner factory:         unitree_r1_flat_plus_gait_ppo_runner_cfg()
Experiment:             r1_flat_plus_gait
Policy contract:        flat_plus_gait_h4_v1
Parent policy:           flat_plus_h4_v1
Mode set:                STAND, WALK, W2S
Gait encoding:           one-hot 3D, order STAND,WALK,W2S
Actor input:             335D
Critic input:            101D
Action:                  24D raw joint-position offset
Actor/critic topology:   MLP (512,256,128), ELU
Policy/FSM rate:         50 Hz
```

Không đưa `RUN` và `R2W` vào v1. Bài báo mới xác nhận Stand, Walk và W2S trên G1 thật; running mới được báo cáo trong simulation. Factory Flat-Meta đặt range ban đầu `vx/vy = [-0.6,0.6] m/s`, còn shared curriculum hiện hữu có thể mở rộng về sau, nhưng cả hai đều chưa phải một running curriculum được nghiệm thu như envelope tới 4 m/s trong bài báo. Khi muốn thêm chạy, phải tạo mode schema 5D, task, reward/curriculum, model và contract mới; không đổi ý nghĩa 3D của artifact này.

Không thay MLP bằng LSTM trong candidate chính. Nếu vừa thay history/reward/curriculum/gait ID/lẫn network, kết quả tốt hơn hoặc xấu hơn sẽ không thể quy cho thành phần nào. LSTM được giữ thành ablation riêng ở mục 13.8.

### 13.2. Observation contract 335D

Gait one-hot chính xác:

| Enum | One-hot | Ý nghĩa |
|---|---|---|
| `STAND=0` | `[1,0,0]` | Command dừng và robot đã ổn định |
| `WALK=1` | `[0,1,0]` | Có command tịnh tiến hoặc quay |
| `W2S=2` | `[0,0,1]` | Đang triệt vận tốc và thu chân về đứng |

Không có vector `[0,0,0]`, multi-hot hoặc giá trị mềm trong contract v1. Mỗi row phải finite, chỉ chứa `0/1`, và tổng bằng 1.

Actor layout:

```text
[0,332)    = H4 của canonical base frame 83D
[332,335)  = current gait one-hot [STAND,WALK,W2S]
```

Toàn bộ offset H4 ở mục 4 giữ nguyên. Gait ID chỉ được nối **một lần ở hiện tại**, không stack bốn lần. Cách triển khai khuyến nghị là tạo observation group `gait` 3D riêng, rồi để runner ghép group theo đúng thứ tự:

```python
observations = {
  "actor": actor_h4_group,       # [332], group history_length=4
  "critic": critic_current_group, # [98], group history_length=1
  "gait": gait_current_group,    # [3], không history
}

runner_cfg.obs_groups = {
  "actor": ("actor", "gait"),
  "critic": ("critic", "gait"),
}
```

Cách này sạch hơn việc chèn `gait_id` vào actor group rồi phải override history từng term, đồng thời giữ được H4 implementation của parent. Nếu thêm `gait_id` trực tiếp vào group có `history_length=4`, gait ID cũng bị stack và actor sẽ thành `(83+3) x 4 = 344D`. `344D` là contract khác, không được chạy dưới tên `flat_plus_gait_h4_v1`.

Critic giữ current observation 98D, rồi append current gait ID:

```text
[0,98)     = current Flat-Meta critic observation
[98,101)   = current gait one-hot
```

Gait ID là deployable intent/mode, không phải privileged information. Contact truth, ground reaction force, base linear velocity lý tưởng hoặc simulator gait label không được chèn vào actor.

RSL-RL `MLPModel` mặc định concatenate mọi group rồi dùng một `EmpiricalNormalization` trên toàn vector. Cách đó không an toàn cho curriculum: ở phase WALK-only, hai chiều STAND/W2S có variance bằng 0; khi chúng xuất hiện, giá trị normalized có thể tăng đột ngột do chỉ còn epsilon. Task phải dùng model wrapper riêng với semantics:

```text
normalize H4 332D bằng normalizer kế thừa parent
pass-through gait one-hot 3D ở giá trị raw 0/1
concat [normalized_h4, raw_gait3]
MLP (512,256,128) -> action24
```

Critic làm tương tự: normalize prefix 98D, pass-through gait3. ONNX vẫn nhận một tensor phẳng 335D và tự split ở offset 332; HB/C++ không tự normalize gait ID. Model wrapper chỉ thay tiền xử lý input, không đổi topology MLP.

Chỉ override `get_latent()` là **không đủ**. Stack khai báo của checkout là `mjlab==1.2.0`, kéo theo `rsl-rl-lib==5.0.1`. Trong `MLPModel` của version này:

- `get_latent()` concatenate tất cả group rồi normalize;
- `update_normalization()` cũng concatenate tất cả group độc lập với `get_latent()` rồi update running statistics;
- `as_onnx()` trả `_OnnxMLPModel`, và `forward()` của wrapper export gọi `obs_normalizer(x)` trên toàn tensor phẳng.

Vì vậy `GaitConditionedMLPModel` bắt buộc phải sở hữu và test cả ba đường:

```text
get_latent(obs):
  prefix = obs[actor_or_critic]
  gait = obs[gait]
  return concat(prefix_normalizer(prefix), gait)

update_normalization(obs):
  prefix_normalizer.update(obs[actor_or_critic])
  # không concatenate/update gait

as_onnx():
  return _OnnxPartialNormMLPModel(...)

_OnnxPartialNormMLPModel.forward(x):
  prefix = x[..., :P]       # P=332 actor
  gait = x[..., P:P+3]
  return mlp(concat(prefix_normalizer(prefix), gait))
```

Critic dùng cùng contract với `P=98`; actor ONNX dùng `P=332`, flat input size 335 và custom dummy input `[1,335]`. Không được kế thừa `_OnnxMLPModel` mặc định cho export gait policy. Tùy cách thay normalizer, đường mặc định có thể normalize sai gait mà không báo lỗi hoặc fail do shape; cả hai đều là contract failure.

Base 83D và H4 tiếp tục dùng semantics hiện tại, gồm phase 0.6 s và phase observation bị zero khi command dưới threshold hiện hành. V1 không thay đồng thời phase semantics. Nếu cần giữ phase tuần hoàn trong W2S hoặc stack lịch sử gait ID, đó phải là ablation/contract mới.

History không reset khi `WALK -> W2S -> STAND` hoặc `STAND -> WALK`; transition cần nhìn thấy chuyển động trước đó. History chỉ reset ở episode reset, policy activation/reactivation hoặc safety handover như mục 4.3.

### 13.3. Một gait FSM duy nhất cho train, simulator và HB

Không để observation, reward và runtime tự suy ra mode bằng ba đoạn threshold khác nhau. Tạo một `GaitModeState`/`GaitConditionedVelocityCommand` làm nguồn duy nhất cho:

- one-hot đưa vào actor/critic;
- mask reward;
- curriculum và command program;
- telemetry/metrics;
- golden trace Python/C++.

FSM candidate:

```text
                  move request
STAND --------------------------------> WALK
  ^                                      |
  |                                      | stop request
  | R1-stable continuously for T_settle  v
  +------------------------------------- W2S

W2S -- move request --> WALK
```

Quy tắc candidate ban đầu:

- `stop request`: `||v_xy_cmd|| < 0.10 m/s` và `|yaw_cmd| < 0.10 rad/s`.
- `move request`: `||v_xy_cmd|| > 0.15 m/s` hoặc `|yaw_cmd| > 0.15 rad/s`.
- Dải giữa hai threshold giữ state trước đó để tránh chatter.
- `W2S -> STAND`: stop request vẫn còn và R1 deployable-stability predicate đúng liên tục trong `T_settle`.
- `STAND/W2S -> WALK`: move request; reset toàn bộ settle timer.
- Khi reset/activate: nếu có move request thì `WALK`; nếu stop request và stability predicate đúng thì `STAND`; còn lại `W2S`.
- Nếu W2S vượt timeout mà chưa ổn định, không ép sang Stand; giữ W2S, phát telemetry timeout và để safety layer xử lý nếu state xấu.

Các threshold trên là candidate cho smoke/simulation, chưa phải hằng số hardware. Trước khi khóa contract phải đo log đứng yên/đi chậm/quay tại chỗ và chọn hysteresis không làm mất command nhỏ hợp lệ.

Bài báo chỉ chuyển W2S sang Stand khi **low speed và double support cùng duy trì 1.5 s**. R1 v1 cố ý bỏ contact truth khỏi deployment FSM và thay double-support bằng IMU angular velocity + joint-velocity RMS; do đó `1.5 s` không còn là threshold đã được paper xác nhận cho predicate mới. Chỉ dùng `1.5 s` là **numeric seed ban đầu** trong sweep/calibration, sau đó tune `T_settle` bằng log và closed-loop transition matrix của R1 trước khi khóa `flat_plus_gait_fsm_v1`. Trong simulation, paper predicate có thể được log song song là oracle diagnostic, nhưng không được điều khiển actor/FSM deploy.

Stability predicate của FSM deployment chỉ được dùng tín hiệu có trên robot thật, ví dụ base angular velocity từ IMU và joint-velocity RMS. Threshold cụ thể phải lấy từ phân bố log baseline đứng ổn định, rồi khóa trong manifest. Không dùng MuJoCo foot contact hoặc true base linear velocity để quyết định gait ID nếu HB không có estimator tương đương đã được xác minh.

Double-support/contact truth vẫn có thể dùng trong reward và critic khi train vì không đi vào runtime FSM. Nếu sau này có foot-contact estimator phần cứng, phải kiểm tra false-positive/false-negative và Python/C++ parity trước khi tạo một FSM contract mới có contact gating.

Training phải có latch per-environment `gait_for_action[k]`. Observation `o_k`, action `a_k` và reward của transition do `a_k` tạo ra đều dùng đúng latch `gait_for_action[k]`; reward không được đọc live FSM state có thể đã đổi. Chỉ sau khi reward đó được tính, command manager mới chốt final command cho observation kế tiếp và FSM update đúng một lần để tạo `gait_for_action[k+1]`. Reset phải khởi tạo `gait_for_action[0]` trước observation đầu tiên.

Invariant latch từ **final command** áp dụng cho mọi runtime. Cảnh báo GUI cụ thể chỉ thuộc `play` với Viser joystick: GUI handles mặc định là `None`, checkbox mặc định tắt, và khi bật thì `UniformVelocityCommand.compute()` chỉ override ba command của **một selected env index** sau `super().compute()`. Headless training, offscreen training video và Native viewer không đi qua override này. Riêng Viser parity, gait command phải update/latch selected env sau khi `super().compute()` đã trả về; nếu update FSM trong `_update_command()`, gait ID có thể trễ một observation. Partial reset chỉ reset state/timer/latch của đúng `env_ids`.

### 13.4. Gait ID khác phase và A-RMA latent

Ba tín hiệu có vai trò độc lập:

| Tín hiệu | Câu hỏi nó trả lời | Nguồn |
|---|---|---|
| Command 3D | Robot cần đi/quay nhanh bao nhiêu? | Người điều khiển hoặc command generator |
| Phase 2D | Đang ở đâu trong chu kỳ bước chân? | Gait clock 0.6 s |
| Gait ID 3D | Đang tối ưu hành vi WALK, W2S hay STAND? | Deployable gait FSM |
| A-RMA `z_hat` | Dynamics/ma sát/tải/motor hiện tại có đặc điểm gì? | Adapter suy ra từ history |

Gait ID không được đưa vào privileged-factor vector rồi bắt A-RMA suy luận lại. Nó là intended mode biết trước. A-RMA latent không được dùng thay gait ID vì latent basis không có nhãn ổn định và có thể thay đổi giữa các lần train.

### 13.5. Reward routing theo đúng mode

Reward tổng quát:

```text
r_total = r_shared
        + I[gait=WALK]  * r_walk
        + I[gait=W2S]   * r_w2s
        + I[gait=STAND] * r_stand
```

Router lấy mask trực tiếp từ `GaitModeState`. Các reward mới không được tự tính lại `total_command > 0.1` bằng công thức riêng. Điều này sửa nguồn bất nhất hiện tại: phase dùng norm 3D, trong khi một số reward dùng `norm(v_xy) + abs(yaw)`.

Phân nhóm candidate:

| Nhóm | Reward/penalty |
|---|---|
| Shared | linear/angular command tracking, torso orientation, body angular velocity, angular momentum, termination, joint acceleration, joint limits, action rate |
| WALK | phase contact pattern, foot clearance, foot slip/drag, soft landing, walking posture, straight-knee diagnostic/candidate reward |
| W2S | smooth slowdown, action/target smoothness, double-support acquisition, soft landing, foot drag/slip, posture interpolation về đứng |
| STAND | stillness, zero drift, double support, feet flat/alignment, upright torso, standing/default pose, stance slip |

`variable_posture` trong task hiện tại chọn standing/walking/running bằng magnitude command. Riêng `flat_plus_gait`, thay nó bằng mode-aware posture reward để `W2S` có target/tolerance rõ và không vô tình rơi vào nhánh `running_threshold=1.5`. Không sửa hàm/config reward của legacy task.

Trước khi thêm W2S reward, phải có routing-parity test trên phần command domain mà các predicate legacy cùng cho một kết quả và nằm ngoài deadband hysteresis `0.10–0.15`. Baseline hiện dùng nhiều công thức threshold khác nhau, nên parity tuyệt đối trên toàn bộ grid là bất khả thi. Mọi command thuộc vùng predicate bất đồng/deadband phải được liệt kê thành **expected divergence**; shared reward value không phụ thuộc routing vẫn phải match ở mọi điểm. Sau gate này mới thêm W2S và các term mới.

Trọng số ban đầu phải xuất phát từ scale reward R1 hiện tại. Không copy trực tiếp bảng weight của G1 vì G1 có 23 action, morphology, torque/PD, observation và simulator khác. Mỗi reward mới bắt đầu ở weight 0, được ramp trong curriculum và có ablation riêng.

### 13.6. Phần human-inspired nên lấy và phần chưa nên lấy

Nên lấy ngay:

- explicit W2S thay vì đổi đột ngột Walk -> Stand;
- double-support/feet-flat mục tiêu cho Stand;
- smooth deceleration và contact acquisition cho W2S;
- straight-knee metric trong stance;
- log angular momentum riêng của tay, chân và toàn thân;
- giữ angular-momentum regulation để khuyến khích tay bù chuyển động chân.

Repo đã có `angular_momentum_penalty`; trước hết giữ weight baseline và bổ sung decomposition metrics. Chỉ thêm arm-leg compensation reward mới nếu log cho thấy tổng yaw momentum cao hoặc tay không bù chân. Không bật đồng thời toàn bộ shoulder/elbow/symmetry/amplitude reward của paper, vì sẽ không biết term nào cải thiện chuyển động và có nguy cơ làm tay cạnh tranh với balance.

Straight-knee cũng bắt đầu là metric. Chỉ chuyển thành reward khi xác nhận current policy crouch quá mức trong stance và term không đẩy knee tới joint limit hoặc tăng impact khi hạ chân.

Domain randomization v1 giữ đúng phần inherited từ Flat-Meta. Không copy thêm mass/PD-gain randomization của paper trong cùng gait release. Repo có pattern random `pseudo_inertia`/`pd_gains`, nhưng exporter hiện có nguy cơ đọc gain đang bị randomize từ model để ghi metadata deploy. Mass/motor/PD randomization chỉ được mở như experiment riêng sau khi metadata lấy canonical common-PD defaults và có test chứng minh gain deploy không phụ thuộc sample simulator cuối cùng.

### 13.7. Curriculum ba phase đã điều chỉnh cho R1

Không dùng lịch ba phase của paper nguyên xi vì v1 không có Run/R2W. Curriculum R1 vẫn giữ tinh thần tăng dần độ khó:

`unitree_r1_flat_meta_env_cfg()` hiện chỉ đặt command range ban đầu về `vx/vy=[-0.6,0.6]`, nhưng vẫn kế thừa shared `command_vel` curriculum; shared term có thể mở rộng về sau tới `vx=[-1,2]`, `vy=[-1,1]`. Vì vậy factory gait bắt buộc phải:

```python
cfg.curriculum.pop("command_vel", None)
cfg.curriculum["gait_curriculum"] = ...
```

Nếu không thay term này, task có thể vô tình chạy ngoài command contract và nhánh posture `running` hiện hữu có thể được kích hoạt dù v1 không có `RUN`. Exact range ở từng gait phase phải do gait curriculum riêng sở hữu và được lưu trong config/checkpoint provenance.

#### C0 — WALK bootstrap

- Gait ID cố định `WALK`.
- Command sampler loại bỏ vùng stop; không sinh command zero nhưng vẫn phủ forward/lateral/yaw trong envelope Flat-Meta.
- Chỉ shared + WALK rewards hoạt động.
- Warm-start từ `flat_plus_h4_v1`, sau đó cho phép toàn bộ actor/critic tiếp tục học.
- Gate bằng survival, tracking, slip/contact pattern và action-rate trên nhiều seed; không mở C1 chỉ vì đã đủ số iteration.

#### C1 — thêm STAND và W2S

- Bổ sung các segment command đi -> zero-hold và zero -> đi.
- Bật FSM `WALK/W2S/STAND`, gait one-hot và reward routing đầy đủ ba mode.
- Ramp W2S/Stand rewards từ 0; không bật toàn bộ weight trong một update.
- Zero hold phải dài hơn `T_settle` đã calibrate cộng observation margin để môi trường thực sự quan sát được W2S -> Stand.
- Log occupancy, transition count, dwell time và timeout theo từng mode. Không chấp nhận curriculum nếu một mode gần như không có sample.

#### C2 — transition robustness

- Trộn command step, ramp, stop/restart và yaw-only transition thay vì chỉ IID resample.
- Randomize thời điểm stop trong cả left/right stance phase.
- Đặt push/domain perturbation trước, trong và sau W2S; giữ range trong safety envelope đã xác minh.
- Mở đầy đủ Flat-Meta command/friction distribution.
- Fine-tune tất cả ba mode để giảm catastrophic forgetting của Walk khi thêm Stand.

Lần triển khai đầu nên chạy mỗi phase bằng command/config rõ ràng và resume từ checkpoint đã gate, thay vì tự động nhảy phase theo global step. Sau khi pipeline ổn định mới có thể đóng gói thành `GaitCurriculumCfg`; phase/stage và checkpoint lineage phải được lưu để resume không quay sai curriculum.

Phải phân biệt hai kiểu load:

- actor-only warm-start từ checkpoint `flat_plus`: copy model/normalizer cần thiết nhưng đặt `common_step_counter=0`, stage `C0`, reset FSM/H4 và làm mới observation cache;
- same-task resume `flat_plus_gait`: restore iteration, optimizer, normalizer, `common_step_counter` và **global curriculum stage**; environment mới khởi tạo phải reset H4/FSM/latch, sau đó sinh mode ban đầu từ command/stability trước observation đầu tiên.

Không restore per-environment FSM/history từ checkpoint training thông thường vì physics state, command state và RNG tương ứng không được restore. Chỉ một chế độ exact mid-rollout resume riêng, snapshot/restore nguyên tử toàn bộ environment + command + RNG + H4 + FSM, mới được giữ state per-env.

Runner hiện có restore `common_step_counter` từ checkpoint; nếu tái sử dụng mù cho actor-only warm-start, gait curriculum có thể nhảy thẳng sang phase cuối. Converter/runner gait phải unit-test riêng hai semantics này.

Không random gait label độc lập với command/FSM. Ví dụ gán `STAND` trong khi command tiến 0.6 m/s tạo target mâu thuẫn và khiến actor học bỏ qua gait ID.

### 13.8. Network, warm-start và ablation LSTM

Candidate chính giữ topology actor/critic `(512,256,128)` ELU. Chỉ first layer đổi:

```text
Actor: [512,332] -> [512,335]
Critic: [512,98]  -> [512,101]
```

Warm-start bảo toàn output:

1. Copy 332 cột actor H4 vào `[0,332)`.
2. Đặt ba cột gait mới `[332,335)` bằng zero.
3. Copy bias, layer sau và action std.
4. Với critic, copy 98 cột cũ và zero ba cột gait mới, hoặc khởi tạo critic mới; quyết định phải giống giữa các ablation.
5. Copy nguyên normalizer H4 cho prefix 332D; gait3 đi qua nhánh raw/pass-through và không có running statistics. Không tự normalize one-hot ở C++.

Ngay sau converter, action của policy 335D phải bằng parent 332D cho cùng H4 input với mọi gait ID, `atol <= 1e-6`. Đây chỉ là initialization parity; sau PPO actor phải học sử dụng gait ID.

Ablation tối thiểu:

| Arm | Input/network | Mục đích |
|---|---|---|
| `flat_plus` | H4 332D, MLP | History-only control |
| `flat_plus_gait` | H4 332D + gait3, MLP | Candidate đầy đủ |
| no gait input | H4 332D, cùng routing/curriculum | Đo giá trị conditioning |
| no reward routing | H4 + gait3, mọi gait reward cùng bật | Đo routing như paper |
| no curriculum | H4 + gait3, ba mode từ đầu | Đo curriculum như paper |
| no W2S | Vẫn 335D; W2S channel luôn 0, Stand/Walk đổi trực tiếp | Đo giá trị transition mode mà không đổi capacity/input shape |
| shuffled/counterfactual gait ID | H4 + gait3 | Xác nhận actor thực sự dùng ID |

Nhánh recurrent riêng:

```text
Tên:             flat_plus_gait_lstm
Contract:        flat_plus_gait_lstm_v1
Per-step input:  current base83 + current gait3 = 86D
Paper-fidelity:  LSTM hidden 64, 1 layer, MLP [32]
Output:          action24
```

Ngoài paper-sized network, cần một LSTM capacity-matched có số parameter actor nằm trong khoảng ±10% của H4-MLP; nếu không, so sánh sẽ trộn lợi ích recurrence với chênh lệch capacity. LSTM contract phải có `h/c` state I/O, reset mask, sequence rollout, truncated-BPTT và export/runtime parity riêng. Không đưa LSTM vào artifact `flat_plus_gait_h4_v1`.

Mọi quyết định dùng ít nhất 3 seed, cùng command program, reward, domain randomization, budget và parent initialization. Kết quả paper không chứng minh LSTM tốt hơn frame stack vì paper không có ablation này.

### 13.9. Ma trận file dự kiến cho task mới

Training repo:

| File/khu vực | Thay đổi dự kiến | Cách cô lập |
|---|---|---|
| `src/tasks/velocity/config/r1/gait_contract.py` | Contract mode order, dimensions, FSM fields, metadata | File mới; không đổi H4/legacy constants |
| `src/tasks/velocity/mdp/gait_command.py` | `GaitMode`, state/timer, command programs và one-hot | Command term mới chỉ task gait dùng |
| `src/tasks/velocity/mdp/gait_observations.py` | `gait_id()` đọc state canonical | File mới; không đổi output các observation cũ |
| `src/tasks/velocity/mdp/gait_rewards.py` | Mode masks, W2S/Stand terms và metrics | File mới; legacy reward functions giữ nguyên |
| `src/tasks/velocity/mdp/gait_curriculums.py` | Gait curriculum/stage controls | File mới; không đổi command curriculum task khác |
| `src/tasks/velocity/mdp/__init__.py` | Export các module gait mới | Chỉ thêm symbol, không thay symbol cũ |
| `src/tasks/velocity/config/r1/env_cfgs.py` | `unitree_r1_flat_plus_gait_env_cfg()` kế thừa Flat-Plus, thêm current `gait` group và reward router | Không mutate factory parent/shared config |
| `src/tasks/velocity/config/r1/rl_cfg.py` | Runner config `r1_flat_plus_gait` | MLP/PPO baseline giữ nguyên ở task khác |
| `src/tasks/velocity/config/r1/__init__.py` | Register `Unitree-R1-Flat-Plus-Gait` | Registration opt-in mới |
| `src/tasks/velocity/rl/gait_conditioned_model.py` | `GaitConditionedMLPModel` override `get_latent()`, `update_normalization()` và `as_onnx()`; custom ONNX wrapper split/normalize prefix rồi nối gait raw | Tránh cả training-stat contamination lẫn export normalize sai gait |
| `src/tasks/velocity/rl/flat_plus_gait_runner.py` | Export qua custom `as_onnx()`, ghi metadata multi-group và phân biệt warm-start với resume | Không sửa behavior của `VelocityOnPolicyRunner`/Flat-Plus runner |
| `scripts/init_flat_plus_gait_checkpoint.py` | Converter 332/98 -> 335/101 | Artifact mới, không sửa parent |
| `scripts/check_flat_plus_gait.py` | Check task/layout/FSM/reward/ONNX metadata | Checker mới |
| `tests/test_flat_plus_gait_contract.py` | Unit/integration tests gait task | Không đổi expectation legacy |

HB runtime:

| File/khu vực | Thay đổi dự kiến |
|---|---|
| `src/gait/GaitModeScheduler.hpp` | FSM deployable, hysteresis, timers và one-hot; độc lập với legacy `GaitScheduler` phase clock |
| `src/policy/FlatPlusGaitController.hpp` | Build base83 -> H4 -> append current gait3 -> inference24 |
| `src/policy/FlatPlusGaitPolicyProfile.hpp` | Exact contract/dim/mode order/capability |
| `src/app/Application.cpp` | Dispatch exact contract và update FSM một lần mỗi policy step chỉ ở route mới |
| `src/config/Tuning.*` | Whitelist/validate profile và FSM fields; default legacy giữ nguyên |
| `config/locomotion_flat_plus_gait.example.yaml` | Cấu hình opt-in; không sửa `config/locomotion.yaml` |
| `policies/flat/policy_flat_plus_gait_h4_v1.onnx` | Model namespace riêng |
| `src/logging/` | Gait mode, dwell, transition reason, timeout và command telemetry |
| `tests/` | FSM/layout/reset/metadata/regression tests |

MuJoCo C++ simulator:

| File/khu vực | Thay đổi dự kiến |
|---|---|
| `src/controllers/locomotion/GaitModeScheduler.*` | Cùng semantics FSM với HB |
| `src/controllers/locomotion/FlatPlusGaitController.*` | H4 332 + gait3 packing |
| `src/app/PolicyApplication.cpp` | Exact contract dispatch và scripted transition scenarios |
| `src/runtime/Tuning.{hpp,cpp}`, `src/onnx/OnnxModel.{hpp,cpp}`, `src/onnx/PolicyContract.{hpp,cpp}` và `src/runtime/PolicyRunner.hpp` | Route fail-closed `(flat_plus_gait_h4_v1,335)`, parse/validate gait metadata và tensor `[1,335] -> [1,24]`; reject unknown/mismatch |
| `config/` | Profile opt-in, gait period 0.6 s, FSM fields |
| `policy/locomotion/gait/` | Artifact/manifest riêng |
| `tests/` | Cross-runtime FSM/golden trace/closed-loop tests |

Không dùng enum `STAND` của slope command hoặc previous-expert one-hot trong `rma_meta` làm gait ID. Chúng có semantics và runtime khác.

### 13.10. Metadata, artifact và runtime order

ONNX/manifest tối thiểu:

```text
policy_contract=flat_plus_gait_h4_v1
base_observation_schema=r1_common_pd_base83_v1
base_observation_dim=83
actor_input_schema=flat_plus_gait_h4_v1
actor_input_dim=335
critic_input_dim=101
action_dim=24
actor_obs_groups=actor,gait
actor_obs_group_dims=332,3
critic_obs_groups=critic,gait
critic_obs_group_dims=98,3
actor_normalized_prefix_dim=332
critic_normalized_prefix_dim=98
gait_normalization=none_raw_onehot
normalizer_contract=prefix_only_then_raw_gait_v1
onnx_preprocess_contract=normalize_prefix_then_append_raw_gait_v1
actor_history_steps=4
actor_history_order=term_major_oldest_to_newest
actor_history_padding=repeat_first
gait_mode_schema=stand_walk_w2s_onehot3_v1
gait_mode_dim=3
gait_mode_order=stand,walk,w2s
gait_mode_temporal_semantics=current_only
gait_mode_sample_count=1
gait_fsm_contract=flat_plus_gait_fsm_v1
gait_fsm_update_hz=50
gait_latch_semantics=observation_action_resulting_reward_v1
w2s_stability_predicate=r1_deployable_stability_v1
w2s_settle_seed_s=1.5
w2s_settle_seed_origin=paper_low_speed_and_double_support_reference_only
w2s_settle_dwell_s=<calibrated_R1_value>
w2s_settle_steps=<ceil(w2s_settle_dwell_s*50)>
command_curriculum=gait_curriculum_v1
gait_phase_semantics=legacy_command_threshold_zero
policy_hz=50
gait_period_s=0.6
source_paper=arxiv:2505.20619v3
```

Final FSM thresholds/stability predicate/timeout cũng phải nằm trong manifest hoặc một schema được hash từ manifest. Runtime từ chối model nếu mode order, dimension, FSM version, phase semantics hoặc gait period không khớp; không đoán bằng `input_dim=335`.

Exporter metadata mặc định chỉ mô tả term của actor group và không biết runner đã concatenate thêm group `gait`. Vì vậy `FlatPlusGaitRunner` bắt buộc ghi rõ `actor_obs_groups=actor,gait`, `critic_obs_groups=critic,gait`, group dims, final slice `[332,335)` và partial-normalizer contract. Không được dùng metadata base rồi suy ra rằng model vẫn chỉ chứa H4 332D.

Metadata `gait_normalization=none_raw_onehot` chỉ là khai báo; nó không thay đổi ONNX graph và checker metadata không tự chứng minh được preprocessing. Gate export phải đi qua đúng production runner/custom `as_onnx()`, tạo prefix running statistics không identity, thử cả ba gait ID, rồi so PyTorch evaluation với ONNX Runtime. Chỉ gắn metadata sau khi graph parity này PASS.

Artifact:

```text
checkpoints/r1_flat_plus_gait/h4_v1/
  params/train_config.yaml
  params/gait_contract.yaml
  checkpoints/model_*.pt
  exported/policy_flat_plus_gait_h4_v1.onnx
  reports/ablation.json
  reports/closed_loop.json
  SHA256SUMS

HB/high_level_2/policies/flat/
  policy_flat_plus_gait_h4_v1.onnx
```

Contract thời gian chung cho training và runtime:

```text
reset: resolve c_0 -> update FSM -> latch g_0 -> build o_0

policy transition k:
1. dùng latch g_k khi tạo base83/H4 + gait3 cho observation o_k
2. ONNX inference o_k -> raw action a_k; lưu a_k làm previous action kế tiếp
3. apply a_k qua physics/PD
4. training reward của transition này chỉ đọc latch g_k
5. sau reward, resolve final command c_(k+1), gồm sampled/heading/GUI override
6. update FSM đúng một lần và latch g_(k+1)
7. đọc proprioception mới, tạo/append base83, pack H4 và build o_(k+1)
```

HB/C++ không tính training reward, nhưng mỗi command event phải được coi là xảy ra giữa action trước và observation kế tiếp; update/latch gait trước khi pack observation đó. Golden trace phải lưu `command`, `g_k`, `o_k`, `a_k`, reset flag và transition reason để phát hiện lệch một step. Gait transition không reset history. Episode/policy activation reset cả history và FSM theo quy tắc mục 13.3.

Config opt-in sau acceptance:

```yaml
flat_policy_contract: flat_plus_gait_h4_v1
flat_model: policy_flat_plus_gait_h4_v1.onnx
```

Hai dòng active trong `HB/high_level_2/config/locomotion.yaml` vẫn giữ `legacy_83`/`policy_goc_2.onnx` trong toàn bộ quá trình phát triển.

### 13.11. Test và acceptance riêng cho gait task

Unit/contract tests bắt buộc:

- `test_gait_one_hot_order_and_sum`
- `test_invalid_or_multihot_gait_rejected`
- `test_actor_layout_is_h4_332_plus_current_gait3`
- `test_gait_id_is_not_stacked_to_344`
- `test_critic_layout_is_98_plus_gait3`
- `test_fsm_walk_w2s_stand_and_restart`
- `test_fsm_hysteresis_prevents_chatter`
- `test_w2s_requires_continuous_settle_dwell`
- `test_w2s_settle_seconds_to_steps_uses_locked_policy_rate`
- `test_partial_env_reset_does_not_reset_other_fsm_states`
- `test_viser_joystick_latches_selected_env_final_command`
- `test_viser_joystick_does_not_change_other_envs`
- `test_headless_compute_has_no_gui_override`
- `test_fsm_uses_no_simulator_only_contact_truth`
- `test_observation_action_and_resulting_reward_share_latched_gait`
- `test_next_gait_updates_only_after_current_reward`
- `test_gait_transition_does_not_reset_h4`
- `test_episode_activation_resets_fsm_and_h4`
- `test_phase_semantics_remain_contract_exact`
- `test_mode_reward_masks_are_mutually_exclusive`
- `test_two_mode_router_matches_legacy_on_unambiguous_domain`
- `test_router_expected_divergence_is_locked_in_deadband`
- `test_curriculum_resume_preserves_stage`
- `test_standard_resume_resets_per_env_fsm_and_h4`
- `test_actor_only_warm_start_resets_counter_stage_and_obs_cache`
- `test_inherited_command_vel_curriculum_removed`
- `test_walk_bootstrap_never_samples_stop_dead_zone`
- `test_raw_gait_bypasses_actor_and_critic_normalizers`
- `test_partial_normalizer_updates_prefix_only`
- `test_curriculum_mode_unlock_has_no_normalization_spike`
- `test_onnx_export_matches_partial_normalizer`
- `test_332_to_335_warm_start_parity`
- `test_python_hb_cpp_gait_trace_parity`
- `test_gait_resolver_requires_exact_contract_335_pair`
- `test_unknown_contract_dimension_pair_fails_before_profile_resolution`
- `test_legacy_flat_plus_and_legacy83_unchanged`

`test_onnx_export_matches_partial_normalizer` phải tạo non-identity running stats cho prefix, chạy cả ba one-hot với batch lớn hơn 1, export qua `FlatPlusGaitRunner` thực tế và so output PyTorch/ONNX Runtime với `max_abs_error <= 1e-5`. Test normalization riêng phải assert running-stat shape chỉ là 332/98 và gait3 không có statistics/update path.

Closed-loop scenario matrix:

| Scenario | Kiểm tra chính |
|---|---|
| Stand 20 s | drift, slip, double support, action/torque noise |
| Walk ổn định ở nhiều `vx/vy/yaw` | tracking, fall, clearance, contact phase |
| Walk -> zero từ nhiều tốc độ | stop distance/time, W2S success, impact/slip |
| Walk -> zero ở nhiều gait phase | transition không phụ thuộc một chân dẫn cố định |
| Stand -> Walk | restart delay, stumble, tracking transient |
| Walk -> W2S -> Walk trước khi Stand | timer reset và recovery |
| Command quanh threshold | không chatter mode/action |
| Yaw-only -> stop -> restart | semantics quay tại chỗ |
| Push trước/trong/sau W2S | recovery và timeout |
| Handover Stand/Dance -> Locomotion | reset/initial mode đúng |

Metrics tối thiểu theo mode và transition:

- fall fraction, episode length và time-to-fall;
- velocity/yaw tracking RMSE;
- zero-command displacement/drift;
- W2S completion rate, settle time, stop distance và timeout rate;
- foot slip, double-support fraction, impact/soft-landing proxy;
- action rate, jerk, torque/energy proxy và safety clamp count;
- mode occupancy, transition count và dwell distribution;
- knee extension metric và arm/leg/total angular momentum;
- p50/p95/p99 inference + FSM + packing latency.

Trước full train phải khóa acceptance thresholds. Candidate ban đầu: ít nhất 95% W2S success trong simulator test set, không tăng fall fraction, tracking regression không quá 5% so với `flat_plus`, và stand drift/action-rate phải tốt hơn hoặc bằng parent. Không dùng một return tổng duy nhất để quyết định vì reward routing làm scale return thay đổi.

Hardware chỉ bắt đầu sau contract, ONNX parity, C++ parity và closed-loop simulator PASS. Trình tự: tethered Stand -> tốc độ thấp Walk -> W2S -> restart -> yaw; không thử Run/R2W dưới contract này.

### 13.12. Trình tự triển khai task `flat_plus_gait`

#### Gait Phase 0 — khóa parent và paper-derived contract

- Chỉ bắt đầu full training sau khi `flat_plus_h4_v1` qua Gate 7 simulator.
- Khóa mode order, dimensions, FSM inputs và reward grouping.
- Lưu paper version `2505.20619v3`; không tuyên bố reproduce paper.

**Gate G0:** parent hash/config/metrics và gait contract được ghi đầy đủ.

#### Gait Phase 1 — FSM, observation và reward-router tests

- Implement command/FSM, one-hot, group `gait` current-only riêng và reward masks trong task mới.
- Chạy routing parity trước khi bật reward mới.
- Xác minh train/HB/C++ dùng cùng transition trace.

**Gate G1:** 335/101/24 đúng, không có simulator-only leakage và golden trace PASS.

#### Gait Phase 2 — warm-start và WALK bootstrap

- Convert checkpoint H4 332D -> gait335D và chạy parity.
- Train/smoke C0-WALK; xác nhận policy không regression chỉ vì thêm ba cột zero-init.

**Gate G2:** WALK tracking/survival đạt parent envelope trên ít nhất ba seed.

#### Gait Phase 3 — STAND/W2S curriculum

- Resume checkpoint C0-WALK đã qua Gate G2.
- Bật transition command programs, mode-aware rewards và staged weight ramp.
- Theo dõi occupancy/timeout/catastrophic forgetting.

**Gate G3:** Stand, W2S và restart đạt threshold đã khóa; Walk không collapse.

#### Gait Phase 4 — robustness và ablation

- Chạy C2 curriculum với perturbation quanh transition.
- Chạy no-ID/no-routing/no-curriculum/no-W2S và LSTM research arms với cùng budget.
- Chọn candidate bằng multi-seed closed-loop metrics, không chọn bằng return đơn.

**Gate G4:** có báo cáo attribution và candidate được chọn có evidence.

#### Gait Phase 5 — export, simulator và hardware gate

- Export ONNX/metadata/artifact; PyTorch/ONNX parity.
- Chạy full C++ matrix và legacy regression.
- Sau PASS mới copy artifact qua HB namespace opt-in và chạy tethered sequence.

**Gate G5:** chỉ đổi active config bằng commit/thao tác riêng sau khi hardware acceptance và rollback rehearsal PASS.

### 13.13. Rủi ro và Definition of Done

| Rủi ro | Biểu hiện | Biện pháp |
|---|---|---|
| Gait ID bị stack | Input 344D thay vì 335D | Separate current `gait` group + dimension unit test |
| One-hot bị empirical-normalize | Mode mới mở tạo input spike gần `1/eps` | Partial normalizer: normalize prefix, pass-through gait3 |
| Inherited command curriculum còn chạy | Range tự mở rộng và chạm nhánh running ngầm | Pop `command_vel`, gait curriculum sở hữu exact range |
| Train/runtime FSM khác nhau | Mode/action đổi khác cùng trace | Shared schema + Python/HB/C++ golden trace |
| Dùng contact truth trong FSM | Simulation tốt, hardware sai mode | Chỉ deployable stability predicate |
| Reward vẫn tự threshold | Actor thấy mode A nhưng reward mask B | Một canonical `GaitModeState` |
| W2S bị kẹt | Timeout cao, không vào Stand | Settle diagnostics; không force transition mù |
| Actor bỏ qua gait ID | Counterfactual ID không đổi action | shuffled/fixed-ID ablation và sensitivity metric |
| Stand làm quên Walk | Tracking/survival giảm sau C1 | Mixed replay/occupancy floor và per-mode gate |
| Copy G1 weights | Reward scale/morphology sai | R1 baseline scale + staged ablation |
| Quy lợi ích cho LSTM sai | Nhiều biến đổi đồng thời | H4-MLP candidate + recurrent ablation riêng |
| Mode jitter quanh zero | Action chatter | enter/exit hysteresis + dwell tests |

Task chỉ được coi là hoàn tất khi:

- [ ] Task/contract/artifact mới hoàn toàn opt-in; legacy/H4 hashes và routes không đổi.
- [ ] Một actor duy nhất xử lý `STAND/WALK/W2S`; không có expert selector.
- [ ] Actor/critic/action đúng `335/101/24` và gait ID chỉ xuất hiện một lần.
- [ ] H4/critic prefix được normalize; gait3 raw không cập nhật/đi qua empirical normalizer.
- [ ] Shared `command_vel` curriculum bị loại khỏi task gait và exact range thuộc gait curriculum.
- [ ] FSM không phụ thuộc simulator-only truth và parity qua Python/HB/C++.
- [ ] Gait ID observation và reward mask đồng bộ cùng timestep.
- [ ] Curriculum WALK -> STAND/W2S -> robustness có checkpoint lineage rõ.
- [ ] Reward routing/no-routing, curriculum/no-curriculum và W2S/no-W2S được ablate.
- [ ] Ít nhất 3 seed và full closed-loop transition matrix có báo cáo.
- [ ] ONNX metadata/checker, PyTorch parity và cross-runtime packing PASS.
- [ ] Simulator acceptance, HB preflight, tethered Stand/Walk/W2S/restart PASS.
- [ ] Rollback về `legacy_83`/`policy_goc_2.onnx` đã được thử.

### 13.14. Quan hệ với A-RMA tương lai

Nếu sau này kết hợp task này với latent `Z=8`, actor hybrid sẽ là:

```text
H4 base observation 332D
+ current gait ID 3D
+ A-RMA estimated latent 8D
= actor input 343D
```

Adapter vẫn đọc history riêng `[K,83]` time-major; không đọc `[K,335]` hoặc `[K,343]`. Gait ID là intent tường minh, còn `z_hat` là dynamics latent suy luận. Nhánh này phải có tên/contract riêng, ví dụ `flat_plus_gait_arma_h4_k50_z8_v1`, và chỉ được tạo sau khi cả gait task lẫn A-RMA baseline độc lập đã có acceptance.

## 14. Roadmap A-RMA cho privileged/hidden information

### 14.1. Làm rõ yêu cầu

Trong A-RMA, không đưa trực tiếp các observation chỉ simulator biết vào actor khi deploy. Luồng đúng là:

```text
Teacher training:
  privileged factors e_t ──> environment encoder mu ──> teacher latent z*_t
  deployable actor input x_t + z*_t ──> actor pi ──> raw action 24D

Deployment/RMA:
  deployable history H_t ──> adaptation module phi ──> estimated latent z_hat_t
  deployable actor input x_t + z_hat_t ──> actor pi ──> raw action 24D

A-RMA fine-tune:
  freeze phi and mu
  fine-tune actor pi bằng PPO khi nó thực sự nhận z_hat_t không hoàn hảo
```

Phần fine-tune actor bằng estimated latent ở bước cuối là điểm phân biệt A-RMA với pipeline RMA hai pha cơ bản. Critic dùng privileged state nhưng actor không có adaptation latent chỉ là asymmetric actor-critic, chưa phải A-RMA.

Khi deploy, robot không được load/đọc `e_t`, privileged encoder `mu` hoặc critic. Chỉ adapter `phi`, actor và deployable sensors được phép tham gia inference.

### 14.2. Quan hệ giữa H4 và A-RMA

H4 không xung đột với A-RMA nếu giữ đúng ba schema độc lập:

```text
Canonical base frame:
  r1_common_pd_base83_v1
  shape [83], một full frame tại 50 Hz

History-only actor H4:
  flat_plus_h4_v1
  shape [332]
  term-major, oldest -> newest trong từng term

A-RMA adapter history:
  flat_plus_base83_sequence_kN_v1
  shape [K,83]
  time-major: full_frame(oldest) -> full_frame(newest)
```

Task gait-conditioned thêm một actor-input schema độc lập `flat_plus_gait_h4_v1 = H4 332D + current gait3 = 335D`. Đây không thay canonical base frame 83D hoặc adapter-history schema.

Adapter không được nhận `[K,332]`; đó là history-of-history, dư thừa và sai vai trò. Cả H4 packer và A-RMA adapter history phải lấy dữ liệu từ cùng nguồn full frame 83D trước khi tạo view riêng.

Hai history có thể có horizon, normalizer, padding và update rate khác nhau:

- H4 actor history: ngắn, H=4, term-major, `repeat_first`, append 50 Hz.
- A-RMA adaptation history: dài hơn, time-major `[K,83]`, contract startup riêng, append 50 Hz.

History bank phải track `valid_steps` và episode/reset boundary. Không window nào được chứa frame của hai episode hoặc hai lần activation khác nhau.

### 14.3. Hai phương án actor A-RMA cần so sánh

Giả sử latent dimension `Z=8`:

| Nhánh | Input actor | Dimension | Vai trò |
|---|---|---:|---|
| `A-RMA-O83` | current 83D + `z_hat` | 91 | Gần kiến trúc A-RMA chuẩn; temporal system identification nằm ở adapter |
| `A-RMA-H4` | H4 332D + `z_hat` | 340 | Hybrid giữ short history trực tiếp trong actor và thêm latent |
| `A-RMA-Gait-H4` | H4 332D + gait3 + `z_hat` | 343 | Chỉ mở sau khi gait task và A-RMA độc lập đều đã qua acceptance |

Khuyến nghị dùng `A-RMA-O83` làm đối chứng A-RMA chính vì nó tách rõ đóng góp của adaptation latent. `A-RMA-H4` là biến thể hợp lệ nhưng phải có contract và báo cáo riêng; actor và adapter đều xử lý thông tin temporal nên không thể quy toàn bộ improvement cho A-RMA.

Warm-start có thể bảo toàn output:

- `A-RMA-O83`: mở first layer từ `[512,83]` thành `[512,83+Z]`, copy 83 cột cũ và đặt cột latent bằng zero.
- `A-RMA-H4`: mở first layer từ `[512,332]` thành `[512,332+Z]`, copy 332 cột H4 và đặt cột latent bằng zero.
- `A-RMA-Gait-H4`: mở first layer từ `[512,335]` thành `[512,335+Z]`, copy toàn bộ gait actor và zero các cột latent mới.

Ngay sau mapping, output mới phải bằng policy parent với mọi latent input trong tolerance. Không có phép nén H4 -> O83 bảo toàn output tổng quát; O83 phải bắt đầu từ actor 83D, train lại hoặc distill thành experiment riêng.

### 14.4. Candidate A-RMA đầu tiên

Đây là candidate để benchmark, chưa phải giá trị tối ưu:

```text
Base observation:          [83] float32
Privileged factor vector:  [E] train-only, schema chưa khóa
Latent:                    [8] bounded, candidate so sánh thêm Z=16
Adapter history:           [50,83] time-major
History append rate:       50 Hz
K=50 sampled duration:     1.00 s; oldest-current span 0.98 s
Adapter inference rate:    bắt đầu 50 Hz để đúng semantics; sau đó ablate 25/10 Hz
Actor inference rate:      50 Hz
Actor output:              [24] raw joint-position action
Adapter startup:           candidate repeat_first + valid_steps
```

Tên contract ví dụ:

```text
flat_plus_arma_o83_k50_z8_v1
flat_plus_arma_h4_k50_z8_v1
```

Thay đổi bất kỳ `K`, `Z`, actor-input mode, factor order, padding, history order hoặc update cadence đều phải tạo contract/artifact mới.

### 14.5. Privileged factor schema

Không dùng tên chung chung như `hidden_obs`. Mỗi factor cần schema versioned ghi rõ:

- tên, offset và dimension;
- unit, nominal value, scale/normalization và clip;
- nguồn simulator và domain-randomization event tương ứng;
- range train và held-out range;
- episode-static hay time-varying;
- simulator-only hay có thể đo trên robot;
- lý do cho rằng ảnh hưởng của factor có thể suy ra từ history 83D.

Candidate factor cho Flat dynamics:

- foot-ground friction;
- body/link mass và inertia scale;
- payload hoặc torso COM offset;
- motor strength/torque/gain scale;
- joint damping/friction;
- actuator/control latency;
- restitution/compliance nếu các đại lượng này thực sự được randomize.

Chỉ giữ một factor nếu nó thỏa cả năm điều kiện:

1. Thực sự được randomize và có thể đọc chính xác trong simulator.
2. Có ảnh hưởng đo được lên locomotion.
3. Ảnh hưởng có khả năng quan sát từ gyro/gravity/joint/action history.
4. Teacher dùng oracle latent cải thiện closed-loop so với `z=0`/shuffled latent.
5. Adapter thu hẹp đủ oracle-to-estimated performance gap.

Không copy nguyên factor schema 12D của `rma_meta_v1`. Schema đó gồm ground normal và uphill direction, được tạo cho slope expert selector; trên Flat chúng gần như hằng số. Ngoài ra phải audit factor có khớp đúng trường được randomize hay không, ví dụ motor Kp/Kd, actuator gain, encoder bias và latency không được coi là cùng một đại lượng.

Base linear velocity, ideal contact force hoặc terrain truth cũng không tự động là factor tốt. Nếu không đủ observable từ deployable history, adaptation module không thể tái tạo đáng tin cậy; A-RMA fine-tune chỉ giúp actor chịu sai số, không tạo ra thông tin không tồn tại.

### 14.6. Module hiện có có thể và không thể tái sử dụng

Checkout hiện có `src/tasks/velocity/rma_meta/`, nhưng đây là policy categorical chọn giữa các frozen experts Flat/Slope-Up/Slope-Down/HOLD, không phải low-level actor sinh action 24D.

Có thể tái sử dụng dưới dạng pattern/generic utility:

- `EnvironmentFactorEncoder` và GRU `AdaptationModule` topology;
- episode-level dataset split, `valid_steps` masking và adapter normalizer;
- sharded manifests, hashes, atomic checkpoints và provenance;
- flow teacher -> collect -> supervised adaptation -> deployment-aware PPO;
- ONNX component metadata/checker pattern.

Không được tái sử dụng trực tiếp:

- `RmaMetaCfg` có expert names, HOLD và meta rate 10 Hz;
- `MetaActorCritic`, categorical PPO, previous-expert one-hot và switch penalty;
- expert registry, target crossfade và selection runtime;
- factor schema, trained encoder/adapter weights hoặc latent basis hiện tại.

Low-level A-RMA cần namespace riêng, ví dụ:

```text
src/tasks/velocity/arma_locomotion/
configs/arma/flat_plus_arma_o83_k50_z8_v1.yaml
scripts/train_arma_teacher.py
scripts/collect_arma_adaptation.py
scripts/train_arma_adaptation.py
scripts/evaluate_arma_rma_gap.py
scripts/finetune_arma_actor.py
scripts/export_arma_bundle.py
scripts/check_arma_bundle.py
tests/test_arma_locomotion_core.py
```

`VelocityOnPolicyRunner`/MLP config hiện tại không đủ biểu diễn `mu(e)`, `phi(history)` và latent-conditioned continuous actor. Cần custom actor-critic/continuous PPO runner hoặc một runner riêng, không sửa categorical `rma_meta` thành low-level actor.

### 14.7. Các phase training A-RMA

#### A0 — Factor và leakage audit

- Khóa factor schema/ranges/normalization.
- Xác nhận từng factor thực sự được randomize và log được.
- Tạo held-out dynamics combinations/ranges.
- Unit-test privileged factors không xuất hiện trong base 83D, adapter input hoặc deploy ONNX inputs.

#### A1 — Privileged teacher PPO

- Train encoder `mu(e_t) -> z*_t` cùng continuous actor `pi(x_t,z*_t) -> action24`.
- Critic có thể nhận privileged terms theo một train-only schema riêng.
- Chạy ablation `z=0`, shuffled/stale `z` và oracle `z*`.
- Nếu actor không phụ thuộc latent hoặc oracle latent không cải thiện closed-loop, không chuyển sang A2.

#### A2 — Thu adaptation dataset

Mỗi sample phải chứa đồng bộ:

```text
base history [K,83]
valid_steps
episode_id/reset_id/policy_step_id
teacher latent z*_t
privileged factors e_t
command
raw action
domain-randomization parameters
```

- Thu initial data on-policy từ privileged teacher.
- Sau khi có adapter đầu tiên, thu thêm rollout do estimated latent điều khiển để giảm distribution shift.
- Không cho window đi qua reset.
- Split train/validation theo whole episode và thêm held-out dynamics split; không random-split các frame kề nhau.

#### A3 — Supervised adaptation

Train:

```text
z_hat_t = phi(history_t)
L_adapt = ||z_hat_t - stop_gradient(z*_t)||^2
```

Adapter có normalizer riêng trên base83 frames hợp lệ. Ngoài latent MSE/cosine cần đo action error khi thay `z*` bằng `z_hat`, response time sau dynamics change và closed-loop gap theo từng factor/range.

#### A4 — RMA closed-loop gate

Freeze teacher actor/encoder, thay oracle latent bằng `z_hat` nhưng chưa fine-tune actor. So sánh:

```text
Oracle: pi(x,z*)
RMA:    pi(x,z_hat)
```

Nếu RMA thất bại lớn, phải sửa factor/history/adapter trước khi dùng PPO fine-tune để che lỗi hệ thống.

#### A5 — A-RMA deployment-aware PPO

- Freeze adaptation module và privileged encoder; assert không có gradient/optimizer parameter thuộc hai module này.
- Actor chạy bằng `z_hat` từ chính history/reset/padding/update cadence sẽ dùng khi deploy.
- Fine-tune continuous actor, và critic nếu thiết kế yêu cầu, bằng optimizer/iteration lineage mới.
- Đưa sensor noise, latent age, warm-up và reset behavior vào rollout.

#### A6 — Export và parity

Khuyến nghị export hai component riêng:

```text
adaptation.onnx: [1,K,83] -> [1,Z]
actor.onnx:      [1,83+Z] hoặc [1,332+Z] -> [1,24]
```

Nếu dùng gait-conditioned branch, actor contract riêng là `[1,335+Z] -> [1,24]`; gait ID vẫn do deployable FSM cung cấp, không do adapter dự đoán.

Phải có PyTorch/ONNX parity cho từng component và composed pipeline. Python, HB và simulator phải tạo cùng history, latent và action trên canonical trace.

### 14.8. Artifact, metadata và runtime bundle

Namespace riêng, không ghi đè H4/common-PD/rma_meta:

```text
artifacts/flat_plus_arma/o83_k50_z8_v1/
  checkpoints/
    teacher_final.pt
    adaptation_best.pt
    arma_finetune_final.pt
  exported/
    actor.onnx
    adaptation.onnx
  params/
    train_config.yaml
    base_observation_schema.json
    privileged_factor_schema.json
    latent_schema.json
    bundle_manifest.yaml
  reports/
    adaptation_validation.json
    closed_loop_evaluation.json
  SHA256SUMS
```

Actor và adapter chỉ hợp lệ khi cùng latent lineage. Cùng `latent_dim=8` không có nghĩa hai model có cùng latent basis. Bundle manifest và ONNX metadata phải cross-pin:

```text
bundle_contract
component_role=actor|adaptation
component_contract
source_checkpoint_sha256
base_observation_schema
actor_input_schema
privileged_factor_schema
latent_schema
latent_dim
adapter_history_steps
adapter_history_order
adapter_history_padding
actor_hz
adapter_history_append_hz
adapter_inference_hz
normalizer_sha256
companion_component_sha256
max_latent_age_steps
```

Runtime phải fail-closed nếu component hash, schema, normalization hoặc latent lineage không khớp. Latent NaN/Inf/quá cũ không được tiếp tục điều khiển vô hạn; controller phải chuyển qua safety path đã xác minh.

HB và C++ simulator sau này cần composite controller chạy adapter rồi actor. H4 v1 vẫn là bundle một component; manifest nên hỗ trợ danh sách component ngay từ đầu nhưng không thêm dormant A-RMA model/config vào active route.

### 14.9. Acceptance matrix A-RMA

So sánh tối thiểu:

| Arm | Mục đích |
|---|---|
| H1 83D | Baseline hiện tại |
| H4 332D | Short-history baseline |
| Gait-H4 335D | Gait-conditioned baseline, nếu đánh giá nhánh gait+A-RMA |
| Oracle latent | Upper bound của factor/latent design |
| RMA estimated latent | Đo oracle-to-estimated gap trước fine-tune |
| A-RMA final | Đo lợi ích thật của Phase A5 |
| Zero/shuffled/stale latent | Chẩn đoán actor có thực sự dùng latent hay không |

Ma trận phải gồm nominal, held-out friction/mass/COM/motor-strength combinations, payload và các transition giữa dynamics/terrain hợp lệ. Báo cáo:

- tracking, fall/time-to-fall, recovery và zero-command drift;
- jerk/action rate, energy/torque proxy và safety clamps;
- latent MSE/cosine nhưng không dùng chúng làm acceptance duy nhất;
- oracle-vs-estimated action/return gap;
- adaptation response time và latent age;
- actor latency, adapter latency và composed p99 latency dưới budget 20 ms.

Các test bắt buộc bổ sung:

- `test_privileged_factors_absent_from_deploy_inputs`
- `test_factor_schema_matches_randomized_sim_state`
- `test_adapter_history_is_k_by_83_time_major`
- `test_adapter_never_consumes_h4_332_history`
- `test_history_never_crosses_episode_reset`
- `test_adapter_padding_matches_training_and_cpp`
- `test_previous_action_semantics_match_base83`
- `test_actor_warm_start_parity_before_latent_training`
- `test_adapter_and_encoder_frozen_during_arma_finetune`
- `test_actor_adapter_latent_lineage_mismatch_rejected`
- `test_torch_onnx_composed_pipeline_parity`
- `test_adapter_update_rate_and_latent_age`
- `test_nan_or_stale_latent_fails_safe`
- `test_h4_and_legacy_contracts_unchanged`

### 14.10. Những việc nên chuẩn bị ngay trong H4 v1

Để H4 không khóa đường A-RMA nhưng vẫn giữ scope hiện tại nhỏ:

1. Giữ pure/versioned `BaseObservationSchema83` và `BuildBaseObservation83()`.
2. Coi H4 là `ActorHistoryViewH4`, không đổi tên canonical observation thành 332D.
3. History storage lưu full frame 83D, có `valid_steps`, reset ID và policy-step ID; layout/padding thuộc view.
4. Canonical golden trace lưu full 83D, reset flag và previous raw action; vector 332D là derived expected output.
5. Metadata tách `base_observation_schema` và `actor_input_schema`; dùng prefix `actor_history_*` thay cho `history_*` chung chung.
6. Adapter sau này lấy base83 trực tiếp và dùng normalizer riêng, không de-normalize H4 input.
7. Manifest hỗ trợ component role/list; H4 chỉ có một actor component.
8. Runtime dispatch bằng exact bundle/contract, không hard-code giả định mọi policy chỉ có một ONNX.
9. Log/replay tool có khả năng lưu base83 per-step; privileged factors chỉ được bật trong train-only collector tương lai.
10. Không thêm task/model/config A-RMA chưa dùng vào release H4.

Các chuẩn bị này không biến H4 thành A-RMA và không mở rộng actor input H4. Chúng chỉ giữ base frame, history representation và artifact composition đủ rõ để thêm A-RMA bằng một release độc lập sau này.

## 15. Nguồn kỹ thuật

- mJLab v1.2.0 `ObservationManager`: <https://github.com/mujocolab/mjlab/blob/v1.2.0/src/mjlab/managers/observation_manager.py>
- mJLab v1.2.0 `CircularBuffer`: <https://github.com/mujocolab/mjlab/blob/v1.2.0/src/mjlab/utils/buffers/circular_buffer.py>
- mJLab v1.2.0 ONNX exporter metadata: <https://github.com/mujocolab/mjlab/blob/v1.2.0/src/mjlab/rl/exporter_utils.py>
- RSL-RL v5.0.1 `MLPModel` normalization/ONNX wrapper: <https://github.com/leggedrobotics/rsl_rl/blob/v5.0.1/rsl_rl/models/mlp_model.py>
- Gait-conditioned multi-phase curriculum for Unitree G1, v3: <https://arxiv.org/abs/2505.20619v3>
- RMA, RSS 2021: <https://arxiv.org/abs/2107.04034>
- A-RMA, IROS 2022: <https://arxiv.org/abs/2205.15299>
- A-RMA project/method page: <https://ashish-kmr.github.io/a-rma/>
