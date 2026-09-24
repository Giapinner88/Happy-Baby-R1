# V5 — A-RMA: privileged information qua adaptation latent

> Adapter và actor deploy đều lấy lịch sử từ canonical Flat cơ bản 83D. Latent
> là conditioning được học, không thay thế hay mở rộng base observation nguồn.

| | |
|---|---|
| **Contract** | `flat_plus_arma_o83_k50_z8_v1` (chính) · `flat_plus_arma_h4_k50_z8_v1` (biến thể) |
| **Actor input** | 91 = base83 + latent 8 · hoặc 340 = H4 332 + latent 8 |
| **Adapter input** | `[50, 83]` time-major |
| **Bundle** | **hai** ONNX: `adaptation.onnx` + `actor.onnx` |
| **Trạng thái** | Đã có teacher/adapter/A-RMA finetune + export o83; closed-loop flat PASS, chưa hardware |
| **Khối lượng** | Lớn, nhiều pha, cần collect dataset |
| **Nguồn** | [RMA (RSS 2021)](https://arxiv.org/abs/2107.04034) · [A-RMA (IROS 2022)](https://arxiv.org/abs/2205.15299) |

## Điều kiện tiên quyết và trạng thái hiện tại

Sau khi **V1 qua Gate 7** (closed-loop simulator), và độc lập với V3 — hai
nhánh không phụ thuộc nhau về contract. Có thể chuẩn bị factor/schema trước,
nhưng không tạo artifact hoặc chạy training A-RMA trước Gate 7. Nếu muốn kết
hợp gait + A-RMA thì đó là contract thứ ba, chỉ mở sau khi **cả hai** đã có
acceptance riêng.

**Cập nhật 2026-09-08.** Gói kết quả chứa adapter `[50,83]`, actor `91 -> 24`,
checkpoint `model_20000.pt` và bundle cross-hash. Bundle đã được staging vào
`simulate_cpp/policy/locomotion/arma/`. `arma_preflight`, runtime finite/NaN
test và closed-loop flat smoke đều PASS. Đây mới là bằng chứng contract và
simulator; chưa phải hardware acceptance.

## Đã triển khai (2026-09-06) — chỉ phần schema

Plan cho phép chuẩn bị factor/schema trước Gate 7 nhưng cấm tạo artifact hoặc
train. Đúng phần được phép đó đã có:

### A-RMA là add-on của từng flat policy, không phải task độc lập

Không có task nào tên `Unitree-R1-ARMA`. Contract luôn mang tên flat policy mà
nó bọc: `flat_plus_arma_<arm>_k50_z8_v1`, và width là **prefix của flat actor đó
cộng latent**. Adapter thì arm-independent — luôn ăn `[K,83]` từ base frame
chung — nên một adapter dùng cho nhiều actor.

Registry sinh từ chính các flat contract (`ac.ARMS`), không gõ tay:

| Arm | Prefix | Prefix schema | Actor | Trạng thái |
|---|---:|---|---:|---|
| `o83` | 83 | `r1_common_pd_base83_v1` | 91 | đối chứng chính |
| `h4` | 332 | `flat_plus_h4_v1` | 340 | biến thể |
| `h5` | 415 | `flat_plus_h5_v1` | 423 | biến thể |
| `gait_h4` | 335 | `flat_plus_gait_h4_v1` | 343 | **gated** — §14.3 chỉ mở sau khi gait và A-RMA đều qua acceptance riêng |

`gated` nghĩa là **đặt tên được nhưng không build được**: contract string và
width phải tồn tại thì mới phát biểu được cái gate, còn `require_buildable()`
raise nếu có ai định tạo artifact.

Test `test_every_flat_contract_has_an_arma_arm` bắt buộc mọi flat contract đã
khai phải có arm tương ứng — thêm một flat variant mà quên A-RMA thì test đỏ,
chứ không phải đến lúc cần mới phát hiện.

> Bản trước hardcode đúng 2 arm và suy `actor_prefix_schema` bằng
> `BASE_83 if arm == "o83" else contract_id(4)`. Nhánh `else` đó chỉ đúng khi có
> đúng 2 arm; thêm arm thứ ba là metadata khai sai cha mà **không lỗi gì báo ra**
> — đúng class lỗi mà cả vòng metadata sinh ra để chặn. Đã thay bằng tra bảng,
> có negative control.

| Có | File |
|---|---|
| Ba schema tách biệt, contract name có K và Z, latent bound, bundle 2 component cross-pin hash | `src/tasks/velocity/config/r1/arma_contract.py` |
| Adapter history view `[K,83]` time-major, dựng lại từ buffer per-term chứ **không** reshape vector 332D | `src/tasks/velocity/mdp/arma_observations.py` |
| Privileged factor vector 25D + reader tường minh từng factor | cùng file |
| Test (18), gồm test layout time-major đo trên tensor thật | `tests/test_arma_contract.py` |

### Factor schema theo đúng thứ được randomize

Plan liệt kê nhiều factor candidate. Trong env này **ba** factor thật sự được
randomize và đã đo là varies per-env (8 env, `Unitree-R1-Flat-Meta`):

| Factor | Dim | Event | Đo được |
|---|---:|---|---|
| `foot_friction` | 1 | `foot_friction` | ✅ cả 14 foot geom varies, chung một giá trị mỗi env (`shared_random=True`) |
| `base_com_offset` | 3 | `base_com` | ✅ varies, std ≈ 0.025/0.035/0.031 trên biên ±0.05 |
| `encoder_bias` | 24 | `encoder_bias` | ✅ varies, trong ±0.015 rad |

Tổng 28D.

> **Bẫy index.** `asset_cfg.body_ids`/`geom_ids` là entity-local; `sim.model.*`
> là global model. Reader phải đi qua `robot.indexing.body_ids[...]` /
> `indexing.geom_ids[...]`. Đọc thẳng id local sẽ trả về nominal của body khác —
> một cột hằng số trông y như "DR không chạy". Xem nợ đã biết ở `00_INDEX.md`.

Mass/inertia scale, motor strength, joint damping, latency, restitution **không**
được randomize ở đây — khai chúng vào schema sẽ tạo cột hằng số, đúng cái bẫy
§14.5 cảnh báo. Schema mang nhãn `candidate_unaudited` cho tới khi chạy audit A0.

**Chưa làm:** toàn bộ phase A1-A5 — teacher PPO với oracle latent, collect
dataset, supervised adapter, gate A4 oracle-to-estimated, PPO fine-tune, export
bundle 2 ONNX, checker bundle, và package `src/tasks/velocity/arma_locomotion/`.

## Điểm phân biệt A-RMA với RMA thường

Ba pha, và pha cuối mới là A-RMA:

```text
Teacher : e_t --mu--> z*_t ;  pi(x_t, z*_t) --> action24
RMA     : history --phi--> z_hat_t ;  pi(x_t, z_hat_t) --> action24
A-RMA   : đóng băng phi và mu, FINE-TUNE pi bằng PPO khi nó thực sự nhận z_hat
          không hoàn hảo
```

Critic dùng privileged state còn actor không có latent thì **chỉ là asymmetric
actor-critic**, chưa phải A-RMA.

Lúc deploy: robot **không** được nạp `e_t`, encoder `mu`, hay critic. Chỉ adapter
`phi`, actor, và cảm biến deployable.

## Ba schema phải giữ độc lập

```text
r1_common_pd_base83_v1          [83]      frame chuẩn, 50 Hz
flat_plus_h4_v1                 [332]     term-major, actor-only history
flat_plus_base83_sequence_k50   [50,83]   time-major, adapter history
```

Adapter **không** nhận `[K,332]` — đó là history-của-history, dư thừa và sai vai
trò. Cả hai view phải lấy từ **cùng** nguồn frame 83D. Đây chính là lý do
`BaseObservationHistory` của V1 lưu frame nguyên vẹn thay vì buffer 332 float.

## Việc dễ làm sai nhất

| | |
|---|---|
| Chọn privileged factor bừa | Factor không quan sát được từ history thì adapter không tái tạo nổi; A-RMA chỉ giúp actor chịu sai số, không tạo ra thông tin không tồn tại |
| Copy schema 12D của `rma_meta_v1` | Schema đó có ground normal + uphill direction, làm cho slope selector; trên Flat gần như hằng số |
| Tái dùng `rma_meta` làm low-level actor | Nó là policy **categorical chọn expert**, không sinh action 24D |
| Lineage latent lệch | Cùng `latent_dim=8` không có nghĩa cùng latent basis; actor và adapter phải cross-pin hash |
| Bỏ qua A4 gate | Nếu RMA thất bại lớn mà đã dùng PPO fine-tune để che thì đang che lỗi hệ thống |

## Bundle hai thành phần

Đây là điểm khác biệt vận hành lớn nhất so với V1/V3: runtime phải chạy **hai**
model nối tiếp (adapter → actor), fail-closed khi hash/schema/lineage không
khớp, và có đường an toàn khi latent NaN/Inf/quá cũ. `model_manifest.conf` hiện
tại chỉ mô tả một model — cần mở rộng thành danh sách component. Kế hoạch V1 đã
cố ý để manifest hỗ trợ được điều này từ đầu.

---

## Kế hoạch chi tiết (nguyên văn từ bản gộp 2026-09-05)

### Roadmap A-RMA

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
