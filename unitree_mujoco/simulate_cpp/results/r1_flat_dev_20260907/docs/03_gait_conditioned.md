# V3 — `flat_plus_gait_h4_v1`: gait-conditioned locomotion

> Nguồn proprioception duy nhất: canonical Flat cơ bản 83D. H4 là view dẫn xuất;
> gait one-hot 3D chỉ là conditioning tường minh, không thay base observation.

| | |
|---|---|
| **Contract** | `flat_plus_gait_h4_v1` |
| **Task** | `Unitree-R1-Flat-Plus-Gait` |
| **Actor / critic / action** | 335 / 101 / 24 |
| **Cha** | `flat_plus_h4_v1` (V1) |
| **Mode** | `STAND`, `WALK`, `W2S` — one-hot 3D |
| **Trạng thái** | **ĐÃ TRIỂN KHAI LÕI (phía train)** — chưa train/export/acceptance |
| **Khối lượng** | **Lớn nhất trong roadmap** |
| **Nguồn** | [arXiv:2505.20619v3](https://arxiv.org/abs/2505.20619v3) |

## Điều kiện tiên quyết

Chỉ bắt đầu sau khi **V1 qua Gate 7** (closed-loop simulator). Task này warm-start
từ checkpoint H4, nên nếu H4 chưa được chứng minh thì mọi so sánh đều trôi.

> **Ghi chú 2026-09-06.** Phần code phía training đã được viết trước Gate 7
> theo yêu cầu trực tiếp, để sẵn sàng chạy ngay sau khi H4/H5 xong. Điều kiện
> tiên quyết ở trên **vẫn còn hiệu lực về mặt kết luận**: nếu H4 chưa qua Gate 7
> thì kết quả gait không quy được về đâu, vì cha của nó chưa được chứng minh.
> Code sẵn sàng không phải là gate đã qua.

## Hai correction so với plan, đo được từ run thật (2026-09-07)

**1. Stability predicate phải LỌC trước khi lấy norm.**
§13.3 mô tả predicate trên tín hiệu tức thời. Đo trên run đầu: với nhiễu
exploration σ=0.263, predicate qua **0.1%** số bước và STAND vào được **0 lần**
trong 20000 iteration, trong khi cùng policy chạy tất định qua 93% và vào STAND
sau 1.74 s. Mode chỉ đạt được bởi policy đã ngừng thăm dò — tức chỉ lúc deploy,
không bao giờ lúc học.

Sửa: EMA τ=0.2 s trên **vector có dấu**, rồi mới lấy norm. Lọc `|ω|` thì vô ích:
`|ω|` không âm nên nhiễu dịch cả kỳ vọng (0.088 → 0.623). Sau khi lọc, dưới
nhiễu: `|ω|`=0.141, rms=0.170, qua **87.1%**, STAND 77.1%. **Ngưỡng 0.25/0.60
giữ nguyên**, vẫn một predicate cho ba runtime. Contract:
`filtered_imu_ang_vel_norm_and_joint_vel_rms_v2`.

**2. Phạt lúc dừng phải mask theo LỆNH, không theo mode được cấp.**
§13.5 route mọi reward theo mode. Nhưng các term `stand_*` là **phạt**, còn cửa
vào STAND lại là stability predicate — một hàm của trạng thái robot, tức policy
điều khiển được. Nên chiến lược tối ưu là **không bao giờ đủ ổn định để vào
STAND**. Đo được: khi ramp chạm weight đủ ở iteration 6000, occupancy tụt
0.040 → 0.009 còn `w2s_timeouts` tăng 0.27 → 0.44.

Nó gần như miễn phí vì `joint_vel_rms` — chính đại lượng chặn cửa — không bị
phạt ở đâu cả; `w2s_smooth_slowdown` chỉ phạt vận tốc *base*.

Sửa, lệch có chủ ý với bảng bucket §13.5:
- `stand_*` mask theo **`stop_requested`** (ngoại sinh, suy từ lệnh, không né được)
- thêm `stop_joint_stillness` phạt đúng đại lượng chặn cửa
- `w2s_*` giữ mask theo mode W2S — chúng nói về động lực học pha chuyển tiếp

Mode vẫn là **intent**: người điều khiển ra lệnh dừng, robot không được quyền phủ
quyết. Observation vẫn mang mode từ FSM; chỉ mask reward là đổi nguồn.

## Đã triển khai (2026-09-06, phía training)

| Thành phần | File |
|---|---|
| Contract 335/101, mode order, FSM constants, metadata | `src/tasks/velocity/config/r1/gait_contract.py` |
| FSM + latch + command program C0/C1/C2 | `src/tasks/velocity/mdp/gait_command.py` |
| `gait_id()` one-hot | `src/tasks/velocity/mdp/gait_observations.py` |
| Reward router + W2S/STAND terms + `gait_posture` | `src/tasks/velocity/mdp/gait_rewards.py` |
| Stage control + reward ramp | `src/tasks/velocity/mdp/gait_curriculums.py` |
| Model normalize-prefix-only (3 đường: latent/update/export) | `src/tasks/velocity/rl/gait_conditioned_model.py` |
| Runner + export gate | `src/tasks/velocity/rl/flat_plus_gait_runner.py` |
| Converter 332→335 | `scripts/init_flat_plus_gait_checkpoint.py` |
| Checker | `scripts/check_flat_plus_gait.py` |
| Test (29) | `tests/test_flat_plus_gait_contract.py` |

Đã đo trên env thật: actor 332+3, critic 98+3, action 24, one-hot hợp lệ, C0 pin
WALK, ONNX `[1,335] → [1,24]` với normalizer đúng 332 (tức wrapper partial-norm
được export chứ không phải `_OnnxMLPModel` mặc định), 54 khoá metadata.

**Chưa làm:** train, C++/HB runtime, golden trace cross-runtime, acceptance.
Phần ablation phụ ở §04 (no-gait-input, no-routing, no-curriculum, no-W2S,
shuffled ID) cũng chưa có task riêng.

## Khối lượng thật sự

Đây không phải "thêm 3 chiều vào observation". Cần mới:

- `GaitModeState` / `GaitConditionedVelocityCommand` — FSM **dùng chung** cho
  train, simulator và HB; ba đoạn threshold riêng là cách chắc chắn nhất để ba
  runtime bất đồng
- reward router theo mode + reward riêng cho W2S/STAND
- curriculum ba phase C0→C1→C2 với checkpoint lineage
- `GaitConditionedMLPModel` override **cả ba** đường `get_latent()`,
  `update_normalization()` và `as_onnx()` — chi tiết ở §13.2 bên dưới, và đây là
  cái bẫy dễ mất nhất
- `GaitModeScheduler` phía HB + simulator, độc lập với `GaitScheduler` phase clock
- converter 332→335, golden trace có `g_k` và transition reason

## Đối chiếu với bài báo — đã kiểm

| Bài báo | Bản R1 này |
|---|---|
| 5 mode: Stand/Walk/W2S/Run/R2W, one-hot 5D | **3 mode**, one-hot 3D; Run/R2W là contract khác |
| LSTM hidden 64 + MLP [32] | **MLP (512,256,128)** làm candidate chính; LSTM là ablation riêng → `04_gait_lstm.md` |
| Curriculum 3 phase, phase 3 là Running | Phase 3 thay bằng transition robustness |
| W2S→Stand khi low speed **và double support** giữ 1.5 s | Bỏ double support (HB không có contact estimator tin cậy); 1.5 s chỉ là **hạt giống** phải tune lại |
| Envelope tới 4.0 m/s | Giữ envelope Flat-Meta `vx,vy ∈ [-0.6,0.6]` |

---

## Kế hoạch chi tiết (nguyên văn từ bản gộp 2026-09-05)

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


> *(Mục 13.8 về network và ablation LSTM đã tách sang `04_gait_lstm.md`.)*

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
