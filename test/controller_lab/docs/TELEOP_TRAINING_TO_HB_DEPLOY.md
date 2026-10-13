# ALMI teleop training → HB deployment

> Ngày rà soát: 2026-09-24  
> Trạng thái: tài liệu thiết kế và checklist triển khai; chưa phải xác nhận robot thật đã an toàn.  
> Policy package: [../policies/loco-teleop/](../policies/loco-teleop/)

## 1. Kết luận ngắn

ALMI hiện đã có năm policy theo curriculum:

```text
locomotion → squat → teleop_gesture → walk_teleop → unified
```

Tuy nhiên, “teleop” trong training hiện tại **không phải** là raw stream từ tay cầm,
Quest, UDP hoặc DDS. Simulator tạo một reference cho 10 khớp tay, rồi policy học giữ
thân/đi bộ trong khi bám reference đó.

Policy hiện tại dùng contract:

```text
input : 105 floats
output: 24 floats
rate  : 50 Hz
name  : r1_almi_v2
```

Trong khi `HB/controller` hiện đang chạy contract `flat_plus_gait_h4_v1` với input
335 chiều. Vì vậy không được chỉ đổi `flat_model` sang một file ALMI ONNX. Cần thêm
adapter 105-D trong HB, test parity với IsaacLab, rồi mới bắt đầu các bước trên robot.

Package hiện tại là artifact để tích hợp và kiểm thử; chưa được đánh dấu
`direct_deployable` trong [`policies/loco-teleop/MANIFEST.yaml`](../policies/loco-teleop/MANIFEST.yaml).

## 2. Nguồn sự thật trong code

Khi có khác biệt giữa tài liệu cũ và tài liệu này, dùng các file sau làm source of truth:

| Hạng mục | File |
|---|---|
| 105-D observation, 24 joint, IDL map, scale | `ALMI-Open/ALMI_RL/r1_runtime/contract.py` |
| Torch implementation của observation/action | `ALMI-Open/ALMI_RL/r1_runtime/tensor_contract.py` |
| IsaacLab action/observation | `ALMI-Open/ALMI_RL/isaaclab_r1/mdp/actions.py`, `mdp/observations.py` |
| Stage và cách sinh reference | `ALMI-Open/ALMI_RL/isaaclab_r1/stages.py`, `mdp/events.py` |
| Reward/termination | `ALMI-Open/ALMI_RL/isaaclab_r1/mdp/rewards.py`, `mdp/terminations.py` |
| USD, actuator và PD lúc train | `ALMI-Open/ALMI_RL/isaaclab_r1/asset.py` |
| PPO | `ALMI-Open/ALMI_RL/isaaclab_r1/agents/rsl_rl_cfg.py` |
| HB policy loader và action safety | `HB/controller/src/policy/PolicyController.hpp` |
| HB joint/IDL/runtime timing | `HB/controller/src/config/RobotSpec.hpp` |
| HB profile hiện tại | `HB/controller/docs/flat_plus_locomotion.md` |
| Sim-to-real checklist đã có | `HB/controller/docs/PLAN_mimic_sim2real.md` |

## 3. Curriculum đang train

Các checkpoint đã đóng gói trong `HB/controller/policies/loco-teleop/`:

| Stage | Task | Reference chính | Checkpoint package |
|---|---|---|---|
| `locomotion` | `ALMI-R1-Locomotion-v1` | velocity command, không teleop | `model_9999.pt` |
| `squat` | `ALMI-R1-Squat-v1` | crouch reference | `model_14998.pt` |
| `teleop_gesture` | `ALMI-R1-TeleopGesture-v1` | arm reference, đứng tại chỗ | `model_24997.pt` |
| `walk_teleop` | `ALMI-R1-WalkTeleop-v1` | arm reference + velocity | `model_34996.pt` |
| `unified` | `ALMI-R1-Unified-v1` | locomotion + squat + teleop hỗn hợp | `model_41000.pt` |

### 3.1. Cách sample mode

| Stage | Crouch | Teleop | Stationary teleop |
|---|---:|---:|---:|
| locomotion | 0% | 0% | không |
| squat | 35% | 0% | không |
| teleop_gesture | 0% | 100% | có |
| walk_teleop | 0% | 100% | không |
| unified | 20% | 12.5% | không |

Ở `unified`, mỗi environment được chọn một mode locomotion, squat hoặc teleop khi
reset. Với `teleop_gesture`, command vận tốc của environment teleop bị đặt về zero và
`rel_standing_envs=1.0`, nên policy phải giữ thăng bằng trong khi bám tay.

### 3.2. Giới hạn của implementation hiện tại

Trong code hiện tại, reference được khởi tạo khi environment reset. Giá trị `8.0 s`
trong stage spec chủ yếu được dùng để tính finite-difference cho
`arm_velocity_reference`; nó **không biến simulator thành một nguồn teleop streaming
liên tục 8 giây một lần**.

Do đó, kết quả hiện tại chưa chứng minh được:

- latency và jitter của nguồn teleop thật;
- mất gói, hold-last-value và reconnect;
- retargeting từ hand pose/VR pose sang 10 joint reference;
- tương quan tự nhiên giữa các khớp tay;
- giới hạn acceleration/jerk của người điều khiển;
- sai số calibration giữa teleop device, robot và simulator.

Đây là phần phải bổ sung trước khi gọi là “teleop deployable”.

## 4. Teleop đang được biểu diễn như thế nào

### 4.1. Reference đầu vào

ALMI không đưa raw teleop packet vào network. Policy chỉ nhìn thấy:

```text
arm_position_reference : 10 giá trị rad
arm_velocity_reference : 10 giá trị rad/s
```

Thứ tự 10 joint là:

```text
left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw,
left_elbow, left_wrist_roll,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw,
right_elbow, right_wrist_roll
```

Tư thế tay mặc định là:

```text
left :  [0.35,  0.18, 0.0, 0.87, 0.0]
right:  [0.35, -0.18, 0.0, 0.87, 0.0]
```

Reference teleop hiện tại được sinh bằng offset độc lập, uniform:

```text
arm_position_reference = default_arm + U(-0.35, +0.35) [rad]
```

Velocity reference hiện tại được tính xấp xỉ:

```text
arm_velocity_reference = clip(
    (arm_position_reference - default_arm) / 8.0,
    -4.0,
    +4.0
) [rad/s]
```

Đây là distribution tổng hợp cho training ban đầu, không phải dữ liệu chuyển động của
người dùng.

### 4.2. Crouch/squat reference

`crouch_reference` có 2 phần tử. Với squat mode, alpha được sample trong `[0.35, 0.85]`.
Reward hiện tại hướng knee về:

```text
knee_target = default_knee + 0.9 * crouch_reference[0]
```

Squat giúp policy học thay đổi chiều cao trọng tâm, giữ hai chân hỗ trợ và chịu được tư
thế teleop mà không buộc người dùng phải đứng thẳng tuyệt đối. Vì HB cần teleop ổn định,
squat nên giữ trong curriculum; nhưng phải kiểm tra riêng về torque, contact và
self-collision trước hardware.

### 4.3. Ý nghĩa đối với HB teleop

HB phải chuyển nguồn teleop thật thành đúng hai vector reference trên. Có hai lựa chọn:

1. **Joint-reference path**: teleop source đã cung cấp 10 joint angle → lọc, clamp,
   giới hạn tốc độ rồi đưa vào observation.
2. **Pose-retargeting path**: teleop source cung cấp hand pose/VR pose → IK/retargeting
   tạo 10 joint angle → áp cùng clamp, rate limit và quy ước dấu.

Không được để HB nhận pose frame của thiết bị rồi nhét trực tiếp vào 10 ô joint. Frame,
axis, handedness, zero-offset và sign phải được calibration bằng golden pose.

## 5. Observation contract 105 chiều

Observation phải được nối đúng thứ tự sau, không có history và không có actor
normalization:

| Offset | Block | Kích thước | Tiền xử lý |
|---:|---|---:|---|
| 0 | `gyro` | 3 | `base_ang_vel * 0.25` |
| 3 | `gravity` | 3 | projected gravity trong body frame, không scale |
| 6 | `velocity_command` | 3 | `[vx, vy, yaw] * [2.0, 2.0, 0.25]` |
| 9 | `gait_phase` | 2 | `[sin(2πt/0.6), cos(2πt/0.6)]` |
| 11 | `joint_position` | 24 | `q_policy - default_q` |
| 35 | `joint_velocity` | 24 | `dq_policy * 0.05` |
| 59 | `last_action` | 24 | action policy step trước, sau clip `[-1,1]` |
| 83 | `crouch_reference` | 2 | reference hiện tại |
| 85 | `arm_position_reference` | 10 | rad, theo arm joint order |
| 95 | `arm_velocity_reference` | 10 | rad/s, theo arm joint order |
| 105 | tổng | **105** | `float32` |

Phase phải được zero khi norm command không vượt ngưỡng `0.1`. Phải dùng cùng điều
kiện này ở HB; chỉ cần khác phase hoặc thứ tự sin/cos là policy nhận một trạng thái
khác hoàn toàn.

### 5.1. Command range lúc train

```text
vx  ∈ [-0.60,  0.80] m/s
vy  ∈ [-0.25,  0.25] m/s
yaw ∈ [-0.60,  0.60] rad/s
```

HB không nên cho joystick vượt range train ở giai đoạn đầu. Nếu cần range lớn hơn,
phải retrain hoặc chứng minh bằng perturbed simulation; không chỉ đổi clamp runtime.

## 6. Action contract và joint mapping

Network output có 24 phần tử. IsaacLab clip output về `[-1,1]`, sau đó tạo position
target:

```text
q_target[i] = default_q[i] + clip(action[i], -1, +1) * action_scale[i]
```

Action scale theo đúng policy-joint order:

```text
0.15, 0.15, 0.15, 0.15, 0.3125, 0.3125,
0.15, 0.15, 0.15, 0.15, 0.3125, 0.3125,
0.15, 0.15,
0.375, 0.375, 0.4125, 0.4125, 0.4125,
0.375, 0.375, 0.4125, 0.4125, 0.4125
```

### 6.1. Policy joint order

```text
0..5   left  hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
6..11  right hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
12..13 waist_roll, waist_yaw
14..18 left  shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
19..23 right shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
```

ALMI IDL map là:

```text
POLICY_TO_IDL = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,15,16,17,18,19,22,23,24,25,26]
HEAD_TO_IDL   = [29,30]
```

Head không phải output của policy ALMI; khi dùng package này, head phải có policy/runtime
riêng hoặc giữ target an toàn đã định nghĩa. Không được tự suy ra head target từ 24 output.

### 6.2. Cảnh báo về scale của HB hiện tại

`HB/controller/src/config/RobotSpec.hpp` đang có action scale cho các profile cũ
khác ALMI. Khi thêm ALMI profile, phải đọc và validate `action_scale` từ metadata của
ALMI ONNX hoặc một profile constants được kiểm chứng; không dùng lại mảng scale của
flat/mimic hiện tại.

## 7. Simulator, actuator và nhịp điều khiển

### 7.1. Nhịp

```text
physics dt       = 0.005 s
decimation       = 4
policy frequency = 50 Hz
episode length   = 20 s
```

HB runtime hiện có vòng DDS 500 Hz và policy decimation 10, tương đương giữ target
policy trong 10 vòng low-level. Adapter phải bảo đảm:

- inference chạy đúng một lần mỗi 20 ms;
- `last_action` chỉ cập nhật sau mỗi inference thành công;
- nếu inference lỗi/NaN thì không đưa output rác vào frame tiếp theo;
- timestamp teleop và timestamp policy được log để đo latency thật.

### 7.2. Actuator contract lúc train

| Nhóm | Kp | Kd | Effort limit | Velocity limit |
|---|---:|---:|---:|---:|
| hip/knee | 100 | 2 | 60 Nm | 18.8 rad/s |
| ankle pitch/roll | 40 | 2 | 50 Nm | 30 rad/s |
| waist | 100 | 2 | 60 Nm | 18.8 rad/s |
| upper arm | 40 | 2 | 60 Nm | 18.8 rad/s |
| forearm/elbow/wrist | 20 | 1 | 33 Nm | 33.4 rad/s |

Armature lúc train là `0.01`. Đây là thông số mô phỏng/PD mà policy đã thích nghi;
HB phải lấy đúng gain, action scale, default pose và effort envelope từ model metadata
hoặc manifest tương ứng.

## 8. Kiến trúc HB cần xây dựng

Không sửa route `flat_plus` để giả vờ nhận ALMI. Nên thêm một profile/controller riêng,
ví dụ `r1_almi_v2`, theo chain sau:

```text
LowState + IMU + command + teleop source
              │
              ▼
      AlmiReferenceAdapter
      - frame/sign/calibration
      - clamp + rate limit
      - timeout + hold/retract policy
              │
              ▼
      AlmiObservationBuilder (105-D)
      - exact order/scales/phase
      - q/dq in ALMI joint order
              │
              ▼
      ONNX policy r1_almi_v2 (50 Hz)
              │
              ▼
      action clip + default_q + action_scale
              │
              ▼
      policy → IDL map + safety supervisor
              │
              ▼
      LowCmd (500 Hz hold)
```

### 8.1. Việc bắt buộc trong controller

Controller mới cần:

1. kiểm tra `policy_contract=r1_almi_v2`;
2. kiểm tra input `105`, output `24`;
3. kiểm tra `observation_layout`, `policy_joint_names`, `policy_to_idl`;
4. đọc/kiểm tra `default_joint_pos`, `action_scale`, `joint_stiffness`,
   `joint_damping` từ ONNX metadata;
5. từ chối chạy nếu thiếu metadata hoặc sai dimension;
6. xây `last_action` từ action đã clip và chỉ cập nhật khi inference finite;
7. map 24 target vào đúng IDL slots, giữ các slot ngoài policy theo safe state;
8. ghi log model hash, metadata hash, observation checksum và saturation counters.

`PolicyController.hpp` đã có các nguyên tắc fail-loud và target conversion cơ bản,
nhưng `BuildObservation()` hiện phải được mở rộng bằng một ALMI profile cụ thể. Chỉ
đổi đường dẫn model là chưa đủ.

### 8.2. Teleop timeout không được bypass policy safety

Teleop UDP hiện tại của HB có timeout, smoothing và retract logic. Khi dùng ALMI policy:

- teleop packet hợp lệ phải cập nhật **reference input**, không ghi đè trực tiếp
  `q_target[14..23]` sau policy;
- packet timeout phải chuyển reference về một trạng thái đã định nghĩa và ghi rõ
  `last_action`/phase behavior;
- smoothing/rate limit phải được đưa vào distribution training hoặc được chứng minh
  không làm thay đổi latency/phase quá mức;
- `teleop_lock_base=true` phải tương ứng với command zero và stationary behavior,
  không được dùng nhầm với `walk_teleop`.

## 9. Những việc phải bổ sung để sim-to-real đúng hơn

### 9.1. Teleop distribution

Distribution hiện tại `U(-0.35,+0.35)` độc lập từng joint là chưa đủ cho deploy. Cần
thêm dataset/replay format chứa ít nhất:

```text
timestamp nguồn teleop
timestamp nhận ở HB
10 arm joint reference hoặc hand pose
reference velocity/acceleration
command vx/vy/yaw
tracking validity / packet loss
calibration version
```

Sau đó:

1. đo percentile của biên độ, vận tốc và gia tốc thật;
2. retarget vào cùng 10 joint order của ALMI;
3. thêm correlated pose sampling, không chỉ independent joint noise;
4. mô phỏng delay/jitter/dropout/hold-last-value;
5. giới hạn reference bằng joint limit, velocity limit, acceleration limit và arm
   self-collision envelope;
6. train lại `teleop_gesture`, `walk_teleop`, rồi `unified` bằng distribution đã đo.

### 9.2. Dynamics randomization

Theo các bài học trong `PLAN_mimic_sim2real.md`, simulation acceptance nên có:

- PD gain randomization, khoảng ban đầu đề xuất `0.8–1.2` quanh nominal;
- effort limit randomization theo hướng yếu hơn, khoảng `0.7–1.0`;
- armature randomization;
- friction/backlash gần với đo thực;
- mass/COM/inertia perturbation;
- actuator/bus delay khoảng `0–15 ms`, sau đó thay bằng số đo DDS thực;
- sensor noise và bias đúng theo log robot;
- command/reference timing jitter.

Không nên xem các khoảng trên là chân lý cuối. Chúng là dải khởi đầu để policy không
phụ thuộc vào simulator lý tưởng; số đo robot thật phải cập nhật lại dải.

### 9.3. PD và metadata

Sai Kd giữa train và deploy từng là nguyên nhân đã được ghi nhận trong HB. Vì vậy:

- không để runtime tự fallback sang gain khác nếu model thiếu metadata;
- log Kp/Kd thực tế sau khi áp tuning scale;
- so sánh nominal train với command thật trên `LowCmd`;
- đo closed-loop tracking error, delay, torque saturation và dq noise từng joint;
- chỉ bật LPF sau khi log chứng minh nhiễu vượt giả định train, vì LPF cũng thêm phase delay.

## 10. Validation ladder trước robot thật

Các gate dưới đây là điều kiện đề xuất cho ALMI profile. Chúng không phải tuyên bố
policy hiện tại đã đạt.

### Gate A — artifact/contract

- `SHA256SUMS` pass;
- ONNX input `[batch,105]`, output `[batch,24]`;
- mọi output finite với zero/random bounded observation;
- metadata contract, layout, joint map, scale, gain đầy đủ;
- checkpoint sidecar trỏ đúng task, USD hash và checkpoint hash.

### Gate B — observation parity

Tạo cùng một state fixture và so sánh:

```text
IsaacLab build_observation_from_state
       ==
HB AlmiObservationBuilder
```

Phải test riêng phase đứng yên, phase đi bộ, command threshold, q/dq zero, teleop
reference, timeout và sign từng arm joint.

### Gate C — headless evaluation

Với fixed seed, ghi:

- unexpected termination/fall rate;
- velocity tracking error;
- arm reference tracking error;
- action saturation;
- torque/velocity limit utilization;
- finite-state violations;
- phase/command/reference traces.

### Gate D — perturbed simulation

Chạy lại Gate C với delay, gain, effort, mass/COM, friction và noise perturbation. Một
checkpoint chỉ được promote nếu không sụp ở các perturbation đã công bố. Simulator
nominal chạy mượt một mình không đủ để kết luận sim-to-real.

### Gate E — HB dry-run, không gửi torque thật

- load ONNX bằng `run_r1 --preflight` hoặc test tương đương;
- kiểm metadata/shape/hash;
- feed recorded LowState + recorded teleop vào adapter;
- so sánh action và target với golden output;
- kiểm tra timeout, NaN/Inf, rate limit, joint guard và e-stop;
- đo inference period, DDS period và jitter.

### Gate F — robot thật có kiểm soát

Thứ tự tối thiểu:

1. robot được treo/đỡ, không tải nguy hiểm;
2. policy chỉ giữ default pose, low gain và rate limit nhỏ;
3. đứng yên không teleop;
4. stationary teleop chậm, biên độ nhỏ;
5. squat nhỏ có người và e-stop;
6. walk command chậm;
7. walk + teleop;
8. tăng dần envelope sau khi review log.

Mỗi bước phải có operator, e-stop, timeout và phương án quay về known-good flat policy.

## 11. Definition of done cho HB ALMI teleop

Chỉ coi route này sẵn sàng cho robot thật khi tất cả điều kiện sau đạt:

- HB có `r1_almi_v2` profile riêng, không lẫn với 335-D `flat_plus`;
- có test golden observation/action giữa Python/IsaacLab và C++;
- có adapter teleop với calibration, timeout, dropout, rate limit và frame contract;
- policy được đánh giá trên nominal và perturbed simulation;
- Kp/Kd/action scale/default pose khớp artifact đang chạy;
- có log DDS period/jitter, tracking error, delay và torque saturation;
- có preflight/manifest/hash test cho model thực tế được nạp;
- đã qua dry-run không torque, warm-up, low-gain, tethered và e-stop;
- người vận hành đã xác nhận từng gate bằng log, không chỉ bằng việc “robot không ngã
  trong một lần chạy”.

## 12. Checklist triển khai tiếp theo

### P0 — adapter và parity

- [ ] Thêm `AlmiPolicyController`/profile `r1_almi_v2` vào HB.
- [ ] Thêm `AlmiObservationBuilder` đúng layout 105-D.
- [ ] Thêm parser metadata và fail-loud validation.
- [ ] Viết golden fixture cho zero state, standing, walking và teleop.
- [ ] Viết test policy-to-IDL map, action scale và `last_action`.
- [ ] Không đổi route `flat_plus` đang ổn định.

### P1 — teleop source thật

- [ ] Chốt nguồn teleop và frame convention.
- [ ] Chốt joint-reference hay pose-retargeting path.
- [ ] Ghi calibration/zero/sign/version vào log.
- [ ] Ghi packet timestamp, receive timestamp, dropout và jitter.
- [ ] Đo envelope position/velocity/acceleration thật.

### P2 — retrain sim-to-real

- [ ] Tích hợp recorded reference distribution.
- [ ] Tích hợp delay/dropout/smoothing tương ứng HB.
- [ ] Bổ sung dynamics randomization.
- [ ] Train lại theo thứ tự `teleop_gesture → walk_teleop → unified`.
- [ ] Promote checkpoint chỉ sau Gate C/D.

### P3 — robot thật

- [ ] DDS dry-run và preflight.
- [ ] Calibration/PD verification.
- [ ] Tethered stationary teleop.
- [ ] Squat envelope nhỏ.
- [ ] Walk + teleop theo từng cấp.
- [ ] Đóng gói model đã được promote cùng manifest/hash/log kết quả.

## 13. Lệnh kiểm tra artifact hiện tại

Từ `/home/ubuntu22/train_mujoco`:

```bash
cd /home/ubuntu22/train_mujoco
cd HB/controller/policies/loco-teleop
sha256sum -c SHA256SUMS
cat MANIFEST.yaml
```

Artifact hiện tại chỉ vượt qua contract/integrity checks. Trước khi có ALMI adapter và
Gate A–F ở trên, không dùng lệnh chọn `flat_model` hiện tại để nạp trực tiếp một trong
các `policy.onnx` bên dưới `loco-teleop/`.

