# Mimic mất thăng bằng trên robot thật (sim2real gap)

> **Trạng thái: đã sửa xong phần code. CHƯA test trên robot lần nào.**
> Blocker: cây code không build được vì tính năng ngồi ghế đang làm dở (mục 8).
> Ngày rà soát: 2026-07-14.

## 1. Hiện tượng

Policy mimic chạy **mượt và rất thăng bằng trên MuJoCo (C++)**, nhưng ra robot thật thì dễ mất
thăng bằng, nhiều động tác khiến robot **phải bước lùi ra sau** để bắt lại thăng bằng (trong sim
không hề có). Policy flat (locomotion) thì chạy tốt trên robot thật.

Sự bất đối xứng "flat sống — mimic chết" chính là manh mối dẫn tới bug Kd ở mục 2.1.

---

## 2. ĐÃ THAY ĐỔI — 9 file

### 2.1 `src/config/RobotSpec.hpp` — sửa `kKdTrain` (bug chính)

Từ `2.0` ở cả 24 khớp → `3,3,3,3,2,2 | 3,3,3,3,2,2 | 3,3 | 2×10`.

`r1_constants.py` (bên train) đổi damping **2.0 → 3.0** cho hip/knee/waist ngày **2026-07-09**
(comment: *"Raised from 2.0: less under-damped"*). `kKdTrain` phía deploy vẫn giữ 2.0 — giá trị của
thời kỳ trước đó, bị bỏ quên. → mimic chạy với chân **under-damped 33%** so với lúc train. Sim lý
tưởng không lộ ra (không có trễ để mất ổn định); robot thật thì thiếu damping + trễ truyền động =
dao động nảy → **bước lùi**.

**Vì sao flat không dính:** flat **có metadata** trong ONNX, mà `PolicyController::Init()` ưu tiên
metadata và ghi đè fallback → flat miễn nhiễm. Mimic không có metadata → rơi vào fallback sai.

### 2.2 Bốn file `policies/dance/*/*.onnx` — gắn metadata

`doremon`, `lacmong1`, `lacmong2`, `pokemon`: trước đây **trống hoàn toàn**, nay mang đủ
`joint_names`, `joint_stiffness`, `joint_damping`, `default_joint_pos`, `action_scale`, cộng key
`metadata_note` ghi rõ đây là giá trị **backfill**, không phải do run train sinh ra.

Giá trị lấy bằng cách tái tạo logic `get_base_metadata()` của mjlab (đọc từ `r1_constants.py` theo
thứ tự khớp XML). **Đã kiểm chứng:** dùng chính logic đó sinh lại metadata của **flat** (file có
metadata thật) → trùng khít từng ký tự cả 5 key. Trọng số không bị đụng: inference trước/sau khi gắn
cho output **giống hệt** (`max|Δ| = 0`).

Sau bước này 4 policy dance **tự khai báo gains của chính nó**; mục 2.1 chỉ còn là lưới an toàn.

### 2.3 `src/policy/PolicyController.hpp` — fail loud

Thiếu bất kỳ khoá metadata nào trong 4 khoá bắt buộc → **in lỗi to và từ chối chạy policy đó**
(throw), thay vì im lặng dùng fallback. Chính cái im lặng đó đã giấu bug 2.1.

### 2.4 `unitree_rl_mjlab/src/tasks/tracking/rl/runner.py` — vá export

`save()` export **hai** file — một file nhúng motion data, và một `policy.onnx` thuần — nhưng chỉ gắn
metadata cho file thứ nhất. `policy.onnx` (file **duy nhất đem đi deploy**) không được gắn gì. Runner
của velocity thì gắn đúng → nên flat có, mimic không.

Lỗi cấu trúc có sẵn trong upstream: quét 60+ file `.onnx` trong workspace thì **mọi** policy mimic
(129/154/219 chiều, kể cả bản G1 mẫu của Unitree) đều trống metadata, còn **mọi** policy
flat/velocity (83/98/270) đều có. Đã thêm 1 dòng gắn cho `policy.onnx`.

### 2.5 `unitree_rl_mjlab/.../tracking_env_cfg.py` + `config/r1/env_cfgs.py` — DR (fix gốc)

Task tracking trước đó **gần như không có DR nào liên quan sim2real**: chỉ `push_robot`, `base_com`
(chỉ torso), `encoder_bias`, `foot_friction`. Đã thêm:

| DR | Dải | Chống lại |
|---|---|---|
| `pd_gains` | kp, kd ×0.8–1.2 | gains motor thật lệch — **đúng loại lỗi ở 2.1** |
| `effort_limits` | ×0.7–1.0 (chỉ giảm) | motor thật không bao giờ khoẻ hơn spec; ép policy không dựa vào moment đỉnh |
| `joint_armature` | 0.005–0.02 (nominal 0.01) | quán tính rotor |
| `joint_friction` | 0.0–0.2 N·m | ma sát khô hộp số (XML hiện = 0) |
| `pseudo_inertia` | alpha ±0.0477 → **khối lượng ±10%** | pin, PC2, vỏ. Dùng `pseudo_inertia` chứ **không** dùng `dr.body_mass` vì cái đó không đổi tensor quán tính |
| trễ actuator | `delay_max_lag=3` = **0–15 ms** | robot thật có trễ bus; sim và train đều bằng 0 |

Trễ đặt trong `config/r1/env_cfgs.py` bằng `dataclasses.replace` (vì `R1_ARTICULATION` là hằng dùng
chung với task velocity — policy flat đang chạy tốt, sửa tại chỗ sẽ rò sang đó).

⚠️ **Chưa chạy thử được.** mjlab **không nằm trong repo này** và không có venv nào ở đây cài nó — máy
train mới có. Tôi đã đối chiếu chữ ký hàm với source mjlab tìm được trên máy, nhưng nếu **máy train
dùng mjlab version khác** thì mấy hàm này có thể không tồn tại. Chạy 2 lệnh này trên máy train trước
khi train:

```bash
python -c "from mjlab.envs.mdp import dr; print([hasattr(dr,n) for n in ('pd_gains','effort_limits','joint_armature','joint_friction','pseudo_inertia')])"
python -c "from mjlab.actuator import BuiltinPositionActuatorCfg as C; import dataclasses; print([f.name for f in dataclasses.fields(C) if 'delay' in f.name])"
```
Kỳ vọng: `[True, True, True, True, True]` và list có `delay_min_lag`/`delay_max_lag`.

### 2.6 `tools/dds_probe.cpp` + `CMakeLists.txt` — công cụ đo trên robot (C2 + C3)

**Chỉ nghe DDS, không gửi gì** → an toàn chạy song song `run_r1` lúc robot đang nhảy. Bắt cả
`rt/lowcmd` (q_des, kp, kd) lẫn `rt/lowstate` (q, dq, tau_est) rồi đối chiếu.

Cách dùng — hai terminal:

```bash
# Terminal 1: ssh robot -> ./run_r1  (đứng dậy, sẵn sàng bấm điệu nhảy)
# Terminal 2 (chạy từ máy dev):
./scripts/probe_robot.sh
```

`probe_robot.sh` tự dò IP robot, build `dds_probe`, chạy nó, và kéo CSV về `logs/probe/`.
Card mạng lấy thẳng từ `network_interface` trong `tuning.yaml` nên luôn khớp với `run_r1`.
Cho robot nhảy trọn 1 điệu rồi Ctrl-C để lấy báo cáo.

Báo cáo 3 thứ:
1. **Nhịp vòng điều khiển** — chu kỳ lowcmd/lowstate p50/p99/max. Jitter lớn = trượt nhịp, mà train
   giả định policy step đúng 20 ms.
2. **Từng khớp** — sai số bám lệnh (rms/max), **trễ vòng kín** (ms), moment lớn nhất, **% thời gian
   bão hoà** (so với effort limit lúc train). Nghi phạm số 1: `L/R_ank_pitch` (Kp=40, limit 50 N·m,
   robot 30 kg).
3. **Nhiễu dq** — so với ±0.5 rad/s mà train giả định. Đây là thứ quyết định có nên bật LPF không
   (mục 7).

**Đã verify:** bơm dữ liệu giả có trễ đúng 12 ms và 1 khớp ở 96% giới hạn moment → probe báo đúng
`12.0 ms` ở mọi khớp và bắt đúng **chỉ mình** khớp bị bão hoà.

### 2.7 `unitree_mujoco/simulate/` — sim lệch có chủ đích (F4)

`src/param.h`, `src/unitree_sdk2_bridge.h`, `config.yaml`. Ba nút mới, **mặc định = 0/1.0/1.0 nên
hành vi y hệt trước**:

```yaml
sim2real_delay_ms: 0      # trễ lệnh xuống motor (ms).       Nghiệm thu: 15
sim2real_gain_scale: 1.0  # gains motor thật / gains gửi.    Nghiệm thu: 0.8
sim2real_mass_scale: 1.0  # khối lượng+quán tính / CAD.      Nghiệm thu: 1.1
```

Lý do phải có: xem mục 3.

---

## 3. Sim hiện tại KHÔNG dùng để nghiệm thu được

`unitree_mujoco/unitree_robots/r1/xmls/r1.xml` và XML lúc train là **cùng một model**: 27 body, cùng
tổng khối lượng **30.182 kg**, IMU cùng ở pelvis.

Nên "chạy mượt trên MuJoCo" chỉ xác nhận: mapping khớp đúng, obs nối đúng dây, gains truyền đúng chỗ.
Nó **không thể** lộ ra bất kỳ gap sim-to-real nào:

| Robot thật có | Sim & train có |
|---|---|
| trễ truyền động ~5–15 ms | 0 |
| ma sát khô, backlash hộp số | 0 |
| khối lượng/COM thật (pin, PC2, vỏ) | số CAD chính xác tuyệt đối |
| motor bão hoà moment | effort limit lý tưởng |

→ **Đừng nghiệm thu bản sửa Kd bằng MuJoCo mặc định.** Bật 3 tham số ở 2.7 thì sim mới có ý nghĩa.

## 4. Đã verify là ĐÚNG — không cần đào lại

| Hạng mục | Kết quả |
|---|---|
| Layout obs 129 chiều | ✅ khớp `actor` terms khi `has_state_estimation=False`: command(48) + anchor_ori(6) + gyro(3) + q_rel(24) + dq(24) + last_action(24) |
| Biểu diễn 6D rotation | ✅ train lấy 2 cột đầu của `R_robot^T·R_ref`; C++ dùng `(init*ref).conj()*real` rồi `.transpose()` → cùng kết quả |
| Ghép quat torso | ✅ `pelvis_imu ⊗ Rx(waist_roll) ⊗ Rz(waist_yaw)` đúng thứ tự chuỗi động học XML |
| Index torso trong npz | ✅ = 14 (npz có 26 body, đã bỏ world) |
| `action_scale` | ✅ = 0.25·effort/stiffness, khớp cả 24 khớp |
| `default_joint_pos` | ✅ khớp HOME_KEYFRAME |
| Nhịp phát clip | ✅ npz fps=50 = policy 50Hz, phase tiến đúng 1 frame/policy step |
| Vị trí IMU | ✅ pelvis ở cả train, sim lẫn robot thật |
| Soft-start (`mimic_warmup_s`) | ✅ đã xử lý đúng vấn đề RSI — xem `PLAN_mimic_soft_start.md` |

**Pipeline observation của mimic không có lỗi.**

---

## 5. VIỆC CẦN CHECK

| # | Việc | Vì sao |
|---|---|---|
| **C1** | Mở wandb (hoặc máy đã train) của 4 run dance, đọc config `damping` | **Quan trọng nhất, làm TRƯỚC khi test robot.** Con số 3.0 là **suy gián tiếp** (mốc thời gian: `r1_constants.py` sửa 09/07, 4 file dance mtime 10–12/07, cùng đợt với các flat mang 3.0). Export đã làm mất metadata nên không đọc ngược được từ file. Nếu hoá ra là 2.0 → phải **revert 2.1 và 2.2** |
| **C2** | Chạy `dds_probe` lúc robot nhảy | Xem khớp nào bão hoà / trễ nhiều. Công cụ đã sẵn sàng (2.6) |
| **C3** | Đọc mục "nhịp vòng điều khiển" của `dds_probe` | Lấy số trễ thật → thay dải ước lượng 0–15 ms ở 2.5 và `sim2real_delay_ms` ở 2.7 |
| **C4** | Cân + đo COM robot thật, so với XML (30.182 kg) | Lệch nhiều thì model train sai từ gốc → sửa XML (F3) |
| **C5** | Chạy 2 lệnh check mjlab ở mục 2.5 trên máy train | Xác nhận API DR tồn tại trước khi train |

## 6. VIỆC CẦN LÀM TIẾP

| # | Việc | Ưu tiên |
|---|---|---|
| **F1** | Sửa xong tính năng ngồi ghế (mục 8) → build → test Kd mới trên robot | chặn mọi thứ |
| **F2** | Nếu vẫn nảy: quét `policy_kd_scale` 1.0 → 1.2 → 1.35 → 1.5, **giữ `policy_kp_scale: 1.0`** | sau F1 |
| **F3** | Retrain với DR (2.5) sau khi có số đo từ C2/C3/C4 | **fix gốc** |
| **F4** | Nghiệm thu policy mới bằng sim lệch (2.7) trước khi đưa lên robot | cùng F3 |

### Vì sao F2 vặn Kd chứ không vặn Kp

- Tăng **Kd** thường **an toàn**: thêm cản, dập dao động — đúng thứ cần để bù độ trễ thật.
- Tăng **Kp** **rủi ro**: vừa khuếch đại dao động do trễ, vừa **dịch điểm cân bằng tĩnh** của mọi khớp
  (Kp cao → khớp ít võng dưới trọng lực, mà policy học cách bù đúng độ võng của Kp=100). Vặn Kp là
  đẩy robot ra xa điều kiện lúc train.

`policy_kp_scale`/`policy_kd_scale` **nhân lên sau** khi đọc metadata nên vẫn vặn được bình thường.
Nhưng phải **restart `run_r1`** (Init chỉ chạy 1 lần), và chỉ có **một hệ số chung cho cả 24 khớp**.

## 7. Bẫy — đừng làm

**Đừng bật `joint_vel_lpf_hz` / `imu_gyro_lpf_hz` để chữa mất thăng bằng.** Hai bộ lọc này **không**
ảnh hưởng tới moment: phần damping do board motor tính bằng tốc độ khớp **do chính motor đo**, ở tần
số cao — code mình không đụng vào (`LowCmdSender` gửi `dq_des = 0`). Chúng chỉ lọc `state_.dq` và
`state_.gyro`, tức chỉ đổi thứ **policy nhìn thấy**.

Mà lúc train đã cộng sẵn nhiễu ±0.5 rad/s vào `joint_vel` và ±0.2 vào `ang_vel` → **policy vốn đã
quen nhiễu**. Bật lọc thì loại đi thứ nó không sợ, nhưng **thêm độ trễ pha** (lọc bậc 1 ở 15 Hz ≈
+10 ms) — thứ nó chưa từng thấy, và đúng là thứ đang giết nó.

→ Chỉ bật nếu mục 3 của `dds_probe` cho thấy nhiễu dq thật sự vượt ±0.5 rad/s.

## 8. Blocker hiện tại

`src/app/Application.cpp` (phần ngồi ghế) gọi 8 trường không tồn tại trong `Tuning`:

```
sit_spread, sit_hip_deg, sit_lean_deg, sit_seated_lean_deg,
sit_settle_time_s, sit_arm_forward, sit_arm_elbow, sit_ankle_gravity_gain
```

→ **`run_r1` không build được**, nên bản sửa Kd chưa test được lần nào. Đây là việc đang làm dở của
tính năng ngồi ghế (Application.cpp đã lên bản mới, `Tuning.hpp`/`tuning.yaml` chưa theo), **không
liên quan sim2real**. Chủ repo tự bổ sung sau.

`dds_probe` **không** dính blocker này (target riêng, không phụ thuộc `Application.cpp`) — build và
chạy được ngay.
