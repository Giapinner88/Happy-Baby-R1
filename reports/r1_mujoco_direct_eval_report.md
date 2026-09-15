# R1 MuJoCo Direct Eval Investigation Report
**Project:** Unitree - Happy Baby R1
**Date:** 2026-07-08
**Status:** Working finding

Tai lieu nay ghi lai van de policy R1 flat v2 di duoc trong MJLab nhung khong
di dung trong runtime Unitree MuJoCo bridge, cach da test, ket qua so sanh, va
ban sua thu nghiem `--direct-eval`.

Quy tac trong investigation nay:

- Khong sua `third_party/`.
- Robot/runtime asset dung tu workspace:
  `asset/mujoco/unitree_robots/r1/R1.xml`.
- Log, video, CSV, checkpoint nam trong `data/`.
- Policy test la R1 policy, khong map qua G1.

## 1. Tom tat ket luan

Policy v2 da train xong **khong phai nguyen nhan chinh** lam robot khong tien
trong runtime MuJoCo cu.

Bang cung checkpoint flat v2 va cung command `vx=0.3`:

| Duong chay | Ket qua |
| :--- | :--- |
| MJLab native play | Robot di toi dung toc do, `avg_vx` xap xi `0.294 m/s` |
| Unitree MuJoCo bridge cu | Robot dung/re tai cho, `avg_vx` xap xi `-0.020 m/s` |
| MuJoCo direct eval moi | Robot di toi duoc, `avg_vx` xap xi `0.272 m/s` |

Sai khac lon nam o **deploy/runtime actuator path**:

- MJLab train/play dung `BuiltinPositionActuator`: policy output thanh target
  joint position, sau do MuJoCo position actuator tu tinh force theo stiffness,
  damping va force limit.
- Runtime bridge cu dung `policy_runner.py -> DDS LowCmd -> UnitreeSdk2Bridge`
  roi tu tinh torque PD va ghi vao XML `<motor>` actuator.

Khi runtime duoc dua ve gan semantics cua MJLab bang direct eval, robot lap tuc
di dung huong va gan dung toc do command. Dieu nay chi ra bridge/actuator
deployment la noi can sua tiep, khong phai retrain policy ngay.

## 2. Background

Run train flat v2 da hoan thanh:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-07_17-37-04_r1_flat_walk_v2
```

Artifact chinh:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-07_17-37-04_r1_flat_walk_v2/model_10000.pt
data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-07_17-37-04_r1_flat_walk_v2/policy.onnx
```

Symlink deploy mac dinh da tro toi ban v2:

```text
data/models/unitree_mujoco_policy/r1_velocity.onnx
```

Policy config:

- Obs dim: `83`.
- Action dim: `24`.
- Control dt: `0.02 s` / `50 Hz`.
- Action la joint position delta quanh default joint position.
- Command train flat v2 co `lin_vel_x` trong mien tien toi, bao gom
  `0.3 m/s`.

## 3. Trieu chung ban dau

Khi chay ONNX v2 bang runtime bridge cu:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0
```

Robot khong nga, base height va projected gravity kha on dinh, nhung khong di
toi. CSV runtime cu:

```text
data/sim_state_logs/policy_runner_11-37-52_2026-07-08.csv
```

Chi so do duoc:

| Metric | Gia tri |
| :--- | ---: |
| Rows | `710` |
| Stable command window | `cmd_vx > 0.28` |
| Stable window duration | `11.58 s` |
| Displacement `dx` | `-0.2366 m` |
| `avg_vx_from_pos` | `-0.0204 m/s` |
| `mean base_vx` | `-0.0204 m/s` |
| `median base_vx` | `-0.0180 m/s` |
| `base_z` mean | `0.7343 m` |
| `proj_grav_z` mean | `-0.9994` |

Nhan xet: robot can bang duoc nhung gan nhu chi shuffle/re lui nhe, khong co
locomotion tien toi theo command.

## 4. Test doi chung bang MJLab native

Da chay MJLab play native voi checkpoint v2, sau do chay them fixed-command
test `vx=0.3` truc tiep trong MJLab.

Ket qua fixed command:

```text
steps: 0..699
sim dt: 13.98 s
first_pos: [-0.3282, -0.3526, 0.7581]
last_pos:  [3.7821, -0.2233, 0.7198]
dx: +4.1103 m
avg_vx_from_pos: +0.294 m/s
post_mean_vel_b.vx: +0.3048 m/s
post_z mean/min/max: 0.7193 / 0.7186 / 0.7199
any_done: False
```

Video MJLab:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-07_17-37-04_r1_flat_walk_v2/videos/play/rl-video-step-0.mp4
```

Nhan xet: cung checkpoint v2, policy trong MJLab native di duoc va track command
tot. Vay policy/training khong phai loi goc.

## 5. Cac kha nang da loai tru

### 5.1. Sai ONNX

Runtime bridge va direct eval deu load dung:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/2026-07-07_17-37-04_r1_flat_walk_v2/policy.onnx
```

### 5.2. Sai joint order

Joint order MJLab action target trung voi runtime:

```text
left_hip_pitch
left_hip_roll
left_hip_yaw
left_knee
left_ankle_pitch
left_ankle_roll
right_hip_pitch
right_hip_roll
right_hip_yaw
right_knee
right_ankle_pitch
right_ankle_roll
waist_roll
waist_yaw
left_shoulder_pitch
left_shoulder_roll
left_shoulder_yaw
left_elbow
left_wrist_roll
right_shoulder_pitch
right_shoulder_roll
right_shoulder_yaw
right_elbow
right_wrist_roll
```

Runtime XML joint names chi them suffix `_joint`, mapping nay da khop.

### 5.3. Sai action scale

Action scale trong runtime config khop voi `env.yaml` cua training:

| Nhom joint | Scale |
| :--- | ---: |
| Hip/knee/waist | `0.15` |
| Ankle | `0.3125` |
| Shoulder pitch/roll | `0.375` |
| Shoulder yaw/elbow/wrist roll | `0.4125` |

### 5.4. Command ngoai phan phoi

Loi voi command lui `vx=-0.2` co the do ngoai phan phoi train stage dau, nhung
case `vx=+0.3` nam trong mien train. Vi MJLab native track duoc `vx=0.3`, command
khong phai nguyen nhan.

## 6. Khac biet actuator quan trong

### 6.1. MJLab training/play

Trong `training/happy_baby_r1_training/mjlab_r1_robot_cfg.py`, R1 dung:

```text
BuiltinPositionActuatorCfg
```

Theo MJLab implementation:

- Policy output raw action.
- `JointPositionAction` xu ly:

```text
processed_action = raw_action * scale + default_joint_pos
target = processed_action - encoder_bias
set_joint_position_target(target)
```

- `BuiltinPositionActuator` ghi target position vao `ctrl`.
- MuJoCo position actuator tu tinh force theo:

```text
force = stiffness * (ctrl - q) - damping * qd
```

- `ctrllimited = False`.
- `forcelimited = True`.
- Force range gioi han theo effort limit.
- Armature duoc set `0.01`, frictionloss cho actuator path nay la `0.0`.

### 6.2. Runtime bridge cu

Runtime cu co path:

```text
policy_runner.py -> DDS LowCmd -> UnitreeSdk2Bridge.LowCmdHandler -> mj_data.ctrl
```

Trong bridge:

```python
mj_data.ctrl[i] = (
    tau
    + kp * (q_cmd - sensordata_pos)
    + kd * (dq_cmd - sensordata_vel)
)
```

Trong XML runtime, actuators la `<motor>` torque actuators:

```xml
<motor name="left_hip_pitch" joint="left_hip_pitch_joint" ctrlrange="-60 60" />
```

Vay policy action target da bi bien thanh torque o tang bridge, thay vi duoc
apply nhu position target cua MJLab. Sai khac nay du lon de lam gait hoc trong
MJLab khong trien khai dung trong runtime bridge.

## 7. Ban sua thu nghiem: `--direct-eval`

Da them mode direct eval vao:

```text
scripts/run_unitree_mujoco_policy.py
sim/unitree_mujoco_policy/unitree_mujoco2.py
```

Khi chay:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0
```

Launcher khong start `policy_runner.py`. Simulator tu:

1. Load ONNX policy.
2. Patch 24 XML actuators trong RAM tu motor torque semantics sang MJLab-style
   position actuator semantics.
3. Doc state truc tiep tu `mj_data`.
4. Build observation giong policy runner:
   - base angular velocity body frame
   - projected gravity
   - velocity command
   - gait phase
   - joint position relative
   - joint velocity relative
   - last action
5. Chay ONNX moi `0.02 s`.
6. Ghi `target_q` truc tiep vao `mj_data.ctrl`.
7. Ghi CSV vao `data/sim_state_logs`.
8. Neu bat `--record-video`, xuat mp4 vao `data/runs/unitree_mujoco_policy`.

Patch actuator chi dien ra trong `MjModel` runtime, khong sua XML tren disk.

## 8. Ket qua direct eval

### 8.1. Smoke test ngan

Command:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 4 \
  --cmd-vx 0.3 \
  --cmd-start 0.5 \
  --cmd-ramp 0.5
```

CSV:

```text
data/sim_state_logs/unitree_mujoco2_11-59-46_2026-07-08.csv
```

Ket qua:

| Metric | Gia tri |
| :--- | ---: |
| Rows | `188` |
| Total `dx` | `+0.6807 m` |
| Stable-window `dx` | `+0.6618 m` |
| Stable-window duration | `2.44 s` |
| Stable `avg_vx_from_pos` | `+0.2712 m/s` |
| Stable `mean base_vx` | `+0.2716 m/s` |
| `base_z` mean | `0.7306 m` |
| `proj_grav_z` mean | `-0.9993` |

### 8.2. Direct eval dai hon, khong video

Command:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0
```

CSV:

```text
data/sim_state_logs/unitree_mujoco2_12-00-45_2026-07-08.csv
```

Ket qua:

| Metric | Gia tri |
| :--- | ---: |
| Rows | `692` |
| Sim time | `0.00..13.82 s` |
| Total `dx` | `+3.2122 m` |
| Stable command window | `cmd_vx > 0.28` |
| Stable window duration | `11.68 s` |
| Stable-window `dx` | `+3.1716 m` |
| `avg_vx_from_pos` | `+0.2715 m/s` |
| `mean base_vx` | `+0.2721 m/s` |
| `median base_vx` | `+0.2752 m/s` |
| `cmd_vx` mean/min/max | `0.2997 / 0.2815 / 0.3000` |
| `base_z` min/mean/last | `0.7254 / 0.7301 / 0.7279 m` |
| `proj_grav_z` min/mean/last | `-1.0000 / -0.9995 / -0.9998` |

### 8.3. Direct eval co video

Command:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0 \
  --record-video \
  --video-width 1280 \
  --video-height 720 \
  --video-fps 50 \
  --video-codec h264
```

Video:

```text
data/runs/unitree_mujoco_policy/2026-07-08_12-00-19/mujoco_policy.mp4
```

CSV:

```text
data/sim_state_logs/unitree_mujoco2_12-00-20_2026-07-08.csv
```

Ghi chu: render video headless lam sim cham hon wall-time tren may hien tai, nen
run video 14 giay wall-time chi sinh `92` policy rows, tuong duong khoang
`1.82 s` sim-time. Dung video de xem pipeline visual, dung run khong video de
lay metric locomotion chuan.

## 9. Bang so sanh tong hop

| Test | Path CSV/video | Stable `dx` | Stable duration | `avg_vx_from_pos` | Nhan xet |
| :--- | :--- | ---: | ---: | ---: | :--- |
| MJLab native fixed `vx=0.3` | MJLab play/ad-hoc metric | `+4.1103 m` | `13.98 s` | `+0.294 m/s` | Policy tot trong env train |
| Runtime bridge cu `vx=0.3` | `policy_runner_11-37-52_2026-07-08.csv` | `-0.2366 m` | `11.58 s` | `-0.0204 m/s` | Khong locomotion dung |
| Runtime direct eval `vx=0.3` | `unitree_mujoco2_12-00-45_2026-07-08.csv` | `+3.1716 m` | `11.68 s` | `+0.2715 m/s` | Locomotion da dung huong |

## 10. Giai thich nguyen nhan

Policy flat v2 hoc trong MJLab voi gia dinh:

```text
action -> target joint position -> MuJoCo position actuator -> bounded actuator force
```

Runtime bridge cu thuc thi:

```text
action -> target joint position -> external PD torque calculation -> MuJoCo motor actuator
```

Hai pipeline nay nhin ben ngoai deu co `kp/kd`, nhung khong dong nhat:

- Loai actuator khac nhau: position actuator vs motor torque actuator.
- Noi gioi han khac nhau: MJLab gioi han force actuator; bridge dua torque vao
  motor ctrlrange.
- `ctrl` co y nghia khac nhau: target position trong MJLab, torque trong bridge.
- Runtime bridge di qua DDS/threading/publish-subscribe, co them timing va
  sensor feedback path.
- MJLab set mot so thuoc tinh actuator/joint trong spec, nhu armature/friction
  theo actuator config.

Vay khi deploy bang bridge cu, policy dang bi dua vao mot plant khac voi plant
ma no da hoc. Ket qua la robot van giu than on dinh duoc nhung gait khong sinh
ra tich luy chuyen vi tien toi.

## 11. Trang thai code hien tai

### 11.1. Launcher

File:

```text
scripts/run_unitree_mujoco_policy.py
```

Them flag:

```text
--direct-eval
```

Khi flag bat:

- Khong start process `policy_runner.py`.
- Start moi `unitree_mujoco2.py`.
- Set env:

```text
POLICY_DIRECT_EVAL=1
```

### 11.2. Simulator

File:

```text
sim/unitree_mujoco_policy/unitree_mujoco2.py
```

Them cac thanh phan chinh:

- `patch_mjlab_position_actuators(model)`
- `DirectMjlabPolicyController`
- Direct ONNX inference loop trong `SimulationThread`
- CSV logging bang `SimStateLogger`
- Video recorder van dung chung voi runtime

### 11.3. Verification da chay

```text
conda run --no-capture-output -n r1_env python -m py_compile \
  scripts/run_unitree_mujoco_policy.py \
  sim/unitree_mujoco_policy/unitree_mujoco2.py

git diff --check
```

Ket qua: OK.

## 12. Cach chay lai

### 12.1. Direct eval khong video

Dung de lay metric locomotion:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0
```

### 12.2. Direct eval co video

Dung de xem hanh vi:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --direct-eval \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0 \
  --record-video \
  --video-codec h264 \
  --video-width 1280 \
  --video-height 720 \
  --video-fps 50
```

### 12.3. Bridge cu de doi chung

Dung khi can so sanh:

```bash
conda run --no-capture-output -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --scene scene.xml \
  --duration 14 \
  --cmd-vx 0.3 \
  --cmd-start 1.0 \
  --cmd-ramp 1.0
```

## 13. Han che cua direct eval hien tai

Direct eval la mode debug/deploy gan MJLab, chua phai cau tra loi cuoi cung cho
robot that:

- No bypass DDS/LowCmd, nen khong kiem chung full Unitree SDK bridge.
- No patch actuator trong `MjModel` runtime, khong tao XML rieng de inspect bang
  tool ngoai.
- Video headless dang lam sim cham; can toi uu neu muon long video/eval batch.
- Direct eval chi nen dung scripted command de ket qua doc CSV/reproduce duoc.
  Manual keyboard/gamepad control da bi loai khoi direct eval vi no lam phep do
  phu thuoc vao nguoi dieu khien va de lam policy mat on dinh khi doi huong lien tuc.
- Van can doi chieu them sensor frame/timing neu sau nay dua lai qua DDS bridge.

## 14. Huong di tiep theo

Sau loi nay, thu tu uu tien la:

1. Train lai flat v2 dua tren log/CSV da thu duoc, muc tieu gait nhanh hon,
   nhe hon va on dinh hon khi command thay doi.
2. Sua bridge de chay duoc trong MuJoCo theo cung policy/eval. Chi khi bridge
   simulation di duoc moi coi deploy path du tin cay de nghien cuu noi xuong
   he that.

De runtime bridge cu co hanh vi gan policy train, co hai huong ky thuat:

### Huong A: giu direct eval lam runtime simulation chinh

Dung `--direct-eval` cho local evaluation, video, terrain playground, va policy
debug. Day la duong ngan nhat de xem policy co tot khong trong MuJoCo workspace.

Can lam tiep:

- Them log/contact/foot diagnostics.
- Cho `scene_playground.xml` va cac terrain khac chay regression.
- Toi uu video recorder hoac chay video theo sim-time thay vi wall-time.

### Huong B: sua bridge de match MJLab semantics

Neu muc tieu la deploy qua Unitree-style lowcmd bridge, can lam bridge khop voi
policy training:

- Xac dinh ro low-level robot that nhan position target hay torque target.
- Neu sim bridge de test policy, can tao mode position-actuator thay vi motor
  torque actuator.
- Neu bat buoc dung torque bridge, can retrain/domain-randomize theo torque
  actuator pipeline do.
- So sanh per-step `target_q`, `q`, `dq`, `ctrl/torque`, contact force giua
  MJLab native va bridge.
- Viet regression test: same ONNX, same command `vx=0.3`, pass neu
  `avg_vx_from_pos > 0.2 m/s` trong 10 s stable command.

Khuyen nghi hien tai: giu `--direct-eval` de danh gia policy bang scripted
commands va terrain trong workspace. Sau do sua bridge cu nhu mot deployment
target rieng; neu bridge simulation chua pass thi chua dua policy xuong he that.

## 15. Resolution (2026-07-10): bridge da PASS

Muc 13-14 da duoc giai quyet. Bridge DDS gio di duoc gan het nhu direct eval
voi cung ONNX (flat_v2), cung command `vx=0.3`.

### 15.1. Root cause thuc su (khac gia thuyet actuator semantics)

Da do va loai tru tung yeu to: PD gains / action_scale / thu tu obs / obs scale
deu khop train; gyro va projected_gravity cua bridge trung khop direct eval ve
frame/dau (diff = 0, IMU site = base); actuator position mode ghi target dung.

Hai bug that su nam trong duong policy_runner (bridge), khong phai actuator:

1. **`gait_phase` sai** — clock bi ramp/snap + nhan `gait_scale`, khac ham
   `phase(period=0.6)` luc train (clock chay tu do, bat/tat cung theo
   `||cmd|| < 0.1`, bien do luon 1.0). Feed phase out-of-distribution khi bat
   lenh -> nga. Da sua trong `policy_runner.py`.
2. **`last_action` khong bao gio cap nhat** — luon = 0, lam hong 24 obs cuoi.
   Direct eval co `self.last_action = action.copy()`, bridge thi thieu. Day la
   manh ghep cuoi. Da them `last_action = action.copy()` sau khi apply target.

Ca hai bridge mode (position va torque) deu fail nhu nhau truoc khi sua va deu
pass sau khi sua -> khang dinh actuator semantics chua bao gio la nguyen nhan.

### 15.2. Ket qua regression (pass tieu chi avg_vx > 0.2 m/s trong ~12 s)

| Duong chay              | dx     | avg base_vx | Nga? |
|-------------------------|--------|-------------|------|
| direct-eval (baseline)  | +3.21m | +0.271 m/s  | Khong |
| bridge position + fix   | +3.59m | +0.272 m/s  | Khong |
| bridge torque + fix     | +3.67m | +0.278 m/s  | Khong |

### 15.3. Loai tru latency thuan

Them `POLICY_OBS_DELAY_STEPS` (env-gate, mac dinh 0) vao direct eval de gia lap
tre DDS. Delay 1-3 step (20-60 ms) khong lam nga (dx van +2.5m, dung thang) ->
policy chiu duoc latency; nga la do obs sai, khong phai tre.

### 15.4. Bug van hanh: orphan process (da sua)

Launcher chay children qua `conda run`; `proc.terminate()` cu chi bao SIGTERM
cho wrapper -> `python` con mo coi, van publish `rt/lowcmd` -> nhieu loan cac lan
chay sau (chinh la artifact "giam tai cho" gia luc dau). Da sua trong
`scripts/run_unitree_mujoco_policy.py`: `start_new_session=True` cho moi child,
`terminate()` `killpg` ca nhom, them signal handler SIGTERM/SIGINT + `atexit`.
Kiem chung: SIGTERM / SIGINT / thoat binh thuong deu con 0 orphan.

## 16. Safety gate audit (2026-07-10)

Sau khi bridge pass, da kiem tra cac safety gate (ap dung cho ca hai duong).

### 16.1. Gate hoat dong (da test)

| Gate               | Env / flag                     | Mac dinh              | Ket qua test |
|--------------------|--------------------------------|-----------------------|--------------|
| Command clamp      | `POLICY_CMD_LIMIT_*` / preset  | conservative (0.35,0.15,0.6) | vx=1.0 -> clamp 0.35 |
| Command slew       | `POLICY_CMD_SLEW_*`            | (0.6,0.35,1.2)        | peak slew do duoc 0.600 = limit |
| Target rate limit  | `POLICY_TARGET_RATE_LIMIT`     | 4.0 rad/s             | active (log xac nhan) |
| Fall guard         | `POLICY_FALL_GUARD_GRAVITY_Z`  | -0.55                 | trigger -> giu DEFAULT_Q |
| Warmup (FixStand)  | `POLICY_WARMUP_SECONDS`        | 0 (tat)               | chi bridge co |
| Action fade        | `POLICY_FADE_SECONDS`          | 0 (tat)               | ca hai duong |

Config safety in ra luc khoi dong o ca launcher (`Safety: preset=...`) lan policy
(`[policy] safety cmd_limit=... target_rate_limit=... fall_guard_gravity_z=...`).

### 16.2. Gap can luu y truoc khi xuong robot that

1. **`action_clip` mac dinh = 0 (TAT).** Raw ONNX action tung cham +-2.6. Hien
   chi co target_rate_limit chan o ha nguon; thieu tuyen phong thu dau. Nen bat
   (vd clip ~4-6) cho robot that.
2. **Khong co warmup mac dinh; direct-eval thieu warmup hoan toan.** Robot that
   can FixStand ramp luc bat policy de tranh giat.
3. **Chua co emergency stop tu ben ngoai.** Chi co fall guard (tu dong), phim ESC
   (can cua so pygame focus), va kill process. Robot that can them kenh DDS kill /
   e-stop phan cung.
4. **Fall guard "giu DEFAULT_Q" thay vi damping.** Trong sim chong runaway tot;
   tren robot that, giu pose dung khi dang nga co the "cai" lai cu nga -> can xem
   lai failure mode mong muon.

Cac gate cot loi (clamp / slew / rate-limit / fall-guard) du dung cho sim.
