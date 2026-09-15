# R1 MJLab Training Reading Guide
**Project:** Unitree - Happy Baby R1
**Status:** Draft / Working

Tai lieu nay la ban doc tong hop cho pipeline train R1 bang
`unitree_rl_mjlab`, sau do kiem chung/deploy bang `unitree_mujoco` hoac runtime
noi bo cua Happy-Baby-R1. Muc tieu la hieu ro train cai gi, train nhu the nao,
cau truc file nam o dau, moi truong train la gi, policy sinh ra dung tiep ra
sao, va workspace nay chuan hoa artifact nhu the nao.

Quy tac nen nho truoc tien:

- `third_party/unitree_rl_mjlab` va `third_party/unitree_mujoco` la repo goc de
  tham chieu/call. Khong sua truc tiep source vendor khi lam pipeline noi bo.
- Moi ket qua thuc nghiem cua Happy-Baby-R1 di vao `data/`.
- Host env `r1_env` dung cho MJLab/MuJoCo. IsaacLab de Docker xu ly sau.

## 1. Ban do tong the

Pipeline MJLab cho R1 co bon lop:

| Lop | Duong dan | Vai tro |
| :--- | :--- | :--- |
| Robot asset train | `third_party/unitree_rl_mjlab/src/assets/robots/unitree_r1/` | MJCF R1, mesh, actuator, collision, default pose |
| Task RL | `third_party/unitree_rl_mjlab/src/tasks/velocity/` | Dinh nghia env, observation, action, command, reward, termination, curriculum |
| Script train/play | `third_party/unitree_rl_mjlab/scripts/train.py`, `scripts/play.py` | Entrypoint cua upstream Unitree RL MJLab |
| Runtime/deploy | `third_party/unitree_rl_mjlab/deploy/robots/r1/`, `third_party/unitree_mujoco/`, `sim/unitree_mujoco_policy/` | Chay policy ONNX qua DDS/MuJoCo de kiem chung gan voi robot that |

Trong workspace nay, khong chay truc tiep de ghi log vao vendor tree. Dung
wrapper:

```bash
python scripts/training/r1_policy_workspace.py train mjlab --terrain flat --num-envs 4096
python scripts/training/r1_policy_workspace.py collect mjlab
```

Wrapper chay script goc cua Unitree nhung dat working directory vao
`data/runs/mjlab`, nen checkpoint/log/export nam duoi `data/`.

## 2. Train cai gi?

Baseline hien tai la **R1 velocity tracking policy**:

- Task ID flat: `Unitree-R1-Flat`
- Task ID rough: `Unitree-R1-Rough`
- Experiment name: `r1_velocity`
- Algorithm: PPO qua `rsl_rl`
- Policy output: joint position action cho cac joint R1
- Deploy artifact chinh: `policy.onnx`

Y nghia cua task: policy nhan lenh van toc mong muon `twist`
`(lin_vel_x, lin_vel_y, ang_vel_z/heading)` va hoc dieu khien vi tri joint de R1
dung/di theo lenh do ma van giu than on dinh, chan tiep dat hop ly, han che
truot chan, va chiu duoc nhieu randomization co ban.

Day khong phai motion imitation. Motion imitation trong upstream dung nhom task
`Tracking-No-State-Estimation` va motion `.npz`; pipeline do co the them sau,
nhung baseline R1 hien tai trong workspace la velocity locomotion.

## 3. R1 robot model trong train

File quan trong:

- `src/assets/robots/unitree_r1/xmls/r1.xml`
- `src/assets/robots/unitree_r1/r1_constants.py`

`r1_constants.py` lam cac viec chinh:

- Load MJCF R1 bang `mujoco.MjSpec`.
- Goi `update_assets(...)` de nap mesh vao spec.
- Dinh nghia actuator theo nhom: leg, ankle, waist, arm, wrist.
- Dinh nghia home pose:
  - base cao khoang `z=0.76`
  - hip pitch `-0.1`
  - knee `0.3`
  - ankle pitch `-0.2`
  - shoulder/elbow o tu the FixStand tuong ung.
- Dinh nghia collision:
  - foot collision co `condim=3`, friction mac dinh `0.6`
  - self collision duoc bat trong baseline.
- Tao `R1_ACTION_SCALE` tu effort/stiffness cua actuator.

Action trong task la `JointPositionActionCfg` voi `use_default_offset=True`.
Nghia la policy khong xuat torque truc tiep; policy xuat delta/action quanh
default joint position, sau do moi truong bien thanh target position cho
actuator PD.

## 4. Moi truong train

Factory goc nam o:

```text
third_party/unitree_rl_mjlab/src/tasks/velocity/velocity_env_cfg.py
```

R1 override nam o:

```text
third_party/unitree_rl_mjlab/src/tasks/velocity/config/r1/env_cfgs.py
```

Cac thong so cot loi:

| Hang muc | Gia tri/hieu ung |
| :--- | :--- |
| Physics backend | MuJoCo/MuJoCo Warp qua MJLab |
| Timestep MuJoCo | `0.005` s |
| Decimation | `4` |
| RL step | `0.02` s, tuong duong 50 Hz |
| Episode length | `20.0` s |
| Default train env count | upstream default `1`, khi train that thuong override `4096` |
| Rough terrain | Terrain generator + raycast height scan + curriculum |
| Flat terrain | Plane, bo `terrain_scan` va `height_scan` |

`Unitree-R1-Rough` dung terrain generator va curriculum dia hinh.
`Unitree-R1-Flat` go raycast/height scan de train tren mat phang. Workspace mac
dinh dung flat cho smoke va baseline dau tien vi de debug hon.

### 4.1. Hinh dung ve `num_envs`

`--num-envs` la so ban sao moi truong/robot chay song song trong cung mot job
train. Neu chay:

```bash
python scripts/training/r1_policy_workspace.py train mjlab --terrain rough --num-envs 1000
```

thi khong phai mot con R1 duy nhat di 1000 lan lien tiep. Dung hon la 1000 ban
sao R1 cung dang duoc sim tren GPU. Moi ban sao co episode rieng, command rieng,
reset rieng, randomization rieng, va co the dung o vi tri/dia hinh khac nhau
trong terrain grid. PPO thu thap rollout tu tat ca ban sao nay roi cap nhat
cung mot policy.

Mot iteration PPO voi R1 config hien tai thuong gom:

```text
num_envs * num_steps_per_env = num_envs * 24 samples
```

Vi du:

| `num_envs` | Samples moi iteration | Cach dung |
| ---: | ---: | :--- |
| `1` | `24` | Smoke test pipeline, bat loi import/config/export |
| `16` den `128` | `384` den `3072` | Debug reward/observation nhanh hon, it ton VRAM |
| `512` den `1000` | `12288` den `24000` | Train thu nghiem vua phai, xem policy co hoc dung huong khong |
| `4096` | `98304` | Baseline train GPU lon theo style upstream |

Noi theo cach ban hinh dung:

- `--num-envs 1000 --terrain rough`: gan nhu cho 1000 con R1 cung luc tap di
  tren nhieu mien dia hinh; co con o flat, co con o bac thang, co con o doc,
  co con o nen go ghe hoac song nhe.
- `--num-envs 100 --terrain flat`: 100 con cung luc tap di/chay/quay tren mat
  phang, moi con nhan command van toc khac nhau theo thoi gian.
- `--num-envs 1 --max-iterations 1`: chi la test xem may co chay duoc pipeline,
  khong co y nghia hoc policy.

### 4.2. Rough terrain la nhung gi?

R1 rough task dung `ROUGH_TERRAINS_CFG` tu MJLab core. Cau hinh nay tao terrain
grid kich thuoc patch `8m x 8m`, `10` hang do kho, `20` cot loai dia hinh.
Thanh phan hien co:

| Terrain | Ty le | Y nghia |
| :--- | ---: | :--- |
| `flat` | `0.2` | Mat phang de robot hoc gait co ban |
| `pyramid_stairs` | `0.2` | Bac thang xuoi/di len theo dang pyramid |
| `pyramid_stairs_inv` | `0.2` | Bac thang nguoc/di xuong |
| `hf_pyramid_slope` | `0.1` | Mat doc |
| `hf_pyramid_slope_inv` | `0.1` | Mat doc nguoc |
| `random_rough` | `0.1` | Nen go ghe ngau nhien |
| `wave_terrain` | `0.1` | Nen dang song |

Curriculum dia hinh lam viec theo tung env. Neu mot robot di duoc du xa tren
patch hien tai, env do co the duoc day len muc kho hon. Neu no di qua kem so
voi command, env do bi ha xuong muc de hon. Vi vay trong cung mot job train,
co the co nhom robot dang hoc flat/de, nhom khac dang gap bac thang/doc kho hon.

MJLab core cung co `STAIRS_TERRAINS_CFG` rieng cho stairs-only curriculum, nhung
R1 rough task hien tai dang dung `ROUGH_TERRAINS_CFG`. Neu sau nay muon train
"1000 con chi di bac thang", cach dung dung la tao overlay/config rieng chon
`STAIRS_TERRAINS_CFG` hoac mot terrain generator chi gom stairs, khong sua truc
tiep `third_party`.

### 4.3. Moi env khac nhau o nhung diem nao?

Trong cung mot batch song song, cac env khong nhat thiet giong nhau:

- Command velocity duoc resample moi `3.0` den `8.0` giay.
- Base spawn co x/y/yaw ngau nhien.
- Terrain level/type co the khac nhau neu dung rough terrain.
- Robot co the bi push theo interval `5.0` den `6.0` giay.
- Foot friction, encoder bias va base COM offset duoc randomize.
- Episode cua env nao nga/het gio thi reset rieng, khong can doi cac env khac.

Vi the "100 con chay nhay lung tung" trong RL thuc chat la 100 ban sao robot
dang bi ep giai quyet nhieu tinh huong khac nhau cung luc, nhung tat ca cung
dung mot policy network. Policy tot la policy co hanh vi on dinh trung binh qua
toan bo nhung tinh huong do.

## 5. Observation/action/command

Actor observation trong velocity task gom:

| Term | Y nghia |
| :--- | :--- |
| `base_ang_vel` | Toc do goc than tu IMU |
| `projected_gravity` | Huong trong luc trong frame robot, giup policy biet do nghieng |
| `command` | Lenh velocity `twist` hien tai |
| `phase` | Sin/cos gait phase chu ky `0.6` s; ve 0 khi command gan dung yen |
| `joint_pos` | Joint position relative |
| `joint_vel` | Joint velocity relative |
| `actions` | Action truoc do |
| `height_scan` | Chi co o rough terrain |

Critic observation rong hon actor. Critic thay them base linear velocity, foot
height, foot air/contact time va contact force. Day la actor-critic setup binh
thuong: critic duoc dung thong tin day du hon de hoc value function, actor van
duoc giu gan voi thong tin deploy duoc.

Action:

- Mot action vector cho joint position target.
- Scale theo tung nhom joint R1 qua `R1_ACTION_SCALE`.
- Output cua policy duoc clip theo cau hinh runner khi dua vao env wrapper.

Command:

- `UniformVelocityCommandCfg`
- Resample moi `3.0` den `8.0` giay.
- Khoang train ban dau:
  - `lin_vel_x`: `(-1.0, 2.0)`
  - `lin_vel_y`: `(-1.0, 1.0)`
  - `ang_vel_z`: `(-1.0, 1.0)`
  - heading: `(-pi, pi)`
- Curriculum command day command range tu de den kho hon theo step train.

## 6. Reward, termination, curriculum

Reward chinh cua velocity task:

| Reward | Weight | Muc tieu |
| :--- | ---: | :--- |
| `track_linear_velocity` | `1.0` | Di theo van toc xy duoc command, phat z velocity |
| `track_angular_velocity` | `1.0` | Quay theo yaw command, han che roll/pitch angular vel |
| `body_orientation_l2` | `-1.0` | Giu than dung |
| `pose` | `1.0` | Giu tu the hop ly theo che do dung/di/chay |
| `body_ang_vel` | `-0.05` | Giam rung/lac than |
| `angular_momentum` | `-0.025` | Khuyen khich chuyen dong toan than tu nhien hon |
| `is_terminated` | `-200.0` | Phat nang khi episode fail |
| `joint_acc_l2` | `-2.5e-7` | Giam gia toc joint |
| `joint_pos_limits` | `-10.0` | Tranh cham gioi han joint |
| `action_rate_l2` | `-0.05` | Lam action muot |
| `foot_gait` | `0.5` | Khuyen khich gait trai/phai lech pha |
| `foot_clearance` | `-1.0` | Tranh nhac chan qua sai target clearance |
| `foot_slip` | `-0.25` | Giam truot chan khi tiep dat |
| `soft_landing` | `-1e-3` | Giam luc dap chan |
| `stand_still` | `-1.0` | Khi command nho, giu joint gan default |
| `self_collisions` | `-1.0` | R1 override them phat self-collision |

Termination:

- `time_out`: het episode.
- `fell_over`: `bad_orientation` neu nghieng qua `70` do.

Domain randomization/event:

- Reset base pose quanh vi tri ban dau va yaw ngau nhien.
- Reset joint ve offset ban dau.
- Push robot moi `5-6` giay.
- Random foot friction `0.3-1.6`.
- Random encoder bias `-0.015..0.015`.
- Random base COM offset `+-0.05` m moi truc.

## 7. PPO/runner config

File:

```text
third_party/unitree_rl_mjlab/src/tasks/velocity/config/r1/rl_cfg.py
```

Config chinh:

| Hang muc | Gia tri |
| :--- | :--- |
| Actor MLP | `(512, 256, 128)`, activation `elu` |
| Critic MLP | `(512, 256, 128)`, activation `elu` |
| Obs normalization | Bat cho actor va critic |
| Action distribution | Gaussian, `init_std=1.0`, scalar std |
| PPO epochs | `5` |
| Mini-batches | `4` |
| LR | `1e-3`, adaptive schedule |
| gamma/lam | `0.99 / 0.95` |
| desired KL | `0.01` |
| max grad norm | `1.0` |
| steps per env | `24` |
| max iterations | `10001` |
| save interval | `100` |

Runner class cua R1 velocity la `VelocityOnPolicyRunner`.
Khi save checkpoint, runner goi export ONNX:

```text
model_<iter>.pt
policy.onnx
```

Policy ONNX duoc attach metadata tu env de deploy/doc audit de hon.

## 8. Cau truc train script

Entrypoint upstream:

```text
third_party/unitree_rl_mjlab/scripts/train.py
```

Luon doc theo flow nay:

1. Import `mjlab.tasks` va `src.tasks` de register task.
2. Lay task ID tu CLI, vi du `Unitree-R1-Flat`.
3. Load env config va RL config tu registry.
4. Chon GPU bang `--gpu-ids`.
5. Set `CUDA_VISIBLE_DEVICES` va `MUJOCO_GL=egl`.
6. Tao `ManagerBasedRlEnv`.
7. Wrap bang `RslRlVecEnvWrapper`.
8. Tao runner, resume checkpoint neu co.
9. Dump config vao `params/env.yaml` va `params/agent.yaml`.
10. Chay `runner.learn(...)`.

Upstream log path mac dinh:

```text
logs/rsl_rl/<experiment_name>/<date_time>/
```

Workspace wrapper doi current working directory thanh:

```text
data/runs/mjlab/
```

Vi vay log path thuc te trong Happy-Baby-R1 la:

```text
data/runs/mjlab/logs/rsl_rl/r1_velocity/<run>/
```

## 9. Lenh chay trong workspace nay

Setup mot dong:

```bash
./setup_env.sh
```

Kiem tra workspace:

```bash
python scripts/training/r1_policy_workspace.py status
```

Smoke test nho:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/training/r1_policy_workspace.py train mjlab \
  --terrain flat \
  --num-envs 1 \
  --max-iterations 1 \
  --run-name smoke \
  --agent.save-interval=1 \
  --gpu-ids None
```

Train GPU baseline:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/training/r1_policy_workspace.py train mjlab \
  --terrain flat \
  --num-envs 4096 \
  --max-iterations 10001 \
  --run-name r1_flat_v1
```

Train rough terrain sau khi flat da on:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/training/r1_policy_workspace.py train mjlab \
  --terrain rough \
  --num-envs 4096 \
  --max-iterations 10001 \
  --run-name r1_rough_v1
```

Collect policy ve thu muc deploy artifact cua workspace:

```bash
python scripts/training/r1_policy_workspace.py collect mjlab
```

Ket qua mong doi:

```text
data/policies/mjlab/<run>/model_<iter>.pt
data/policies/mjlab/<run>/policy.onnx
```

## 10. Play/kiem chung trong MJLab

`scripts/play.py` cua upstream dung de xem policy ngay trong MJLab/MuJoCo.
Vi workspace dat log trong `data/runs/mjlab`, khi play truc tiep nen chay tu
working directory do hoac dua checkpoint absolute path.

Dang lenh tham khao:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/data/runs/mjlab
PYTHONPATH=/home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab \
  PYTHONNOUSERSITE=1 \
  conda run -n r1_env python /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/scripts/play.py \
    Unitree-R1-Flat \
    --checkpoint_file=/home/ubuntu22/Projects/Happy-Baby-R1/data/policies/mjlab/<run>/model_<iter>.pt \
    --num-envs=1
```

Neu chi can artifact de dua sang runtime/deploy, khong bat buoc play moi collect
vi `VelocityOnPolicyRunner.save()` da export `policy.onnx` moi lan save.

## 11. Unitree MuJoCo dung de lam gi?

`third_party/unitree_mujoco` khong phai framework train. No la simulator MuJoCo
co bridge Unitree SDK2/DDS, dung de kiem chung controller/policy theo luong gan
voi robot that hon:

- Simulator publish/subscribe cac message `LowCmd`, `LowState`,
  `SportModeState`, `IMUState`.
- C++ simulator nam o `third_party/unitree_mujoco/simulate`.
- Python simulator nam o `third_party/unitree_mujoco/simulate_python`.
- `interface=lo` va `domain_id=1` la setting an toan cho local simulation.

Trong upstream `unitree_mujoco/simulate/config.yaml`, danh sach comment hien co
co `go2`, `b2`, `b2w`, `h1`, `go2w`, `g1`, `h2`; chua coi R1 la robot upstream
chinh thuc o file config do. Repo Happy-Baby-R1 co asset MuJoCo R1 rieng o:

```text
assets/mujoco/unitree_robots/r1/
```

va script load/test model:

```bash
python scripts/simulation/run_r1_mujoco_model.py --env flat --duration 10
```

Vi vay can tach hai viec:

1. **Train R1 policy**: dung `unitree_rl_mjlab` R1 asset/task.
2. **Runtime/deploy check**: dung `unitree_mujoco`/local runtime khi observation,
   joint order, default pose, DDS topic va ONNX shape da khop.

## 12. Deploy/runtime R1 policy

Upstream `unitree_rl_mjlab` da co deploy skeleton cho R1:

```text
third_party/unitree_rl_mjlab/deploy/robots/r1/
```

File `deploy/robots/r1/config/config.yaml` dinh nghia FSM:

- `Passive`
- `FixStand`
- `Velocity`

Luong dung:

```text
Passive -> FixStand -> Velocity
```

`FixStand` ramp ve default pose trong khoang `0..3` giay. `Velocity` load:

```text
config/policy/velocity/<version>/params/deploy.yaml
config/policy/velocity/<version>/exported/policy.onnx
```

`State_RLBase.cpp` tao `ManagerBasedRLEnv` tu `deploy.yaml`, load ONNX bang
`OrtRunner`, va map action ve `lowcmd.motor_cmd()[joint_id].q()`.

Dieu nay rat quan trong: policy deploy khong chi can `policy.onnx`. No can ca
bo metadata/config deploy khop voi observation/action/joint order cua luc train.
Trong workspace nay, policy collect hien gom checkpoint va ONNX. Buoc tiep theo
neu deploy R1 that la sinh/kiem `deploy.yaml` va mapping R1 tu dung task train.

## 13. Chuan hoa artifact va provenance

Quy uoc thu muc:

| Loai | Duong dan |
| :--- | :--- |
| Train logs/checkpoints | `data/runs/mjlab/logs/rsl_rl/r1_velocity/<run>/` |
| Policy da collect | `data/policies/mjlab/<run>/` |
| Cache pip/warp/matplotlib | `data/cache/` |
| Model/runtime asset noi bo | `data/models/` |
| State logs khi runtime | `data/sim_state_logs/` |

Moi lan train can ghi ro it nhat:

- Date/time run.
- Task ID: `Unitree-R1-Flat` hay `Unitree-R1-Rough`.
- Asset source: R1 MJCF trong `third_party/unitree_rl_mjlab` hay asset local.
- Command line day du.
- Env: `r1_env`, package pins, GPU/CPU.
- `num_envs`, `max_iterations`, `save_interval`, terrain.
- Checkpoint chon deploy: `model_<iter>.pt`.
- ONNX tu cung run voi checkpoint nao.
- Ket qua play/smoke/runtime: dung duoc, rung, nga, hay crash.

Khong copy checkpoint/ONNX vao `third_party`. Neu can dua policy vao deploy tree
de build official controller, copy co chu dich va ghi provenance; khong coi do
la baseline upstream.

## 14. Tieu chi ket qua train

Mot run train co the coi la co gia tri khi dat cac muc:

1. Training khong NaN/crash.
2. Reward velocity tracking tang on dinh, termination/fall giam.
3. Play trong MJLab dung duoc voi `Unitree-R1-Flat` hoac `Unitree-R1-Rough`.
4. `policy.onnx` load duoc va input/output shape khop runtime.
5. Robot khong rung manh trong pha FixStand/warmup.
6. Neu dua sang Unitree MuJoCo/DDS runtime, policy khong nga ngay tu spawn va
   log state co du lieu hop ly.

Smoke run `1` iteration chi xac nhan pipeline, khong chung minh policy tot.
Policy de deploy can train dai, play, va kiem chung runtime rieng.

## 15. Tai lieu nen doc tiep

- Setup MJLab host env: [mjlab_installation.md](mjlab_installation.md)
- Workspace train/export wrapper: [r1_policy_workspace.md](r1_policy_workspace.md)
- Unitree MuJoCo runtime: [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md)
- Third-party policy: [third-party_build.md](third-party_build.md)
- Safety rules: [../safety/safety_rules.md](../safety/safety_rules.md)
