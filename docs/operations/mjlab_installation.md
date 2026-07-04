# MJLab / Unitree RL MJLab One-Time Setup
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-008
**Status:** Draft / Working

Tai lieu nay noi chuoi cach cai `third_party/mjlab` va
`third_party/unitree_rl_mjlab` mot lan cho may lab. Co the doc tai lieu trong
hai repo third-party nay de tham khao version, phu thuoc va loi thuong gap,
nhung khong cai dat truc tiep theo pipeline upstream neu no khong phu hop voi
moi truong chinh cua Happy-Baby-R1.

`mjlab` la framework MuJoCo/MuJoCo Warp. `unitree_rl_mjlab` la repo Unitree
chua task, train/play script va deploy config cho robot Unitree tren MJLab.
Ca hai doc lap voi `unitree_sim_isaaclab`, khong phai thanh phan cua Isaac Sim
teleop stack.

## 1. Ranh gioi third-party

| Thanh phan | Duong dan trong repo nay | Vai tro |
| :--- | :--- | :--- |
| MJLab | `third_party/mjlab` | Framework robot learning tren MuJoCo/MuJoCo Warp |
| Unitree RL MJLab | `third_party/unitree_rl_mjlab` | Unitree tasks, scripts train/play va deploy config tren MJLab |
| Isaac Lab | `third_party/IsaacLab` | Framework rieng; MJLab muon tuong thich API style |
| Unitree Sim Isaac Lab | `third_party/unitree_sim_isaaclab` | Simulator Isaac Lab/Quest teleop rieng, khong phai noi cai MJLab |

Khong tron cai dat MJLab/Unitree RL MJLab voi `unitree_sim_env` cua Isaac Sim.
Dung env rieng de tranh xung dot Python/CUDA package.

## 2. Tai lieu goc can doc

- MJLab README: `third_party/mjlab/README.md`
- MJLab install guide tham khao: `third_party/mjlab/docs/source/installation.rst`
- MJLab dependency manifest: `third_party/mjlab/pyproject.toml`
- MJLab Dockerfile: `third_party/mjlab/Dockerfile`
- Unitree RL MJLab README: `third_party/unitree_rl_mjlab/README.md`
- Unitree RL MJLab setup tham khao: `third_party/unitree_rl_mjlab/doc/setup_en.md`

Theo docs goc, MJLab:

- Can NVIDIA GPU cho training.
- Ho tro evaluation tren Linux/macOS/Windows WSL.
- Can Python `>=3.10,<3.14`.
- Khuyen nghi dung `uv`; pip/venv/conda van co duong classic.

Theo docs goc, Unitree RL MJLab dung Python 3.11, can NVIDIA GPU cho training,
va cai editable bang `pip install -e .`.

Trong Happy-Baby-R1, cac thong tin tren la input tham khao; lenh cai dat chinh
la cac lenh o muc 5, muc 6 hoac muc 7 cua tai lieu nay.

## 3. Baseline lab

| Muc | Gia tri |
| :--- | :--- |
| Repo root | `/home/ubuntu22/Projects/Happy-Baby-R1` |
| MJLab root | `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/mjlab` |
| Unitree RL MJLab root | `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab` |
| Version trong `pyproject.toml` | `1.4.0` |
| Python | 3.10, 3.11, 3.12 hoac 3.13 |
| Training | Linux + NVIDIA GPU |
| Recommended manager | `uv` |

Dat bien tien loi:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
export HB_R1_ROOT="$PWD"
export MJLAB_ROOT="$PWD/third_party/mjlab"
export UNITREE_RL_MJLAB_ROOT="$PWD/third_party/unitree_rl_mjlab"
```

## 4. Kiem tra da cai chua

Truoc khi cai lai:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
test -d third_party/mjlab
test -d third_party/unitree_rl_mjlab

cd third_party/mjlab
uv run python -c "import mjlab; print('mjlab OK')"
```

Neu chi dung Unitree RL MJLab, kiem tra rieng:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab
python -m pip show unitree_rl_mjlab
python -c "import src; print('unitree_rl_mjlab package OK')"
```

Neu cac lenh tren OK, khong can cai lai. Chay task/demo/training theo pipeline
hoac runbook thuc nghiem cua Happy-Baby-R1; docs third-party chi de doi chieu.

Chi cai lai khi:

- Doi may/env Python.
- Xoa `.venv`/conda env.
- Pull MJLab hoac Unitree RL MJLab version moi.
- Doi CUDA/PyTorch/MuJoCo-Warp stack.

## 5. Lay source third-party

Neu cac repo chua co trong `third_party`, clone tu root repo:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
mkdir -p third_party

test -d third_party/mjlab || \
  git clone https://github.com/mujocolab/mjlab.git third_party/mjlab

test -d third_party/unitree_rl_mjlab || \
  git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git third_party/unitree_rl_mjlab
```

## 6. Cai dat MJLab core bang uv

Day la command da chon cho pipeline chinh khi dung source checkout `third_party/mjlab`:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/mjlab
uv sync
uv run demo
```

Sau khi setup xong, cac command entrypoint co dang:

```bash
uv run train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 4096
uv run play Mjlab-Your-Task-Id --agent zero
uv run list-envs
```

Voi multi-GPU, pipeline co the dung dang lenh:

```bash
uv run train Mjlab-Velocity-Flat-Unitree-G1 \
  --gpu-ids "[0, 1]" \
  --env.scene.num-envs 4096
```

## 7. Cai dat Unitree RL MJLab

Unitree RL MJLab co pipeline rieng tren source checkout cua Unitree. Dung env
rieng, khong cai vao `unitree_sim_env`:

```bash
conda create -n unitree_rl_mjlab python=3.11 -y
conda activate unitree_rl_mjlab
python -m pip install --upgrade pip

sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab
python -m pip install -e .
```

Smoke test command shape:

```bash
python scripts/train.py Unitree-R1-Flat --env.scene.num-envs=4096
python scripts/play.py Unitree-R1-Flat --checkpoint_file=logs/rsl_rl/r1_velocity/<run>/model_<step>.pt
```

Motion imitation command shape:

```bash
python scripts/csv_to_npz.py \
  --input-file src/assets/motions/g1/dance1_subject2.csv \
  --output-name dance1_subject2.npz \
  --input-fps 30 \
  --output-fps 50 \
  --robot g1

python scripts/train.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/dance1_subject2.npz \
  --env.scene.num-envs=4096
```

Artifact sinh ra trong `third_party/unitree_rl_mjlab/logs/`,
`deploy/.../exported/`, hoac `src/assets/motions/...` phai duoc coi la artifact
thuc nghiem. Neu muon dung trong pipeline noi bo Happy-Baby-R1, copy/ghi
provenance sang `data/` hoac tai lieu thuc nghiem, khong coi vendor tree la noi
luu artifact chinh.

## 8. Cai MJLab core bang pip/conda

Neu khong dung `uv`, dung env rieng:

```bash
conda create -n mjlab python=3.11 -y
conda activate mjlab
python -m pip install --upgrade pip
python -m pip install mjlab
demo
```

Neu muon dung source local trong `third_party/mjlab`, uu tien `uv sync` o muc 6.
Dung pip editable cho source local chi khi can dev va da doc
`third_party/mjlab/docs/source/installation.rst`.

## 9. Docker/clusters

MJLab co Docker path rieng trong docs goc:

```text
third_party/mjlab/docs/source/installation.rst
third_party/mjlab/Dockerfile
```

Vi day la third-party doc lap, khong dung Docker image `unitree-sim:latest` cua
`unitree_sim_isaaclab` de suy luan ve MJLab.

## 10. Quan he voi workspace Unitree RL MJLab cu

Workspace cu nhu `/home/ubuntu22/train_mujoco/unitree_rl_mjlab` hoac
`sim/mujoco_lab_env/unitree_rl_mjlab` khong phai baseline cua repo nay. Baseline
moi la `third_party/unitree_rl_mjlab`.

Neu can giu lai train/play R1 tu workspace cu, tao tai lieu thuc nghiem rieng
va ghi ro:

- Repo/checkout nao dang chay.
- Task ID.
- Motion file.
- Checkpoint file.
- Env Python.

Tai lieu cai dat nay chi tra loi cau hoi: "MJLab/Unitree RL MJLab third-party
da cai chua, va cai mot lan nhu the nao?"

## 11. Tai lieu lien quan

- Third-party build policy: [third-party_build.md](third-party_build.md)
- Development environment: [development_environment_setup_guide.md](development_environment_setup_guide.md)
- Unitree MuJoCo policy runtime noi bo: [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md)
