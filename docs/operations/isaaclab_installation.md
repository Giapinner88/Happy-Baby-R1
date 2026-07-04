# Isaac Lab / Unitree Sim End-to-End Pipeline
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-007
**Status:** Draft / Working

Tai lieu nay la pipeline chinh cua `Happy-Baby-R1` cho Isaac Lab / Unitree Sim,
di tu may moi den luc chay duoc simulator/teleop. Co the tham khao tai lieu
trong `unitree_sim_isaaclab`, nhung khong cai dat truc tiep theo cac tai lieu do
neu no khong khop voi moi truong chinh. Tat ca buoc lay source, cai
dependencies, build image/env, verify, van hanh va ghi artifact deu duoc dieu
phoi tu repo chinh nay.

`third_party/` chi la noi chua source/vendor lay tu ben ngoai. No la input cua
pipeline, khong phai noi dat quy trinh van hanh rieng.

## 1. Ranh gioi third-party

| Thanh phan | Duong dan trong repo nay | Vai tro |
| :--- | :--- | :--- |
| Isaac Lab | `third_party/IsaacLab` | Framework robot learning doc lap, chay tren Isaac Sim |
| Unitree Sim Isaac Lab | `third_party/unitree_sim_isaaclab` | Lop tich hop Unitree task/robot/DDS tren Isaac Lab |
| XR Teleoperate | `third_party/xr_teleoperate` | Quest/Vuer teleoperation stack |
| Teleimager | `third_party/unitree_sim_isaaclab/teleimager` | Camera ZMQ/WebRTC stack |

Tai lieu Happy-Baby-R1 noi chuoi cac thanh phan nay thanh pipeline chinh:

1. Lay/cap nhat source third-party vao `third_party/`.
2. Cai Isaac Sim, Isaac Lab, Unitree Sim dependencies bang lenh da dieu chinh cho moi truong chinh.
3. Build Docker image hoac local env theo baseline lab.
4. Verify bang lenh trong repo chinh.
5. Van hanh theo [teleop_quest3_vi.md](teleop_quest3_vi.md).
6. De artifact/log noi bo trong `data/`, `media/`, `docs/`, khong de trong vendor tree.

Neu can sua source third-party, ghi provenance/patch rieng; khong tron logic noi
bo vao vendor tree.

## 2. Tai lieu tham khao

Doc trong third-party chi dung de tham khao version, phu thuoc, loi thuong gap
va provenance. Khong copy/paste nguyen xi vao moi truong chinh neu pipeline nay
da co lenh rieng:

- Isaac Lab overview: `third_party/IsaacLab/README.md`
- Isaac Lab install index: `third_party/IsaacLab/docs/source/setup/installation/index.rst`
- Isaac Lab pip/source install:
  - `third_party/IsaacLab/docs/source/setup/installation/pip_installation.rst`
  - `third_party/IsaacLab/docs/source/setup/installation/source_installation.rst`
- Unitree Sim README: `third_party/unitree_sim_isaaclab/README.md`
- Unitree Sim install docs:
  - `third_party/unitree_sim_isaaclab/doc/isaacsim5.1_install.md`
  - `third_party/unitree_sim_isaaclab/doc/isaacsim5.0_install.md`
  - `third_party/unitree_sim_isaaclab/doc/isaacsim4.5_install.md`
- Quest 3 runbook trong pipeline chinh: [teleop_quest3_vi.md](teleop_quest3_vi.md)

Quy uoc trong Happy-Baby-R1: pipeline nay phai co du cac buoc de cai lai tu dau,
bao gom ca third-party. Khi tai lieu third-party mau thuan voi pipeline nay, uu
tien pipeline nay va ghi lai ly do dieu chinh. Tren may da setup xong thi khong
lap lai moi ngay; moi lan van hanh sau do chi chay simulator va teleop theo
runbook.

## 3. Baseline lab

| Muc | Gia tri |
| :--- | :--- |
| Repo root | `/home/ubuntu22/Projects/Happy-Baby-R1` |
| Unitree Sim root | `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab` |
| Isaac Lab root | `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/IsaacLab` |
| Docker image van hanh | `unitree-sim:latest` |
| Docker base | `nvidia/cuda:12.2.0-runtime-ubuntu22.04` |
| Isaac Sim baseline | `5.1.0` neu dung Dockerfile hien tai |
| Python cho Isaac Sim 5.x | Python 3.11 |
| Simulator env trong container | conda env `unitree_sim_env` |
| Quest/Vuer port | `8012` |
| Head camera WebRTC port | `60001` |

Dat bien tien loi:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
export HB_R1_ROOT="$PWD"
export UNITREE_SIM_ROOT="$PWD/third_party/unitree_sim_isaaclab"
export ISAACLAB_ROOT="$PWD/third_party/IsaacLab"
```

## 4. Pipeline tu dau

### 4.1. Lay repo chinh

Neu may chua co workspace:

```bash
mkdir -p /home/ubuntu22/Projects
cd /home/ubuntu22/Projects
git clone <HAPPY_BABY_R1_REMOTE_URL> Happy-Baby-R1
cd Happy-Baby-R1
```

Neu repo da co san:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
```

### 4.2. Lay third-party source vao pipeline chinh

Chi clone vao `third_party/` khi thu muc tuong ung chua co. Neu da co san trong
checkout hien tai, bo qua buoc clone va sang buoc verify.

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1
mkdir -p third_party

test -d third_party/IsaacLab || \
  git clone https://github.com/isaac-sim/IsaacLab.git third_party/IsaacLab

test -d third_party/unitree_sim_isaaclab || \
  git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git third_party/unitree_sim_isaaclab

test -d third_party/xr_teleoperate || \
  git clone https://github.com/unitreerobotics/xr_teleoperate.git third_party/xr_teleoperate

test -d third_party/unitree_sdk2_python || \
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git third_party/unitree_sdk2_python
```

Khoi tao submodule cua Unitree Sim va XR Teleoperate:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
git submodule update --init --depth 1

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate
git submodule update --init --depth 1
```

### 4.3. Chon huong cai dat cua moi truong chinh

Huong chinh cua Happy-Baby-R1 la:

- Simulator/Isaac Lab: chay trong Docker image `unitree-sim:latest`.
- Teleop/Vuer/Quest host: chay trong env `tv`.
- ROS 2 Foxy / DDS cua repo chinh: khong tron voi Python/Isaac Sim env.

Tai lieu upstream sau chi de doi chieu, khong phai lenh cai mac dinh cho host:

```text
third_party/unitree_sim_isaaclab/doc/isaacsim5.1_install.md
third_party/unitree_sim_isaaclab/doc/isaacsim5.0_install.md
third_party/unitree_sim_isaaclab/doc/isaacsim4.5_install.md
```

Chi dung local non-Docker install khi co ly do ro rang, tao env rieng, va ghi
lai sai khac so voi pipeline Docker.

### 4.4. Host prerequisites cua pipeline

Tren host, cai cac tool co ban de lay source, build Docker, tai assets va chay
GUI/GPU:

```bash
sudo apt update
sudo apt install -y git git-lfs curl cmake build-essential
nvidia-smi
```

Kiem tra Docker nhan GPU:

```bash
sudo docker run --rm --gpus all nvidia/cuda:12.2.0-runtime-ubuntu22.04 nvidia-smi
```

Neu Docker khong nhan GPU, cai NVIDIA Container Toolkit theo muc host
prerequisites trong [teleop_quest3_vi.md](teleop_quest3_vi.md), roi quay lai
pipeline nay de build image.

## 5. Kiem tra da cai chua

Truoc khi cai lai, kiem tra cac dau hieu da co:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1

test -d third_party/IsaacLab
test -d third_party/unitree_sim_isaaclab
test -d third_party/unitree_sim_isaaclab/assets
sudo docker image inspect unitree-sim:latest >/dev/null
```

Neu image `unitree-sim:latest` da ton tai va assets da co, khong can cai lai
cho cac buoi teleop/pick-and-place thong thuong. Chuyen sang
[teleop_quest3_vi.md](teleop_quest3_vi.md).

Chi cai lai khi:

- Doi may hoac setup lai OS.
- Xoa Docker image/env.
- Doi NVIDIA driver/CUDA/Isaac Sim/Isaac Lab major version.
- Pull vendor source moi va muon rebuild baseline.
- Assets third-party bi thieu hoac sai version.

## 6. Cai Isaac Sim / Isaac Lab cho pipeline chinh

Khong cai Isaac Sim/Isaac Lab truc tiep vao Python/ROS env cua repo chinh. Moi
truong chinh dung Docker image `unitree-sim:latest`; Dockerfile cua
`third_party/unitree_sim_isaaclab` la noi dong goi Isaac Sim, Isaac Lab,
CycloneDDS, Unitree SDK Python va dependencies can cho simulator.

Tai lieu upstream duoc dung de doi chieu version:

```text
third_party/unitree_sim_isaaclab/doc/isaacsim5.1_install.md
```

Build image tu source third-party nam trong pipeline chinh:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
sudo docker build -t unitree-sim:latest -f Dockerfile .
```

Neu can proxy:

```bash
sudo docker build \
  --build-arg http_proxy=http://127.0.0.1:7890 \
  --build-arg https_proxy=http://127.0.0.1:7890 \
  -t unitree-sim:latest -f Dockerfile .
```

Kiem tra image:

```bash
sudo docker image inspect unitree-sim:latest >/dev/null
```

## 7. Local install chi de debug

Local install tren host khong phai baseline cua Happy-Baby-R1 vi de xung dot voi
ROS 2 Foxy/Python/DDS cua moi truong chinh. Chi dung khi can debug third-party
va phai tao env rieng:

```text
conda env rieng: unitree_sim_env
khong source ROS 2 Foxy trong terminal nay
khong cai vao r1_env hay Python system cua repo chinh
```

Khi can local install, doc tai lieu third-party de biet version phu thuoc, sau do
ghi lai lenh da dieu chinh cho may lab truoc khi coi la baseline moi:

```text
third_party/unitree_sim_isaaclab/doc/isaacsim5.1_install.md
third_party/IsaacLab/docs/source/setup/installation/index.rst
```

## 8. Tai assets cho simulator

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
sudo apt update
sudo apt install -y git-lfs
. fetch_assets.sh
```

Kiem tra asset quan trong:

```bash
find assets -name PackingTable.usd
```

Neu assets da co, khong chay lai `fetch_assets.sh` tru khi asset bi thieu hoac
muon update theo upstream. Day van la asset cua third-party simulator, nhung
duoc quan ly nhu mot buoc setup cua pipeline Happy-Baby-R1.

## 9. Noi vao pipeline van hanh chinh

Sau khi source, image, assets va env da san sang, daily operation khong cai lai.
Thu tu pipeline la:

1. Chay simulator/container theo [teleop_quest3_vi.md](teleop_quest3_vi.md).
2. Test camera `https://<HOST_IP>:60001`.
3. Chay teleop host env `tv`.
4. Mo Quest/Vuer `https://<HOST_IP>:8012`.

Setup env `tv` cho teleop cung nam trong [teleop_quest3_vi.md](teleop_quest3_vi.md)
vi no la phan van hanh Quest/Vuer, nhung van thuoc pipeline chinh
Happy-Baby-R1. Neu loi runtime, doc log va debug theo runbook teleop; chi quay
lai tai lieu cai dat nay khi loi that su nam o missing source/image/env/assets.

## 10. Tai lieu lien quan

- Quest 3 teleop: [teleop_quest3_vi.md](teleop_quest3_vi.md)
- Third-party build policy: [third-party_build.md](third-party_build.md)
- Golden machine spec: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
