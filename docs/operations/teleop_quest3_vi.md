# Cai dat va chay Unitree Sim teleoperation voi Meta Quest 3

Runbook nay la tai lieu van hanh cua pipeline chinh `Happy-Baby-R1` cho luong
mo phong pick-and-place voi model G1, Dex3 va Meta Quest 3. Luong nay gia dinh
Isaac Lab / Unitree Sim da duoc cai mot lan theo
[isaaclab_installation.md](isaaclab_installation.md); moi lan van hanh sau do
chi chay simulator, camera check, teleop va task.

## Pham vi va third-party

Repo `Happy-Baby-R1` chi luu tai lieu van hanh, quy uoc, log va artifact noi bo.
Nhung repo/tool sau duoc coi la third-party/external stack, khong phai source
code noi bo cua repo nay:

- `unitree_sim_isaaclab`: simulator Isaac Lab va cac task G1/Dex3.
- `xr_teleoperate`: server teleoperation/Vuer va bridge Quest 3.
- `teleimager`: camera streaming/WebRTC/ZMQ cho simulator va Quest/browser.

Tai lieu trong cac repo third-party chi dung de doi chieu version, phu thuoc va
loi thuong gap. Viec cai dat moi truong phai di qua pipeline chinh
[isaaclab_installation.md](isaaclab_installation.md), khong chay nguyen pipeline
upstream neu no khong khop voi layout/env cua Happy-Baby-R1.

Duong dan ben duoi la layout dang dung tren may lab:

- Third-party simulator root: `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab`
- Third-party teleop root: `/home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate`
- Simulator Docker image: `unitree-sim:latest`
- Teleop host env: conda env `tv`
- Headset: Meta Quest 3, mo bang Meta Browser
- Host IP phai kiem tra moi lan bang `hostname -I`; tai thoi diem debug
  2026-07-01, IP dang la `192.168.1.47`, khong phai IP cu `192.168.1.66`.
- Task mac dinh: `Isaac-PickPlace-Cylinder-G129-Dex3-Joint`
- Robot mac dinh: `G1_29 + Dex3`

Neu clone/checkout third-party vao duong dan khac, thay cac lenh `cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab` bang
duong dan local tuong ung.

Neu IP host thay doi, thay tat ca IP vi du trong tai lieu bang IP moi. Xem IP bang:

```bash
hostname -I
```

## 0. Thu tu chay dung

Thu tu moi lan chay:

1. Chay simulator trong Docker.
2. Test camera WebRTC `https://192.168.1.66:60001`.
3. Chay teleop tren host, chi chay mot instance.
4. Mo Vuer tren Quest: `https://192.168.1.66:8012/?ws=wss%3A%2F%2F192.168.1.66%3A8012`.
5. Bam `Virtual Reality`.
6. Nhan `r` trong terminal teleop de bat dau control.

Khong dung `Ctrl+Z` de tat teleop, vi process co the bi stop va giu port `8012`. Neu muon thoat, bam `q` trong terminal teleop hoac dung `Ctrl+C`.

## 1. Kiem tra host

Kiem tra NVIDIA driver:

```bash
nvidia-smi
```

Kiem tra Docker nhan GPU:

```bash
sudo docker run --rm --gpus all nvidia/cuda:12.2.0-runtime-ubuntu22.04 nvidia-smi
```

Neu gap loi:

```text
could not select device driver "" with capabilities: [[gpu]]
```

cai NVIDIA Container Toolkit:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 2. Tai assets va build Docker

Chay tu repo root:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
sudo apt update
sudo apt install -y git-lfs
. fetch_assets.sh
```

Build Docker image. Neu khong co proxy `127.0.0.1:7890`, khong them build arg proxy:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
sudo docker build -t unitree-sim:latest -f Dockerfile .
```

Neu co proxy local that su dang chay:

```bash
sudo docker build \
  --build-arg http_proxy=http://127.0.0.1:7890 \
  --build-arg https_proxy=http://127.0.0.1:7890 \
  -t unitree-sim:latest -f Dockerfile .
```

## 3. Cai teleop env `tv` tren host

Chay trong host, khong chay trong Docker:

```bash
conda activate tv
export PYTHONNOUSERSITE=1

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sdk2_python
python -m pip install -e .

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate
git submodule update --init --depth 1
python -m pip install -r requirements.txt

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop/televuer
python -m pip install -e .

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop/teleimager
python -m pip install -e .

cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop/robot_control/dex-retargeting
python -m pip install -e .
```

Kiem tra teleop import:

```bash
conda activate tv
export PYTHONNOUSERSITE=1
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
python -c "from televuer import TeleVuerWrapper; print('televuer OK')"
python -c "from teleimager.image_client import ImageClient; print('teleimager OK')"
python teleop_hand_and_arm.py --help
```

Luon dung:

```bash
export PYTHONNOUSERSITE=1
```

Ly do: may nay tung co package trong `~/.local` lam nhieu env bi lay nham NumPy/package.

## 4. Chuan bi cert cho HTTPS/WSS

Vuer va WebRTC dung HTTPS/WSS nen browser se hoi trust certificate. Tao cert vao:

```text
~/.config/xr_teleoperate/cert.pem
~/.config/xr_teleoperate/key.pem
```

Lenh tao cert cho IP `192.168.1.66`:

```bash
mkdir -p ~/.config/xr_teleoperate
cd ~/.config/xr_teleoperate

openssl genrsa -out rootCA.key 2048
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 3650 \
  -out rootCA.pem -subj "/CN=xr-teleoperate"

openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=192.168.1.66"

cat > server_ext.cnf <<'EOF'
subjectAltName = @alt_names
[alt_names]
IP.1 = 192.168.1.66
DNS.1 = localhost
EOF

openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
  -CAcreateserial -out cert.pem -days 3650 -sha256 -extfile server_ext.cnf
```

Them env vao `~/.bashrc` neu chua co:

```bash
echo 'export XR_TELEOP_CERT="$HOME/.config/xr_teleoperate/cert.pem"' >> ~/.bashrc
echo 'export XR_TELEOP_KEY="$HOME/.config/xr_teleoperate/key.pem"' >> ~/.bashrc
source ~/.bashrc
```

## 5. Camera config

File camera config tren host:

```text
teleimager/cam_config_server.yaml
```

Head camera nen de VP8 de Quest/browser de hien thi:

```yaml
head_camera:
  enable_zmq: true
  zmq_port : 55555
  enable_webrtc: true
  webrtc_port : 60001
  webrtc_codec: vp8
```

Docker command ben duoi se mount file nay vao container. Neu sua config, phai restart simulator container.

## 6. Chay simulator trong Docker

Terminal 1 tren host:

```bash
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
xhost +SI:localuser:root

sudo docker run --gpus all -it --rm \
  --network host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display \
  -e LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:$LD_LIBRARY_PATH \
  -e DISPLAY=$DISPLAY \
  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.config/xr_teleoperate:/root/.config/xr_teleoperate:ro" \
  -v "$PWD/assets:/home/code/unitree_sim_isaaclab/assets:ro" \
  -v "$PWD/teleimager/cam_config_server.yaml:/home/code/unitree_sim_isaaclab/teleimager/cam_config_server.yaml:ro" \
  unitree-sim:latest /bin/bash
```

Trong container:

```bash
cd /home/code/unitree_sim_isaaclab
conda activate unitree_sim_env

python sim_main.py \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
  --enable_dex3_dds \
  --robot_type g129
```

Sau khi Isaac Sim mo len, click chuot trai vao cua so Sim mot lan de kich hoat render.

Log tot nen co:

```text
head_camera ... WebRTC 60001
left_wrist_camera ... WebRTC 60002
right_wrist_camera ... WebRTC 60003
Preferred VP8 for port:60001
controller started, start main loop
```

## 7. Test camera

Mo tren Ubuntu browser hoac Quest Browser:

```text
https://192.168.1.66:60001
```

Neu co warning certificate, chon `Advanced` / `Proceed`, roi bam `Start`.

Test bang OpenCV tren host:

```bash
conda activate tv
export PYTHONNOUSERSITE=1
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop
python -m teleimager.image_client --host 192.168.1.66
```

Neu OpenCV hien cac cua so camera tot, simulator camera va ZMQ stream da OK.

## 8. Chay teleop tren host

Truoc khi chay, nen dam bao khong co teleop cu giu port `8012`:

```bash
ss -ltnp | grep 8012
```

Neu co process cu va browser bi timeout, kill no:

```bash
pkill -9 -f teleop_hand_and_arm.py
```

Chay teleop, Terminal 2 tren host:

```bash
source ~/.bashrc
conda activate tv
export PYTHONNOUSERSITE=1
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop

python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee dex3 \
  --sim \
  --img-server-ip 192.168.1.66
```

Sau vai giay, port `8012` phai listen:

```bash
ss -ltnp | grep 8012
```

Test local tren Ubuntu:

```text
https://127.0.0.1:8012/?ws=wss%3A%2F%2F127.0.0.1%3A8012
```

Neu muon record dataset, them `--record`:

```bash
python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee dex3 \
  --sim \
  --record \
  --img-server-ip 192.168.1.66
```

## 9. Ket noi Meta Quest 3

Quest 3 phai cung WiFi/LAN voi Ubuntu host.

Tren Quest:

1. Mo app `Browser`.
2. Mo camera de accept cert va test stream:

   ```text
   https://192.168.1.66:60001
   ```

   Chon `Advanced` / `Proceed`, roi bam `Start`.
3. Mo Vuer:

   ```text
   https://192.168.1.66:8012/?ws=wss%3A%2F%2F192.168.1.66%3A8012
   ```
4. Bam `Virtual Reality`.
5. Allow cac quyen XR/tracking.
6. Dat tay gan pose ban dau cua robot de tranh giat manh.
7. Quay lai terminal teleop tren host va nhan:

   ```text
   r
   ```

Trong khi teleop:

- `r`: reset/start tracking.
- `s`: start recording, neu dang chay voi `--record`.
- `s` lan nua: stop va save episode.
- `q`: thoat teleop.

Data record mac dinh:

```text
xr_teleoperate/teleop/utils/data
```

## 10. Chay cac task khac

Chi doi `--task` va DDS flag trong lenh `sim_main.py`. Voi teleop Dex3, van dung:

```bash
python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee dex3 \
  --sim \
  --img-server-ip 192.168.1.66
```

### G1 29DoF + Dex3

Pick cylinder:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129
```

Pick red block:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129
```

Stack blocks:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129
```

Pick red block into drawer:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Pick-Redblock-Into-Drawer-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129
```

Move cylinder wholebody:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex3-Wholebody \
  --enable_dex3_dds --robot_type g129
```

### G1 29DoF + Dex1 gripper

Simulator:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint \
  --enable_dex1_dds --robot_type g129
```

Teleop:

```bash
python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee dex1 \
  --sim \
  --img-server-ip 192.168.1.66
```

## 11. Replay va generate data

Replay dataset da record:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129 \
  --replay \
  --file_path "/home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop/utils/data"
```

Generate data tu replay:

```bash
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129 \
  --replay \
  --file_path "/home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate/teleop/utils/data" \
  --generate_data \
  --generate_data_dir "./data_generated"
```

## 12. Debug nhanh

Kiem tra port:

```bash
ss -ltnp | grep -E ':60000|:60001|:8012'
```

Ket qua mong doi:

- `60000`: camera config server tu simulator.
- `60001`: head camera WebRTC tu simulator.
- `8012`: Vuer server tu teleop.

Camera `60001` OK nhung Vuer `8012` khong mo:

```bash
pkill -9 -f teleop_hand_and_arm.py
```

Sau do chay lai teleop mot lan duy nhat.

`8012` listen nhung browser timeout:

- Co the teleop cu bi treo giu port.
- Kill teleop cu bang lenh tren.
- Chay lai teleop va test local `https://127.0.0.1:8012/?ws=wss%3A%2F%2F127.0.0.1%3A8012`.

Quest khong mo duoc nhung Ubuntu mo duoc:

- Kiem tra Quest va Ubuntu cung WiFi/LAN.
- Kiem tra IP host con la `192.168.1.66`.
- Cho phep firewall neu dang bat:

  ```bash
  sudo ufw allow 8012/tcp
  sudo ufw allow 60001/tcp
  ```

Teleop loi `No module named meshcat`:

```bash
conda activate tv
export PYTHONNOUSERSITE=1
cd /home/ubuntu22/Projects/Happy-Baby-R1/third_party/xr_teleoperate
python -m pip install -r requirements.txt
```

Sim loi thieu `PackingTable.usd`:

- Kiem tra assets tren host da tai bang `. fetch_assets.sh`.
- Kiem tra Docker run co mount:

  ```bash
  -v "$PWD/assets:/home/code/unitree_sim_isaaclab/assets:ro"
  ```

Camera hien den tren browser nhung OpenCV co hinh:

- Camera sim/ZMQ da OK.
- Kiem tra `teleimager/cam_config_server.yaml` co `webrtc_codec: vp8`.
- Restart simulator container de config moi co hieu luc.

