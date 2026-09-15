# Lightning AI IsaacLab + NGC Training Operation
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-007
**Author:** Integration Lead
**Status:** Working / validated setup notes

Tài liệu này chuẩn hóa cách dựng môi trường huấn luyện Isaac Lab trên Lightning AI Studio bằng Docker và image NVIDIA/NGC. Lightning AI chỉ dùng cho training, evaluation, video/debug offline, checkpoint export và artifact management. Không dùng Lightning AI để điều khiển robot trực tiếp.

Ghi chú kiểm chứng gần nhất: Lightning AI Studio host Ubuntu 24.04 LTS, Docker container Isaac Lab/Isaac Sim nền Ubuntu 22.04 LTS, 1x NVIDIA L4 24GB VRAM. Cấu hình này đã chạy được smoke test PhysX headless và training thử `rsl_rl` cho ANYmal-C.

## 1. Phạm vi và ranh giới

| Thành phần | Vai trò | Có điều khiển robot trực tiếp? |
| :--- | :--- | :--- |
| Máy trạm vận hành local | ROS 2 Foxy, CycloneDDS, low-level command, ghi log, kiểm tra an toàn, đồng bộ artifact | Có, qua SOP riêng |
| Lightning AI Studio | Train Isaac Lab, chạy eval/play/video offline, quản lý checkpoint/model artifact | Không |
| Docker Isaac Lab/Isaac Sim | Môi trường runtime ổn định cho Omniverse, PhysX, Vulkan/EGL, Python Isaac Lab | Không |
| NVIDIA NGC | Cung cấp base image/container và artifact chuẩn NVIDIA | Không |

Nguyên tắc chính:

1. Lightning AI không thay thế máy trạm vận hành trong [system_communication_topology.md](../architecture/system_communication_topology.md).
2. Không cài Isaac Lab native trực tiếp trên host Lightning nếu mục tiêu là training ổn định. Dùng container để tránh lệch `isaacsim`, `pxr`, `omni.kit`, Vulkan/EGL và driver library.
3. Không đưa checkpoint/log/runtime local vào `third_party`. `third_party` chỉ chứa vendor source; artifact phát hành của dự án đi về `data/models/...`, log đi về `data/sim_state_logs` hoặc object storage đã review.
4. Mọi thay đổi về task, observation, action, reward, termination, curriculum hoặc export format phải khóa theo git commit, container tag và dataset/config version.

## 2. Trạng thái repo hiện tại

Repo Happy-Baby-R1 hiện có các ranh giới liên quan sau:

| Đường dẫn | Trạng thái / vai trò |
| :--- | :--- |
| `sim/isaac_lab_env/` | Placeholder cho Isaac Lab env nội bộ, hiện chỉ có `.gitkeep` |
| `sim/unitree_mujoco_policy/` | Runtime policy MuJoCo nội bộ, không phải Isaac Lab training stack |
| `scripts/bridge/run_unitree_mujoco_policy.py` | Launcher MuJoCo policy local |
| `data/models/unitree_mujoco_policy/` | ONNX/motion artifact cho MuJoCo policy |
| `data/models/isaac_lab/` | Chưa có trong checkout hiện tại; tạo khi export checkpoint/model từ Isaac Lab |
| `third_party/unitree_mujoco` | Vendor upstream, giữ sạch |
| `third_party/unitree_rl_mjlab` | Vendor policy/training của Unitree nếu được clone, không trộn log/checkpoint dự án |

Vì `sim/isaac_lab_env/` chưa có wrapper training riêng, lệnh train Isaac Lab phải chạy từ checkout Isaac Lab trong container, hoặc từ vendor training repo tương ứng nếu sau này repo đã clone và pin version rõ ràng.

## 3. Điều kiện tiên quyết

Trước khi bắt đầu, xác nhận:

1. Studio đang chạy bằng GPU hardware. Không dùng CPU-only Studio cho Isaac Lab Docker.
2. Host có Docker và nhìn thấy GPU:

```bash
nvidia-smi
docker --version
```

3. Nếu cần pull image từ NGC, có `NGC_API_KEY` hợp lệ:

```bash
export NGC_API_KEY=<your-ngc-api-key>
docker login nvcr.io -u '$oauthtoken' -p "$NGC_API_KEY"
```

4. Repo Happy-Baby-R1 đã clone và biết git SHA dùng cho run:

```bash
cd ~/Projects/Happy-Baby-R1
git rev-parse --short HEAD
```

5. Có vị trí lưu artifact tách khỏi vendor source:

```bash
mkdir -p data/models/isaac_lab data/datasets data/processed
mkdir -p ~/isaaclab_storage/{logs,checkpoints,artifacts,videos}
```

## 4. Chọn hardware trên Lightning AI

Cấu hình tối thiểu đã kiểm chứng cho smoke test và training thử:

| Thành phần | Mức tối thiểu |
| :--- | :--- |
| GPU | 1x NVIDIA L4 24GB VRAM |
| CPU | >= 16 vCPU |
| RAM | >= 64GB |
| Storage | >= 1TB nếu giữ dataset/checkpoint dài hạn |
| Host OS | Lightning AI Studio Ubuntu 24.04 LTS |
| Training runtime | Docker Isaac Lab/Isaac Sim, thường là Ubuntu 22.04 trong container |

Với training humanoid dài hoặc nhiều environment hơn, ưu tiên GPU có VRAM lớn hơn hoặc multi-GPU. L4 24GB dùng tốt để xác thực pipeline, nhưng không nên xem là baseline hiệu năng cuối cùng.

## 5. Dựng Isaac Lab bằng Docker

Cách ổn định nhất là dùng workflow Docker của Isaac Lab, không tự ghép package native trên host.

Từ host Lightning:

```bash
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
```

Nếu release Isaac Lab đang dùng có tag cố định, checkout tag đó trước khi build:

```bash
git checkout <isaac-lab-tag-or-commit>
```

Khởi động container từ root Isaac Lab:

```bash
python docker/container.py start
python docker/container.py enter
```

Một số release/documentation cũng cho phép gọi trực tiếp:

```bash
./docker/container.py start
./docker/container.py enter base
```

Nếu đang đứng trong thư mục `IsaacLab/docker`, lệnh tương đương là:

```bash
./container.py start
./container.py enter
```

Không thêm flag `--eula accept` vào `docker run`; Docker không có flag này. Với raw Docker command, dùng biến môi trường:

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  <isaac-lab-or-isaac-sim-image> \
  nvidia-smi
```

Với workflow `docker/container.py` của Isaac Lab, việc chạy container Isaac Sim/Isaac Lab đồng nghĩa với việc người dùng phải chấp nhận NVIDIA Software License Agreement. Nếu không đồng ý EULA thì không chạy container.

## 6. Cấu hình headless đúng trên Lightning

Lightning AI Studio thường không có X server/display vật lý. Không bật X11 forwarding khi build/start container headless.

Khi `container.py` hỏi bật X11 forwarding, chọn `n`.

Nếu đã chọn nhầm `y` và gặp lỗi `KeyError: 'DISPLAY'`, sửa file cấu hình của Isaac Lab:

```bash
cd ~/IsaacLab
grep -n "X11_FORWARDING" docker/.container.cfg
sed -i 's/^X11_FORWARDING_ENABLED=.*/X11_FORWARDING_ENABLED=0/' docker/.container.cfg
python docker/container.py start
```

Nếu file cấu hình ở release của bạn nằm trong `IsaacLab/docker/.container.cfg`, thao tác từ thư mục `IsaacLab/docker`:

```bash
grep -n "X11_FORWARDING" .container.cfg
sed -i 's/^X11_FORWARDING_ENABLED=.*/X11_FORWARDING_ENABLED=0/' .container.cfg
./container.py start
```

Không chữa lỗi này bằng cách tự đặt `DISPLAY=:0` trên cloud headless. Mục tiêu là chạy EGL/Vulkan headless, không ép X11 giả.

## 7. Sửa lỗi Docker không thấy GPU

Kiểm tra nhanh:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

Nếu lỗi:

```text
could not select device driver "nvidia" with capabilities: [[gpu]]
```

nghĩa là Docker daemon chưa được cấu hình với NVIDIA Container Toolkit. Cách cấu hình theo hướng hiện tại của NVIDIA:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Sau đó chạy lại:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

Nếu image Lightning dùng phiên bản toolkit cũ và không có `nvidia-ctk`, có thể gặp hướng dẫn cũ dạng:

```bash
sudo nvidia-container-toolkit --mode=config --id=docker
sudo systemctl restart docker
```

Ưu tiên `nvidia-ctk runtime configure --runtime=docker` khi command này tồn tại.

## 8. Sửa lỗi thiếu NVML

Nếu container sập với lỗi:

```text
libnvidia-ml.so.1: cannot open shared object file
RuntimeError: The container is not running
```

kiểm tra trước:

```bash
nvidia-smi
```

Nguyên nhân thường gặp trên Lightning AI là Studio vẫn đang ở CPU hardware. Chuyển Studio sang GPU hardware, ví dụ NVIDIA L4 24GB hoặc GPU mạnh hơn, rồi restart terminal/container. Khi host không có GPU thật và driver NVML, container không thể map `libnvidia-ml.so.1`.

## 9. Smoke test trước khi train dài

Vào container Isaac Lab:

```bash
cd ~/IsaacLab
python docker/container.py enter
```

Trong container, kiểm tra CUDA và Isaac Lab:

```bash
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO_GPU')"
python -c "import isaaclab; print('IsaacLab OK')"
```

Chạy PhysX headless smoke test:

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless
```

Kỳ vọng log có dòng tương tự:

```text
[INFO]: Setup complete...
```

Nếu script chạy vòng lặp liên tục, dừng bằng `Ctrl-C` sau khi thấy setup hoàn tất và GPU không crash.

Chạy training smoke test ngắn với task mẫu:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Anymal-C-v0 \
  --headless
```

Để smoke test, chỉ cần quan sát vài iteration đầu rồi dừng. Nếu release hỗ trợ flag giới hạn iteration, dùng flag đó để tránh chạy dài ngoài ý muốn.

## 10. Huấn luyện task mục tiêu

Chỉ chạy task đã xác nhận có trong registry của release Isaac Lab đang dùng. Ví dụ với task Unitree G1 nếu release đó có sẵn:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Unitree-G1-v0 \
  --headless
```

Nếu task R1/G1 nội bộ chưa tồn tại trong `sim/isaac_lab_env/`, không ghi tài liệu như thể repo đã có training wrapper. Việc cần làm trước khi train R1 thật sự:

1. Chốt USD/URDF/MJCF source cho robot và mapping joint/action.
2. Định nghĩa observation/action/reward/termination/curriculum.
3. Pin git SHA của Isaac Lab, Happy-Baby-R1 và dataset/config.
4. Chạy smoke test trên task mẫu trước, sau đó mới chạy task nội bộ.

Metadata tối thiểu cho mỗi run:

```text
run_id:
date:
happy_baby_r1_git_sha:
isaac_lab_git_sha:
container_image:
host_hardware:
task_name:
seed:
num_envs:
dataset_or_config_version:
checkpoint_dir:
notes:
```

## 11. Debug bằng video offline

Không bật camera/video trong training chính nếu đang tối ưu throughput. Sau khi có checkpoint, chạy play/eval riêng với camera:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task <TASK_NAME> \
  --checkpoint <CHECKPOINT_PATH> \
  --headless \
  --enable_cameras \
  --video
```

Lưu video ra artifact storage hoặc copy về `media/videos/` nếu video được chọn làm tư liệu dự án. Không commit video lớn nếu chưa có policy lưu trữ rõ ràng.

## 12. Đồng bộ artifact về repo local

Sau khi run hoàn tất, copy các thành phần sau về máy trạm hoặc object storage:

1. Checkpoint cuối cùng.
2. Config đã chạy.
3. Metadata run.
4. Metric summary.
5. Video debug nếu có.

Khuyến nghị layout cho artifact Isaac Lab:

```text
data/models/isaac_lab/<task_name>/<experiment_name>/<timestamp>/
  checkpoint.pt
  config.yaml
  metadata.yaml
  metrics.csv
```

Nếu artifact dùng cho MuJoCo runtime hiện tại, chỉ copy model đã export/review sang đúng thư mục runtime tương ứng, ví dụ `data/models/unitree_mujoco_policy/`. Không để checkpoint train dở dang lẫn với model phát hành.

## 13. Troubleshooting nhanh

| Lỗi | Nguyên nhân thường gặp | Cách xử lý |
| :--- | :--- | :--- |
| `unknown flag: --eula` | Trộn flag của installer/Omniverse vào `docker run` | Bỏ `--eula`; dùng `-e ACCEPT_EULA=Y` nếu chạy raw Docker |
| `KeyError: 'DISPLAY'` | Bật X11 forwarding trên host headless | Set `X11_FORWARDING_ENABLED=0` trong `docker/.container.cfg`, rebuild/start container |
| `could not select device driver "nvidia"` | Docker chưa cấu hình NVIDIA runtime | Cài `nvidia-container-toolkit`, chạy `sudo nvidia-ctk runtime configure --runtime=docker`, restart Docker |
| `libnvidia-ml.so.1` missing | Studio đang CPU-only hoặc host driver/GPU chưa sẵn | Chuyển Studio sang GPU hardware, kiểm tra `nvidia-smi`, restart container |
| Native import `isaacsim`, `pxr`, `omni.kit` lỗi | Cài native trên host bị lệch package/graphics libs | Vào container Isaac Lab và dùng Python/`isaaclab.sh` của container |
| FPS thấp hoặc OOM | Quá nhiều env/camera/video/log | Giảm `num_envs`, tắt camera trong train, chạy video ở bước play/eval riêng |

## 14. Checklist trước khi chốt run

1. `nvidia-smi` trên host chạy được.
2. `docker run --gpus all ... nvidia-smi` chạy được.
3. X11 forwarding tắt trên Lightning headless.
4. `create_empty.py --headless` chạy tới `[INFO]: Setup complete...`.
5. Training smoke test chạy được vài iteration.
6. Git SHA, container image, seed, task name và hardware đã ghi lại.
7. Checkpoint cuối cùng load lại được trong cùng container.
8. Artifact đã sync về `data/models/isaac_lab/...` hoặc object storage đã review.
9. Không có log/checkpoint/runtime project-owned nằm trong `third_party`.

## 15. Tài liệu liên quan

* Kiến trúc truyền thông hệ thống: [../architecture/system_communication_topology.md](../architecture/system_communication_topology.md)
* Cấu hình golden machine: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
* Quy trình vận hành robot: [SOP_v0.md](SOP_v0.md)
* Quy ước đặt tên file: [naming_convention.md](naming_convention.md)
* Third-party build: [third-party_build.md](third-party_build.md)
* Runtime policy MuJoCo: [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md)
* Isaac Lab Docker Guide: [https://isaac-sim.github.io/IsaacLab/v2.0.2/source/deployment/docker.html](https://isaac-sim.github.io/IsaacLab/v2.0.2/source/deployment/docker.html)
* NVIDIA Container Toolkit install guide: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
* Lightning AI Studio overview: [https://lightning.ai/docs/overview/ai-studio](https://lightning.ai/docs/overview/ai-studio)
