# Thuc hanh 07 - Giao tiep ROS2 va conda
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-007
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Muc tieu

Bai nay kiem tra xem node ROS2 chay trong conda co the giao tiep voi node ROS2 chay trong terminal thuong hay khong.

## 2. Dieu kien

- ROS 2 Humble da duoc cai dat va co the source.
- Conda env co Python 3.10 (khuyen nghi de tuong thich voi ROS 2 Humble).
- `unitree_sdk2_python` da duoc cai dat trong conda va co file vi du: [third_party/unitree_sdk2_python/example/h1_2/low_level/h1_2_low_level_example.py](third_party/unitree_sdk2_python/example/h1_2/low_level/h1_2_low_level_example.py).
- Neu test qua mang, cau hinh DDS theo [../network_setup_checklist.md](../network_setup_checklist.md).

## 3. Thiet lap chung

- Su dung alias da duoc setup:
	- `load_ros` cho terminal ROS 2.
	- `load_ml` cho terminal conda.
- Neu chi test local, tam thoi tat config CycloneDDS custom:
	- `unset CYCLONEDDS_URI`

Dat cac bien giong nhau o ca hai terminal:

```bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
```

Neu test qua mang va can CycloneDDS config, dam bao `CYCLONEDDS_URI` da duoc set (tu `~/.zshrc`):

```bash
export CYCLONEDDS_URI=file://$PWD/../../../config/cyclonedds_config.xml
```

## 4. Cac buoc test

### 4.1. Baseline ROS2 (khong dung conda)

*Muc nay chi de xac nhan ROS2 pub/sub chay binh thuong truoc khi test DDS voi conda.*

```bash
exec zsh
load_ros
unset CYCLONEDDS_URI  # chi ap dung cho test local
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
ros2 run demo_nodes_cpp talker
```

```bash
exec zsh
load_ros
unset CYCLONEDDS_URI  # chi ap dung cho test local
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
ros2 run demo_nodes_py listener
```

### 4.2. Test trang thai dong co (ROS2 robot + conda high code)

**Muc tieu:** Robot (ROS2 low code) gui `lowstate`, workstation (conda high code) gui `lowcmd`.

*Luu y:* ROS2 topic `lowstate`/`/lowcmd` se map sang DDS `rt/lowstate`/`rt/lowcmd`, tuong thich voi unitree_sdk2_python.

#### 4.2.1. Robot (ROS2 low code)

- Dam bao robot/sim dang chay driver ROS2 va co topic `lowstate` va `/lowcmd`.
- Kiem tra nhanh:

```bash
exec zsh
load_ros
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0  # test qua mang
ros2 topic list | grep -E "lowstate|lowcmd"
```

- Mo them terminal de quan sat trang thai (tuy chon):

```bash
./install/unitree_ros2_example/read_low_state_hg
```

#### 4.2.2. Workstation (Conda high code)

*Khong dung `load_ros` o phia conda. Day la DDS truc tiep qua unitree_sdk2_python.*

```bash
exec zsh
load_ml
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
python third_party/unitree_sdk2_python/example/h1_2/low_level/h1_2_low_level_example.py
```

Neu muon an toan hon, chay tren simulator truoc. Script nay se gui lenh dieu khien thuc.
Neu dung model khac, chon vi du tuong ung trong [third_party/unitree_sdk2_python/example](third_party/unitree_sdk2_python/example).

### 4.3. Dao chieu (bat buoc)

- Dung `talker` o conda va `listener` o terminal thuong.
- Dam bao van nhan duoc message trong 30-60 giay.

## 5. Tieu chi dat

- Neu chay test 4.1: listener nhan message lien tuc >= 30 giay.
- Neu chay test 4.2: conda in ra thong tin IMU (rpy) va ROS2 thay duoc `/lowcmd`.

## 6. Neu loi

- Kiem tra `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY` co giong nhau hay khong.
- Neu test qua mang, dat `ROS_LOCALHOST_ONLY=0`.
- Khong cai `rclpy` bang pip trong conda; dung `load_ros` de lay ROS 2 system-level.
- Neu `ros2` khong chay trong conda, do la binh thuong voi test 4.2 (conda chi dung unitree_sdk2_python).
- Neu thay log `deprecated element` hoac `unknown element` tu CycloneDDS va `rmw_create_node` fail, thu:
	- `echo $CYCLONEDDS_URI` de xac nhan dang dung file XML.
	- `unset CYCLONEDDS_URI` de test local, sau do cap nhat file XML cho phu hop version CycloneDDS.

## 7. Ghi log

- Su dung mau log tai [../../templates/test_log_template.md](../../templates/test_log_template.md).
- Luu log vao thu muc quy dinh sau khi ket thuc test.
