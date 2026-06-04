# Thuc hanh 08 - Mo phong state/control 4-DOF (2 terminal)
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-008
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Muc tieu

Mo phong dung quy trinh nhu hardware that:

- Robot (low code) gui `state` + `imu` ve workstation.
- Controller (high code) tinh `torque` va gui `cmd` ve robot.
- Bat buoc chay 2 terminal: 1 terminal ROS, 1 terminal conda.

## 2. So do kenh giao tiep

```
[Robot ROS terminal]
  state  -> UDP:15110
  imu    -> UDP:15111
  cmd   <- UDP:15112

[Conda terminal]
  state <- UDP:15110
  imu   <- UDP:15111
  cmd   -> UDP:15112
```

## 3. Chay nhu hardware that

### 3.1. Terminal A (ROS - robot)

```bash
exec zsh
load_ros
python3 sim/unitree_r1_robot_sim.py --duration 12
```

### 3.2. Terminal B (Conda - controller)

```bash
exec zsh
load_ml
python3 sim/unitree_r1_controller_sim.py --duration 12
```

## 4. Dau ra mong doi

- Terminal robot in `pos/vel` cho 4 joint.
- Terminal controller in `pos` + `rpy` + `tau`.
- Chay het thoi gian `--duration` thi tu dung.

## 5. Dieu chinh thong so

- Thay `--target` de doi vi tri muc tieu.
- Thay `--kp`, `--kd` de xem luc dieu khien thay doi.
- Thay `--dt` de tang giam toc do vong lap.

## 6. Luu y an toan

- Day la mo phong, khong tac dong den hardware that.
- Khi chuyen sang robot that, can kiem tra an toan theo [../SOP_v0.md](../SOP_v0.md).
