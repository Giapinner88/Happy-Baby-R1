# 01 — System Architecture

## 1. Mục tiêu

Hệ thống nhận chuỗi pose từ Quest 3:

$$
\mathcal{T}_{XR}
=
\left\{
T_H(t_k),\;
T_{W_L}(t_k),\;
T_{W_R}(t_k)
\right\}
$$

và sinh joint trajectory:

$$
\mathcal{T}_{R1}
=
\left\{
q_d(t_k)
\right\}.
$$

Mục tiêu là **teleoperation trajectory tracking**, không learning.

## 2. Đường tín hiệu hiện tại

```text
Quest 3 (Quest Browser, immersive WebXR session)
      │ HTTPS + WSS :8012
      ▼
quest_bridge.py                   env `tv`                30 Hz
      │ newline-delimited R1TeleopCommand JSON
      ▼
run_r1_quest3_live.py             env `unitree_sim_env`
      │ mapper → IK → rate limit
      ├──────────────► Isaac Sim
      │
      └─ hardware target path
         run_r1_quest3_hardware_targets.py
              │ JSONL 12 joints
              ▼
         high_level_sidecar.py
              │ UDP 127.0.0.1:5560
              ▼
         hb_high_level (run_r1)
              └── sole `rt/lowcmd` publisher
```

Hai Conda environments không dùng chung interpreter vì `vuer` xung đột với IsaacLab. Chúng chạy thành hai process nối bằng pipe.

Vì cả hai process chạy cùng host, `time.monotonic()` là shared timebase và command age có thể đo trực tiếp.

## 3. Interface chính

Quest-side output sau wrapper:

```text
head_pose          (4x4)
left_wrist_pose    (4x4)
right_wrist_pose   (4x4)
```

IK-side input:

$$
T_{EE,L,d}(t_k),\qquad T_{EE,R,d}(t_k).
$$

Robot-side output:

$$
q_d(t_k).
$$

## 4. Nguyên tắc thiết kế

- Không tự sinh neutral command khi XR data invalid.
- Quest bridge, mapper, IK và hardware writer phải tách riêng.
- Vendor upstream là reference, không mặc định là accepted project method.
- Simulation và hardware evidence phải có boundary rõ.
