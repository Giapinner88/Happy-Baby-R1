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
      ├── optional robot-camera WebRTC plane (display only)
      │      A-button edge: Quest passthrough ↔ robot camera
      │ newline-delimited R1TeleopCommand JSON
      ▼
run_r1_upstream_ik_stream.py      env `tv`
      │ initial-head anchor → vendor R1_A5_ArmIK → 12 joints
      ▼
run_r1_quest3_live.py             Isaac Sim Python + IsaacLab source path
      │ `teleop/r1/isaaclab_robot.py` owns R1 articulation/actuator config
      │ validate/apply joint targets
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

Trong hardware workflow, vector đã qua hardware rate limiter được fan-out theo
cùng `sequence_id`: stdout ưu tiên đi sidecar, mirror queue hữu hạn đi Isaac.
Nhánh mirror không có DDS/UDP và không được phép back-pressure đường robot.
TeleImager ZMQ camera recorder là nhánh evidence riêng; WebRTC plane chỉ dùng
để quan sát trong Quest.

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
- Camera WebRTC chỉ là nhánh hiển thị trong Vuer; nó không đi vào schema lệnh,
  IK hoặc kênh actuation. Mất ảnh không tự thay đổi deadman.
- Hardware input có ba semantics: cò phải giữ = ACTIVE, nhả = PAUSED/explicit
  STOP nhưng session sống, cò trái = reset pending được phát sau khi anchor mới
  được xác nhận bằng ba mẫu ACTIVE.
- Vendor upstream là solver duy nhất của pipeline active; wrapper project giữ
  frame/anchor, validation và evidence boundary.
- Simulation và hardware evidence phải có boundary rõ.
