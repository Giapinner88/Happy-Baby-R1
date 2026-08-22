# R1 Teleoperation Documentation

Bộ tài liệu tham chiếu cho pipeline **Meta Quest 3 → Unitree R1**.

Phạm vi hiện tại:

$$
\boxed{
\text{Quest }SE(3)\text{ pose stream}
\rightarrow
\text{retargeted wrist trajectory}
\rightarrow
\text{R1-A5 IK}
\rightarrow
\text{joint trajectory}
}
$$

Không dùng learning trong control loop.

## Cấu trúc tài liệu

1. [`01_system_architecture.md`](01_system_architecture.md)  
   Kiến trúc tổng thể và đường tín hiệu hiện tại.

2. [`02_quest_to_robot_frames.md`](02_quest_to_robot_frames.md)  
   OpenXR frame, basis conversion, wrist convention, head-yaw reference và waist offset.

3. [`03_r1_a5_ik.md`](03_r1_a5_ik.md)  
   R1-A5 kinematics, giới hạn 5-DoF và upstream Unitree nonlinear IK.

4. [`04_trajectory_tracking.md`](04_trajectory_tracking.md)  
   Cách hiểu trajectory tracking trong implementation hiện tại; position/orientation/velocity metrics.

5. [`05_project_solvers_and_evidence.md`](05_project_solvers_and_evidence.md)  
   Hai solver lịch sử trong project, schema boundary và quy tắc không trộn evidence.

6. [`06_runtime_and_latency.md`](06_runtime_and_latency.md)  
   Rate, latency, queue policy và các metadata đã quan sát.

7. [`07_hardware_boundary.md`](07_hardware_boundary.md)  
   Hardware architecture, sole `rt/lowcmd` owner và accepted pilot envelope.

8. [`08_development_baseline.md`](08_development_baseline.md)  
   Baseline phát triển tiếp và thứ tự benchmark.

## Nguồn chính

- Unitree upstream `xr_teleoperate`
- Unitree `televuer`
- Open-TeleVision
- KineDex
- evidence/method records trong project hiện tại

Các metadata, commit, schema và experiment IDs được giữ nguyên ở file evidence/runtime tương ứng.
