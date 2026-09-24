# So sánh giới hạn khớp: g1_29dof.xml vs bảng tham chiếu

- Nguồn XML: `unitree_mujoco/unitree_robots/g1/g1_29dof.xml` (thuộc tính `range` của `<joint>`).
- Nguồn tham chiếu: bảng ảnh bạn gửi (mình chép lại cột Radian).
- Chuẩn hoá hiển thị: luôn hiển thị theo cặp (min,max) với min<=max.
- Nếu lệch, mình thử thêm trường hợp **đảo dấu** (nhân -1) để phản ánh quy ước chiều dương khác nhau giữa bảng và XML.

## Bảng so sánh (radian)

| idx | Joint (bảng) | Joint (XML) | XML range (rad) | Ref range dùng để so (rad) | Δmin | Δmax | Kết luận | Ghi chú |
|---:|---|---|---:|---:|---:|---:|---|---|
| 0 | L_LEG_HIP_PITCH | left_hip_pitch_joint | [-2.530700, 2.879800] | [-2.530700, 2.879800] | +0 | +0 | ✅ |  |
| 1 | L_LEG_HIP_ROLL | left_hip_roll_joint | [-0.523600, 2.967100] | [-0.523600, 2.967100] | +0 | +0 | ✅ |  |
| 2 | L_LEG_HIP_YAW | left_hip_yaw_joint | [-2.757600, 2.757600] | [-2.757600, 2.757600] | +0 | +0 | ✅ |  |
| 3 | L_LEG_KNEE | left_knee_joint | [-0.087267, 2.879800] | [-0.087267, 2.879800] | +0 | +0 | ✅ |  |
| 4 | L_LEG_ANKLE_PITCH | left_ankle_pitch_joint | [-0.872670, 0.523600] | [-0.872670, 0.523600] | +0 | +0 | ✅ |  |
| 5 | L_LEG_ANKLE_ROLL | left_ankle_roll_joint | [-0.261800, 0.261800] | [-0.261800, 0.261800] | +0 | +0 | ✅ |  |
| 6 | R_LEG_HIP_PITCH | right_hip_pitch_joint | [-2.530700, 2.879800] | [-2.530700, 2.879800] | +0 | +0 | ✅ |  |
| 7 | R_LEG_HIP_ROLL | right_hip_roll_joint | [-2.967100, 0.523600] | [-2.967100, 0.523600] | +0 | +0 | ✅ (đảo dấu) | bảng dùng quy ước dấu ngược |
| 8 | R_LEG_HIP_YAW | right_hip_yaw_joint | [-2.757600, 2.757600] | [-2.757600, 2.757600] | +0 | +0 | ✅ |  |
| 9 | R_LEG_KNEE | right_knee_joint | [-0.087267, 2.879800] | [-0.087267, 2.879800] | +0 | +0 | ✅ |  |
| 10 | R_LEG_ANKLE_PITCH | right_ankle_pitch_joint | [-0.872670, 0.523600] | [-0.872670, 0.523600] | +0 | +0 | ✅ |  |
| 11 | R_LEG_ANKLE_ROLL | right_ankle_roll_joint | [-0.261800, 0.261800] | [-0.261800, 0.261800] | +0 | +0 | ✅ |  |
| 12 | WAIST_YAW | waist_yaw_joint | [-2.618000, 2.618000] | [-2.618000, 2.618000] | +0 | +0 | ✅ |  |
| 13 | WAIST_ROLL | waist_roll_joint | [-0.520000, 0.520000] | [-0.520000, 0.520000] | +0 | +0 | ✅ |  |
| 14 | WAIST_PITCH | waist_pitch_joint | [-0.520000, 0.520000] | [-0.520000, 0.520000] | +0 | +0 | ✅ |  |
| 15 | L_SHOULDER_PITCH | left_shoulder_pitch_joint | [-3.089200, 2.670400] | [-3.089200, 2.670400] | +0 | +0 | ✅ |  |
| 16 | L_SHOULDER_ROLL | left_shoulder_roll_joint | [-1.588200, 2.251500] | [-1.588200, 2.251500] | +0 | +0 | ✅ |  |
| 17 | L_SHOULDER_YAW | left_shoulder_yaw_joint | [-2.618000, 2.618000] | [-2.618000, 2.618000] | +0 | +0 | ✅ |  |
| 18 | L_ELBOW | left_elbow_joint | [-1.047200, 2.094400] | [-1.047200, 2.094400] | +0 | +0 | ✅ |  |
| 19 | L_WRIST_ROLL | left_wrist_roll_joint | [-1.972220, 1.972220] | [-1.972222, 1.972222] | +2.054e-06 | -2.054e-06 | ✅ |  |
| 20 | L_WRIST_PITCH | left_wrist_pitch_joint | [-1.614430, 1.614430] | [-1.614430, 1.614430] | -4.42e-07 | +4.42e-07 | ✅ |  |
| 21 | L_WRIST_YAW | left_wrist_yaw_joint | [-1.614430, 1.614430] | [-1.614430, 1.614430] | -4.42e-07 | +4.42e-07 | ✅ |  |
| 22 | R_SHOULDER_PITCH | right_shoulder_pitch_joint | [-3.089200, 2.670400] | [-3.089200, 2.670400] | +0 | +0 | ✅ |  |
| 23 | R_SHOULDER_ROLL | right_shoulder_roll_joint | [-2.251500, 1.588200] | [-2.251500, 1.588200] | +0 | +0 | ✅ |  |
| 24 | R_SHOULDER_YAW | right_shoulder_yaw_joint | [-2.618000, 2.618000] | [-2.618000, 2.618000] | +0 | +0 | ✅ |  |
| 25 | R_ELBOW | right_elbow_joint | [-1.047200, 2.094400] | [-1.047200, 2.094400] | +0 | +0 | ✅ |  |
| 26 | R_WRIST_ROLL | right_wrist_roll_joint | [-1.972220, 1.972220] | [-1.972222, 1.972222] | +2.054e-06 | -2.054e-06 | ✅ |  |
| 27 | R_WRIST_PITCH | right_wrist_pitch_joint | [-1.614430, 1.614430] | [-1.614430, 1.614430] | -4.42e-07 | +4.42e-07 | ✅ |  |
| 28 | R_WRIST_YAW | right_wrist_yaw_joint | [-1.614430, 1.614430] | [-1.614430, 1.614430] | -4.42e-07 | +4.42e-07 | ✅ |  |

## Góc (độ) quy đổi từ XML (tham khảo)

| idx | Joint | XML range (deg) |
|---:|---|---:|
| 0 | L_LEG_HIP_PITCH | [-145.00, 165.00] |
| 1 | L_LEG_HIP_ROLL | [-30.00, 170.00] |
| 2 | L_LEG_HIP_YAW | [-158.00, 158.00] |
| 3 | L_LEG_KNEE | [-5.00, 165.00] |
| 4 | L_LEG_ANKLE_PITCH | [-50.00, 30.00] |
| 5 | L_LEG_ANKLE_ROLL | [-15.00, 15.00] |
| 6 | R_LEG_HIP_PITCH | [-145.00, 165.00] |
| 7 | R_LEG_HIP_ROLL | [-170.00, 30.00] |
| 8 | R_LEG_HIP_YAW | [-158.00, 158.00] |
| 9 | R_LEG_KNEE | [-5.00, 165.00] |
| 10 | R_LEG_ANKLE_PITCH | [-50.00, 30.00] |
| 11 | R_LEG_ANKLE_ROLL | [-15.00, 15.00] |
| 12 | WAIST_YAW | [-150.00, 150.00] |
| 13 | WAIST_ROLL | [-29.79, 29.79] |
| 14 | WAIST_PITCH | [-29.79, 29.79] |
| 15 | L_SHOULDER_PITCH | [-177.00, 153.00] |
| 16 | L_SHOULDER_ROLL | [-91.00, 129.00] |
| 17 | L_SHOULDER_YAW | [-150.00, 150.00] |
| 18 | L_ELBOW | [-60.00, 120.00] |
| 19 | L_WRIST_ROLL | [-113.00, 113.00] |
| 20 | L_WRIST_PITCH | [-92.50, 92.50] |
| 21 | L_WRIST_YAW | [-92.50, 92.50] |
| 22 | R_SHOULDER_PITCH | [-177.00, 153.00] |
| 23 | R_SHOULDER_ROLL | [-129.00, 91.00] |
| 24 | R_SHOULDER_YAW | [-150.00, 150.00] |
| 25 | R_ELBOW | [-60.00, 120.00] |
| 26 | R_WRIST_ROLL | [-113.00, 113.00] |
| 27 | R_WRIST_PITCH | [-92.50, 92.50] |
| 28 | R_WRIST_YAW | [-92.50, 92.50] |

## Kết luận

- Khớp hoàn toàn: 28/29
- Khớp sau khi đảo dấu: 1/29
- Không khớp: 0/29