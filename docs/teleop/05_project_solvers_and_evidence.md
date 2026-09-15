# 05 — Project Solvers and Evidence Boundaries

## 1. Solver A — coupled upper-body

File:

```text
teleop/r1/upper_body_ik.py
```

Đặc điểm:

- variables: 12 / 13 / 14 theo `body_mode`;
- task dimension: 15;
- Jacobian: central finite difference;
- wrist orientation: weighted best fit;
- null-space posture bias;
- step limit + hard joint clipping;
- stagnation detection;
- seed restart từ nominal configuration.

Dùng cho schema-3 / upper-body path.

Target head được dựng bằng FK của actual head chain.

Không thay bằng:

$$
R_z(yaw)R_y(pitch)
$$

nếu không khớp thứ tự joint thật trong URDF.

Status:

```text
simulation pilot method, not accepted for robot actuation
```

## 2. Solver B — independent 5-DoF arm

File:đề trọng tâm không phải “Quest có velocity hay khôn

```text
teleop/r1/ik.py
```

Đặc điểm:

- variables: 5;
- task: endpoint position 3D + imposed wrist roll;
- Jacobian: analytic geometric Jacobian.

Dùng cho schema-2 legacy evidence.

## 3. Evidence compatibility

Hai solver không được coi là equivalent.

Schema-2 và schema-3 evidence không được trộn vì:

- variable set khác;
- waist/head ownership khác;
- task formulation khác;
- solver behavior khác.

Older runs remain immutable legacy evidence.

Không được silently reinterpret evidence từ solver/method này bằng solver/method khác.

## 4. Method records

`03_r1_a5_ik.md`

- kinematic/method reference cho schema-2 / T002-style independent-arm runs;
- các statement `audited` là đã đọc từ asset/repo và test;
- các statement `assumed` phải verify trước khi dùng làm evidence.

File này cùng `03_r1_a5_ik.md`

- định nghĩa boundary schema-3 coupled waist/arms/head pilot;
- không tương thích với schema-2 evidence.
