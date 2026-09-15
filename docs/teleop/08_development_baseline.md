# 08 — Development Baseline

## 1. Baseline pipeline

```text
Quest raw XR poses
        ↓
validated vendor wrapper
        ↓
OpenXR → robot basis
        ↓
wrist local-frame alignment
        ↓
head-yaw-relative mapping
        ↓
waist/workspace offset
        ↓
SE(3) wrist target sequence
        ↓
R1-A5 IK
        ↓
joint smoothing / rate limiting
        ↓
simulation first
        ↓
hardware only through sole-lowcmd-owner path
```

## 2. Development order

### Stage 1 — Frame correctness

Xác nhận bằng replay + visualization:
- head pose;
- left/right wrist pose;
- basis conversion;
- wrist alignment;
- head-yaw transform;
- waist offset.

### Stage 2 — Solver benchmark

Chạy **cùng một target sequence** qua:
1. upstream `R1_A5_ArmIK`;
2. current independent arm solver;
3. current coupled upper-body solver nếu cần.

Không benchmark bằng các input trajectory khác nhau.

### Stage 3 — Metrics

Log tối thiểu:

$$
e_p,\qquad
e_R,\qquad
e_v
$$

và:
- solver success/failure;
- solve time;
- joint saturation;
- achieved rate;
- command age.

### Stage 4 — 5-DoF trade-off

Đánh giá:
- translation weight;
- rotation weight;
- wrist orientation components có thể bỏ/giảm;
- sensitivity gần joint limits;
- reachable workspace.

### Stage 5 — Compute optimization

Current observed bottleneck là IK compute, không phải transport.

Ưu tiên:
- analytic Jacobian nếu có thể;
- reuse FK;
- warm start;
- reduced problem dimension;
- profiling từng stage.

### Stage 6 — Velocity tracking

Chỉ thêm explicit velocity-level controller nếu pose-sequence baseline đã tốt nhưng:

$$
e_v
$$

vẫn là failure mode chính.

## 3. Known gaps

1. Quest page mở chưa đủ; phải vào immersive WebXR session.
2. Có hai vendor trees:
   - `third_party/xr_teleoperate`
   - `third_party/xr_teleoperate_v1_6`

   Chúng pinned riêng và không được coi là interchangeable.
3. Robot address `10.42.0.33` chưa có DHCP reservation được xác nhận trong review 2026-08-18.
4. Current coupled Jacobian finite difference là compute bottleneck.
5. Chưa có hardware run xác nhận trajectory tracking accuracy/smoothness/full-pipeline rate.
6. Chưa thiết lập collision-free claim.
7. Current upper-body pilot chưa accepted cho robot actuation.
8. Velocity tracking hiện là evaluation metric, chưa phải vendor IK input.

## 4. Formulation để bám

$$
\boxed{
\text{Quest }SE(3)\text{ pose stream}
\rightarrow
\text{retargeted R1 wrist trajectory}
\rightarrow
\text{constrained 5-DoF IK}
\rightarrow
\text{joint trajectory}
}
$$
