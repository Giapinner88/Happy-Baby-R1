# 04 — Trajectory Tracking

## 1. Baseline interpretation

Trong implementation hiện tại, desired trajectory là **chuỗi pose theo thời gian**:

$$
\boxed{
\mathcal{T}_d
=
\{T_{EE,d}(t_k)\}_{k=0}^{N}
}
$$

Không cần giả định rằng IK bắt buộc nhận explicit $v_d$ hoặc $\omega_d$.

## 2. Desired velocity để đánh giá

Linear velocity có thể suy ra:

$$
v_{d,k}
\approx
\frac{p_{d,k}-p_{d,k-1}}{\Delta t}.
$$

Actual Cartesian velocity:

$$
v_k
=
J_v(q_k)\dot q_k.
$$

## 3. Tracking metrics

Position error:

$$
e_p(t)=p_d(t)-p(t)
$$

Orientation error:

$$
e_R(t)
=
\log(R(t)R_d(t)^T)^\vee
$$

Velocity error:

$$
e_v(t)=v_d(t)-v(t).
$$

Có thể log thêm:
- solve time;
- solver status;
- joint saturation;
- joint limit activation;
- command age;
- achieved control rate.

## 4. Vì sao chưa thêm velocity-level controller

Vendor pipeline hiện:
- nhận pose sequence;
- warm start bằng nghiệm trước;
- thêm smoothness cost;
- lọc joint solution;
- dùng low-level joint servo.

Upstream IK không thực hiện explicit Cartesian velocity tracking.

Do đó baseline project nên benchmark pose-sequence tracking trước.

Chỉ thêm explicit velocity-level control nếu:
1. frame mapping đúng;
2. pose IK ổn định;
3. compute rate đủ;
4. nhưng measured $e_v$ vẫn không đạt yêu cầu.

## 5. Research question thực tế hơn

Vấn đề trọng tâm không phải “Quest có velocity hay không”, mà là:

$$
\boxed{
SE(3)\ target\ (6D)
\rightarrow
R1\ arm\ (5DoF)
}
$$

và trade-off:

$$
\boxed{
\text{position accuracy}
\;\text{vs}\;
\text{orientation fidelity}
}
$$

trên một trajectory realtime.
