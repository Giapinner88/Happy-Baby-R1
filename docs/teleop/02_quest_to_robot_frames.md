# 02 — Quest/OpenXR to Robot Frames

## 1. Raw XR frame

Vendor wrapper ghi:

```text
OpenXR basis:
    x right
    y up
    z back

Robot basis:
    x front
    y left
    z up
```

Raw XR data nằm trong WORLD frame do XR device odometry định nghĩa:

$$
{}^{W_Q}T_H,\qquad
{}^{W_Q}T_{A_L},\qquad
{}^{W_Q}T_{A_R}.
$$

## 2. OpenXR basis → robot basis

Unitree dùng:

$$
T_{R\leftarrow Q}
=
\begin{bmatrix}
0&0&-1&0\\
-1&0&0&0\\
0&1&0&0\\
0&0&0&1
\end{bmatrix}
$$

và similarity transform:

$$
\boxed{
{}^{W_R}T
=
T_{R\leftarrow Q}
\,{}^{W_Q}T\,
T_{Q\leftarrow R}
}
$$

tương ứng:

$$
x_R=-z_Q,\qquad
y_R=-x_Q,\qquad
z_R=y_Q.
$$

## 3. Wrist initial-pose convention alignment

Hand tracking wrist pose của OpenXR không cùng local-axis convention với Unitree arm URDF.

Left:

$$
T_{L,\text{align}}
=
\begin{bmatrix}
1&0&0&0\\
0&0&-1&0\\
0&1&0&0\\
0&0&0&1
\end{bmatrix}
$$

Right:

$$
T_{R,\text{align}}
=
\begin{bmatrix}
1&0&0&0\\
0&0&1&0\\
0&-1&0&0\\
0&0&0&1
\end{bmatrix}
$$

Sau đó:

$$
{}^{W_R}T_A^{U}
=
{}^{W_R}T_A^{XR}
T_{\text{align}}.
$$

Controller tracking là ngoại lệ: source ghi rằng controller pose đã theo Unitree arm URDF initial convention nên không cần bước alignment này.

## 4. Head-yaw reference

Mode mặc định:

```text
arm_reference_mode = "head_yaw"
```

Unitree không dùng full head rotation.

Từ head $x$-axis, bỏ thành phần vertical:

$$
\tilde x_H
=
\begin{bmatrix}
x_{H,x}\\
x_{H,y}\\
0
\end{bmatrix}
$$

chuẩn hóa:

$$
\hat x_H
=
\frac{\tilde x_H}{\|\tilde x_H\|}
$$

với:

$$
\hat z=
\begin{bmatrix}
0\\0\\1
\end{bmatrix},
\qquad
\hat y_H
=
\hat z\times\hat x_H.
$$

Head-yaw frame:

$$
\boxed{
{}^{W_R}R_{H_y}
=
\begin{bmatrix}
\hat x_H & \hat y_H & \hat z
\end{bmatrix}
}
$$

Wrist pose relative to head yaw:

$$
\boxed{
{}^{H_y}R_A
=
({}^{W_R}R_{H_y})^T
{}^{W_R}R_A
}
$$

$$
\boxed{
{}^{H_y}p_A
=
({}^{W_R}R_{H_y})^T
\left(
{}^{W_R}p_A
-
{}^{W_R}p_H
\right)
}
$$

Ý nghĩa:
- loại head translation khỏi wrist target;
- dùng yaw làm heading reference;
- bỏ head pitch/roll để tránh wrist motion giả khi chỉ cúi/ngẩng/nghiêng đầu.

## 5. Head → waist offset

Vendor wrapper thêm:

$$
\boxed{
p_{\text{waist}}
=
p_{H_y}
+
\begin{bmatrix}
0.15\\
0\\
0.45
\end{bmatrix}
}
$$

Đây là workspace/retargeting offset trong source, không phải head-to-waist transform được suy ra từ URDF.

## 6. Chuỗi transform cuối

```text
XR WORLD pose
→ Robot basis
→ Unitree wrist convention
→ Head-yaw relative pose
→ Waist/workspace offset
→ wrist target for IK
```

Hay:

$$
{}^{W_Q}T_A
\rightarrow
{}^{W_R}T_A^{XR}
\rightarrow
{}^{W_R}T_A^{U}
\rightarrow
{}^{H_y}T_A^{U}
\rightarrow
{}^{W_{IK}}T_{EE,d}.
$$

## 7. Hand keypoints

Hand keypoints đi theo nhánh riêng:

$$
p^{W_R}_{hand}
=
T_{R\leftarrow Q}p^{W_Q}_{hand}
$$

$$
p^{A}_{hand}
=
({}^{W_R}T_A)^{-1}
p^{W_R}_{hand}.
$$

Sau đó đổi initial hand convention bằng `T_TO_UNITREE_HAND`.

Arm wrist retargeting và finger retargeting là hai nhánh khác nhau.
