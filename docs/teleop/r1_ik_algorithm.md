# Thuật toán IK của R1 — viết tường minh

Tài liệu này trình bày **đầy đủ phép toán** của hai bộ giải IK trong repo, đúng
theo code hiện tại. Mục tiêu: đọc xong có thể tự cài lại được, và biết chính xác
bộ giải làm gì khi nó **không** giải được.

Bổ sung cho [`r1_teleop_pipeline.md`](r1_teleop_pipeline.md), tài liệu đó mô tả
toàn tuyến tín hiệu; ở đây chỉ có IK.

## 0. Hai bộ giải, hai bài toán khác nhau

Repo có **hai** bộ giải riêng biệt, không phải hai phiên bản của một thứ.

| | Solver A — thân trên ghép | Solver B — một tay |
|---|---|---|
| File | `teleop/r1/upper_body_ik.py` | `teleop/r1/ik.py` |
| Biến khớp | 12 / 13 / 14 (theo `body_mode`) | 5 |
| Chiều nhiệm vụ | 15 | 3 (+1 áp đặt) |
| Jacobian | sai phân trung tâm | **giải tích** (hình học) |
| Hướng cổ tay | khớp tốt nhất có trọng số | **không** giải, roll áp trực tiếp |
| Dùng ở | T007 schema-3, đường phần cứng | T007 schema-2 (legacy) |

Solver A là đường đang dùng. Solver B giữ lại cho các run schema-2 tái lập được.

---

## 1. Ký hiệu và quy ước

| Ký hiệu | Nghĩa |
|---|---|
| `q ∈ ℝⁿ` | vector khớp, `n = model.dof` |
| `q⁻, q⁺` | giới hạn dưới/trên, đọc từ URDF |
| `q_nom` | tư thế nominal khai báo trong config |
| `p*, R*` | vị trí và hướng **mục tiêu** |
| `p(q), R(q)` | vị trí và hướng **hiện tại** theo FK |
| `e(q)` | vector sai số nhiệm vụ |
| `J = ∂e/∂q` | Jacobian của sai số nhiệm vụ |
| `W` | ma trận trọng số đường chéo |
| `λ` | hệ số giảm chấn (`damping`) |

Đơn vị: mét, radian. Mọi khung biểu diễn trong `pelvis_link`.

**Quan trọng về dấu — hai solver dùng hai quy ước khác nhau.** Đây là chỗ dễ
sai nhất khi đọc chéo hai file:

| | Jacobian là của | Quan hệ | Bước đi |
|---|---|---|---|
| Solver A | **sai số nhiệm vụ**, `J = ∂e/∂q` | `e(q+Δq) ≈ e + JΔq` | `Δq = −J⁺(We)` — **dấu âm** |
| Solver B | **vị trí endpoint**, `J = ∂p/∂q` | `∂e/∂q = −J` | `Δq = +Jᵀ(JJᵀ)⁻¹e` — **dấu dương** |

Cả hai đều đúng và tương đương; chỉ khác ở chỗ dấu âm đã được nuốt vào định
nghĩa Jacobian hay chưa. Chép công thức từ file này sang file kia mà không đổi
dấu sẽ làm bộ giải chạy **ra xa** mục tiêu.

---

## 2. Động học thuận

### 2.1 Một khớp quay

Mỗi khớp URDF cho một biến đổi thuần nhất 4×4:

```text
T_j(q_j) = [ R_origin,j · Rot(a_j, q_j)   t_origin,j ]
           [ 0    0    0                  1          ]
```

- `t_origin,j`, `R_origin,j` — gốc cố định của khớp, đọc từ `<origin>` của URDF.
  `R_origin` theo quy ước trục cố định XYZ: `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
- `a_j` — trục quay đơn vị, đọc từ `<axis>`.
- `Rot(a, θ)` — công thức Rodrigues:

```text
Rot(a, θ) = I + sin θ · [a]ₓ + (1 − cos θ) · [a]ₓ²

           ⎡  0   −a_z   a_y ⎤
   [a]ₓ =  ⎢ a_z    0   −a_x ⎥
           ⎣−a_y   a_x    0  ⎦
```

Cài đặt: `kinematics.axis_angle_to_matrix`, `kinematics.joint_transform`.

### 2.2 Chuỗi tay và điểm điều khiển

Nhân dồn dọc chuỗi từ gốc `waist_yaw_link`:

```text
T_arm(q) = T_shoulder_pitch · T_shoulder_roll · T_shoulder_yaw · T_elbow · T_wrist_roll
```

End-effector **không** phải gốc khớp cổ tay mà là frame ảo của vendor R1-A5:

```text
p_EE = t_wrist + R_wrist · d ,      d = (0.20, 0, 0) m
R_EE = R_wrist
```

`d` nằm dọc trục **+x cục bộ** của `wrist_roll_link`. FK và Jacobian **bắt buộc**
dùng cùng một điểm; dùng lệch nhau là lỗi đã từng xảy ra và làm target vươn xa
kẹt cách đích vài centimet.

### 2.3 Chuỗi thân trên

```text
T_waist(q)  = T_waist_roll(q_roll) · T_waist_yaw(q_yaw)      (roll cố định 0 nếu không điều khiển)
T_L(q)      = T_waist · T_arm,L(q_L)
T_R(q)      = T_waist · T_arm,R(q_R)
T_head(q)   = T_waist · T_head_pitch(q_hp) · T_head_yaw(q_hy)
```

Chú ý thứ tự chuỗi đầu: **pitch trước, yaw sau**. Target đầu phải dựng bằng
chính FK này (`model.head_rotation`), không được viết tay `Rz(yaw)·Ry(pitch)` —
hai thứ tự khác nhau khi cả hai góc khác 0, và sai lầm đó từng làm target đầu
bất khả thi.

---

## 3. Solver A — thân trên ghép

### 3.1 Vector sai số nhiệm vụ

15 phần tử, xếp theo đúng thứ tự này:

```text
        ⎡ p*_L − p_L(q) ⎤   0:3    vị trí cổ tay trái      (m)
        ⎢ p*_R − p_R(q) ⎥   3:6    vị trí cổ tay phải      (m)
e(q) =  ⎢ Log(R*_L R_Lᵀ)⎥   6:9    hướng cổ tay trái       (rad)
        ⎢ Log(R*_R R_Rᵀ)⎥   9:12   hướng cổ tay phải       (rad)
        ⎣ Log(R*_H R_Hᵀ)⎦   12:15  hướng đầu               (rad)
```

`R* Rᵀ` là **xoay còn thiếu** để đi từ hướng hiện tại tới hướng mục tiêu.
`Log` đưa nó về vector xoay (trục × góc) trong ℝ³.

### 3.2 Log của SO(3), đủ ba nhánh

`so3_log` phải xử lý riêng hai điểm kỳ dị số học, nếu không sẽ chia cho 0.

```text
θ = arccos( clamp( (tr(R) − 1)/2 , −1, +1 ) )

v = ( R₂₁ − R₁₂ ,  R₀₂ − R₂₀ ,  R₁₀ − R₀₁ )ᵀ
```

**Nhánh 1 — góc nhỏ, `θ < 1e-8`:**

```text
Log(R) = v / 2
```

Vì khi θ→0 thì `sin θ ≈ θ` và `θ/(2 sin θ) → 1/2`.

**Nhánh 2 — gần π, `π − θ < 1e-5`:** `sin θ → 0` nên công thức chung vỡ. Lấy
trục từ đường chéo:

```text
aᵢ = sqrt( max( (Rᵢᵢ + 1)/2 , 0 ) )
```

Chọn `pivot = argmax aᵢ` (thành phần lớn nhất, mẫu số an toàn nhất), rồi suy
hai thành phần còn lại từ phần đối xứng của `R`, ví dụ với `pivot = 0`:

```text
a₁ = (R₀₁ + R₁₀) / (4 a₀)
a₂ = (R₀₂ + R₂₀) / (4 a₀)
```

Chuẩn hoá `a`, trả về `θ·a`.

**Nhánh 3 — thông thường:**

```text
Log(R) = θ · v / (2 sin θ)
```

Trước mọi thứ, `_proper_rotation` từ chối ma trận không trực giao
(`‖RᵀR − I‖ > 1e-6`) hoặc `det(R) ≠ +1` — chặn cả phép phản chiếu.

### 3.3 Trọng số

`W = diag(w)` với `w` gồm ba khối:

```text
w = [ w_pos ×6 , w_ori ×6 , w_head ×3 ]
```

| Khối | Khoá | Giá trị T007 |
|---|---|---|
| vị trí (6) | `position_weight` | `10.0` |
| hướng cổ tay (6) | `wrist_orientation_weight` | `0.1` |
| hướng đầu (3) | `head_orientation_weight` | `0.5` |

Tỉ lệ `100:1` giữa vị trí và hướng cổ tay là **cố ý**: mỗi tay chỉ có 5 khớp,
không thể đồng thời đạt vị trí 3-DoF và hướng 3-DoF tuỳ ý. Hướng cổ tay được
đặt dung sai `π` nên **không bao giờ chặn việc chấp nhận** — nó chỉ uốn nghiệm
trong phần dư động còn lại, và residual được ghi lại nguyên vẹn.

### 3.4 Jacobian — sai phân trung tâm

```text
J[:, i] = ( e(q + δ·eᵢ) − e(q − δ·eᵢ) ) / (2δ) ,   δ = finite_difference_rad = 1e-5
```

Kích thước `15 × n`. Sai số cắt cụt bậc `O(δ²)`.

**Chi phí.** Mỗi cột cần 2 lần `e(·)`, mỗi lần là 1 FK toàn thân + 3 `so3_log`.
Với `n = 13`: **26 lần FK mỗi vòng lặp**.

| Phép | Đo trên máy trạm |
|---|---|
| `forward_kinematics` | 0.147 ms |
| `_task_error` | 0.200 ms |
| `upper_body_task_jacobian` | 5.163 ms |
| Một vòng lặp (J + e) | ≈ 5.36 ms |

Jacobian chiếm **~96%** chi phí một vòng lặp. Đây là nút thắt tốc độ duy nhất
đáng kể của bộ giải. Jacobian giải tích sẽ giảm nó khoảng một bậc, hiện **chưa
làm**.

### 3.5 Bước đi — bình phương tối thiểu có giảm chấn

Mỗi vòng giải bài toán tuyến tính hoá:

```text
Δq = argmin  ‖ W (e + J Δq) ‖²  +  λ² ‖Δq‖²
```

Số hạng `λ²‖Δq‖²` là phần **giảm chấn** (Levenberg–Marquardt). Đạo hàm theo `Δq`
và cho bằng 0:

```text
( (WJ)ᵀ(WJ) + λ² I ) Δq = − (WJ)ᵀ (W e)
```

Code đặt:

```text
H   = (WJ)ᵀ(WJ) + λ² I           # n×n, đối xứng nửa xác định dương
J⁺  = H⁻¹ (WJ)ᵀ                  # np.linalg.solve, KHÔNG nghịch đảo tường minh
Δq_primary = − J⁺ (W e)
```

**Vì sao phải giảm chấn.** Ở lân cận điểm kỳ dị, `WJ` mất hạng, `(WJ)ᵀ(WJ)` suy
biến và pseudo-inverse thuần đòi bước **vô hạn**. Thêm `λ²I` làm mọi trị riêng
tối thiểu bằng `λ²`, nên `‖Δq‖` luôn hữu hạn. Giá phải trả: nghiệm bị lệch nhẹ
so với bình phương tối thiểu thuần — đổi độ chính xác lấy tính ổn định.

### 3.6 Chiếu null-space cho tư thế

Phần dư động được dùng để kéo tư thế về `q_nom` mà **không** phá nhiệm vụ chính:

```text
P  = I − J⁺ (WJ)                          # chiếu vào null-space của WJ
Δq = Δq_primary + k_post · P (q_nom − q)   # k_post = posture_weight
```

`P` triệt tiêu mọi thành phần nằm trong không gian hàng của `WJ`, tức thành phần
sẽ làm thay đổi sai số nhiệm vụ. Chỉ phần "vô hình" với nhiệm vụ mới đi qua.

> **Lưu ý:** vì `λ > 0`, `P` là chiếu **gần đúng** (`P² ≠ P` chính xác). Với `λ`
> nhỏ so với trị kỳ dị của `WJ` thì sai lệch không đáng kể.

**Vì sao cần bias này.** Bài toán dư động: 5 khớp mỗi tay cho 3 ràng buộc vị trí.
Không có bias, hai lần giải cùng một target từ hai seed khác nhau sẽ dừng ở hai
cấu hình khuỷu khác nhau, và các trace không còn so sánh được với nhau.

**Hệ quả quan trọng — bias này yếu khi nhiệm vụ bất khả thi.** Target ngoài tầm
với làm `WJ` gần đủ hạng theo mọi hướng hữu ích, null-space co lại, nên `P` gần
như bằng 0. Đo được: tăng `posture_weight` gấp 10 lần chỉ kéo `waist_yaw` từ
`73°` xuống `56°` — **không** phải công cụ để sửa tư thế xấu.

### 3.7 Giới hạn bước và kẹp cứng

```text
nếu ‖Δq‖ > s_max:   Δq ← Δq · s_max / ‖Δq‖        # s_max = max_joint_step_rad
q ← clip( q + Δq , q⁻ , q⁺ )
```

Giới hạn bước là **vùng tin cậy**: giữ cho xấp xỉ tuyến tính `e + JΔq` còn đúng.
Kẹp giới hạn khớp áp ở **mọi** vòng lặp, không phải chỉ ở cuối, nên mọi iterate
đều là tư thế hợp lệ về mặt cơ khí.

### 3.8 Tiêu chí hội tụ

Kiểm tra **trước** khi tính Jacobian, nên một seed đã đúng sẽ thoát ngay ở vòng 1:

```text
tasks_ok  ⟺  max(‖e₀:₃‖, ‖e₃:₆‖)   ≤ position_tolerance_m
        và  max(‖e₆:₉‖, ‖e₉:₁₂‖)  ≤ wrist_orientation_tolerance_rad
        và      ‖e₁₂:₁₅‖          ≤ head_orientation_tolerance_rad
```

Tư thế nominal **không** nằm trong điều kiện này. Nó là chi phí phụ. Trước đây
việc lặp tiếp sau khi dung sai vật lý đã đạt làm target hợp lệ trượt ngân sách
lặp chỉ vì null-space chưa tắt hẳn.

### 3.9 Phát hiện trì trệ — đo tương đối, không tuyệt đối

Điểm số vô hướng:

```text
S(q) = ‖W e(q)‖²
```

Mỗi vòng:

```text
nếu S < S_best − 1e-14:                       # có cải thiện bất kỳ
        meaningful ← S < S_best · (1 − 1e-4)  # cải thiện ĐÁNG KỂ (tương đối)
        S_best ← S ;  q_best ← q
        stagnant ← 0 nếu meaningful, ngược lại stagnant + 1
ngược lại:
        stagnant ← stagnant + 1

nếu stagnant ≥ 30:  q ← q_best ; status ← projected_to_reachable_boundary
```

Hai chi tiết đáng chú ý:

1. **`q_best` cập nhật theo cải thiện bất kỳ**, nhưng **bộ đếm trì trệ chỉ reset
   theo cải thiện đáng kể**. Nhờ vậy nghiệm trả về vẫn là iterate tốt nhất, mà
   vẫn dừng được khi bộ giải chỉ đang bò tiệm cận.
2. **Ngưỡng tương đối `1e-4`, không phải epsilon tuyệt đối.** Điểm số có trọng
   số của một target ngoài tầm cỡ `O(1e3)`; một epsilon tuyệt đối sẽ coi việc bò
   tiệm cận là tiến bộ thật và bộ đếm **không bao giờ** kích hoạt. Tỉ số không
   thứ nguyên nên nó là hằng số của phương pháp số, không phải dung sai vật lý
   ẩn.

Trả về `q_best` chứ **không** phải iterate cuối, để không trả ra một dao động
muộn tuỳ tiện.

> **Bẫy cấu hình:** `_STAGNATION_ITERATIONS = 30` là hằng số cứng. Nếu đặt
> `max_iterations ≤ 30` thì nhánh trì trệ **không bao giờ chạy được**, và mọi
> lần không hội tụ đều mang nhãn `iteration_budget_exhausted`. T007 dùng
> `max_iterations = 40` nên nhánh này có hiệu lực.

### 3.10 Mã giả đầy đủ

```text
solve_upper_body_ik(model, target, q_seed, q_nom, cfg):
    validate(cfg); validate(target)
    q      ← clamp(q_seed)
    q_nom  ← clamp(q_nom)
    W      ← diag([w_pos×6, w_ori×6, w_head×3])
    q_best ← q ;  S_best ← +∞ ;  stagnant ← 0
    status ← "iteration_budget_exhausted"

    for k = 1 .. cfg.max_iterations:
        e ← task_error(model, q, target)
        S ← ‖W e‖²
        if S < S_best − 1e-14:
            meaningful ← S < S_best·(1 − 1e-4)
            S_best ← S ;  q_best ← q
            stagnant ← 0 if meaningful else stagnant+1
        else:
            stagnant ← stagnant + 1

        if tasks_ok(e, cfg):                 # kiểm tra TRƯỚC khi tính J
            status ← "converged" ;  break

        J  ← central_difference_jacobian(model, q, target, cfg.δ)
        H  ← (WJ)ᵀ(WJ) + cfg.λ² I
        try:  J⁺ ← solve(H, (WJ)ᵀ)
        except LinAlgError:  status ← "singular_system" ; break

        Δq ← −J⁺(W e) + cfg.k_post · (I − J⁺WJ)(q_nom − q)
        if ‖Δq‖ > cfg.s_max:  Δq ← Δq · cfg.s_max/‖Δq‖
        q  ← clip(q + Δq, q⁻, q⁺)

        if stagnant ≥ 30:
            q ← q_best ;  status ← "projected_to_reachable_boundary" ; break
    else:
        q ← q_best                            # hết ngân sách lặp

    e_final ← task_error(model, q, target)
    converged ← (status == "converged") ∧ tasks_ok(e_final, cfg)
    margins   ← min(q − q⁻, q⁺ − q)
    clamped   ← { tên khớp : margin ≤ 1e-9 }
    return (q, converged, status, k, residuals(e_final), min margins, clamped)
```

### 3.11 Bốn trạng thái trả về

| `status` | Nghĩa | Dispatch? |
|---|---|---|
| `converged` | đạt toàn bộ dung sai vật lý | có |
| `projected_to_reachable_boundary` | trì trệ, trả iterate gần nhất | chỉ khi bật `allow_projected_position_solution` |
| `iteration_budget_exhausted` | hết ngân sách, vẫn đang cải thiện | như trên |
| `singular_system` | `solve` thất bại | **không bao giờ** |

`converged` được **tính lại** trên `e_final` sau vòng lặp, không tin vào cờ nội
bộ — nếu nhánh trì trệ ghi đè `q` bằng `q_best` thì residual cuối phải phản ánh
đúng `q` được trả về.

### 3.12 Khởi động lại hạt giống

Nằm ở `whole_upper_body.py`, không nằm trong bộ giải.

Seed nối tiếp (nghiệm khung trước) giữ quỹ đạo liên tục, nhưng cũng là **cái
bẫy**: khép tay vào sát người dẫn tới tư thế gập mà nghiệm duỗi trước đó không
vượt sang được. Đo trên đúng mẫu hỏng:

| Seed | Residual vị trí |
|---|---|
| nghiệm khung trước (như chạy thật) | `0.2068 m` |
| `q_nom` | `0.0297 m` |
| tốt nhất trong 40 seed ngẫu nhiên | `0.0129 m` |

Tăng `max_iterations` lên 400, tăng `s_max`, giảm `λ`, giảm trọng số hướng —
**tất cả trả về đúng `0.2068 m` ở vòng 31**. Đây là **cực tiểu địa phương**;
chỉ seed mới thoát được, không phải tham số.

Điều kiện khởi động lại, phải đúng **cả ba**:

```text
1.  cfg.seed_restart_residual_m ≠ None
2.  ¬converged  ∧  max(‖e_L,pos‖, ‖e_R,pos‖) > cfg.seed_restart_residual_m
3.  cả hai target nằm TRONG tầm với
```

Nếu đúng, giải lại **một lần** từ `q_nom` và giữ kết quả có sai số vị trí nhỏ hơn.

**Chặn tầm với** dùng bất đẳng thức tam giác trên hình học asset:

```text
reach_max = Σ ‖t_origin,j‖  (j từ khớp dưới vai đến hết chuỗi)  +  ‖d‖
```

Suy từ URDF chứ **không** phải số tinh chỉnh, và bảo thủ theo cấu tạo: không tư
thế nào đặt EE xa hơn thế. Đo: chặn `0.5828 m` so với `0.5584 m` lấy mẫu thực
tế. Vai được tính tại **tư thế thân đã giải**, nên khoá hay thả eo không đổi ý
nghĩa của phép kiểm.

Chặn này giữ chi phí: bỏ nó thì restart bắn cả vào target thật sự ngoài tầm,
`261.7 ms`/target; có nó thì `149.6 ms` — **không đắt hơn** bản chưa sửa
(`157.0 ms`), vì seed tốt làm các khung sau hội tụ nhanh hơn.

---

## 4. Solver B — một tay, vị trí + roll

`teleop/r1/ik.py`. Bài toán khác hẳn, nên thuật toán cũng khác.

### 4.1 Tách roll ra khỏi bài toán

Chuỗi 5 khớp, nhiệm vụ là 4 vô hướng: 3 vị trí + 1 góc roll cổ tay. Roll **là
toạ độ của chính khớp cuối**, nên nó được **áp trực tiếp**, không giải:

```text
q₄ ← clip(roll*, q⁻₄, q⁺₄)          # khớp wrist_roll
```

Rồi bài toán vị trí giải trên **4 khớp còn lại**:

```text
position_indices = 0..3
```

Vì sao không gộp cả hai vào một hệ bình phương tối thiểu: làm vậy cho phép bộ
giải **đánh đổi sai số roll lấy sai số vị trí**. Phương pháp không cho phép điều
đó — roll là lệnh, không phải thứ để thương lượng.

Còn lại: 4 khớp cho 3 ràng buộc ⇒ dư 1 bậc, giải bằng bias tư thế null-space.

### 4.2 Jacobian giải tích

Khác Solver A, ở đây Jacobian tính **chính xác** bằng công thức hình học. Với
khớp quay `i`:

```text
J[:, i] = a_i,world × ( p_EE − o_i,world )
```

trong đó `a_i,world` là trục khớp và `o_i,world` là gốc khớp, cả hai trong khung
gốc chuỗi. Lấy 3 hàng đầu (vị trí), cột `0..3`.

Rẻ hơn sai phân trung tâm khoảng một bậc, và không có sai số cắt cụt.

> `position_jacobian` vi phân **EE ảo** (đã cộng offset `d`), không phải gốc khớp
> cổ tay. Dùng lệch nhau là lỗi đã từng xảy ra.

### 4.3 Bước đi

Cùng dạng DLS nhưng chỉ 3 hàng:

```text
JJᵀ  = J Jᵀ + λ² I₃
Δq_primary = Jᵀ (JJᵀ)⁻¹ e                     # e = p* − p(q)
P    = I₄ − Jᵀ (JJᵀ)⁻¹ J
Δq   = Δq_primary + k_post · P (q_nom − q)
```

Đây là dạng **hàng đầy** (`JJᵀ` nhỏ, 3×3) thay vì dạng cột đầy của Solver A —
tương đương về mặt toán khi `λ` nhỏ, nhưng rẻ hơn khi số nhiệm vụ < số khớp.

### 4.4 Điều kiện dừng — đo bước **đã áp**, không phải bước **mong muốn**

```text
applied = ‖ q_pos,mới − q_pos,cũ ‖
nếu position_ok ∧ applied ≤ posture_tolerance_rad:  dừng
```

Chi tiết này quan trọng: **tại giới hạn khớp, bias tư thế mong muốn không bao giờ
nhỏ đi**, nhưng thay đổi mà nó tạo ra thì có (vì bị kẹp). Đo bước đã áp nên vòng
lặp kết thúc thay vì đốt hết ngân sách.

Điều kiện `position_ok` không đủ một mình: sai số vị trí đạt dung sai chỉ sau
vài vòng, trong khi bias null-space vẫn đang dịch khuỷu. Dừng ở đó thì khớp dư
động nằm ở đâu tuỳ seed, và hai lần giải cùng target sẽ khác nhau.

### 4.5 Trì trệ và trạng thái

Bộ đếm trì trệ ở đây đo **residual vị trí tuyệt đối**, không phải điểm số tương đối:

```text
nếu residual < best − 1e-10:        best ← residual ; q_best ← q ; stagnant ← 0
ngược lại nếu residual > tol:       stagnant ← stagnant + 1
nếu stagnant ≥ 30:                  q ← q_best ; projected_to_reachable_boundary
```

| `status` | Sinh ra khi |
|---|---|
| `converged` | vị trí và roll đều trong dung sai |
| `tolerance_not_met` | thoát vòng lặp bình thường nhưng residual vẫn quá lớn |
| `projected_to_reachable_boundary` | trì trệ 30 vòng |
| `iteration_budget_exhausted` | hết `max_iterations` |
| `roll_target_clamped_to_limit` | roll yêu cầu nằm ngoài dải khớp |
| `singular_system` | `solve` thất bại |

`roll_target_clamped_to_limit` là trạng thái **riêng của Solver B**: endpoint có
thể vẫn tới đúng chỗ, nhưng hướng đã lệnh **không** được tôn trọng. Nó ép
`converged = False` để việc đó không bị báo cáo là bám hoàn hảo.
`roll_residual_rad` trả về là `max` của sai số thực tế và sai số do kẹp, nên
không giấu phần bị kẹp.

---

## 5. Tính chất số học và chế độ hỏng

### 5.1 Không có mặc định nào trong bộ giải

`ArmIKConfig` và `UpperBodyIKConfig` **bắt buộc mọi trường** — không field nào có
giá trị mặc định. Experiment phải khai báo. Đây là lựa chọn có chủ ý: một con số
mặc định trong file này sẽ lặng lẽ đi vào mọi kết quả hạ nguồn mà không ai kiểm
toán nó.

`validate()` chặn: mọi dung sai và trọng số phải hữu hạn và **dương**;
`max_iterations ≥ 1`; `posture_weight ≥ 0` (cho phép bằng 0 để tắt bias).

### 5.2 Ngân sách lặp **không đơn điệu**

Trên một trace đo được, chất lượng nghiệm **không** cải thiện đều theo số vòng:

| `max_iterations` | `|waist_yaw|` lớn nhất | Tần số |
|---|---|---|
| 12 | `10.2°` | 16.6 Hz |
| 20 | **`136.8°`** | 10.0 Hz |
| 40 | `10.1°` | 5.9 Hz |

12 và 40 đều ổn, 20 phân kỳ. Nguyên nhân: với `max_iterations = 20` và ngưỡng
trì trệ cứng bằng 30, **nhánh trì trệ không bao giờ kích hoạt**, nên bộ giải
chạy hết 20 vòng rồi dừng ở đâu thì dừng — đủ lâu để đi lang thang, chưa đủ lâu
để ổn định.

**Kết luận vận hành: `max_iterations` không phải núm tinh chỉnh an toàn.** Đổi
nó phải đo lại, không được nội suy.

### 5.3 Cực tiểu địa phương là chế độ hỏng chính

Bộ giải là **địa phương**: nó đi xuống từ seed và không có cơ chế thoát hố.
Tư thế gập (khép tay vào người) nằm ở một hố khác với tư thế duỗi. Bằng chứng ở
§3.12. Cách xử lý duy nhất đã được kiểm chứng là **đổi seed**.

### 5.4 Cái mà thuật toán **không** làm

- Không tránh va chạm, không tự va chạm.
- Không xét cân bằng, động lực học, hay mô hình cơ cấu chấp hành.
- Không bù trễ.
- Không đảm bảo liên tục giữa hai khung — tính liên tục đến từ seed nối tiếp và
  rate limiter, không phải từ bộ giải.
- Không đảm bảo toàn cục. Nghiệm trả về là nghiệm địa phương tốt nhất tìm được
  từ seed đã cho.
- Không sinh vận tốc. Đầu ra thuần tuý là **vị trí khớp**; quỹ đạo do
  `OnlineJointLimiter` tạo ra.

### 5.5 Kiểm thử đang có

`tests/teleop/test_r1_upper_body_ik.py` phủ: hai profile URDF, nạp transform và
giới hạn, lệch khung giữa hai asset, các ca biên của SO(3), **kiểm chứng tinh
lọc Jacobian bằng epsilon thứ hai** (không so nó với chính nó), khớp nối eo tới
cả hai cổ tay và đầu, vòng FK→IK, target bất khả thi không hội tụ, giới hạn
khớp, dispatch nguyên tử, hold, reset, và ca hỏng thật ở §3.12 giữ nguyên dưới
dạng số cứng.

Những kiểm thử này chứng minh **nhất quán nội bộ về động học và phần mềm**.
Chúng **không** chứng minh bám quỹ đạo, trễ, khoảng hở va chạm, mô-men, hay giới
hạn an toàn phần cứng — chưa có phép đo nào trên robot thật.
