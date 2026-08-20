# Đường tín hiệu teleop R1 — từ Quest tới khớp

Tài liệu này mô tả **code hiện tại**, không phải thiết kế mong muốn. Mỗi mục
đều dẫn tới file và hàm cụ thể để đối chiếu.

Đơn vị dùng thống nhất toàn tuyến: **mét, radian, giây** trên đồng hồ
`time.monotonic()`. Không có nơi nào dùng độ hay milimet.

## 1. Toàn cảnh

```text
Quest 3 (Quest Browser, phiên WebXR immersive)
   │  HTTPS + WSS cổng 8012
   ▼
quest_bridge.py                    env `tv`     30 Hz
   │  QuestCommandBridge.build()   → R1TeleopCommand (JSON 1 dòng / lệnh)
   │  stdout, newline-delimited
   ▼
run_r1_quest3_live.py              env `unitree_sim_env`
   │  R1TeleopMapper.map()         → R1TeleopTargets   (khung robot)
   │  WholeUpperBodyIsaacLabSink   → IK → rate limit
   ├──────────────► Isaac Sim (mô phỏng)
   │
   └─ hoặc nhánh phần cứng:
      run_r1_quest3_hardware_targets.py  → JSONL 12 khớp
         │  ssh
         ▼
      high_level_sidecar.py        trên robot
         │  UDP 127.0.0.1:5560
         ▼
      hb_high_level (run_r1)       ← publisher rt/lowcmd DUY NHẤT
```

Hai môi trường Conda không dùng chung được một trình thông dịch (`vuer` xung
đột với IsaacLab), nên bridge và simulator là hai tiến trình nối bằng pipe.
Cả hai chạy trên một máy nên `time.monotonic()` là cùng một trục thời gian;
tuổi lệnh được **đo trực tiếp**, không ước lượng.

---

## 2. Đầu vào: Quest → lệnh chuẩn hoá

`teleop/r1/bridge.py`

### 2.1 Mẫu thô từ vendor

`QuestTransportSample` tách khỏi kiểu dữ liệu của vendor wrapper:

| Trường | Ý nghĩa |
|---|---|
| `motion_data_ready` | vendor đã nhận event controller/hand chưa |
| `head_pose_matrix` | ma trận 4×4 |
| `left_wrist_pose_matrix` | ma trận 4×4 |
| `right_wrist_pose_matrix` | ma trận 4×4 |
| `deadman_pressed` | cò phải đang giữ |
| `reset_requested` | cò trái (cạnh lên) |

Ma trận ở **cơ sở robot mà vendor wrapper đã đổi sẵn**. Module này không đổi
cơ sở nào của riêng nó (`bridge.py` dòng 8–10).

### 2.2 Bốn lớp kiểm tra trước khi phát lệnh

`QuestCommandBridge.build()` fail-closed: sai bất kỳ lớp nào thì **không phát
lệnh nào cả**, chứ không phát lệnh "trung tính".

1. **`motion_data_ready` false** → `dropped_sample_count++`. Mở trang web thôi
   chưa đủ; phải vào phiên immersive mới có dữ liệu.
2. **Ma trận không hợp lệ** → `rejected_sample_count++`:
   - không phải 4×4, hoặc chứa giá trị không hữu hạn;
   - `_orthonormality_defect` > `max_orthonormality_defect` (mặc định `1e-3`);
   - `|det(R) − 1|` > `max_rotation_determinant_error` (mặc định `1e-3`),
     bắt cả phép phản chiếu.
3. **Pose đứng yên** quá `max_pose_stale_s` (mặc định `0.5 s`) →
   `dropped_sample_count++` và đánh dấu `disconnected`.
4. Qua hết → phát `R1TeleopCommand`, `sequence_id` **tăng nghiêm ngặt**.

Lớp 3 tồn tại vì cờ `motion_data_ready` của vendor **latch**: đặt true ở event
đầu tiên và không bao giờ xoá. Tháo kính ra thì wrapper vẫn tiếp tục phát mẫu
mang pose cũ. So sánh pose trùng khít từng bit là quan sát **duy nhất** phân
biệt được hai tình huống. Ngưỡng `0.5 s` có căn cứ đo: trong run
`t001_b_20260802T111348Z`, chuỗi đứng yên dài nhất khi vận hành thật là
`0.2 s`, còn khi tháo kính là `5.9 s`.

> Đọc `dropped_sample_count` cho đúng: nó **không phải** mất gói. Nó gộp cả
> thời gian chờ người vận hành vào VR. Đánh giá kết nối bằng `connect_count`,
> `disconnect_count` và `rejected_sample_count`.

### 2.3 Lược đồ lệnh

`teleop/r1/schema.py`, `SCHEMA_VERSION = 1`. Lược đồ **không chứa kiểu DDS**
nào.

```python
R1TeleopCommand(
    sequence_id, timestamp_monotonic_s, deadman_enabled,
    head_pose, left_wrist_pose, right_wrist_pose,   # Pose = Vector3 + Quaternion
    base_velocity,                                  # luôn 0 ở v1
    source_frame="quest_headset", schema_version=1, reset_requested=False)
```

`Pose.from_dict` **chuẩn hoá quaternion ngay khi nạp**, nên phía sau không cần
lo norm. `from_dict` từ chối `schema_version` khác 1, `sequence_id` âm và
`timestamp_monotonic_s` âm.

`base_velocity` bị ép bằng 0 tại nguồn: `BridgeConfig.base_velocity_source`
chỉ nhận `"constant_zero"`, constructor ném lỗi nếu khác. Lý do ghi thẳng
trong code: chưa có cấu hình nào được duyệt cung cấp giới hạn vận tốc cho R1,
bịa ra một con số là đưa số chưa kiểm toán vào luồng lệnh.

---

## 3. Mapping: khung nguồn → khung robot

`teleop/r1/mapping.py`, `R1TeleopMapper.map()`

### 3.1 Ba cổng chặn, theo đúng thứ tự

| Điều kiện | `reason` trả về |
|---|---|
| `source_frame` khác cấu hình | `source_frame_mismatch` |
| `received − timestamp > command_timeout_s` | `command_timeout` |
| `deadman_enabled` false | `deadman_released` |

Khi bị chặn, `_disabled()` trả về `R1TeleopTargets` với `enabled=False`,
`left/right_wrist_target=None`, head 0, velocity 0. **Không phải lệnh giữ
nguyên** — sink mới là nơi quyết định giữ.

`command_timeout_s` = `0.5 s` trong `T001/config/r1_quest3_sim_v1.json`, bằng
đúng `max_pose_stale_s` của bridge, để hai lớp fail-closed hiểu "cũ" giống
nhau.

### 3.2 Phép biến đổi

`_transform_pose()` áp **calibration**: xoay quanh trục z một góc
`yaw_rad` rồi tịnh tiến `translation_m`.

```text
p' = Rz(yaw)·p + t
q' = q_yaw ⊗ q
```

Cấu hình T001 hiện tại đặt `translation_m = [0,0,0]` và `yaw_rad = 0`, tức
**biến đổi đồng nhất**. Calibration đang có mặt trong code nhưng chưa được
dùng để bù lệch thật.

### 3.3 Head: quaternion → yaw/pitch

`_yaw_pitch()` trích từ quaternion đã chuẩn hoá:

```text
yaw   = atan2(2(wz + xy), 1 − 2(y² + z²))
pitch = asin(clamp(2(wy − zx), −1, 1))
```

`asin` được clamp trước để không NaN ở biên. **Head roll bị bỏ**, không có
đường nào mang nó đi tiếp.

Kết quả bị kẹp vào `head_yaw_range_rad` (mặc định ±π) và
`head_pitch_range_rad` (mặc định ±π/2).

### 3.4 Quyền sở hữu khớp

`R1A5WholeUpperBodyOwnership` chia khớp theo `body_mode`, và `validate()` ném
lỗi nếu hai nhóm giao nhau. Đây là cơ chế ngăn policy locomotion và IK cùng
ghi một khớp.

| `body_mode` | Khớp thân trên | DoF |
|---|---|---|
| `arms_head` | 2 tay + đầu | 12 |
| `waist_yaw` | thêm `waist_yaw_joint` | 13 |
| `full_upper_body` | thêm `waist_roll_joint` | 14 |

Khớp nào không thuộc thân trên thì thuộc `lower_body`, luôn luôn, không có
khớp "vô chủ".

---

## 4. Mô hình động học

`teleop/r1/upper_body_kinematics.py`, `teleop/r1/kinematics.py`

### 4.1 Đọc từ URDF, không chép tay

Hình học lấy trực tiếp từ `assets/R1.urdf` bằng regex, nên đổi asset không thể
âm thầm làm sai một workspace đã đo. Hai trong năm gốc khớp vai mang `rpy`
khác 0 (±0.26187 rad) — chép tay kiểu "chỉ tịnh tiến" sẽ mất phần này.

Mỗi khớp quay:

```text
T_j(q_j) = T_origin,j · Rot(axis_j, q_j)
```

`_rpy_to_matrix` theo quy ước URDF trục cố định XYZ: `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.

### 4.2 Khung và điểm điều khiển

- Gốc chuỗi tay: `waist_yaw_link`
- Mọi target và FK biểu diễn trong `pelvis_link`
- **End-effector là frame ảo của vendor R1-A5: 0.20 m dọc trục +x cục bộ của
  `wrist_roll_link`**, không phải gốc khớp cổ tay

Điểm cuối cùng này từng là lỗi: Jacobian vi phân gốc cổ tay trong khi FK dùng
tool offset thì hệ tự mâu thuẫn, khiến target vươn xa bị kẹt cách đích vài
centimet. `position_jacobian()` nay vi phân đúng điểm EE ảo.

### 4.3 Thứ tự khớp (thẩm quyền)

`waist_yaw` mode, 13 giá trị:

```text
[waist_yaw,
 left:  shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll,
 right: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll,
 head_pitch, head_yaw]
```

Chú ý **head_pitch đứng trước head_yaw**. Các trace bằng chứng cũ lại dùng
`[head_yaw, head_pitch]`, nên sink ghi cả hai dạng (`head_pitch_yaw_target_rad`
và `head_target_rad`) để không phải suy đoán khi đọc lại.

`arms_head` bỏ phần tử đầu; `full_upper_body` chèn `waist_roll` lên đầu. Chỉ
số không hard-code: `waist_yaw_index`, `left_arm_slice`, `right_arm_slice`,
`head_slice` tự suy ra từ mode.

### 4.4 Target đầu phải khả thi theo cấu tạo

`head_rotation(pitch, yaw)` trả về FK của chính chuỗi đầu:

```python
T = joint_transform(head_pitch, pitch) @ joint_transform(head_yaw, yaw)
```

**Không được thay bằng `Rz(yaw) @ Ry(pitch)`.** Chuỗi R1 áp pitch trước yaw;
hai thứ tự khác nhau bất cứ khi nào cả hai góc khác 0. Lỗi này từng làm target
đầu bất khả thi, đẩy residual lên 0.12 rad so với dung sai 0.03, và khiến bộ
giải xoắn `waist_yaw` tới 108° để đuổi theo.

---

## 5. IK: bài toán và cách giải

`teleop/r1/upper_body_ik.py`, `solve_upper_body_ik()`

### 5.1 Vector sai số nhiệm vụ (15 phần tử)

```text
e(q) = [ pL* − pL(q),                 3   vị trí cổ tay trái
         pR* − pR(q),                 3   vị trí cổ tay phải
         Log(RL*·RL(q)ᵀ),             3   hướng cổ tay trái
         Log(RR*·RR(q)ᵀ),             3   hướng cổ tay phải
         Log(RH*·RH(q)ᵀ) ]            3   hướng đầu
```

`so3_log` xử lý riêng hai nhánh số học: góc nhỏ (`< 1e-8`) dùng xấp xỉ tuyến
tính, gần π (`π − θ < 1e-5`) dùng đường chéo ma trận và pivot theo trục lớn
nhất. `_proper_rotation` từ chối ma trận không trực giao hoặc `det ≠ +1`.

### 5.2 Trọng số

Mỗi nhóm nhân trọng số rồi mới bình phương:

| Nhóm | Khoá cấu hình | Giá trị T007 |
|---|---|---|
| vị trí (6) | `position_weight` | `10.0` |
| hướng cổ tay (6) | `wrist_orientation_weight` | `0.1` |
| hướng đầu (3) | `head_orientation_weight` | `0.5` |

Hướng cổ tay cố tình yếu và dung sai đặt `π`: mỗi tay chỉ có **5 khớp**, không
thể đồng thời đạt vị trí 3-DoF và hướng 3-DoF tuỳ ý. Hướng cổ tay là **khớp
tốt nhất có trọng số**, được ghi lại chứ không dùng để chấp nhận.

### 5.3 Jacobian

`upper_body_task_jacobian()` — **sai phân trung tâm**, không giải tích:

```text
J[:, i] = ( e(q + δeᵢ) − e(q − δeᵢ) ) / (2δ)      δ = finite_difference_rad = 1e-5
```

Kích thước `15 × dof`. Mỗi cột cần 2 lần FK ⇒ **26 lần FK mỗi vòng lặp** ở
13 DoF. Đo trên máy trạm: FK 0.147 ms, Jacobian 5.163 ms, tức Jacobian chiếm
~96% chi phí một vòng lặp. Đây là nút thắt tốc độ, không phải đường truyền.

### 5.4 Bước lặp

Damped least squares có chiếu null-space:

```text
H       = JᵀWᵀWJ + λ²I                  λ = damping = 0.02
J⁺      = H⁻¹ (WJ)ᵀ
Δq      = −J⁺·(W·e)  +  posture_weight · (I − J⁺WJ)·(q_nominal − q)
nếu ‖Δq‖ > max_joint_step_rad:  Δq ← Δq · max_joint_step_rad / ‖Δq‖
q       ← clip(q + Δq, lower, upper)
```

- **Giảm chấn λ** giữ bước hữu hạn ở điểm kỳ dị, nơi pseudo-inverse thuần sẽ
  đòi bước vô hạn.
- **Null-space** chỉ đưa tư thế về `nominal` theo thành phần **không** làm hỏng
  nhiệm vụ chính. Đây là cách giải dư động (5 khớp cho 4 ràng buộc) sao cho
  hai lần giải cùng một target ra cùng một kết quả.
- **Kẹp cứng** vào giới hạn URDF ở **mọi** vòng lặp, không phải chỉ ở cuối.

Hệ quả cần biết: vì bias tư thế bị chiếu null-space, khi nhiệm vụ **bất khả
thi** thì null-space gần như không còn, nên tăng `posture_weight` gấp 10 lần
gần như **không** kéo được `waist_yaw` về (đo được: 73° → 56°).

### 5.5 Điều kiện dừng

```text
tasks_ok = max(‖eL_pos‖, ‖eR_pos‖) ≤ position_tolerance_m
        và max(‖eL_ori‖, ‖eR_ori‖) ≤ wrist_orientation_tolerance_rad
        và ‖eH_ori‖              ≤ head_orientation_tolerance_rad
```

Tư thế nominal là **chi phí phụ, không phải điều kiện chấp nhận**. Trước đây
lặp tiếp sau khi dung sai vật lý đã đạt làm target hợp lệ trượt ngân sách lặp
chỉ vì null-space chưa tắt hẳn.

Trạng thái trả về:

| `status` | Nghĩa |
|---|---|
| `converged` | đạt toàn bộ dung sai vật lý |
| `projected_to_reachable_boundary` | điểm gần nhất tìm được, sau khi trì trệ |
| `iteration_budget_exhausted` | hết `max_iterations`, vẫn đang cải thiện |
| `singular_system` | `np.linalg.solve` thất bại |

Trì trệ: sau **30** vòng liên tiếp không cải thiện điểm số
(`_STAGNATION_ITERATIONS`, hằng số cứng trong module), trả về `best_q` —
iterate tốt nhất từng thấy — chứ **không** phải iterate cuối, để không trả về
một dao động muộn tuỳ tiện.

Hệ quả của ngưỡng cứng này: nếu cấu hình đặt `max_iterations ≤ 30` thì nhánh
trì trệ **không bao giờ chạy được**, và mọi lần không hội tụ sẽ mang nhãn
`iteration_budget_exhausted`. Cấu hình T007 hiện dùng `max_iterations = 40`
nên nhánh này có hiệu lực.

### 5.6 Khởi động lại hạt giống

`whole_upper_body.py` — `_should_restart()`

Seed nối tiếp (nghiệm khung trước) là thứ giữ quỹ đạo liên tục, nhưng cũng là
cái bẫy: **khép tay vào sát người** dẫn tới tư thế gập mà nghiệm duỗi trước đó
không vượt sang được, và bước DLS không leo ra khỏi hố.

Đo trên đúng mẫu hỏng:

| Seed | Residual |
|---|---|
| nghiệm trước (như chạy thật) | 0.2068 m |
| nominal | 0.0297 m |
| tốt nhất trong 40 seed ngẫu nhiên | 0.0129 m |

Tăng `max_iterations` lên 400, tăng bước, giảm damping, giảm trọng số hướng —
**tất cả đều trả về đúng 0.2068 m ở vòng 31**. Chỉ seed mới đổi được kết quả.

Điều kiện khởi động lại, phải đúng cả ba:
1. `seed_restart_residual_m` khác `None`;
2. chưa hội tụ và sai số vị trí **vượt** ngưỡng đó;
3. **cả hai** target nằm trong tầm với — `_within_reach()`.

Tầm với dùng `ArmChain.max_reach_from_shoulder_m`: bất đẳng thức tam giác trên
các tịnh tiến link dưới vai cộng tool offset. **Suy từ asset, không phải số
tinh chỉnh**, và bảo thủ theo cấu tạo (0.5828 m so với 0.5584 m lấy mẫu). Vai
được tính tại **tư thế thân đã giải**, nên khoá hay thả eo không đổi ý nghĩa
của phép kiểm.

Cổng tầm với là thứ giữ chi phí: bỏ nó thì restart bắn cả vào target thật sự
ngoài tầm, 261.7 ms/target; có nó thì 149.6 ms, tức **không đắt hơn** bản
chưa sửa.

---

## 6. Vận tốc và quỹ đạo

`teleop/r1/rate_limit.py`, `OnlineJointLimiter`

IK trả về **vị trí**, không trả vận tốc. Quỹ đạo do rate limiter sinh ra, bậc
hai:

```text
v_req  = clip((q_goal − q) / dt,        ±max_velocity_rad_s)
v      = v + clip(v_req − v,            ±max_acceleration_rad_s2 · dt)
v      = clip(v,                        ±max_velocity_rad_s)
q_next = q + v·dt
```

Ba chi tiết quan trọng:

1. **Chống vọt lố**: nếu `(goal − q)·(goal − q_next) ≤ 0` thì đặt thẳng
   `q = goal`, `v = 0`. Không dao động quanh đích.
2. **Kẹp giới hạn khớp** ở chính limiter, **không** thừa so với việc IK đã
   kẹp: limiter có quán tính, vận tốc tích luỹ có thể mang lệnh vượt biên kể
   cả khi mọi target yêu cầu đều nằm trong biên.
3. **Xoá vận tốc của khớp bị kẹp**. Giữ lại thì nó cứ tích phân ép vào giới hạn
   và làm chậm lúc đảo chiều.

`hold()` chỉ xoá vận tốc, **giữ nguyên vị trí** — đó là hành vi "giữ nguyên tư
thế" khi mất tín hiệu.

Giá trị T007 hiện tại: `max_joint_velocity_rad_s = 1.5`,
`max_joint_acceleration_rad_s2 = 4.0`, nguồn ghi trong `rate_limit_source` —
lấy từ replay offline của schema-2, **là tinh chỉnh mô phỏng, không phải giới
hạn an toàn phần cứng**.

Sink còn từ chối khởi động nếu vận tốc khai báo vượt bất kỳ giới hạn khớp nào
trong URDF đã chọn.

**Không có nội suy quỹ đạo.** Mỗi chu kỳ điều khiển lấy lệnh **mới nhất** và
vứt phần còn lại; rate limiter là thứ duy nhất tạo ra tính liên tục.

---

## 7. Đầu ra

### 7.1 Mô phỏng

`WholeUpperBodyIsaacLabSink.apply_upper_body()` gửi **nguyên tử**: hoặc cả
`dof` khớp cùng nhận target mới, hoặc **toàn bộ** giữ `last_target`. Không có
trạng thái nửa vời.

Điều kiện chấp nhận:

```python
projected = allow_projected_position_solution and status in {
    "projected_to_reachable_boundary", "iteration_budget_exhausted"}
usable = converged or allow_nonconverged_solution or projected
```

`singular_system` **không bao giờ** đi qua `projected` — lỗi giải thật vẫn giữ.

Vòng điều khiển ở `run_r1_quest3_live.py` rút cạn hàng đợi mỗi chu kỳ, bỏ lệnh
có `sequence_id` không tăng, và chỉ giữ `newest`. Đây là lý do bridge 30 Hz và
vòng 10–20 Hz không tạo hàng tồn: tuổi lệnh đo được 17.6 ms trung bình.

### 7.2 Phần cứng

`teleop/hardware/high_level_sidecar.py` (hiện **chỉ có** trong
`hardware/teleop/src/` và trên robot, **không có** trong `teleop/` của
workspace).

Sidecar **không tạo publisher DDS nào**. Nó subscribe `rt/lowstate` chỉ để đọc
chế độ và watchdog. Đầu ra duy nhất là **một gói UDP loopback**:

```text
đích   : 127.0.0.1:5560
struct : "<IIBBBB12f"   magic 0x314C5455, sequence, 3 cờ, padding, 12 float
khớp   : 12 khớp arms_head, motor index (15,16,17,18,19, 22,23,24,25,26, 29,30)
```

`hb_high_level` vẫn là **publisher `rt/lowcmd` duy nhất**.

Ràng buộc biên độ — điểm an toàn cốt lõi:

```python
desired = start_q + clip(source − source_zero, ±max_offset_rad)
```

`start_q` là tư thế đo được **lúc sidecar khởi động**; `source_zero` là mẫu
teleop đầu tiên. Nghĩa là robot chỉ di chuyển **lệch tương đối** so với tư thế
lúc bắt đầu, tối đa `max_offset_rad` (mặc định `0.15 rad ≈ 8.6°`, khoá trong
khoảng `0.02–0.30`). Sai số tuyệt đối của IK **không** truyền thẳng xuống motor.

Bốn watchdog, chạm cái nào là dừng:

| Điều kiện | `stop_reason` | `status` |
|---|---|---|
| không lệnh > `input_timeout_s` (0.75 s) | `input_watchdog` | completed |
| không lowstate > `state_timeout_s` (0.20 s) | `lowstate_watchdog` | **failed** |
| `mode_machine` đổi khác `expected_mode_machine` | `mode_machine_changed` | **failed** |
| stdin đóng | `stream_closed` | completed |

---

## 8. Bảng kiểm tra hợp lệ

| Tầng | Kiểm tra | Hỏng thì |
|---|---|---|
| bridge | `motion_data_ready` | không phát lệnh |
| bridge | 4×4, hữu hạn | không phát lệnh, `rejected++` |
| bridge | trực giao ≤ 1e-3 | không phát lệnh, `rejected++` |
| bridge | \|det−1\| ≤ 1e-3 | không phát lệnh, `rejected++` |
| bridge | pose đổi trong 0.5 s | không phát lệnh, `disconnected` |
| schema | `schema_version == 1` | `ValueError` |
| schema | seq ≥ 0, timestamp ≥ 0 | `ValueError` |
| schema | quaternion norm ≠ 0 | `ValueError` |
| mapper | `source_frame` khớp | `enabled=False` |
| mapper | tuổi ≤ `command_timeout_s` | `enabled=False` |
| mapper | deadman đang giữ | `enabled=False` |
| runner | `sequence_id` tăng nghiêm ngặt | bỏ dòng, ghi `invalid_lines` |
| sink | quyền sở hữu đủ khớp | `ValueError` (lỗi lập trình) |
| sink | có cả hai target cổ tay | giữ toàn bộ |
| sink | nominal đúng độ dài mode | `ValueError` lúc cấu hình |
| sink | nominal trong giới hạn URDF | `ValueError` lúc khởi tạo |
| sink | vận tốc ≤ giới hạn URDF | `ValueError` lúc khởi tạo |
| IK | ma trận xoay hợp lệ | `UpperBodyIKConfigError` |
| IK | dung sai vật lý | `converged` false → sink quyết định |
| limiter | giới hạn khớp | kẹp + xoá vận tốc khớp đó |
| sidecar | `mode_machine` | dừng, `failed` |
| sidecar | lowstate tươi ≤ 0.20 s | dừng, `failed` |
| sidecar | lệnh tươi ≤ 0.75 s | dừng |
| sidecar | biên độ ≤ `max_offset_rad` | kẹp |

Nguyên tắc xuyên suốt: **không tầng nào bịa ra lệnh trung tính**. Fail-closed
nghĩa là im lặng hoặc giữ nguyên, để tầng sau tự phát hiện khoảng trống bằng
watchdog của chính nó.

---

## 9. Giới hạn đã biết

- **Không có ngưỡng số nào có mặc định trong code giải.** `ArmIKConfig` và
  `UpperBodyIKConfig` bắt buộc mọi trường; experiment khai báo. Cố ý như vậy để
  không đưa số chưa kiểm toán vào kết quả.
- Hướng cổ tay là khớp tốt nhất, không phải ràng buộc. 5 khớp không thể đạt
  pose 6-DoF tuỳ ý.
- Head roll không được điều khiển và bị bỏ ngay ở mapper.
- Calibration hiện là biến đổi đồng nhất; chưa bù lệch thật.
- Jacobian sai phân trung tâm chiếm ~96% chi phí mỗi vòng lặp. Jacobian giải
  tích là hướng tăng tốc rõ ràng nhất, chưa làm.
- Ngân sách lặp **không đơn điệu**: trên một trace đo được, 12 và 40 vòng đều
  ổn (xoắn eo ~10°) nhưng 20 vòng phân kỳ tới 136.8°. Đây **không phải** núm
  tinh chỉnh an toàn.
- `teleop/hardware/` (sidecar phần cứng) **không có nguồn trong workspace**:
  nó chỉ tồn tại ở `hardware/teleop/src/teleop/hardware/` và trên robot.
  `sync_from_workspace.sh` dùng `rsync --delete` nhưng đã có
  `--exclude 'hardware/'` nên sync **không xoá** nó. Đổi lại, phần này nằm
  ngoài nguyên tắc "một nguồn sự thật" mà `sync_from_workspace.sh` đặt ra cho
  `teleop/r1/`: sửa sidecar phải sửa trực tiếp trong package.
- Toàn bộ số liệu ở tài liệu này là **mô phỏng và replay offline**. Chưa có
  run phần cứng nào xác nhận bám quỹ đạo, độ mượt hay tốc độ vòng điều khiển.
