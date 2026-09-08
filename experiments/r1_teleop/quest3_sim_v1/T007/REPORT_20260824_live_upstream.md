# T007 — Phiên Quest trực tiếp đầu tiên trên đường upstream (2026-08-24)

Run: `runs/t007_whole_upper_body_20260824T063623Z` — `make teleop-arms`, ba tiến trình thật,
headset thật. Đây là mục "chưa làm" số 1 của [REPORT_20260823_upstream.md](REPORT_20260823_upstream.md),
nay đã chạy.

## Quyết định

Bộ giải vendor `R1_A5_ArmIK` chạy nguyên xi được lấy làm **baseline chuẩn** cho teleop tay R1.
Căn cứ là chất lượng chuyển động quan sát trực tiếp: mượt, không giật, khác hẳn các bộ giải
tự viết trong repo. Mọi bộ giải mới từ nay phải so với đường này, không phải ngược lại.

## Số đo phiên live

| | |
|---|---|
| nhịp điều khiển | 30.011 Hz (yêu cầu 30) |
| thời lượng | 88.06 s, dừng bằng SIGINT |
| command nhận | 1650, **0 dòng không hợp lệ** |
| target áp dụng | 1258 / 2643 bước; 1165 hold (`deadman_released`, `command_timeout`) |
| trễ lệnh tươi | trung bình 19.5 ms, max 36.0 ms |
| clamp theo giới hạn asset | **0** |
| vận tốc khớp áp dụng | trung bình 0.83, p95 2.33, **max 5.94 rad/s** — dưới trần asset 18.8 rad/s |
| `sim_to_wall_ratio` | 0.99998 |

**Xung vận tốc của lần e2e trước không tái xuất hiện.** Lần replay 2026-08-23 có 1/1486 mẫu
đạt 20.31 rad/s; phiên live này max 5.94 rad/s trên 1257 mẫu. Chưa đủ để đóng mục đó — chỉ có
nghĩa là nó không phải hiện tượng của mọi phiên.

### Sai số bám tay (tính lại từ evidence, runner chưa ghi sẵn metric này)

FK của repo trên nghiệm vendor, so với target trong `neutral_waist_yaw_link`:

- residual bộ điều khiển: trái p95 185.1 / max 204.9 mm; phải p95 188.0 / max 195.7 mm
- end-to-end (target so với FK của PhysX): trái p95 211.0 / max 223.5 mm; phải p95 213.2 / max 224.9 mm
- phân bố trái: min 2.7, trung bình 94.8, std 54.9 mm — **không phải lệch khung hằng số**, mà là
  sai số bám thật, lớn dần khi với xa (vector trung bình lệch -74.7 mm theo x, tức hụt về phía trước)

Vẫn trên ngưỡng dự án (p95 150 mm). Chấp nhận theo chủ trương "chạy được trước", nhưng chưa đạt
thì chưa được ghi là đạt.

## Đầu nhìn chéo — nguyên nhân và phần đã sửa

Vendor **không hề điều khiển đầu**: `head_pitch_joint`/`head_yaw_joint` chỉ xuất hiện trong danh
sách khớp bị khoá của `robot_arm_ik.py`. Thứ duy nhất vendor tính từ head pose là
`get_Brobot_world_head_yaw_rot`, và nó **cố tình bỏ pitch lẫn roll**, chỉ dùng yaw để đưa target
tay về khung head-yaw. Nghĩa là không có baseline upstream nào cho đầu để copy — toàn bộ phần
đầu là của repo này.

**Lỗi đã tìm ra và sửa.** `R1.urdf` mắc pitch *ngoài* yaw: `head_pitch_joint` treo vào
`waist_yaw_link`, `head_yaw_joint` treo vào `head_pitch_link`. Rotation của đầu vì thế là
`Ry(pitch) @ Rz(yaw)`, trục nhìn là `(cos p cos y, sin y, -sin p cos y)`. Cả hai chỗ tính ngược
lại đều dùng công thức ZYX sách giáo khoa, tức giả định yaw nằm ngoài:

- `scripts/teleop/run_r1_upstream_ik_stream.py::head_angles`
- `teleop/r1/mapping.py::_yaw_pitch` (đường coupled cũng dính)

Hai cách này **trùng nhau khi chỉ yaw hoặc chỉ pitch**, và chỉ lệch khi hai góc cùng khác 0 —
nên nó hiện ra như cái đầu chỉ sai trục chứ không như một con số sai rõ ràng. Đo trên chính
trace phiên live: sai số hướng nhìn trung bình 3.06°, p95 5.63°, max 7.33°, tương quan 0.93 với
`|sin p · sin y|` — đúng dạng cross-coupling. Nghịch đảo đúng theo chuỗi (`y = asin(fy)`,
`p = atan2(-fz, fx)`) cho sai số **0.000°** trên toàn bộ 1255 mẫu.

`R1A5UpperBodyModel.head_rotation()` phía FK vốn đã ghi rõ trong docstring rằng không được thay
bằng `Rz(yaw) @ Ry(pitch)`. Phía FK biết; phía nghịch đảo thì không. Nay ghim bằng test round-trip
trong `tests/teleop/test_r1_upstream_ik_bridge.py` và `tests/teleop/test_r1_teleop.py`, cả hai
đối chiếu thẳng với `head_rotation()` thay vì với một công thức chép tay.

`arcsin` chặn yaw ở ±90° trong khi khớp với tới ±115°. Mất 25° cuối, đổi lấy việc không có nhánh
lật đầu khi người đeo quay quá xa.

## Chưa làm

1. **Yaw của đầu lệch +44.7° gần như suốt phiên** (median 43.7°, std chỉ 6.2°, min 29.4°, max 69.8°).
   Chưa kết luận là lỗi: giả thuyết còn lại là người đeo thật sự quay sang một bên cả phiên. Quyết
   định là **giữ yaw tuyệt đối, không zero**, và xác nhận bằng một phiên có nhìn thẳng trước khi
   đụng vào code. Tay không dính vì vendor đã khử yaw khỏi target tay.
2. Roll của headset tới 42° không biểu diễn được — R1 không có khớp roll ở đầu. Cố hữu, không sửa được.
3. Sai số bám tay vẫn trên ngưỡng (mục trên).
4. Runner chưa tự ghi residual tay cho đường upstream; số ở trên phải tính lại từ evidence.
5. Báo cáo này **không** cho phép actuation R1. Hardware gate chưa tick mục nào.

---

# Phần 2 — Phiên `t007_whole_upper_body_20260824T071530Z`: tay quay ngược khi xoay đầu

## Yaw của đầu: mục treo số 1 ở trên đã giải quyết

Phiên này head yaw mean **8.4°**, median 6.3°, std 24.7°, biên độ -72.6° đến +85.4°. So với phiên
trước (mean 44.7°, std 6.2°), xác nhận **44.7° kia là tư thế người đeo, không phải lỗi gốc toạ độ**.
Giữ yaw tuyệt đối là đúng; không cần zero. Mục "chưa làm" số 1 của Phần 1 đóng lại.

Chỉ số phiên: 30.017 Hz, 57.8 s, 1660 command, 0 dòng không hợp lệ, 0 clamp giới hạn asset,
trễ lệnh trung bình 19.9 ms / max 36.1 ms, vận tốc khớp max 9.91 rad/s (trần asset 18.8).

## Hiện tượng: xoay đầu thì tay chạy ngược chiều

Đo trên trace: phương vị của target cổ tay so với head yaw có **slope -0.51 độ/độ**
(trái -0.512, phải -0.497, corr -0.35). Target dịch ngang **1.8 cm cho mỗi 10° xoay đầu**.
Hệ số không tròn -1.0 vì phương vị đo quanh gốc hông chứ không quanh đầu, và vì tay người
cũng đang chuyển động thật trong lúc đó.

## Đây không phải điểm yếu của bộ giải upstream

Bộ giải `R1_A5_ArmIK` **không bao giờ nhìn thấy head pose**. Nó nhận hai ma trận cổ tay và trả về
góc khớp; không có đường nào để nó biết đầu đang quay.

Nguồn thật là `arm_reference_mode` của TeleVuer, một **tuỳ chọn** vendor cung cấp, và giá trị đang
dùng là do repo này chọn chứ không phải do vendor ép:

```python
# tv_wrapper.transform_IPunitree_Brobot_world_arm_to_head_then_waist
if arm_reference_mode == "head_yaw":
    R = get_Brobot_world_head_yaw_rot(Brobot_world_head[:3, :3])
    arm[:3, 3] = R.T @ (world_arm[:3, 3] - head[:3, 3])   # <-- khử yaw của đầu khỏi target tay
else:  # "head_position"
    arm[:3, 3] = world_arm[:3, 3] - head[:3, 3]           # chỉ trừ vị trí, không xoay
```

Ở chế độ `head_yaw`, target tay được nhân trái bằng `R_head_yaw^T`. Bàn tay đứng yên trong không
gian mà đầu xoay `+Δ` thì target xoay `-Δ` — đúng cái đang thấy. Với vendor thì lựa chọn này nhất
quán, vì robot của họ **không điều khiển đầu**: camera gắn cứng, nên "tay so với hướng nhìn" mới là
khung đúng. Ở đây đầu R1 lại đang bám theo đầu người, nên hai bên khử yaw hai lần.

`head_position` là giá trị vendor hỗ trợ sẵn, chọn bằng tham số khởi tạo `TeleVuerWrapper`.
**Chuyển sang nó không phải là sửa upstream.** Cái phải sửa nằm trong repo này:
`BridgeConfig.__init__` đang chốt cứng `arm_reference_mode == "head_yaw"` như một phần của
frame contract đã audit ([bridge.py](../../../../teleop/r1/bridge.py)), và
`frame_contract.ARM_REFERENCE_MODE` cũng vậy.

Quy mô ảnh hưởng, tính ngược từ chính trace này (phép đổi khả nghịch chính xác, không cần chạy lại):
đổi sang `head_position` làm target dịch **trung bình 101 mm, p95 361 mm, max 561 mm**.

## Quyết định

**Tạm chấp nhận, không đổi.** Ghi lại ở đây để lần sau không ai đi tối ưu bộ giải vì một triệu
chứng mà bộ giải không gây ra. Khi nào muốn thử: đổi `arm_reference_mode` sang `head_position`
trong `BridgeConfig` cùng cái contract đang chốt nó, chạy lại một phiên, so trên cùng một trace.

### Cập nhật 2026-09-08

Quyết định tạm thời trên đã được thay thế cho đường live upstream. Thay vì đổi
sang `head_position` (vẫn làm tay phụ thuộc head translation), streamer giờ
chốt head position/yaw ở mẫu deadman ổn định đầu tiên, đảo transform
current-head của vendor về robot-basis XR world rồi biểu diễn wrist lại theo
anchor ban đầu. Cách này loại cả head translation lẫn yaw khỏi target tay mà
không sửa `third_party` hoặc vector nghiệm của `R1_A5_ArmIK`. Protocol mới được
khai báo trong `r1_t007_upstream_stream_live.json`; các run cũ không comparable.
