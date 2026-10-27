# Nhóm policy ArmFree — checkpoint cuối 14999

Cả bốn bản: no mirror, 4096 env, 15000 iteration, settle 0.5 s, chu kỳ phase 0.6 s.
Tên trong cột đầu là giá trị chép vào `walk_policy` hoặc `teleop_policy` của
`controller/config/locomotion.yaml`.

| Tên chọn trong config | Run | vx tối đa train | Damping chân | Action gối / cổ chân |
| --- | --- | --- | --- | --- |
| `armfree/nomirror_d20` | AB_NoMirror_D20 | 1.5 m/s | 2.0 tất cả 12 khớp | 0.1575 / 0.328125 |
| `armfree/nomirror_d25` | AB_NoMirror_D25 | 1.5 m/s | gối/cổ chân 2.5, hông 2.0 | 0.1575 / 0.328125 |
| `armfree/speed20_oldaction_d20` | Speed20_OldAction | 2.0 m/s | 2.0 tất cả 12 khớp | 0.1575 / 0.328125 |
| `armfree/speed20_nominal_d20` | Speed20_NominalAction | 2.0 m/s | 2.0 tất cả 12 khớp | 0.15 / 0.3125 |

Ví dụ dùng chung một bản cho hai phím chọn policy:

```yaml
walk_policy: armfree/nomirror_d20
teleop_policy: armfree/nomirror_d20
```

Đây là ví dụ lựa chọn, chưa áp dụng vào config active. Có thể chọn hai bản khác
nhau nếu thỏa kiểm tra default pose của runtime. Teleop H4 của checkout này được
xác định theo contract; cờ `teleop_h4` cũ không quyết định quyền nạp bundle.
Chọn Speed20 không tự tăng giới hạn `slow_*` / `fast_*` của HB.

Mỗi thư mục con chứa `policy.onnx`, `policy.yaml` (contract + SHA-256),
`packaging.json` (checkpoint, hash nguồn, PD/action/default pose chính xác và kết
quả đối chiếu actor), cùng README nguồn. ONNX được chép nguyên byte từ gói đã
kiểm tra. `hb_bundle` trong ONNX và `bundle` trong manifest giữ ID gói gốc dài;
`installation.selector` ghi đường dẫn nhóm ngắn dùng tại HB.

Runtime đi kèm hỗ trợ đúng phase liên tục từ reset, mask khi norm command <0.1,
và FSM RMS 12 khớp chân. Các nhánh được chọn bằng metadata. Các policy cũ và mọi
giá trị tuning active được giữ nguyên khi thêm nhóm này.

Hai bản Swing sửa vung chân bị loại do lỗi W2S, không nằm trong nhóm này. Các bản
Speed20 ở đây vẫn còn hạn chế vung chân ngang khi chạy nhanh và lệch tay thoáng qua
khi dừng; xem nguồn đánh giá trong `packaging.json`. Đây là gói đã kiểm trong mô
phỏng và contract runtime, chưa nghiệm thu chạy robot.

Trong repo training `unitree_rl_mjlab_gait_pro`, cài lại nhóm từ các gói đã kiểm:

```bash
python scripts/package_armfree_locomotion_hb.py --install-selected-grouped \
  --hb-root /home/ubuntu22/train_mujoco/robot_r1/Happy-Baby-R1-develop
```

Lệnh trên cài artifact; runtime hỗ trợ contract và nhóm đi cùng thay đổi HB này.
Tại HB, chạy `./hb policy list` để xem lựa chọn; build bằng `./hb build` và kiểm
config đã chọn bằng `./hb preflight` trên máy trạm. Hồ sơ cài và kiểm tra nằm trong
`unitree_rl_mjlab_gait_pro/artifacts/hb_grouped_locomotion_20261026/`.
