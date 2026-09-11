# R1 Quest 3 teleop trong Isaac

Đây là đường mô phỏng canonical cho baseline. Pipeline nhận pose Quest, biểu diễn
wrist theo head anchor ban đầu, giải tay bằng upstream `R1_A5_ArmIK`, ánh xạ
head pitch/yaw và áp 12 joint targets vào R1 trong Isaac. Không tiến trình nào
trên đường này import Unitree DDS hoặc gửi lệnh robot.

## Điều kiện

- Chạy từ root repository.
- Environment `tv` có Vuer, CasADi và Pinocchio 3 bindings cho transport/IK.
- Environment Isaac được launcher gọi theo cấu hình hiện tại.
- Certificate và private key khớp SAN của `HOST_IP`; không đưa chúng vào Git.
- Cấu hình transport dùng chung: `config/r1_quest3_sim.json`.
- Cấu hình solver active: `experiments/r1_teleop/quest3_sim_v1/baseline/config/`.

## Chạy

Kiểm tra pipeline và path trước:

```bash
make teleop-dry-run HOST_IP=192.168.1.106
```

Chạy baseline upstream:

```bash
make teleop HOST_IP=192.168.1.106
```

Launcher tạo run bất biến dưới
`experiments/r1_teleop/quest3_sim_v1/baseline/runs/`. Giữ head và controllers ở
tư thế neutral trong ba mẫu deadman đầu; mẫu thứ ba chốt head position/yaw cho
cả session. Nhả rồi bóp lại deadman không đổi anchor; restart pipeline nếu cần
calibrate lại.

Tạo file stop theo path launcher in ra hoặc dùng `Ctrl+C`, sau đó chờ thông báo
finalize evidence trước khi đóng terminal. Launcher tự sinh `figures/`, giữ
`simulator_view.mp4` và viết `artifact_manifest.json`; thiếu data, figures hoặc
video làm artifact gate trả lỗi.

## Diagnostics

`capture_quest_transport.py` là diagnostic transport và mặc định ghi
`results/smoke/`; nó không phải canonical run và không thay cho baseline evidence.
`plot_r1_quest3_telemetry.py` tạo derived data từ run đã có và không được sửa
artifact nguồn.

Các report T007 cũ dưới `baseline/history/` chỉ diễn giải evidence lịch sử;
launcher active không còn cung cấp coupled/differential/offline workflow.

Kết quả mô phỏng chỉ thiết lập simulation behavior. Nó không đóng
[hardware gate](../../hardware/teleop/docs/hardware_gate.md) và không cấp quyền
chạy robot thật.
