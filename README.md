# Happy Baby R1 Teleop

Workspace nội bộ AiRA-Laboratory cho pipeline Meta Quest 3 → Unitree R1-A5.
Nhánh `teleop` hiện chỉ giữ phần phục vụ teleop tay/đầu: transport Vuer, chuẩn
hoá frame, upstream IK, mô phỏng Isaac, adapter phần cứng và sole-owner
`rt/lowcmd`. Các workflow locomotion/training/MuJoCo cũ không còn thuộc nhánh
này.

## Trạng thái

- Pipeline upstream tay/đầu đã có baseline mô phỏng và đường hardware giới hạn.
- Hardware chỉ được thử khi robot treo/cố định, có người giữ E-stop và theo
  hardware gate; chưa có bằng chứng cho phép robot đứng hoặc chạy trên sàn.
- Head anchor ban đầu tách chuyển động đầu khỏi target tay. Tracking phần cứng
  sau các sửa đổi gần nhất vẫn cần đo và ghi evidence mới.
- `hb_high_level` phải là publisher `rt/lowcmd` duy nhất. Sidecar teleop chỉ đọc
  `rt/lowstate` và gửi UTL1 qua loopback.

## Chạy

Đọc [runbook mô phỏng](docs/teleop/r1_quest3_teleop_sim.md), sau đó:

```bash
make help
make teleop-dry-run HOST_IP=<IP-workstation>
make teleop HOST_IP=<IP-workstation>
```

Live mở Isaac và ghi evidence dưới
`experiments/r1_teleop/quest3_sim_v1/baseline/runs/`. Sau mỗi run, artifact gate
tự sinh figures và chỉ trả thành công khi đủ data, figures và video. Dry-run chỉ in pipeline nhưng
có thể tạo certificate.

Hardware phải theo [runbook robot](docs/teleop/r1_quest3_teleop_hardware.md).
IP robot lấy từ `ROBOT` hoặc `~/.config/hb/robot.env`; không hard-code địa chỉ
động vào source.

## Bố cục

- `teleop/r1/`: implementation dùng chung cho schema, mapping, IK và runtime.
- `scripts/teleop/`: entrypoint live, replay, diagnostics và bundle export.
- `experiments/.../baseline/`: cấu hình, record và evidence đối chứng vendor.
- `hardware/teleop/`: sidecar/deploy package; `hardware/high_level_lock/` là
  sole-owner C++ phục vụ pilot treo.
- `assets/`: URDF/USD/mesh R1; `config/`: cấu hình dùng chung không chứa secret.
- `docs/teleop/` và `docs/safety/`: phương pháp, runbook và giới hạn an toàn.
- `third_party/xr_teleoperate*`: hai revision vendor bắt buộc cho transport và
  upstream R1-A5 IK; không sửa trực tiếp.

Xem [bản đồ tài liệu](docs/README.md), [entrypoint teleop](scripts/teleop/README.md),
[baseline config](experiments/r1_teleop/quest3_sim_v1/baseline/config/README.md) và
[hardware gate](hardware/teleop/docs/hardware_gate.md).

Không chia sẻ code, dữ liệu, hình ảnh, certificate hay private key khi chưa
được phép.
