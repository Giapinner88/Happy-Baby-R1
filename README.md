# Happy Baby R1

Workspace nội bộ AiRA-Laboratory cho Unitree R1. Nhánh `teleop` tập trung vào
Quest 3 → upstream IK → tay/đầu robot, với head anchor ban đầu tách chuyển động
đầu khỏi target tay. Không chia sẻ code, dữ liệu hoặc hình ảnh khi chưa được phép.

## Chạy

Sau khi cài môi trường theo [runbook simulator](docs/teleop/r1_quest3_teleop_sim.md):

```bash
make help
make teleop-dry-run HOST_IP=<IP-workstation>
make teleop HOST_IP=<IP-workstation>
```

Live mở Isaac và ghi video/evidence dưới `experiments/r1_teleop/quest3_sim_v1/T007/runs/`.
Dry-run chỉ in pipeline nhưng có thể tạo cert. `teleop-arms` là alias của `teleop`.

Hardware: theo [runbook robot](docs/teleop/r1_quest3_teleop_hardware.md).
IP robot lấy từ `ROBOT` hoặc `~/.config/hb/robot.env`.
Run sim `t007_whole_upper_body_20260909T072030Z` là tham chiếu hình ảnh;
source alignment đã deploy, tracking hardware sau sửa vẫn cần đo.

## Tra cứu

- [Entrypoint teleop](scripts/teleop/README.md): launcher, replay và diagnostics.
- [Tài liệu](docs/README.md): cài đặt, phương pháp, vận hành và policy.
- [Config T007](experiments/r1_teleop/quest3_sim_v1/T007/config/README.md): upstream chuẩn và đối chứng.
- [Hardware gate](hardware/teleop/docs/hardware_gate.md): giới hạn và evidence còn thiếu.

Code dùng chung ở `teleop/r1/`; triển khai robot ở `hardware/`; model ở
`assets/`; ROS 2 ở `src/`. Vendor `third_party/` giữ nguyên.
Workstation Ubuntu 22.04 dùng Conda theo runbook; robot Ubuntu 20.04 dùng
Python 3.8 / ROS 2 Foxy. Run cũ giữ nguyên config và kết quả của chính nó.
