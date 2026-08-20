# HB R1 Teleop

Quest 3 teleoperation cho R1: bridge nhận pose từ headset, giải IK phần thân
trên, và ghi joint target xuống robot.

> **Trạng thái: CHƯA ĐƯỢC PHÉP CHẠY TRÊN ROBOT THẬT.**
> Bộ giải IK có bằng chứng mô phỏng trong `experiments/r1_teleop/quest3_sim_v1/`
> và một smoke test phần cứng giới hạn khi robot treo; chưa có bằng chứng cho
> robot đứng trên sàn. `hb_teleop.service` mặc định **không** được enable và
> hardware gate trong `docs/hardware_gate.md` vẫn chưa đóng đủ.

Cập nhật 2026-08-18: direct-lowcmd smoke ban đầu đã xác nhận 12 encoder có thể
chuyển động. Kiến trúc active sau D003 dùng sidecar không có DDS publisher và
giữ `hb_high_level` làm chủ `rt/lowcmd` duy nhất; kiến trúc này còn cần bounded
hardware validation. Kết quả không cho phép chạy robot trên sàn. Quy trình nằm tại
`../../docs/operations/r1_quest3_teleop_hardware.md`.

## Nguồn gốc

Logic IK/mapping là của `Happy-Baby-R1/teleop/r1/`. Package này **không**
sao chép lại thuật toán; `scripts/sync_from_workspace.sh` đồng bộ chúng vào
`src/teleop/` để giữ một nguồn sự thật duy nhất. Package nằm ngay trong
workspace nên nguồn được suy ra từ vị trí thư mục; không cần đặt biến.

Method record: `Happy-Baby-R1/docs/teleop/r1_upper_body_ik.md`

## Bố cục

```text
scripts/    sync, check_vuer, preflight, deploy, install_service
config/     robot.env.example, teleop.env.example, profile IK
systemd/    hb_teleop.service.in (KHÔNG tự enable)
src/        teleop runtime (đồng bộ từ workspace)
tests/      kiểm tra tĩnh: an toàn, ownership, không có DDS ngoài ý muốn
docs/       cổng phần cứng và quy trình vận hành
```

## Chuẩn bị (máy dev)

```bash
mkdir -p ~/.config/hb
cp config/robot.env.example ~/.config/hb/robot.env   # sửa ROBOT cho đúng máy
./scripts/sync_from_workspace.sh                     # kéo teleop/ từ workspace
./scripts/preflight.sh                               # kiểm tra tĩnh, chạy được ở máy dev
```

## Kiểm tra đường Quest trước khi đeo kính

```bash
./scripts/check_vuer.sh
```

Kiểm tra host IP, chứng chỉ (SAN + hạn), cổng 8012 và môi trường `tv`. Chạy cái
này trước; đeo kính lên rồi mới phát hiện hỏng thì mất thời gian hơn nhiều.

## Xem trước rồi mới đẩy

```bash
./scripts/deploy_teleop.sh diff     # rsync --dry-run, KHÔNG ghi gì lên robot
./scripts/deploy_teleop.sh deploy   # chỉ copy file; không restart, không arm
```

`deploy` chỉ đồng bộ file. Nó không enable service, không khởi động motor và
không tự arm. Bật service là thao tác thủ công, sau khi đã đọc
`docs/hardware_gate.md`.

Từ root workspace có một lệnh gộp để sync nguồn, kiểm tra đường Quest và copy:

```bash
make teleop-hardware-prepare ROBOT=unitree@192.168.1.104
```

Lệnh này **không chạy teleop trên robot**. Nó fail-closed cho tới khi module
`teleop.hardware.run_teleop` tồn tại và các cổng trong `docs/hardware_gate.md`
đã được đóng. Nó cũng không install/start/enable service hay arm motor.

## An toàn

Không vận hành robot thật khi chưa thỏa điều kiện trong
`Happy-Baby-R1/docs/safety/safety_rules.md`: có người giữ E-stop, không chạy
một mình, đã chạy dry-run, và có ghi log kết quả.

SOP smoke test hardware treo trên giá:
[`r1_quest3_teleop_hardware.md`](../../docs/operations/r1_quest3_teleop_hardware.md).
