# Deploy

Chạy từ máy trạm, ở gốc repo. Robot chỉ nhận bản đồng bộ vào `/home/unitree/HB`, không sửa tại chỗ.
Deploy chép đúng thư mục máy trạm, kể cả thay đổi chưa commit.

## Lệnh deploy

| Đổi gì | Lệnh | Restart | Cần DISARMED |
| --- | --- | --- | --- |
| Bất kỳ, dùng hằng ngày | `./hb deploy` | chỉ phần có thay đổi | chỉ khi controller đổi |
| Code C++, config, policy, dance | `./hb deploy controller` | `hb_high_level` | có |
| Voice, preset giọng, integration (PTT) | `./hb deploy voice` | voice / presets / integration | không |
| Teleop Quest | `./hb deploy teleop` | runtime teleop | không |
| Mã tìm vật | `./hb deploy object-search` | không restart service | không |
| Lần đầu, sửa systemd, nghi robot lệch | `./hb deploy all` | tất cả | có |
| Chép + build + preflight, chưa chạy | thêm `--no-restart` | không | không |
| Xem trước | `./hb diff` (= `--dry-run`) | không | không |
| Qua Tailscale | `./hb field [đối tượng]` | như `deploy` | như `deploy` |
| Quay về bản trước lần deploy gần nhất | `./hb rollback` | phần có trong backup | có nếu có controller |
| Kéo config đã sửa trên robot về | `./hb pull [--dry-run]` | không | không |

Tên cũ vẫn chạy: `deploy policy|dance|high` = `deploy controller`, `deploy presets|integration` = `deploy voice`.

## Quy tắc an toàn (tự động)

- Robot ARMED: không chép gì của controller, chỉ deploy voice/teleop. Màn hình báo đỏ; DISARMED (giữ R1+R2 khi đứng yên) rồi chạy lại `./hb deploy`.
- Controller: build + preflight `run_r1` trên máy trạm, rồi trên robot bằng binary mới, trước khi restart. Lỗi ở bất kỳ bước nào thì trả bản cũ (config, policy, `run_r1`) và không restart.
- Bản mới không khởi động được: tự quay về bản cũ và chạy lại. Nếu vẫn không lên, báo "ROBOT KHÔNG CÓ CONTROLLER".
- Đã chép nhưng chưa restart (`--no-restart`, robot ARMED lúc restart): lần `./hb deploy` sau tự restart.
- Voice/integration/presets restart không kéo theo `hb_high_level`.
- Backup trên robot: `~/HB_backups`, giữ 3 bản (`HB_BACKUP_KEEP`), không chứa dance và secret.

## Đọc kết quả

Mỗi bước một dòng `✓` / `✗` / `⚠` kèm thời gian. Cuối log có mục **Ảnh hưởng tới robot**:
`✗` là robot không chạy được hoặc chưa cập nhật, `⚠` là gián đoạn hoặc cần thao tác (vd phải arm lại). Log đầy đủ: `~/.local/state/hb/deploy/`.

## Service

| Việc | Lệnh |
| --- | --- |
| Trạng thái / log | `./hb status` · `./hb logs [all\|high\|voice\|teleop\|…]` |
| Start / stop / restart | `./hb start\|stop\|restart <all\|high\|integration\|voice\|presets\|teleop>` (high/all cần DISARMED) |
| Trước auto-calib | `./hb stop high`, xong thì `./hb start high` |
| Shell trên robot | `./hb ssh [lệnh]` |
| Build / test máy trạm | `./hb build` · `./hb test` · `./hb preflight` |

## Kết nối

- Địa chỉ robot: `ROBOT=unitree@<ip>` → `~/.config/hb/robot.env` → dò LAN → Tailscale `100.82.165.36` (`HB_ROBOT_TAILSCALE`).
- Sudo hỏi một lần ở đầu. Tailscale SSH có thể in link `login.tailscale.com`: mở, xác nhận, lệnh tự chạy tiếp.
- Secret nằm trong `/etc/hb/stack.env` trên robot; deploy không ghi đè, không đồng bộ.
