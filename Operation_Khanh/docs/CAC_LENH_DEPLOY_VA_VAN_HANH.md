# Các lệnh deploy và vận hành HB

Script deploy tự dò lần lượt: `192.168.12.2`, `192.168.1.33`, `100.82.165.36` và `unitree-r1`.
Thư mục trên robot: `/home/unitree/HB`

Không ghi mật khẩu vào lệnh hoặc file. SSH và `sudo` sẽ hỏi mật khẩu khi cần.

## 1. Chọn quy trình theo phần code đã sửa

Chạy trên máy đang chứa source `HB`. Nếu chuyển sang máy khác, chỉ cần `cd` tới đúng thư mục `HB`; script tự xác định đường dẫn project.

### Khi sửa `high_level_2` hoặc không chắc đã sửa phần nào

Đưa robot về `DISARMED` và tư thế an toàn, sau đó deploy toàn stack:

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy
./r1_integration/scripts/deploy_stack.sh status
```

Lệnh này đồng bộ và build cả ba phần, sau đó restart `hb_integration`, `hb_high_level` và `hb_voice`. High-level chỉ được restart khi script xác nhận `DISARMED`.

Nếu thay policy locomotion, dùng một lệnh để xác nhận rồi deploy toàn stack:

```bash
./r1_integration/scripts/deploy_stack.sh deploy --accept-policy
```

Lệnh tự build/preflight trong `/tmp`, cập nhật manifest, đồng bộ, build ARM64 và
restart có kiểm tra `DISARMED`. Không dùng tùy chọn này nếu chỉ sửa code/voice.

### Khi chỉ sửa `voice_r1`

Có thể dùng khi high-level đang armed vì lệnh chỉ restart coordinator âm thanh
và voice, không restart tiến trình motor/high-level:

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy --restart-voice
./r1_integration/scripts/deploy_stack.sh status
```

Lệnh này vẫn đồng bộ, build ARM64 và preflight toàn bộ. Nó restart
`hb_integration` rồi `hb_voice` để giao thức gate luôn đồng bộ; mic/loa khóa
trong lúc restart. Nếu local có thay đổi `high_level_2`, binary mới chưa được
nạp cho tới lần deploy toàn stack hoặc restart high-level an toàn sau đó.

### Chỉ đồng bộ, build và preflight

Không restart service nào:

```bash
./r1_integration/scripts/deploy_stack.sh deploy --no-restart
```

Muốn chỉ định địa chỉ khác và bỏ qua bước dò:

```bash
ROBOT=unitree@10.0.0.25 ./r1_integration/scripts/deploy_stack.sh deploy --restart-voice
```

## 2. Chi tiết deploy toàn stack

Chỉ dùng khi high-level đang `DISARMED` và robot ở tư thế an toàn:

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh diff
./r1_integration/scripts/deploy_stack.sh deploy
./r1_integration/scripts/deploy_stack.sh status
```

`deploy` tự tạo backup, đồng bộ ba thư mục, build trên ARM64, chạy preflight và chỉ restart high-level nếu chứng minh được trạng thái `DISARMED`.

## 3. Chỉ kiểm tra file khác nhau

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh diff
```

Không có output nghĩa là local và robot đang đồng bộ theo các file mà script quản lý.

## 4. Restart riêng từng service

Restart voice; không tác động tiến trình motor:

```bash
ssh -t unitree@192.168.12.2 'sudo systemctl restart hb_voice.service && systemctl --no-pager --full status hb_voice.service'
```

Restart coordinator; mic/loa sẽ fail-closed trong thời gian restart:

```bash
ssh -t unitree@192.168.12.2 'sudo systemctl restart hb_integration.service && systemctl --no-pager --full status hb_integration.service'
```

Restart high-level có kiểm tra an toàn. Lệnh sẽ từ chối nếu high-level chưa `DISARMED`:

```bash
ssh -t unitree@192.168.12.2 '
set -e
grep -qx "high_alive=1" /run/hb/status.env
grep -qx "high_armed=0" /run/hb/status.env
grep -q "^high_state=DISARMED" /run/hb/status.env
sudo systemctl restart hb_high_level.service
systemctl --no-pager --full status hb_high_level.service
'
```

## 5. Start/stop riêng voice

```bash
ssh -t unitree@192.168.12.2 'sudo systemctl stop hb_voice.service'
```

```bash
ssh -t unitree@192.168.12.2 'sudo systemctl start hb_voice.service'
```

## 6. Xem trạng thái hệ thống

Từ thư mục local `HB`:

```bash
./r1_integration/scripts/deploy_stack.sh status
```

Hoặc đọc trực tiếp trên robot:

```bash
ssh unitree@192.168.12.2 '
cat /run/hb/status.env
systemctl show hb_high_level hb_integration hb_voice \
  -p Id -p ActiveState -p SubState -p MainPID -p NRestarts --no-pager
'
```

Các giá trị voice bình thường khi không giữ Select:

```text
ready=1
high_alive=1
high_busy=0
remote_alive=1
ptt=0
mic_allowed=0
speaker_allowed=1
```

Trạng thái runtime voice bình thường:

```text
state=ready
openai_ready=1
mic_ready=1
attempt=1
last_reason=none
```

Nếu `state=reconnecting`, xem `last_reason`. Supervisor sẽ tự thử lại; không
cần SSH để restart. Lệnh `status` trả lỗi nếu OpenAI/mic chưa ready hoặc trạng
thái đã cũ quá 5 giây.

## 7. Xem log

Log voice trực tiếp; nhấn `Ctrl+C` để thoát:

```bash
ssh unitree@192.168.12.2 'journalctl -u hb_voice.service -f -n 100 --no-pager -o short-iso'
```

100 dòng log voice gần nhất:

```bash
ssh unitree@192.168.12.2 'journalctl -u hb_voice.service -n 100 --no-pager -o short-iso'
```

Log high-level:

```bash
ssh unitree@192.168.12.2 'journalctl -u hb_high_level.service -f -n 100 --no-pager -o short-iso'
```

Log coordinator:

```bash
ssh unitree@192.168.12.2 'journalctl -u hb_integration.service -f -n 100 --no-pager -o short-iso'
```

## 8. Chỉ build trên robot

```bash
ssh unitree@192.168.12.2 'HB_ROOT=/home/unitree/HB bash /home/unitree/HB/r1_integration/scripts/build_on_robot.sh'
```

## 9. Chạy preflight thủ công

```bash
ssh -t unitree@192.168.12.2 "sudo bash -c 'set -a; source /etc/hb/stack.env; set +a; HB_ROOT=/home/unitree/HB bash /home/unitree/HB/r1_integration/scripts/preflight.sh all'"
```

Chỉ kiểm tra voice, thay `all` bằng `voice`.

## 10. Lấy thay đổi từ robot về local

Luôn xem trước:

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh pull --dry-run
```

Chỉ khi chắc chắn muốn ghi phiên bản trên robot về local:

```bash
./r1_integration/scripts/deploy_stack.sh pull
```

## 11. Rollback bản backup gần nhất

Lệnh này ghi đè source trên robot bằng backup gần nhất, build lại và kích hoạt theo kiểm tra an toàn. Chỉ dùng khi bản deploy mới bị lỗi:

```bash
cd /home/ubuntu22/train_mujoco/HB
./r1_integration/scripts/deploy_stack.sh rollback
```

Sau rollback:

```bash
./r1_integration/scripts/deploy_stack.sh status
```
