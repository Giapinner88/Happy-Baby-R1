# Deploy từ máy dev

Chạy trên **máy dev**, từ thư mục gốc repo. Trên chính robot thì dùng `make build && make preflight && make install` (xem [README](../README.md)). Không rsync hay
restart service thủ công: `deploy_stack.sh` tự tạo backup trên robot, build
ARM64, preflight và chỉ kích hoạt high-level khi robot đã được chứng minh là
`DISARMED`.

## Deploy nhanh: một lệnh

```bash
make deploy
```

Khi chỉ đổi policy locomotion hoặc `config/locomotion.yaml`, dùng đường deploy
nhanh dưới đây để không sync/build toàn bộ stack:

```bash
make deploy-policy
```

Lệnh này chạy preflight bằng binary local đã build, tự cập nhật manifest theo
policy đang chọn, sync policy + locomotion config + manifest, rồi chỉ restart
`hb_high_level.service` sau khi robot được xác nhận `DISARMED`. Nếu contract mới
không qua preflight local hoặc cần build C++ mới, dùng lại `make deploy`.

Nếu đồng thời đổi dance policy/asset, gom vào cùng một lần restart:

```bash
make deploy-policy ARGS=--with-dance
```

Tùy chọn này sync thêm `config/dance.yaml` và mirror `policies/dance/` (xóa
asset dance cũ trên robot nếu không còn ở máy dev). Manifest vẫn chỉ ghi nhận
locomotion; dance được kiểm tra bằng preflight và SHA riêng của `dance.yaml`.

Lệnh này kiểm tra dung lượng `/tmp`, chấp nhận SHA của `flat_model` đang chọn,
rồi chạy backup → sync toàn bộ HB → build ARM64 → preflight → restart-safe,
bao gồm WebXR/vendor IK teleop trên robot. Robot được tự
dò; nếu cần ép IP thì đặt `ROBOT` trước lệnh:

```bash
ROBOT=unitree@10.42.0.33 make deploy
```

`make deploy` **có chấp nhận policy tự động**. Chỉ dùng nó khi checkpoint đã
được nghiệm thu. High-level vẫn không thể restart nếu robot chưa `DISARMED`.

Khi robot chưa `DISARMED`: `hb_integration` + `hb_voice` + `hb_voice_presets`
**vẫn được restart**,
sau đó script in `PENDING_RESTART` và thoát **mã 3** — chỉ `run_r1` là chưa
kích hoạt; teleop robot-local cũng chưa được restart. Deploy báo đỏ không có
nghĩa là chưa có gì lên robot.

Chỉ sửa `voice` thì dùng lệnh hẹp hơn, không đụng tới service high-level và
không ghi đè manifest:

```bash
./integration/scripts/deploy_stack.sh deploy --restart-voice
```

Không đặt API key vào repository hay dòng lệnh. Secret và cấu hình phần cứng
của robot nằm trong `/etc/hb/stack.env` và script deploy không ghi đè file này.

## Kiểm tra sau deploy

```bash
./integration/scripts/deploy_stack.sh status
```

Kết quả cần có high-level, integration, voice, presets và teleop runtime
`active`, `high_alive=1`, và voice có
`openai_ready=1`, `mic_ready=1`. Cảnh báo `remote_alive=0` nghĩa là tay cầm
R3-1 chưa được phát hiện; PTT sẽ giữ fail-closed cho tới khi remote hoạt động.

## Voice preset offline

Đặt recordings tại `voice_presets/assets/`, gán chúng cho `UP`, `RIGHT`,
`DOWN`, `LEFT` trong `voice_presets/config/presets.yaml`, rồi deploy riêng:

```bash
./integration/scripts/deploy_stack.sh deploy-presets
```

Lệnh này không sync/build/restart `controller`. Giữ `L1` một mình ba giây để
bật/tắt mode (có cue), kể cả khi `run_r1` đã armed bằng `R1+R2`. Khi mode bật,
giữ `L1` và nhấn `UP`, `RIGHT`, `DOWN`, hoặc `LEFT` để phát slot tương ứng.
Phím hướng riêng lẻ không phát preset và tổ hợp L1+phím hướng không kích hoạt
gesture phím hướng trần. Nhấn lặp cùng tổ hợp sau 1,5 giây để hủy; lần lặp sớm
hơn được bỏ qua. `B` vẫn là nút dừng khẩn cấp tùy chọn.
Preset chờ thêm `playback.end_tail_s` (mặc định 0,4 giây)
sau PCM cuối để không cắt đuôi loa. Xem [cấu hình preset](../voice_presets/README.md)
để đổi file/volume/thời gian đệm.

## Khi full deploy chờ restart

Đưa robot về `DISARMED` (không armed, không chạy motion) trước. Sau đó:

```bash
make deploy
./integration/scripts/deploy_stack.sh status
```

`make deploy` chỉ kích hoạt high-level khi xác nhận `high_armed=0` và
`high_state=DISARMED`, rồi mới restart teleop robot-local.

## Rollback

Nếu preflight/runtime sau deploy không đạt, rollback về backup gần nhất rồi
kiểm tra lại:

```bash
./integration/scripts/deploy_stack.sh rollback
./integration/scripts/deploy_stack.sh status
```

Không chạy rollback hoặc restart high-level khi robot đang armed.

## Dừng/bật service có kiểm soát

`stop-all` và `stop-high` chỉ chạy khi robot đã `DISARMED`. Đây là guard bắt
buộc để không bỏ motor runner lúc robot còn armed.

```bash
# Dừng toàn bộ stack: high-level + integration + voice + presets.
./integration/scripts/deploy_stack.sh stop-all

# Dừng riêng motor runner — dùng trước calibration.
./integration/scripts/deploy_stack.sh stop-high

# Dừng riêng PTT/audio coordinator; high-level và voice vẫn chạy.
./integration/scripts/deploy_stack.sh stop-integration

# Dừng riêng voice; high-level và integration vẫn chạy.
./integration/scripts/deploy_stack.sh stop-voice

# Dừng riêng preset player; các phần còn lại vẫn chạy.
./integration/scripts/deploy_stack.sh stop-presets
```

Bật lại từng service khi cần:

```bash
./integration/scripts/deploy_stack.sh start-high
./integration/scripts/deploy_stack.sh start-integration
./integration/scripts/deploy_stack.sh start-voice
./integration/scripts/deploy_stack.sh start-presets

# Hoặc bật lại cả bốn service.
./integration/scripts/deploy_stack.sh start-all
```

## Calibration

Trước calibration, robot phải `DISARMED`, sau đó dừng riêng high-level:

```bash
./integration/scripts/deploy_stack.sh stop-high
```

Sau calibration, khởi động lại high-level và kiểm tra status:

```bash
./integration/scripts/deploy_stack.sh start-high
./integration/scripts/deploy_stack.sh status
```

## Sự cố thường gặp

| Hiện tượng                                                    | Xử lý                                                                                                                           |
| ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `No space left on device` khi local preflight                  | Giải phóng dung lượng an toàn trên máy dev, đặc biệt`/tmp`, rồi chạy lại `make deploy`.                          |
| `flat_model is missing` hoặc `sha256sum ... Is a directory` | Cập nhật source có`integration/scripts/config_reader.sh`; không thêm `flat_model` trùng vào `config/tuning.yaml`. |
| `manifest=... but tuning selects ...`                          | Chạy`make deploy`; nó sẽ cập nhật manifest theo `flat_model` hiện tại.                                                 |
| `PENDING_RESTART`                                              | Đưa robot về `DISARMED`, rồi chạy lại `make deploy` để nạp cả high-level và teleop. |
