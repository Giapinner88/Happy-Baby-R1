# Camera mắt R1 trong Quest/Vuer

## Trạng thái và phạm vi

Source code hiện có đầy đủ các tầng: capture `videohub` trên robot, ghi JPEG
gốc, preview HTTP loopback, SSH local forward, subscriber latest-only và
`ImageBackground` full-field trong Vuer. Tính năng được bật mặc định chỉ cho
`make teleop-hardware`; simulation-only `make teleop` không tự mở SSH hay camera
robot.

Đã kiểm chứng local bằng unit/integration tests. Camera R1 riêng lẻ trước đó đã
trả 10/10 JPEG 1280×720 qua `VideoClient.GetImageSample()` trên `eth10`. Workflow
gateway/tunnel/Quest tích hợp chưa được chạy lại trên phần cứng trong lần cập
nhật này, nên chưa được gọi là field-ready.

## Chạy

```bash
# Camera R1 + ghi JPEG gốc + Isaac mirror + hardware (mặc định)
make teleop-hardware HOST_IP=100.95.122.105 ROBOT=100.82.165.36

# Tắt riêng camera robot
HB_ROBOT_CAMERA=0 make teleop-hardware \
  HOST_IP=100.95.122.105 ROBOT=100.82.165.36
```

`ROBOT` là target SSH bình thường của robot. Không nhập thêm camera IP hoặc
camera port. Launcher chạy gateway ngay trên robot, nơi `eth10` truy cập dịch vụ
`videohub`, rồi forward endpoint loopback về workstation.

LAN SSH được ưu tiên cho video. Tailscale dùng được khi đường truyền trực tiếp;
một kết quả chạy qua DERP relay không được dùng làm bằng chứng hiệu năng ngoài
thực địa.

## Nút điều khiển

| Input | Hành vi |
|---|---|
| Bắt đầu session | Quest passthrough |
| Cạnh nhấn A tay phải, frame còn mới | Robot camera full-field |
| Giữ A | Không toggle lặp |
| Cạnh nhấn A tiếp theo | Quay lại Quest passthrough |
| Cò phải | Deadman; nhả cò pause/hold, không đóng session |
| Cò trái | Recalibration; dùng được độc lập với A và cò phải |

Mẫu controller đầu tiên chỉ arm edge detector. Nếu A đã được giữ lúc Quest kết
nối, view không tự đổi.

## Luồng dữ liệu

```text
R1 eye camera
  → videohub RPC trên robot eth10
  → gateway read-only, tối đa 15 Hz
      ├─ JPEG 1280×720 nguyên bản → bounded writer + manifest/hash
      └─ JPEG 640×360 quality 60 → HTTP 127.0.0.1:8765
  → SSH local forward theo target ROBOT
  → workstation subscriber 10 Hz, latest-only
  → shared RGB buffer riêng
  → Vuer ImageBackground full-field ↔ Quest passthrough
```

Gateway không tạo command publisher và không sở hữu motor. HTTP/JPEG/network
chạy ngoài command-loop thread. Quest không kết nối trực tiếp tới robot; pixel
đi qua Vuer HTTPS/WSS hiện có.

## Freshness và lỗi

- Chỉ cho bật robot view khi source age và workstation receive age không quá
  0,5 giây.
- Khi đang xem robot, frame vượt 1,0 giây tự gỡ background và trở về Quest
  passthrough.
- Lỗi/tắc HTTP không chặn loop điều khiển và không thay đổi deadman.
- Nếu camera được yêu cầu nhưng gateway/tunnel chưa có frame mới lúc startup,
  launcher fail trước khi mở control pipeline.
- Local port đang bị process khác giữ sẽ làm startup fail; launcher không kill
  process lạ.

Các event `camera_view_changed`, `camera_enable_rejected` và
`camera_runtime_fallback` được ghi trong `bridge_connection.jsonl`.

## Evidence và recording

Mỗi frame hợp lệ được ghi nguyên JPEG, không recompress, trên robot. Sau khi
gateway dừng, launcher copy và kiểm manifest, byte count, SHA-256, queue drops
và disk errors tại:

```text
results/smoke/<run-id>/robot_camera_original/
```

Các file chính:

- `frames/frame_<sequence>.jpg`
- `manifest.jsonl`
- `summary.json`
- `gateway_summary.json`
- `camera_evidence_validation.json`
- `camera_artifact_status.json`

Queue writer hữu hạn; queue drop hoặc disk error làm run incomplete, không làm
queue tăng vô hạn và không chặn preview.

## Tương thích và override

`ROBOT_CAMERA_WEBRTC_URL=https://.../offer` vẫn được hỗ trợ cho camera server
ngoài. Khi biến này có giá trị, nó được ưu tiên thay gateway built-in. Nhánh
WebRTC cũ không có health callback/freshness fallback mạnh như local preview.

`ROBOT_CAMERA_ZMQ_ENDPOINT` chỉ còn là recorder tương thích cho publisher ZMQ
ngoài; built-in camera đã tự ghi JPEG gốc nên không cần biến này.

## Kiểm thử trước pilot

Không cần robot để chạy verification source:

```bash
conda run --no-capture-output -n unitree_sim_env \
  python -m pytest tests/teleop hardware/teleop/tests -q
bash -n scripts/teleop/run_r1_quest3_hardware.sh
```

Trước pilot treo robot, cần làm riêng read-only gateway/tunnel smoke, sau đó mới
kiểm Quest. Acceptance ngoài thực địa vẫn yêu cầu ít nhất 10 phút: ≥10 FPS,
frame-age p95 <250 ms, <1% mẫu >500 ms, không queue growth/drop, camera loss
trở về passthrough trong 1 giây và semantics hai cò không đổi.

Đo photon-to-Quest cần test quang học với target đổi thời gian/flash; không thể
suy ra từ timestamp monotonic của hai máy.
