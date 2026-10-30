# Tìm và chỉ vật trên R1

`object_search/` là gói **runtime** lấy từ dự án VLA bằng
`python3 -m experiments.r1_object_grounding.p7_hb_merge.export_object_search`.
`manifest.json` ghi hash của mã và calibration. Không sửa file trong gói trực tiếp:
chỉnh ở VLA, xuất lại, chạy preflight rồi đồng bộ về HB.

Kiến trúc: `hb_high_level.service` là **owner duy nhất** gửi `rt/lowcmd`.
`r1-p5-agent.service` chụp ảnh/đọc trạng thái và gửi mục tiêu qua giao thức
controller; `r1-p6-search.service` gọi OpenAI và điều phối quét 3 góc đầu,
sau đó tối đa một lượt thân 180° và quét lại. `hb_voice.service` nạp tool
search từ chính thư mục này sau khi cài unit và tạo marker
`/etc/hb/object_search_enabled`. Tác vụ chỉ chạy trong R2+A (walk); R2+X vẫn
dành cho policy teleop. Nhấn START có thể điều khiển đi lại thủ công; khi đó
điều khiển thủ công ưu tiên hơn lệnh quay tìm vật.

Trên máy trạm, đồng bộ **mã nguồn** bằng `./hb deploy object-search` (không
restart dịch vụ). `./hb field` cũng đồng bộ gói này cùng HB. Sau lần đồng bộ
đầu, trên robot chuẩn bị Python runtime dưới user `unitree`:

```bash
bash /home/unitree/HB/integration/scripts/install_object_search.sh
```

Trước khi chuyển service, đỡ robot và đưa về DISARMED theo SOP. Xác minh
`hb_high_level.service` trỏ tới `/home/unitree/HB/controller/build/run_r1`, chỉ
có một `run_r1`, và kiểm tra các drop-in cũ bằng
`systemctl cat r1-p5-agent.service r1-p6-search.service hb_voice.service`.
Nếu còn `/etc/systemd/system/r1-p6-search.service.d/body-turn.conf` của bản
P6 cũ, sao lưu và chuyển nó ra ngoài thư mục `.d` vì nó ghi đè `ExecStart`
của gói HB. Kiểm tra cả drop-in voice cũ: không để `PYTHONPATH` hoặc
`HB_OBJECT_SEARCH_ROOT` trỏ về `/home/unitree/r1_p6_app`.

Sau khi đã xử lý drop-in cũ, cài unit **cùng tên** hiện có (không tạo service
thứ hai), xem lại đường dẫn rồi mới restart:

```bash
sudo HB_ROOT=/home/unitree/HB bash /home/unitree/HB/integration/scripts/install_object_search.sh --install-units
systemctl cat r1-p5-agent.service r1-p6-search.service
sudo systemctl enable r1-p5-agent.service r1-p6-search.service
sudo systemctl restart r1-p5-agent.service r1-p6-search.service hb_voice.service
systemctl is-active hb_high_level.service r1-p5-agent.service r1-p6-search.service hb_voice.service
journalctl -u r1-p6-search.service -u r1-p5-agent.service -n 60 --no-pager
```

`/etc/hb/stack.env` phải có `OPENAI_API_KEY`; robot cần Internet. Preflight
P6 từ chối khi owner, policy/calibration, agent hoặc nguồn sender xung đột.
Nếu service không lên, xem log và **không** bật lại bản P4/P6 ở đường dẫn cũ
song song. Kiểm tra thao tác thực tế với người đỡ và E-stop: R2+A, yêu cầu
tìm một vật phía trước, sau đó vật chỉ có phía sau; ghi lại log, ảnh và hành
động. Gói đã qua kiểm tra tĩnh, chưa xác nhận trên robot sau khi chuyển repo.
