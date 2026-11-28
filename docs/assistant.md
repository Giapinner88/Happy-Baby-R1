# Trợ lý giọng nói: tìm vật, chỉ vật, đi tới vật

Tài liệu cho người vận hành và người phát triển. Phần voice chung (mic, loa, ElevenLabs) ở
[voice.md](voice.md); lệnh deploy ở [deploy.md](deploy.md); nút tay cầm ở [controls.md](controls.md);
**đọc [safety.md](safety.md) trước khi cho robot di chuyển.**

## 1. Trạng thái (cập nhật 27/11/2026)

| Phần | Trạng thái |
|---|---|
| Mã trong repo (`develop`, commit `d94f18a`) | Có đủ |
| Tính năng trên robot bằng stack thử nghiệm `r1_assistant` | Đã chạy thật ngày 21/11 |
| **Cùng tính năng chạy từ `~/HB` theo tài liệu này** | **Chưa chạy trên robot.** `install_assistant.sh` mới thử với thư mục giả |
| Giọng ElevenLabs | Đã cấu hình; chưa thử vì chưa có key. Thiếu key thì dùng giọng OpenAI |
| `~/HB/assistant` trên robot ngày 27/11 | Chưa deploy; robot đang chạy `r1_assistant` |

Lần bật đầu tiên trên HB là lần kiểm chứng: làm đủ mục 4, có người giữ E-stop, và ghi lại kết quả.

## 2. Tính năng và câu lệnh

Giữ `SELECT` trên tay cầm, nói một câu, rồi nhả.

| Bạn nói (ví dụ) | Robot làm | Robot trả lời (ví dụ) |
|---|---|---|
| "Đi tới bình cứu hỏa" | Quét đầu ±25°, chưa thấy thì quay thân 180°; quay về phía vật; đi tới khi mép thân cách vật khoảng 30 cm | "Tới nơi rồi nè." / "Mình tới gần rồi." |
| "Đi tới bình nước" (có hai bình) | Hỏi lại, bạn trả lời "bên trái"/"bên phải" | "Bạn muốn bình bên trái hay bên phải?" |
| (vật bị dời khi robot đang đi) | Nhìn lại, tìm lại, đổi hướng, đi tới chỗ mới (tối đa 2 lần) | "Mình tìm lại được rồi, tới nơi rồi nè." |
| "Tìm và chỉ vào số 9" | Quét tìm rồi chỉ tay | "Nó ở đằng kia nè." |
| "Bạn thấy gì trước mặt?" | Chụp một ảnh và mô tả | Mô tả ngắn |
| "Vẫy tay" | Cử chỉ có sẵn của HB | |
| "Dừng lại" | Hủy tác vụ đang chạy | "Mình dừng rồi." |

"Tới nơi rồi nè." chỉ dùng khi camera nhìn lại thấy vật ở trước mặt. "Mình tới gần rồi." nghĩa là robot dừng
theo ước lượng quãng đi và chưa nhìn lại được. Cả hai đều không phải phép đo khoảng cách đã kiểm chứng.

**Camera và riêng tư:** ảnh chỉ được chụp và gửi tới OpenAI sau một câu nói mới của người dùng (một câu mở đúng
một lần, trong 60 s; một từ lửng như "cái" bị từ chối). Chốt không phân biệt người nói.

## 3. Kiến trúc

```text
SELECT + giọng nói
  -> hb_voice (OpenAI Realtime; công cụ từ assistant/overlay)
  -> hb_assistant  127.0.0.1:5595           điều phối tác vụ
       tìm   -> object_search/.venv, r1-p5-agent (đầu, thân, ảnh), OpenAI nhận diện
       đi    -> camera + độ sâu (assistant/models/dac01) -> APV 127.0.0.1:5573
  -> hb_high_level  (owner duy nhất gửi rt/lowcmd)
```

| Đường dẫn trong repo | Nội dung |
|---|---|
| `assistant/` | Mã chạy (xuất từ VLA, **không sửa tay**), `config/`, `EXPORT.json` (hash, commit nguồn) |
| `integration/scripts/install_assistant.sh` | `prepare` / `pin` / `check` / `enable` / `disable` |
| `integration/systemd/hb_assistant.service.in` | Unit của dịch vụ điều phối |
| `integration/scripts/run_voice.sh` | Nạp công cụ assistant khi có `/etc/hb/assistant_enabled` |
| `voice/config/tuning.yaml`, `voice/hb_voice/config.py` | ElevenLabs và tự chuyển giọng OpenAI khi thiếu key |

## 4. Cài đặt lần đầu

Yêu cầu: `/etc/hb/stack.env` có `OPENAI_API_KEY`; robot có Internet; `controller/build/run_r1` được build từ
`develop` (có bộ nhận lệnh đi tới; `check` sẽ báo nếu thiếu); mô hình độ sâu `dac01` có sẵn ở
`~/r1_local_approach/models/dac01` hoặc được build lại.

### 4.1 Máy trạm: đưa mã lên robot

```bash
git pull --ff-only
./hb deploy voice          # voice + integration; restart voice
./hb deploy assistant      # assistant/ + script; không restart gì
```

> **Không dùng `./hb deploy` (tự động) hoặc `deploy all` cho việc này.** Deploy chép đúng thư mục máy trạm, kể cả
> thay đổi chưa commit; nếu có thay đổi ở `controller/` (policy, config đi bộ) thì chúng cũng lên robot.

### 4.2 Trên robot (user `unitree`)

Robot được đỡ và **DISARMED** từ bước 2, có người giữ E-stop.

```bash
# 1. Một lần: tạo ~/HB/assistant/.venv và chép mô hình vào ~/HB/assistant/models/dac01
bash ~/HB/integration/scripts/install_assistant.sh prepare

# 2. Chỉ khi stack thử nghiệm r1_assistant đang cài (file /etc/r1-assistant/enabled tồn tại): gỡ kích hoạt.
#    File của nó vẫn nằm trên đĩa.
A=$(cat /etc/r1-assistant/enabled)
R=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]+"/rollback.json"))["release"])' "$A")
sudo python3 "$R/bin/r1-assistant" rollback --activation "$A"

# 3. Chế độ search (chạy r1-p5-agent). Chế độ này TẮT tự khởi động teleop Quest.
sudo bash ~/HB/object_search/runtime/runtime_mode.sh search

# 4. Kiểm tra (chỉ đọc), rồi bật
bash ~/HB/integration/scripts/install_assistant.sh check
sudo bash ~/HB/integration/scripts/install_assistant.sh enable --operator-ready
```

`enable` làm các việc sau: ghim cấu hình nhìn/chỉ theo `walk_policy` robot đang chọn; ghi cấu hình chạy vào
`/etc/hb/assistant/`; thêm `R1_APPROACH_PORT=5573`, `R1_APPROACH_ALLOW_NONZERO=1` vào `/etc/hb/stack.env` (bản sao
`stack.env.assistant.bak`); tắt `hb_local_approach`; restart `hb_high_level`, `hb_voice`; bật `hb_assistant`.

### 4.3 Dùng hằng ngày

1. Đặt robot xuống, bàn giao và ARM theo [controls.md](controls.md), chọn đi bộ (R2+A). Tìm vật cần quyền đầu/tay
   (START); nếu thân bị khóa thì owner từ chối lệnh đi.
2. Giữ `SELECT`, nói lệnh.
3. Dừng: nói "dừng lại", hoặc dùng tay cầm / E-stop. Chạm cần điều khiển là bộ điều khiển lấy lại quyền và tác vụ
   dừng, không tự chạy lại.

## 5. An toàn và giới hạn

- **Khi đang đi, robot không nhìn đường, không phát hiện người hay vật cản.** Khu vực phải trống, luôn có người giữ
  E-stop. Không dùng gần trẻ em khi chưa có kiểm chứng riêng.
- Giới hạn mặc định (`assistant/config/approach.json`): đi 0.16 m/s, quay 0.2 rad/s, dừng khi tâm thân cách vật khoảng 0,55 m (mép thân cách khoảng 30 cm; `standoff_m`, nhỏ nhất 0,43 m), tổng
  quãng lệnh 3 m mỗi tác vụ, mỗi đoạn kế hoạch tối đa 0.5 m, tối đa 2 lần tìm lại trong 150 s.
- Quãng đi tính theo tích phân lệnh. Trong 5 đoạn thẳng 0.25–0.55 m, đo thước được 0.875–1.01 lần. Chưa đo trên quãng
  dài, sàn khác, hoặc khi quay.
- Camera độ sâu đọc gần như phẳng 1.3–1.4 m khi vật cách 1.5–2.0 m, nên robot đi từng đoạn rồi đo lại.
- Policy đi bộ dùng ngày 21/11 làm robot trôi khi quay tại chỗ; sau mỗi lần quay robot đo lại.
- Chưa chọn được "vật gần nhất" khi có nhiều vật giống nhau (chọn vật thấy trước, hoặc hỏi lại).
- Độ chính xác chỉ tay chưa được xác nhận bằng đo.
- Ngày 21/11 có một lần owner thu hồi quyền đi giữa chừng (`status=12`) chưa rõ nguyên nhân.

## 6. Giọng ElevenLabs

`tuning.yaml` đã chọn `tts.provider: "elevenlabs"`. Thêm `ELEVENLABS_API_KEY` và `ELEVENLABS_VOICE_ID` vào
`/etc/hb/stack.env` (`sudoedit`, xem [voice.md](voice.md) mục 3) rồi:

```bash
sudo systemctl restart hb_voice
journalctl -u hb_voice -n 80 --no-pager | grep -iE "elevenlabs|OpenAI voice"
```

Thiếu key: log in "using the OpenAI voice" và robot vẫn nói. Quay lại giọng OpenAI: bỏ hai dòng key rồi restart.

## 7. Tắt, quay lại như cũ

```bash
sudo bash ~/HB/integration/scripts/install_assistant.sh disable          # DISARMED
sudo bash ~/HB/object_search/runtime/runtime_mode.sh quest               # nếu cần teleop Quest
```

## 8. Chẩn đoán

```bash
systemctl is-active hb_high_level hb_voice r1-p5-agent hb_assistant
journalctl -u hb_assistant -u hb_voice -n 100 --no-pager
curl -s -X POST -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:5595/status
```

Dữ liệu mỗi lượt ở `~/hb_assistant/runs/`:

| Đường dẫn | Dùng để |
|---|---|
| `<task-id>/task.json` | Kết quả cả tác vụ: `status`, `approach_rounds`, `camera_confirmed`, `object_moved` |
| `search/<lượt>/events.jsonl`, `result.json` | Bước tìm: góc quét, quay thân, nhận diện |
| `approach/<id>/control.jsonl`, `observations.jsonl`, `result.json` | Bước đi: pha, khoảng cách đo, lệnh, tracker |
| `approach/<id>/tracker_lost/` | 8 ảnh và số liệu khi mất tracker |
| `approach/<id>/owner_status_trace.json` | Trạng thái owner khi lệnh bị từ chối |

| Thông báo / hiện tượng | Xử lý |
|---|---|
| `... r1_assistant stack is installed` | Làm bước 2 ở mục 4.2 |
| `r1-p5-agent is not running` | Chọn chế độ search (bước 3) |
| `depth model missing` | Chép `dac01` vào `~/HB/assistant/models/` hoặc build lại, rồi `prepare` |
| `policy pin out of date` | Đã đổi `walk_policy`: `bash ~/HB/integration/scripts/install_assistant.sh pin` |
| `lacks the approach receiver` | `./hb deploy controller` (đổi controller/policy đang chạy: xem kỹ `./hb diff` trước) |
| `Read-only file system: '/run/hb-approach/camera.lock'` (robot nói "chưa nhìn rõ") | Thiếu `/etc/tmpfiles.d/hb-approach.conf` (bản `enable` cũ): chạy lại `enable`, hoặc tạo file đó rồi `sudo systemctl restart hb_voice` |
| Robot nói "Mình chưa nghe rõ" | Chốt camera không nhận được câu nói rõ; giữ SELECT và nói lại |
| `head target not tracking` trong log tìm | Kiểm tra quyền đầu/tay (START) trong R2+A |
| Dừng vì "tay cầm vừa lấy lại quyền" | Kiểm tra cần điều khiển/chế độ, rồi ra lệnh lại |

## 9. Phát triển tiếp

Mã nguồn và kiểm thử nằm ở dự án VLA (nhánh `vla`), thư mục `integrations/r1_assistant`, `ai_modules/navigation`,
`ai_modules/vision`, `experiments/r1_object_grounding`. Sửa ở đó, chạy kiểm thử, rồi xuất lại sang HB:

```bash
# trong repo VLA
python3 scripts/interaction/export_r1_assistant_hb.py --hb-root <đường dẫn repo HB> --force
```

Sau đó commit `assistant/` trong HB và làm lại mục 4.1; chạy `disable` rồi `enable` để nạp cấu hình mới. Không sửa tay
trong `assistant/` (lần xuất sau sẽ ghi đè). `EXPORT.json` ghi commit nguồn; nếu `source_has_uncommitted_changes` là
`true` thì mã nguồn tương ứng chưa được commit trong VLA.

`object_search/` còn bản P4 cũ hơn bản trong `assistant/app`, và `voice/hb_voice/approach_tools.py` (đường
`hb_local_approach` cũ) vẫn còn; khi assistant bật thì hai phần này không được dùng cho lệnh giọng nói.
