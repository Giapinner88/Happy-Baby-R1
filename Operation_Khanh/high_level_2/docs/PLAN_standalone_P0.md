# PLAN — P0 để CHẠY ĐỘC LẬP an toàn (headless) cho HB high_level_2

> Trạng thái: **ĐÃ CODE (2026-07-15), build OK trên máy dev (x86_64).** Chờ build aarch64
> trên robot + verify (robot offline giữa chừng). Cổng `arm_gate_enabled: false` mặc định.
> Mục tiêu: 3 việc P0 bắt buộc để deploy độc lập (headless, R3-1, không laptop)
> mà an toàn. P1/P2 xem phần cuối `docs/` + các plan khác.

Ba việc P0:
1. **Đảm bảo run_r1 là NGUỒN DUY NHẤT ghi `rt/lowcmd`** (cổng `kDisarmed`).
2. **Damp khi nhận SIGTERM/SIGINT** (systemctl stop/restart, tắt máy).
3. **Giám sát PIN thấp** — ⛔ **BỊ CHẶN, xem P0-3.**

## 🔬 ĐO TRÊN ROBOT THẬT — 2026-07-15 (không còn giả định nào)

Robot bật, **CHƯA vào dev mode**, `run_r1` **KHÔNG chạy**. Tất cả đo bằng công cụ **chỉ nghe**.

### ✅ Bằng chứng 1 — built-in ĐANG phát `rt/lowcmd`, và PC2 NGHE THẤY

```
Nghe rt/lowcmd trên eth10 trong 5 giây (run_r1 KHÔNG chạy):
  → 3104 gói  (~621 Hz)   crc cuối = 0x57BBC2E7   kp[0] = 0.0
```

**Nguồn duy nhất = built-in controller.** Hai kết luận:
1. 🔴 **Xung đột là THẬT** — built-in bơm `rt/lowcmd` **621 Hz** ngay cả khi robot đứng yên.
   `run_r1` publish thêm = 2 nguồn ghi cùng topic. (`kp=0` → nó đang giữ robot **mềm**.)
2. ✅ **Lớp 2 của P0-1 DÙNG ĐƯỢC — detector KHÔNG MÙ.** Đây là giả định sống còn của cả
   thiết kế cổng, giờ **đã chứng minh**. `arm_require_seen_builtin` sẽ hoạt động đúng.

### ✅ Bằng chứng 2 — TOPIC PIN (gỡ chặn P0-3)

Quét 93 topic đang publish. Pin ở:

| Topic | Kiểu | Tần số |
|---|---|---|
| **`rt/lf/bmsstate`** | `unitree_hg::msg::dds_::BmsState_` | **20.2 Hz** |
| `rt/lf/secondary_bmsstate` | `BmsState_` | (pin phụ) |
| `rt/lf/battery_alarm` | `std_msgs::String_` | (cảnh báo dạng chữ) |

Đọc thử → **chạy ngon**: `soc = 90%`, `soh = 99%`, `current = -2352` (âm = đang xả),
`bmsvoltage[0] = 36540` (≈36.5 V).

⇒ **P0-3 hết bị chặn.** Dùng `rt/lf/bmsstate`, đọc `soc()` (0-100).
Bản thảo cũ viết *"đọc pin từ `LowState_`"* — **SAI**, `hg/LowState_` không có trường pin nào.

### ✅ Bằng chứng 3 — auto-start CHƯA hề tồn tại
`systemctl is-enabled hb_high_level` → **No such file or directory**. `pgrep run_r1` → rỗng.
Robot **chưa** cài service. Tốt: không có nguy cơ tự bật ngoài ý muốn ngay lúc này.

### ✅ Kiểm chứng khác (header SDK)
- `LowCmd_` **CÓ** `std::array<uint32_t,4> reserve_` và `uint32_t crc_` → đối chiếu CRC **dùng được**.
- `Application::Run()` **đã sẵn** `sender_.Damping()` ở lối thoát → P0-2 chỉ cần bắt tín hiệu.

> Công cụ đo để trong `/tmp` trên robot (chỉ nghe, chạy lại được):
> `/tmp/cmdcount` (đếm `rt/lowcmd`), `/tmp/topics` (liệt kê topic), `/tmp/bms` (đọc pin).

---

## P0-1. Một nguồn `rt/lowcmd` duy nhất (QUAN TRỌNG NHẤT)

**✅ ĐÃ XÁC NHẬN (user, 2026-07-13):** khi bật nguồn robot, nếu **KHÔNG chuyển sang
"chế độ phát triển"** thì controller **built-in của Unitree tự chạy** và cũng publish
`rt/lowcmd`. Nếu run_r1 (auto-start systemd) chạy cùng lúc → **2 lệnh động cơ xung đột
→ có thể PHÁ ROBOT.**

> ⛔ **HỆ QUẢ QUAN TRỌNG:** thiết lập auto-start hiện tại (`systemctl enable
> hb_high_level`) **CHƯA an toàn** — nó bật run_r1 **bất kể** robot đã vào chế độ
> phát triển hay chưa. Phải sửa để run_r1 **chỉ** chạy khi built-in đã TẮT.

**Vào chế độ phát triển (ĐÃ XÁC NHẬN user 2026-07-13):** bật nguồn robot → **nhấn GIỮ
L2+R2** → dev mode (built-in nhả quyền). Là thao tác **THỦ CÔNG mỗi lần bật nguồn**,
KHÔNG tự giữ qua reboot.

**⇒ Cửa sổ nguy hiểm rõ ràng:** lúc mới bật nguồn (TRƯỚC khi bấm L2+R2), built-in
đang chạy. Nếu run_r1 auto-start **publish `rt/lowcmd` ngay lúc đó** (kể cả Damping
ở IDLE) → đè lên built-in → xung đột. Vậy auto-start "mù" (bật là publish luôn) là SAI.

### Bước 1 — CHẨN ĐOÁN ✅ ĐÃ CHẠY (2026-07-15) — KẾT QUẢ: built-in KHÔNG ở PC2
```bash
systemctl list-units --type=service --state=running | grep -iE "unitree|sport|ai_|..."  # -> RỖNG
ps aux | grep -iE "sport|ai_sport|mc_service|basic_service|lowlevel" | grep -v grep       # -> RỖNG
```
**Cả hai rỗng** → trên PC2 **không có** tiến trình/service điều khiển motor nào. Kết hợp với
khảo sát DDS (`HE_THONG_ROBOT.md`): built-in ở **bo `.161`**, publish `rt/lowcmd` 621 Hz,
**không đăng nhập được**.

### Bước 2 — ⛔ CÁCH CŨ (disable service) BẤT KHẢ THI
Không có service built-in nào **trên PC2** để `Conflicts=`/`systemctl stop`. Built-in ở
`.161` (bo khoá, chỉ DDS). ⇒ **Bỏ hẳn hướng này.** Chỉ còn **cách B (passive-until-armed)** —
xem §THIẾT KẾ CHI TIẾT. run_r1 tự chặn mình cho tới khi phát hiện built-in đã im (dev mode).
- ⚠ Chỉ tắt sau khi xác nhận đúng service điều khiển chuyển động — tắt nhầm có
  thể mất chức năng khác (loa/mic/estop cứng). Voice/loa đi qua service "voice"
  RIÊNG, KHÔNG phải service chuyển động → không đụng.

### Test
`pgrep -a run_r1` = 1 dòng; sau reboot chỉ run_r1 chạy; robot phản hồi mượt, không
tự giật khi chưa bấm gì.

---

## P0-2. Damp khi nhận SIGTERM/SIGINT

**Rủi ro hiện tại:** `systemctl stop/restart` gửi **SIGTERM** → process **chết ngay**,
KHÔNG chạy `sender_.Damping()` (chỉ chạy khi thoát bằng ESC/hết vòng). Robot dựa
vào low-level timeout để mềm ra → có **cửa sổ rủi ro** giữ lệnh cuối.

### Thiết kế (nhỏ, an toàn)
- `main.cpp`: cài handler cho `SIGTERM` + `SIGINT` → **chỉ** đặt cờ
  `std::atomic<bool> g_stop=true` (async-signal-safe, không gọi DDS trong handler).
- `Application::Run()`: vòng lặp kiểm `g_stop` → `running_=false` → thoát vòng →
  **`sender_.Damping()` cuối hàm chạy như thường** (trên main thread, an toàn).
- `hb_high_level.service`: thêm `TimeoutStopSec=3` để systemd **chờ** damp xong
  rồi mới SIGKILL (mặc định 90s cũng đủ, nhưng ghi rõ 3s cho gọn).

### Files chạm
`src/main.cpp` (handler + cờ), `src/app/Application.hpp/.cpp` (đọc cờ trong vòng),
`scripts/hb_high_level.service` + `install_service.sh` (`TimeoutStopSec`).

### Test
`systemctl stop hb_high_level` → log thấy "Đang thoát — xả motor"; robot mềm ngay,
không giữ cứng tư thế cuối.

---

## P0-3. Giám sát PIN thấp — ✅ ĐÃ GỠ CHẶN (2026-07-15)

**Rủi ro:** chạy độc lập lâu → pin cạn giữa chừng → mất nguồn động cơ → **ngã**.
Hiện **không đọc pin**.

### Nguồn dữ liệu (ĐÃ ĐO, không phải đoán)

Pin **KHÔNG** nằm trong `LowState_` (bản thảo cũ sai). Nó ở topic riêng:

**`rt/lf/bmsstate`** → `unitree_hg::msg::dds_::BmsState_` → **20.2 Hz** ✅ đọc thử chạy tốt

| Trường | Kiểu | Đo được lúc test |
|---|---|---|
| `soc()` | `uint8_t` | **90** (% pin) ← **dùng cái này** |
| `soh()` | `uint8_t` | 99 (% sức khoẻ pin) |
| `current()` | `int32_t` | −2352 (âm = đang xả) |
| `bmsvoltage()[0]` | `uint32_t` | 36540 (≈36.5 V) |

(Còn `rt/lf/secondary_bmsstate` — pin phụ. Chưa dùng; cân nhắc giám sát luôn sau.)

### Thiết kế
- Subscriber **riêng** cho `rt/lf/bmsstate` (20 Hz — **không** đọc trong vòng 500Hz).
- 2 ngưỡng + throttle (nói 1 lần mỗi ~60s, không spam):
  - **Cảnh báo** (`battery_warn_pct`, vd 20%): voice "pin yếu, còn X phần trăm".
  - **Nguy cấp** (`battery_critical_pct`, vd 8%): theo `battery_critical_action`:
    `voice` (chỉ nói) | `sit` (tự ngồi ghế an toàn) | `damp` (xả lực chủ động).
- Nếu robot không có SoC sạch → dùng **điện áp** với ngưỡng volt tương ứng.

### Config thêm (tuning.yaml)
```yaml
battery_monitor_enabled: true
battery_topic: "rt/lf/bmsstate"  # ✅ ĐÃ XÁC MINH trên robot (BmsState_, 20Hz)
battery_warn_pct: 20
battery_critical_pct: 8
battery_critical_action: sit     # ✅ CHỐT (user 2026-07-15): sit — KHÔNG damp
battery_announce_period_s: 60    # throttle cảnh báo
battery_stale_s: 30              # không nhận BmsState quá lâu -> cảnh báo "mất tín hiệu pin"
voice_battery_low: "Pin yếu, hãy đưa robot về sạc"
voice_battery_critical: "Pin cạn, robot ngồi xuống"
```

> ✅ **Vì sao `sit` chứ không `damp`:** xả lực khi robot **đang đứng** = **ngã**. Phải cho nó
> **tự ngồi xuống thấp** (đi qua đúng luồng ngồi 4 pha đã có) rồi mới xả. Cùng nguyên tắc
> với cả hệ thống: *không tắt policy giữ thăng bằng khi robot đang tự đứng một mình*.

> ⚠ **`battery_stale_s` là bắt buộc.** Nếu subscribe nhầm topic thì sẽ **không nhận gì**
> — mà "không nhận gì" **trông y hệt** "pin đầy, không có cảnh báo". Phải phát hiện được
> **im lặng** và báo, nếu không thì tính năng giám sát pin **tệ hơn là không có**.

### Files chạm
`src/safety/BatteryMonitor.hpp` (mới — subscriber riêng + throttle), `Application.cpp`
(khởi tạo + hành động khi nguy cấp), `Tuning.hpp/.cpp`, `tuning.yaml`, docs.

### Test
Giả lập ngưỡng cao (đặt `battery_warn_pct` ~95) để nghe voice; kiểm `sit`/`damp` có
người đỡ. Kiểm luôn: **rút/đổi tên topic sai** → phải nghe cảnh báo "mất tín hiệu pin",
KHÔNG được im lặng.

---

## Thứ tự đề xuất & rủi ro (soát 2026-07-15)

| | Việc | Code offline được? | Chặn bởi |
|---|---|---|---|
| **1** | **P0-2** SIGTERM → damp | ✅ **được ngay** | không gì. `Run()` đã sẵn `Damping()` ở lối thoát, chỉ cần bắt tín hiệu |
| **2** | **P0-1** cổng `kDisarmed` 2 lớp + 2-bis | ✅ code được, nhưng **để `arm_gate_enabled: false`** | cần **robot** để verify lớp 2 **thật sự nghe** được built-in (`foreign_count_` > 0) |
| **3** | **P0-3** giám sát pin | ❌ **không** | cần **robot** để tìm **tên topic** `BmsState_` |

**P0-1 + P0-2 ràng buộc chéo** ở lối thoát: *"không damp nếu chưa arm"* (Damping cũng là
ghi `rt/lowcmd`). Nên code P0-2 phải để sẵn chỗ móc cờ `armed_` vào.

Tất cả đi qua đường an toàn sẵn có (Damping / state machine), không bypass. Test
đầu tiên **có người đỡ/treo robot**.

⛔ **Cho tới khi P0-1 chạy và được verify: KHÔNG bật auto-start.**
`systemctl is-enabled hb_high_level` phải là **disabled**.

---

## Cách làm auto-start AN TOÀN (chọn 1) — có xét L2+R2 là thao tác thủ công

| Cách | Làm gì | Ưu / Nhược |
|---|---|---|
| **A. Disable built-in vĩnh viễn** | `systemctl disable --now <builtin>` → nó không bao giờ tự chạy; **khỏi cần bấm L2+R2**; run_r1 auto-start là nguồn duy nhất | Sạch nhất, headless thật. **Chỉ dùng được NẾU** "dev mode" thực chất là 1 systemd service tắt được (kiểm bằng chẩn đoán). Mất app gốc (bạn chỉ dùng run_r1). ⚠ chắc chắn không tắt nhầm safety cứng/loa (loa đi service "voice" riêng) |
| **B. run_r1 PASSIVE-UNTIL-ARMED (cổng 2 lớp)** ⭐ | run_r1 vẫn auto-start nhưng **KHÔNG publish `rt/lowcmd` nào (kể cả Damping)** cho tới khi arm. **Arm = giữ L2+R2 ≥5s (ý định người) VÀ built-in im lowcmd ≥400ms (xác nhận đã nhả)** — xem §THIẾT KẾ CHI TIẾT bên dưới | Tôn trọng thiết kế hãng, an toàn **kể cả khi dev mode là combo firmware**; chặn được **cả boot-race lẫn ca "giữ L2+R2 không ăn"**; chặn luôn sự cố **lỡ tay** mở run_r1. Không đụng cấu hình gốc. Đổi lại: thêm state `kDisarmed` + logic arm; **mất auto-recovery** khi crash (phải giữ L2+R2 lại) |
| **C. KHÔNG auto-start** | Bỏ `systemctl enable`; bật nguồn → L2+R2 → rồi mới `systemctl start` (cần SSH/laptop) | An toàn nhờ người, nhưng **mất tính headless** (phải có cách start run_r1 sau L2+R2) |

> **Đề xuất: B (passive-until-armed)** — vì dev mode là **combo L2+R2 (firmware), thao
> tác tay mỗi lần bật**, nên để run_r1 tự chạy nhưng "im" cho tới khi bạn vào dev mode
> là an toàn nhất mà vẫn headless. **A** dùng nếu chẩn đoán cho thấy built-in chỉ là 1
> service tắt được và bạn OK bỏ hẳn app gốc.

### ✅ Bằng chứng thực tế (quyết định thiết kế)

**User (2026-07-13):** trước khi cài auto, **lỡ tay mở run_r1 khi CHƯA vào dev mode
→ xung đột, robot PHẢN ỨNG DỮ DỘI.**

⇒ Kết luận chắc: **factory mode CÓ tuồn `rt/lowstate` sang PC2** → run_r1 vào IDLE và
publish `rt/lowcmd` (Damping xen kẽ lệnh built-in trên **cùng topic** → motor nhận lệnh
mâu thuẫn ở tần số cao → giật dữ dội). **KHÔNG "an toàn sẵn" — BẮT BUỘC có cổng chặn.**
Sự cố lặp lại được **bất cứ khi nào** mở run_r1 (tay/auto) chưa vào dev mode.

**Tạm thời tới khi có cổng:** CHỈ mở run_r1 SAU khi chắc đã vào dev mode. KHÔNG bật auto.

---

## THIẾT KẾ CHI TIẾT — Cổng `kDisarmed` 2 LỚP

### Vì sao phải 2 lớp (không dùng riêng lớp nào)

| Chỉ nghe DDS-silence | Chỉ nghe nút L2+R2 |
|---|---|
| ❌ **Dính BOOT-RACE:** lúc bật nguồn, built-in **chưa kịp** publish; run_r1 thấy "im" 400ms → **arm nhầm** → built-in init xong publish tiếp → **xung đột**. Tăng `arm_silence_ms` không cứu (built-in có thể mất vài giây mới chạy). | ❌ **L2+R2 finicky:** giữ 5s **có khi vẫn chưa chuyển**. Arm chỉ vì thấy nút → built-in **vẫn đang chạy** → xung đột. |

⇒ **Arm chỉ khi CẢ HAI đúng:**
1. **Ý ĐỊNH NGƯỜI** — đã giữ **L2+R2 liên tục ≥ `arm_hold_s` (5s)**. Chặn boot-race
   (lúc boot chưa ai bấm → không bao giờ arm).
2. **XÁC NHẬN KỸ THUẬT** — built-in đã **im `rt/lowcmd` ≥ `arm_silence_ms` (400ms)**.
   Chặn ca "giữ 5s nhưng dev mode không ăn" (built-in còn phát → chưa arm).

Hai lớp bù **đúng** điểm yếu của nhau.

### Máy trạng thái
```
kDisarmed ──(ĐK1 giữ L2+R2 ≥5s  VÀ  ĐK2 lowcmd im ≥400ms)──▶ kWaitingForState ──▶ kIdle ──▶ ...
                                                                      │
        (BẤT KỲ lúc nào đang armed mà thấy lowcmd LẠ)                 │
kConflict ◀───────────────────────────────────────────────────────────┘
   └─ im hoàn toàn, LATCH, chỉ thoát bằng arm lại có chủ đích
```
Trong `kDisarmed` **và** `kConflict`: **KHÔNG** gọi `sender_.Send`/`Damping`, **KHÔNG**
chạy policy/watchdog → **im tuyệt đối trên `rt/lowcmd`** (không có gì để đè built-in).

### Lớp 1 — chốt ý định (đọc nút từ lowstate)
run_r1 vẫn nhận `rt/lowstate` ở factory mode (đã chứng minh) → đọc `wireless_remote`:
```cpp
// mỗi tick trong kDisarmed
bool both = remote.L2 && remote.R2;
hold_s_ = both ? (hold_s_ + kLoopDt) : 0.0f;      // đứt tay là reset
if (hold_s_ >= tuning_.arm_hold_s) intent_latched_ = true;  // CHỐT, giữ luôn
```
- **Latch:** giữ đủ 5s là **chốt vĩnh viễn** — người không phải giữ tiếp trong lúc
  built-in nhả. Giữ hụt (đứt tay) → reset về 0, phải giữ lại.
- Nếu L2+R2 không chuyển được dev mode → built-in còn phát → **ĐK2 chưa đạt → vẫn chưa arm**;
  người cứ giữ lại lần nữa cho tới khi ăn.

### Lớp 2 — phát hiện built-in, chạy LIÊN TỤC (trước VÀ sau arm)

> ⚠ **Sửa lỗ hổng (user phát hiện 2026-07-13):** bản nháp trước có `if (armed_) return;`
> → sau khi arm, run_r1 **mù** với `rt/lowcmd`. Nếu built-in **quay lại** lúc đang chạy
> (thoát dev mode, built-in restart…) → **không phát hiện được → xung đột dữ dội lần nữa.**
> ⇒ Detector phải **chạy mãi**, và phải **phân biệt được gói của CHÍNH MÌNH**.

**Nhận diện gói của mình** (2 cách, đều KHÔNG phụ thuộc DDS loopback):

| Cách | Làm gì | Đánh giá |
|---|---|---|
| (a) Đóng dấu `reserve[0]` | `LowCmd_` có sẵn `std::array<uint32_t,4> reserve` → đặt `reserve[0] = kHbMagic (0x48425231)` **trước** khi tính CRC (reserve nằm TRƯỚC crc trong struct → CRC bao luôn, không lệch) | Tất định, đơn giản. ⚠ rủi ro nhỏ nếu low-level có đọc `reserve` |
| **(b) Đối chiếu CRC** ⭐ | Ring 16 CRC vừa publish; gói nhận về **CRC trùng → của mình**, không trùng → **của built-in** | **KHÔNG đổi một byte nào gửi xuống motor → rủi ro = 0.** Đụng độ CRC32 ≈ không thể. **CHỌN CÁI NÀY** |

```cpp
void Application::OnLowCmdSeen(const void* msg) {
    const auto& c = *static_cast<const LowCmd_*>(msg);
    if (IsOurCrc(c.crc())) return;              // gói của chính mình -> bỏ
    last_foreign_lowcmd_ms_ = NowMs();          // atomic (thread DDS)
    ++foreign_count_;
    if (armed_ && <đủ arm_conflict_min gói lạ trong arm_conflict_window_ms>)
        conflict_flag_ = true;                  // 🔴 BUILT-IN QUAY LẠI KHI ĐANG CHẠY
}
```

> ⚠ **Sửa robustness (2026-07-15):** KHÔNG bật conflict ngay ở **1 gói lạ** — một gói đến trễ
> lúc bàn giao (hoặc trùng CRC hi hữu) sẽ nhả quyền oan giữa lúc đang đi -> ngã. Chỉ bật khi
> thấy **≥ `arm_conflict_min` (3) gói lạ trong `arm_conflict_window_ms` (150ms)**. Built-in thật
> phun ~621 Hz nên vượt ngưỡng trong ~3ms -> vẫn ngắt gần như tức thì, nhưng miễn nhiễm gói lạc.
`last_foreign_lowcmd_ms_` khởi tạo = lúc bắt đầu vòng lặp → luôn có cửa sổ quan sát tối
thiểu, không arm tức thì.

### 🔴 Lớp 2-bis — CHỐNG DETECTOR MÙ (lỗ hổng phát hiện 2026-07-15)

**Lỗ hổng của bản thảo cũ:** lớp 2 kết luận *"400ms không thấy `lowcmd` lạ → built-in đã nhả"*.
Nhưng nếu run_r1 **KHÔNG NHÌN THẤY ĐƯỢC** `lowcmd` của built-in (khác DDS domain, PC1 vs PC2,
khác card mạng, QoS lệch…) thì **im lặng LUÔN ĐÚNG** → **lớp 2 luôn pass** → cổng rút xuống
còn **mỗi lớp 1**.

Mà lớp 1 chính là cái **hay hụt** (*"giữ 5s có khi chưa ăn"*). Ghép lại ra **đúng thảm hoạ**:

> Giữ L2+R2 5s → dev mode **KHÔNG ăn** → built-in **vẫn chạy** → detector **mù** nên báo
> "im lặng" → run_r1 **ARM** → **2 nguồn lệnh đánh nhau.**

**Gốc lỗi (nhớ kỹ):** *"chưa bao giờ thấy built-in"* **KHÔNG PHẢI** *"built-in đã tắt"*.
Bản thảo cũ lẫn hai cái đó. Im lặng chỉ có nghĩa **khi đã chứng minh mình nghe được**.

**Bịt:** đòi **bằng chứng dương** — phải **đã từng đếm được ≥ `arm_min_foreign_seen` gói
`lowcmd` của built-in** thì mới được tin vào im lặng.

```cpp
bool ok_silence = (silence >= tuning_.arm_silence_ms) &&
                  (!tuning_.arm_require_seen_builtin ||
                   foreign_count_.load() >= tuning_.arm_min_foreign_seen);
```

| Kịch bản | `foreign_count_` | Kết quả | Đúng chưa |
|---|---|---|---|
| **Auto-start lúc boot** (built-in đang sống) | tăng đều → rồi im khi nhả | **arm** | ✅ đúng mục đích P0-1 |
| **Detector mù**, built-in vẫn chạy | **= 0 mãi** | **KHÔNG arm**, log to | ✅ **bịt được thảm hoạ** |
| **Chạy tay SAU khi đã vào dev mode** | = 0 (hợp lệ) | **KHÔNG arm** | ⚠ phải đặt `arm_require_seen_builtin: false` |

⇒ **`arm_require_seen_builtin: true` là dành cho AUTO-START** (đúng mục đích của P0-1).
Khi **chạy tay** để test thì đặt **false** — vì lúc đó **bạn** là người bảo đảm thứ tự.
Ghi rõ 2 chế độ này vào `huong_dan_van_hanh.md`.

### 🔴 CƠ CHẾ NGẮT — built-in quay lại lúc đang armed

**Bắt buộc: KHÔNG được Damping.** Damping **cũng là ghi `rt/lowcmd`** → vẫn 2 nguồn đánh
nhau. Hành động **duy nhất** không xung đột là **ngừng publish**.

```cpp
// đầu Tick(), trước mọi thứ khác
if (conflict_flag_.load() && armed_) {
    armed_ = false;                 // NGỪNG PUBLISH NGAY (không Send, không Damping)
    state_ = AppState::kConflict;   // latch — KHÔNG tự arm lại
    std::cerr << "\n[FATAL] 🔴 PHÁT HIỆN LOWCMD LẠ khi đang điều khiển — "
                 "BUILT-IN QUAY LẠI. Đã NHẢ QUYỀN (ngừng publish).\n";
    music_.Say(tuning_.voice_conflict, ...);   // "Phát hiện xung đột điều khiển, đã nhả quyền"
}
```
- **`kConflict` = im hoàn toàn**, không publish gì; **không tự arm lại** (tránh bật-tắt
  liên tục). Muốn chạy lại phải **arm lại có chủ đích** (giữ L2+R2 + built-in im).
- **Cái giá phải trả (nói thẳng):** nhả quyền **giữa lúc robot đang đi** → built-in tiếp
  quản → **có thể giật/ngã**. Nhưng **KHÔNG tồn tại** hành động nào vừa "không publish"
  vừa "làm robot an toàn" — mâu thuẫn nội tại. **Đánh nhau kéo dài chắc chắn tệ hơn**
  (= "phản ứng dữ dội" đã gặp). ⇒ Nhả ngay + cảnh báo to, người bấm **E-stop cứng**.

### Điều kiện ARM (gộp)
```cpp
bool Application::TickDisarmed() {
    UpdateArmIntent();                                   // lớp 1
    int64_t silence = NowMs() - last_foreign_lowcmd_ms_.load();   // lớp 2
    bool ok_intent  = !tuning_.arm_require_button || intent_latched_;
    // lớp 2 + 2-bis: im lặng CHỈ đáng tin khi đã chứng minh mình nghe được built-in
    bool can_hear   = !tuning_.arm_require_seen_builtin ||
                      foreign_count_.load() >= tuning_.arm_min_foreign_seen;
    bool ok_silence = (silence >= tuning_.arm_silence_ms) && can_hear;

    if (ok_intent && ok_silence) {
        armed_ = true;
        std::cout << "[Application] ARM — built-in đã nhả (" << silence
                  << "ms im), ý định người OK.\n";
        music_.Say(tuning_.voice_armed, ...);
        state_ = AppState::kWaitingForState;             // vào luồng cũ
    } else if (tick_ % 500 == 0) {                       // LOG 1Hz tự-kiểm
        std::cout << "[kDisarmed] chờ bàn giao | giữ L2+R2: " << hold_s_ << "/"
                  << tuning_.arm_hold_s << "s (chốt=" << intent_latched_
                  << ") | gói lowcmd built-in đã thấy: " << foreign_count_
                  << " | im: " << silence << "ms\n";
    }
    return true;   // KHÔNG publish gì
}
```

### Lối thoát — KHÔNG damp nếu chưa arm (QUAN TRỌNG)
Damping cũng là **ghi `rt/lowcmd`** → nếu thoát lúc disarmed mà damp là **lại đè built-in**:
```cpp
if (armed_) { sender_.Damping(); sleep(200ms); }   // disarmed -> thoát IM LẶNG
```
→ Áp cho **cả** lối thoát thường **và** SIGTERM của P0-2. (Ràng buộc chéo giữa P0-1 và P0-2.)

### LOG tự-kiểm — bắt buộc ở lần chạy đầu trên robot
Log 1Hz ở trên in `foreign_count_`. Ở factory mode, khi built-in đang chạy:
- **`foreign_count_` tăng đều (>0)** → run_r1 **thấy** lowcmd của built-in → **lớp 2 tin được**.
- **= 0 mà built-in rõ ràng đang chạy** → run_r1 **mù** với lowcmd built-in → **KHÔNG tin lớp 2**
  → phải chuyển sang **cách A (disable built-in)**.

Cũng kiểm luôn: khi bạn giữ L2+R2, log có thấy `hold_s_` tăng không → xác nhận
**run_r1 đọc được nút ở factory mode**. Nếu **không** thấy → factory mode không tuồn nút
sang PC2 → tắt lớp 1 (`arm_require_button: false`) và tìm cách khác cho ý định người.

> Đây là bước **chỉ đọc log**, robot vẫn do built-in giữ, chưa cho đứng → an toàn.

### Config (tuning.yaml)
```yaml
arm_gate_enabled: true          # bật cổng kDisarmed. false = hành vi cũ (CHỈ khi đã disable built-in)
arm_require_button: true        # lớp 1: phải giữ L2+R2 mới arm (chống boot-race)
arm_hold_s: 5.0                 # giữ L2+R2 liên tục bấy nhiêu giây = ý định bàn giao
arm_silence_ms: 400             # lớp 2: built-in im lowcmd bấy nhiêu ms = đã nhả thật

# lớp 2-bis (CHỐNG DETECTOR MÙ):
arm_require_seen_builtin: true  # true = AUTO-START. Chỉ tin "im lặng" SAU KHI đã thật sự
                                #        nghe được lowcmd của built-in ít nhất 1 lần.
                                # false = CHẠY TAY (bạn tự bảo đảm đã vào dev mode trước).
arm_min_foreign_seen: 5         # số gói lowcmd built-in tối thiểu phải thấy để tin detector

voice_armed: "Đã sẵn sàng nhận điều khiển"
voice_conflict: "Phát hiện xung đột điều khiển, đã nhả quyền"   # khi built-in quay lại
```

> ⚠ **Đặt `arm_require_seen_builtin` sai là hỏng cả cổng.** Để `true` mà chạy tay sau khi
> đã vào dev mode → không bao giờ arm (khó chịu nhưng an toàn). Để `false` mà auto-start
> → **mất lớp 2-bis, quay lại đúng lỗ hổng cũ** (nguy hiểm). Nghi ngờ thì để `true`.

### Các ca biên
- **E-stop lúc disarmed:** run_r1 không publish → không damp được — nhưng **đúng**
  (built-in đang giữ quyền, mình không xen vào). Arm rồi thì E-stop chạy như thường.
- **run_r1 restart giữa lúc đang dev mode:** vào disarmed; built-in đã tắt nên lowcmd im
  (ĐK2 ✓), nhưng **ĐK1 chưa có** (chưa ai bấm lại) → **phải giữ L2+R2 lần nữa để arm**.
  ⚠ Đây là **giá phải trả** của lớp 1: restart tự động (crash) sẽ **không tự về chạy** mà
  chờ người. **An toàn hơn nhưng kém tự phục hồi** — cân nhắc; muốn tự phục hồi thì đặt
  `arm_require_button: false` (chấp nhận rủi ro boot-race) hoặc dùng cách A.
- **Không bao giờ vào dev mode:** run_r1 nằm disarmed **vĩnh viễn, im lặng** — an toàn.
- **Khoảng trống** giữa lúc built-in nhả và run_r1 arm: robot **mềm vài trăm ms** — lúc
  bật nguồn robot phải được **đỡ/treo** (vốn đã là quy tắc).

### Files chạm
- `LowCmdSender.hpp` — **ring 16 CRC vừa publish** + `bool IsOurCrc(uint32_t) const`
  (để detector phân biệt gói của mình). Không đổi nội dung gói gửi robot.
- `Application.hpp` — `AppState::kDisarmed`, **`AppState::kConflict`**; members `armed_`,
  `intent_latched_`, `hold_s_`, `last_foreign_lowcmd_ms_` (atomic), `foreign_count_`
  (atomic), **`conflict_flag_` (atomic)**, `lowcmd_sub_`; khai **`OnLowCmdSeen()`**,
  `TickDisarmed()`, `UpdateArmIntent()`.
- `Application.cpp` — `InitDds()` subscribe `rt/lowcmd`; `Run()` state đầu = `kDisarmed`,
  **guard damp theo `armed_`**; `Tick()` nhánh disarmed + **kiểm `conflict_flag_` ĐẦU
  TIÊN mỗi tick** → nhả quyền + `kConflict`.
- `Tuning.hpp/.cpp` + `tuning.yaml` — 5 khóa `arm_*` + `voice_armed` + `voice_conflict`.
- `docs/huong_dan_van_hanh.md` — quy trình bật nguồn mới (giữ L2+R2 5s → nghe "đã sẵn
  sàng nhận điều khiển" → mới điều khiển được) + xử lý khi nghe cảnh báo xung đột.

---

## Chốt / còn treo

**Đã chốt:**
1. ✅ Dev mode = **GIỮ L2+R2 đủ lâu**, thủ công mỗi lần bật nguồn (giữ 2s có khi chưa ăn).
2. ✅ Factory mode **CÓ** tuồn lowstate sang PC2 → run_r1 publish → **xung đột thật**
   (bằng chứng: sự cố robot phản ứng dữ dội). ⇒ **BẮT BUỘC có cổng.**
3. ✅ Cổng **`kDisarmed`**: lớp 1 (L2+R2 ≥5s) **VÀ** lớp 2 (lowcmd im ≥400ms) **VÀ**
   lớp 2-bis (**đã chứng minh nghe được built-in**) — thiết kế ở trên.
4. ✅ **P0-2 làm trước, làm ngay** — không phụ thuộc robot, và hiện `systemctl stop` =
   robot **rơi cứng**.
5. ✅ **P0-3 hoãn** — pin KHÔNG nằm trong `LowState_`; cần tên topic `BmsState_` từ robot.

6. ✅ **CHỐT (user 2026-07-15) — CHẤP NHẬN mất tự-phục-hồi.** Bật `arm_require_button: true`:
   run_r1 crash/restart sẽ **KHÔNG tự chạy lại**, phải có người giữ L2+R2. An toàn > tiện.
7. ✅ **CHỐT (user 2026-07-15) — pin nguy cấp = `sit`** (ngồi xuống rồi mới xả, KHÔNG damp
   khi đang đứng).
8. ✅ **Lớp 2 đã được CHỨNG MINH trên robot** — PC2 nghe thấy `rt/lowcmd` của built-in ở
   621 Hz. Detector không mù. (Xem §ĐO TRÊN ROBOT THẬT.)
9. ✅ **Topic pin = `rt/lf/bmsstate`** — đã xác minh, đọc được `soc` = 90%.

10. ✅ **Lớp 1 ĐÃ XÁC MINH trên robot (2026-07-15).** Giữ L2+R2 → `wireless_remote` byte[2..3]
    = **`0x0030`** (bit4 R2 + bit5 L2), đọc được liên tục 7 giây. Factory mode **CÓ** tuồn nút
    sang PC2. **Không còn ẩn số nào** — cả 2 lớp đều chạy được trên phần cứng này.

## 🔒 Cách A (disable built-in) — BẤT KHẢ THI trên phần cứng này

Khảo sát 2026-07-15 (xem `HE_THONG_ROBOT.md`): built-in nằm trên **bo riêng
`192.168.123.161`**, Linux headless, **MỌI cổng TCP đóng — không SSH/web/telnet**, chỉ nói
DDS/UDP. ⇒ **Không đăng nhập được để `systemctl disable`.** Cách **duy nhất** làm nó im là
**bắt tay dev-mode qua L2+R2** (đã đo: `rt/lowcmd` 621 Hz → 0). **Bắt buộc cách B.**
