# Hướng dẫn vận hành — HB R1 High-Level

Tài liệu này hướng dẫn chạy `high_level_2` theo **2 cách**:

- **Cách A — Headless + Auto (thực địa):** điều khiển bằng **tay cầm R3-1**, chương
  trình tự chạy nền trong robot. **Mất wifi/SSH vẫn chạy bình thường.** ← dùng chính.
- **Cách B — Chạy tay có bàn phím (dev/test):** dùng bàn phím laptop qua `ssh -Y`.
  Chỉ dùng khi ngồi máy phát triển; **mất SSH sẽ dừng**.

> ⚠ **KHÔNG chạy đồng thời 2 bản `run_r1`.** Cả hai cùng ra lệnh động cơ (`rt/lowcmd`)
> → xung đột, nguy hiểm. Muốn chạy Cách B phải **tắt bản Auto trước** (xem §4).
>
> 🛑 **XUNG ĐỘT với controller BUILT-IN (bo .161):** khi bật nguồn, built-in gốc **tự
> lái và publish `rt/lowcmd`**. Hai nguồn cùng ra lệnh = **đánh nhau, PHÁ ROBOT**.
>
> **Cơ chế "QUAN SÁT" (cổng arm):** run_r1 auto-start nhưng **nằm im**, chỉ theo dõi
> built-in. Nó **chỉ tiếp quản** khi: (1) đã NGHE built-in đang lái, (2) thấy built-in
> **im hẳn** (bạn L2+R2), (3) bạn **giữ R1+R2** xác nhận. Đang lái mà thấy built-in
> phát lại → **NGẮT NGAY**.
>
> ⚠ **Quy trình bàn giao 2 nút (làm đúng thứ tự):**
>
> 1. Bật nguồn → run_r1 tự chạy, **nằm im** (built-in đang lái).
> 2. Giữ **L2+R2** → đưa **built-in** vào dev mode (robot **rũ mềm** = built-in đã buông).
>    **Bấm cho BUILT-IN.** L2+R2 là **TOGGLE** — chỉ bấm **một lần**, bấm lại là bật built-in dậy.
> 3. Giữ **R1+R2** ~5s → **run_r1 tiếp quản**, nói *"Máy tính phát triển đã sẵn sàng"*.
>    (R1+R2 là combo riêng của run_r1, KHÔNG toggle built-in.)
>
> 🔒 **AN TOÀN:** run_r1 **không bao giờ arm nếu chưa nghe được built-in** (chống arm mù khi
> DDS lỗi). Nếu bấm R1+R2 mà chưa L2+R2 (built-in còn sống) → **từ chối arm, đứng im** →
> không xung đột. Nếu DDS crash làm run_r1 điếc → cũng **không arm** (thà đứng im còn hơn
> đánh nhau). Cái giá: restart run_r1 lúc built-in đã tắt sẵn → phải **cold-boot** để arm lại.
> (Config: `arm_require_button: true`, `arm_no_builtin_timeout_s: 0` — xem §1.)

---

## 1. Cấu hình tuning.yaml liên quan

| Khóa                 | Ý nghĩa                                         | Đặt cho Cách A (headless)                                                                                                      |
| --------------------- | ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `network_interface` | card mạng DDS của robot                         | **⚠ Phải ĐÚNG.** Auto không truyền tham số → lấy từ đây. Kiểm bằng `ip a` trên robot (thường `eth10`). |
| `dev_no_keyboard`   | `true` = không mở bàn phím X11, thuần R3-1 | **`true`** (thuần R3-1, chắc chắn không dính lỗi đứt X11).                                                        |
| `voice_enabled`     | bật giọng nói thông báo                      | **`true`** — headless không màn hình, **giọng nói là kênh phản hồi chính**.                              |
| `safe_stop_enabled` | mất hết input thì đứng yên (không sụp)    | **`true`**.                                                                                                               |

Các tham số khác (gains, tốc độ, dance, ngồi ghế…) không liên quan cách chạy — chỉnh
theo nhu cầu, sửa xong **chạy lại** là ăn (bản Auto cần `systemctl restart`).

---

## 2. Cài Auto (systemd) — LÀM 1 LẦN

> 🛑 **BẮT BUỘC TRƯỚC KHI BẬT AUTO:** phải đảm bảo controller **built-in của Unitree
> đã TẮT** (không thì xung đột `rt/lowcmd` → phá robot — xem cảnh báo đầu tài liệu và
> [PLAN_standalone_P0.md](PLAN_standalone_P0.md) §P0-1). Chưa xử lý xong việc này thì
> **CHƯA `systemctl enable`** — chạy tay Cách B (có kiểm soát) an toàn hơn.

Trên **máy dev**, đẩy code sang robot:

```bash
cd ~/…/HB/high_level_2
bash scripts/deploy_to_robot.sh          # rsync + build từ xa
```

Trên **robot** (SSH vào):

```bash
cd ~/HB/high_level_2
bash scripts/build.sh                    # nếu chưa build
bash scripts/install_service.sh          # cài dịch vụ auto (cần sudo)
```

Sau bước này, dịch vụ tên **`hb_high_level`** đã được cài + bật tự khởi động.

---

## 3. CÁCH A — Vận hành headless bằng R3-1 (hằng ngày)

### 3.1 Bật

- **Bật nguồn robot** → chương trình **tự chạy**, sẵn sàng nhận R3-1. Không cần laptop.
- Hoặc bật tay: `sudo systemctl start hb_high_level`.
- Robot nói **"Máy tính phát triển đã sẵn sàng"** khi kết nối xong.

### 3.2 Bảng điều khiển đầy đủ (R3-1 + bàn phím)

Cùng một chức năng, hai kênh: **R3-1** (Cách A, headless) hoặc **bàn phím**
(Cách B, `ssh -Y`). "—" = kênh đó không có nút cho chức năng ấy.

| Chức năng                                                                            | R3-1 (tay cầm)                                          | Bàn phím                             |
| -------------------------------------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------- |
| **Bàn giao**: built-in→dev mode, rồi arm run_r1                               | **L2+R2** *(1 lần)* → giữ **R1+R2** ~5s | —                                     |
| Khóa đứng / gồng cứng (STAND LOCK)*(đang NHẢY → huỷ điệu, xem ⚠ dưới)* | **L2 + Lên**                                      | **0**                            |
| Bật đi bộ (LOCOMOTION)*(phải đang STAND LOCK)*                                  | **R2 + A**                                         | **1**                            |
| Nhảy điệu 2 —`lacmong1`                                                          | **R1 + Lên**                                      | **2**                            |
| Nhảy điệu 3 —`lacmong2`                                                          | **R1 + Phải**                                     | **3**                            |
| Nhảy điệu 4 —`pokemon`                                                           | **R1 + Xuống**                                    | **4**                            |
| Nhảy điệu 5 —`doremon`                                                           | **R1 + Trái**                                     | **5**                            |
| Nhảy điệu 6                                                                         | **R1 + A**                                         | **6**                            |
| Nhảy điệu 7 / 8                                                                     | —                                                       | **7 / 8**                        |
| Tiến / Lùi                                                                           | **Stick trái ↑ / ↓**                            | **W / S**                        |
| Đi ngang trái / phải                                                                | **Stick trái ← / →**                            | **A / D**                        |
| Xoay trái / phải                                                                     | **Stick phải ← / →**                            | **Q / E**                        |
| Tốc độ NHANH                                                                        | **R2 + Lên**                                      | **Tab** *(đảo nhanh↔chậm)* |
| Tốc độ CHẬM                                                                        | **R2 + Xuống**                                    | **Tab** *(đảo nhanh↔chậm)* |
| Reset policy                                                                           | —                                                       | **R**                            |
| Ngồi xuống ghế                                                                      | **L2 + Trái**                                     | **9**                            |
| Nằm xuống*(từ STAND LOCK / đi bộ)*                                                | **L2 + X**                                         | —                                     |
| Đứng dậy*(đang nằm)* —**B1** chuẩn bị · **B2** đứng dậy        | **L2 + Lên** → **L2 + X**                  | —                                     |
| **DỪNG KHẨN CẤP** (xả lực)                                                  | **L2 + B** *(giữ)*                              | **ESC** *(dừng + thoát app)* |
| **XẢ LỰC HOÀN TOÀN** (zero-torque, bẻ khớp được) *(chỉ từ IDLE)*    | **L2 + Y** *(bật/tắt)*                         | —                                     |

> **Khác biệt cần nhớ:**
>
> - Điệu **7 / 8** và **Reset (R)** chỉ có trên bàn phím — tay cầm không map.
> - Tốc độ: tay cầm có 2 nút riêng (nhanh/chậm), bàn phím dùng **Tab đảo qua lại**.
> - E-stop tay cầm **chỉ có một combo: L2 + B** (chuẩn Unitree, giữ là xả lực).
>   *⚠ Tránh combo của HÃNG: **L2 + R2** = vào chế độ phát triển; **L1 + L2** = hãng
>   đã dùng. Đừng map chức năng của mình lên hai combo đó — và cẩn thận khi giữ L2:
>   lỡ chạm R2 là thành L2+R2 (đổi chế độ) giữa lúc đang chạy.*
> - E-stop chỉ xả lực (app vẫn chạy, đứng dậy lại bằng L2+Lên); **ESC** trên bàn
>   phím thì xả lực **rồi thoát** app.
> - **Zero-torque (L2+Y):** xả lực HOÀN TOÀN (limp, bẻ khớp được — khác Damping còn
>   hơi cứng). Bật từ IDLE. **Thoát bằng 3 cách:** L2+Y (về Damping), **L2+B** (E-stop
>   → Damping), hoặc **L2+Lên** (đứng dậy gồng cứng luôn). ⚠ Đứng thẳng mà bật zero-torque
>   là NGÃ — chỉ dùng khi robot đã nằm/được đỡ.
> - Tên điệu lấy từ tuning.yaml (`dance_2..8`). Thêm/đổi điệu: sửa tuning.yaml và
>   tạo folder tương ứng trong `policies/dance/`.

> ⏱ **Bấm KHÓA ĐỨNG hoặc NGỒI lúc robot đang ĐI / đang NHẢY → robot DỪNG LẠI trước
> (~1 giây) rồi mới làm.** Đây là **cố ý**, không phải lag: nếu gồng cứng ngay giữa
> sải chân (một chân còn trên không, còn quán tính) thì **robot ngã**. Nó giữ policy
> chạy để tự dừng và đứng vững, xong mới chuyển. **Cứ bấm bất cứ lúc nào — an toàn.**
> Chỉnh thời gian dừng: `settle_time_s` trong tuning.yaml.

> 🛑 **`0` / `L2+Lên` khi ĐANG NHẢY = HUỶ ĐIỆU, KHÔNG phải khóa cứng.**
> Robot tắt nhạc, **tự đứng thẳng người lên (~1s)**, rồi **về chế độ đi bộ, đứng yên tại chỗ**
> — policy **luôn chạy** suốt quá trình nên nó **tự giữ thăng bằng, không ngã**. Lý do:
> lúc đang nhảy robot đứng **một mình giữa sàn, không ai đỡ**; gồng cứng lúc đó là ngã.
>
> ⏱ **Khoảng ~1s "đứng thẳng người" đó là bình thường, đừng bấm lại.** Xem giải thích ở
> khối 🩰 bên dưới — bỏ bước này là robot **đạp loạn để gượng**. Chỉnh: `mimic_cooldown_s`.
>
> **Muốn gồng cứng thì bấm `0` LẦN NỮA** (lúc này đang ở đi bộ) — hãy bấm khi bạn **đã
> tới đứng cạnh robot**. Lần bấm thứ 2 **bị chặn trong 2 giây đầu** sau khi huỷ điệu
> (`dance_abort_lock_block_s`), phòng bấm nhầm 2 phát liên tiếp.
>
> Nghe là biết đang ở đâu: huỷ điệu → *"bật chế độ đi bộ"*; khóa cứng → *"đã khóa đứng"*.

### 3.3 Robot nói cho bạn biết (thay màn hình)

Không cần nhìn, robot đọc trạng thái: *"đã khóa đứng"*, *"bật chế độ đi bộ"*,
*"đang ngồi xuống"*, *"kích hoạt chế độ an toàn"*, tên điệu khi nhảy. Sửa nội dung
ở các dòng `voice_*` trong tuning.yaml.

> 🔊 **Voice "đã khóa đứng" phát TRƯỚC, không phải sau.** Bấm khóa cứng → robot **nói
> ngay**, chờ **1.5s** (`stand_lock_warn_s`) rồi **mới** ép cứng — để bạn kịp đưa tay đỡ.
> Đến từ **đi bộ** thì suốt 1.5s đó **policy vẫn chạy**, robot tự đứng vững, **không ngã**.
> Thứ tự đúng khi thao tác: **bấm phím trước, cầm/treo robot sau** — cầm trước rồi mới bấm
> thì policy sẽ thấy chân lơ lửng.

> **Khi bấm điệu nhảy**: robot **đứng yên tại chỗ đọc tên điệu ~2s** (vẫn đang chạy
> policy đi bộ nên **tự giữ thăng bằng**), đọc xong mới **vào tư thế mở màn của điệu**
> rồi bật nhạc + nhảy. Khoảng dừng này là **bình thường**, không phải treo. Chỉnh bằng
> `mimic_announce_delay_s` trong tuning.yaml (đặt ~ bằng độ dài file lời đọc).

> **Vì sao có ~1s robot "vào dáng" rồi mới có nhạc?** Tư thế chân của điệu nhảy **khác
> tư thế đứng 28–45°** — robot bắt buộc phải đổi tư thế chân để vào điệu. Nếu bật policy
> nhảy khi robot còn đứng thường, policy thấy sai lệch lớn ngay nhịp đầu → **giật mạnh, ngã**.
> Nên robot có bước **"khởi động mềm"** (`mimic_warmup_s: 1.2`): policy nhảy **bật ngay và
> tự đưa chân vào tư thế mở màn**, clip **đứng yên** chờ. Vì policy đang chạy nên robot
> **tự giữ thăng bằng** suốt lúc chân di chuyển. Vào dáng xong → **nhạc nổi + nhảy**.
>
> Tăng `mimic_warmup_s` (1.2 → 1.8) nếu vẫn thấy chông chênh lúc vào dáng; giảm nếu muốn
> vào nhạc nhanh hơn.

> 🩰 **Vì sao RA khỏi điệu cũng mất ~1s?** (`mimic_cooldown_s: 1.0`) — Chiều ngược lại của
> "vào dáng", và **cũng bắt buộc**. Khi bấm `0` hoặc khi **hết điệu**, robot đang ở **tư thế
> nhảy**: người cúi, chân sải, hai bên lệch nhau. Policy **đi bộ** chưa từng thấy tư thế đó
> → giao thẳng cho nó thì nó **đạp loạn xạ để gượng thăng bằng**.
>
> Nên robot có bước **"dừng mềm"**: **giữ policy nhảy chạy tiếp**, nhưng kéo mục tiêu của nó
> từ tư thế nhảy **về tư thế đứng**. Chính policy nhảy **tự đứng robot thẳng dậy** — vẫn giữ
> thăng bằng vì nó đang chạy. **Đứng thẳng xong** mới bàn giao cho policy đi bộ, lúc đó
> hai bên **cùng ở tư thế đứng** → êm.
>
> ⚙ **Bàn giao theo ĐIỀU KIỆN (không chỉ theo giờ).** Trước đây luôn chờ đủ `mimic_cooldown_s`
> giây mới giao — có bài (vd Pokemon) trong 1s đó robot lại **chúi tới ~33° gần ngã** rồi mới
> nhả cho đi bộ → đi bộ nhận thân đang đổ, **đá loạn**. Nay hễ robot đã **đứng đủ yên**
> (nghiêng ngang `< mimic_handover_tilt` VÀ gyro thân `< mimic_handover_gyro`) và qua
> `mimic_handover_min_s` thì **giao ngay** lúc còn gần thẳng; nếu mãi không yên thì tối đa vẫn
> giao ở `mimic_cooldown_s`. Muốn quay lại kiểu-theo-giờ: đặt `mimic_handover_min_s ≥ mimic_cooldown_s`.
>
> Áp dụng cho **cả 3 đường ra**: bấm `0` · hết điệu · bấm ngồi (`L2+Trái`) giữa lúc đang nhảy.
> **Nguyên tắc chung của cả hệ thống: không bao giờ tắt policy giữ thăng bằng khi robot
> đang tự đứng một mình.**

### 3.4 Trình tự chuẩn

Treo/đỡ robot → bật nguồn → **L2+Lên** (đứng dậy) → đặt xuống đất →
**R2+A** (đi bộ) → **R1+D-pad** (nhảy, tự về đi bộ khi xong) → **L2+Trái** (ngồi) khi xong.

### 3.5 Ngồi ghế — làm đúng cách

> 🪑 **CHIỀU CAO GHẾ: 43 cm** (mặt ngồi cách sàn). Đúng bằng tầm **gối robot khi đứng**
> (43.8 cm). Ghế **cao hơn** → robot ngồi nông → **dễ ngã ngửa**. Ghế thấp hơn vài cm
> vẫn được (robot rơi nốt đoạn còn lại). Nếu dùng ghế cao khác, phải sửa
> `sit_hip_deg` / `sit_knee_deg` — xem bảng trong tuning.yaml.

Bấm **L2+X**, robot ngồi **4 pha**:

| Pha                  | Robot làm gì                                                                       | Thời gian                   |
| -------------------- | ------------------------------------------------------------------------------------ | ---------------------------- |
| 1. Chỉnh chân đế | đứng thẳng,**mở rộng 2 chân** cho vững                                  | `sit_gather_time_s` (1.5s) |
| 2. Hạ người       | gập hông+gối,**đổ thân về trước** + **vươn 2 tay ra trước** | `sit_descent_time_s` (4s)  |
| 3. Ngồi hẳn        | mông chạm ghế →**thu tay về**, dựng thân lại                           | `sit_settle_time_s` (1.5s) |
| 4. Giữ              | giữ tư thế ngồi                                                                  | vô hạn                     |

> 🪑 **Đổ thân + vươn tay ở pha 2 là BẮT BUỘC, không phải trang trí.** Khi gập gối,
> hông lùi ra sau; nếu thân vẫn thẳng đứng thì **trọng tâm rơi ra ngoài gót → robot
> ngã ngửa**. Đổ thân về trước và vươn tay kéo trọng tâm về lại giữa bàn chân.
> Trông giống hệt cách người ngồi xuống ghế.

**Bàn chân bám mặt đất, không bám hông.** Góc cổ chân được tính từ **IMU** (góc nghiêng
thật của thân so với mặt đất), nên lòng bàn chân **không bị vênh** khi thân đổ. Chỉnh
độ bám bằng `sit_ankle_gravity_gain` (0 = tắt, 0.4 = mặc định, 1 = ép phẳng tuyệt đối).

Tham số trong tuning.yaml: `sit_hip_deg`, `sit_knee_deg` (độ sâu) · `sit_lean_deg`,
`sit_arm_forward` (cân bằng khi hạ) · `sit_seated_lean_deg` (dáng ngồi cuối) ·
`sit_spread` (rộng chân đế).

### 3.6 Đứng dậy lại sau khi ngồi / sau safe-stop

- Sau **ngồi ghế**: robot giữ tư thế ngồi → bấm **L2+Lên** để đứng dậy lại, hoặc
  **L2+B** để xả lực.
- Sau **safe-stop** (mất rồi có lại tín hiệu): robot đang đứng yên → cầm R3-1 điều
  khiển tiếp là được.

### 3.7 Đứng lên / Nằm xuống (L2+X) — quỹ đạo ghi từ built-in

Khác với ngồi ghế (script tay), đứng lên/nằm xuống **phát lại nguyên văn** quỹ đạo
đã ghi lúc bạn tự điều khiển robot bằng built-in — **PD thuần, KHÔNG có policy cân
bằng**. Vì vậy **lần chạy thật đầu tiên luôn phải có người đứng đỡ sẵn**, giống quy
trình ngồi ghế/khóa đứng.

**Quy trình đầy đủ:**

```
LOCOMOTION ──[L2+X: nằm]──► (dừng ~1s dưới policy) ──► phát liedown.npz
           ──► DAMPING (rũ mềm, nằm nghỉ trên sàn)

DAMPING(nằm) ──[L2+Lên: chuẩn bị]──► gồng cứng về tư thế nằm-chuẩn, GIỮ (chờ)
             ──[L2+X: đứng dậy]──► phát getup.npz ──► tự sang LOCOMOTION (policy cân bằng)
```

- **Nằm xuống** (`L2+X` khi đang **STAND LOCK** hoặc **LOCOMOTION/MIMIC**): robot dừng
  lại (nếu đang đi/nhảy thì settle ~1s dưới policy trước), phát `liedown.npz`, xong
  **tự rũ mềm (Damping) nằm nghỉ trên sàn**.
- **Đứng dậy = 2 bước** (chủ ý, để robot vào trạng thái xác định trước khi nâng người):
  1. **L2+Lên** — robot gồng cứng đưa khớp về đúng **tư thế nằm-chuẩn** (frame đầu clip)
     rồi **GIỮ**, chờ lệnh.
  2. **L2+X** — phát `getup.npz` nâng robot đứng dậy, xong **tự chuyển sang LOCOMOTION**
     để policy giữ thăng bằng ngay (không dừng ở STAND LOCK).
- Bấm **L2+X** thẳng khi đang nằm (chưa bấm L2+Lên) → robot nhắc "bấm L2+Lên trước".
- Nếu **chưa ghi file** (`motions/getup.npz` / `motions/liedown.npz` chưa tồn tại):
  log báo lỗi, robot **không làm gì** — an toàn mặc định.

**Cách ghi quỹ đạo** (làm 1 lần, trên robot thật):

```bash
cd ~/HB/high_level_2/build
./record_motion eth0 ../motions/raw     # đổi eth0 thành network_interface của bạn
```

Tool này **chỉ nghe DDS, không gửi gì** — an toàn chạy song song built-in. Sau khi
chạy, dùng tay cầm/built-in cho robot **nằm xuống rồi đứng lên** (vài lần nếu muốn
chọn bản đẹp nhất). Tool tự nhận ra từng đoạn chuyển động và in ra ngay:

```
[record_motion] >> Đoạn #1: 4.30s, 215 frame, phím giữ lúc bắt đầu='L2+down'
                  -> ../motions/raw/capture_001_L2_down.npz
```

Đối chiếu nhãn phím + thời điểm với động tác bạn vừa làm để biết đoạn nào là nằm
xuống, đoạn nào là đứng lên, rồi tự copy/đổi tên:

```bash
cp ../motions/raw/capture_001_xxx.npz ../motions/getup.npz
cp ../motions/raw/capture_002_xxx.npz ../motions/liedown.npz
```

Không cần build lại `run_r1` chỉ vì đổi file `.npz` (chỉ cấu hình, nạp lúc khởi
động) — khởi động lại `run_r1` là robot nhận file mới. Tham số PD/tốc độ phát lại
nằm ở `getup_*` / `liedown_*` trong tuning.yaml (xem comment tại chỗ).

### 3.8 Xem log / dừng

```bash
journalctl -u hb_high_level -f          # xem log realtime
sudo systemctl stop hb_high_level       # dừng
sudo systemctl restart hb_high_level    # nạp lại sau khi sửa tuning.yaml
```

---

## 4. CÁCH B — Chạy tay có bàn phím (dev/test)

Chỉ dùng khi ngồi máy có màn hình (dev PC / NoMachine / `ssh -Y`).

```bash
# 1. TẮT bản auto trước (bắt buộc — tránh 2 bản ra lệnh động cơ)
sudo systemctl stop hb_high_level

# 2. Đổi dev_no_keyboard: false trong config/tuning.yaml (nếu đang true)

# 3. Chạy tay (đang trong phiên ssh -Y, có $DISPLAY)
cd ~/HB/high_level_2/build && ./run_r1
```

Bàn phím **chỉ mở khi CẢ HAI**: `dev_no_keyboard: false` **và** có `$DISPLAY`.
Khi mở, một cửa sổ "R1 ROBOT CONTROL" hiện ra — **phải click vào cửa sổ** mới
nhận phím.

Bảng phím đầy đủ: xem **cột "Bàn phím" ở §3.2**.

Xong test, quay lại Auto:

```bash
# thoát bản chạy tay (ESC), rồi:
sudo systemctl start hb_high_level
```

Kiểm tra không chạy trùng: `pgrep -a run_r1` — chỉ nên thấy **1** dòng.

---

## 5. Chức năng AN TOÀN (tự động, không cần thao tác)

| Sự cố                                                                                          | Robot làm gì                                                                                | Vì sao                                                                                                                                                                    |
| ------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Ngã** — khi **đi bộ**, **nhảy**, hoặc **vào tư thế mở màn** | **Damping** (xả lực)                                                                  | mềm ra hấp thụ va đập; ngưỡng`fall_*`. *"Vào tư thế mở màn" là quãng robot đứng giữa sàn không ai đỡ mà policy đã tắt → phải có bảo vệ.* Hai đường: nghiêng thuần `> fall_tilt_deg`, hoặc "lật nhanh" `> fall_flip_tilt_deg` **kèm** gyro `> fall_flip_gyro` (hạ về 4.0 để bắt cú lurch lúc bàn giao). |
| **Vung khớp loạn** — một khớp có `\|dq\|` vượt ngưỡng (robot đá lung tung) | **Damping** | lớp an toàn **ngoài độ nghiêng**: `max\|dq\|` trên 24 khớp `> joint_speed_limit` liên tục quá `joint_speed_debounce_ms` → xả lực. Tắt bằng `joint_speed_guard_enabled: false`. |
| **Ngã** khi **khóa đứng** / **ngồi ghế**                                 | ❗**KHÔNG** xả lực (giữ cứng)                                                      | Cố ý: quy trình luôn có**người đứng đỡ** ở 2 trạng thái này                                                                                           |
| **Mất DDS lowstate** > 1s                                                                 | **Damping**                                                                             | mất cảm biến → policy "mù" → không giữ thăng bằng được                                                                                                        |
| **Mất TOÀN BỘ input** (R3-1 + bàn phím), DDS còn | Gói 0 dưới 3s: ngừng nhận lệnh; quá 3s + debounce: ép vận tốc về 0 và cảnh báo một lần | còn cảm biến → policy tự cân bằng; KHÔNG damp. Reconnect phải trung tính 200ms |
| **Đứt SSH/wifi** (chạy Auto headless)                                                   | **Không ảnh hưởng** — vẫn chạy bằng R3-1                                        | Auto không mở cửa sổ + tách khỏi phiên SSH                                                                                                                          |
| **Đứt X11/SSH** (đang chạy Cách B có bàn phím)                                     | **Damping + thoát**                                                                    | thư viện X ép thoát khi mất kết nối → damp trước khi thoát                                                                                                      |
| **L2 + B / ESC**                                                                           | **Damping** (E-stop thủ công)                                                         | luôn thắng mọi thứ                                                                                                                                                     |

> Vì sao "mất DDS → damp" nhưng "mất tay cầm → đứng yên"? Vì **mất DDS = mất cảm
> biến** (policy không chạy được → damp), còn **mất tay cầm = chỉ mất lệnh** (cảm
> biến còn → policy vẫn tự đứng vững → giữ nguyên).

---

## 6. Xử lý sự cố

| Triệu chứng                           | Nguyên nhân                                         | Cách xử lý                                                                                        |
| --------------------------------------- | ----------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Robot giật/loạn, chập chờn xả lực | **2 bản run_r1** cùng chạy                   | `pgrep -a run_r1`; tắt bớt (`systemctl stop` + kill bản tay)                                  |
| "Chờ kết nối DDS…" mãi             | sai`network_interface`                              | Kiểm`ip a` trên robot, sửa tuning.yaml đúng card mạng, `systemctl restart`                 |
| Không nghe robot nói                  | `voice_enabled: false`, hoặc TTS tiếng Việt kém | bật`voice_enabled: true`; nếu TTS tệ → thay `voice_*` bằng đường dẫn file .mp3 thu sẵn |
| R3-1 không điều khiển được       | tay cầm chưa kết nối / hết pin                   | kiểm remote; log`journalctl -u hb_high_level -f`                                                  |
| Sửa tuning.yaml không thấy đổi     | bản Auto đã nạp config lúc bật                  | `sudo systemctl restart hb_high_level`                                                             |
| Muốn tắt hẳn auto-boot               | —                                                    | `sudo systemctl disable hb_high_level` (bật lại: `enable`)                                     |

---

## 7. Lệnh systemd hay dùng (cheat-sheet)

```bash
sudo systemctl start   hb_high_level     # chạy ngay
sudo systemctl stop    hb_high_level     # dừng
sudo systemctl restart hb_high_level     # nạp lại config
sudo systemctl status  hb_high_level     # trạng thái
sudo systemctl enable  hb_high_level     # bật tự khởi động lúc boot
sudo systemctl disable hb_high_level     # tắt tự khởi động
journalctl -u hb_high_level -f   
sudo journalctl -u hb_high_level -n 30 # xem log realtime
pgrep -a run_r1                          # đếm số bản đang chạy (nên = 1)
```
