# PLAN — Đứng lên sau khi NGỒI (stand-up-from-sit) cho R1 high_level_2

> Trạng thái: **BẢN THẢO ĐỂ DUYỆT — CHƯA CODE** (lưu 2026-07-13, làm sau).
> Phạm vi: thêm routine đứng lên an toàn từ tư thế ngồi ghế, đi qua ĐÚNG state
> machine + safety hiện có.
>
> ⚠️ **CẦN VIẾT LẠI TRƯỚC KHI CODE (2026-07-14).** Bản thảo này dựa trên tính năng
> ngồi **3 pha** cũ và công thức feet-flat `ankle = -(hip+knee)` — cả hai **đã bị bỏ**
> vì chính công thức đó khóa thân thẳng đứng và làm robot **ngã ngửa**.
> Ngồi hiện là **4 pha**, tư thế ngồi cuối có **thân đổ trước `sit_seated_lean_deg`**
> và bàn chân bám **mặt đất qua IMU**. Đứng lên phải xuất phát từ tư thế đó, và
> phải **đổ thân + vươn tay** khi rời ghế (đảo ngược pha hạ) — xem `PLAN_sit_balanced.md`.

---

## 0. Hiện trạng — đã có nhưng NGÂY THƠ

L2+Lên (phím 0) từ tư thế ngồi (`kSafeShutdown`) **đã** đưa về `kStandUp` trong
[HandleTransitions](../src/app/Application.cpp) → [RunStandUp](../src/app/Application.cpp)
nội suy **thẳng** từ pose ngồi → default trong `stand_up_time_s` (2.5s), gains
`stand_*`, rate `stand_rate_limit`.

**Vấn đề:**
- **Mở vòng, không quản trọng tâm (CoM).** Từ squat sâu (gối ~90°) duỗi thẳng lên
  mà không dời CoM ra trước.
- **Robot lưng nặng (pin + PC lưng) → CoM lệch SAU** → duỗi thẳng dễ **ngả ngửa**.
- Dùng **chung** routine với đứng-dậy-từ-idle (hình học khác hẳn ghế).
- **Không tận dụng tay** làm đối trọng.

---

## 1. Ràng buộc cơ khí R1 (quyết định cách làm)

- **Waist chỉ có roll + yaw, KHÔNG có pitch** → **không nghiêng thân ra trước được**
  như người đứng dậy khỏi ghế.
- ⇒ Cách thay thế để dời CoM ra trước: **vươn 2 tay ra trước** (shoulder_pitch,
  policy idx 14 & 19) làm đối trọng — đây là "cái nghiêng thân" của R1. Giữ
  **feet-flat** để bàn chân bám sàn (ankle_pitch = -(hip+knee) mỗi tick như sit).
- Ankle-lean (dorsiflex) để dời CoM sẽ phá feet-flat → chỉ để làm tùy chọn phụ,
  test cẩn thận; **tay là công cụ CoM-forward chính**.

---

## 2. Thiết kế: đứng lên 3 pha (đảo ngược sit + dời CoM)

| Pha | Tên | Làm gì | Thời gian |
|---|---|---|---|
| **A** | **CHỒM TỚI** (prep) | Còn ngồi: **vươn 2 tay ra trước** (shoulder_pitch) + nghiêng nhẹ hip ra trước để **kéo CoM về phía bàn chân**. Chưa duỗi gối. | `rise_prep_time_s` ~0.8s |
| **B** | **ĐỨNG LÊN** (rise) | Duỗi gối + hông đồng thời (đảo pha hạ của sit), **giữ feet-flat**, **giữ tay vươn trước** để không ngả sau. Rate chậm. | `rise_time_s` ~2.5s |
| **C** | **ỔN ĐỊNH** | Về default_q, **thu tay về** (chỉ SAU khi chân đã đứng vững để không giật CoM), mở lại stance nếu cần → vào **STAND_LOCK**. | ~0.5s |

---

## 3. Tham số mới (tuning.yaml, nhóm `rise_*`)

```yaml
rise_prep_time_s: 0.8      # thời gian chồm tới lấy đà
rise_time_s: 2.5           # thời gian duỗi lên
rise_arm_reach: 0.5        # vươn tay ra trước (rad, shoulder_pitch) — ĐỐI TRỌNG CoM
rise_hip_lean: 0.1         # nghiêng hip ra trước lúc lấy đà (rad)
rise_kp_leg: 200           # gains khi đứng lên (mặc định = stand)
rise_kd: 3
rise_rate_limit: 4         # rad/s (chậm hơn stand chút cho êm)
rise_abort_tilt_deg: 40    # đang lên mà nghiêng quá góc này -> DAMP (chống ngã)
```
> Mặc định gains = `stand_*` cho an toàn; chỉ tách tên để tinh chỉnh riêng.

---

## 4. Máy trạng thái

- Thêm state **`kStandingUp`** (đứng lên từ ghế), **tách** khỏi `kStandUp` (đứng dậy
  từ idle) vì cần phase dời-CoM riêng.
- **L2+Lên (phím 0)** từ `kSafeShutdown` → `kStandingUp` → (xong) → `kStandLock`.
- L2+Lên từ idle / policy vẫn → `kStandUp` cũ (KHÔNG đổi).
- *(Phương án tối giản thay thế: dùng 1 cờ `from_sit` cho RunStandUp thay vì state
  mới — ít code hơn nhưng lẫn logic. Đề xuất: state riêng cho rõ ràng.)*

---

## 5. An toàn (bắt buộc)

- **Abort khi nghiêng:** trong pha B kiểm nghiêng (tái dùng logic fall-detector);
  vượt `rise_abort_tilt_deg` → `EnterIdle` (damp). Ghế sau đỡ nên **ngả sau ít nguy**,
  ngả **trước** mới nguy → canh chừng chiều trước.
- **L2+B / ESC** cắt lực bất cứ lúc nào (đã có).
- **Voice** "đang đứng dậy" (thêm `voice_stand_up` hoặc tái dùng `voice_stand_lock`).
- **Test có người đỡ** lần đầu — mở vòng nên không "bắt" lại được nếu trượt.

---

## 6. Điểm cần test/tinh chỉnh trên robot

- **Lượng vươn tay** (`rise_arm_reach`) + **chiều** shoulder_pitch (VERIFY dấu trên
  robot) — lưng nặng nên có thể phải vươn tay nhiều mới đủ kéo CoM ra trước.
- Ghế cao/thấp khác → chỉnh `rise_time_s` + pose ngồi (`sit_knee_deg`).
- Nếu vẫn ngả sau: tăng `rise_arm_reach` / `rise_hip_lean`, hoặc chậm `rise_rate_limit`.

---

## 7. Files sẽ chạm (khi code)

- `src/app/Application.hpp` — state `kStandingUp` + timer/phase + gains `rise_*`.
- `src/app/Application.cpp` — `RunStandingUp()` (3 pha) + nhánh transition từ
  `kSafeShutdown` + abort nghiêng.
- `src/config/Tuning.hpp` / `Tuning.cpp` — nhóm `rise_*`.
- `config/tuning.yaml` — khối `rise_*`.
- `docs/huong_dan_van_hanh.md` — cập nhật mục đứng dậy sau ngồi (tự đồng bộ).

---

## 8. Câu cần chốt trước khi code

1. **State riêng `kStandingUp`** (đề xuất) hay cờ `from_sit` tối giản?
2. Giá trị mặc định (`rise_arm_reach: 0.5`, `rise_time_s: 2.5`...) — giữ hay đổi?
3. Voice: thêm mốc `voice_stand_up` riêng hay tái dùng `voice_stand_lock`?

> Duyệt xong tôi code theo pha A→C, test có người đỡ, mỗi bước báo lại.
