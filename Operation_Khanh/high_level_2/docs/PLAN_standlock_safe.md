# PLAN — Voice báo trước khi khóa cứng + huỷ điệu nhảy an toàn

> **Trạng thái: ĐÃ CODE (2026-07-15), build OK, CHƯA test trên robot thật.**
> `stand_lock_warn_s: 1.5`, `dance_abort_lock_block_s: 2.0`, `mimic_cooldown_s: 1.0`.
>
> **Đã bổ sung so với bản thảo — SOFT-STOP:** huỷ điệu KHÔNG giao thẳng tư thế nhảy cho
> policy đi bộ (nó chưa từng thấy tư thế đó → **đạp loạn để gượng**). Thay vào đó **giữ
> policy mimic chạy**, kéo reference của nó từ tư thế nhảy **về tư thế đứng** trong 1s →
> chính nó **tự đứng robot thẳng dậy** (vẫn giữ thăng bằng) → **rồi** mới bàn giao.
> Áp dụng cho cả **3 đường ra khỏi điệu**: bấm `0`, hết điệu, và bấm ngồi giữa lúc nhảy.
>
> Ghi chú: chờ cảnh báo khóa cứng phải nằm **trước khi rời `kLocomotion`** (lúc policy còn
> chạy), KHÔNG phải bên trong `kStandUp` như bản thảo — vào `kStandUp` là policy đã tắt,
> chờ ở đó thì robot vẫn đứng cứng đơ và vẫn ngã.
> Phạm vi: `HandleTransitions` / `BeginStandUp` / `RunStandUp` trong `Application.cpp`,
> `Tuning`, `tuning.yaml`, `docs/huong_dan_van_hanh.md`.

---

## 1. Hai vấn đề

### 1.1 Voice nói SAU khi đã gồng cứng
`voice_stand_lock` phát ở cuối `RunStandUp()` khi `progress >= 1.0` — tức **sau khi robot
đã gồng cứng xong**. Người đứng đỡ **không được báo trước**.

`BeginSit()` làm đúng: phát voice **ngay lúc bắt đầu**. Hai đường đang bất đối xứng.

### 1.2 Bấm `0` khi đang NHẢY → tắt policy giữa sàn → ngã
`0` (= `L2+Lên`) từ `kMimic` hiện đi thẳng tới gồng cứng:

```
kMimic → dừng dưới policy (settle ~1s)     <- policy CÒN chạy, OK
       → kStandUp    : policy TẮT, ép về default_q open-loop, Kp=200, 2.5s
       → kStandLock  : policy TẮT, gồng cứng VÔ HẠN
```

**2.5s ramp + gồng cứng vô hạn** không có gì giữ thăng bằng, và **bảo vệ ngã cũng TẮT**
ở 2 trạng thái này (cố ý, vì giả định "luôn có người đỡ").

Giả định đó **đúng khi đang đi bộ** (bạn chủ động khóa, có người bên cạnh), nhưng **SAI
khi đang nhảy** — robot đang biểu diễn **một mình giữa sàn**, bấm `0` là để **huỷ điệu gấp**,
không phải để gồng cứng.

---

## 2. Thiết kế: `0` từ MIMIC = HUỶ ĐIỆU, KHÔNG gồng cứng

Không thêm trạng thái mới. "Đứng yên mà vẫn giữ thăng bằng" **chính là `kLocomotion`
với vận tốc = 0** — đã có sẵn.

| Đang ở | Bấm `0` (`L2+Lên`) | Policy |
|---|---|---|
| **kMimic** (đang nhảy) | **huỷ điệu → về ĐI BỘ, đứng yên tại chỗ** | **CHẠY → không ngã** |
| **kLocomotion** (đang đi bộ) | khóa cứng — **như cũ** | tắt (có người đỡ) |
| **kIdle / kSafeShutdown** | đứng dậy → khóa cứng — **như cũ** | tắt (có người đỡ) |

Muốn gồng cứng khi đang nhảy → bấm `0` **hai lần**: lần 1 huỷ điệu về đi bộ, lần 2 khóa
cứng. Lần 2 là hành động **cố ý**, lúc bạn đã tới đứng cạnh robot.

### 2.1 Luồng mới

```
Đang NHẢY
  └─ bấm 0 ──> tắt nhạc, bỏ policy mimic, bật policy locomotion, vận tốc = 0
              └─> ĐI BỘ (đứng yên) [policy CHẠY -> tự giữ thăng bằng]
                    ├─ stick ──> đi tiếp
                    ├─ R1+D-pad ──> nhảy điệu khác
                    ├─ L2+X ──> ngồi
                    └─ bấm 0 lần nữa ──> voice cảnh báo ──> KHÓA CỨNG
```

**Không đổi phím.** Không thêm state. Chỉ đổi đích đến của `0` khi đang ở `kMimic`.

### 2.2 Voice báo TRƯỚC khi khóa cứng
- Chuyển `music_.Say(voice_stand_lock)` từ **cuối `RunStandUp()`** lên **đầu `BeginStandUp()`**
  (giống `BeginSit()`).
- Thêm `stand_lock_warn_s` (mặc định **1.5s**): phát voice xong **chờ** rồi mới bắt đầu ép cứng
  → người đỡ kịp phản ứng. Trong lúc chờ, đến từ `kLocomotion` thì **policy vẫn chạy** nên
  không ngã; đến từ `kIdle` thì robot đang nằm/treo, vô hại.
- Huỷ điệu nhảy: phát `voice_locomotion` (*"bật chế độ đi bộ"*) — đã có sẵn, không cần voice mới.

---

## 3. Cần sửa gì

| File | Sửa |
|---|---|
| `Application.cpp` · `HandleTransitions` | `want_stand_lock` khi `state_ == kMimic` → **KHÔNG** gọi `RequestFromPolicy(kToStandLock)` nữa, mà: `music_.Stop()`, `ActivatePolicy(locomotion_)`, `state_ = kLocomotion`, `input_.ZeroVelocity()`, phát `voice_locomotion`. Nhánh `kLocomotion` giữ nguyên `RequestFromPolicy(kToStandLock)`. |
| `Application.cpp` · `BeginStandUp` | phát `voice_stand_lock` **ở đây**; đặt `stand_warn_timer_ = 0` |
| `Application.cpp` · `RunStandUp` | chờ hết `stand_lock_warn_s` rồi mới tăng `stand_timer_` (giữ nguyên tư thế lúc chờ); **bỏ** `music_.Say(voice_stand_lock)` ở cuối |
| `Application.hpp` | thêm `float stand_warn_timer_ = 0.0f;` |
| `Tuning` + `tuning.yaml` | `stand_lock_warn_s: 1.5` |
| `huong_dan_van_hanh.md` | §3.2: ghi rõ `0` khi **đang nhảy** = huỷ điệu về đi bộ (không khóa cứng); bấm 2 lần mới khóa cứng. §3.3: voice khóa cứng nói **trước** 1.5s. |

---

## 4. Rủi ro / phản biện

- **Đổi thói quen**: người quen "đang nhảy bấm 0 là cứng" giờ sẽ thấy robot về đi bộ.
  Voice khác nhau (*"bật chế độ đi bộ"* vs *"đã khóa đứng"*) giúp nghe là biết.
- **Bấm 0 hai lần quá nhanh** khi đang nhảy → vẫn ra gồng cứng giữa sàn.
  Edge-detect (`!last.up`) đã bắt buộc **nhả phím** giữa 2 lần, nhưng vẫn có thể bấm nhanh.
  → Nếu muốn chắc: chặn `want_stand_lock` trong ~1s đầu sau khi vừa huỷ điệu.
  **Cần bạn quyết**: có thêm chặn này không?
- Trong lúc chờ `stand_lock_warn_s` từ `kLocomotion`, robot **vẫn đứng dưới policy** —
  nếu bạn đã cầm/treo robot lên rồi mới bấm `0` thì policy sẽ thấy chân lơ lửng.
  Hiện quy trình là **bấm `0` trước, cầm sau** → không sao. Giữ đúng thứ tự đó.
- **Chưa test trên robot** — như mọi thứ khác trong repo này.
