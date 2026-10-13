# Giới hạn cổ chân R1 — MJCF nói một đằng, phần cứng đi một nẻo

**Ngày:** 2026-09-08 · **Kết luận:** tầm `ankle_roll` trong MJCF (`±0.2618 rad`) **không**
mô tả phần cứng. Đừng dùng nó để siết deploy.

## Vì sao vấn đề này được mở

Audit ban đầu báo `motions/getup.npz` có **697 giá trị vượt tầm MJCF**, trong đó
`ankle_roll` chiếm 508, và kết luận là "target vượt range phần cứng → ép motor tì
vào chặn cơ khí". Kết luận đó **sai tiền đề**.

## Bằng chứng

`getup.npz` là bản sao của `motions/captures_20260725/capture_016_L2_X.npz`, mà
`tools/record_motion.cpp:166` ghi `motor_state().q()` — **encoder thật**, không phải
lệnh. Quét toàn bộ 74.442 frame encoder trong `motions/captures*`:

| | trái | phải |
|---|---|---|
| max | +0.3686 | +0.3429 |
| min | −0.3527 | −0.3390 |
| trung bình | −0.042 | +0.060 |

Ba dấu hiệu, cùng chỉ về một phía:

1. **Không có plateau.** Đỉnh trong `getup.npz` đi `0.3598 → 0.3681 → 0.3678` rồi
   xuống — một đường cong trơn, tự quay đầu. Tì vào chặn cơ khí thì phải thấy một
   loạt frame dính cùng một giá trị.
2. **Phân bố thoải dần** (411 → 365 → 314 → 240 → 162 → 105 → 79 → 39 → 17 → 0 khi
   siết ngưỡng), đúng dạng một đỉnh trơn ghé qua vùng biên, không phải dạng dồn
   đống ở một bức tường.
3. **Hai chân lệch zero ngược dấu ~0.05 rad**, đúng dạng sai lệch calib từng chân —
   không thể có nếu cả hai bị một tầm phần mềm ±0.2618 đối xứng chặn.

## Đính chính: `safe_limits.py` không phải nguồn độc lập

Trước đây tôi trích bảng giới hạn trong GUI cũ (`±13°`) như một nguồn thứ hai xác
nhận MJCF. **Không phải.** Đối chiếu cả 8 khớp cho thấy nó là ~90% của chính bảng
MJCF (hip_pitch 90.5/89.0%, ankle_pitch 90.0/84.8%, waist_roll 90/90%, ankle_roll
86.7/86.7%). Nó kế thừa cùng một con số, không đo gì thêm.

## Vấn đề thật nằm ở NGỒI, không phải ở getup

`getup.npz` chỉ ghé 0.369 trong 0.16 s rồi rời đi. Còn tư thế **ngồi**:
`sit_rest_spread: 0.45` đẩy target cổ chân lên 0.45–0.62, luôn bị clamp, và pha 4
giữ **vô hạn** (`sit_release_after: false`). Trần cũ `0.44` nghĩa là robot ngồi bao
lâu thì giữ cổ chân ở 0.44 bấy nhiêu — xa hơn mọi quan sát 0.07 rad, với Kp=200.

## Đã làm

- Gom ba con số rời (sit `±0.44`, `ApplyAnkleFlat` `±0.44`, cặp pitch) về
  `spec::kAnklePitch*` / `spec::kAnkleRollLimit` trong `RobotSpec.hpp`, kèm nguồn.
- Hạ trần roll `0.44 → 0.37` = giá trị lớn nhất đã chứng minh trên phần cứng, làm
  tròn lên. Không frame nào của `getup.npz`/`liedown.npz` bị cắt (max 0.3681).
- Giữ nguyên trần pitch = MJCF: ở hướng đó MJCF **chặt hơn** quan sát nên sai về
  phía an toàn (cắt 0.012 rad của một đoạn chuyển tiếp).
- `tests/ankle_limits_test.cpp` khoá cả hai vế: trần không cắt quỹ đạo đã ghi, và
  trần không vượt vùng đã chứng minh.

## Còn nợ — cần robot

`0.37` là **giá trị lớn nhất đã chứng minh**, không phải giới hạn cơ khí đã đo.
Muốn có con số thật thì phải đo trên robot: cho cổ chân roll đi tới hai biên ở
Kp thấp, ghi `q` và dòng, tìm điểm `q` ngừng tăng trong khi dòng vẫn tăng. Có số đó
rồi thì sửa cả `spec::kAnkleRollLimit` **và** tầm trong MJCF (sim đang chặn ở
±0.2618, tức sim không tái hiện được vùng robot thật đi được).
