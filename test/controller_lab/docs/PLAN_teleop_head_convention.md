# PLAN — Đồng bộ quy ước đầu (yaw/pitch) của `teleop/` theo chuẩn `controller`

> Quyết định 2026-08-18: **`controller` là chuẩn.**
> **TRẠNG THÁI 2026-08-18: ĐÃ TRIỂN KHAI.** 3 file trong `teleop/` đã sửa; kiểm chứng
> byte-level với `teleop_send_test.py` khớp tuyệt đối; `pytest teleop/tests` 20 passed.
> Còn lại: mục 5.3 — xác nhận trên robot đầu **quay** chứ không **gật**.
>
> Phạm vi đã chốt: **CHỈ** đồng bộ quy ước đầu. KHÔNG đụng `hb_teleop.service`,
> KHÔNG đụng `deploy_stack.sh`, KHÔNG thêm đường phát UTL1 runtime.
> (B1 sender runtime / B3 service+deploy: để lượt sau, mục 7.)

---

## 1. Chuẩn — không bàn lại

| Hạng mục | Giá trị chuẩn | Nguồn |
|---|---|---|
| Motor IDL đầu | **29 = head_yaw**, **30 = head_pitch** | `src/config/RobotSpec.hpp:45-46`, dùng nhất quán ở `RobotSpec.hpp:36` |
| Đường ghi | `SetHead(cmd, kHeadYawIdl, hy, …)` / `kHeadPitchIdl, hp` | `src/robot/LowCmdSender.hpp:83-84` |
| Đường đọc | `head[0] = motor_state[kHeadYawIdl]`, `head[1] = …kHeadPitchIdl` | `tools/record_motion.cpp:174-175` |
| Payload UTL1 | 2 float cuối = **(head_yaw, head_pitch)** | `src/input/TeleopReceiver.hpp` (struct `Packet`) |
| Clamp | `teleop_head_yaw_max: 1.0`, `teleop_head_pitch_max: 0.9` | `config/gestures.yaml:90-91` |

Bộ phát tham chiếu đã đúng chuẩn: `controller/scripts/teleop_send_test.py`.

---

## 2. Hiện trạng phía `teleop/` — chỗ nào lệch

| File | Quy ước hiện tại | Theo chuẩn? |
|---|---|---|
| `src/teleop/hardware/high_level_sidecar.py:22-28` | `JOINT_NAMES` = `… head_pitch_joint, head_yaw_joint` ứng với `MOTOR_INDICES … 29, 30` → **29=pitch, 30=yaw** | ❌ ngược |
| `src/teleop/hardware/high_level_sidecar.py:84-90` | `encode_target` nhận `[pitch, yaw]` rồi **hoán vị** khi pack | ❌ (hệ quả của trên) |
| `src/teleop/hardware/run_teleop.py:25` | `HEAD_INDICES = (29, 30)  # pitch, yaw` | ❌ ngược |
| `src/teleop/hardware/run_teleop.py:32-38` | `STREAM_JOINT_NAMES` = `… head_pitch_joint, head_yaw_joint` | ❌ ngược |
| `src/teleop/hardware/run_teleop.py:55` | `pilot_offsets` lắc slot **30** và gọi đó là "head yaw" | ⚠ xem 4.3 |
| `tests/test_high_level_sidecar.py:33,38` | Khẳng định thứ tự `[pitch, yaw]` + hoán vị khi pack | ❌ (theo sidecar) |
| `src/teleop/r1/live_arm_head.py:16-18` | `… head_yaw_joint, head_pitch_joint` | ✅ đã đúng chuẩn |
| `src/teleop/r1/mapping.py:44,63` | `… head_pitch_joint, head_yaw_joint` | ❌ nhưng **ngoài phạm vi** (mục 3) |
| `src/teleop/r1/whole_upper_body.py:221-222` | `head_slice` = `[pitch, yaw]` | ❌ nhưng **ngoài phạm vi** (mục 3) |

Ghi chú: `r1/live_arm_head.py` và `r1/mapping.py` **đã mâu thuẫn nhau ngay trong
`r1/`** từ trước. Đây là nợ của workspace nguồn, không phải của HB.

---

## 3. Ranh giới quy đổi đặt ở `hardware/`, KHÔNG sửa `r1/`

`teleop/scripts/sync_from_workspace.sh`:

```bash
rsync -a --delete \
    --exclude '__pycache__/' --exclude '*.pyc' --exclude 'hardware/' \
    "$WORKSPACE/teleop/" "$TELEOP_DIR/src/teleop/"
```

- `src/teleop/r1/**` bị **`--delete` ghi đè** mỗi lần sync → sửa ở HB sẽ **mất**.
  Nguồn thật nằm ở máy khác (`src/SOURCE.txt`: `/home/ubuntu22/Projects/Happy-Baby-R1`,
  commit `7e223e4`, worktree **dirty**) — không sửa được từ đây.
- `src/teleop/hardware/**` được **`--exclude`** → là mã sở hữu bởi HB, sửa an toàn.
- `tests/` nằm ngoài `src/` → không bị sync đụng tới.

**Quy tắc rút ra (ghi vào code dưới dạng comment):** `r1/` giữ nguyên quy ước nội
bộ của nó; **mọi quy đổi sang chuẩn `controller` nằm trong `hardware/`** — đúng
chỗ biên giới, vì `hardware/` cũng chính là nơi đóng gói UTL1.

---

## 4. Thay đổi cụ thể

### 4.1 `teleop/src/teleop/hardware/high_level_sidecar.py`

`JOINT_NAMES` — đổi 2 tên cuối, `MOTOR_INDICES` **giữ nguyên**:

```python
    "right_wrist_roll_joint", "head_yaw_joint", "head_pitch_joint",   # 29, 30 theo RobotSpec.hpp
)
MOTOR_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30)
```

`encode_target` — bỏ hoán vị (nguồn đã là thứ tự UTL1):

```python
def encode_target(sequence: int, positions: list[float]) -> bytes:
    """Encode arms[10] + head (yaw, pitch) — trùng thứ tự UTL1 của controller."""
    if len(positions) != 12:
        raise ValueError("expected 12 arms/head positions")
    arms = positions[:10]
    head_yaw, head_pitch = positions[10:]
    return PACKET.pack(TELEOP_MAGIC, sequence, 1, 1, 1, 0, *arms, head_yaw, head_pitch)
```

Thêm 1 comment đầu file nêu quy tắc mục 3 (chuẩn = `controller/src/config/RobotSpec.hpp`).

Không đụng: `PACKET`, `TELEOP_MAGIC`, `parse_target`, envelope an toàn, watchdog.

### 4.2 `teleop/src/teleop/hardware/run_teleop.py`

- `:25` → `HEAD_INDICES = (29, 30)  # yaw, pitch (RobotSpec.hpp:45-46)`
- `:32-38` `STREAM_JOINT_NAMES` → 2 tên cuối đổi thành `head_yaw_joint, head_pitch_joint`

### 4.3 `pilot_offsets` (`run_teleop.py:52-57`) — quyết định con

Hàm đang trả `{19: +a, 26: -a, 30: +a}` với docstring "symmetric wrists and head yaw".
Theo chuẩn HL2, slot 30 là **pitch**, nên hoặc:

- **(a) Khuyến nghị — giữ slot 30, sửa docstring/comment thành "head pitch".**
  Pilot này đã có bằng chứng phần cứng ở biên độ 0.03 rad; đổi slot = tự làm
  hỏng cơ sở so sánh của các lần đo trước, mà mục tiêu lượt này chỉ là quy ước.
- (b) Đổi sang slot 29 nếu thật sự muốn pilot quay đầu → phải chạy lại pilot và
  ghi lại bằng chứng.

Plan này đi theo **(a)**.

### 4.4 `teleop/tests/test_high_level_sidecar.py`

```python
    assert positions[-2:] == [1.0, 1.1]   # head_yaw, head_pitch
    ...
    assert encoded[16:] == pytest.approx((positions[10], positions[11]))  # UDP: yaw, pitch
```

### 4.5 KHÔNG đổi (cố ý)

- `src/teleop/r1/**` — mục 3.
- `src/teleop/r1/upper_body_kinematics.py:295-301, 369-372`: `waist_yaw_link →
  head_pitch → head_yaw` là **thứ tự chuỗi động học lấy từ URDF**, không phải nhãn
  slot. Comment tại chỗ đã ghi rõ hai thứ tự này cho ra phép quay khác nhau khi cả
  hai góc ≠ 0. Đảo = sai FK.
- `tests/test_static_safety.py:135` `HEAD_INDICES == (29, 30)` — slot không đổi.
- Toàn bộ `controller/` — đang là chuẩn, không sửa gì.

---

## 5. Kiểm chứng

1. **Tĩnh (máy dev):** `teleop/scripts/preflight.sh` — parse toàn bộ `src/teleop`
   + chạy `pytest tests/`. Phải PASS.
2. **So byte với bộ phát tham chiếu (máy dev, không cần robot):**
   dựng 12 giá trị mốc, gọi `high_level_sidecar.encode_target` và
   `teleop_send_test.pack` với cùng `(arm, yaw, pitch)` → 2 gói 60 byte phải
   **giống hệt** ở 8 byte cuối. Đây là kiểm định chuẩn thật sự, không phải test
   tự khẳng định.
3. **Trên robot (cần người vận hành, ngoài lượt sửa code):**
   `teleop_enabled: true` → arm bằng **giữ START** → `teleop_send_test.py --hold-arms --head-yaw 0.3`
   (một trục một lần, tay giữ yên để không nhiễu phép thử).
   - Đầu **quay** ⇒ chuẩn HL2 đúng, đóng việc.
   - Đầu **gật** ⇒ nhãn trong `RobotSpec.hpp:45-46` sai; sửa **HL2**, không revert python.

---

## 6. Rủi ro

- **Nhãn của HL2 chưa từng được phần cứng chứng minh.** `record_motion` đọc slot 29
  vào `head[0]` rồi replay ghi `head[0]` xuống slot 29 — đối xứng, nên nhãn có sai
  thì gesture/dance vẫn phát đúng. Chỉ teleop (ngữ nghĩa đến từ ngoài) mới lộ.
  → Hệ quả tốt: **nếu sau này phải hoán vị `kHeadYawIdl`/`kHeadPitchIdl`, mọi `.npz`
  đã ghi vẫn phát đúng**, vì record và play dùng chung hằng số. Không có nợ dữ liệu.
- Thay đổi `JOINT_NAMES` làm đổi hợp đồng stdin của sidecar. Hiện **chưa có process
  nào phát stream đó** (đã grep toàn `teleop/src`), nên không phá ai.
- `parse_target` so tuple tên **chính xác** ⇒ producer viết sau này mà dùng thứ tự
  cũ sẽ bị từ chối im lặng (fail-closed, đúng hướng) — ghi rõ trong comment.

## 7. Ngoài phạm vi lượt này

- **B1** — đường phát UTL1 runtime (bỏ clamp ±0.15 rad / duration 120 s của sidecar,
  giữ watchdog, không tạo DDS publisher). Đây mới là mảnh nối thật vào HL2.
- **B3** — `hb_teleop.service` hiện `ExecStart=teleop.hardware.run_teleop` (tự publish
  `rt/arm_sdk`/`rt/lowcmd`) + `Conflicts=hb_high_level.service`: **kiến trúc 2-writer**,
  ngược với tích hợp. Và `deploy_stack.sh` không sync `teleop/`.
- Nợ upstream: `r1/mapping.py`, `r1/whole_upper_body.py` còn `[pitch, yaw]` → phải sửa
  ở workspace `Happy-Baby-R1` nếu sau này nối thẳng IK vào đường UTL1.

## 8. Rollback

`git checkout -- teleop/src/teleop/hardware/ teleop/tests/`. Chỉ 3 file, không có
state trên robot, không đụng build C++.
