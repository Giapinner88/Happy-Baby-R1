# PLAN — Đường Quest 3 (viết lại 2026-08-19 sau khi có bundle `hb-teleop`)

> **Đính chính bản trước.** Bản plan ngày 18/08 viết khi HB chưa có bundle: nó nói phải tự
> dựng URDF, tự viết bộ phát, và khuyến nghị chạy web+IK trên PC2. **Sai cả ba.** Bundle
> chứa một chuỗi Quest **đã tồn tại và đã từng chạy** (smoke test robot treo trên giá).
> Việc còn lại KHÔNG phải viết mới mà là **nối chuỗi đó vào `controller`**.

> **Trạng thái triển khai 2026-08-19:** hai blocker code đã đóng; 12/12
> CTest local, 4/4 CTest Goal B trên ARM64 và 23/23 pytest teleop đều qua.
> `run_r1` mới đã active trên `192.168.1.104`, listener là
> `127.0.0.1:5560`, `high_armed=0`, `DISARMED`; sidecar đã copy nhưng không
> chạy. Chưa chạy Quest/sim hoặc bounded movement trên robot thật.

---

## 1. Chuỗi thật (đã có, sau khi gộp nằm ở `HB/teleop/`)

```
Quest 3 --WebXR/HTTPS:8012--> scripts/teleop/quest_bridge.py            [conda env: tv]
   | JSONL pose cổ tay + đầu, 30 Hz, deadman = cò phải
   v
scripts/teleop/run_r1_quest3_hardware_targets.py                        [conda env: unitree_sim_env]
   | IK coupled, body_mode="arms_head" (THÂN BỊ ĐÓNG BĂNG), 10 Hz
   | JSONL 12 khớp: tay[10] + head_yaw + head_pitch
   v  ssh
robot: teleop.hardware.high_level_sidecar
   | UDP 127.0.0.1:5560, PHÁT LẠI 100 Hz, ±0.15 rad quanh tư thế đầu phiên
   v
run_r1 (chủ rt/lowcmd duy nhất)
```

Ba điều thiết kế này giải thích các câu hỏi trước đó:
- **Sidecar phát lại 100 Hz** → nguồn 10 Hz vẫn không chạm watchdog 300 ms. Vậy **không cần**
  viết `utl1_sender` như bản plan cũ đề xuất; sidecar CHÍNH LÀ đường chạy.
- **Mất SSH = stdin đóng** → sidecar gửi `enable=0` rồi thoát. Fail-closed đúng, nên việc để
  bridge+IK trên workstation (thay vì PC2) là chấp nhận được — bỏ khuyến nghị PC2 của bản cũ.
- **`body_mode="arms_head"`** đóng băng thân (`run_r1_quest3_hardware_targets.py:113`), nên IK
  không tự giải `waist_yaw`. Đây đúng là điều tôi lo ở bản cũ, và nó đã được xử lý sẵn — không
  cần chuyển sang `live_arm_head`.

## 2. Blocker cũ đã tự tan

| Bản cũ nói | Thực tế |
|---|---|
| Không có URDF → phải convert từ `r1.xml` | `assets/R1.urdf` có trong bundle, đã đặt tại `teleop/src/assets/R1.urdf`. Đã nạp thử: `load_arm_chain("left"/"right")` OK |
| Phải đổi `torso_link` → `waist_yaw_link` | Không cần: URDF này **đã có** `waist_yaw_link` |
| `r1.xml` không có khớp đầu → đầu phải lấy trực tiếp | URDF **có** `head_pitch_joint`/`head_yaw_joint`; IK coupled dùng được |
| Không có process nào phát stream JSON | Có: `run_r1_quest3_hardware_targets.py` |
| televuer chưa có | Có, đã đặt tại `teleop/third_party/xr_teleoperate/teleop/televuer/` |

## 3. Hai blocker ban đầu — đã xử lý trong source hiện tại

### 3.1 Chuỗi Quest điều khiển ở ZERO TORQUE — `controller` không có đường đó
Bản C++ trong bundle có **`Application::RunZeroTorqueTeleop()`** (`src/app/Application.cpp:1672`),
được dispatch ở `case AppState::kZeroTorque` (`:772`): robot **treo trên giá, chân xả lực**,
lấy `q` encoder làm gốc, trộn tay teleop vào, rồi gửi bằng `SendUpperBodyZeroTorque(...)` với
gain riêng `teleop_arm_kp: 40`, `teleop_arm_kd: 2`, `teleop_max_rate_rad_s: 0.30`. Deadman là
cò Quest (`packet.enable`), và còn đòi R3 còn sống (`InputActive()`) để người giữ E-stop luôn
có quyền. `InitTeleop()` của nó đặt `teleop_runtime_on_ = true` ngay từ đầu.

Source hiện tại đã có `RunZeroTorqueTeleop()` và
`SendUpperBodyZeroTorque()`. Cờ START của Mode L không được dùng chung: Mode Z
được gate riêng bằng state `kZeroTorque`, remote R3 hợp lệ và deadman cò Quest.

### 3.2 Script kiểm tra cổng loopback sẽ chặn ngay từ đầu
`scripts/teleop/run_r1_quest3_hardware.sh` gác bằng:
```bash
ss -H -lun "sport = :5560" | grep -q "127.0.0.1:5560"
```
Receiver hiện bind `INADDR_LOOPBACK`, khóa ABI 60 byte và từ chối NaN/Inf,
tay vượt ±3.5 rad hoặc đầu vượt clamp. Runtime robot đã xác minh
`ss` in `127.0.0.1:5560`.

## 4. Hai CHẾ ĐỘ teleop — phải chọn, đừng bật cả hai

| | **Mode Z** (bundle) | **Mode L** (`controller` hiện tại) |
|---|---|---|
| Trạng thái robot | treo giá, ZERO TORQUE, chân limp | đứng trên sàn, LOCOMOTION, chân giữ thăng bằng |
| Cổng vào | state ZERO TORQUE + R3 hợp lệ; deadman = cò Quest | **giữ START 3 s** + cò Quest |
| Gốc tham chiếu | `q` encoder hiện tại | `default_q` của policy |
| Gain tay | `teleop_arm_kp/kd` riêng (40/2) | gain của policy |
| Bằng chứng | code + x86/ARM64 tests; **chưa bounded run cho implementation này** | **chưa có** |
| Rủi ro | thấp: robot không đứng | cao: tay lệch CoM khi đang thăng bằng |

**Khuyến nghị:** bring-up Quest bằng **Mode Z** trước (đã có SOP + bằng chứng), rồi mới thử
Mode L. Nghĩa là port `RunZeroTorqueTeleop` + `SendUpperBodyZeroTorque` + 3 núm gain sang
`controller`, giữ Mode L song song, và **chốt bằng trạng thái**: Mode Z chỉ chạy ở
`kZeroTorque`, Mode L chỉ ở `kLocomotion` — không có trạng thái nào cả hai cùng ghi.

## 5. Việc, theo thứ tự

| # | Việc | Trạng thái |
|---|---|---|
| 1 | Port Mode Z + sender/gain/rate | **Xong; x86 + ARM64 test qua** |
| 2 | Loopback + `Valid()` + twin sim | **Xong; runtime bind loopback** |
| 3 | Dựng env tối giản `tv`, `unitree_sim_env` | **Xong; import/smoke qua** |
| 4 | Cert HTTPS SAN = `192.168.1.105` | **Xong; hạn tới 2028-11-20** |
| 5 | Robot config + deploy sidecar/high-level | **Xong; sidecar không chạy** |
| 6 | Đeo Quest và chạy **sim** | **Chưa làm** |
| 7 | Bounded run khi robot treo, có người giữ E-stop | **Chưa làm; hardware gate** |

## 6. Đã sửa lúc gộp (nếu không sửa thì chuỗi im lặng không chạy)

`run_r1_quest3_hardware_targets.py`: tách `MODEL_JOINT_NAMES` (thứ tự mô hình IK, `[pitch, yaw]`
— **không được đổi**, đó là thứ tự chuỗi động học) khỏi `JOINT_NAMES` của **stream** (chuẩn
`RobotSpec.hpp`: `[yaw, pitch]`), rồi hoán vị khi emit bằng `STREAM_FROM_MODEL`. Không có bước
này thì `high_level_sidecar.parse_target` so tuple tên **chính xác** → từ chối mọi frame,
teleop không bao giờ bật và không in ra lý do. Thay đổi này **phân kỳ với upstream**, phải mang
về repo `Happy-Baby-R1` khi có dịp.

## 7. Giới hạn phần cứng (không đổi)

Tay deploy 5-DoF, cổ tay chỉ có `wrist_roll` → bám **vị trí + hướng một phần**, không xoay cổ
tay đủ 6-DoF, không có bàn tay để teleop ngón.

---

## Phụ lục A — Mã Mode Z ghi lại nguyên văn (bundle đã xoá 2026-08-19)

Bundle không còn trên máy này. Đây là những gì đã đọc được từ
`hardware/high_level/src/app/Application.cpp` trước khi xoá, ghi lại để port không cần
mở lại repo `Happy-Baby-R1`.

**Dispatch** (`:771-772`)
```cpp
        case AppState::kZeroTorque:
            RunZeroTorqueTeleop();
```

**`InitTeleop()` — khác controller ở 2 dòng cuối** (`:479-485`)
```cpp
    teleop_.Init(def_arm, /*enabled=*/true, tuning_.teleop_udp_port,
                 tuning_.teleop_timeout_ms, tuning_.teleop_blend_in_s,
                 tuning_.teleop_retract_s, tuning_.teleop_smooth_hz,
                 tuning_.teleop_head_yaw_max, tuning_.teleop_head_pitch_max);
    teleop_runtime_on_ = true;  // arm-head profile mặc định bật; packet.enable là deadman.
    std::cout << "[Teleop] Lang nghe loopback UDP cong " << tuning_.teleop_udp_port
              << " (tay+dau). Arm-head san sang; co phai la deadman.\n";
```

**`RunZeroTorqueTeleop()`** (`:1672-1698`, nguyên văn)
```cpp
void Application::RunZeroTorqueTeleop() {
    // R3 phải còn sống để người giữ E-stop luôn có quyền. Quest
    // trigger và watchdog UDP là lớp deadman thứ hai.
    const bool operator_ready = input_.gamepad().InputActive();
    teleop_.Update(spec::kLoopDt, teleop_runtime_on_ && operator_ready);
    if (!teleop_.Active()) {
        // Trong thời gian thụ động, đồng bộ slew origin theo encoder để
        // lần bóp cò tiếp theo không kéo về target cũ.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            sender_.SyncToState(shared_low_state_);
        }
        sender_.ZeroTorque(estimator_.state().mode_machine);
        return;
    }

    std::array<float, spec::kNumJoints> target = estimator_.state().q;
    teleop_.BlendInto(target);
    HeadTarget head;
    if (teleop_.HeadValid()) {
        head = {true, teleop_.HeadYaw(), teleop_.HeadPitch(),
                tuning_.teleop_max_rate_rad_s};
    }
    sender_.SendUpperBodyZeroTorque(target, tuning_.teleop_arm_kp,
                                    tuning_.teleop_arm_kd,
                                    tuning_.teleop_max_rate_rad_s,
                                    estimator_.state().mode_machine, head);
}
```

**Núm cấu hình** (`config/tuning.yaml:338-350`)
```yaml
# Arm-head teleop chạy trong ZERO TORQUE; Quest trigger là deadman.
teleop_arm_kp: 40.0
teleop_arm_kd: 2.0
teleop_max_rate_rad_s: 0.30
```

### Mảnh DUY NHẤT không ghi lại được
`LowCmdSender::SendUpperBodyZeroTorque(...)` — tôi chỉ đọc **chữ ký lời gọi**, chưa đọc thân
hàm. Phải viết lại từ ý định (suy ra được từ chỗ gọi + `LowCmdSender` hiện có của controller):

- Chỉ đặt `q/kp/kd` cho **10 khớp tay** (policy 14..23) và **đầu** (idl 29/30) theo `HeadTarget`.
- **Mọi khớp còn lại** (chân 0..11, waist 12..13): `kp=kd=0`, `tau=0` — tức vẫn zero-torque thật.
- Slew `q` theo `max_rate_rad_s * kLoopDt` (dùng `Slew` + `last_cmd_q_` đã có).
- `SyncToState(low)` đặt `last_cmd_q_` = encoder hiện tại để không giật khi vào/ra.
- CRC như `Send()` thường.

Nếu cần bản gốc thì lấy lại từ repo `Happy-Baby-R1` branch `develop` trên máy
`ubuntu22-X670E-Taichi`: `hardware/high_level/src/robot/LowCmdSender.hpp`.

---

## Phụ lục B — Vá "phiên chết vì một frame" (2026-08-19)

**Triệu chứng lần chạy 20260818T184130Z:** bridge phát 303 command, sidecar nhận **đúng 1**
target rồi `stop=input_watchdog`. Bridge log cho thấy cò **giữ suốt**, không hề nhả.

**Cơ chế:** `run_r1_quest3_hardware_targets.py` có ba đường `return 0` đóng stdout ngay lập
tức, và một đường im lặng:
- `mapper.map()` trả `enabled=False` khi frame **tre hơn `command_timeout_s` (0.5 s)** —
  không phân biệt với nhả cò. Chỉ cần `print()` bị chặn vì SSH nghẽn là frame kế thành stale
  → `[STOP] deadman released or command disabled` → **thoát cả phiên**.
- IK từ chối một nghiệm → không emit gì → im > 0.75 s → sidecar `input_watchdog` kết thúc.

**Vá:**
1. **Phân biệt nhả thật với trễ**: nhả chỉ tính theo `newest.deadman_enabled` của chính lệnh,
   và phải liên tục quá `--release-debounce-s` (0.2 s). Trễ/IK từ chối → **giữ**, không thoát.
2. **Giữ target cuối** (`solution_kind: "hold"`) phát tiếp ở nhịp điều khiển để sidecar không
   chạm watchdog; robot đứng yên giữ tư thế. Tối đa `--hold-max-s` (2.0 s), quá thì **ngừng
   phát có chủ ý** để watchdog nhả tay về policy — fail-safe vẫn còn nguyên.
3. **Tách hai bộ đếm sequence**: `previous_sequence` lọc frame Quest vào, `out_sequence` đánh
   số frame ra. Dùng chung một biến thì frame `hold` sẽ bơm bộ lọc lên và **vứt mất frame
   Quest thật** có seq nhỏ hơn.

**Nghiệm thu offline** (không cần robot, dùng `make_preflight_command_stream.py` + pacer ghi
lại `timestamp_monotonic_s`):

| Kịch bản | Kết quả |
|---|---|
| Bình thường | 60 emit, seq tăng dần, kết thúc bằng `[STOP] deadman released (0.20s)` |
| Gián đoạn 1.0 s giữa chừng | 71 emit (12 `hold`) — **phiên sống sót**, trước đây là chết |
| Gián đoạn 3.0 s (> hold_max) | giữ đúng 2.0 s rồi `[STOP] giữ quá 2.0s` để watchdog nhả |
| Sidecar nuốt lại output | 39/39 chấp nhận (19 `exact` + 20 `hold`), 0 từ chối |

**Chưa chứng minh:** nguyên nhân gốc lần chạy thật (stale do SSH nghẽn / IK từ chối / lý do
khác) — evidence local không đủ. Lần chạy tới `ik_diagnostics.jsonl` đã được script truyền
sẵn vào `RUN_DIR`, cộng thêm log `[HOLD]`/`[STOP]` có ghi lý do, sẽ chốt được.
