# high_level_lock — bản cô lập, khoá cứng khớp ngoài teleop

**Đây không phải nguồn sự thật.** Nguồn sự thật của high-level là
`hardware/high_level/` tại commit `bb70a20` (nhánh `develop`), và bản đang chạy
trên robot là `/home/unitree/HB/high_level_2`. Cây này là bản sao nguyên vẹn của
`bb70a20` cộng đúng một thay đổi hành vi, tồn tại để chứng minh teleop chạy được
trong điều kiện cô lập **trước** khi hợp nhất ngược. Không sửa gì ở đây với ý
định giữ lâu dài; mọi thứ chứng minh được sẽ được port về nguồn sự thật.

## Thay đổi duy nhất so với bb70a20

`SendUpperBodyZeroTorque` của bản gốc đặt `kp=kd=0` cho cả 35 motor rồi chỉ cấp
PD cho 10 khớp tay và, nếu `head_valid`, hai khớp đầu. Nghĩa là **chân và eo
limp** trong suốt phiên teleop.

Bản này giữ chúng tại chính encoder được chốt ở frame teleop chuyển sang active:

| | bb70a20 | bản này |
|---|---|---|
| chân IDL 0-11, **eo IDL 12-13** | `kp=kd=0`, limp | `q` = tư thế chốt, `kp/kd` = `teleop_lock_*` |
| tay IDL 15-19 / 22-26 | teleop PD | không đổi |
| đầu IDL 29 (pitch) / 30 (yaw) | teleop PD khi `head_valid` | không đổi |
| slot IDL còn lại | thụ động | thụ động — không nằm trong `kSdkToIdl`, khoá một motor không biết là gì thì tệ hơn |

File đã sửa: `src/robot/LowCmdSender.hpp`, `src/config/Tuning.{hpp,cpp}`,
`src/app/Application.cpp`, `config/tuning.yaml` (thêm `include`),
`config/teleop_lock.yaml` (mới).

Tư thế khoá được chốt **cùng thời điểm** sidecar chốt `source_zero`/`start_q`,
và được nhả ngay khi teleop hết active — giữ chân bị khoá sau khi nhả cò nghĩa
là robot còn được cấp dòng trong lúc không ai điều khiển.

Khoá **giữ tư thế, không đỡ trọng lượng**. `teleop_lock_kp` bị chặn trên ở 60
(kKpTrain của chân là 100) vì một số cao hơn ở đây gần như chắc chắn là gõ nhầm.

## Eo bị khoá — chốt ngày 2026-08-24

Eo (roll IDL 12, yaw IDL 13) nằm trong danh sách khoá. Teleop hiện chỉ lái tay
và đầu, nên eo tự do chỉ là một thân trên đung đưa lẫn thẳng vào phép đo đang
cần làm; khoá nó lại làm phép đo tay/đầu sạch hơn chứ không phải khắt khe hơn.

Đây là điều kiện của thứ tự làm việc: **tay và đầu phải chứng minh được trước**,
rồi mới tính tới chuyện cho teleop lái eo. Ngày nào tới bước đó, phải bỏ IDL 13
khỏi `lockset::kNonTeleopIdl` **trước** — khoá và target cùng ghi một khớp là
hai bên đánh nhau.

Ràng buộc đó không để lại cho comment giữ. `lockset::DisjointFromTeleop()` duyệt
danh sách khoá đối chiếu với đầu (29/30) và mười khớp tay lấy qua
`spec::MotorIdl`, và `static_assert` chặn ngay lúc biên dịch. Đã thử ngược: nhét
IDL 30 vào danh sách thì build hỏng với đúng thông điệp đó. Cái sai kiểu này
không ném exception ở đâu cả — nó chỉ hiện ra thành một khớp cứng đờ hoặc rung
trên robot thật, và lúc đó thì đã muộn.

## Vì sao phải cô lập

`hb_high_level` là publisher `rt/lowcmd` duy nhất ([D003](../../decisions/r1_teleop/D003_single_lowcmd_owner.md)).
Hai bản chạy song song là đúng thứ D003 cấm. Nên bản này **thay thế tạm thời**
bản đang chạy chứ không chạy cùng: dừng service, chạy foreground, xong thì bật
service lại. Không cài service cho cây này.

## Build

```bash
./scripts/build.sh          # cmake + make, ra build/run_r1
```

Cần Unitree SDK2 tại `/opt/unitree_robotics` — có trên robot, không có trên
workstation này. Đã kiểm được ở workstation: cả ba file C++ đã sửa
`-fsyntax-only` sạch với header SDK thật. Link và chạy thì chưa.

## Chạy thử (robot treo, có người giữ E-stop)

```bash
# robot
sudo systemctl stop hb_high_level          # nhường quyền rt/lowcmd
cd ~/HB/high_level_lock && ./build/run_r1  # foreground, đọc config/tuning.yaml
```

Vào Dev Mode và ZERO TORQUE như thường lệ: `L2+R2`, giữ `R1+R2` 3 giây,
`L2+Y`. Rồi chạy teleop từ workstation như trong
[r1_quest3_teleop_hardware.md](../../docs/operations/r1_quest3_teleop_hardware.md).

Xong:

```bash
sudo systemctl start hb_high_level
```

## Chưa làm

- Chưa build, chưa link, chưa chạy trên robot.
- `teleop_lock_kp/kd = 20/3` là số chọn theo `kKdTrain` của hông/eo, **chưa đo
  và chưa được duyệt**. Phải xem chân có rung ở giá trị này không trước khi tin.
- Hợp nhất ngược về `hardware/high_level/` chưa làm, và cố ý chưa làm.
