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

Khoá **giữ đúng tư thế robot đang treo sẵn**, chốt tại encoder ở frame bóp cò.
Nó không kéo khớp tới tư thế khác, nên PD chỉ phải dập dao động quanh một điểm
robot vốn đã đứng yên ở đó — không phải nâng chân lên. Vì thế `teleop_lock_kp`
khởi điểm 20 là hợp lý, và nếu trên giá chân vẫn đung đưa thì **chỉnh trong
`config/teleop_lock.yaml`, không cần build lại**; trần là 100, bằng `kKpTrain`
của hông/eo, tức bằng độ cứng policy đi bộ giữ chân. Rung hoặc kêu thì tăng
`teleop_lock_kd` trước khi hạ kp.

Không có cú giật lúc vào: khi teleop chưa active, `SyncToState` đã đồng bộ
`last_cmd_q_` theo encoder mỗi vòng, nên tại frame chốt thì gốc slew và tư thế
khoá trùng nhau.

Nếu dây treo giữ robot ở một tư thế **không** cân bằng trọng lực (ví dụ chân bị
đai kéo cong), khoá phải sinh mô-men chống lại trọng lượng thật chứ không chỉ
dập dao động, và 20 nhiều khả năng không đủ. Nhìn chân có sụt xuống sau khi bóp
cò không là biết ngay.

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

Build **trên robot**; workstation không có SDK. Trên robot SDK nằm ở
`/usr/local/lib/cmake/unitree_sdk2`, tức đường tìm mặc định của CMake, nên
`CMAKE_PREFIX_PATH` trong CMakeLists trỏ `/opt/unitree_robotics` không dùng tới
mà vẫn `find_package` được.

Đã build và link thành công trên robot ngày 2026-08-24 tại
`~/HB/high_level_lock`, và `./build/run_r1 --preflight` cho `high_level_lock: OK`.
Preflight chỉ nạp model, **không tạo DDS publisher** (kiểm trong `InitControllers`),
nên chạy được cạnh `hb_high_level.service` đang active mà không phạm D003.

Cảnh báo thiếu file dance/getup/liedown khi preflight là bình thường: pilot này
tắt gesture và không cần motion npz nào.

### Một khác biệt so với bản đang chạy

`tuning.yaml` của bb70a20 đặt `flat_model: policy_goc.onnx`, và file đó **không
có** trong cây nguồn — bản đang chạy trên robot có, repo thì không, nên preflight
chết ngay bước nạp model. `config/teleop_lock.yaml` vì thế trỏ sang
`policy_11_07.onnx`, đúng model mà profile pilot treo của repo đã chọn, cùng
contract `legacy_83`.

Model đi bộ được nạp nhưng pilot này không bao giờ dùng: nó chỉ chạy trong ZERO
TORQUE. Nếu có phiên nào rời khỏi ZERO TORQUE thì đây là điểm khác biệt phải
tính tới — mà pilot này thì không được phép rời.

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

- Đã build và preflight trên robot; **chưa chạy `Run()`**, tức chưa lần nào
  publish `rt/lowcmd`, và chưa lần nào khoá thật một khớp.
- `teleop_lock_kp/kd = 20/3` **chưa đo trên robot thật**. Chỉnh được bằng yaml
  nên đây là việc quan sát lúc chạy, không phải việc sửa code.
- Hợp nhất ngược về `hardware/high_level/` chưa làm, và cố ý chưa làm.
