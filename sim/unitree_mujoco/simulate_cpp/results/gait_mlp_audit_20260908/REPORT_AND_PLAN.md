# Kiểm tra trực tiếp gait MLP và kế hoạch cải thiện

Ngày 2026-09-08. Phần đo ban đầu được thực hiện trước khi sửa training.
Code training đã được cập nhật sau đó; xem mục cập nhật triển khai cuối tài liệu.
Config HB và ONNX đang deploy chưa thay đổi.

## Artifact và điều kiện

- Policy: flat_plus_gait_h4_v1 / policy_flat_plus_gait_h4_v1.onnx, actor 335D.
- SHA-256 đã đối chiếu HB và bundle: 6a4d259313697287c4858e707d08a0d5cdc5f9a2af4ed05b5c3d4a055abd5301.
- Runtime: simulate_cpp/build/run_policy + simulate/build/unitree_mujoco, DDS loopback.
- Scene hợp lệ: scene_slope_0.xml, nền phẳng không vật cản.
- Mỗi lượt: warmup 5 s; đo 18 s, lệnh tiến trong 12 s rồi command zero 6 s. Một lượt mỗi tốc độ, không kiểm tra nhiều seed hoặc perturbation.
- flat04 có ghi video; flat02/flat06 không ghi. Render ghi 163 ảnh camera thực và encode 687 frame; không dùng video để đo va đập hoặc cadence chính xác.
- Video có overlay policy_goc.onnx hard-coded trong simulate/src/main.cc:856. Đó không phải tên policy nạp thực tế; log xác nhận FlatPlusGaitController 335D và đúng đường dẫn gait.

## Kết quả

Cả ba lượt nền phẳng RESULT=PASS theo bộ kiểm tra thân hiện hữu.
Tốc độ đo ở t=3..11 s sau warmup; tilt tương đối so với tư thế bắt đầu đo, lấy toàn lượt; biên độ z lấy đoạn đi đều. Stop displacement là dịch chuyển thuần trong t=15..18 s, không phải quãng đường hãm.

| Run / command 0.2, 0.4, 0.6 m/s | Vx thực m/s | Tilt tương đối max độ | Biên độ z đi đều mm | Dịch chuyển cuối stop mm |
|---|---:|---:|---:|---:|
| flat02 | 0.177 | 1.06 | 2.90 | 0.79 |
| flat04 | 0.357 | 1.90 | 2.76 | 1.27 |
| flat06 | 0.573 | 2.30 | 2.54 | 1.31 |

Hai lượt gait04 trên scene.xml mặc định ngã ở khu vực bậc thang. Scene có stair_1 tại x=1 m, mặt đầu x=0.85 m. Loại hai lượt này khỏi đánh giá đi nền phẳng; không quy lỗi ngã đó cho training WALK.

## Kết luận và giới hạn

- Đã tái hiện chạy đúng policy gait trong C++ simulator. Ở ba lệnh trong dải train, thử nghiệm ngắn chưa tái hiện mất vững thân trên nền phẳng.
- Tốc độ thực thấp hơn command, khoảng 4.4–11.3%. Đây là điểm cần đo lại sau cải thiện, không suy ra chiều dài bước từ tốc độ đơn thuần.
- Ảnh ngang thể hiện gối gập; chưa có số đo góc gối theo contact để kết luận mức gập quá mức.
- CSV autotest thân vẫn không có foot position, joint state hoặc contact force. Telemetry physics riêng đã bổ sung các trường này; `meta_trace` chỉ điền cho meta/ARMA và không dùng để đánh giá gait.
- Chưa so sánh policy mặc định vì chưa xác nhận artifact nào là baseline người dùng nói tới.
- PASS ở đây là ổn định thân trong thử nghiệm ngắn; chưa phải nghiệm thu chất lượng bước hay hardware.

## Kế hoạch sửa ngay task hiện tại, giữ MLP

1. Bổ sung đường đo opt-in trong simulator cho q/dq thực, foot positions, contact, lực normal theo physics substep và gait state. Tính góc gối giữa pha trụ, đỉnh swing, touchdown vertical velocity, step length/cadence, slip và impulse. Đây là đo kiểm, không thay controller. Chốt đúng baseline trước so sánh A/B.
2. Trong factory gait hiện tại, điều chỉnh posture riêng hip/knee/ankle để bớt kéo mọi pha về pose gập mặc định. Thêm shaping duỗi gối vừa phải ở giữa pha trụ; giảm/tắt tại touchdown và swing. Không thay default pose làm mốc action; không ép khóa gối.
3. Sửa reward swing để đánh giá đỉnh chiều cao theo từng bước, relative ground/sole reference; xử lý reset bộ nhớ theo env. Chọn khoảng clearance từ số đo và hình học R1, không mặc định mọi tốc độ phải nhấc 10 cm.
4. Thêm shaping hạ chân êm và kiểm tra lực/xung lực tiếp đất; route cho WALK và W2S. Chuẩn hóa lực theo trọng lượng, kiểm tra reward scale thực tế; không tăng weight tùy ý trước đo.
5. Giữ period 0.6 s trong lần thử đầu. Cải thiện tracking và động tác chân trước; nếu còn bước vụn, kiểm tra quan hệ cadence/step length ở cùng tốc độ thực rồi mới chọn shaping. Không thưởng dạng chân hoặc sải chân vô điều kiện.
6. Fine-tune MLP từ checkpoint gait có provenance, lưu run mới trong cùng task. Ramp reward mới; đặt lại tiến độ curriculum có chủ đích. C0 WALK phải qua đánh giá trước C1/C2, không để step counter resume tự đẩy ngay sang phase cuối.
7. Nghiệm thu nhiều seed và đi thẳng/quay/lùi/stop-restart, so gait cũ và gait mới. Yêu cầu không giảm tracking/stability, giảm touchdown impact, clearance đạt khoảng mục tiêu, stance knee tự nhiên hơn. Export parity và chạy closed-loop ONNX trước xét hardware.

Giữ task ID Unitree-R1-Flat-Plus-Gait, MLP 512/256/128, obs 335D/action 24D. Chỉ chỉnh reward/curriculum của gait; không tạo task mới. Chưa có thay đổi training nào được thực hiện trong audit này.

## Telemetry MuJoCo theo từng bước physics

Telemetry được bật riêng bằng `MUJOCO_GAIT_TELEMETRY=<csv>`, không thay đổi
policy. CSV ghi site position/velocity, contact, tổng normal force trên bảy
foot collision geoms mỗi bên, knee q/dq và touchdown event. Model R1 có tổng
khối lượng 28.93 kg, tương đương trọng lượng khoảng 284 N. Một dòng cuối có
thể bị ngắt khi `run_rough_stack.sh` gửi SIGTERM cho simulator sau khi policy
autotest kết thúc; các dòng không đủ cột được bỏ khi tính toán.

Các số dưới đây lấy trong đoạn có lệnh đi 5--17 s theo clock physics (gồm tăng tốc đầu đoạn; chưa đồng bộ tuyệt đối với clock policy). `knee stance`
là trung bình q của các mẫu đang contact; `touchdown |vz|` là vận tốc thẳng
đứng tuyệt đối tại raw first contact; cột `Median step` bên dưới thực chất là
STRIDE theo trục world X: khoảng cách touchdown liên tiếp của cùng một chân,
không phải bước trái-sang-phải. Đã bỏ các event dưới 5 cm; đó chỉ là lọc sơ bộ,
chưa thay thế phân đoạn contact có hysteresis. Không dùng các event này như
ground truth landing khi còn chattering.

| Command vx | Foot | Peak z | Knee stance | Knee range | Max normal force | Touchdown | Median step |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0.2 m/s | L / R | 1.13 / 1.28 cm | 0.280 / 0.280 rad | 0.219--0.504 | 379 / 360 N | 0.251 / 0.260 m/s | 0.106 / 0.106 m |
| 0.4 m/s | L / R | 1.89 / 2.62 cm | 0.287 / 0.292 rad | 0.191--0.652 | 329 / 320 N | 0.236 / 0.258 m/s | 0.215 / 0.213 m |
| 0.6 m/s | L / R | 3.16 / 4.31 cm | 0.316 / 0.305 rad | 0.161--0.793 | 435 / 335 N | 0.246 / 0.308 m/s | 0.343 / 0.335 m |

`Peak z` đang cách target clearance 10 cm rất xa: ở 0.2 m/s chỉ đạt khoảng
1.1--1.3 cm, ở 0.4 m/s đạt 1.9--2.6 cm. Vì vậy quan sát “chân nhấc thấp” đã
được xác nhận bằng physics, không chỉ là cảm nhận từ video. Lực max ở 0.2 và
0.6 m/s lần lượt khoảng 1.34 và 1.53 lần trọng lượng robot; cần dùng impulse
hoặc force tại touchdown sau khi lọc contact để đánh giá độ “đập” chính xác.

Các file đo: `flat02_physics.csv`, `flat04_physics.csv`, `flat06_physics.csv`.

Lệnh tái chạy, ví dụ:

```bash
MUJOCO_HEADLESS=1 \
MUJOCO_GAIT_TELEMETRY=/tmp/flat04_physics.csv \
./run_rough_stack.sh 0 \
  --locomotion-policy policy/locomotion/flat_plus/policy_flat_plus_gait_h4_v1.onnx \
  --autotest gait_measure gesture=0 vx=0.4 warmup=5 hold=18 stop_after=12
```

## Quyết định sửa task

Không sửa default pose và không đổi period ở vòng đầu. Clearance tăng theo tốc
độ đã được đo; reward `feet_clearance` nhân sai số với vận tốc ngang là một
giả thuyết cần ablation, chưa chứng minh là nguyên nhân duy nhất. Gối stance
gần 0.3 rad cũng chưa chứng minh posture là nguyên nhân của dáng gập.

Thứ tự thay đổi nên là:

1. Bật reward peak swing theo từng chân và từng bước, reset state theo env;
   giữ một target hợp lý theo hình học R1 và tăng trọng số từ thấp.
2. Thêm reward duỗi gối có điều kiện contact/stance giữa pha trụ; tắt hoặc
   giảm khi swing và gần touchdown để không khóa chân hay làm va đập mạnh.
3. Thêm penalty vận tốc đi xuống gần touchdown và theo dõi force/impulse;
   chỉ tăng weight sau khi có số đo.
4. Fine-tune cùng task MLP hiện tại, trước hết ở C0 WALK. Chỉ mở W2S/STAND sau
   khi peak clearance, touchdown và knee stance không xấu đi.
5. Chạy lại đúng ba command trên, thêm quay/lùi và stop--restart; so với ba
   CSV baseline ở trên trước khi export ONNX.

## So sánh policy cũ ở cùng 0.4 m/s

Đã đo thêm hai artifact 83D vì tên “policy mặc định” trong workspace có hai
ứng viên: `policy_8.onnx` là rollback trong `locomotion.yaml`, còn
`policy_goc_2.onnx` là rollback trong example Flat-Plus. Cả hai đều chạy cùng
scene `scene_slope_0.xml`, cùng thời lượng và cùng telemetry physics.

| Policy | Vx thực | Peak z L/R | Knee stance L/R | Max force L/R | Touchdown | Median step |
|---|---:|---:|---:|---:|---:|---:|
| gait 335D | 0.359 m/s | 1.89 / 2.62 cm | 0.287 / 0.292 rad | 329 / 320 N | 0.236 / 0.258 m/s | 0.215 / 0.213 m |
| `policy_8` 83D | 0.395 m/s | 4.61 / 4.32 cm | 0.381 / 0.412 rad | 406 / 511 N | 0.415 / 0.349 m/s | 0.244 / 0.233 m |
| `policy_goc_2` 83D | 0.330 m/s | 3.00 / 2.60 cm | 0.336 / 0.375 rad | 397 / 305 N | 0.277 / 0.353 m/s | 0.194 / 0.199 m |

Đính chính: hai policy cũ có q gối stance LỚN hơn, tức gập hơn theo quy ước
khớp R1, không phải duỗi hơn. Peak foot site cao hơn hoặc tương đương gait.
`policy_8` đạt stride theo world X dài hơn,
nhưng lượt đo này trôi ngang gần 2 m nên chưa thể gọi nó là baseline tốt cho
độ tự nhiên; `policy_goc_2` cũng cần giữ đúng route trước khi dùng làm chuẩn.
Gait hiện tại có raw touchdown velocity thấp hơn `policy_8`, nhưng clearance
thấp hơn nhiều. Hai policy có tốc độ thực khác nhau; chưa phải so sánh cùng
tốc độ thực hoặc bằng chứng policy cũ tiếp đất êm hơn.

## Rà soát bổ sung: reward, vung chéo và train/deploy

### Các đính chính phải dùng khi triển khai

- Không dùng đề xuất cũ q gối 0.38--0.42 rad để làm chân thẳng: FK trên MJCF
  xác nhận tăng q từ 0.3 lên 0.4 làm góc gập hip-knee-ankle tăng khoảng 20 lên
  25 độ. Mục tiêu phải là duỗi vừa phải theo hình học chân giữa pha trụ, không
  một góc cứng chung cho mọi pha.
- Max force trên toàn đoạn không đồng nghĩa peak impact tại touchdown. Site
  velocity không phải vận tốc mọi điểm toe/heel. Telemetry hiện còn raw contact,
  chưa có yaw thân để đo swing trong heading frame hay hướng bàn chân.
- Kiểm tra sơ bộ CSV 0.4: foot_y-base_y quét khoảng 2.1--2.2 cm trong swing;
  còn chưa bù yaw nên chưa xác nhận chân cắt qua đường giữa thân.
- Mức clearance 3.5--5 cm và touchdown 0.22 m/s trong trao đổi trước chỉ là
  candidate để sweep; chưa có căn cứ nghiệm thu để chốt thành yêu cầu cứng.

### Khôi phục đúng cấu hình artifact trước fine-tune

`GAIT/gait/env.yaml` và ONNX đang chạy khác source r1_constants.py hiện tại:

| Thành phần | Bundle đã train / ONNX | Source training hiện tại |
|---|---|---|
| Hip roll default L/R | +0.0349/-0.0349 (ONNX làm tròn 0.035) | 0/0 |
| Hip yaw default L/R | -0.0477/+0.0477 (ONNX làm tròn 0.048) | 0/0 |
| Ankle roll default L/R | -0.0349/+0.0349 | 0/0 |
| Knee damping | 2.5 | 2.0 |
| Ankle damping | 2.5 | 2.0 |
| Knee action scale | 0.1575 (ONNX 0.157) | 0.15 |
| Ankle action scale | 0.328125 (ONNX 0.328) | 0.3125 |

HB PolicyController và C++ FlatHistoryController đọc default/action scale/gains
từ ONNX; không lấy các mảng fallback làm giá trị thực. HB gain scale hiện 1.0;
runtime.yaml tắt gyro/dq LPF và locomotion pitch trim. Vì vậy chưa phát hiện
HB nạp nhầm damping 3.0 từ fallback cho gait.

Khi fine-tune: khôi phục các giá trị bundle bằng override riêng trong factory
gait, giữ cùng task ID và MLP. Không sửa common robot constants cho mọi task.
Sai khác default hip roll/yaw đáng chú ý với vung chéo, nhưng chưa được chứng
minh là nguyên nhân: deploy đang dùng chính default mà artifact đã học.

### Khác biệt runtime cần đo

- Train physics 0.005 s x decimation 4; C++ physics 0.002 s, policy 50 Hz qua
  DDS. Train dùng position actuator; C++ dùng torque motor tính PD qua bridge.
- Train foot condim=3; C++ XML condim=6. Cần đối chiếu compiled contact model,
  friction, effort limits và solver trước khi so lực/impulse.
- C++ command dùng EMA alpha=0.1; HB dùng accel/decel limiter và heading hold.
  Train command C2 có step/hold/ramp riêng. Cần so command SAU xử lý.
- HB sender slew-limit 30 rad/s ở 500 Hz (0.06 rad/tick), có activation blend;
  C++ gửi trực tiếp target_q. Thêm log target trước/sau sender và tỷ lệ chạm
  slew limit, tái hiện vào validation/training nếu có tác động đáng kể.
- Gait train envelope vx/vy +/-0.6, yaw +/-1.0; HB có lệnh vượt envelope này.
  Mở tốc độ phải đi cùng curriculum và nghiệm thu, không chỉ tăng giới hạn YAML.

### Reward đề xuất cho cùng task gait

1. `swing_clearance`: penalty thiếu clearance theo expected mid-swing và peak
   theo bước, cùng upper bound mềm chống nhấc quá mức. Mục tiêu có thể tăng theo
   tốc độ; không nhân toàn bộ penalty với foot speed. Không chỉ bỏ hệ số speed
   của hàm cũ vì sẽ phạt cả chân trụ ở z=0. Reset state theo env; dùng mode latch.
2. `swing_lateral_deviation`: phạt phần lệch ngang vượt hành lang từ liftoff tới
   vùng đặt chân, trong heading frame cố định cho bước. Thêm bound signed step
   width chống crossing; không ép y=0, không khóa hip roll/yaw. Điều kiện theo
   command để vẫn quay/đi ngang/đi lùi và bước phục hồi thăng bằng được.
3. `stance_extension`: khuyến khích duỗi hình học chân vừa phải giữa pha trụ,
   giảm khi touchdown và swing. Điều chỉnh posture hip/knee/ankle trong gait
   để bớt xung đột; không đổi default làm mốc action sau export.
4. `soft_landing`: phạt vận tốc đi xuống khi chân hạ gần nền; đánh giá force
   peak/impulse trong cửa sổ touchdown, chuẩn hóa theo mg. Áp dụng WALK và W2S;
   chân trụ đỡ trọng lượng không phải impact và không nên bị phạt như va đập.
5. Giữ velocity tracking, contact timing, slip và action smoothness; log từng
   reward weighted contribution. Ramp từng nhóm; không chọn weight chỉ theo
   giá trị thô khác đơn vị. Contact/ground truth chỉ vào reward/critic/metrics,
   actor giữ proprioception 335D và FSM deployable.

Trình tự: phục hồi contract artifact -> đánh giá fixed checkpoint trên Python
train env và C++ -> sửa swing clearance/lateral trước -> nghiệm thu -> thêm
stance/landing nếu còn thiếu -> C0 đạt mới C1/C2. Đây là kế hoạch tại thời điểm đo.

## Cập nhật triển khai training — 2026-09-08

Đã triển khai reward vào ba task hiện có: `Unitree-R1-Flat`,
`Unitree-R1-Flat-Meta` (83D) và `Unitree-R1-Flat-Plus-Gait` (335D MLP).
Ngưỡng và phạm vi code cuối cùng nằm trong
[R1_WALK_QUALITY_UPDATE.md](../../../../unitree_rl_mjlab_meta/documents/R1_WALK_QUALITY_UPDATE.md).
Các số liệu rollout trong báo cáo vẫn là của **policy trước khi sửa training**;
không dùng chúng làm bằng chứng policy đã học reward mới.
