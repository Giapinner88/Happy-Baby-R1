# Kế hoạch nghiên cứu — Quest 3 teleop arm/wrist R1 trong simulation

## Trạng thái và phạm vi

Đây là kế hoạch, không phải bằng chứng đã chạy. File này là pipeline có thẩm
quyền của thí nghiệm: nó sở hữu toàn bộ câu hỏi T-series, evidence bắt buộc và
gate của từng protocol. Các run mới nằm ở
`experiments/r1_teleop/quest3_sim_v1/Txxx/runs/<run-id>/`, trong đó `Txxx` sở
hữu metadata, config và figure của protocol; run ID vẫn mang tiền tố của
protocol (`t001_a_...` thuộc `T001`). Record thực thi nằm tại thư mục T sở hữu
nó. Lộ trình và boundary hardware được chốt
trong [`D001`](../../../decisions/r1_teleop/D001_teleop_stage_gates.md).

Asset R1 hiện xác nhận 5 DOF mỗi tay: shoulder pitch/roll/yaw, elbow và wrist
roll. `left_hand_collision`/`right_hand_collision` là collision geometry; không
có finger hoặc gripper joint trong asset hiện hành. Vì vậy chuỗi dưới đây chỉ
chứng minh **arm + wrist-roll + rigid hand endpoint**, không tuyên bố dexterous
hand teleoperation hay grasping.

Biên an toàn giữ nguyên: IsaacLab trực tiếp trong `unitree_sim_env`, không DDS,
ROS hardware, `LowCmd` hay import `hardware/high_level/`. Base velocity tắt cho
đến khi policy gate IsaacLab đã pass; các thí nghiệm đầu chỉ chạy upper body và
head với base đứng yên.

## Câu hỏi chính

Với một kết nối Quest → bridge → R1 simulator đã được quan sát, calibration
khai báo và joint ownership rời nhau, R1 simulator có thể theo dõi các mục tiêu
arm/wrist trong vùng reachable mà không vi phạm joint/rate/collision
constraints và fail-closed đúng khi input không hợp lệ hay bị mất không?

Trước mỗi experiment, config phải khóa threshold pass/fail, simulator timestep,
solver, seed và trace. Kế hoạch này định nghĩa metric và gate, không tự bịa số
threshold khi chưa có audit limit và baseline.

## Method gate trước khi chạy IK

Tạo method record `docs/teleop/r1_arm_wrist_ik.md` trước T002, với các mục sau:

- frame `quest_headset`, calibrated `r1_base`, torso và wrist endpoint;
- vị trí target 3D, wrist-roll scalar; quaternion đầy đủ trong schema không được
  hiểu là target orientation 6D vì arm hiện chỉ có 5 DOF;
- mapping Quest pose → calibrated target, scale (nếu được thêm), offset neutral,
  workspace clip và continuity qua wrap của wrist roll;
- objective IK, joint-limit/rate/acceleration constraints, collision handling,
  solver stopping/fallback và command hold behavior;
- ownership: IK chỉ ghi 10 arm/wrist joint + head; locomotion policy chỉ ghi
  lower body/base command; waist chỉ được phân lại qua method change rõ ràng;
- unit, sign, update rate và cơ chế timestamp/latency.

Gate M: method có unit/frame audit, mapping bằng zero/identity case, joint list
khớp asset, và test cho joint-ownership. Nếu method gate fail, sửa method trước
khi tạo run evidence.

## Chuỗi pilot và study

T001 được tách thành hai evidence gate khác nhau:

- **T001-A:** chỉ thu Quest transport/state data; không tạo R1 command và
  không gọi simulator.
- **T001-B:** dùng live Quest data để chứng minh Quest → project bridge →
  Isaac Sim R1, bao gồm connect/reconnect và fail-closed khi deadman hoặc
  connection mất.

T001-A không tự chứng minh simulator connection. T001-B chỉ được mở sau khi
T001-A có data hợp lệ theo protocol; không cần lặp lại T001-A nếu selected
capture hiện có vẫn tương thích với bridge/config cần kiểm tra.

### Trạng thái evidence cập nhật 2026-08-02

Toàn bộ evidence run cũ của T001 đã được xóa theo yêu cầu để reset thí nghiệm.
T001-A và T001-B đều quay về trạng thái executable, chưa có run retained. T002
vẫn đóng cho đến khi một T001-A mới pass ở endpoint cố định, rồi T001-B mới
được ghi đủ evidence theo protocol.

Trình tự thao tác sau reset chính là chuỗi T-series bên dưới: T001-A capture →
T001-B simulation application → T002 trở đi. Không có lớp stage riêng nào chồng
lên nó nữa. Điều này ngăn một tuyên bố về calibration, synchronization hay mimic
được đưa ra trước khi có evidence T001 capture/simulation mới.

### Shared mapper preflight (không mang số T001)

Synthetic mapper preflight không phải kiểm tra kết nối teleop và toàn bộ output
preflight cũ đã bị xóa trong reset. Có thể chạy lại nó ở namespace disposable,
nhưng không được suy từ `FakeIsaacLabSink` sang kết nối Quest hay physics
simulator và không được dùng làm evidence T001-A/T001-B.

| ID | Câu hỏi hẹp | Protocol / biến độc lập | Evidence bắt buộc | Gate tiếp theo |
| --- | --- | --- | --- | --- |
| T001-A | Quest có tạo được transport/state data hợp lệ trên host không? | Một headset, host endpoint và input mode cụ thể; capture-only, không bridge R1, không simulator. | Raw vendor transport telemetry, config/provenance/status, sample count, `motion_data_ready`, finite pose-matrix check. | Chỉ qua khi data live đạt criteria đã khai báo. Đây là precondition cho T001-B, không phải proof of Isaac Sim connection. |
| T001-B | Live Quest data có đi qua project bridge tới R1 Isaac Sim, được quan sát và fail-closed khi ngắt không? | T001-A data/live session, bridge cụ thể tạo `R1TeleopCommand` JSONL; base đứng yên; bridge gửi vào simulator sink đã tích hợp, không DDS/hardware. Test connect, reconnect, mất connection/deadman và clock handshake. | Phiên bản/command bridge, startup & connection logs, raw live JSONL, calibration session, bridge-to-sink acknowledgement, clock/latency record, hold event khi disconnect, video simulator. | Chỉ qua khi đã quan sát end-to-end path và disconnect tạo hold; output từ `FakeIsaacLabSink` không thay thế bằng chứng này. Mở T002. |
| T002 | Những wrist endpoint nào reachable trong joint limits, và mapper có đưa các target cùng bridge/sink đó vào IK đúng frame không? | Base cố định; lưới mục tiêu 3D được khai báo trong workspace; neutral calibration; left/right chạy riêng rồi đồng thời. Trước IK, replay neutral/offset/head-turn/deadman/stale/frame/sequence trace qua đúng bridge/sink của case. | Resolved IK config, raw JSONL/calibration, mapped target & safety events, target/solved joint trace, FK residual, solver status, limit margin, self/contact events, video. | Mapper phải giữ ownership và fail-closed; chỉ giữ workspace reachable. Failure do unreachable, collision hoặc solver non-convergence phải được giữ trong manifest. |
| T003-A | Rate limiter có theo trajectory arm/wrist chậm trong **nominal valid input** không? | Một trace deterministic reach-hold-return, horizontal/vertical và wrist-roll; không chèn safety event. | Per-step raw command/target/state/action, position & wrist-roll error CSV, q/qdot/qdd/torque, video và metric nominal. | Chọn baseline nominal theo metric đã khai báo. |
| T003-B | Mỗi fail-closed transition có giữ target đúng không, và transient quan sát được là gì? | Ba run/case độc lập: deadman release, stale timeout, non-increasing sequence. Không gộp các injection vào cùng run. | Raw trace, event log, q/qdot/qdd/torque, video và verification hold/no lower-body theo **từng** case. | Mỗi case phải hold/zero theo method; so sánh smoothness với T003-A không được gộp maxima. |
| T004 | Calibration sai lệch ảnh hưởng thế nào đến usable workspace? | Study matrix: yaw offset, translation offset và left/right trace; fixed solver từ T003. | Machine-readable case manifest, complete status table, error/limit/collision metrics, aggregate CSV/plots, representative video cho success/borderline/failure. | Xác định vùng calibration sử dụng được hoặc kết luận cần calibration procedure tốt hơn. |
| T005 | Input jitter, packet loss và watchdog có giữ simulator an toàn không? | Replays của T003 được inject timestamp jitter, drop, stale packet, deadman transitions và sequence violation. | Raw injected trace, latency distribution, clamp/hold events, final posture/state, video của failure cases. | Mọi path invalid/stale/deadman phải hold upper body hoặc zero command theo method; không có lệnh cũ tiếp tục chạy. |
| T006 | Hai tay có vận hành đồng thời trong workspace được chọn mà không va chạm hoặc chồng quyền sở hữu không? | Bilateral gestures từ T002/T003, mirror/asymmetric/cross-body cases; base vẫn đứng yên. | Dual-arm trace, collision/contact events, joint ownership audit, metrics/video và status cho từng case. | Chỉ các region pass mới thành candidate trace cho live Quest bridge. |
| T007 | Live Quest bridge có tái tạo được hành vi trace replay trong simulator không? | Live input được bridge thành `R1TeleopCommand` JSONL; replay lại chính raw trace offline; base đứng yên. | Raw live JSONL, calibration session record, clock/latency record, online and offline target/state comparison, video. | Sai khác online/offline trong threshold đã chốt; nếu không thì chẩn đoán timestamp/frame/bridge. |
| T008 (sau policy gate) | Locomotion policy và arm teleop có cùng chạy mà không ghi trùng joint hay làm mất ổn định không? | Chỉ sau policy manifest IsaacLab R1 được promote và evaluation pass. So sánh base đứng yên với các slow base commands đã được policy đánh giá; upper-body traces pass T006. | Selected policy manifest/signature, policy evaluation reference, disjoint action ownership record, base/arm state-action trace, fall/timeout/collision events, video. | Đây vẫn là simulator-only; fail giữ velocity disabled và quay về T006/T007. |
| T009 (provisional) | Có thể tái lập một subset stationary, fail-closed của T008 trên R1 hardware dưới safety procedure đã duyệt không? | Protocol hardware riêng, base đứng yên và chỉ action subset đã được chọn sau review; chưa khai báo command/threshold. | Hardware-specific config/provenance, operator and safety log, raw robot state/action/event record, video, stop/E-stop events, protocol-deviation record. | Chỉ mở khi Gate H đạt và một experiment record hardware được duyệt. Không suy hardware readiness từ simulation. |
| T010 (provisional) | Subset capability nào từ T009 có thể được kiểm tra trên hardware dưới protocol có giám sát không? | Xác định sau T009 analysis; không được tự động kế thừa command, threshold hoặc scope từ simulation. | Evidence và validity contract được khai báo trong record T010 sau T009. | Chỉ mở sau T009 valid/analysed và safety decision riêng. |

T001-A, T001-B, T002, T003-A và T003-B là bounded pilots theo `experiment.md`. T004 là
simulation study vì có case matrix. T005–T008 là bounded experiments trừ khi
analysis buộc phải mở rộng ma trận. Không chạy T008 để “thử cho biết” trước
policy gate. T009 và T010 là hai experiment hardware dự kiến, không phải phần
mở rộng tự động của T008.

## Gate H — simulation sang hardware

T009 không được mở chỉ vì một simulation video đẹp. Trước khi tạo protocol
hardware, cần có: T001-A valid data; T001-B valid end-to-end Isaac Sim evidence;
T002–T008 complete theo protocol và mọi failure/invalid case được giữ; method,
asset/controller/signature compatibility được review cho deployment target; và
SOP/safety procedure, E-stop operator, test zone, stop/fallback behavior cùng
logging được duyệt cho experiment hardware. Các acceptance threshold cụ thể,
robot state, action subset, giới hạn vận hành và rollback condition thuộc record
T009; chúng chưa được định nghĩa trong simulation plan này.

## Contract dữ liệu chung

Mỗi evidence run phải lưu:

- immutable resolved config, command, git commit, environment/simulator version,
  asset hash, seed và run status;
- raw normalized Quest command trace, calibration record và input injection
  record (nếu có);
- target IK, joint target, observed joint/base state, action, timestamps và
  raw collision/contact/watchdog events ở sampling rate đã khai báo;
- checkpoint/model/solver version khi có learned component;
- metric CSV, script tạo plot, plot PNG và MP4 từ camera đã định danh;
- `evidence_completeness.json` liệt kê rõ hiện diện/thiếu dữ liệu.

Metric tối thiểu: endpoint position residual, wrist-roll residual, solver
convergence rate, distance to joint limit, peak/rms joint velocity và
acceleration, collision/contact count, command-to-target/state latency, hold /
watchdog event count và tỷ lệ case pass/fail/invalid. T008 thêm base velocity
tracking, fall/termination và joint ownership violations. Mỗi metric phải mang
unit, frame, sampling window và quy tắc aggregation trong config của experiment.

## Quy tắc validity và analysis

- Crash, missing raw trace/video, asset-hash mismatch, config không resolved,
  or ownership overlap là execution/verification failure, không phải successful
  teleop.
- Unreachable target, joint-limit saturation hay collision với config hợp lệ là
  scientific failure hợp lệ và phải được giữ để xác định workspace.
- Mỗi study phải có status cho tất cả case; aggregation không được bỏ case fail.
- Sau T004, T006 và T008 mới tạo `analysis.md`; analysis so sánh trace thành
  công, failure, borderline và giải thích cạnh tranh trước khi vẽ figure evidence.

## Nhánh hand/finger riêng

Finger/gripper teleop bị out of scope cho v1 vì asset không có DOF. Chỉ mở nhánh
mới khi có asset R1 có joint, limit, actuator và observation/action signature
được kiểm chứng. Khi đó phải tạo method mới, schema version mới (không sửa ngầm
`R1TeleopCommand` v1), asset compatibility record, T001-B-style interface pilot
và chuỗi reach/grasp riêng. Kết quả arm/wrist ở đây không được dùng làm bằng
chứng cho finger control.
