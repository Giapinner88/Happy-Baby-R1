# Mirrored hardware + Isaac validation — implementation disclosure

## Scope and requested behavior

Hardware teleop phải bật Isaac đồng thời để ghi chuyển động mô phỏng, ghi camera
robot và cho phép so sánh cùng input. Nhả cò phải phải pause thay vì đóng toàn
bộ pipeline; cò trái phải reset/rehome được trong cùng session.

Status: implemented and code-verified; chưa workflow-verified với Quest, camera
và R1 thật sau thay đổi này. Không có evidence mới cho phép chạy robot trên sàn.

## Effective architecture

```text
Quest → TeleVuer → upstream IK → hardware rate limiter (10 Hz)
                                  │ same vector + sequence
                                  ▼
                         non-blocking fanout
                           │             │
                           │             └── Isaac fixed-base → simulator_view.mp4
                           ▼
                    sidecar → sole owner → R1

TeleImager head camera ── WebRTC ──► Vuer/Quest
                        └ ZMQ JPEG ─► recorder → robot_camera.mp4 + receive timestamps
```

Fanout stdout hardware là authoritative. Mirror Isaac dùng queue 512 dòng; nếu
đầy, dòng mirror cũ bị bỏ và ghi đếm, không làm chậm hardware. Isaac crash cũng
không đóng primary path. Đây là lựa chọn AI-selected vì an toàn command path ưu
tiên hơn evidence completeness.

## Input state machine

| State/event | Producer/sidecar behavior |
|---|---|
| right trigger held | `operator_enabled=true`, active target packet |
| right trigger released | fresh `operator_enabled=false`, explicit STOP packet mỗi tick |
| right trigger re-held | active target tiếp tục, không reconnect |
| left trigger rising edge | reset pending; upstream bỏ anchor cũ |
| next 3 active samples | xác nhận anchor mới; mẫu solved thứ ba mang reset/rehome |

Nhả cò trong initial automatic homing vẫn abort homing fail-closed; operator phải
giữ cò xuyên suốt pha này. Semantics pause/resume áp dụng sau khi initial homing
hoàn tất.

## Evidence and comparison

`compare_sim_hardware_run.py` join bằng Quest `sequence_id` /
`upstream_sequence_id` và báo ba lớp sai khác:

1. producer → simulator command: wiring parity, kỳ vọng bằng zero;
2. producer → robot target: ảnh hưởng mapping/envelope/head gate phía robot;
3. simulated state → encoder state: tổng sai khác plant/model/tracking, không
   được diễn giải riêng là controller error.

Camera timestamp là workstation receive time. Nó không đồng bộ exposure ở
camera, vì giao thức JPEG hiện không mang hardware capture timestamp.

## Semantic compatibility

| Contract | Change | Evidence impact |
|---|---|---|
| action source | same rate-limited vector fan-out | run mới có parity traceable |
| deadman | EOF → explicit PAUSED state | hardware evidence cũ cần caveat |
| recalibration | reset deferred until new anchor confirmed | run cũ không chứng minh behavior mới |
| simulator | fixed-base mirror starts by default in hardware workflow | cần reproduction |
| camera | optional ZMQ recorder | receive-time synchronization only |
| sole lowcmd owner | unchanged | sidecar remains UDP-only |

## Validation actually performed

- code verified: pause/resume sidecar loop, payload parsing, reset calibrator,
  non-blocking fanout, named-joint comparison, shell/Python syntax and full test suite;
- workflow verified: robot-camera recorder đã nhận 41 frame từ synthetic ZMQ
  publisher, không lỗi decode và tạo MP4; combined Quest + Isaac + robot +
  camera run not yet executed;
- scientifically reproduced: no.

The launcher also rejects paired evidence when `fanout_stats.json` reports a
mirror failure, a dropped command, or mismatched input/output line counts; the
gate result is stored as `fanout_evidence` in `auxiliary_status.json`.

Smallest next run is a suspended, E-stop-attended pilot: initial home, one
active motion, one pause/resume, one paused left-trigger reset, one camera view
toggle, then stop. Accept only if fanout drops are zero, producer→sim command
error is numerical zero, both videos exist, robot encoder evidence is complete,
and no watchdog/stream close occurs at the intentional pause.
