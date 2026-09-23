# R1 Videohub Camera in Quest — Design

## 1. Scope and status

- **Feature:** show the R1 EDU head camera in Quest and switch between Quest
  passthrough and a full-field robot view with a controller button.
- **Status:** design approved in conversation; implementation not started.
- **Affected workflow:** `make teleop-hardware` only.
- **Unaffected workflows:** simulation-only `make teleop`, IK mathematics,
  command schema, sidecar and sole-owner `rt/lowcmd` process.

The camera source has already been hardware-verified independently:
`VideoClient.GetImageSample()` on robot DDS interface `eth10` returned 10/10
complete 1280×720 JPEG frames with 13.5–23.3 ms successful-call latency. The
standalone diagnostic remains `scripts/teleop/test_r1_eye_camera.py`.

## 2. Requested outcome

When hardware teleoperation starts, the camera path starts with it. Quest begins
in passthrough. A rising edge on the right-controller A button changes to a
monocular robot-camera image that covers the full field of view; the next rising
edge returns to Quest passthrough. Holding A must not repeat the toggle.

Camera transport and rendering must remain independent of deadman, recalibration,
IK and actuation. Losing the camera must never create a second motor writer or
block the command loop.

## 3. Selected architecture

```text
R1 main computer camera
        │
        ▼
videohub DDS RPC on robot eth10
        │  one VideoClient, latest-frame mailbox
        ├──────────────► original JPEG recording on robot
        │
        ▼
640×360 JPEG quality 60, latest frame only
        │
robot HTTP server on 127.0.0.1:8765
        │
SSH local forward created by hardware launcher
        │
http://127.0.0.1:8765/preview.jpg on workstation
        │
background subscriber: fetch → decode BGR → latest frame only
        │
project-owned switchable TeleVuer scene
        │
ImageBackground full field of view ↔ Quest passthrough
```

The design intentionally does not add WebRTC/`aiortc`. The current `tv`
environment has OpenCV, NumPy, ZMQ and Vuer but not `aiortc`; the robot has the
Unitree SDK and OpenCV but not PyZMQ. HTTP over an SSH local forward therefore
uses existing dependencies on both tiers and does not expose an unauthenticated
camera socket to LAN or Tailscale.

## 4. Component responsibilities

### 4.1 Robot camera gateway

A project-owned module under `hardware/teleop/src/teleop/hardware/` owns the
only `VideoClient` for this workflow.

- Initialize DDS domain 0 on `eth10`.
- Set videohub RPC timeout to 0.1 s and cap requests at 15 Hz.
- Accept only complete JPEGs with SOI and EOI markers.
- Keep one original JPEG and one preview JPEG in latest-frame mailboxes.
- Decode, resize and encode preview once per accepted source frame, not once per
  HTTP request.
- Serve HTTP only on `127.0.0.1`; default port is 8765.
- Provide `GET /preview.jpg` and `GET /health`.
- Never create a DDS publisher, import a low-command message or contact the
  high-level owner.

`/preview.jpg` returns the latest 640×360 quality-60 JPEG and headers containing
the source sequence, robot-computed frame age and source dimensions. `/health`
returns counters for successful RPC calls, failures by code, invalid JPEGs,
preview encode failures, recording failures and latest-frame age.

### 4.2 Robot recording

When the hardware launcher supplies a unique recording directory, the gateway
writes every accepted original JPEG without recompression and an append-only
JSONL manifest containing sequence number, robot monotonic receive time, byte
count, dimensions and SHA-256. Disk writing runs in a bounded worker queue so it
cannot block capture or preview. A full queue records a drop counter; it does
not grow without limit.

The gateway refuses to overwrite an existing recording directory. At shutdown
it writes a summary after flushing the bounded queue. The hardware launcher
copies this directory into the local immutable run bundle and marks the run
incomplete when the camera process fails, no original frame exists, the manifest
does not match the files, or a recording drop occurred.

### 4.3 SSH transport lifecycle

The hardware launcher starts one foreground remote camera process through a
dedicated SSH connection with local forwarding:

```text
workstation 127.0.0.1:8765 → robot 127.0.0.1:8765
```

It checks that the local port is free, uses `ExitOnForwardFailure=yes`, waits for
`/health` to report a fresh frame, and only then starts the Quest/control
pipeline. Cleanup terminates the SSH process, which terminates the foreground
remote gateway. The launcher must not kill an unknown process occupying either
port.

Built-in camera integration is enabled by default for hardware teleoperation and
can be disabled explicitly with `HB_ROBOT_CAMERA=0`. If enabled startup fails,
the launcher exits before opening the command pipeline rather than silently
running without the requested camera.

### 4.4 Workstation preview subscriber

A background subscriber in the `tv` environment performs bounded HTTP requests
through the local tunnel.

- Request at 10 Hz by default; configurable within 1–15 Hz.
- HTTP timeout: 0.4 s.
- Decode only successful complete JPEG responses.
- Keep only the newest decoded BGR frame and discard superseded frames.
- Never retry a request concurrently or accumulate a response queue.
- Expose counters, last error, source age, receive age and sequence.

The Quest/control loop only reads a non-blocking snapshot from this subscriber.
HTTP, JPEG decoding and tunnel stalls never execute on the command-loop thread.

### 4.5 Switchable Vuer scene

The project-owned adapter continues to subclass the pinned TeleVuer class rather
than editing `third_party/`. It allocates an independent 360×640×3 shared RGB
buffer for the robot preview. A new frame is copied into that buffer only after
successful decoding.

When robot view is enabled, the Vuer child process upserts one monocular
`ImageBackground` using the same full-background geometry as the vendor
immersive mode. When disabled, it removes that keyed background, revealing Quest
passthrough. Controller acquisition remains active in both views.

## 5. View and controller semantics

| Input/state | Behavior |
|---|---|
| Session starts | Quest passthrough |
| Right A rising edge with fresh preview | Robot camera, full field of view |
| Right A held | No additional toggle |
| Next right A rising edge | Quest passthrough |
| Right trigger | Deadman only; unchanged |
| Left trigger | Recalibration only; unchanged |
| Other controller buttons | No camera effect unless explicitly configured |

The existing configurable camera-toggle button contract remains, but right A is
the canonical default. The first controller sample only arms edge detection, so
a button already held when the session connects cannot unexpectedly hide
passthrough.

## 6. Freshness and fallback

- Enabling robot view requires both robot-reported source age and workstation
  receive age to be at most 0.5 s.
- While robot view is active, either age exceeding 1.0 s automatically removes
  the background and returns to Quest passthrough.
- A rejected enable and an automatic fallback each produce a structured
  connection-log event with the reason and observed ages.
- Camera failure does not change deadman state, emit reset, alter IK output or
  stop the control loop.
- Camera startup failure is different from runtime failure: requested camera
  startup is a preflight gate; runtime staleness is a display fallback.

No cross-machine monotonic timestamps are subtracted. The robot computes frame
age before writing the HTTP response; the workstation separately measures
request/receive age.

## 7. Bandwidth and latency contract

The verified original frames are approximately 356 KB. Sending them at 10–15
FPS would require roughly 28.5–42.8 Mbps before SSH and Vuer overhead, so original
JPEGs are not used as the live preview transport.

The selected preview default is 640×360, JPEG quality 60 at 10 Hz. Prior evidence
on this R1 measured approximately 15.3 KB per such preview frame, or about 1.2
Mbps at 10 Hz and 1.8 Mbps at 15 Hz, with approximately 41 ms mean preview encode
time. These values are evidence-informed defaults, not a claim about final Quest
latency.

The transport never queues old frames. TCP loss may delay one HTTP response, but
the next successful response returns the newest mailbox frame. WebRTC remains a
possible later optimization if a declared field test shows this bounded preview
cannot meet the acceptance gate.

## 8. Performance acceptance gate

Before declaring the camera view field-ready, run a suspended-hardware pilot for
at least 10 minutes under the intended access point and network topology.

Required outcomes:

- displayed source rate at least 10 FPS;
- source/receive frame-age p95 below 250 ms;
- fewer than 1% of samples older than 500 ms;
- no unbounded queue or monotonic memory growth;
- every forced camera interruption returns to passthrough within 1.0 s;
- deadman release and sidecar watchdog behavior remain unchanged;
- raw recording manifest matches recorded files with zero queue drops.

End-to-end photon-to-Quest latency cannot be inferred from clocks on different
machines or from server counters. It requires a separate optical test with a
visible time-changing target or flash recorded together with the Quest display.

## 9. Security and network boundaries

- Camera HTTP binds robot loopback only.
- The workstation endpoint also binds loopback only and exists through SSH local
  forwarding.
- The Quest receives pixels through its existing authenticated HTTPS/WSS Vuer
  session; it never contacts the robot HTTP service.
- No camera credentials or query tokens are written to logs.
- The detected `ROBOT` SSH target may be LAN or Tailscale. LAN is preferred for
  video; a Tailscale DERP-relayed path is not accepted as field-ready evidence.

## 10. Error reporting and observability

The robot summary and bridge connection log must make these states distinct:

- videohub timeout code such as 3104;
- malformed or truncated JPEG;
- preview encode failure;
- HTTP/tunnel request failure;
- stale-frame enable rejection;
- automatic passthrough fallback;
- recording queue drop or disk error;
- operator-requested view transition.

Logs record counters and sanitized endpoints, never JPEG content or private keys.

## 11. Testing strategy

Implementation follows test-first development.

1. Unit-test the robot puller with a fake `VideoClient`: success, RPC timeout,
   truncated JPEG, latest-frame replacement and bounded recording queue.
2. Exercise the real HTTP server on loopback: health before/after first frame,
   JPEG headers, frame-age header and shutdown.
3. Unit-test the workstation subscriber: newest-frame semantics, timeout,
   malformed JPEG, source-age propagation and no blocking snapshot.
4. Test Vuer scene behavior with a fake session: full-field background added and
   removed on state changes, no repeated upsert for an unchanged frame, and
   controller node remains present.
5. Preserve existing edge-latch tests and add stale-enable/auto-fallback tests.
6. Test launcher wiring and cleanup without opening a hardware command channel.
7. Run the full repository suite.
8. Run a read-only robot gateway/tunnel smoke and inspect the rendered frame on
   the workstation before asking the operator to test in Quest.

## 12. Consequential choices

| Choice | Value | Origin | Rationale |
|---|---|---|---|
| Robot view layout | full field, monocular | specified | Researcher request |
| Toggle | right A rising edge | inherited/approved | Existing project adapter; independent of triggers |
| Initial view | Quest passthrough | approved | Fail-visible and preserves direct surroundings |
| Source | one videohub `VideoClient` | empirically selected | Verified R1 EDU camera interface |
| Preview | 640×360, JPEG quality 60, 10 Hz | empirically selected | Measured 23.8× payload reduction and 15 FPS smoke capacity |
| RPC timeout/rate cap | 0.1 s / 15 Hz | empirically selected | Existing R1 measurements |
| HTTP timeout | 0.4 s | AI-selected | Above observed source gaps without blocking control |
| Enable freshness | 0.5 s | approved | Reject visibly stale initial robot view |
| Runtime fallback | 1.0 s to passthrough | approved | Avoid prolonged frozen full-field view |
| Hardware default | camera enabled; explicit opt-out | AI-selected | Requested integrated hardware behavior |
| Transport | HTTP loopback through SSH | approved | Existing dependencies, no exposed camera socket |
| Original image | record on robot without recompression | approved | Preserve camera evidence without wireless raw-stream cost |

## 13. Semantic compatibility

| Contract | Status | Compatibility |
|---|---|---|
| Observation/display | changed | Adds opt-in visual source, not controller input |
| Action/actuation | unchanged | No command schema or target change |
| Deadman/recalibration | unchanged | Triggers retain existing roles |
| Control timing | unchanged by contract | Camera work remains off command-loop thread |
| Data/evidence | changed | Adds original JPEG manifest and camera status |
| Simulation baseline | unchanged | Hardware integration does not alter sim-only runs |
| Prior evidence | compatible with caveat | Old runs contain no integrated R1 camera evidence |

## 14. Review surface

The implementation plan must keep the consequential behavior concentrated in:

- robot gateway module: source, preview, recording and health;
- workstation subscriber: transport, decode and freshness;
- `robot_camera_vuer.py`: shared frame and full-field scene switching;
- `quest_bridge.py`: non-blocking feed, button events and fallback logging;
- `run_r1_quest3_hardware.sh`: process lifecycle, tunnel, preflight and artifact
  collection;
- focused tests for each boundary plus the existing static motor-safety test.

No vendor file under `third_party/` may be modified.
