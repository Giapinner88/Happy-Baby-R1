# R1 Videohub Camera in Quest — Implementation Plan

> **For agentic workers:** REQUIRED: execute this plan with `superpowers:executing-plans`, task by task. Apply `superpowers:test-driven-development` to every behavior change and `superpowers:verification-before-completion` before reporting readiness.

**Goal:** Start the R1 EDU eye-camera path automatically with hardware teleoperation, record original robot frames, and let the operator toggle a fresh full-field robot view in Quest with the right A button without changing deadman, recalibration, IK, or motor ownership.

**Architecture:** A read-only robot process pulls `videohub` JPEGs, stores originals through a bounded writer, and publishes a pre-encoded 640×360 preview over loopback HTTP. A dedicated SSH process forwards that loopback endpoint to the workstation. A background subscriber decodes only the latest preview into a project-owned TeleVuer shared buffer; right-A edges add/remove one monocular `ImageBackground`, with freshness-gated enable and automatic passthrough fallback.

**Tech Stack:** Python 3, Unitree SDK `VideoClient`, OpenCV, NumPy, `http.server`, `urllib.request`, SSH local forwarding, Vuer/TeleVuer, pytest, Bash launcher.

**Spec:** `docs/superpowers/specs/2026-09-22-r1-videohub-quest-camera-design.md`

**Global Constraints:** Do not edit `third_party/`. Do not create a DDS publisher or any second motor writer. Preserve simulation-only `make teleop` and the existing optional WebRTC path. Network and JPEG work must never run on the command-loop thread. Do not kill an unknown port owner. The repository's `AGENTS.md` reserves commits for the researcher, so every task ends at a review checkpoint and does not create a commit.

**Review Focus:** Check (1) complete-JPEG and header validation, (2) bounded queues/latest-only behavior under slow disk or network, (3) SSH death and occupied-port startup failures, (4) frame-age gates and automatic fallback, and (5) right-A edge handling remaining independent of both triggers.

---

## Task 1: Build the read-only robot capture core

**Files:**

- Create: `hardware/teleop/src/teleop/hardware/r1_camera_gateway.py`
- Create: `hardware/teleop/tests/test_r1_camera_gateway.py`
- Modify: `hardware/teleop/tests/test_static_safety.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class CameraFrame:
    sequence: int
    received_monotonic_s: float
    original_jpeg: bytes
    preview_jpeg: bytes
    source_width: int
    source_height: int

class LatestFrameMailbox:
    def put(self, frame: CameraFrame) -> None: ...
    def snapshot(self) -> CameraFrame | None: ...

def is_complete_jpeg(payload: bytes) -> bool: ...
def encode_preview(payload: bytes, *, width: int, height: int, quality: int) -> tuple[bytes, int, int]: ...

class VideohubPuller:
    def run_once(self) -> bool: ...
```

- [ ] **Step 1: Write failing capture tests**

Use a fake client whose `GetImageSample()` returns `(code, bytes)` and an injected monotonic clock. Cover a valid JPEG, non-zero RPC code, empty/truncated JPEG, preview-encode failure, monotonically increasing accepted sequence, and replacement of the single mailbox slot.

```python
def test_puller_accepts_latest_complete_jpeg_only():
    client = FakeVideoClient([(0, JPEG_A), (3104, b""), (0, b"\xff\xd8cut")])
    mailbox = LatestFrameMailbox()
    puller = VideohubPuller(client, mailbox, preview_encoder=fake_preview, clock=FakeClock())
    assert puller.run_once() is True
    assert puller.run_once() is False
    assert puller.run_once() is False
    assert mailbox.snapshot().sequence == 1
    assert puller.counters.rpc_failures_by_code == {3104: 1}
    assert puller.counters.invalid_jpeg_count == 1
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `pytest -q hardware/teleop/tests/test_r1_camera_gateway.py -k 'puller or jpeg or mailbox'`

Expected: import or attribute failures because the gateway does not exist.

- [ ] **Step 3: Implement the minimum capture core**

Initialize `VideoClient` only inside the CLI/factory so unit tests do not require DDS. `VideohubPuller.run_once()` must accept only code `0`, complete SOI/EOI markers, and a successful OpenCV decode/preview encode. Update counters under the same lock as the latest frame. Use a one-slot mailbox, never a FIFO for display.

- [ ] **Step 4: Add static safety assertions**

Extend `test_static_safety.py` to assert that the gateway source contains neither `ChannelPublisher` nor `LowCmd_` nor `rt/lowcmd`. Also assert that importing the module does not initialize DDS or bind a socket.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `pytest -q hardware/teleop/tests/test_r1_camera_gateway.py hardware/teleop/tests/test_static_safety.py`

Expected: all tests pass; no Unitree hardware is required.

- [ ] **Step 6: Review checkpoint**

Run: `git diff --check -- hardware/teleop/src/teleop/hardware/r1_camera_gateway.py hardware/teleop/tests/test_r1_camera_gateway.py hardware/teleop/tests/test_static_safety.py`

Do not commit; the researcher owns commits.

## Task 2: Add bounded original recording and loopback HTTP

**Files:**

- Modify: `hardware/teleop/src/teleop/hardware/r1_camera_gateway.py`
- Modify: `hardware/teleop/tests/test_r1_camera_gateway.py`

**Interfaces:**

```python
class OriginalJpegRecorder:
    def submit(self, frame: CameraFrame) -> bool: ...
    def close(self) -> dict[str, object]: ...

def build_http_server(
    mailbox: LatestFrameMailbox,
    counters: GatewayCounters,
    *, host: str, port: int, clock: Callable[[], float],
) -> ThreadingHTTPServer: ...

def main(argv: Sequence[str] | None = None) -> int: ...
```

- [ ] **Step 1: Write failing recorder tests**

Verify refusal to reuse an existing recording directory, byte-for-byte original JPEG output, append-only JSONL entries with sequence/monotonic time/size/dimensions/SHA-256, bounded `queue.Queue(maxsize=N)`, a counted submit rejection when full, a counted disk exception, flush on close, and a terminal `summary.json`.

```python
def test_recorder_never_blocks_when_queue_is_full(tmp_path):
    recorder = OriginalJpegRecorder(tmp_path / "run", queue_capacity=1, writer=BlockingWriter())
    assert recorder.submit(frame(1)) is True
    assert recorder.submit(frame(2)) is False
    assert recorder.snapshot().queue_drop_count == 1
```

- [ ] **Step 2: Write failing loopback HTTP tests**

Start the real server on `127.0.0.1` and port `0`. Assert `/health` before a frame reports `ready: false`; after insertion, `/preview.jpg` returns the exact preview bytes and `X-R1-Sequence`, `X-R1-Source-Age-Ms`, `X-R1-Source-Width`, and `X-R1-Source-Height`. Assert unknown paths return 404 and the server shuts down cleanly.

- [ ] **Step 3: Run the focused tests and verify RED**

Run: `pytest -q hardware/teleop/tests/test_r1_camera_gateway.py -k 'recorder or http or health'`

Expected: missing recorder/server behavior.

- [ ] **Step 4: Implement recorder, HTTP server, and CLI**

The CLI defaults are `--interface eth10`, `--rpc-timeout-s 0.1`, `--capture-hz 15`, `--preview-width 640`, `--preview-height 360`, `--preview-quality 60`, `--host 127.0.0.1`, and `--port 8765`. Reject any non-loopback `--host`. Install SIGINT/SIGTERM handlers, stop capture, flush the writer, write the summary, then close HTTP.

The writer failure is observable but must not stop capture/preview. Its summary makes the run incomplete later. The HTTP handler must snapshot existing bytes and must never encode or contact DDS.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `pytest -q hardware/teleop/tests/test_r1_camera_gateway.py hardware/teleop/tests/test_static_safety.py`

Expected: all tests pass.

- [ ] **Step 6: Review checkpoint**

Run: `git diff --check -- hardware/teleop/src/teleop/hardware/r1_camera_gateway.py hardware/teleop/tests/test_r1_camera_gateway.py`

Do not commit; the researcher owns commits.

## Task 3: Implement the workstation latest-frame subscriber

**Files:**

- Create: `scripts/teleop/robot_camera_stream.py`
- Create: `tests/teleop/test_robot_camera_stream.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class RobotCameraSnapshot:
    sequence: int
    bgr: np.ndarray
    robot_source_age_s: float
    received_monotonic_s: float

class HttpRobotCameraSubscriber:
    def start(self) -> None: ...
    def snapshot(self) -> RobotCameraSnapshot | None: ...
    def stats(self) -> dict[str, object]: ...
    def close(self) -> None: ...
```

- [ ] **Step 1: Write failing subscriber tests**

Inject a fetch function and clock. Cover successful decode, timeout, non-200 response, incomplete JPEG, invalid/missing/non-finite/negative age or sequence headers, duplicate/out-of-order sequence, and replacement by the newest frame. Confirm `snapshot()` returns immediately while the fetch function is blocked.

```python
def test_snapshot_does_not_wait_for_blocked_http_fetch():
    fetch_started, release = Event(), Event()
    subscriber = HttpRobotCameraSubscriber(..., fetch=blocking_fetch(fetch_started, release))
    subscriber.start()
    assert fetch_started.wait(1.0)
    started = time.monotonic()
    assert subscriber.snapshot() is None
    assert time.monotonic() - started < 0.02
    release.set()
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `pytest -q tests/teleop/test_robot_camera_stream.py`

Expected: import failure because the subscriber does not exist.

- [ ] **Step 3: Implement the subscriber**

Use one daemon thread, one sequential HTTP request at a time, default 10 Hz and timeout 0.4 s. Validate endpoint scheme/host as loopback by default, JPEG markers, headers, and OpenCV decode. Copy the decoded array into an immutable snapshot reference under a short lock. `close()` sets an event, closes the active response when possible, and joins with a bounded timeout.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `pytest -q tests/teleop/test_robot_camera_stream.py`

Expected: all tests pass.

- [ ] **Step 5: Review checkpoint**

Run: `git diff --check -- scripts/teleop/robot_camera_stream.py tests/teleop/test_robot_camera_stream.py`

Do not commit; the researcher owns commits.

## Task 4: Add a full-field local-frame TeleVuer adapter

**Files:**

- Modify: `scripts/teleop/robot_camera_vuer.py`
- Modify: `tests/teleop/test_robot_camera_vuer.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class RobotCameraFrameConfig:
    toggle_button: str = "right_a"
    width: int = 640
    height: int = 360
    aspect_ratio: float = 16.0 / 9.0

def build_frame_switchable_televuer_wrapper(
    base_wrapper_cls: type,
    *, config: RobotCameraFrameConfig,
) -> type: ...

# Added to the generated wrapper instance:
def set_robot_camera_frame(self, bgr: np.ndarray, sequence: int) -> bool: ...
def set_robot_camera_enabled(self, enabled: bool) -> None: ...
```

- [ ] **Step 1: Preserve existing toggle/WebRTC tests and add failing frame-view tests**

Use fake TeleVuer/Vuer session objects. Verify one independent `360×640×3` shared RGB buffer; BGR-to-RGB copy only for a newer sequence; one monocular `ImageBackground` with `height=1`, `distanceToCamera=1`, `aspect=16/9`, JPEG format and stable key; removal on disable; controller acquisition remains mounted in both states; unchanged frame/state does not cause repeated scene mutation.

- [ ] **Step 2: Add controller regression cases**

Keep the first held A sample as an arming sample. Add a sequence where right trigger is held while A rises and another where left trigger rises while A is held. Assert one toggle per A edge and no trigger-derived toggle.

- [ ] **Step 3: Run the focused test and verify RED**

Run: `pytest -q tests/teleop/test_robot_camera_vuer.py`

Expected: missing local-frame config/builder assertions fail; existing tests remain green.

- [ ] **Step 4: Implement without modifying vendored code**

Keep `RobotCameraConfig` and `build_switchable_televuer_wrapper()` backward-compatible for the optional WebRTC simulation path. Add the local-frame builder beside it and use the same geometry as vendor immersive mono. Allocate shared memory in the parent and unlink it exactly once during close.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `pytest -q tests/teleop/test_robot_camera_vuer.py`

Expected: all old and new tests pass.

- [ ] **Step 6: Review checkpoint**

Run: `git diff --check -- scripts/teleop/robot_camera_vuer.py tests/teleop/test_robot_camera_vuer.py`

Do not commit; the researcher owns commits.

## Task 5: Wire freshness gating and fallback into `quest_bridge.py`

**Files:**

- Modify: `scripts/teleop/quest_bridge.py`
- Modify: `tests/teleop/test_r1_teleop.py`
- Modify: `tests/teleop/test_robot_camera_stream.py`

**Arguments:**

```text
--robot-camera-preview-url http://127.0.0.1:8765/preview.jpg
--robot-camera-http-timeout-s 0.4
--robot-camera-poll-hz 10
--camera-enable-max-age-s 0.5
--camera-fallback-max-age-s 1.0
```

- [ ] **Step 1: Write failing argument and state-machine tests**

Reject simultaneous `--robot-camera-preview-url` and `--robot-camera-webrtc-url`. Assert startup remains passthrough, fresh right-A edge enables, stale right-A edge is rejected, next A edge disables, and either robot-source age or workstation receive age above 1.0 s forces fallback.

Use an injected fake subscriber and wrapper for bridge-level tests. Assert the transition log contains `camera_view_changed`, `camera_enable_rejected`, or `camera_runtime_fallback` with ages/reason, and never contains endpoint query secrets.

- [ ] **Step 2: Add trigger semantics regression test**

Feed one continuous telemetry session through right-trigger release/repress and left-trigger recalibration while A is pressed/released. Assert right-trigger release emits paused/no command but does not close TeleVuer; left trigger still produces a reset edge; camera state changes only on A rising edges.

- [ ] **Step 3: Run focused tests and verify RED**

Run: `pytest -q tests/teleop/test_r1_teleop.py tests/teleop/test_robot_camera_stream.py tests/teleop/test_robot_camera_vuer.py`

Expected: preview arguments and freshness behavior are absent.

- [ ] **Step 4: Implement non-blocking bridge wiring**

Construct/start the subscriber before entering the command loop, read only `snapshot()` inside the loop, and call `set_robot_camera_frame()` only for a new sequence. Measure workstation receive age as `loop_start - received_monotonic_s`; use the robot-computed age from the response header separately. Disable before logging fallback. Close subscriber before wrapper in `finally`.

WebRTC behavior remains available and mutually exclusive. If no camera argument is supplied, do not import/start camera transport and preserve current behavior.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `pytest -q tests/teleop/test_r1_teleop.py tests/teleop/test_robot_camera_stream.py tests/teleop/test_robot_camera_vuer.py`

Expected: all tests pass, including the original trigger tests.

- [ ] **Step 6: Review checkpoint**

Run: `git diff --check -- scripts/teleop/quest_bridge.py scripts/teleop/robot_camera_stream.py scripts/teleop/robot_camera_vuer.py tests/teleop`

Do not commit; the researcher owns commits.

## Task 6: Own the SSH camera lifecycle from a testable manager

**Files:**

- Create: `scripts/teleop/run_r1_camera_transport.py`
- Create: `tests/teleop/test_r1_camera_transport.py`
- Modify: `scripts/teleop/run_r1_quest3_hardware.sh`
- Create: `tests/teleop/test_r1_hardware_camera_wiring.py`

**Interfaces:**

```python
def assert_local_port_free(host: str, port: int) -> None: ...
def build_ssh_command(config: CameraTransportConfig) -> list[str]: ...
def wait_until_ready(url: str, process: Popen, timeout_s: float) -> dict[str, object]: ...
def main(argv: Sequence[str] | None = None) -> int: ...
```

- [ ] **Step 1: Write failing manager tests**

Assert the SSH command contains `BatchMode=yes`, `ExitOnForwardFailure=yes`, `ServerAliveInterval`, `-L 127.0.0.1:<local>:127.0.0.1:<remote>`, and the foreground remote module with `PYTHONPATH=/home/unitree/HB/teleop/src`. Assert paths/arguments are safely passed as separate argv and remote shell values are `shlex.quote`-escaped.

Test occupied local port fails without terminating the owner; pre-ready SSH exit propagates its exit code; health `ready:false` keeps waiting; fresh `ready:true` atomically writes a ready JSON; timeout terminates only the child process; SIGTERM terminates the child and returns deterministically.

- [ ] **Step 2: Run manager tests and verify RED**

Run: `pytest -q tests/teleop/test_r1_camera_transport.py`

Expected: import failure because the manager does not exist.

- [ ] **Step 3: Implement manager with dependency injection**

Use `subprocess.Popen` without `shell=True`, a bounded `urllib` health probe, and atomic ready-file replacement. Keep the remote camera gateway in the SSH foreground. Do not use `pkill`, `fuser -k`, or any broad cleanup command.

- [ ] **Step 4: Write failing hardware-launcher wiring tests**

Verify the launcher defaults `HB_ROBOT_CAMERA=1`, starts the manager and waits for its ready file before the Quest/control pipeline, passes `--robot-camera-preview-url`, records the manager PID, terminates it during cleanup, and records a separate camera exit status. With `HB_ROBOT_CAMERA=0`, verify none of those camera arguments/processes are present. Preserve explicit external WebRTC compatibility.

- [ ] **Step 5: Modify the hardware launcher**

Use defaults:

```bash
HB_ROBOT_CAMERA="${HB_ROBOT_CAMERA:-1}"
ROBOT_CAMERA_LOCAL_PORT="${ROBOT_CAMERA_LOCAL_PORT:-8765}"
ROBOT_CAMERA_REMOTE_PORT="${ROBOT_CAMERA_REMOTE_PORT:-8765}"
ROBOT_CAMERA_READY_TIMEOUT_S="${ROBOT_CAMERA_READY_TIMEOUT_S:-20}"
```

Create a unique remote recording directory derived from `RUN_ID`; start the manager before Isaac/Quest; fail closed before the command pipeline when requested camera startup fails. Add the local preview URL to `CAMERA_ARGS`. During teardown, wait for the manager, copy the remote camera directory into `$RUN_DIR/robot_camera_original`, and validate `summary.json`, manifest/file count, SHA-256 values, and zero queue drops. Mark the run incomplete in status JSON on any mismatch rather than deleting evidence.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run: `pytest -q tests/teleop/test_r1_camera_transport.py tests/teleop/test_r1_hardware_camera_wiring.py tests/teleop/test_r1_upstream_pilot_wiring.py`

Expected: camera lifecycle tests and existing launcher tests pass.

- [ ] **Step 7: Review checkpoint**

Run: `bash -n scripts/teleop/run_r1_quest3_hardware.sh`

Run: `git diff --check -- scripts/teleop/run_r1_camera_transport.py scripts/teleop/run_r1_quest3_hardware.sh tests/teleop/test_r1_camera_transport.py tests/teleop/test_r1_hardware_camera_wiring.py`

Do not commit; the researcher owns commits.

## Task 7: Update operator documentation and usage surfaces

**Files:**

- Modify: `docs/teleop/13_vuer_robot_camera.md`
- Modify: `scripts/teleop/README.md`
- Modify: `Makefile`
- Modify: `hardware/teleop/src/SOURCE.txt` only if the final module list is documented there

- [ ] **Step 1: Write documentation checks where existing tests cover CLI/help**

Extend the nearest launcher/help test to require `HB_ROBOT_CAMERA=0`, right-A toggle wording, full-field view, freshness fallback, local recording location, and the fact that no manual camera IP/port is needed beyond the normal `ROBOT` SSH target.

- [ ] **Step 2: Run the documentation/help test and verify RED**

Run the focused test identified in Step 1.

Expected: new usage text is missing.

- [ ] **Step 3: Document exact workflows**

Document:

```bash
# Built-in R1 camera, recording, sim mirror and hardware (default)
make teleop-hardware ROBOT=100.82.165.36

# Explicit camera opt-out
HB_ROBOT_CAMERA=0 make teleop-hardware ROBOT=100.82.165.36
```

State: start in Quest passthrough; right A toggles full-field robot view; right trigger is deadman; left trigger is recalibration; stale video falls back to passthrough; originals live in the immutable run bundle; LAN SSH is preferred and a DERP-relayed Tailscale result is not field acceptance evidence.

- [ ] **Step 4: Run focused checks and verify GREEN**

Run the selected documentation/help test and `make help`.

- [ ] **Step 5: Review checkpoint**

Run: `git diff --check -- docs/teleop/13_vuer_robot_camera.md scripts/teleop/README.md Makefile hardware/teleop/src/SOURCE.txt`

Do not commit; the researcher owns commits.

## Task 8: Verify locally, then smoke-test the read-only camera path on R1

**Files:**

- Modify only if evidence conventions require it: `docs/teleop/13_vuer_robot_camera.md`
- Generated local evidence: `/tmp/hb_r1_camera_gateway_smoke_<timestamp>/`
- Generated run evidence during operator test: `results/smoke/<run-id>/`

- [ ] **Step 1: Run the complete local suite**

Run: `pytest -q`

Expected: all repository tests pass, with only already-declared skips.

- [ ] **Step 2: Run syntax/import checks in the actual environments**

Run the gateway tests/import in the robot-compatible environment and the subscriber/Vuer tests/import in `conda run -n tv`. Confirm OpenCV/NumPy types and CLI help in both locations.

- [ ] **Step 3: Deploy only the read-only module and open the tunnel smoke**

Use the existing teleop deployment workflow to place the new package on the verified robot. Start `run_r1_camera_transport.py` without the hardware command pipeline, wait for ready, fetch at least 100 previews, and save one decoded frame plus health/stats. Do not start or restart a motor service for this smoke.

- [ ] **Step 4: Verify smoke evidence**

Require at least 10 FPS over the sample window, valid/decodeable 640×360 JPEGs, no source/receive sample older than 500 ms, no recording drop/disk error, and a manifest whose hashes match the copied originals. Terminate SSH and confirm robot loopback port 8765 is no longer listening.

- [ ] **Step 5: Force a camera interruption without actuation**

Terminate the foreground camera SSH child during a display-only bridge smoke. Verify the bridge removes the background within 1.0 s, keeps the Vuer/controller session alive, and logs `camera_runtime_fallback`. Confirm no hardware command pipeline was opened.

- [ ] **Step 6: Apply completion verification**

Invoke `superpowers:verification-before-completion`. Re-run the focused tests, full suite, `bash -n`, repository path search for `Happy-Baby-R1-1`, and `git diff --check`. Inspect `git status --short` so unrelated user changes are not presented as ours.

- [ ] **Step 7: Hand off the suspended-hardware Quest test**

Report `READY FOR QUEST TEST` only after Steps 1–6 pass. Give the exact `make teleop-hardware ROBOT=...` command and a short checklist: Quest starts in passthrough; A enables full-field robot video; held A does not repeat; next A returns to passthrough; right-trigger release pauses but does not disconnect; left-trigger recalibrates; forced camera loss returns to passthrough; sim and robot original recordings are present.

The 10-minute intended-network performance gate and photon-to-Quest optical latency test require the operator and are not claimed complete by the display-only smoke.
