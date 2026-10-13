"""Quest camera scene and loopback signaling; no pixels enter the control socket."""
from __future__ import annotations
import asyncio
import json
import math
from pathlib import Path
from types import SimpleNamespace
from quest_tracking import TrackingGate
from multiprocessing import Array, Value
import threading
import time
from urllib.request import Request, urlopen


def raw_trigger(state):
    """Inspect WebXR's original [0,1] trigger before the vendor's inversion."""
    state = state if isinstance(state, dict) else {}
    raw = state.get("triggerValue")
    try:
        value = float(raw)
        valid = not isinstance(raw, bool) and math.isfinite(value) and 0 <= value <= 1
    except (TypeError, ValueError):
        value, valid = 0.0, False
    digital = state.get("trigger")
    digital_valid = type(digital) is bool
    return {"digital_present": "trigger" in state, "digital_valid": digital_valid,
            "digital_pressed": digital if digital_valid else False,
            "analog_present": "triggerValue" in state, "analog_valid": valid,
            "analog_value": value if valid else None}


class WebRTCCameraClient:
    """Poll media health off the control thread. Commands never perform DDS work."""
    def __init__(self, config):
        self.url = f"http://127.0.0.1:{config.media_port}"
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.desired = False
        self.status = {}
        self.received = 0.0
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, name="webrtc-health", daemon=True)
        self.thread.start()

    def close(self):
        self.set_capture_requested(False)
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1)
        # Best effort shutdown is outside the control loop.
        try:
            self.request("/view", {"enabled": False})
        except Exception:
            pass

    def request(self, path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        req = Request(self.url + path, data=data, headers={"Content-Type": "application/json"})
        with urlopen(req, timeout=0.4) as response:
            return json.load(response)

    def set_capture_requested(self, requested):
        with self.lock:
            self.desired = bool(requested)
            self.status = {}
            self.received = 0.0

    def snapshot(self):
        with self.lock:
            status = dict(self.status)
            received = self.received
        age = status.get("frame_age_s")
        age = math.inf if age is None else float(age) + max(0, time.monotonic() - received)
        fresh = status.get("enabled") and received and time.monotonic() - received < 0.8 and math.isfinite(age)
        return fresh, int(status.get("frame_sequence", -1)), age, status.get("error")

    def peer_status(self):
        with self.lock:
            return dict(self.status)

    def run(self):
        applied = None
        while not self.stop.is_set():
            with self.lock:
                desired = self.desired
            try:
                if desired != applied:
                    self.request("/view", {"enabled": desired})
                    applied = desired
                status = self.request("/health")
                with self.lock:
                    # A result from the preceding view must never enable the new view.
                    if self.desired == desired:
                        self.status, self.received = status, time.monotonic()
            except Exception as exc:
                with self.lock:
                    self.status = {"error": f"{type(exc).__name__}: {exc}"}
                    self.received = 0.0
                applied = None
            self.stop.wait(0.1)


def build_camera_wrapper(tv_wrapper_module, config, **kwargs):
    vendor = tv_wrapper_module.TeleVuer
    from vuer.schemas import MotionControllers, WebRTCVideoPlane

    class CameraTeleVuer(vendor):
        _BACKGROUND_KEY = "hb-robot-camera-background"

        def __init__(self, *args, **vendor_kwargs):
            self.client_status = Array("c", 8192)
            # Server gate transitions must not erase the browser's button,
            # pose and focus evidence at precisely the moment of a STOP.
            self.server_status = Array("c", 2048)
            self.tracking = Array("d", [0.0] * 6)  # seen, received, valid, rearm, sequence, raw right trigger
            self.arm_tracking = Array("b", [False, False])
            self.camera_enabled = Value("b", False)
            # count, received time, previous gap, max gap; per-side raw fields.
            self.raw_controller = Array("d", [0.0] * 18)
            self.raw_head = Array("d", [0.0] * 3)
            super().__init__(*args, **vendor_kwargs)

        def set_robot_camera_enabled(self, enabled):
            self.camera_enabled.value = bool(enabled)

        async def on_controller_move(self, event, session, fps=60):
            if not getattr(event, "hb_snapshot", False):
                return
            now = time.monotonic()
            value = event.value if isinstance(event.value, dict) else {}
            with self.raw_controller.get_lock():
                previous = self.raw_controller[1]
                gap = now - previous if previous else 0.0
                self.raw_controller[0] += 1
                self.raw_controller[1:4] = [now, gap, max(gap, self.raw_controller[3])]
                for index, side in enumerate(("left", "right")):
                    raw = raw_trigger(value.get(side + "State"))
                    self.raw_controller[4 + index * 7:11 + index * 7] = [
                        raw["digital_present"], raw["digital_valid"], raw["digital_pressed"],
                        raw["analog_present"], raw["analog_valid"], raw["analog_value"] or 0,
                        bool(raw["digital_valid"] or raw["analog_valid"])]
            # The pinned vendor API requires two matrices. Its inactive matrix
            # is only a compatibility placeholder, explicitly excluded from IK.
            vendor_value = dict(value)
            mask = value.get("arm_tracking", {"left": True, "right": True})
            for side in ("left", "right"):
                if not mask[side]:
                    vendor_value[side] = list(getattr(self, side + "_arm_pose_shared"))
            await super().on_controller_move(SimpleNamespace(value=vendor_value), session, fps)
            # Missing/invalid input must release instead of reusing old trigger values.
            for side in ("left", "right"):
                raw = raw_trigger(value.get(side + "State"))
                getattr(self, side + "_ctrl_trigger_shared").value = bool(raw["digital_pressed"])
                getattr(self, side + "_ctrl_triggerValue_shared").value = raw["analog_value"] if raw["analog_valid"] else 0.0

        async def on_cam_move(self, event, session, fps=60):
            if not getattr(event, "hb_snapshot", False):
                return
            now = time.monotonic()
            with self.raw_head.get_lock():
                self.raw_head[0] += 1
                self.raw_head[1] = now
                self.raw_head[2] = bool(event.value.get("camera", {}).get("matrix"))
            await super().on_cam_move(event, session, fps)

        def release_tracking(self):
            for side in ("left", "right"):
                getattr(self, side + "_ctrl_trigger_shared").value = False
                getattr(self, side + "_ctrl_triggerValue_shared").value = 0.0
                getattr(self, side + "_ctrl_aButton_shared").value = False

        def input_diagnostics(self):
            now = time.monotonic()
            with self.client_status.get_lock():
                browser_status = json.loads(self.client_status.value.decode() or "{}")
            with self.raw_controller.get_lock():
                values = list(self.raw_controller[:])
            with self.raw_head.get_lock():
                head = list(self.raw_head[:])
            result = {"tracking_protocol": "xr-frame-v3", "browser_status": browser_status,
                      "arm_tracking": dict(zip(("left", "right"), map(bool, self.arm_tracking[:]))),
                      "controller_events": int(values[0]), "controller_age_s": now - values[1] if values[1] else None,
                      "controller_last_gap_s": values[2], "controller_max_gap_s": values[3],
                      "tracking_frames_seen": bool(self.tracking[0]),
                      "tracking_age_s": now - self.tracking[1] if self.tracking[0] else None,
                      "tracking_valid": bool(self.tracking[2]),
                      "tracking_requires_release": bool(self.tracking[3]),
                      "tracking_sequence": int(self.tracking[4]),
                      "tracking_original_right_trigger_value": self.tracking[5],
                      "head_events": int(head[0]), "head_age_s": now - head[1] if head[1] else None}
            with self.server_status.get_lock():
                result['tracking_server_status'] = json.loads(self.server_status.value.decode() or '{}')
            for index, side in enumerate(("left", "right")):
                p = values[4 + index * 7:11 + index * 7]
                result[side + "_raw"] = {
                    "digital_present": bool(p[0]), "digital_valid": bool(p[1]), "digital_pressed": bool(p[2]),
                    "analog_present": bool(p[3]), "analog_valid": bool(p[4]),
                    "analog_value": p[5] if p[4] else None, "input_valid": bool(p[6])}
            return result

        def _vuer_run(self):
            from aiohttp import ClientSession, ClientTimeout, web

            async def proxy(request):
                path = "/offer" if request.path.endswith("offer") else "/health"
                try:
                    async with ClientSession(timeout=ClientTimeout(total=6)) as client:
                        body = await request.read() if request.method == "POST" else None
                        async with client.request(request.method, f"http://127.0.0.1:{config.media_port}{path}", data=body,
                                                  headers={"Content-Type": "application/json"}) as response:
                            return web.Response(status=response.status, body=await response.read(), content_type="application/json")
                except (TimeoutError, OSError) as exc:
                    return web.json_response({"error": str(exc)}, status=503)

            if config is not None:
                self.vuer._add_route("/camera/offer", proxy, method="POST")
                self.vuer._add_route("/camera/health", proxy, method="GET")
            original_index = self.vuer.socket_index

            async def index(request):
                if request.headers.get("Upgrade", "").lower() == "websocket":
                    return await original_index(request)
                html = (Path(self.vuer.client_root) / "index.html").read_text()
                bootstrap = Path(__file__).with_name("quest_tracking.js").read_text()
                html = html.replace("<head>", "<head><script>" + bootstrap + "</script>", 1)
                return web.Response(text=html, content_type="text/html",
                                    headers={"Cache-Control": "no-store"})

            async def script(request):
                return web.Response(text=Path(__file__).with_name("quest_tracking.js").read_text(),
                                    content_type="application/javascript",
                                    headers={"Cache-Control": "no-store"})

            def record_status(value, *, server=False):
                value = {**value, "received_monotonic_s": time.monotonic()}
                raw = json.dumps(value, ensure_ascii=False).encode()
                buffer = self.server_status if server else self.client_status
                if len(raw) < len(buffer):
                    with buffer.get_lock():
                        buffer.value = raw

            async def diagnostic(request):
                try:
                    value = await request.json()
                    if not isinstance(value, dict) or len(json.dumps(value)) > 4096:
                        raise ValueError("invalid diagnostic")
                    record_status(value)
                    return web.json_response({"accepted": True})
                except (ValueError, TypeError):
                    return web.json_response({"error": "invalid diagnostic"}, status=400)

            async def tracking_status(request):
                return web.json_response(self.input_diagnostics())

            owner_lock = asyncio.Lock()

            async def tracking_socket(request):
                ws = web.WebSocketResponse(max_msg_size=8192, heartbeat=2, timeout=0.25)
                async with owner_lock:
                    current = getattr(self, "tracking_socket", None)
                    if current is not None:
                        last_seen = max(self.tracking[1], self.tracking_connected_at)
                        if time.monotonic() - last_seen <= 0.5:
                            raise web.HTTPConflict(text="Quest operator already connected")
                    await ws.prepare(request)
                    # Reserve ownership before closing a stale socket: its finally
                    # block cannot release the new operator, even during awaits.
                    self.tracking_socket = ws
                    self.tracking_connected_at = time.monotonic()
                    with self.tracking.get_lock():
                        self.tracking[2] = 0
                        self.tracking[3] = True
                        self.release_tracking()
                if current is not None:
                    await current.close(code=1000, message=b"stale control lease replaced")
                gate = TrackingGate()
                last_gate_report = None
                client = request.query.get("client", "unknown")[:128]
                record_status({"stage": "socket_connected", "client": client,
                               "version": request.query.get("version", "unknown")[:64]}, server=True)
                try:
                    async for message in ws:
                        if self.tracking_socket is not ws:
                            break
                        if message.type != web.WSMsgType.TEXT:
                            break
                        try:
                            value = json.loads(message.data)
                        except (ValueError, TypeError):
                            value = {}
                        with self.tracking.get_lock():
                            normalized = gate.update(value, time.monotonic())
                            valid = normalized is not None
                            original_trigger = value["rightState"]["triggerValue"] if valid else 0
                            self.tracking[:] = [1, time.monotonic(), valid,
                                               gate.release_required, gate.last_sequence, original_trigger]
                            self.arm_tracking[:] = [bool(normalized["arm_tracking"][side]) if valid else False for side in ("left", "right")]
                            if valid:
                                event = SimpleNamespace(value=normalized, hb_snapshot=True)
                                await self.on_cam_move(event, None)
                                await self.on_controller_move(event, None)
                            else:
                                self.release_tracking()
                        gate_report = (valid, gate.release_required, gate.reason, tuple(self.arm_tracking[:]))
                        if gate_report != last_gate_report:
                            await ws.send_json({"type": "tracking_gate", "valid": valid,
                                                "requires_release": gate.release_required,
                                                "reason": gate.reason,
                                                "arm_tracking": dict(zip(("left", "right"), map(bool, self.arm_tracking[:])))})
                            last_gate_report = gate_report
                        if not valid or gate.release_required:
                            record_status({"stage": "tracking_blocked", "reason": gate.reason, "client": client}, server=True)
                finally:
                    # An old connection must not release a newer operator.
                    if self.tracking_socket is ws:
                        with self.tracking.get_lock():
                            self.tracking[2] = 0
                            self.tracking[3] = True
                            self.release_tracking()
                        self.tracking_socket = None
                        record_status({"stage": "socket_closed", "client": client}, server=True)
                return ws

            self.vuer.socket_index = index
            self.vuer._add_route("/hb_tracking.js", script, method="GET")
            self.vuer._add_route("/tracking", tracking_socket, method="GET")
            self.vuer._add_route("/tracking/diagnostic", diagnostic, method="POST")
            self.vuer._add_route("/tracking/status", tracking_status, method="GET")
            super()._vuer_run()

        async def main_pass_through(self, session):
            session.upsert(MotionControllers(stream=True, key="motionControllers", left=True, right=True), to="bgChildren")
            visible = False
            try:
                while True:
                    enabled = bool(self.camera_enabled.value)
                    if enabled and not visible:
                        session.upsert(WebRTCVideoPlane(src="/camera/offer", key=self._BACKGROUND_KEY,
                            iceServer=None, webRTCOptions={"iceServers": []}, aspect=config.width / config.height,
                            height=1, distanceToCamera=1), to="bgChildren")
                        visible = True
                    elif visible and not enabled:
                        session.remove @ self._BACKGROUND_KEY
                        visible = False
                    await asyncio.sleep(1.0 / self.display_fps)
            except AssertionError as exc:
                if "Websocket session is missing" not in str(exc):
                    raise

    original = tv_wrapper_module.TeleVuer
    tv_wrapper_module.TeleVuer = CameraTeleVuer
    kwargs = dict(kwargs)
    if config is not None:
        kwargs.update(display_mode="pass-through", zmq=False, webrtc=False, binocular=False,
                      img_shape=(config.height, config.width))
    try:
        return tv_wrapper_module.TeleVuerWrapper(**kwargs)
    finally:
        tv_wrapper_module.TeleVuer = original
