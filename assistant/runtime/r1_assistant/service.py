"""One robot-local task at a time, observable completion and cancellation."""
from collections import OrderedDict
import copy
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time
import uuid

from .config import GESTURES, write


class Coordinator:
    def __init__(self, config, backends):
        self.config, self.backends = config, backends
        self.lock = threading.RLock()
        self.active = None
        self.results = OrderedDict()
        self.requests = OrderedDict()
        self.cancelled_requests = OrderedDict()
        self.provenance = {}
        if config.get("release_manifest"):
            self.provenance["release_manifest_sha256"] = hashlib.sha256(Path(config["release_manifest"]).read_bytes()).hexdigest()
            owner = Path(config["hb_root"]) / "build_metadata.json"
            self.provenance["owner_build"] = json.loads(owner.read_text())
        self.root = Path(config["runs_dir"])
        self.root.mkdir(parents=True, exist_ok=True)
        for path in sorted(self.root.glob("*/task.json"), key=lambda p: p.stat().st_mtime)[-128:]:
            old = json.loads(path.read_text())
            if old.get("request_id"):
                signature = (old["action"], old.get("query", ""), old.get("gesture_id"))
                reply = {"ok": False, "status": "previous_request", "task_id": old["task_id"],
                         "answer": "Yêu cầu này đã được nhận trước đó; không tự thực hiện lại."}
                self.requests[old["request_id"]] = signature, reply
                old["status"] = "interrupted" if old.get("status") in ("starting", "running") else old["status"]
                self.results[old["task_id"]] = old

    def _trim(self):
        for mapping, limit in ((self.requests, 128), (self.results, 64), (self.cancelled_requests, 128)):
            while len(mapping) > limit:
                mapping.popitem(last=False)

    def _persist(self, task):
        task["provenance"] = self.provenance
        task["reproduction"] = {"action": task["action"], "query": task.get("query"),
                                "gesture_id": task.get("gesture_id"),
                                "note": "Use a new request_id for an intentional rerun; never replay motion automatically"}
        write(self.root / task["task_id"] / "task.json", task)

    def snapshot(self, task_id=None):
        with self.lock:
            if task_id:
                task = self.results.get(task_id)
                if task is None:
                    return {"status": "unknown_task", "task_id": task_id, "ok": False}
            else:
                task = self.active["task"] if self.active else next(reversed(self.results.values()), None)
            if task is None:
                return {"status": "idle", "active": False, "mode": self.config["mode"]}
            result = copy.deepcopy(task)
            result["active"] = bool(self.active and self.active["task"]["task_id"] == task["task_id"])
            result["mode"] = self.config["mode"]
            return result

    def submit(self, payload):
        if not isinstance(payload, dict):
            return 400, {"status": "invalid_request"}
        action = payload.get("action")
        if action not in ("gesture", "find_and_point", "approach"):
            return 400, {"status": "unknown_action"}
        request_id = payload.get("request_id")
        if not isinstance(request_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", request_id):
            return 400, {"status": "invalid_request_id"}
        query = payload.get("query", "")
        if not isinstance(query, str):
            return 400, {"status": "invalid_query"}
        if action != "gesture" and (not isinstance(query, str) or not 1 <= len(query.strip()) <= 180):
            return 400, {"status": "invalid_query"}
        gesture = payload.get("gesture_id")
        if action == "gesture" and gesture not in GESTURES:
            return 400, {"status": "unknown_gesture"}
        signature = (action, query.strip(), gesture)
        with self.lock:
            if request_id in self.cancelled_requests:
                return 409, {"ok": False, "status": "cancelled_before_admission"}
            if request_id in self.requests:
                previous, reply = self.requests[request_id]
                if previous != signature:
                    return 409, {"status": "request_id_conflict", "ok": False}
                return 200, dict(reply, duplicate=True)
            if self.active:
                return 409, {"status": "busy", "ok": False,
                             "task_id": self.active["task"]["task_id"],
                             "answer": "Robot đang có tác vụ khác; hãy chờ hoặc yêu cầu dừng."}
            task = {"task_id": uuid.uuid4().hex, "request_id": request_id,
                    "action": action, "query": query.strip(), "gesture_id": gesture,
                    "status": "starting", "created_monotonic_s": time.monotonic(),
                    "physical_arrival_verified": False}
            record = {"task": task, "cancel": threading.Event(), "thread": None}
            self.active = record
            self.results[task["task_id"]] = task
            self._trim()
            self._persist(task)
            # Persist admission before dispatch. No task replay after a restart.
            write(self.root / task["task_id"] / "resolved_config.json", self.config)
            if action == "gesture":
                try:
                    owner = self.backends.start_gesture(task)
                except Exception as exc:
                    task.update(status="fault", reason=type(exc).__name__,
                                answer="Chưa xác nhận được trạng thái cử chỉ. Hãy yêu cầu dừng.")
                    self._persist(task)
                    return 503, dict(self.snapshot(), ok=False)
                if owner.get("status") == "shadow" or owner.get("ok") is not True:
                    task.update(status=owner.get("status", "rejected"), result=owner)
                    self.active = None
                    self._persist(task)
                    reply = dict(owner, task_id=task["task_id"])
                    self.requests[request_id] = signature, reply
                    self._trim()
                    return 200, reply
                if owner.get("status") != "accepted":
                    task.update(status="fault", result=owner)
                    self._persist(task)
                    return 503, dict(self.snapshot(), ok=False)
            task["status"] = "running"
            record["thread"] = threading.Thread(target=self._run, args=(record,), daemon=True)
            reply = {"ok": True, "status": "accepted", "task_id": task["task_id"],
                     "mode": self.config["mode"], "physical_arrival_verified": False,
                     "answer": "Đã nhận tác vụ." if self.config["mode"] == "execute" else "Đang quan sát; robot chưa chuyển động."}
            self.requests[request_id] = signature, reply
            while len(self.requests) > 128:
                self.requests.popitem(last=False)
            record["thread"].start()
            return 202, reply

    def _run(self, record):
        task, cancel = record["task"], record["cancel"]

        def progress(value):
            with self.lock:
                task["progress"] = copy.deepcopy(value)

        try:
            result = self.backends.run(task, cancel, progress)
            stop = result.get("stop") or {}
            if (stop or result.get("motion_requests_sent", 0)) and stop.get("stop_acknowledged") is not True:
                raise RuntimeError("motion stop not acknowledged")
            if stop.get("release_acknowledged") is False:
                raise RuntimeError("motion ownership release not acknowledged")
            with self.lock:
                task.update(status=result.get("status", "failed"), result=result,
                            finished_monotonic_s=time.monotonic())
                self._persist(task)
                if self.active is record:
                    self.active = None
                while len(self.results) > 64:
                    self.results.popitem(last=False)
        except BaseException as exc:
            cancel.set()
            try:
                stopped = self.backends.cancel(task)
            except Exception:
                stopped = {"status": "unconfirmed"}
            with self.lock:
                task.update(status="fault", reason=str(exc), stop=stopped,
                            answer="Tác vụ lỗi; chưa xác nhận đã dừng. Hãy dùng tay cầm và kiểm tra trạng thái.")
                # Retain ownership until recovery confirms idle; do not admit motion.
                self._persist(task)

    def cancel(self, task_id=None, request_id=None):
        with self.lock:
            record = self.active
            if request_id is not None:
                if not isinstance(request_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", request_id):
                    return 400, {"status": "invalid_request_id"}
                self.cancelled_requests[request_id] = True
                self._trim()
                if record is None or record["task"].get("request_id") != request_id:
                    return 200, {"status": "cancel_recorded", "answer": "Yêu cầu hủy đã được ghi nhận."}
            if record is None:
                return 200, {"status": "idle", "active": False}
            if task_id and task_id != record["task"]["task_id"]:
                return 409, {"status": "task_id_mismatch"}
            record["cancel"].set()
            task = record["task"]
            # Keep admission locked until the bounded cancel RPC returns. A new
            # task must not replace the backend that this cancellation targets.
            try:
                result = self.backends.cancel(task)
                thread = record["thread"]
                if (thread is None or not thread.is_alive()) and self.backends.recover(task):
                    task.update(status="cancelled", result=result)
                    self._persist(task)
                    if self.active is record:
                        self.active = None
            except Exception as exc:
                result = {"status": "stop_unconfirmed", "reason": type(exc).__name__}
        return 200, {"status": "cancel_requested", "task_id": task["task_id"],
                     "stop": result, "answer": "Đã yêu cầu dừng; đang chờ xác nhận."}

    def reserve_camera(self):
        with self.lock:
            if self.active:
                return 409, {"status": "busy", "answer": "Camera hoặc robot đang có tác vụ khác."}
            task = {"task_id": uuid.uuid4().hex, "action": "inspect_camera", "status": "running",
                    "created_monotonic_s": time.monotonic()}
            self.active = {"task": task, "cancel": threading.Event(), "thread": None}
            self.results[task["task_id"]] = task
            self._trim()
            return 200, {"status": "reserved", "task_id": task["task_id"]}

    def release_camera(self, task_id):
        with self.lock:
            if not self.active or self.active["task"].get("action") != "inspect_camera" or self.active["task"]["task_id"] != task_id:
                return 409, {"status": "task_id_mismatch"}
            self.active["task"]["status"] = "completed"
            self.active = None
            return 200, {"status": "released"}

    def tick(self):
        with self.lock:
            record = self.active
            if not record:
                return
            task = record["task"]
            age = time.monotonic() - task["created_monotonic_s"]
            if task["action"] == "inspect_camera":
                if age > self.config["camera_lease_s"]:
                    # A crashed voice process releases its flock. Never expire a live RPC.
                    with open(self.config["camera_lock"], "a") as lease:
                        try:
                            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        except BlockingIOError:
                            return
                        self.active = None
                        task["status"] = "expired"
                return
            expired = age > self.config["task_timeout_s"] and not record["cancel"].is_set()
        if expired:
            self.cancel(task["task_id"])

    def route(self, path, payload):
        if not isinstance(payload, dict):
            return 400, {"status": "invalid_request"}
        if path == "/tasks":
            return self.submit(payload)
        if path == "/status":
            return 200, self.snapshot(payload.get("task_id"))
        if path == "/cancel":
            if self.active and self.active["task"]["action"] == "inspect_camera":
                return 200, {"status": "camera_busy", "answer": "Đang chờ camera; không có chuyển động do tác vụ này."}
            return self.cancel(payload.get("task_id"), payload.get("request_id"))
        if path == "/camera/acquire":
            return self.reserve_camera()
        if path == "/camera/release":
            return self.release_camera(payload.get("task_id"))
        if path == "/gesture/status":
            return 200, self.backends.owner("status")
        if path == "/gesture/cancel":
            with self.lock:
                if not self.active or self.active["task"]["action"] != "gesture":
                    return 200, {"ok": True, "status": "idle"}
                task_id = self.active["task"]["task_id"]
            code, result = self.cancel(task_id)
            return code, dict(result, ok=True)
        return 404, {"status": "unknown_endpoint"}


def make_server(coordinator):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            try:
                self.connection.settimeout(2)
                size = int(self.headers.get("Content-Length", "0"))
                if self.headers.get_content_type() != "application/json" or not 0 < size <= 4096:
                    raise ValueError("JSON object required")
                raw = self.rfile.read(size)
                if len(raw) != size:
                    raise ValueError("incomplete body")
                code, result = coordinator.route(self.path, json.loads(raw))
            except (ValueError, TypeError, UnicodeError):
                code, result = 400, {"status": "invalid_request"}
            except Exception as exc:
                code, result = 503, {"status": "unavailable", "reason": type(exc).__name__}
            data = json.dumps(result, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer((coordinator.config["host"], coordinator.config["port"]), Handler)
    server.daemon_threads = True
    return server
