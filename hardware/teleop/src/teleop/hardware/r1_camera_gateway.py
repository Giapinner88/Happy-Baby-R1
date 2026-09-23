"""Read-only R1 videohub capture and preview primitives.

Hardware initialization is deliberately deferred to the command-line entry
point. Importing this module only defines data and image-processing helpers.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    sequence: int
    received_monotonic_s: float
    original_jpeg: bytes
    preview_jpeg: bytes
    source_width: int
    source_height: int


@dataclass
class GatewayCounters:
    successful_frame_count: int = 0
    rpc_failures_by_code: dict[int, int] = field(default_factory=dict)
    rpc_exception_count: int = 0
    invalid_jpeg_count: int = 0
    preview_encode_failure_count: int = 0
    recording_submit_failure_count: int = 0

    def as_dict(self) -> dict[str, object]:
        return {
            "successful_frame_count": self.successful_frame_count,
            "rpc_failures_by_code": dict(self.rpc_failures_by_code),
            "rpc_exception_count": self.rpc_exception_count,
            "invalid_jpeg_count": self.invalid_jpeg_count,
            "preview_encode_failure_count": self.preview_encode_failure_count,
            "recording_submit_failure_count": self.recording_submit_failure_count,
        }


class LatestFrameMailbox:
    """A single replaceable slot; slow consumers never create a backlog."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._frame: CameraFrame | None = None

    def put(self, frame: CameraFrame) -> None:
        with self._lock:
            self._frame = frame

    def snapshot(self) -> CameraFrame | None:
        with self._lock:
            return self._frame


@dataclass(frozen=True)
class RecorderSnapshot:
    accepted_frame_count: int
    written_frame_count: int
    queue_drop_count: int
    write_error_count: int
    last_write_error: str | None

    def as_dict(self) -> dict[str, object]:
        return {
            "accepted_frame_count": self.accepted_frame_count,
            "written_frame_count": self.written_frame_count,
            "queue_drop_count": self.queue_drop_count,
            "write_error_count": self.write_error_count,
            "last_write_error": self.last_write_error,
        }


class OriginalJpegRecorder:
    """Write original JPEGs on a bounded worker queue without recompression."""

    _STOP = object()

    def __init__(
        self,
        output_dir: Path,
        *,
        queue_capacity: int = 64,
        writer: Callable[[Path, bytes], None] | None = None,
    ) -> None:
        if queue_capacity <= 0:
            raise ValueError("queue_capacity must be positive")
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self._frames_dir = self.output_dir / "frames"
        self._frames_dir.mkdir()
        self._manifest = (self.output_dir / "manifest.jsonl").open("x", encoding="utf-8")
        self._queue: queue.Queue[CameraFrame | object] = queue.Queue(maxsize=queue_capacity)
        self._writer = writer if writer is not None else self._write_bytes
        self._lock = threading.Lock()
        self._accepted_frame_count = 0
        self._written_frame_count = 0
        self._queue_drop_count = 0
        self._write_error_count = 0
        self._last_write_error: str | None = None
        self._closed = False
        self._summary: dict[str, object] | None = None
        self._thread = threading.Thread(
            target=self._worker,
            name="r1-camera-recorder",
            daemon=True,
        )
        self._thread.start()

    @staticmethod
    def _write_bytes(path: Path, payload: bytes) -> None:
        path.write_bytes(payload)

    def submit(self, frame: CameraFrame) -> bool:
        with self._lock:
            if self._closed:
                return False
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                self._queue_drop_count += 1
                return False
            self._accepted_frame_count += 1
            return True

    def snapshot(self) -> RecorderSnapshot:
        with self._lock:
            return RecorderSnapshot(
                accepted_frame_count=self._accepted_frame_count,
                written_frame_count=self._written_frame_count,
                queue_drop_count=self._queue_drop_count,
                write_error_count=self._write_error_count,
                last_write_error=self._last_write_error,
            )

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                assert isinstance(item, CameraFrame)
                relative_path = Path("frames") / f"frame_{item.sequence:08d}.jpg"
                image_path = self.output_dir / relative_path
                self._writer(image_path, item.original_jpeg)
                record = {
                    "sequence": item.sequence,
                    "received_monotonic_s": item.received_monotonic_s,
                    "bytes": len(item.original_jpeg),
                    "source_width": item.source_width,
                    "source_height": item.source_height,
                    "sha256": hashlib.sha256(item.original_jpeg).hexdigest(),
                    "file": relative_path.as_posix(),
                }
                self._manifest.write(json.dumps(record, sort_keys=True) + "\n")
                self._manifest.flush()
                with self._lock:
                    self._written_frame_count += 1
            except Exception as exc:  # noqa: BLE001 - preserve preview on disk failure
                with self._lock:
                    self._write_error_count += 1
                    self._last_write_error = f"{type(exc).__name__}: {exc}"
            finally:
                self._queue.task_done()

    def close(self) -> dict[str, object]:
        with self._lock:
            if self._closed:
                assert self._summary is not None
                return dict(self._summary)
            self._closed = True
        self._queue.put(self._STOP)
        self._thread.join()
        self._manifest.close()
        summary = self.snapshot().as_dict()
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        with self._lock:
            self._summary = dict(summary)
        return summary


def is_complete_jpeg(payload: bytes) -> bool:
    """Return true only when the byte string has JPEG start/end markers."""

    return payload.startswith(b"\xff\xd8") and payload.rstrip(b"\x00").endswith(b"\xff\xd9")


def encode_preview(
    payload: bytes,
    *,
    width: int = 640,
    height: int = 360,
    quality: int = 60,
) -> tuple[bytes, int, int]:
    """Decode one source JPEG and encode a fixed-size preview once."""

    if width <= 0 or height <= 0:
        raise ValueError("preview dimensions must be positive")
    if not 1 <= quality <= 100:
        raise ValueError("preview quality must be in [1, 100]")
    if not is_complete_jpeg(payload):
        raise ValueError("source is not a complete JPEG")

    source = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if source is None or source.ndim != 3 or source.shape[2] != 3:
        raise ValueError("source JPEG could not be decoded as BGR")
    source_height, source_width = source.shape[:2]
    interpolation = cv2.INTER_AREA if width <= source_width and height <= source_height else cv2.INTER_LINEAR
    resized = cv2.resize(source, (width, height), interpolation=interpolation)
    ok, encoded = cv2.imencode(
        ".jpg",
        resized,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise ValueError("preview JPEG encoding failed")
    preview = encoded.tobytes()
    if not is_complete_jpeg(preview):
        raise ValueError("preview encoder returned an incomplete JPEG")
    return preview, int(source_width), int(source_height)


class VideohubPuller:
    """Pull one videohub sample at a time into a latest-frame mailbox."""

    def __init__(
        self,
        client: Any,
        mailbox: LatestFrameMailbox,
        *,
        preview_encoder: Callable[[bytes], tuple[bytes, int, int]] = encode_preview,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._client = client
        self._mailbox = mailbox
        self._preview_encoder = preview_encoder
        self._clock = clock
        self._sequence = 0
        self.counters = GatewayCounters()

    def run_once(self) -> bool:
        try:
            code, data = self._client.GetImageSample()
        except Exception:  # noqa: BLE001 - the foreground loop reports counters
            with self._mailbox._lock:
                self.counters.rpc_exception_count += 1
            return False

        code = int(code)
        if code != 0:
            with self._mailbox._lock:
                failures = self.counters.rpc_failures_by_code
                failures[code] = failures.get(code, 0) + 1
            return False

        payload = bytes(data) if data is not None else b""
        if not is_complete_jpeg(payload):
            with self._mailbox._lock:
                self.counters.invalid_jpeg_count += 1
            return False

        try:
            preview, source_width, source_height = self._preview_encoder(payload)
        except Exception:  # noqa: BLE001 - malformed images must not end capture
            with self._mailbox._lock:
                self.counters.preview_encode_failure_count += 1
            return False

        received_monotonic_s = float(self._clock())
        with self._mailbox._lock:
            self._sequence += 1
            self._mailbox._frame = CameraFrame(
                sequence=self._sequence,
                received_monotonic_s=received_monotonic_s,
                original_jpeg=payload,
                preview_jpeg=bytes(preview),
                source_width=int(source_width),
                source_height=int(source_height),
            )
            self.counters.successful_frame_count += 1
        return True

    def counter_snapshot(self) -> dict[str, object]:
        with self._mailbox._lock:
            return self.counters.as_dict()


def _is_loopback_host(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def build_http_server(
    mailbox: LatestFrameMailbox,
    counters: GatewayCounters,
    *,
    host: str,
    port: int,
    clock: Callable[[], float] = time.monotonic,
    recorder: OriginalJpegRecorder | None = None,
) -> ThreadingHTTPServer:
    """Build a loopback-only server that exposes the latest prepared bytes."""

    if not _is_loopback_host(host):
        raise ValueError("camera HTTP host must be loopback")
    if not 0 <= port <= 65535:
        raise ValueError("camera HTTP port must be in [0, 65535]")

    class CameraRequestHandler(BaseHTTPRequestHandler):
        def _send_bytes(self, status: int, content_type: str, payload: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            if self.path == "/health":
                with mailbox._lock:
                    frame = mailbox._frame
                    health = counters.as_dict()
                age_ms = None
                if frame is not None:
                    age_ms = max(0.0, float(clock()) - frame.received_monotonic_s) * 1000.0
                health.update(
                    {
                        "ready": frame is not None,
                        "latest_sequence": frame.sequence if frame is not None else None,
                        "latest_frame_age_ms": round(age_ms, 3) if age_ms is not None else None,
                    }
                )
                if recorder is not None:
                    health["recording"] = recorder.snapshot().as_dict()
                    health["record_dir"] = str(recorder.output_dir.resolve())
                payload = (json.dumps(health, sort_keys=True) + "\n").encode("utf-8")
                self._send_bytes(200, "application/json", payload)
                return
            if self.path == "/preview.jpg":
                frame = mailbox.snapshot()
                if frame is None:
                    self._send_bytes(503, "application/json", b'{"ready": false}\n')
                    return
                age_ms = max(0.0, float(clock()) - frame.received_monotonic_s) * 1000.0
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame.preview_jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-R1-Sequence", str(frame.sequence))
                self.send_header("X-R1-Source-Age-Ms", f"{age_ms:.3f}")
                self.send_header("X-R1-Source-Width", str(frame.source_width))
                self.send_header("X-R1-Source-Height", str(frame.source_height))
                self.end_headers()
                self.wfile.write(frame.preview_jpeg)
                return
            self.send_error(404)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    return ThreadingHTTPServer((host, port), CameraRequestHandler)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--record-dir", type=Path, required=True)
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--rpc-timeout-s", type=float, default=0.1)
    parser.add_argument("--capture-hz", type=float, default=15.0)
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-height", type=int, default=360)
    parser.add_argument("--preview-quality", type=int, default=60)
    parser.add_argument("--record-queue-capacity", type=int, default=64)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.interface.strip():
        parser.error("--interface must not be empty")
    if args.domain < 0:
        parser.error("--domain must be non-negative")
    if not math.isfinite(args.rpc_timeout_s) or args.rpc_timeout_s <= 0:
        parser.error("--rpc-timeout-s must be finite and positive")
    if not math.isfinite(args.capture_hz) or not 0 < args.capture_hz <= 15:
        parser.error("--capture-hz must be finite and in (0, 15]")
    if args.preview_width <= 0 or args.preview_height <= 0:
        parser.error("preview dimensions must be positive")
    if not 1 <= args.preview_quality <= 100:
        parser.error("--preview-quality must be in [1, 100]")
    if args.record_queue_capacity <= 0:
        parser.error("--record-queue-capacity must be positive")
    if not _is_loopback_host(args.host):
        parser.error("--host must be a loopback address")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    return args


def _create_videohub_client(*, domain: int, interface: str, timeout_s: float) -> Any:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    ChannelFactoryInitialize(domain, interface)
    client = VideoClient()
    client.SetTimeout(timeout_s)
    client.Init()
    return client


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        recorder = OriginalJpegRecorder(
            args.record_dir,
            queue_capacity=args.record_queue_capacity,
        )
    except FileExistsError:
        print(f"Refusing to overwrite recording directory: {args.record_dir}", file=sys.stderr)
        return 2

    mailbox = LatestFrameMailbox()
    stop_event = threading.Event()
    server: ThreadingHTTPServer | None = None
    server_thread: threading.Thread | None = None
    previous_handlers: dict[int, Any] = {}
    exit_code = 0
    try:
        client = _create_videohub_client(
            domain=args.domain,
            interface=args.interface,
            timeout_s=args.rpc_timeout_s,
        )
        preview_encoder = lambda payload: encode_preview(
            payload,
            width=args.preview_width,
            height=args.preview_height,
            quality=args.preview_quality,
        )
        puller = VideohubPuller(
            client,
            mailbox,
            preview_encoder=preview_encoder,
        )
        server = build_http_server(
            mailbox,
            puller.counters,
            host=args.host,
            port=args.port,
            recorder=recorder,
        )
        server_thread = threading.Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="r1-camera-http",
            daemon=True,
        )
        server_thread.start()

        def request_stop(_signum: int, _frame: object) -> None:
            stop_event.set()

        if threading.current_thread() is threading.main_thread():
            stop_signals = [signal.SIGINT, signal.SIGTERM]
            if hasattr(signal, "SIGHUP"):
                stop_signals.append(signal.SIGHUP)
            for signum in stop_signals:
                previous_handlers[signum] = signal.signal(signum, request_stop)

        period_s = 1.0 / args.capture_hz
        while not stop_event.is_set():
            started = time.monotonic()
            if puller.run_once():
                frame = mailbox.snapshot()
                assert frame is not None
                if not recorder.submit(frame):
                    with mailbox._lock:
                        puller.counters.recording_submit_failure_count += 1
            remaining = period_s - (time.monotonic() - started)
            if remaining > 0:
                stop_event.wait(remaining)
    except Exception as exc:  # noqa: BLE001 - foreground CLI must report startup/runtime cause
        print(f"[FAIL] R1 camera gateway: {type(exc).__name__}: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        stop_event.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=2.0)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        summary = recorder.close()
        (recorder.output_dir / "gateway_summary.json").write_text(
            json.dumps(
                {
                    "gateway": puller.counter_snapshot() if "puller" in locals() else None,
                    "recording": summary,
                    "exit_code": exit_code,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
