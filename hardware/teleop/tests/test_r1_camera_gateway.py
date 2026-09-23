from __future__ import annotations

import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np
import pytest


GATEWAY_PATH = Path(__file__).resolve().parents[1] / "src/teleop/hardware/r1_camera_gateway.py"


def _gateway():
    spec = importlib.util.spec_from_file_location("r1_camera_gateway_test", GATEWAY_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeVideoClient:
    def __init__(self, samples):
        self._samples = iter(samples)

    def GetImageSample(self):
        return next(self._samples)


class FakeClock:
    def __init__(self, *values: float):
        self._values = iter(values)

    def __call__(self) -> float:
        return next(self._values)


def test_complete_jpeg_requires_both_markers() -> None:
    gateway = _gateway()

    assert gateway.is_complete_jpeg(b"\xff\xd8payload\xff\xd9") is True
    assert gateway.is_complete_jpeg(b"\xff\xd8payload") is False
    assert gateway.is_complete_jpeg(b"payload\xff\xd9") is False
    assert gateway.is_complete_jpeg(b"") is False


def test_mailbox_replaces_the_previous_frame() -> None:
    gateway = _gateway()
    mailbox = gateway.LatestFrameMailbox()
    first = gateway.CameraFrame(1, 1.0, b"one", b"preview-one", 1280, 720)
    second = gateway.CameraFrame(2, 2.0, b"two", b"preview-two", 1280, 720)

    assert mailbox.snapshot() is None
    mailbox.put(first)
    mailbox.put(second)

    assert mailbox.snapshot() == second


def test_puller_accepts_latest_complete_jpeg_only() -> None:
    gateway = _gateway()
    jpeg_a = b"\xff\xd8original-a\xff\xd9"
    jpeg_b = b"\xff\xd8original-b\xff\xd9"
    client = FakeVideoClient(
        [(0, jpeg_a), (3104, b""), (0, b"\xff\xd8cut"), (0, jpeg_b)]
    )
    mailbox = gateway.LatestFrameMailbox()
    previews = {
        jpeg_a: (b"\xff\xd8preview-a\xff\xd9", 1280, 720),
        jpeg_b: (b"\xff\xd8preview-b\xff\xd9", 1280, 720),
    }
    puller = gateway.VideohubPuller(
        client,
        mailbox,
        preview_encoder=previews.__getitem__,
        clock=FakeClock(10.0, 11.0),
    )

    assert puller.run_once() is True
    assert puller.run_once() is False
    assert puller.run_once() is False
    assert puller.run_once() is True

    latest = mailbox.snapshot()
    assert latest is not None
    assert latest.sequence == 2
    assert latest.received_monotonic_s == 11.0
    assert latest.original_jpeg == jpeg_b
    assert latest.preview_jpeg == previews[jpeg_b][0]
    assert puller.counters.successful_frame_count == 2
    assert puller.counters.rpc_failures_by_code == {3104: 1}
    assert puller.counters.invalid_jpeg_count == 1


def test_puller_counts_preview_encode_failure_without_replacing_frame() -> None:
    gateway = _gateway()
    mailbox = gateway.LatestFrameMailbox()

    def fail_encode(_payload: bytes):
        raise ValueError("decode failed")

    puller = gateway.VideohubPuller(
        FakeVideoClient([(0, b"\xff\xd8complete\xff\xd9")]),
        mailbox,
        preview_encoder=fail_encode,
        clock=FakeClock(1.0),
    )

    assert puller.run_once() is False
    assert mailbox.snapshot() is None
    assert puller.counters.preview_encode_failure_count == 1


def test_encode_preview_returns_valid_requested_size_and_source_dimensions() -> None:
    gateway = _gateway()
    source = np.zeros((6, 8, 3), dtype=np.uint8)
    source[:, :, 1] = 200
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok

    preview, source_width, source_height = gateway.encode_preview(
        encoded.tobytes(), width=4, height=3, quality=60
    )
    decoded = cv2.imdecode(np.frombuffer(preview, dtype=np.uint8), cv2.IMREAD_COLOR)

    assert gateway.is_complete_jpeg(preview)
    assert (source_width, source_height) == (8, 6)
    assert decoded.shape == (3, 4, 3)


@pytest.mark.parametrize(
    ("width", "height", "quality"),
    [(0, 360, 60), (640, 0, 60), (640, 360, 0), (640, 360, 101)],
)
def test_encode_preview_rejects_invalid_dimensions_or_quality(
    width: int, height: int, quality: int
) -> None:
    gateway = _gateway()

    with pytest.raises(ValueError):
        gateway.encode_preview(
            b"\xff\xd8complete\xff\xd9", width=width, height=height, quality=quality
        )


def _frame(gateway, sequence: int, received: float = 10.0):
    return gateway.CameraFrame(
        sequence=sequence,
        received_monotonic_s=received,
        original_jpeg=f"\xff\xd8original-{sequence}\xff\xd9".encode("latin1"),
        preview_jpeg=f"\xff\xd8preview-{sequence}\xff\xd9".encode("latin1"),
        source_width=1280,
        source_height=720,
    )


def test_recorder_writes_original_bytes_manifest_and_summary(tmp_path: Path) -> None:
    gateway = _gateway()
    output_dir = tmp_path / "camera"
    frame = _frame(gateway, 7, received=12.5)
    recorder = gateway.OriginalJpegRecorder(output_dir, queue_capacity=2)

    assert recorder.submit(frame) is True
    summary = recorder.close()

    image_path = output_dir / "frames/frame_00000007.jpg"
    assert image_path.read_bytes() == frame.original_jpeg
    manifest = [json.loads(line) for line in (output_dir / "manifest.jsonl").read_text().splitlines()]
    assert manifest == [
        {
            "bytes": len(frame.original_jpeg),
            "file": "frames/frame_00000007.jpg",
            "received_monotonic_s": 12.5,
            "sequence": 7,
            "sha256": hashlib.sha256(frame.original_jpeg).hexdigest(),
            "source_height": 720,
            "source_width": 1280,
        }
    ]
    assert summary["written_frame_count"] == 1
    assert summary["queue_drop_count"] == 0
    assert json.loads((output_dir / "summary.json").read_text()) == summary


def test_recorder_refuses_an_existing_directory(tmp_path: Path) -> None:
    gateway = _gateway()
    output_dir = tmp_path / "existing"
    output_dir.mkdir()

    with pytest.raises(FileExistsError):
        gateway.OriginalJpegRecorder(output_dir)


def test_recorder_queue_is_bounded_and_submit_never_waits(tmp_path: Path) -> None:
    gateway = _gateway()
    writer_started = threading.Event()
    release_writer = threading.Event()

    def blocking_writer(path: Path, payload: bytes) -> None:
        writer_started.set()
        assert release_writer.wait(2.0)
        path.write_bytes(payload)

    recorder = gateway.OriginalJpegRecorder(
        tmp_path / "bounded", queue_capacity=1, writer=blocking_writer
    )
    assert recorder.submit(_frame(gateway, 1)) is True
    assert writer_started.wait(1.0)
    assert recorder.submit(_frame(gateway, 2)) is True

    started = time.monotonic()
    assert recorder.submit(_frame(gateway, 3)) is False
    assert time.monotonic() - started < 0.02
    assert recorder.snapshot().queue_drop_count == 1

    release_writer.set()
    assert recorder.close()["written_frame_count"] == 2


def test_recorder_counts_disk_error_and_continues_shutdown(tmp_path: Path) -> None:
    gateway = _gateway()

    def failed_writer(_path: Path, _payload: bytes) -> None:
        raise OSError("disk full")

    recorder = gateway.OriginalJpegRecorder(
        tmp_path / "disk-error", queue_capacity=1, writer=failed_writer
    )
    assert recorder.submit(_frame(gateway, 1)) is True

    summary = recorder.close()

    assert summary["write_error_count"] == 1
    assert summary["written_frame_count"] == 0
    assert "disk full" in summary["last_write_error"]


def test_http_health_and_preview_use_the_latest_preencoded_frame() -> None:
    gateway = _gateway()
    mailbox = gateway.LatestFrameMailbox()
    counters = gateway.GatewayCounters()
    server = gateway.build_http_server(
        mailbox, counters, host="127.0.0.1", port=0, clock=lambda: 10.25
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(f"{base_url}/health", timeout=1.0) as response:
            health = json.load(response)
        assert health["ready"] is False

        frame = _frame(gateway, 9, received=10.0)
        mailbox.put(frame)
        counters.successful_frame_count = 1
        with urlopen(f"{base_url}/preview.jpg", timeout=1.0) as response:
            assert response.read() == frame.preview_jpeg
            assert response.headers["X-R1-Sequence"] == "9"
            assert response.headers["X-R1-Source-Age-Ms"] == "250.000"
            assert response.headers["X-R1-Source-Width"] == "1280"
            assert response.headers["X-R1-Source-Height"] == "720"
        with urlopen(f"{base_url}/health", timeout=1.0) as response:
            health = json.load(response)
        assert health["ready"] is True
        assert health["latest_sequence"] == 9
        assert health["latest_frame_age_ms"] == 250.0

        with pytest.raises(HTTPError) as error:
            urlopen(f"{base_url}/not-found", timeout=1.0)
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_http_health_identifies_its_recording_session(tmp_path: Path) -> None:
    gateway = _gateway()
    recorder = gateway.OriginalJpegRecorder(tmp_path / "camera_run")
    server = gateway.build_http_server(
        gateway.LatestFrameMailbox(), gateway.GatewayCounters(),
        host="127.0.0.1", port=0, recorder=recorder,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(f"http://127.0.0.1:{server.server_port}/health", timeout=1.0) as response:
            health = json.load(response)
        assert health["record_dir"] == str(recorder.output_dir.resolve())
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)
        recorder.close()


def test_http_rejects_non_loopback_bind() -> None:
    gateway = _gateway()

    with pytest.raises(ValueError, match="loopback"):
        gateway.build_http_server(
            gateway.LatestFrameMailbox(),
            gateway.GatewayCounters(),
            host="0.0.0.0",
            port=8765,
        )


def test_gateway_parser_defaults_and_rejects_non_loopback_host(tmp_path: Path) -> None:
    gateway = _gateway()
    args = gateway.parse_args(["--record-dir", str(tmp_path / "record")])

    assert args.interface == "eth10"
    assert args.rpc_timeout_s == 0.1
    assert args.capture_hz == 15.0
    assert (args.preview_width, args.preview_height, args.preview_quality) == (640, 360, 60)
    assert (args.host, args.port) == ("127.0.0.1", 8765)

    with pytest.raises(SystemExit):
        gateway.parse_args(
            ["--record-dir", str(tmp_path / "record-2"), "--host", "0.0.0.0"]
        )
