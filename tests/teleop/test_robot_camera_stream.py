from __future__ import annotations

from collections.abc import Callable
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from scripts.teleop.robot_camera_stream import (
    HttpCameraResponse,
    HttpRobotCameraSubscriber,
    RobotCameraSnapshot,
)
from scripts.teleop.quest_bridge import LocalCameraViewController, parse_args
from scripts.teleop.robot_camera_vuer import RobotCameraFrameConfig


def _jpeg(width: int = 8, height: int = 6) -> bytes:
    bgr = np.zeros((height, width, 3), dtype=np.uint8)
    bgr[:, :, 2] = 180
    ok, encoded = cv2.imencode(".jpg", bgr)
    assert ok
    return encoded.tobytes()


def _response(
    sequence: str = "1",
    source_age_ms: str = "25.0",
    *,
    payload: bytes | None = None,
    status: int = 200,
    headers: dict[str, str] | None = None,
) -> HttpCameraResponse:
    actual_headers = {
        "Content-Type": "image/jpeg",
        "X-R1-Sequence": sequence,
        "X-R1-Source-Age-Ms": source_age_ms,
    }
    if headers is not None:
        actual_headers = headers
    return HttpCameraResponse(status, actual_headers, _jpeg() if payload is None else payload)


def _wait_until(predicate: Callable[[], bool], timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true")


def test_subscriber_decodes_valid_response_and_keeps_latest_sequence() -> None:
    responses = iter([_response("4", "30"), _response("5", "40")])

    def fetch(_url: str, _timeout_s: float) -> HttpCameraResponse:
        try:
            return next(responses)
        except StopIteration as exc:
            raise TimeoutError("no more responses") from exc

    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg",
        poll_hz=15.0,
        fetch=fetch,
    )
    subscriber.start()
    try:
        _wait_until(
            lambda: subscriber.snapshot() is not None
            and subscriber.snapshot().sequence == 5
        )
        snapshot = subscriber.snapshot()
        assert snapshot is not None
        assert snapshot.sequence == 5
        assert snapshot.robot_source_age_s == 0.04
        assert snapshot.bgr.shape == (6, 8, 3)
        assert snapshot.bgr.flags.writeable is False
    finally:
        subscriber.close()


@pytest.mark.parametrize(
    "response",
    [
        _response(status=503),
        _response(payload=b"\xff\xd8truncated"),
        _response(headers={"Content-Type": "image/jpeg", "X-R1-Source-Age-Ms": "1"}),
        _response(headers={"Content-Type": "image/jpeg", "X-R1-Sequence": "1"}),
        _response(sequence="not-an-int"),
        _response(sequence="-1"),
        _response(source_age_ms="nan"),
        _response(source_age_ms="-0.1"),
        _response(headers={
            "Content-Type": "text/plain",
            "X-R1-Sequence": "1",
            "X-R1-Source-Age-Ms": "1",
        }),
    ],
)
def test_subscriber_rejects_bad_status_payload_or_headers(response: HttpCameraResponse) -> None:
    fetched = threading.Event()

    def fetch(_url: str, _timeout_s: float) -> HttpCameraResponse:
        fetched.set()
        return response

    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg", poll_hz=10.0, fetch=fetch
    )
    subscriber.start()
    try:
        assert fetched.wait(1.0)
        _wait_until(lambda: subscriber.stats()["request_count"] >= 1)
        assert subscriber.snapshot() is None
        stats = subscriber.stats()
        assert stats["successful_frame_count"] == 0
        assert stats["last_error"]
    finally:
        subscriber.close()


def test_subscriber_counts_timeout_without_publishing_a_frame() -> None:
    called = threading.Event()

    def fetch(_url: str, timeout_s: float) -> HttpCameraResponse:
        assert timeout_s == 0.4
        called.set()
        raise TimeoutError("timed out")

    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg", poll_hz=10.0, fetch=fetch
    )
    subscriber.start()
    try:
        assert called.wait(1.0)
        _wait_until(lambda: subscriber.stats()["request_failure_count"] >= 1)
        assert subscriber.snapshot() is None
    finally:
        subscriber.close()


def test_subscriber_rejects_a_valid_jpeg_with_unexpected_dimensions() -> None:
    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg",
        poll_hz=10.0,
        expected_width=8,
        expected_height=6,
        fetch=lambda _url, _timeout: _response(payload=_jpeg(width=9, height=6)),
    )
    subscriber.start()
    try:
        _wait_until(lambda: subscriber.stats()["request_count"] >= 1)
        assert subscriber.snapshot() is None
        assert "dimensions" in str(subscriber.stats()["last_error"])
    finally:
        subscriber.close()


def test_subscriber_ignores_duplicate_and_out_of_order_sequences() -> None:
    responses = iter([_response("8"), _response("8"), _response("7")])

    def fetch(_url: str, _timeout_s: float) -> HttpCameraResponse:
        try:
            return next(responses)
        except StopIteration as exc:
            raise TimeoutError("done") from exc

    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg", poll_hz=15.0, fetch=fetch
    )
    subscriber.start()
    try:
        _wait_until(lambda: subscriber.stats()["stale_sequence_count"] == 2)
        assert subscriber.snapshot().sequence == 8
        assert subscriber.stats()["successful_frame_count"] == 1
    finally:
        subscriber.close()


def test_snapshot_does_not_wait_for_blocked_http_fetch() -> None:
    fetch_started = threading.Event()
    release = threading.Event()

    def fetch(_url: str, _timeout_s: float) -> HttpCameraResponse:
        fetch_started.set()
        assert release.wait(2.0)
        return _response()

    subscriber = HttpRobotCameraSubscriber(
        "http://127.0.0.1:8765/preview.jpg", fetch=fetch
    )
    subscriber.start()
    assert fetch_started.wait(1.0)
    started = time.monotonic()
    assert subscriber.snapshot() is None
    assert time.monotonic() - started < 0.02
    release.set()
    subscriber.close()


@pytest.mark.parametrize(
    "url",
    [
        "ftp://127.0.0.1:8765/preview.jpg",
        "http://robot.local:8765/preview.jpg",
        "http://user:secret@127.0.0.1:8765/preview.jpg",
        "http://127.0.0.1:8765/not-preview",
        "http://127.0.0.1:8765/preview.jpg?token=secret",
    ],
)
def test_subscriber_rejects_non_loopback_or_ambiguous_endpoint(url: str) -> None:
    with pytest.raises(ValueError):
        HttpRobotCameraSubscriber(url)


class _FakeCameraDisplay:
    def __init__(self) -> None:
        self.frames: list[int] = []
        self.enabled: list[bool] = []

    def set_robot_camera_frame(self, _bgr: np.ndarray, sequence: int) -> bool:
        self.frames.append(sequence)
        return True

    def set_robot_camera_enabled(self, enabled: bool) -> None:
        self.enabled.append(enabled)


def _telemetry(*, a: bool = False, left_trigger: bool = False, right_trigger: bool = False):
    return SimpleNamespace(
        right_ctrl_aButton=a,
        left_ctrl_trigger=left_trigger,
        right_ctrl_trigger=right_trigger,
    )


def _snapshot(sequence: int, *, source_age_s: float, received_s: float) -> RobotCameraSnapshot:
    return RobotCameraSnapshot(
        sequence=sequence,
        bgr=np.zeros((360, 640, 3), dtype=np.uint8),
        robot_source_age_s=source_age_s,
        received_monotonic_s=received_s,
    )


def test_local_camera_view_starts_passthrough_and_toggles_only_on_fresh_a_edges() -> None:
    display = _FakeCameraDisplay()
    controller = LocalCameraViewController(
        display,
        RobotCameraFrameConfig(),
        enable_max_age_s=0.5,
        fallback_max_age_s=1.0,
    )
    fresh = _snapshot(3, source_age_s=0.05, received_s=9.9)

    assert controller.update(_telemetry(right_trigger=True), fresh, now_s=10.0) == []
    enabled = controller.update(
        _telemetry(a=True, right_trigger=True), fresh, now_s=10.0
    )
    assert enabled[0]["event"] == "camera_view_changed"
    assert enabled[0]["view"] == "robot_camera"
    assert display.enabled == [True]
    assert display.frames == [3]

    assert controller.update(
        _telemetry(a=True, left_trigger=True, right_trigger=True), fresh, now_s=10.1
    ) == []
    assert controller.update(_telemetry(left_trigger=True), fresh, now_s=10.2) == []
    disabled = controller.update(
        _telemetry(a=True, left_trigger=True), fresh, now_s=10.3
    )
    assert disabled[0]["view"] == "quest_passthrough"
    assert display.enabled == [True, False]


def test_local_camera_view_rejects_stale_enable_and_rearms_next_edge() -> None:
    display = _FakeCameraDisplay()
    controller = LocalCameraViewController(
        display,
        RobotCameraFrameConfig(),
        enable_max_age_s=0.5,
        fallback_max_age_s=1.0,
    )
    stale = _snapshot(1, source_age_s=0.6, received_s=10.0)
    fresh = _snapshot(2, source_age_s=0.05, received_s=10.9)

    controller.update(_telemetry(), stale, now_s=10.0)
    rejected = controller.update(_telemetry(a=True), stale, now_s=10.0)
    assert rejected[0]["event"] == "camera_enable_rejected"
    assert rejected[0]["robot_source_age_s"] == 0.6
    assert display.enabled == []

    controller.update(_telemetry(), fresh, now_s=11.0)
    enabled = controller.update(_telemetry(a=True), fresh, now_s=11.0)
    assert enabled[0]["event"] == "camera_view_changed"
    assert enabled[0]["view"] == "robot_camera"


@pytest.mark.parametrize(
    ("source_age_s", "received_s", "expected_reason"),
    [
        (1.1, 20.0, "robot_source_stale"),
        (0.0, 18.8, "robot_source_and_workstation_receive_stale"),
    ],
)
def test_local_camera_view_falls_back_when_active_frame_becomes_stale(
    source_age_s: float, received_s: float, expected_reason: str
) -> None:
    display = _FakeCameraDisplay()
    controller = LocalCameraViewController(
        display,
        RobotCameraFrameConfig(),
        enable_max_age_s=0.5,
        fallback_max_age_s=1.0,
    )
    fresh = _snapshot(1, source_age_s=0.0, received_s=20.0)
    controller.update(_telemetry(), fresh, now_s=20.0)
    controller.update(_telemetry(a=True), fresh, now_s=20.0)

    event = controller.update(
        _telemetry(a=True),
        _snapshot(2, source_age_s=source_age_s, received_s=received_s),
        now_s=20.0,
    )

    assert event[0]["event"] == "camera_runtime_fallback"
    assert event[0]["reason"] == expected_reason
    assert display.enabled[-1] is False


def test_camera_cli_rejects_simultaneous_local_preview_and_webrtc() -> None:
    with pytest.raises(SystemExit):
        parse_args(
            [
                "--host-ip",
                "10.42.0.1",
                "--robot-camera-preview-url",
                "http://127.0.0.1:8765/preview.jpg",
                "--robot-camera-webrtc-url",
                "https://robot.example/offer",
            ]
        )


def test_redacted_command_removes_camera_query_secrets_for_both_transports() -> None:
    from scripts.teleop.quest_bridge import _redacted_command

    command = _redacted_command(
        [
            "quest_bridge.py",
            "--robot-camera-preview-url",
            "http://127.0.0.1:8765/preview.jpg?token=preview-secret",
            "--robot-camera-webrtc-url",
            "https://robot.example/offer?token=webrtc-secret",
        ]
    )
    assert "preview-secret" not in " ".join(command)
    assert "webrtc-secret" not in " ".join(command)
