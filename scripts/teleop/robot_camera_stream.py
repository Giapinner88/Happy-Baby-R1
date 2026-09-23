"""Bounded HTTP subscriber for the R1 camera preview tunnel."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import ipaddress
import math
import threading
import time
from typing import Any
from urllib.parse import urlsplit
from urllib.request import urlopen

import cv2
import numpy as np


@dataclass(frozen=True)
class HttpCameraResponse:
    status: int
    headers: Mapping[str, str]
    payload: bytes


@dataclass(frozen=True)
class RobotCameraSnapshot:
    sequence: int
    bgr: np.ndarray
    robot_source_age_s: float
    received_monotonic_s: float


def _validate_endpoint(endpoint: str) -> str:
    parsed = urlsplit(endpoint)
    try:
        port = parsed.port
    except ValueError as exc:
        raise ValueError("camera preview URL has an invalid port") from exc
    if parsed.scheme != "http" or not parsed.hostname or port is None:
        raise ValueError("camera preview URL must be explicit loopback HTTP")
    if parsed.username is not None or parsed.password is not None:
        raise ValueError("camera preview URL must not contain credentials")
    try:
        loopback = ipaddress.ip_address(parsed.hostname).is_loopback
    except ValueError:
        loopback = parsed.hostname == "localhost"
    if not loopback:
        raise ValueError("camera preview URL must use a loopback host")
    if parsed.path != "/preview.jpg" or parsed.query or parsed.fragment:
        raise ValueError("camera preview URL must end at /preview.jpg without query or fragment")
    return endpoint


def _header(headers: Mapping[str, str], name: str) -> str | None:
    wanted = name.casefold()
    for key, value in headers.items():
        if key.casefold() == wanted:
            return str(value)
    return None


class HttpRobotCameraSubscriber:
    """Fetch and decode at most one camera response at a time."""

    def __init__(
        self,
        endpoint: str,
        *,
        poll_hz: float = 10.0,
        timeout_s: float = 0.4,
        max_jpeg_bytes: int = 2_000_000,
        expected_width: int | None = None,
        expected_height: int | None = None,
        fetch: Callable[[str, float], HttpCameraResponse] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.endpoint = _validate_endpoint(endpoint)
        if not math.isfinite(poll_hz) or not 1.0 <= poll_hz <= 15.0:
            raise ValueError("poll_hz must be finite and in [1, 15]")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        if max_jpeg_bytes <= 0:
            raise ValueError("max_jpeg_bytes must be positive")
        if (expected_width is None) != (expected_height is None):
            raise ValueError("expected_width and expected_height must be specified together")
        if expected_width is not None and (expected_width <= 0 or expected_height <= 0):
            raise ValueError("expected preview dimensions must be positive")
        self.poll_hz = float(poll_hz)
        self.timeout_s = float(timeout_s)
        self.max_jpeg_bytes = int(max_jpeg_bytes)
        self.expected_width = expected_width
        self.expected_height = expected_height
        self._clock = clock
        self._fetch = fetch if fetch is not None else self._fetch_http
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._active_response: Any | None = None
        self._snapshot: RobotCameraSnapshot | None = None
        self._request_count = 0
        self._request_failure_count = 0
        self._invalid_response_count = 0
        self._stale_sequence_count = 0
        self._successful_frame_count = 0
        self._last_error: str | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("camera subscriber has already been started")
            self._thread = threading.Thread(
                target=self._run,
                name="r1-camera-http-subscriber",
                daemon=True,
            )
            thread = self._thread
        thread.start()

    def snapshot(self) -> RobotCameraSnapshot | None:
        with self._lock:
            return self._snapshot

    def stats(self) -> dict[str, object]:
        with self._lock:
            return {
                "request_count": self._request_count,
                "request_failure_count": self._request_failure_count,
                "invalid_response_count": self._invalid_response_count,
                "stale_sequence_count": self._stale_sequence_count,
                "successful_frame_count": self._successful_frame_count,
                "latest_sequence": self._snapshot.sequence if self._snapshot else None,
                "last_error": self._last_error,
            }

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            response = self._active_response
            thread = self._thread
        if response is not None:
            try:
                response.close()
            except Exception:  # noqa: BLE001 - shutdown remains best effort
                pass
        if thread is not None:
            thread.join(timeout=self.timeout_s + 1.0)

    def _fetch_http(self, endpoint: str, timeout_s: float) -> HttpCameraResponse:
        response = urlopen(endpoint, timeout=timeout_s)
        with self._lock:
            self._active_response = response
        try:
            payload = response.read(self.max_jpeg_bytes + 1)
            return HttpCameraResponse(
                status=int(response.status),
                headers=dict(response.headers.items()),
                payload=payload,
            )
        finally:
            response.close()
            with self._lock:
                if self._active_response is response:
                    self._active_response = None

    def _run(self) -> None:
        period_s = 1.0 / self.poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            self._poll_once()
            remaining = period_s - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop.wait(remaining)

    def _record_error(self, exc: Exception, *, invalid: bool) -> None:
        with self._lock:
            if invalid:
                self._invalid_response_count += 1
            else:
                self._request_failure_count += 1
            self._last_error = f"{type(exc).__name__}: {exc}"

    def _poll_once(self) -> None:
        try:
            response = self._fetch(self.endpoint, self.timeout_s)
            if int(response.status) != 200:
                raise ConnectionError(f"camera preview returned HTTP {response.status}")
        except Exception as exc:  # noqa: BLE001 - retry on the next bounded poll
            with self._lock:
                self._request_count += 1
            self._record_error(exc, invalid=False)
            return

        try:
            content_type = _header(response.headers, "Content-Type")
            if content_type is None or content_type.split(";", 1)[0].strip().lower() != "image/jpeg":
                raise ValueError("camera response Content-Type is not image/jpeg")
            sequence_text = _header(response.headers, "X-R1-Sequence")
            age_text = _header(response.headers, "X-R1-Source-Age-Ms")
            if sequence_text is None or age_text is None:
                raise ValueError("camera response is missing sequence or age headers")
            sequence = int(sequence_text)
            source_age_ms = float(age_text)
            if sequence < 0:
                raise ValueError("camera sequence must be non-negative")
            if not math.isfinite(source_age_ms) or source_age_ms < 0.0:
                raise ValueError("camera source age must be finite and non-negative")
            payload = bytes(response.payload)
            if len(payload) > self.max_jpeg_bytes:
                raise ValueError("camera JPEG exceeds configured byte limit")
            if not payload.startswith(b"\xff\xd8") or not payload.rstrip(b"\x00").endswith(b"\xff\xd9"):
                raise ValueError("camera response is not a complete JPEG")
            decoded = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is None or decoded.ndim != 3 or decoded.shape[2] != 3:
                raise ValueError("camera JPEG could not be decoded as BGR")
            if self.expected_width is not None and decoded.shape[:2] != (
                self.expected_height,
                self.expected_width,
            ):
                raise ValueError(
                    "camera preview dimensions "
                    f"{decoded.shape[1]}x{decoded.shape[0]} do not match "
                    f"{self.expected_width}x{self.expected_height}"
                )
        except Exception as exc:  # noqa: BLE001 - malformed response is observable and skipped
            with self._lock:
                self._request_count += 1
            self._record_error(exc, invalid=True)
            return

        received_monotonic_s = float(self._clock())
        decoded = np.ascontiguousarray(decoded)
        decoded.setflags(write=False)
        with self._lock:
            self._request_count += 1
            if self._snapshot is not None and sequence <= self._snapshot.sequence:
                self._stale_sequence_count += 1
                return
            self._snapshot = RobotCameraSnapshot(
                sequence=sequence,
                bgr=decoded,
                robot_source_age_s=source_age_ms / 1000.0,
                received_monotonic_s=received_monotonic_s,
            )
            self._successful_frame_count += 1
            self._last_error = None
