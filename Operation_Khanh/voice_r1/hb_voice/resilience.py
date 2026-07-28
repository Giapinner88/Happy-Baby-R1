"""Small, dependency-free helpers for resilient headless voice sessions."""

from __future__ import annotations

import os
import re
import sys
import time
from array import array
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEFAULT_VOICE_STATUS_PATH = Path("/run/hb/voice_status.env")


def apply_pcm16_gain(audio: bytes, gain_db: float) -> bytes:
    """Apply bounded digital gain to little-endian signed 16-bit PCM."""
    if not audio or gain_db == 0.0:
        return audio
    samples = array("h")
    samples.frombytes(audio)
    if sys.byteorder != "little":
        samples.byteswap()
    multiplier = 10 ** (gain_db / 20.0)
    for index, sample in enumerate(samples):
        samples[index] = max(-32768, min(32767, round(sample * multiplier)))
    if sys.byteorder != "little":
        samples.byteswap()
    return samples.tobytes()


def pcm_bytes_for_ms(
    sample_rate: int,
    duration_ms: int,
    *,
    channels: int = 1,
    bytes_per_sample: int = 2,
) -> int:
    """Return the minimum whole number of PCM bytes for ``duration_ms``."""
    return (sample_rate * duration_ms * channels * bytes_per_sample + 999) // 1000


@dataclass
class PttTurnAudioBuffer:
    """Buffer the beginning of a PTT turn until it is safe to commit.

    Frames are held only until ``minimum_bytes`` is reached. After that,
    ``add`` returns each frame immediately. A turn that ends before reaching
    the threshold never becomes ready and must not be committed upstream.
    """

    minimum_bytes: int
    capturing: bool = False
    ready: bool = False
    audio_bytes: int = 0
    _pending: list[Any] = field(default_factory=list)

    def begin(self) -> None:
        self.capturing = True
        self.ready = False
        self.audio_bytes = 0
        self._pending.clear()

    def add(self, frame: Any, audio_bytes: int) -> list[Any]:
        if not self.capturing or audio_bytes <= 0:
            return []

        self.audio_bytes += audio_bytes
        if self.ready:
            return [frame]

        self._pending.append(frame)
        if self.audio_bytes < self.minimum_bytes:
            return []

        self.ready = True
        frames = self._pending
        self._pending = []
        return frames

    def finish(self) -> tuple[bool, int]:
        should_commit = self.capturing and self.ready
        audio_bytes = self.audio_bytes
        self.abort()
        return should_commit, audio_bytes

    def abort(self) -> None:
        self.capturing = False
        self.ready = False
        self.audio_bytes = 0
        self._pending.clear()


def is_realtime_connection_error(
    error_frame: Any, expected_processor: Any = None
) -> bool:
    """Identify OpenAI Realtime transport failures that require a new session."""
    if (
        expected_processor is not None
        and getattr(error_frame, "processor", None) is not expected_processor
    ):
        return False

    exception = getattr(error_frame, "exception", None)
    module = type(exception).__module__.lower() if exception is not None else ""
    if module.startswith(("websockets", "aiohttp", "httpx", "httpcore")):
        return True

    details = " ".join(
        str(value)
        for value in (getattr(error_frame, "error", ""), exception)
        if value is not None
    ).lower()
    connection_markers = (
        "error connecting",
        "connect call failed",
        "keepalive ping timeout",
        "ping timeout",
        "no close frame received",
        "connectionclosed",
        "connection closed",
        "connection is closed",
        "connection reset",
        "broken pipe",
        "network is unreachable",
        "no route to host",
        "connection refused",
        "temporary failure in name resolution",
        "name or service not known",
        "errno 101",
        "errno 110",
        "errno 111",
        "errno 113",
        "session expired",
        "session_expired",
        "not connected",
    )
    return any(marker in details for marker in connection_markers)


def realtime_service_state(service: Any) -> tuple[bool, bool]:
    """Return ``(ready, receive_stopped)`` for the pinned Pipecat service."""
    ready = bool(getattr(service, "_api_session_ready", False))
    receive_task = getattr(service, "_receive_task", None)
    receive_stopped = ready and (
        receive_task is None
        or (callable(getattr(receive_task, "done", None)) and receive_task.done())
    )
    if ready and getattr(service, "_websocket", None) is None:
        receive_stopped = True
    return ready, receive_stopped


class VoiceRuntimeStatus:
    """Atomically expose actual voice readiness without secrets."""

    def __init__(self, path: str | Path = DEFAULT_VOICE_STATUS_PATH):
        self.path = Path(path)
        self.attempt = 0
        self.last_reason = "none"

    def update(
        self,
        state: str,
        *,
        openai_ready: bool = False,
        mic_ready: bool = False,
        attempt: int | None = None,
        reason: str | None = None,
    ) -> None:
        if attempt is not None:
            self.attempt = attempt
        if reason:
            self.last_reason = reason
        safe_state = _status_token(state)
        safe_reason = _status_token(self.last_reason)
        content = (
            f"state={safe_state}\n"
            f"openai_ready={int(openai_ready)}\n"
            f"mic_ready={int(mic_ready)}\n"
            f"attempt={self.attempt}\n"
            f"last_reason={safe_reason}\n"
            f"updated_unix={int(time.time())}\n"
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{os.getpid()}.tmp")
        temporary.write_text(content, encoding="utf-8")
        os.chmod(temporary, 0o644)
        temporary.replace(self.path)


def _status_token(value: str) -> str:
    token = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return token[:120] or "unknown"
