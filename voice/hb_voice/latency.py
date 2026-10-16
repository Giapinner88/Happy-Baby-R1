"""Low-overhead, behavior-neutral latency markers for robot voice turns."""

from __future__ import annotations

import re
import time
from collections.abc import Callable

from loguru import logger
from pipecat.frames.frames import Frame, TranscriptionFrame, UserStartedSpeakingFrame
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor


_GESTURE_HINT = re.compile(
    r"\b(vẫy|bắt\s*tay|trái\s*tim|thả\s*tim|khoanh\s*tay|tạo\s*dáng|"
    r"ngầu|nắm\s*tay|quyết\s*tâm|tặng\s*quà|đưa\s*quà|cử\s*chỉ|"
    r"động\s*tác|hạ\s*tay|dừng\s*lại|chào|tạm\s*biệt|hẹn\s*gặp\s*lại|"
    r"rất\s*vui\s*được\s*gặp|dễ\s*thương|đáng\s*yêu|tuyệt\s*vời|"
    r"thông\s*minh|xinh|đẹp|giỏi)\b",
    re.IGNORECASE,
)


def looks_like_gesture_request(text: str) -> bool:
    """Conservative hint used only to decide whether spoken text is buffered."""
    return bool(_GESTURE_HINT.search(text))


class GestureTurnGuardProcessor(FrameProcessor):
    """Select the per-turn guard before the user context reaches Realtime."""

    def __init__(self, callback: Callable[[bool], None]) -> None:
        super().__init__()
        self._callback = callback
        self._gesture_hint = False

    def reset_turn(self) -> None:
        self._gesture_hint = False
        self._callback(False)

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)
        if isinstance(frame, UserStartedSpeakingFrame):
            self.reset_turn()
        elif isinstance(frame, TranscriptionFrame) and frame.text.strip():
            # Transcription can arrive in several fragments. Once any fragment
            # looks gesture-like, keep the guard for the remainder of the turn.
            self._gesture_hint = self._gesture_hint or looks_like_gesture_request(
                frame.text
            )
            self._callback(self._gesture_hint)
        await self.push_frame(frame, direction)


class TurnLatencyTracker:
    """Correlate monotonic timing markers without changing pipeline control flow."""

    def __init__(self) -> None:
        self._sequence = 0
        self._started_at: float | None = None
        self._kind = "unknown"
        self._seen: set[str] = set()

    @property
    def turn_id(self) -> int:
        return self._sequence

    def start_turn(self) -> None:
        self._sequence += 1
        self._started_at = time.monotonic()
        self._kind = "unknown"
        self._seen.clear()
        logger.info("VOICE_LATENCY turn={} event=turn_stopped", self._sequence)

    def set_transcript(self, text: str) -> None:
        if self._started_at is None:
            self.start_turn()
        if self._kind not in {"gesture", "camera"}:
            self._kind = (
                "gesture_candidate" if looks_like_gesture_request(text) else "chat"
            )

    def mark_tool(self, kind: str) -> None:
        self._kind = kind
        self.mark("tool_call")

    def mark(self, metric: str) -> None:
        if self._started_at is None or metric in self._seen:
            return
        self._seen.add(metric)
        elapsed_ms = (time.monotonic() - self._started_at) * 1000.0
        logger.info(
            "VOICE_LATENCY turn={} kind={} metric={} ms={:.1f}",
            self._sequence,
            self._kind,
            metric,
            elapsed_ms,
        )

    def camera_capture(self, *, subprocess_ms: float, total_ms: float) -> None:
        logger.info(
            "VOICE_LATENCY turn={} kind=camera metric=camera_subprocess_ms ms={:.1f}",
            self._sequence,
            subprocess_ms,
        )
        logger.info(
            "VOICE_LATENCY turn={} kind=camera metric=camera_capture_total_ms ms={:.1f}",
            self._sequence,
            total_ms,
        )

