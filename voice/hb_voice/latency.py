"""Low-overhead, behavior-neutral latency markers for robot voice turns."""

from __future__ import annotations

import re
import time
from collections.abc import Callable

from loguru import logger
from pipecat.frames.frames import (
    Frame,
    InterimTranscriptionFrame,
    TranscriptionFrame,
    UserStartedSpeakingFrame,
)
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor


_GESTURE_HINT = re.compile(
    r"\b(vẫy|bắt\s*tay|trái\s*tim|thả\s*tim|khoanh\s*tay|tạo\s*dáng|"
    r"ngầu|nắm\s*tay|quyết\s*tâm|tặng\s*quà|đưa\s*quà|cử\s*chỉ|"
    r"động\s*tác|hạ\s*tay|dừng\s*lại|chào|tạm\s*biệt|hẹn\s*gặp\s*lại|"
    r"rất\s*vui\s*được\s*gặp|dễ\s*thương|đáng\s*yêu|tuyệt\s*vời|"
    r"thông\s*minh|xinh|đẹp|giỏi)\b",
    re.IGNORECASE,
)

_CAMERA_HINT = re.compile(
    r"\b(đây\s+là\s+(?:gì|cái\s+gì|số\s+mấy|màu\s+gì)|"
    r"(?:cái|vật)\s+gì\s+(?:đây|này)|số\s+mấy|màu\s+gì|"
    r"(?:bạn|robot|mình)\s+(?:nhìn|thấy|xem)\s+(?:được\s+)?gì|"
    r"(?:nhìn|xem)\s+(?:giúp|hộ)|trước\s+(?:camera|mặt)|"
    r"(?:tôi|mình|con|bé)\s+đang\s+(?:cầm|giơ|đưa)|"
    r"camera|ảnh\s+(?:này|đây)|hình\s+(?:này|đây))\b",
    re.IGNORECASE,
)


def looks_like_gesture_request(text: str) -> bool:
    """Conservative hint used only to decide whether spoken text is buffered."""
    return bool(_GESTURE_HINT.search(text))


def looks_like_camera_request(text: str) -> bool:
    """Conservative hint for starting one camera capture before the tool call."""
    return bool(_CAMERA_HINT.search(text))


class GestureTurnGuardProcessor(FrameProcessor):
    """Select the gesture guard and prefetch camera from Realtime transcripts."""

    def __init__(
        self,
        callback: Callable[[bool], None],
        camera_prefetch: Callable[[], None] | None = None,
    ) -> None:
        super().__init__()
        self._callback = callback
        self._camera_prefetch = camera_prefetch
        self._gesture_hint = False
        self._transcript_hint = ""

    def reset_turn(self) -> None:
        self._gesture_hint = False
        self._transcript_hint = ""
        self._callback(False)

    def _apply_transcript_hint(self, text: str) -> None:
        self._gesture_hint = self._gesture_hint or looks_like_gesture_request(text)
        self._callback(self._gesture_hint)
        if self._camera_prefetch and looks_like_camera_request(text):
            self._camera_prefetch()

    def observe_transcript(self, text: str, *, incremental: bool) -> None:
        """Apply one Realtime transcript delta or final transcript."""
        self._transcript_hint = (
            self._transcript_hint + text if incremental else text
        )
        self._apply_transcript_hint(self._transcript_hint)

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)
        if isinstance(frame, UserStartedSpeakingFrame):
            self.reset_turn()
        elif isinstance(frame, InterimTranscriptionFrame) and frame.text.strip():
            # Realtime emits incremental deltas upstream. Accumulate them so a
            # phrase split as "số " + "mấy" can still start capture early.
            self.observe_transcript(frame.text, incremental=True)
        elif isinstance(frame, TranscriptionFrame) and frame.text.strip():
            self.observe_transcript(frame.text, incremental=False)
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

