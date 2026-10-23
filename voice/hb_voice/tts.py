"""Robot-specific lifecycle fixes for the external streaming TTS service."""

from __future__ import annotations

import asyncio
from collections import deque

from loguru import logger
from pipecat.services.elevenlabs.tts import ElevenLabsTTSService


class ResilientElevenLabsTTSService(ElevenLabsTTSService):
    """Recover the ElevenLabs socket without discarding the OpenAI session.

    Pipecat already retries its receive loop.  If those retries are exhausted it
    emits an error frame; the application can call :meth:`recover_connection`
    for a fresh, bounded reconnect rather than cancelling the whole pipeline.

    ElevenLabs can also deliver audio after an interrupted context was closed.
    Remember those retired IDs and discard their late packets quietly.  They
    must never recreate an old context or leak into the next reply.
    """

    def __init__(
        self,
        *args,
        reconnect_attempts: int = 3,
        reconnect_initial_s: float = 0.5,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self._hb_reconnect_attempts = reconnect_attempts
        self._hb_reconnect_initial_s = reconnect_initial_s
        self._hb_reconnect_lock = asyncio.Lock()
        self._hb_retired_contexts: deque[str] = deque(maxlen=64)

    def _retire_context(self, context_id: str | None) -> None:
        if context_id and context_id not in self._hb_retired_contexts:
            self._hb_retired_contexts.append(context_id)

    def should_recover_after_error(self, error_frame) -> bool:
        """Let Pipecat finish its own reconnect before starting another one.

        Pipecat reports an ErrorFrame after *each* failed reconnect attempt.
        Starting our recovery on the first frame closes the socket while its
        receive loop is still retrying, extending the period without speech.
        """
        message = str(getattr(error_frame, "error", "")).lower()
        exhausted = (
            "failed to reconnect after" in message
            or "times immediately after connecting" in message
        )
        return exhausted or not self._reconnect_in_progress

    async def append_to_audio_context(self, context_id, frame):
        if not context_id:
            return
        # ``None`` is Pipecat's internal deletion sentinel. It must still reach
        # the queue or the context task/map would leak during reconnect.
        if context_id in self._hb_retired_contexts and frame is not None:
            return
        await super().append_to_audio_context(context_id, frame)

    async def on_audio_context_interrupted(self, context_id: str):
        self._retire_context(context_id)
        await super().on_audio_context_interrupted(context_id)

    async def on_audio_context_completed(self, context_id: str):
        self._retire_context(context_id)
        await super().on_audio_context_completed(context_id)

    async def recover_connection(self) -> bool:
        """Try a fresh WebSocket connection while leaving the pipeline alive."""
        async with self._hb_reconnect_lock:
            delay = self._hb_reconnect_initial_s
            for attempt in range(1, self._hb_reconnect_attempts + 1):
                try:
                    # The failed receive loop may leave contexts that can no
                    # longer receive a final packet. Retire them before the
                    # base disconnect removes its bookkeeping.
                    for context_id in tuple(self.get_audio_contexts()):
                        self._retire_context(context_id)
                    await self._disconnect()
                    await self._connect()
                    if self._websocket is not None:
                        logger.info(
                            "ElevenLabs connection recovered attempt={}", attempt
                        )
                        return True
                except Exception as error:
                    logger.warning(
                        "ElevenLabs reconnect attempt={} failed: {}", attempt, error
                    )
                if attempt < self._hb_reconnect_attempts:
                    await asyncio.sleep(delay)
                    delay *= 2
            return False
