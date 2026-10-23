"""Keep tool-call preambles out of the external TTS stream.

Realtime may emit text before announcing a function call in the same response.
Only response.done tells us whether that text belongs to a tool request. Holding
text until then prevents premature spoken acknowledgements; the response after
the tool result is still delivered normally. Native Realtime audio is untouched.
"""

from loguru import logger


class GestureResponseGuard:
    """Mixin for Pipecat's Realtime service, enabled only for external TTS."""

    def __init__(
        self, *args, guard_gesture_text: bool = False, latency_tracker=None, **kwargs
    ):
        self._guard_gesture_text = guard_gesture_text
        self._latency_tracker = latency_tracker
        self._gesture_response_text = {}
        self._discarded_gesture_responses = set()
        super().__init__(*args, **kwargs)

    def set_gesture_text_guard(self, enabled: bool) -> None:
        """Enable buffering only while gesture tools are actually advertised."""
        if self._guard_gesture_text == enabled:
            return
        self._guard_gesture_text = enabled
        if not enabled:
            # A response that began while the tool was available must never
            # resume speaking after Dev is exited.
            self._discarded_gesture_responses.update(self._gesture_response_text)
            self._gesture_response_text.clear()

    async def _handle_evt_text_delta(self, evt):
        if self._latency_tracker:
            self._latency_tracker.mark("first_model_text")
        if not self._guard_gesture_text:
            await super()._handle_evt_text_delta(evt)
            return
        if evt.response_id in self._discarded_gesture_responses:
            return
        self._gesture_response_text.setdefault(evt.response_id, []).append(evt)

    async def _handle_evt_response_done(self, evt):
        chunks = self._gesture_response_text.pop(evt.response.id, [])
        self._discarded_gesture_responses.discard(evt.response.id)
        tool_names = [
            item.name for item in evt.response.output if item.type == "function_call"
        ]
        if tool_names:
            logger.info(
                "Realtime tool response={} tools={} suppressed_text_chunks={}",
                evt.response.id, tool_names, len(chunks),
            )
            # The controller/camera result will create a second response. Its
            # text is safe to stream because the tool decision has already
            # completed; keeping the guard enabled here delays the real reply
            # until response.done and makes Dev conversations sound sluggish.
            self.set_gesture_text_guard(False)
        elif evt.response.status == "completed":
            for chunk in chunks:
                await super()._handle_evt_text_delta(chunk)
        # Flush before Pipecat emits LLMFullResponseEndFrame (TTS/aggregation).
        await super()._handle_evt_response_done(evt)

    def set_turn_gesture_guard(self, enabled: bool) -> None:
        """Select buffering for the current user turn, not the whole Dev session."""
        self.set_gesture_text_guard(enabled)

    @property
    def turn_gesture_guard_enabled(self) -> bool:
        return self._guard_gesture_text

    async def _handle_interruption(self):
        self._discarded_gesture_responses.update(self._gesture_response_text)
        self._gesture_response_text.clear()
        await super()._handle_interruption()

    async def _disconnect(self):
        self._gesture_response_text.clear()
        self._discarded_gesture_responses.clear()
        await super()._disconnect()
