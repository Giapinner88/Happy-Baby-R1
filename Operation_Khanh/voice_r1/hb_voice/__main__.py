"""Single production entry point for the browser-free voice runtime."""

import asyncio
import signal
import sys

from loguru import logger

from pipecat.processors.frame_processor import FrameProcessor
from pipecat.transports.base_input import BaseInputTransport
from pipecat.transports.base_output import BaseOutputTransport
from pipecat.transports.base_transport import BaseTransport, TransportParams

from .app import run_bot
from .config import VoiceConfig
from .resilience import VoiceRuntimeStatus


class _HeadlessInput(BaseInputTransport):
    def __init__(self, owner: "RobotHeadlessTransport", params: TransportParams):
        super().__init__(params)
        self._owner = owner

    async def start(self, frame):
        await super().start(frame)
        await self.set_transport_ready(frame)
        await self._owner._connected()


class _HeadlessOutput(BaseOutputTransport):
    async def start(self, frame):
        await super().start(frame)
        await self.set_transport_ready(frame)


class RobotHeadlessTransport(BaseTransport):
    """Lifecycle transport; the R1/ALSA processors carry the actual audio."""

    def __init__(self):
        super().__init__(name="RobotHeadlessTransport")
        self._params = TransportParams(audio_in_enabled=False, audio_out_enabled=False)
        self._input = None
        self._output = None
        self._did_connect = False
        self._register_event_handler("on_client_connected")
        self._register_event_handler("on_client_disconnected")

    async def _connected(self):
        if not self._did_connect:
            self._did_connect = True
            await self._call_event_handler("on_client_connected", "robot-headless")

    def input(self) -> FrameProcessor:
        if self._input is None:
            self._input = _HeadlessInput(self, self._params)
        return self._input

    def output(self) -> FrameProcessor:
        if self._output is None:
            self._output = _HeadlessOutput(self._params)
        return self._output


async def run_session(
    config: VoiceConfig, runtime_status: VoiceRuntimeStatus, attempt: int
) -> bool:
    return await run_bot(RobotHeadlessTransport(), config, runtime_status, attempt)


async def supervise() -> None:
    loop = asyncio.get_running_loop()
    stopping = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopping.set)

    config = VoiceConfig.load()
    runtime_status = VoiceRuntimeStatus()
    delay = config.reconnect_initial_s
    attempt = 0
    while not stopping.is_set():
        attempt += 1
        runtime_status.update("starting", attempt=attempt)
        logger.info(f"Starting headless voice session attempt={attempt}")
        session = asyncio.create_task(run_session(config, runtime_status, attempt))
        stop_waiter = asyncio.create_task(stopping.wait())
        done, _ = await asyncio.wait(
            {session, stop_waiter}, return_when=asyncio.FIRST_COMPLETED
        )
        if stop_waiter in done:
            session.cancel()
            await asyncio.gather(session, return_exceptions=True)
            runtime_status.update("stopped", attempt=attempt, reason="service_stopped")
            return

        stop_waiter.cancel()
        await asyncio.gather(stop_waiter, return_exceptions=True)
        connected_once = False
        try:
            connected_once = session.result()
            logger.warning("Voice session ended; reconnecting")
            if connected_once:
                delay = config.reconnect_initial_s
        except Exception as error:
            reason = (
                "session_exception" if runtime_status.last_reason == "none" else None
            )
            runtime_status.update("reconnecting", attempt=attempt, reason=reason)
            logger.exception(f"Voice session failed: {error}")
        try:
            await asyncio.wait_for(stopping.wait(), timeout=delay)
        except asyncio.TimeoutError:
            pass
        if not connected_once:
            delay = min(delay * 2.0, config.reconnect_max_s)


def main() -> None:
    if sys.argv[1:] == ["--check-config"]:
        config = VoiceConfig.load()
        config.load_prompt()
        print(f"VOICE_CONFIG_OK {config.safe_summary()}")
        return
    if sys.argv[1:]:
        raise SystemExit("Usage: python -m hb_voice [--check-config]")
    asyncio.run(supervise())


if __name__ == "__main__":
    main()
