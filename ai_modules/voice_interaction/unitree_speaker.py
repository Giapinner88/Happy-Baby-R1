"""Pipecat processor that mirrors assistant audio to the Unitree R1 speaker bridge."""

import asyncio
import audioop
import os
import time
from pathlib import Path

from loguru import logger

from pipecat.audio.utils import create_stream_resampler
from pipecat.frames.frames import CancelFrame, EndFrame, Frame, OutputAudioRawFrame
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor


class UnitreeSpeakerBridge(FrameProcessor):
    """Mirror output audio frames to `unitree_bridge/r1_bridge speaker`.

    The bridge expects raw PCM s16le, 16 kHz, mono on stdin. Frames continue
    downstream unchanged, so the browser/debug transport still receives audio.
    """

    def __init__(
        self,
        *,
        bridge_path: str | Path,
        network_interface: str,
        app_name: str = "pipecat",
        sample_rate: int = 16000,
        audio_debug: bool = False,
    ):
        super().__init__()
        self._bridge_path = Path(bridge_path)
        self._network_interface = network_interface
        self._app_name = app_name
        self._sample_rate = sample_rate
        self._audio_debug = audio_debug
        self._process: asyncio.subprocess.Process | None = None
        self._stderr_task: asyncio.Task | None = None
        self._resampler = create_stream_resampler(quality="QQ")
        self._debug_last_log_at = time.monotonic()
        self._debug_speaker_bytes = 0
        self._debug_speaker_frames = 0

    def _track_audio(self, audio_bytes: int):
        if not self._audio_debug:
            return

        self._debug_speaker_bytes += audio_bytes
        self._debug_speaker_frames += 1

        now = time.monotonic()
        if now - self._debug_last_log_at < 1.0:
            return

        audio_ms = self._debug_speaker_bytes / (self._sample_rate * 2) * 1000
        logger.info(
            "Audio output debug: "
            f"robot_speaker={self._debug_speaker_bytes} bytes/"
            f"{self._debug_speaker_frames} frames/~{audio_ms:.0f}ms"
        )
        self._debug_last_log_at = now
        self._debug_speaker_bytes = 0
        self._debug_speaker_frames = 0

    async def _read_stderr(self, process: asyncio.subprocess.Process):
        while process.returncode is None and process.stderr:
            chunk = await process.stderr.read(4096)
            if not chunk:
                return
            logger.warning(
                f"Unitree speaker bridge stderr: {chunk.decode(errors='replace').strip()}"
            )

    async def _ensure_process(self) -> asyncio.subprocess.Process | None:
        if self._process and self._process.returncode is None:
            return self._process

        if not self._bridge_path.exists():
            logger.warning(f"Unitree speaker bridge is not built: {self._bridge_path}")
            return None

        env = os.environ.copy()
        self._process = await asyncio.create_subprocess_exec(
            str(self._bridge_path),
            "speaker",
            self._network_interface,
            self._app_name,
            stdin=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        logger.info(
            f"Started Unitree speaker bridge on {self._network_interface}: {self._bridge_path}"
        )
        self._stderr_task = self.create_task(self._read_stderr(self._process))
        return self._process

    async def _write_audio(self, frame: OutputAudioRawFrame):
        process = await self._ensure_process()
        if not process or not process.stdin:
            return

        audio = frame.audio
        if frame.num_channels == 2:
            audio = audioop.tomono(audio, 2, 0.5, 0.5)
        elif frame.num_channels != 1:
            logger.warning(f"Unsupported channel count for Unitree speaker: {frame.num_channels}")
            return

        audio = await self._resampler.resample(audio, frame.sample_rate, self._sample_rate)

        try:
            process.stdin.write(audio)
            await process.stdin.drain()
            self._track_audio(len(audio))
        except (BrokenPipeError, ConnectionResetError) as e:
            logger.warning(f"Unitree speaker bridge pipe closed: {e}")
            if process.stderr:
                stderr = await process.stderr.read()
                if stderr:
                    logger.warning(
                        "Unitree speaker bridge exited: "
                        f"{stderr.decode(errors='replace').strip()}"
                    )
            self._process = None

    async def _stop_process(self):
        if self._stderr_task:
            await self.cancel_task(self._stderr_task)
            self._stderr_task = None

        if not self._process:
            return

        process = self._process
        self._process = None

        if process.stdin:
            process.stdin.close()
            try:
                await process.stdin.wait_closed()
            except Exception:
                pass

        try:
            await asyncio.wait_for(process.wait(), timeout=2.0)
        except TimeoutError:
            process.terminate()

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)

        if direction is FrameDirection.DOWNSTREAM and isinstance(frame, OutputAudioRawFrame):
            await self._write_audio(frame)
        elif isinstance(frame, (CancelFrame, EndFrame)):
            await self._stop_process()

        await self.push_frame(frame, direction)
