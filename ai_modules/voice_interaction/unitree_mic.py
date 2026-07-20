"""Pipecat processor that streams audio from the Unitree R1 microphone bridge."""

import asyncio
import time
from pathlib import Path
from typing import Optional

from loguru import logger

from pipecat.audio.utils import create_stream_resampler
from pipecat.frames.frames import CancelFrame, EndFrame, Frame, InputAudioRawFrame, StartFrame
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor


class UnitreeMicBridge(FrameProcessor):
    """Stream audio from `unitree_bridge/r1_bridge mic` into the pipeline.

    The bridge prints PCM s16le, 16 kHz, mono to stdout, read from the R1's
    UDP mic multicast. Audio is resampled to `target_sample_rate` (the
    realtime LLM service's expected input rate) and queued downstream as
    InputAudioRawFrame, alongside whatever the transport's own input produces.
    """

    def __init__(
        self,
        *,
        bridge_path: str | Path,
        network_interface: str,
        source_sample_rate: int = 16000,
        target_sample_rate: int = 24000,
        read_chunk_bytes: int = 4096,
        segment_seconds: int = 5,
        mic_group_ip: Optional[str] = None,
        mic_port: Optional[int] = None,
        mic_payload_mode: Optional[str] = None,
        audio_debug: bool = False,
        drop_browser_audio: bool = False,
    ):
        super().__init__()
        self._bridge_path = Path(bridge_path)
        self._network_interface = network_interface
        self._source_sample_rate = source_sample_rate
        self._target_sample_rate = target_sample_rate
        self._read_chunk_bytes = read_chunk_bytes
        self._segment_seconds = segment_seconds
        self._mic_group_ip = mic_group_ip
        self._mic_port = mic_port
        self._mic_payload_mode = mic_payload_mode
        self._audio_debug = audio_debug
        self._drop_browser_audio = drop_browser_audio
        self._process: asyncio.subprocess.Process | None = None
        self._read_task: asyncio.Task | None = None
        self._stderr_task: asyncio.Task | None = None
        self._watchdog_task: asyncio.Task | None = None
        self._resampler = create_stream_resampler(quality="QQ")
        self._started_at = 0.0
        self._robot_total_bytes = 0
        self._debug_last_log_at = time.monotonic()
        self._debug_robot_bytes = 0
        self._debug_robot_frames = 0
        self._debug_browser_bytes = 0
        self._debug_browser_frames = 0

    def _track_audio(self, *, source: str, audio_bytes: int):
        if not self._audio_debug:
            return

        if source == "robot":
            self._robot_total_bytes += audio_bytes
            self._debug_robot_bytes += audio_bytes
            self._debug_robot_frames += 1
        else:
            self._debug_browser_bytes += audio_bytes
            self._debug_browser_frames += 1

        now = time.monotonic()
        if now - self._debug_last_log_at < 1.0:
            return

        robot_ms = self._debug_robot_bytes / (self._target_sample_rate * 2) * 1000
        browser_ms = self._debug_browser_bytes / (self._target_sample_rate * 2) * 1000
        logger.info(
            "Audio input debug: "
            f"robot_mic={self._debug_robot_bytes} bytes/"
            f"{self._debug_robot_frames} frames/~{robot_ms:.0f}ms, "
            f"browser_mic={self._debug_browser_bytes} bytes/"
            f"{self._debug_browser_frames} frames/~{browser_ms:.0f}ms, "
            f"drop_browser_audio={self._drop_browser_audio}, "
            f"bridge={self._bridge_status()}"
        )
        self._debug_last_log_at = now
        self._debug_robot_bytes = 0
        self._debug_robot_frames = 0
        self._debug_browser_bytes = 0
        self._debug_browser_frames = 0

    def _bridge_status(self) -> str:
        if not self._process:
            return "not_started"
        return f"pid={self._process.pid} returncode={self._process.returncode}"

    async def _start(self):
        if self._read_task:
            return

        if not self._bridge_path.exists():
            logger.warning(f"Unitree mic bridge is not built: {self._bridge_path}")
            return

        logger.info(
            "Starting Unitree mic bridge loop on "
            f"{self._network_interface}: {self._bridge_path} "
            f"(segment_seconds={self._segment_seconds})"
        )
        self._read_task = self.create_task(self._read_loop())

    async def _start_process(self) -> asyncio.subprocess.Process:
        args = [str(self._bridge_path), "mic", self._network_interface]
        if self._segment_seconds > 0:
            args.append(str(self._segment_seconds))
        if self._mic_group_ip:
            args.append(self._mic_group_ip)
            if self._mic_port or self._mic_payload_mode:
                args.append(str(self._mic_port or 5555))
            if self._mic_payload_mode:
                args.append(self._mic_payload_mode)

        self._process = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        self._started_at = time.monotonic()
        logger.info(
            f"Started Unitree mic bridge on {self._network_interface}: "
            f"{self._bridge_path} (pid={self._process.pid}, args={args})"
        )
        self._stderr_task = self.create_task(self._read_stderr(self._process))
        self._watchdog_task = self.create_task(self._watchdog(self._process))
        return self._process

    async def _stop(self):
        if self._read_task:
            await self.cancel_task(self._read_task)
            self._read_task = None
        if self._stderr_task:
            await self.cancel_task(self._stderr_task)
            self._stderr_task = None
        if self._watchdog_task:
            await self.cancel_task(self._watchdog_task)
            self._watchdog_task = None

        process = self._process
        self._process = None
        if not process:
            return

        if process.returncode is None:
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=2.0)
            except TimeoutError:
                process.kill()

    async def _read_stderr(self, process: asyncio.subprocess.Process):
        if not process.stderr:
            return

        while True:
            line = await process.stderr.readline()
            if not line:
                return
            logger.info(
                "Unitree mic bridge stderr: "
                f"{line.decode(errors='replace').strip()}"
            )

    async def _watchdog(self, process: asyncio.subprocess.Process):
        while process.returncode is None:
            await asyncio.sleep(3.0)

            if process.returncode is not None:
                break
            if self._robot_total_bytes > 0:
                continue

            elapsed = time.monotonic() - self._started_at
            logger.warning(
                "Unitree mic bridge is running but no robot mic audio has been received "
                f"after {elapsed:.1f}s ({self._bridge_status()})."
            )

        logger.warning(f"Unitree mic bridge process stopped ({self._bridge_status()}).")

    async def _read_loop(self):
        while True:
            process = await self._start_process()

            if not process or not process.stdout:
                logger.warning("Unitree mic bridge stdout is not available")
                return

            while True:
                chunk = await process.stdout.read(self._read_chunk_bytes)
                if not chunk:
                    logger.info(f"Unitree mic bridge stdout closed ({self._bridge_status()})")
                    break

                audio = await self._resampler.resample(
                    chunk, self._source_sample_rate, self._target_sample_rate
                )
                self._track_audio(source="robot", audio_bytes=len(audio))
                await self.queue_frame(
                    InputAudioRawFrame(
                        audio=audio,
                        sample_rate=self._target_sample_rate,
                        num_channels=1,
                    ),
                    FrameDirection.DOWNSTREAM,
                )

            await process.wait()
            if process.returncode != 0:
                logger.warning(
                    f"Unitree mic bridge exited with returncode={process.returncode}"
                )

            if self._stderr_task:
                await self.cancel_task(self._stderr_task)
                self._stderr_task = None
            if self._watchdog_task:
                await self.cancel_task(self._watchdog_task)
                self._watchdog_task = None

            if self._process is process:
                self._process = None

            await asyncio.sleep(0.05)

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)

        if isinstance(frame, StartFrame):
            await self.push_frame(frame, direction)
            await self._start()
            return
        elif isinstance(frame, (CancelFrame, EndFrame)):
            await self._stop()
        elif direction is FrameDirection.DOWNSTREAM and isinstance(frame, InputAudioRawFrame):
            self._track_audio(source="browser", audio_bytes=len(frame.audio))
            if self._drop_browser_audio:
                return

        await self.push_frame(frame, direction)
