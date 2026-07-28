"""Pipecat processors for the R1 multicast and future ALSA/USB microphones."""

import asyncio
import time
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Optional

from loguru import logger

from pipecat.audio.utils import create_stream_resampler
from pipecat.frames.frames import (
    CancelFrame,
    EndFrame,
    Frame,
    InputAudioRawFrame,
    StartFrame,
    UserStartedSpeakingFrame,
    UserStoppedSpeakingFrame,
)
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor

from .gate import AudioGate, GateSnapshot
from .resilience import PttTurnAudioBuffer, apply_pcm16_gain, pcm_bytes_for_ms


TurnCommitBarrier = Callable[[], Awaitable[bool]]


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
        segment_seconds: int = 0,
        mic_group_ip: Optional[str] = None,
        mic_port: Optional[int] = None,
        mic_payload_mode: Optional[str] = None,
        input_gain_db: float = 0.0,
        audio_debug: bool = False,
        gate: AudioGate | None = None,
        source_name: str = "r1_multicast",
        minimum_ptt_audio_ms: int = 100,
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
        self._input_gain_db = input_gain_db
        self._audio_debug = audio_debug
        self._gate = gate
        self._source_name = source_name
        self._process: asyncio.subprocess.Process | None = None
        self._read_task: asyncio.Task | None = None
        self._stderr_task: asyncio.Task | None = None
        self._watchdog_task: asyncio.Task | None = None
        self._resampler = create_stream_resampler(quality="QQ")
        self._started_at = 0.0
        self._last_robot_audio_at = 0.0
        self._robot_total_bytes = 0
        self._debug_last_log_at = time.monotonic()
        self._debug_robot_bytes = 0
        self._debug_robot_frames = 0
        self._debug_browser_bytes = 0
        self._debug_browser_frames = 0
        self._pipeline_ready = False
        self._turn_active = False
        self._turn_commit_barrier: TurnCommitBarrier | None = None
        self._minimum_ptt_audio_ms = minimum_ptt_audio_ms
        self._turn_audio = PttTurnAudioBuffer(
            pcm_bytes_for_ms(self._target_sample_rate, minimum_ptt_audio_ms)
        )
        if self._gate:
            self._gate.add_callback(self._on_gate_change)

    def set_turn_commit_barrier(self, barrier: TurnCommitBarrier) -> None:
        """Install a callback that drains downstream audio before PTT commit."""
        self._turn_commit_barrier = barrier

    @property
    def health_ready(self) -> bool:
        return bool(
            self._process
            and self._process.returncode is None
            and self._last_robot_audio_at
            and time.monotonic() - self._last_robot_audio_at < 4.5
        )

    async def _on_gate_change(self, old: GateSnapshot, new: GateSnapshot):
        if not self._pipeline_ready:
            return
        if new.mic_allowed:
            if not self._turn_audio.capturing:
                self._turn_audio.begin()
                logger.info("PTT capture started; waiting for microphone audio")
            return

        if not self._turn_audio.capturing:
            return
        if new.ptt:
            self._turn_active = False
            self._turn_audio.abort()
            return

        should_commit, audio_bytes = self._turn_audio.finish()
        audio_ms = audio_bytes / (self._target_sample_rate * 2) * 1000
        if should_commit and self._turn_active:
            if self._turn_commit_barrier:
                try:
                    drained = await self._turn_commit_barrier()
                except Exception as error:
                    drained = False
                    logger.exception(f"PTT audio drain failed: {error}")
                if not drained:
                    self._turn_active = False
                    logger.error(
                        "PTT turn aborted because queued audio did not drain; "
                        "the voice session will reconnect"
                    )
                    return
            self._turn_active = False
            await self.push_frame(UserStoppedSpeakingFrame(), FrameDirection.DOWNSTREAM)
            logger.info(f"PTT turn committed with {audio_ms:.0f}ms audio")
        else:
            self._turn_active = False
            logger.warning(
                "PTT turn skipped: microphone supplied "
                f"{audio_ms:.0f}ms, minimum is {self._minimum_ptt_audio_ms}ms"
            )

    def _track_audio(self, *, source: str, audio_bytes: int):
        # Watchdog cần bộ đếm này cả khi production đã tắt log audio debug.
        if source == "robot":
            self._robot_total_bytes += audio_bytes
            self._last_robot_audio_at = time.monotonic()
        if not self._audio_debug:
            return

        if source == "robot":
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
            "browser_audio=disabled, "
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
            f"Starting {self._source_name} mic loop on "
            f"{self._network_interface}: {self._bridge_path} "
            f"(segment_seconds={self._segment_seconds}, gain={self._input_gain_db}dB)"
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
        self._last_robot_audio_at = self._started_at
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
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()

    async def _read_stderr(self, process: asyncio.subprocess.Process):
        if not process.stderr:
            return

        while True:
            line = await process.stderr.readline()
            if not line:
                return
            logger.info(
                f"Unitree mic bridge stderr: {line.decode(errors='replace').strip()}"
            )

    async def _watchdog(self, process: asyncio.subprocess.Process):
        while process.returncode is None:
            await asyncio.sleep(3.0)

            if process.returncode is not None:
                break
            silence = time.monotonic() - self._last_robot_audio_at
            if silence < 3.0:
                continue

            elapsed = time.monotonic() - self._started_at
            logger.warning(
                "Unitree mic bridge has no robot mic audio "
                f"for {silence:.1f}s after {elapsed:.1f}s ({self._bridge_status()})."
            )
            if silence >= 9.0:
                logger.warning("Restarting stalled Unitree mic bridge")
                process.terminate()
                return

        logger.warning(f"Unitree mic bridge process stopped ({self._bridge_status()}).")

    async def _read_loop(self):
        restart_delay = 0.5
        while True:
            bytes_before_start = self._robot_total_bytes
            process = await self._start_process()

            if not process or not process.stdout:
                logger.warning("Unitree mic bridge stdout is not available")
                return

            while True:
                chunk = await process.stdout.read(self._read_chunk_bytes)
                if not chunk:
                    logger.info(
                        f"{self._source_name} mic stdout closed ({self._bridge_status()})"
                    )
                    break

                audio = await self._resampler.resample(
                    chunk, self._source_sample_rate, self._target_sample_rate
                )
                audio = apply_pcm16_gain(audio, self._input_gain_db)
                self._track_audio(source="robot", audio_bytes=len(audio))
                if self._gate and not self._gate.snapshot.mic_allowed:
                    continue
                frame = InputAudioRawFrame(
                    audio=audio,
                    sample_rate=self._target_sample_rate,
                    num_channels=1,
                )
                if not self._gate:
                    await self.queue_frame(frame, FrameDirection.DOWNSTREAM)
                    continue
                was_ready = self._turn_audio.ready
                frames = self._turn_audio.add(frame, len(audio))
                if frames and not was_ready:
                    self._turn_active = True
                    await self.push_frame(
                        UserStartedSpeakingFrame(), FrameDirection.DOWNSTREAM
                    )
                    logger.info(
                        "PTT turn started after receiving "
                        f"{self._turn_audio.audio_bytes / (self._target_sample_rate * 2) * 1000:.0f}ms audio"
                    )
                for buffered_frame in frames:
                    # Robot audio originates in this processor. Pushing it
                    # directly avoids re-entering process_frame(), where
                    # downstream InputAudioRawFrame is intentionally treated
                    # as browser audio and dropped in production.
                    await self.push_frame(buffered_frame, FrameDirection.DOWNSTREAM)

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

            if self._robot_total_bytes > bytes_before_start:
                restart_delay = 0.5
            else:
                restart_delay = min(restart_delay * 2.0, 5.0)
            logger.info(f"Mic bridge restart in {restart_delay:.1f}s")
            await asyncio.sleep(restart_delay)

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)

        if isinstance(frame, StartFrame):
            await self.push_frame(frame, direction)
            self._pipeline_ready = True
            await self._start()
            if self._gate:
                await self._on_gate_change(GateSnapshot(), self._gate.snapshot)
            return
        elif isinstance(frame, (CancelFrame, EndFrame)):
            self._pipeline_ready = False
            self._turn_active = False
            self._turn_audio.abort()
            await self._stop()
        elif direction is FrameDirection.DOWNSTREAM and isinstance(
            frame, InputAudioRawFrame
        ):
            self._track_audio(source="browser", audio_bytes=len(frame.audio))
            return

        await self.push_frame(frame, direction)


class AlsaMicBridge(UnitreeMicBridge):
    """ALSA/USB microphone adapter using a stable card name."""

    def __init__(
        self,
        *,
        device: str,
        sample_rate: int,
        gate: AudioGate,
        input_gain_db: float = 0.0,
        audio_debug: bool = False,
    ):
        if device.startswith("hw:") and "CARD=" not in device:
            raise ValueError(
                "ALSA_DEVICE must use a stable card name such as "
                "plughw:CARD=RobotMic,DEV=0; numeric hw:1,0 is not allowed"
            )
        self._alsa_device = device
        super().__init__(
            bridge_path=Path("/usr/bin/arecord"),
            network_interface="alsa",
            source_sample_rate=sample_rate,
            segment_seconds=0,
            input_gain_db=input_gain_db,
            audio_debug=audio_debug,
            gate=gate,
            source_name="alsa_usb",
        )

    async def _start_process(self) -> asyncio.subprocess.Process:
        args = [
            str(self._bridge_path),
            "-q",
            "-D",
            self._alsa_device,
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            str(self._source_sample_rate),
            "-c",
            "1",
        ]
        self._process = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        self._started_at = time.monotonic()
        self._last_robot_audio_at = self._started_at
        logger.info(
            f"Started ALSA mic: device={self._alsa_device} pid={self._process.pid}"
        )
        self._stderr_task = self.create_task(self._read_stderr(self._process))
        self._watchdog_task = self.create_task(self._watchdog(self._process))
        return self._process
