"""Pipecat processor that mirrors assistant audio to the Unitree R1 speaker bridge."""

import asyncio
import os
import sys
import time
from array import array
from pathlib import Path

from loguru import logger

from pipecat.audio.utils import create_stream_resampler
from pipecat.frames.frames import CancelFrame, EndFrame, Frame, OutputAudioRawFrame
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor

from .gate import AudioGate, GateSnapshot


class UnitreeSpeakerBridge(FrameProcessor):
    """Mirror output audio frames to `unitree_bridge/r1_bridge speaker`.

    The bridge expects raw PCM s16le, 16 kHz, mono on stdin. Frames continue
    downstream unchanged for the headless transport lifecycle.
    """

    def __init__(
        self,
        *,
        bridge_path: str | Path,
        network_interface: str,
        response_volume_percent: int,
        response_gain: float,
        app_name: str = "pipecat",
        sample_rate: int = 16000,
        audio_debug: bool = False,
        gate: AudioGate | None = None,
    ):
        super().__init__()
        self._bridge_path = Path(bridge_path)
        self._network_interface = network_interface
        self._app_name = app_name
        self._sample_rate = sample_rate
        self._response_volume_percent = response_volume_percent
        self._response_gain = response_gain
        self._audio_debug = audio_debug
        self._gate = gate
        self._process: asyncio.subprocess.Process | None = None
        self._stderr_task: asyncio.Task | None = None
        self._resampler = create_stream_resampler(quality="QQ")
        self._debug_last_log_at = time.monotonic()
        self._debug_speaker_bytes = 0
        self._debug_speaker_frames = 0
        self._voice_idle_task: asyncio.Task | None = None
        if self._gate:
            self._gate.add_callback(self._on_gate_change)

    async def _on_gate_change(self, old: GateSnapshot, new: GateSnapshot):
        if old.speaker_allowed and not new.speaker_allowed:
            await self._preempt_process()

    async def _mark_voice_speaking(self):
        if not self._gate:
            return
        await self._gate.set_voice_speaking(True)
        if self._voice_idle_task:
            await self.cancel_task(self._voice_idle_task)
        self._voice_idle_task = self.create_task(self._clear_voice_after_idle())

    async def _clear_voice_after_idle(self):
        await asyncio.sleep(0.45)
        if self._gate:
            await self._gate.set_voice_speaking(False)
        self._voice_idle_task = None

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
            line = await process.stderr.readline()
            if not line:
                return
            message = line.decode(errors="replace").strip()
            if message.startswith("r1_bridge speaker ready:"):
                logger.info(f"Unitree speaker bridge: {message}")
            else:
                logger.warning(f"Unitree speaker bridge stderr: {message}")

    async def _ensure_process(self) -> asyncio.subprocess.Process | None:
        if self._gate and not self._gate.snapshot.speaker_allowed:
            return None
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
            str(self._response_volume_percent),
            stdin=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        logger.info(
            f"Started Unitree speaker bridge on {self._network_interface}: "
            f"{self._bridge_path} (response_volume={self._response_volume_percent}%, "
            f"response_gain={self._response_gain})"
        )
        self._stderr_task = self.create_task(self._read_stderr(self._process))
        return self._process

    async def _write_audio(self, frame: OutputAudioRawFrame):
        if self._gate and not self._gate.snapshot.speaker_allowed:
            return
        process = await self._ensure_process()
        if not process or not process.stdin:
            return

        audio = frame.audio
        if frame.num_channels == 2:
            stereo = array("h")
            stereo.frombytes(audio)
            if sys.byteorder != "little":
                stereo.byteswap()
            mono = array(
                "h",
                (
                    (stereo[i] + stereo[i + 1]) // 2
                    for i in range(0, len(stereo) - 1, 2)
                ),
            )
            if sys.byteorder != "little":
                mono.byteswap()
            audio = mono.tobytes()
        elif frame.num_channels != 1:
            logger.warning(
                f"Unsupported channel count for Unitree speaker: {frame.num_channels}"
            )
            return

        audio = await self._resampler.resample(
            audio, frame.sample_rate, self._sample_rate
        )
        audio = self._apply_gain(audio)

        try:
            process.stdin.write(audio)
            await process.stdin.drain()
            self._track_audio(len(audio))
            await self._mark_voice_speaking()
        except (BrokenPipeError, ConnectionResetError) as error:
            logger.warning(f"Unitree speaker bridge pipe closed: {error}")
            await self._finish_process(process, graceful=False)

    def _apply_gain(self, audio: bytes) -> bytes:
        if self._response_gain == 1.0 or not audio:
            return audio
        samples = array("h")
        samples.frombytes(audio)
        if sys.byteorder != "little":
            samples.byteswap()
        for index, sample in enumerate(samples):
            samples[index] = int(sample * self._response_gain)
        if sys.byteorder != "little":
            samples.byteswap()
        return samples.tobytes()

    async def _preempt_process(self):
        if self._voice_idle_task:
            await self.cancel_task(self._voice_idle_task)
            self._voice_idle_task = None
        process = self._process
        self._process = None
        if process:
            await self._finish_process(process, graceful=False)
        if self._gate:
            await self._gate.set_voice_speaking(False)

    async def _finish_process(
        self, process: asyncio.subprocess.Process, *, graceful: bool
    ) -> None:
        if self._stderr_task:
            await self.cancel_task(self._stderr_task)
            self._stderr_task = None
        if self._process is process:
            self._process = None
        if graceful and process.stdin:
            process.stdin.close()
            try:
                await process.stdin.wait_closed()
            except Exception:
                pass
        if process.returncode is None and not graceful:
            process.terminate()
        if process.returncode is None:
            try:
                await asyncio.wait_for(process.wait(), timeout=2.0 if graceful else 1.0)
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()

    async def _stop_process(self):
        if self._voice_idle_task:
            await self.cancel_task(self._voice_idle_task)
            self._voice_idle_task = None
        process = self._process
        if process:
            await self._finish_process(process, graceful=True)
        if self._gate:
            await self._gate.set_voice_speaking(False)

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)

        if direction is FrameDirection.DOWNSTREAM and isinstance(
            frame, OutputAudioRawFrame
        ):
            await self._write_audio(frame)
        elif isinstance(frame, (CancelFrame, EndFrame)):
            await self._stop_process()

        await self.push_frame(frame, direction)
