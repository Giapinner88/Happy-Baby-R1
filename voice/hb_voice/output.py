"""Pipecat processor that mirrors assistant audio to the Unitree R1 speaker bridge."""

import asyncio
import math
import os
import sys
import time
from array import array
from pathlib import Path

from loguru import logger

from pipecat.audio.utils import create_stream_resampler
from pipecat.frames.frames import (
    BotStartedSpeakingFrame,
    BotStoppedSpeakingFrame,
    CancelFrame,
    EndFrame,
    Frame,
    InterruptionFrame,
    OutputAudioRawFrame,
    TTSStartedFrame,
    TTSStoppedFrame,
)
from pipecat.processors.frame_processor import FrameDirection, FrameProcessor

from .gate import AudioGate, GateSnapshot


# Bốn tín hiệu phải phân biệt được QUA PHÒNG ỒN, không cần nhìn log.
#
# Phân biệt chính bằng SỐ TIẾNG và ĐỘ DÀI, không phải hướng cao độ: tai người
# đếm tiếng rất tốt nhưng nghe hướng cao độ rất kém khi ồn và ở xa. Bản cũ dựa
# hẳn vào hướng (660->880 với 880->660) và một nốt đơn cao/thấp, nên cả bốn
# nghe như nhau — "tít tít không biết đâu mà lần".
#
#   F1 (chế độ nghe) = 3 nốt   -> luôn nhiều tiếng nhất
#        bật rảnh tay : đi LÊN  do-mi-sol
#        về bấm giữ   : đi XUỐNG sol-mi-do
#   F2 (nguồn mic)   = 1 hoặc 2 tiếng
#        mic đeo      : 2 tiếng NGẮN, cao
#        mic thân robot: 1 tiếng DÀI, trầm
#
# Nghe được số tiếng là đã biết bấm trúng nút nào (3 = F1, 1-2 = F2), khỏi phải
# phân biệt cao độ; hướng cao độ chỉ còn là lớp xác nhận thứ hai.
_CUE_AMPLITUDE = 3600

# Loa nhỏ hụt bass và tai người nghe tần số thấp nhỏ hơn ở cùng mức: bù biên độ
# cho nốt trầm, không thì tiếng "mic thân robot" chìm mất giữa phòng ồn.
_LOW_TONE_BOOST = 1.4


def _tone_sequence_pcm(
    notes: tuple[tuple[float, float, float], ...],
    *,
    sample_rate: int,
    gap_s: float,
) -> bytes:
    """PCM s16le mono cho một chuỗi nốt (tần số Hz, độ dài giây, hệ số biên độ)."""
    fade_s = 0.006
    samples = array("h")
    for note_index, (frequency, duration_s, amplitude_scale) in enumerate(notes):
        tone_samples = max(1, int(sample_rate * duration_s))
        fade_samples = max(1, int(sample_rate * fade_s))
        peak = min(32000, int(_CUE_AMPLITUDE * amplitude_scale))
        for index in range(tone_samples):
            fade = min(1.0, index / fade_samples, (tone_samples - 1 - index) / fade_samples)
            amplitude = int(peak * max(0.0, fade))
            samples.append(
                int(amplitude * math.sin(2.0 * math.pi * frequency * index / sample_rate))
            )
        if note_index < len(notes) - 1:
            samples.extend([0] * max(1, int(sample_rate * gap_s)))
    if sys.byteorder != "little":
        samples.byteswap()
    return samples.tobytes()


def mode_cue_pcm(enabled: bool, *, sample_rate: int = 16000) -> bytes:
    """Cue BA nốt cho F1 — nhiều tiếng nhất nên không lẫn với F2.

    Đi lên = robot tự nghe (rảnh tay); đi xuống = chỉ mở mic khi giữ Select.
    Ba nốt chứ không phải hai: qua ba nốt thì hướng lên/xuống nghe rõ hẳn, còn
    hai nốt cách nhau một quãng ba thì ở xa gần như không phân biệt được.
    """
    triad = (523.25, 659.25, 783.99)  # do-mi-sol, nằm gọn trong dải loa nhỏ
    if not enabled:
        triad = tuple(reversed(triad))
    return _tone_sequence_pcm(
        tuple((frequency, 0.085, 1.0) for frequency in triad),
        sample_rate=sample_rate,
        gap_s=0.025,
    )


def mic_source_cue_pcm(external: bool, *, sample_rate: int = 16000) -> bytes:
    """Cue cho F2 — 2 tiếng ngắn cao (mic đeo) hoặc 1 tiếng dài trầm (mic thân).

    Số tiếng và độ dài là thứ tách nó khỏi cue ba nốt của F1; ai nghe cũng phân
    biệt được "hai tiếng cộc lốc" với "một tiếng kéo dài" mà không cần luyện tai.
    """
    if external:
        notes = ((1318.51, 0.06, 1.0), (1318.51, 0.06, 1.0))
        gap_s = 0.05
    else:
        notes = ((392.00, 0.30, _LOW_TONE_BOOST),)
        gap_s = 0.0
    return _tone_sequence_pcm(notes, sample_rate=sample_rate, gap_s=gap_s)


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
        speaking_marker_path: str | Path = "/run/hb/voice_speaking",
        playback_tail_s: float = 0.6,
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
        self._speaking_marker_path = Path(speaking_marker_path)
        self._marker_warning_logged = False
        self._process: asyncio.subprocess.Process | None = None
        self._stderr_task: asyncio.Task | None = None
        self._resampler = create_stream_resampler(quality="QQ")
        # Ngắt lượt KHÔNG rút lại được luồng ElevenLabs đang bay: lệnh huỷ tới OpenAI
        # thường rơi vào lúc nó đã trả lời xong ("Cancellation failed: no active
        # response found"), nên audio của lượt cũ vẫn đổ về sau khi lượt mới đã bắt
        # đầu. Hai luồng ghi chung một ống loa = nghe như robot đổi giọng giữa câu.
        # Chốt cờ này lúc bị ngắt và chỉ nhả ở TTSStartedFrame kế tiếp (mốc mở đầu
        # của lượt MỚI), để đuôi lượt cũ bị vứt thay vì phát đè.
        self._drop_until_next_tts = False
        # Hai nguồn cùng gọi _write_audio (pipeline TTS và cue F1/F2) mà chung một
        # stream resampler có trạng thái + chung một stdin: không tuần tự hoá thì các
        # mẩu PCM xen kẽ nhau, đúng thứ tạo ra cảm giác "lật giọng".
        self._write_lock = asyncio.Lock()
        self._debug_last_log_at = time.monotonic()
        self._debug_speaker_bytes = 0
        self._debug_speaker_frames = 0
        self._voice_idle_task: asyncio.Task | None = None
        self._playback_deadline = 0.0
        self._marker_refresh_s = 0.25
        # Grace period after the queued PCM should have finished playing.  It
        # covers what this process cannot observe: the DDS hop to PC1, the
        # speaker's own jitter buffer, and room reverb.  Too short and the mic
        # reopens while the robot is still audible, which in hands-free mode
        # lets the robot answer itself.
        self._playback_tail_s = playback_tail_s
        self._bot_speaking = False
        if self._gate:
            self._gate.add_callback(self._on_gate_change)

    async def _on_gate_change(self, old: GateSnapshot, new: GateSnapshot):
        if (
            old.speaker_allowed and not new.speaker_allowed
        ) or (new.interrupt and not old.interrupt):
            # Double Select is an unconditional emergency stop for the voice
            # channel.  Killing the local speaker bridge flushes PCM that was
            # already queued, which a Realtime response cancellation alone
            # cannot retract.
            await self._preempt_process()

    def _extend_playback_deadline(self, audio_bytes: int):
        """Estimate when PCM already handed to the bridge will finish playing.

        ElevenLabs can produce audio faster than the robot speaker consumes it.
        Consequently, TTSStoppedFrame means "generation ended", not "the robot
        speaker is silent".  Keep the marker alive for the queued PCM duration.
        """
        now = time.monotonic()
        audio_s = max(0, audio_bytes) / float(self._sample_rate * 2)
        self._playback_deadline = max(now, self._playback_deadline) + audio_s

    async def _mark_voice_speaking(self, audio_bytes: int):
        self._extend_playback_deadline(audio_bytes)
        self._touch_speaking_marker()
        if not self._voice_idle_task:
            # Claim the slot before the awaits below: two writers interleaving
            # there would each spawn an idle task, and the loser would clear
            # voice_speaking out from under the winner.
            self._voice_idle_task = self.create_task(self._clear_voice_after_idle())
        if not self._bot_speaking:
            self._bot_speaking = True
            await self.broadcast_frame(BotStartedSpeakingFrame)
        if self._gate:
            await self._gate.set_voice_speaking(True)

    async def _clear_voice_after_idle(self):
        # Refresh mtime while the speaker bridge drains queued PCM.  run_r1
        # deliberately rejects a stale marker, so merely leaving the file in
        # place is insufficient for responses longer than gesture_voice_stale_s.
        try:
            while True:
                remaining = (
                    self._playback_deadline + self._playback_tail_s - time.monotonic()
                )
                if remaining > 0:
                    await asyncio.sleep(min(self._marker_refresh_s, remaining))
                    if self._playback_deadline + self._playback_tail_s > time.monotonic():
                        self._touch_speaking_marker()
                    continue

                if self._gate:
                    await self._gate.set_voice_speaking(False)
                await self._mark_bot_stopped()
                # Releasing the flag takes several awaits, and _mark_voice_speaking
                # only spawns a replacement for this task when the slot is empty -
                # which it is not until the `finally` below.  Audio that landed in
                # that window extended the deadline synchronously, so re-check it:
                # returning now would latch voice_speaking with nobody left to
                # clear it and the microphone would never reopen.
                if self._playback_deadline + self._playback_tail_s <= time.monotonic():
                    return
        finally:
            self._voice_idle_task = None

    async def _mark_bot_stopped(self):
        self._clear_speaking_marker()
        if not self._bot_speaking:
            return
        self._bot_speaking = False
        await self.broadcast_frame(BotStoppedSpeakingFrame)

    def _touch_speaking_marker(self):
        try:
            self._speaking_marker_path.parent.mkdir(parents=True, exist_ok=True)
            self._speaking_marker_path.touch(exist_ok=True)
            self._marker_warning_logged = False
        except OSError as error:
            if not self._marker_warning_logged:
                logger.warning(
                    f"Cannot update voice speaking marker "
                    f"{self._speaking_marker_path}: {error}"
                )
                self._marker_warning_logged = True

    def _clear_speaking_marker(self):
        try:
            self._speaking_marker_path.unlink(missing_ok=True)
            self._marker_warning_logged = False
        except OSError as error:
            if not self._marker_warning_logged:
                logger.warning(
                    f"Cannot clear voice speaking marker "
                    f"{self._speaking_marker_path}: {error}"
                )
                self._marker_warning_logged = True

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
        if self._gate:
            snapshot = self._gate.snapshot
            if not snapshot.speaker_allowed or snapshot.interrupt:
                return
        # Giữ khoá quanh CẢ resample lẫn ghi: resampler là bộ lọc có trạng thái nên
        # hai lời gọi đan nhau vừa làm hỏng trạng thái lọc vừa trộn PCM trong ống.
        async with self._write_lock:
            await self._write_audio_locked(frame)

    async def _write_audio_locked(self, frame: OutputAudioRawFrame):
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
                ((stereo[i] + stereo[i + 1]) // 2 for i in range(0, len(stereo) - 1, 2)),
            )
            if sys.byteorder != "little":
                mono.byteswap()
            audio = mono.tobytes()
        elif frame.num_channels != 1:
            logger.warning(f"Unsupported channel count for Unitree speaker: {frame.num_channels}")
            return

        audio = await self._resampler.resample(audio, frame.sample_rate, self._sample_rate)
        audio = self._apply_gain(audio)

        try:
            process.stdin.write(audio)
            await process.stdin.drain()
            self._track_audio(len(audio))
            await self._mark_voice_speaking(len(audio))
        except (BrokenPipeError, ConnectionResetError) as error:
            logger.warning(f"Unitree speaker bridge pipe closed: {error}")
            await self._finish_process(process, graceful=False)

    async def play_mode_cue(self, enabled: bool) -> bool:
        """Play the local conversation on/off cue if voice owns the speaker."""
        if self._gate and not self._gate.snapshot.speaker_allowed:
            logger.info("Skipping conversation mode cue: speaker is preempted")
            return False
        await self._write_audio(
            OutputAudioRawFrame(
                audio=mode_cue_pcm(enabled, sample_rate=self._sample_rate),
                sample_rate=self._sample_rate,
                num_channels=1,
            )
        )
        return True

    async def play_mic_source_cue(self, external: bool) -> bool:
        """Confirm an F2 press out loud if voice owns the speaker."""
        if self._gate and not self._gate.snapshot.speaker_allowed:
            logger.info("Skipping microphone source cue: speaker is preempted")
            return False
        await self._write_audio(
            OutputAudioRawFrame(
                audio=mic_source_cue_pcm(external, sample_rate=self._sample_rate),
                sample_rate=self._sample_rate,
                num_channels=1,
            )
        )
        return True

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
        await self._mark_bot_stopped()
        if self._gate:
            await self._gate.set_voice_speaking(False)

    async def _finish_process(self, process: asyncio.subprocess.Process, *, graceful: bool) -> None:
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
        await self._mark_bot_stopped()

    async def process_frame(self, frame: Frame, direction: FrameDirection):
        await super().process_frame(frame, direction)

        if isinstance(frame, InterruptionFrame):
            # Vứt mọi audio còn bay tới từ lượt vừa bị huỷ, tới khi lượt MỚI mở màn.
            # _preempt_process() (đường double-tap Select) chỉ xả được PCM ĐÃ xếp
            # hàng; nó không chặn được frame đến muộn — những frame đó sẽ được ghi
            # thẳng vào tiến trình bridge mới và phát đè lên câu trả lời mới.
            self._drop_until_next_tts = True
            # Trạng thái bộ lọc thuộc về luồng vừa bị bỏ; mang sang lượt sau là méo.
            # Tráo dưới khoá: cue F1/F2 có thể đang resample ngay lúc này.
            async with self._write_lock:
                self._resampler = create_stream_resampler(quality="QQ")
        elif isinstance(frame, TTSStartedFrame):
            self._drop_until_next_tts = False

        if direction is FrameDirection.DOWNSTREAM and isinstance(frame, OutputAudioRawFrame):
            if not self._drop_until_next_tts:
                await self._write_audio(frame)
        elif direction is FrameDirection.DOWNSTREAM and isinstance(frame, TTSStoppedFrame):
            # Generation can finish while the speaker bridge still has seconds
            # of queued PCM.  _clear_voice_after_idle sends BotStopped only once
            # the estimated playback deadline has elapsed.
            pass
        elif isinstance(frame, (CancelFrame, EndFrame)):
            await self._stop_process()

        await self.push_frame(frame, direction)
