"""Pipecat processors for the PC2 ALSA input and R1 multicast fallback."""

import asyncio
import re
import time
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
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
from .resilience import (
    PttTurnAudioBuffer,
    apply_pcm16_gain,
    pcm16_peak,
    pcm_bytes_for_ms,
)


TurnCommitBarrier = Callable[[], Awaitable[bool]]
TurnDiscardHook = Callable[[], Awaitable[None]]
MicSourceCallback = Callable[[str], Awaitable[None]]

ALSA_SOURCE = "alsa_usb"
ROBOT_MIC_SOURCE = "r1_multicast"
SOUND_CARDS_PATH = Path("/proc/asound/cards")
SOUND_CARD_ROOT = Path("/proc/asound")
# `/proc/asound/cards` lists one card per pair of lines:
#   " 0 [BOYALINK       ]: USB-Audio - BOYALINK"
# The driver field is what separates a USB capture device from the Jetson's own
# `tegra-hda`/`tegra-ape` cards, which are always present and never the mic.
SOUND_CARD_LINE = re.compile(r"^\s*(\d+)\s+\[([^\]]+)\]:\s*(\S+)\s*-")
USB_AUDIO_DRIVER = "USB-Audio"
# Device name used while no USB capture card exists and none was ever seen.  It
# cannot resolve, and that is the point: see `_start_process`.
ALSA_NO_CARD_DEVICE = "plughw:CARD=NoUsbMic,DEV=0"

# Peak sample (of 32767) at or below which capture counts as dead rather than
# quiet.  Measured on the robot 2026-08-05: a wireless receiver whose
# transmitter was off produced a peak of 1-2 LSB over 8 seconds, so testing for
# perfect zeros would never fire.  A live mic in a quiet room sits far above
# this, which keeps a silent room from being mistaken for broken hardware.
SILENT_PEAK_CEILING = 32

# Hands-free noise gate.  Once a chunk clears the speech floor the gate stays
# open this long, so the quiet consonants and trailing syllables of a sentence
# are not chopped out of the middle of a turn.
HANDS_FREE_HANGOVER_S = 1.5
# Chunks held back while the gate is closed and flushed the moment it opens.
# Speech onsets are soft, so without a pre-roll the first syllable is lost.
# One chunk is ~85-128 ms depending on the source, so three is ~0.3-0.4s.
HANDS_FREE_PREROLL_CHUNKS = 3
# The gate is a new way to go deaf: too high a floor and audible speech is
# thrown away with nothing in the log to explain it.  Say so, and rate-limit
# it so a genuinely quiet room does not flood the journal.
HANDS_FREE_DEAF_WARN_AFTER_S = 15.0
HANDS_FREE_DEAF_WARN_EVERY_S = 60.0

# Ceiling for the recovery back-off.  A receiver that is plugged in but whose
# transmitter is off looks exactly like a recovered mic to a card-presence
# check, so each failed probe doubles the wait up to this.
RECOVER_BACKOFF_MAX_S = 300.0


@dataclass(frozen=True)
class RobotMicFallback:
    """Everything needed to run the PC1 multicast mic as a standby source.

    The external mic on PC2 can vanish mid-event (dead battery, tugged cable,
    dropped USB link).  Without a standby the robot stays deaf until somebody
    notices and restarts the service.
    """

    bridge_path: Path
    network_interface: str
    gain_db: float
    after_failures: int
    recover_check_s: float
    sample_rate: int = 16000
    mic_group_ip: Optional[str] = None
    mic_port: Optional[int] = None
    mic_payload_mode: Optional[str] = None


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
        min_speech_peak: int = 0,
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
        # Đổi nguồn mic (F2 hoặc bộ dò) đánh thức vòng respawn ngay lập tức. Không có
        # nó thì backoff của nguồn VỪA CHẾT bắt nguồn MỚI phải chờ — mà bấm F2 luôn rơi
        # đúng vào lúc backoff đã chạm trần 5s, vì người ta chỉ bấm khi nguồn đang chết.
        self._source_changed = asyncio.Event()
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
        self._conversation_mode_ready = False
        self._mode_transition_pending = False
        self._turn_commit_barrier: TurnCommitBarrier | None = None
        self._turn_discard_hook: TurnDiscardHook | None = None
        self._minimum_ptt_audio_ms = minimum_ptt_audio_ms
        self._min_speech_peak = min_speech_peak
        # A turn only becomes committable once it carries real sound.  Handing
        # OpenAI a silent turn does not produce silence back: it answers the
        # empty turn with a generic greeting, which sounds like the robot
        # talking to itself.
        self._turn_audio = PttTurnAudioBuffer(
            pcm_bytes_for_ms(self._target_sample_rate, minimum_ptt_audio_ms),
            minimum_peak=min_speech_peak,
        )
        # Hands-free mode has no push-to-talk edge to lean on, so the same
        # speech floor has to be enforced on the stream itself.
        self._hands_free_open_until = 0.0
        self._hands_free_preroll: deque[InputAudioRawFrame] = deque(
            maxlen=HANDS_FREE_PREROLL_CHUNKS
        )
        self._hands_free_audible_since = 0.0
        self._hands_free_warned_at = 0.0
        if self._gate:
            self._gate.add_callback(self._on_gate_change)

    def set_turn_commit_barrier(self, barrier: TurnCommitBarrier) -> None:
        """Install a callback that drains downstream audio before PTT commit."""
        self._turn_commit_barrier = barrier

    def set_turn_discard_hook(self, hook: TurnDiscardHook) -> None:
        """Install a callback that drops audio already sent to the Realtime API."""
        self._turn_discard_hook = hook

    async def _discard_active_turn(self) -> None:
        """End a PTT turn whose audio already left this processor.

        Once the turn is `ready` its frames have gone downstream and the
        Realtime service has appended them to the server-side input buffer.
        Aborting locally does not retract them: with no explicit clear the
        fragment sits there uncommitted, and the next release commits it
        together with the new turn - or, if hands-free is switched on, Semantic
        VAD commits the fragment on its own and the robot answers something
        nobody said.

        The pipeline is deliberately left without a matching
        UserStoppedSpeakingFrame: that frame is what commits a turn, which is
        exactly what must not happen here.  The next turn's
        UserStartedSpeakingFrame re-opens the bracket downstream.
        """
        was_active = self._turn_active
        self._turn_active = False
        self._turn_audio.abort()
        if not was_active or not self._turn_discard_hook:
            return
        try:
            await self._turn_discard_hook()
        except Exception as error:
            logger.warning(f"Could not drop the discarded PTT turn audio: {error}")

    async def set_conversation_mode(self, enabled: bool) -> None:
        """Allow the hands-free input path after its Realtime VAD update succeeds."""
        self._conversation_mode_ready = enabled
        self._mode_transition_pending = False
        await self._discard_active_turn()
        self._reset_hands_free_gate()
        logger.info("Conversation microphone path is now {}", "ready" if enabled else "PTT")
        if self._pipeline_ready and self._gate:
            # Re-evaluate without manufacturing another raw mode edge; that
            # edge has already been held closed while the VAD update ran.
            snapshot = self._gate.snapshot
            await self._on_gate_change(snapshot, snapshot)

    def _reset_hands_free_gate(self) -> None:
        self._hands_free_open_until = 0.0
        self._hands_free_preroll.clear()
        self._hands_free_audible_since = 0.0

    def _warn_if_gate_is_deafening_us(self, peak: int, now: float) -> None:
        """Report a floor that is throwing away sound loud enough to be speech."""
        if peak <= SILENT_PEAK_CEILING:
            # Nothing audible at all; that is the mic's problem, and
            # `_note_audio_content` already owns reporting it.
            self._hands_free_audible_since = 0.0
            return
        if not self._hands_free_audible_since:
            self._hands_free_audible_since = now
            return
        if now - self._hands_free_audible_since < HANDS_FREE_DEAF_WARN_AFTER_S:
            return
        if now - self._hands_free_warned_at < HANDS_FREE_DEAF_WARN_EVERY_S:
            return
        self._hands_free_warned_at = now
        logger.warning(
            f"Hands-free has dropped every chunk for "
            f"{now - self._hands_free_audible_since:.0f}s: audio is arriving but its "
            f"peak stays under the speech floor {self._min_speech_peak}/32767. "
            f"If people are talking to the robot, lower input.min_speech_peak"
        )

    def _hands_free_frames(
        self, frame: InputAudioRawFrame, audio: bytes
    ) -> list[InputAudioRawFrame]:
        """Keep sub-speech noise out of OpenAI's Semantic VAD.

        In hands-free mode Semantic VAD is the only turn authority, and it
        commits on servo whine or the tail of the robot's own speaker as
        readily as on a person.  A committed turn that transcribes to nothing
        is not answered with silence - OpenAI replies with a generic greeting,
        and when the previous reply is still playing the two ElevenLabs
        streams overlap, which is audible as the voice changing mid-sentence.
        Observed on the robot 2026-08-05: five assistant turns with no user
        transcript, plus `active response in progress` from the Realtime API.
        """
        if not self._min_speech_peak:
            return [frame]

        now = time.monotonic()
        peak = pcm16_peak(audio)
        if peak >= self._min_speech_peak:
            was_closed = now >= self._hands_free_open_until
            self._hands_free_open_until = now + HANDS_FREE_HANGOVER_S
            self._hands_free_audible_since = 0.0
            if was_closed and self._hands_free_preroll:
                frames = [*self._hands_free_preroll, frame]
                self._hands_free_preroll.clear()
                return frames
            return [frame]

        if now < self._hands_free_open_until:
            return [frame]
        self._warn_if_gate_is_deafening_us(peak, now)
        self._hands_free_preroll.append(frame)
        return []

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
        if old.conv_mode != new.conv_mode:
            # Do not leak either PTT or hands-free frames across a VAD update.
            self._mode_transition_pending = True
            self._conversation_mode_ready = False
            # This runs before the app's own conversation callback, so a
            # half-streamed PTT turn is cleared before Semantic VAD is armed.
            await self._discard_active_turn()
        if self._mode_transition_pending:
            return
        if new.conv_mode:
            # Server-side VAD owns lifecycle frames in hands-free mode.
            return
        if new.mic_allowed:
            if not self._turn_audio.capturing:
                self._turn_audio.begin()
                logger.info("PTT capture started; waiting for microphone audio")
            return

        if not self._turn_audio.capturing:
            return
        if new.ptt:
            # Select is still held but the mic was closed under it: the robot
            # started speaking, or high-level took the audio channel.  The rest
            # of the sentence is gone, so the half of it already sent has to go
            # too rather than ride along with the next turn.
            await self._discard_active_turn()
            return

        should_commit, audio_bytes = self._turn_audio.finish()
        audio_ms = audio_bytes / (self._target_sample_rate * 2) * 1000
        turn_peak = self._turn_audio.peak
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
        elif self._min_speech_peak and turn_peak < self._min_speech_peak:
            self._turn_active = False
            logger.warning(
                f"PTT turn dropped: {audio_ms:.0f}ms of audio peaked at "
                f"{turn_peak}/32767, under the speech floor {self._min_speech_peak}. "
                "Nothing was heard, so nothing is sent and the robot stays silent"
            )
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
            f"primary_mic={self._debug_robot_bytes} bytes/"
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
            logger.warning(f"{self._source_name} mic command is unavailable: {self._bridge_path}")
            return

        logger.info(
            f"Starting {self._source_name} mic loop on "
            f"{self._network_interface}: {self._bridge_path} "
            f"(segment_seconds={self._segment_seconds}, gain={self._input_gain_db}dB)"
        )
        self._read_task = self.create_task(self._read_loop())

    async def _ensure_single_capture(self) -> None:
        """Never let two microphone sources run at once.

        The external mic and the robot mic are alternatives, never a mix: two
        live captures would double every frame into the same session.  The read
        loop is the only caller that spawns, so a survivor here means a previous
        attempt outlived its handover and has to go first.
        """
        process = self._process
        self._process = None
        if not process or process.returncode is not None:
            return
        logger.warning(
            f"A previous mic capture is still alive ({process.pid}); "
            "stopping it before starting the next one"
        )
        process.terminate()
        try:
            await asyncio.wait_for(process.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()

    def _mic_bridge_args(self) -> list[str]:
        """Build the positional argv for `r1_bridge mic`.

        The bridge reads `mic <iface> [seconds] [group_ip] [port] [payload]`
        strictly by position, so the seconds slot has to be filled whenever a
        later slot is used.  Omitting it when `segment_seconds` is 0 - which it
        always is in production - shifted the group IP into the seconds slot and
        the payload mode into the port slot, where `std::stoi("raw")` aborted the
        bridge outright.  That is the documented rollback path to the PC1
        multicast mic (UNITREE_MIC_GROUP_IP in stack.env), so it has to be exact.
        """
        args = [str(self._bridge_path), "mic", self._network_interface]
        if self._segment_seconds > 0 or self._mic_group_ip:
            args.append(str(self._segment_seconds))
        if self._mic_group_ip:
            args.append(self._mic_group_ip)
            if self._mic_port or self._mic_payload_mode:
                args.append(str(self._mic_port or 5555))
            if self._mic_payload_mode:
                args.append(self._mic_payload_mode)
        return args

    async def _start_process(self) -> asyncio.subprocess.Process:
        await self._ensure_single_capture()
        args = self._mic_bridge_args()

        self._process = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        self._started_at = time.monotonic()
        self._last_robot_audio_at = self._started_at
        logger.info(
            f"Started {self._source_name} mic on {self._network_interface}: "
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
                f"{self._source_name} mic stderr: {line.decode(errors='replace').strip()}"
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
                f"{self._source_name} mic has no audio "
                f"for {silence:.1f}s after {elapsed:.1f}s ({self._bridge_status()})."
            )
            if silence >= 9.0:
                logger.warning(f"Restarting stalled {self._source_name} mic process")
                process.terminate()
                return

        logger.warning(f"{self._source_name} mic process stopped ({self._bridge_status()}).")

    async def _read_loop(self):
        restart_delay = 0.5
        while True:
            bytes_before_start = self._robot_total_bytes
            process = await self._start_process()

            if not process or not process.stdout:
                logger.warning(f"{self._source_name} mic stdout is not available")
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
                self._note_audio_content(audio)
                if self._gate and not self._gate.snapshot.mic_allowed:
                    # The mic is muted while the robot speaks.  Close the
                    # hands-free gate too, otherwise a hangover window opened
                    # before the reply survives it and lets the tail of the
                    # robot's own voice through as a fresh turn.
                    self._reset_hands_free_gate()
                    continue
                frame = InputAudioRawFrame(
                    audio=audio,
                    sample_rate=self._target_sample_rate,
                    num_channels=1,
                )
                if not self._gate:
                    await self.queue_frame(frame, FrameDirection.DOWNSTREAM)
                    continue
                snapshot = self._gate.snapshot
                if snapshot.conv_mode:
                    if not self._conversation_mode_ready or self._mode_transition_pending:
                        continue
                    # In hands-free mode OpenAI Semantic VAD is the only turn
                    # authority.  Do not emit local UserStarted/Stopped frames.
                    for gated_frame in self._hands_free_frames(frame, audio):
                        await self.push_frame(gated_frame, FrameDirection.DOWNSTREAM)
                    continue
                if self._mode_transition_pending:
                    continue
                was_ready = self._turn_audio.ready
                frames = self._turn_audio.add(frame, len(audio), pcm16_peak(audio))
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
                    f"{self._source_name} mic exited with returncode={process.returncode}"
                )

            if self._stderr_task:
                await self.cancel_task(self._stderr_task)
                self._stderr_task = None
            if self._watchdog_task:
                await self.cancel_task(self._watchdog_task)
                self._watchdog_task = None

            if self._process is process:
                self._process = None

            produced_audio = self._robot_total_bytes > bytes_before_start
            if produced_audio:
                restart_delay = 0.5
            else:
                restart_delay = min(restart_delay * 2.0, 5.0)
            await self._on_attempt_finished(produced_audio)
            if await self._wait_before_restart(restart_delay):
                # Nguồn khác thì backoff của nguồn cũ không nói gì về nguồn mới.
                restart_delay = 0.5

    async def _wait_before_restart(self, delay: float) -> bool:
        """Chờ trước lần spawn kế. Trả True nếu nguồn mic vừa đổi (bỏ qua phần chờ).

        Backoff tồn tại để một nguồn hỏng không quay vòng đốt CPU. Nó KHÔNG được
        áp cho nguồn mà người vận hành vừa chọn: bấm F2 là phải nghe được ngay,
        chứ không phải im 5 giây rồi mới có tiếng.
        """
        if self._source_changed.is_set():
            self._source_changed.clear()
            logger.info("Mic source changed - respawning immediately")
            return True
        logger.info(f"Mic bridge restart in {delay:.1f}s")
        try:
            await asyncio.wait_for(self._source_changed.wait(), timeout=delay)
        except asyncio.TimeoutError:
            return False
        self._source_changed.clear()
        logger.info("Mic source changed while waiting - respawning immediately")
        return True

    async def _on_attempt_finished(self, produced_audio: bool) -> None:
        """Hook for subclasses that can move to a different microphone source."""

    def _note_audio_content(self, audio: bytes) -> None:
        """Hook for subclasses that care whether the bytes carry any sound.

        Bytes arriving is not the same as sound arriving: a wireless receiver
        whose transmitter is off keeps streaming perfect digital silence.
        """

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
    """ALSA/USB microphone adapter using a stable card name.

    With `fallback` configured this processor owns two spawn strategies and
    moves between them on its own: the PC2 external mic while it is healthy,
    the PC1 multicast mic while it is not.  Everything downstream (read loop,
    gate, resampling, PTT buffering) is shared, so the pipeline never sees the
    swap.
    """

    def __init__(
        self,
        *,
        device: str | None,
        sample_rate: int,
        gate: AudioGate,
        input_gain_db: float = 0.0,
        audio_debug: bool = False,
        min_speech_peak: int = 0,
        silence_s: float = 0.0,
        fallback: RobotMicFallback | None = None,
        switch_mode: str = "auto",
    ):
        # `device=None` is the normal case: find the USB capture card at spawn
        # time.  An explicit ALSA_DEVICE pins one card and is only needed with
        # two USB mics on the bus.
        self._alsa_auto = device is None
        if device is not None:
            if "CARD=APE" in device:
                raise ValueError(
                    "ALSA_DEVICE cannot use the Jetson APE virtual card for the external mic"
                )
            if device.startswith("hw:") and "CARD=" not in device:
                raise ValueError(
                    "ALSA_DEVICE must use a stable card name such as "
                    "plughw:CARD=Audio,DEV=0; numeric hw:1,0 is not allowed"
                )
        self._alsa_device = device
        self._alsa_bridge_path = Path("/usr/bin/arecord")
        self._alsa_sample_rate = sample_rate
        self._alsa_gain_db = input_gain_db
        self._alsa_card = _alsa_card_name(device) if device else None
        self._fallback = fallback
        self._silence_s = silence_s
        # "manual": only the operator's F2 moves the source.  A dead external mic
        # is reported loudly and left dead, because a source that changes by
        # itself mid-event is worse than one that stays where it was put.
        self._auto_switch = switch_mode == "auto"
        self._active_source = ALSA_SOURCE
        self._consecutive_failures = 0
        self._last_sound_at = 0.0
        self._silent_stall = False
        self._recover_task: asyncio.Task | None = None
        # A card-presence check cannot tell a recovered mic from a receiver
        # whose transmitter is still off, so a failed probe stretches the next
        # wait instead of letting the two detectors ping-pong every 20s.
        self._recover_delay_s = fallback.recover_check_s if fallback else 0.0
        self._recovery_probe = False
        self._probe_started_at = 0.0
        self._source_change_callback: MicSourceCallback | None = None
        super().__init__(
            bridge_path=self._alsa_bridge_path,
            network_interface="alsa",
            source_sample_rate=sample_rate,
            segment_seconds=0,
            input_gain_db=input_gain_db,
            audio_debug=audio_debug,
            gate=gate,
            source_name=ALSA_SOURCE,
            min_speech_peak=min_speech_peak,
        )

    @property
    def active_source(self) -> str:
        return self._active_source

    def set_source_change_callback(self, callback: MicSourceCallback) -> None:
        """Let the app react to a swap (session settings, runtime status)."""
        self._source_change_callback = callback

    async def _start_process(self) -> asyncio.subprocess.Process:
        if self._active_source != ALSA_SOURCE:
            return await super()._start_process()

        # Same invariant as the multicast branch: one live capture at a time.
        await self._ensure_single_capture()
        # Resolved per attempt, not once at startup, so a mic plugged in later -
        # or swapped for a different model - is picked up by the next 5s retry
        # instead of needing the service restarted.
        device = self._resolve_alsa_device()
        args = [
            str(self._bridge_path),
            "-q",
            "-D",
            device,
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
        logger.info(f"Started ALSA mic: device={device} pid={self._process.pid}")
        self._stderr_task = self.create_task(self._read_stderr(self._process))
        self._watchdog_task = self.create_task(self._watchdog(self._process))
        return self._process

    @property
    def _alsa_device_label(self) -> str:
        """How to name the external mic in a log line when none is resolved."""
        return self._alsa_device or "no USB capture card"

    def _resolve_alsa_device(self) -> str:
        """The `-D` argument for this attempt, discovered unless pinned.

        Never returns None.  With no card on the bus this hands back a name that
        cannot resolve, so `arecord` exits and the read loop counts a failed
        attempt exactly as it does for a mic that died mid-event - which is what
        drives the fallback, the failure counter and the "press F2" message.
        Returning nothing would end the read loop and leave the robot with no
        microphone at all, standby included.
        """
        if not self._alsa_auto:
            assert self._alsa_device is not None
            return self._alsa_device

        cards = usb_capture_cards()
        if not cards:
            if self._alsa_device:
                logger.warning(
                    f"USB capture card {self._alsa_card!r} is gone from "
                    f"{SOUND_CARDS_PATH}; the mic is unplugged or off the bus"
                )
                self._alsa_device = None
                self._alsa_card = None
            return ALSA_NO_CARD_DEVICE

        _, name = cards[0]
        if name != self._alsa_card:
            if len(cards) > 1:
                others = ", ".join(card for _, card in cards)
                logger.warning(
                    f"{len(cards)} USB capture cards on the bus ({others}); using "
                    f"{name!r} because it has the lowest card index. Set ALSA_DEVICE "
                    "in /etc/hb/stack.env to pin the one you want"
                )
            self._alsa_card = name
            self._alsa_device = f"plughw:CARD={name},DEV=0"
            logger.warning(f"External mic auto-selected: {self._alsa_device}")
        assert self._alsa_device is not None
        return self._alsa_device

    def _probe_has_survived(self, now: float) -> bool:
        """Has this recovery attempt outlived the window that declares mics dead?

        With the silence detector on, surviving `silence_s` of capture is
        exactly the statement "this mic was not found to be dead", which is the
        only honest evidence that returning to it was right.  With the detector
        off there is no such window, so any audible chunk has to do.
        """
        if self._silence_s <= 0:
            return True
        return now - self._probe_started_at >= self._silence_s

    def _note_audio_content(self, audio: bytes) -> None:
        """Fall back when the mic keeps streaming but has gone dead silent.

        A dead wireless transmitter leaves the USB receiver on the bus, happily
        delivering near-silence: `arecord` never fails, so the failure counter
        never moves and the robot goes quietly deaf.  A live mic carries a real
        noise floor well above `SILENT_PEAK_CEILING`, so staying under it means
        the capture chain is dead rather than the room being quiet.

        The alarm runs even with no standby configured: an operator who chose
        the external mic only still needs to be told it went dead.
        """
        if self._active_source != ALSA_SOURCE:
            return

        now = time.monotonic()
        audible = pcm16_peak(audio) > SILENT_PEAK_CEILING
        if self._auto_switch and audible and self._recovery_probe and self._probe_has_survived(now):
            # A single chunk a hair above digital silence is not proof of life:
            # a flickering RF link pops above the ceiling now and then while
            # carrying no voice at all.  Observed on the robot 2026-08-05
            # 13:06-13:08, where those pops cleared the probe and the two
            # detectors went back to trading the source every 20s.  Survival
            # past the silence window is the real proof.
            self._recovery_probe = False
            if self._fallback and self._recover_delay_s != self._fallback.recover_check_s:
                logger.info(
                    f"External mic survived {self._silence_s:.0f}s of capture; "
                    f"recovery checks back to {self._fallback.recover_check_s:.0f}s"
                )
                self._recover_delay_s = self._fallback.recover_check_s

        if self._silence_s <= 0 or self._silent_stall:
            return
        if audible:
            self._last_sound_at = now
            return
        if not self._last_sound_at:
            self._last_sound_at = now
            return
        if now - self._last_sound_at < self._silence_s:
            return

        silent_for = now - self._last_sound_at
        if not self._auto_switch or not self._fallback:
            # Nowhere to go on our own: either there is no standby, or the
            # operator owns the source.  Keep capturing, but never let the robot
            # go deaf silently.  Re-arm so this repeats instead of warning once.
            remedy = (
                "press F2 to switch to the robot mic"
                if self._fallback
                else "check the microphone"
            )
            logger.error(
                f"External mic ({self._alsa_device_label}) has been dead silent for "
                f"{silent_for:.0f}s: peak stayed under {SILENT_PEAK_CEILING}/32767. "
                f"The robot cannot hear anything right now - {remedy}"
            )
            self._last_sound_at = now
            return

        logger.warning(
            f"External mic stayed under peak {SILENT_PEAK_CEILING}/32767 for "
            f"{silent_for:.0f}s ({self._alsa_device_label}); "
            "treating it as dead and switching to the robot mic"
        )
        self._silent_stall = True
        process = self._process
        if process and process.returncode is None:
            # Ends the attempt so the read loop can hand over on the way out.
            process.terminate()

    async def _on_attempt_finished(self, produced_audio: bool) -> None:
        if produced_audio and not self._silent_stall:
            self._consecutive_failures = 0
            return
        if not self._auto_switch:
            # The operator owns the source.  Keep retrying whatever is selected
            # and say so periodically, but never move on our own.
            self._silent_stall = False
            self._consecutive_failures += 1
            every = max(1, self._fallback.after_failures) if self._fallback else 3
            if self._fallback and self._consecutive_failures % every == 0:
                logger.error(
                    f"{self._source_name} mic failed to start "
                    f"{self._consecutive_failures} times in a row. Source switching is "
                    "manual (input.mic_switch), so nothing changed by itself - press F2 "
                    "to move to the other microphone"
                )
            return
        if not self._fallback or self._active_source != ALSA_SOURCE:
            self._silent_stall = False
            return

        # Proven silence already cost `silence_s` of dead air; do not make the
        # operator wait for the restart counter on top of that.
        if self._silent_stall:
            self._consecutive_failures = self._fallback.after_failures
        else:
            self._consecutive_failures += 1
        if self._consecutive_failures < self._fallback.after_failures:
            return
        if not self._fallback.bridge_path.exists():
            logger.error(
                "External mic is down and the robot mic bridge is missing "
                f"({self._fallback.bridge_path}); staying on {self._alsa_device_label}"
            )
            self._silent_stall = False
            return

        if not self._silent_stall:
            logger.warning(
                f"External mic failed {self._consecutive_failures} times in a row "
                f"({self._alsa_device_label}); switching to the robot mic"
            )
        if self._recovery_probe:
            # We came back for this mic and it still gave us nothing.  Without
            # backing off, the silence detector and the card-presence check
            # trade the source every ~20s and the robot is usable in neither.
            self._recovery_probe = False
            previous = self._recover_delay_s
            self._recover_delay_s = min(previous * 2, RECOVER_BACKOFF_MAX_S)
            logger.warning(
                f"The external mic is on the bus but produced no sound; next "
                f"recovery check in {self._recover_delay_s:.0f}s (was {previous:.0f}s). "
                "Turn on or charge the wearable transmitter"
            )
        await self._switch_source(ROBOT_MIC_SOURCE)
        if not self._recover_task:
            self._recover_task = self.create_task(self._watch_for_external_mic())

    async def _on_gate_change(self, old: GateSnapshot, new: GateSnapshot):
        # F2 is an operator command, not a health signal, so it is honoured even
        # while the gate has the microphone shut - the next time it opens, the
        # selected source is already live.
        if self._pipeline_ready and old.mic_external != new.mic_external:
            await self.request_source(new.mic_external)
        await super()._on_gate_change(old, new)

    async def request_source(self, external: bool) -> None:
        """Move to the microphone the operator picked with F2.

        Unlike the health-driven paths this never second-guesses the choice: if
        the selected mic is silent or missing, it stays selected and complains,
        because a source that moves back on its own is exactly what the manual
        switch exists to prevent.
        """
        target = ALSA_SOURCE if external else ROBOT_MIC_SOURCE
        if target == self._active_source:
            return
        if target == ROBOT_MIC_SOURCE and not self._fallback:
            logger.error(
                "F2: cannot switch to the robot mic because "
                "input.fallback_to_robot_mic is false; staying on the external mic"
            )
            return
        if target == ALSA_SOURCE and not self._alsa_bridge_path.exists():
            logger.error(
                f"F2: cannot switch to the external mic because {self._alsa_bridge_path} "
                "is missing; staying on the robot mic"
            )
            return

        logger.warning(f"F2: operator switched the microphone to {target}")
        await self._switch_source(target)
        process = self._process
        if process and process.returncode is None:
            # Ends the current capture so the read loop respawns on the source
            # selected just above.
            process.terminate()

    async def _switch_source(self, source: str) -> None:
        if source == self._active_source:
            return

        if source == ALSA_SOURCE:
            self._bridge_path = self._alsa_bridge_path
            self._network_interface = "alsa"
            self._source_sample_rate = self._alsa_sample_rate
            self._input_gain_db = self._alsa_gain_db
            self._mic_group_ip = None
            self._mic_port = None
            self._mic_payload_mode = None
        else:
            assert self._fallback is not None
            self._bridge_path = self._fallback.bridge_path
            self._network_interface = self._fallback.network_interface
            self._source_sample_rate = self._fallback.sample_rate
            self._input_gain_db = self._fallback.gain_db
            self._mic_group_ip = self._fallback.mic_group_ip
            self._mic_port = self._fallback.mic_port
            self._mic_payload_mode = self._fallback.mic_payload_mode

        self._active_source = source
        # Đánh thức vòng respawn: nguồn mới không phải trả backoff của nguồn cũ.
        self._source_changed.set()
        self._source_name = source
        self._consecutive_failures = 0
        self._silent_stall = False
        self._last_sound_at = 0.0
        # 48 kHz external versus 16 kHz multicast: the old resampler state must
        # not carry across the swap.
        self._resampler = create_stream_resampler(quality="QQ")
        logger.warning(
            f"Microphone source is now {source} "
            f"(rate={self._source_sample_rate}, gain={self._input_gain_db}dB)"
        )
        if self._source_change_callback:
            try:
                await self._source_change_callback(source)
            except Exception as error:
                logger.exception(f"Microphone source callback failed: {error}")

    async def _watch_for_external_mic(self) -> None:
        """Return to the external mic as soon as its card is back on the bus."""
        assert self._fallback is not None
        try:
            while self._active_source != ALSA_SOURCE:
                await asyncio.sleep(self._recover_delay_s)
                if self._active_source == ALSA_SOURCE:
                    break
                if not self._external_card_present():
                    continue
                logger.info(
                    f"USB capture card {self._alsa_card or '(auto)'} is back; "
                    "returning to the external mic"
                )
                # Provisional until the mic proves itself: the card being on
                # the bus says nothing about the transmitter being on.
                self._recovery_probe = True
                self._probe_started_at = time.monotonic()
                await self._switch_source(ALSA_SOURCE)
                process = self._process
                if process and process.returncode is None:
                    # Ends the current attempt so the read loop respawns on the
                    # source selected just above.
                    process.terminate()
        finally:
            self._recover_task = None

    def _external_card_present(self) -> bool:
        if not self._alsa_bridge_path.exists():
            return False
        if self._alsa_auto:
            # Nothing is pinned, so any USB capture card is the external mic -
            # including one whose name differs from the card that was running
            # before, which is precisely the mic-swap case.
            return bool(usb_capture_cards())
        if not self._alsa_card:
            return False
        try:
            cards = SOUND_CARDS_PATH.read_text(encoding="utf-8", errors="replace")
        except OSError as error:
            logger.warning(f"Cannot read {SOUND_CARDS_PATH}: {error}")
            return False
        # `/proc/asound/cards` pads names, so match the closing bracket too:
        # a bare prefix test would accept card "Audio2" as card "Audio".
        return re.search(rf"\[{re.escape(self._alsa_card)}\s*\]", cards) is not None

    async def _start(self):
        # `arecord` missing would make the base class give up before the read
        # loop exists, and the read loop is what counts failures - so the robot
        # would end up with no microphone at all despite a standby being armed.
        if (
            self._fallback
            and self._active_source == ALSA_SOURCE
            and not self._alsa_bridge_path.exists()
        ):
            logger.error(
                f"{self._alsa_bridge_path} is missing; starting on the robot mic instead"
            )
            await self._switch_source(ROBOT_MIC_SOURCE)
            # Starting somewhere that works beats starting deaf, but in manual
            # mode nothing may walk back on its own - F2 does that.
            if self._auto_switch and not self._recover_task:
                self._recover_task = self.create_task(self._watch_for_external_mic())
        await super()._start()

    async def _stop(self):
        if self._recover_task:
            await self.cancel_task(self._recover_task)
            self._recover_task = None
        await super()._stop()


def usb_capture_cards() -> list[tuple[int, str]]:
    """USB sound cards that can record, as `(index, card name)`, lowest first.

    Card names belong to the mic model, not to the robot: swapping a TTGK
    receiver for a BOYALINK renames the card from `Audio` to `BOYALINK` and a
    pinned `ALSA_DEVICE` then points at nothing, which reads in the log exactly
    like a mic that fell off the USB bus (2026-08-18).  Discovering the name
    instead of pinning it makes any USB mic work on the day it is plugged in.

    Card index is deliberately not used to address the device: it shifts with
    plug order (the BOYALINK took index 0 and pushed the Jetson HDA to 1).
    """
    try:
        cards = SOUND_CARDS_PATH.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        logger.warning(f"Cannot read {SOUND_CARDS_PATH}: {error}")
        return []

    found: list[tuple[int, str]] = []
    for line in cards.splitlines():
        match = SOUND_CARD_LINE.match(line)
        if not match or match.group(3) != USB_AUDIO_DRIVER:
            continue
        index = int(match.group(1))
        # A USB headset or speaker is also a USB-Audio card, so playback-only
        # devices have to be dropped: `pcm*c` exists only with capture.
        if not any(SOUND_CARD_ROOT.glob(f"card{index}/pcm*c")):
            continue
        found.append((index, match.group(2).strip()))
    return sorted(found)


def _alsa_card_name(device: str) -> str | None:
    """Pull `Audio` out of `plughw:CARD=Audio,DEV=0` for /proc/asound/cards."""
    marker = "CARD="
    start = device.find(marker)
    if start < 0:
        return None
    name = device[start + len(marker):].split(",", 1)[0].strip()
    return name or None
