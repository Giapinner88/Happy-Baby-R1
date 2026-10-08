"""Independent offline preset player for the Unitree R1 speaker."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import shutil
import signal
import socket
import time
from dataclasses import dataclass
from pathlib import Path

from .config import ROOT, PresetClip, PresetConfig, PresetConfigError
from .controller import ControllerEvent, PresetController


SAMPLE_RATE = 16_000
PCM_BYTES_PER_SECOND = SAMPLE_RATE * 2
STREAM_CHUNK_BYTES = 16_000  # 0.5 s at 16 kHz mono; keep the robot queue bounded.
CANCEL_TIMEOUT_S = 0.5
PROCESS_STOP_TIMEOUT_S = 0.2
REMOTE_STOP_TIMEOUT_S = 0.5
SOCKET_POLL_TIMEOUT_S = 0.25
SPEAKER_APP_NAME = "pipecat"
DEFAULT_STATUS_SOCKET = Path("/run/hb/voice_presets.sock")
DEFAULT_CONTROL_SOCKET = Path("/run/hb/voice_presets_control.sock")
DEFAULT_STATUS_FILE = Path("/run/hb/voice_presets_status.env")
DEFAULT_REPLAY_FILE = Path("/run/hb/voice_presets_replay.env")


def _flag(values: dict[str, str], name: str) -> bool:
    return values.get(name, "0") == "1"


def _integer(values: dict[str, str], name: str, default: int = 0) -> int:
    try:
        return int(values.get(name, str(default)), 10)
    except ValueError:
        return default


@dataclass(frozen=True)
class CoordinatorStatus:
    remote_alive: bool
    buttons: int
    remote_sequence: int
    high_armed: bool
    high_busy: bool
    preset_busy: bool
    cancel_epoch: int

    @classmethod
    def parse(cls, raw: bytes) -> "CoordinatorStatus":
        values: dict[str, str] = {}
        for token in raw.decode("utf-8", errors="replace").split():
            if "=" in token:
                key, value = token.split("=", 1)
                values[key] = value
        if values.get("v") != "1":
            raise ValueError("unsupported preset coordinator protocol")
        return cls(
            remote_alive=_flag(values, "remote_alive"),
            buttons=_integer(values, "buttons") & 0xFFFF,
            remote_sequence=_integer(values, "remote_seq"),
            high_armed=_flag(values, "high_armed"),
            high_busy=_flag(values, "high_busy"),
            preset_busy=_flag(values, "preset_busy"),
            cancel_epoch=_integer(values, "cancel_epoch"),
        )


class PresetPlayer:
    """Owns only the `pipecat` speaker stream while its short coordinator lease lives."""

    def __init__(
        self,
        config: PresetConfig,
        *,
        status_socket: Path = DEFAULT_STATUS_SOCKET,
        control_socket: Path = DEFAULT_CONTROL_SOCKET,
        status_file: Path = DEFAULT_STATUS_FILE,
        replay_file: Path = DEFAULT_REPLAY_FILE,
        bridge_path: Path | None = None,
        network_interface: str | None = None,
    ):
        self._config = config
        self._controller = PresetController(config)
        self._status_socket_path = status_socket
        self._control_socket_path = control_socket
        self._status_file = status_file
        self._replay_file = replay_file
        default_bridge = ROOT.parent / "voice" / "unitree_bridge" / "build" / "r1_bridge"
        self._bridge_path = Path(
            os.getenv("UNITREE_BRIDGE_PATH", str(bridge_path or default_bridge))
        )
        self._network_interface = network_interface or os.getenv(
            "UNITREE_NETWORK_INTERFACE", "eth10"
        )
        self._status: CoordinatorStatus | None = None
        self._last_cancel_epoch: int | None = None
        self._socket: socket.socket | None = None
        self._control: socket.socket | None = None
        self._active_task: asyncio.Task[None] | None = None
        self._active_clip: PresetClip | None = None
        self._active_processes: tuple[asyncio.subprocess.Process, ...] = ()
        self._lease_task: asyncio.Task[None] | None = None
        self._lease_owned = False
        self._stopping = False
        # Track the last chord per direction so a quick repeat remains
        # available for the high-level double-click window without cancelling.
        self._last_direction_press_at: dict[str, float] = {}
        self._pending_replay_key = self._load_pending_replay()

    def validate_runtime(self) -> None:
        if not self._bridge_path.is_file() or not os.access(self._bridge_path, os.X_OK):
            raise PresetConfigError(f"r1_bridge missing or not executable: {self._bridge_path}")
        if not shutil.which("ffmpeg"):
            raise PresetConfigError("ffmpeg is required for preset playback")

    async def run(self) -> None:
        self.validate_runtime()
        self._status_socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._status_socket_path.unlink(missing_ok=True)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.bind(str(self._status_socket_path))
        os.chmod(self._status_socket_path, 0o660)
        self._control = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._control.setblocking(False)
        self._write_status("waiting")
        print(
            "voice_presets ready: "
            f"config={self._config.source_path} bridge={self._bridge_path}"
        )
        loop = asyncio.get_running_loop()
        try:
            while not self._stopping:
                try:
                    # A timeout lets SIGTERM observe `_stopping` promptly. A
                    # bare sock_recv used to keep systemd stop stuck until its
                    # 90-second kill timeout.
                    raw = await asyncio.wait_for(
                        loop.sock_recv(self._socket, 2048), SOCKET_POLL_TIMEOUT_S
                    )
                except asyncio.TimeoutError:
                    continue
                except OSError:
                    if self._stopping:
                        break
                    raise
                try:
                    await self._on_status(CoordinatorStatus.parse(raw))
                except ValueError as error:
                    print(f"voice_presets ignored datagram: {error}")
        finally:
            await self.stop()

    async def stop(self) -> None:
        self._stopping = True
        self._clear_pending_replay()
        await self._cancel_active("service_stop")
        await self._release_lease()
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._control:
            self._control.close()
            self._control = None
        self._status_socket_path.unlink(missing_ok=True)
        self._write_status("stopped")

    async def _on_status(self, status: CoordinatorStatus) -> None:
        previous = self._status
        self._status = status
        if self._last_cancel_epoch is None:
            self._last_cancel_epoch = status.cancel_epoch
        elif status.cancel_epoch != self._last_cancel_epoch:
            self._last_cancel_epoch = status.cancel_epoch
            self._clear_pending_replay()
            await self._cancel_active("coordinator_cancel")

        # These are the only remote/high-level conditions that cancel a request
        # and explicitly forbid automatic replay when the signal comes back.
        if not status.remote_alive:
            self._clear_pending_replay()
            await self._cancel_active("remote_lost")
        if status.high_busy:
            self._clear_pending_replay()
            await self._cancel_active("high_level_busy")

        events = self._controller.update(
            remote_alive=status.remote_alive,
            high_armed=status.high_armed,
            buttons=status.buttons,
            now=time.monotonic(),
        )
        for event in events:
            await self._handle_event(event)

        await self._resume_pending_replay()

        if previous is None or previous.remote_alive != status.remote_alive:
            self._write_status("ready" if status.remote_alive else "remote_lost")

    async def _handle_event(self, event: ControllerEvent) -> None:
        if event.kind == "mode":
            enabled = bool(event.value)
            self._clear_pending_replay()
            frequency = (
                self._config.mode.enabled_cue_hz
                if enabled
                else self._config.mode.disabled_cue_hz
            )
            await self._start_cue(
                frequency, "mode_enabled" if enabled else "mode_disabled"
            )
            self._write_status("mode_on" if enabled else "mode_off")
        elif event.kind == "cancel":
            had_active = self._active_task is not None
            had_pending = self._pending_replay_key is not None
            self._clear_pending_replay()
            if not had_active and not had_pending:
                return
            if not had_active:
                self._write_status("cancelled")
                return
            await self._cancel_active("operator_cancel")
            self._write_status("cancelled")
        elif event.kind == "play":
            key = str(event.value)
            clip = self._config.clips[key]
            if not clip.path:
                print(f"voice_presets {clip.key} is disabled: add assets file in presets.yaml")
                self._write_status("clip_disabled")
                return
            now = time.monotonic()
            # A repeated chord is the local cancel gesture only after the
            # configured repeat window; quicker repeats leave the clip running.
            if self._active_clip and self._active_clip.key == clip.key:
                previous = self._last_direction_press_at.get(key)
                self._last_direction_press_at[key] = now
                if (
                    previous is not None
                    and now - previous < self._config.repeat_cancel_delay_s
                ):
                    print(
                        f"voice_presets ignore {key} repeat inside cancel window "
                        f"({now - previous:.2f}s < {self._config.repeat_cancel_delay_s:.2f}s)"
                    )
                    return
                self._clear_pending_replay()
                await self._cancel_active("repeat_cancel")
                self._write_status("cancelled")
                return
            self._last_direction_press_at[key] = now
            await self._start_clip(clip)

    def _can_play(self) -> bool:
        return bool(
            self._status
            and self._status.remote_alive
            and not self._status.high_busy
        )

    async def _start_clip(self, clip: PresetClip) -> None:
        await self._cancel_active("replaced")
        if not self._can_play():
            self._write_status("blocked")
            return
        self._remember_pending_replay(clip.key)
        self._active_clip = clip
        self._active_task = asyncio.create_task(self._run_clip(clip), name=f"preset-{clip.name}")
        self._write_status(f"playing_{clip.key.lower()}")

    async def _run_clip(self, clip: PresetClip) -> None:
        try:
            if not await self._acquire_lease():
                self._write_status("lease_failed")
                return
            while self._can_play() and self._active_clip == clip:
                try:
                    completed = await self._stream_file(clip)
                except OSError as error:
                    # Broken bridge stdin and a temporarily unavailable decoder
                    # are transport failures, not an operator cancellation.
                    print(f"voice_presets transport error for {clip.name}: {error}")
                    completed = False
                if completed:
                    self._clear_pending_replay()
                    self._write_status("ready")
                    return
                if not self._can_play() or self._active_clip != clip:
                    return
                # Decoder/bridge errors retry the selected clip from its start.
                # Remote loss, high BUSY and B-cancel all cancel this task instead.
                print(f"voice_presets retrying {clip.name} after transport failure")
                await asyncio.sleep(self._config.retry_interval_s)
        except asyncio.CancelledError:
            raise
        finally:
            if self._active_task is asyncio.current_task():
                self._active_clip = None
                self._active_task = None
            await self._release_lease()

    async def _resume_pending_replay(self) -> None:
        key = self._pending_replay_key
        if not key or not self._can_play() or self._active_task:
            return
        clip = self._config.clips.get(key)
        if not clip or not clip.path:
            self._clear_pending_replay()
            return
        # `_start_clip` writes the request atomically again before spawning its
        # worker.  Clear only the in-memory marker here so repeated 10 Hz
        # coordinator datagrams cannot create duplicate playback tasks.
        self._pending_replay_key = None
        print(f"voice_presets resuming {clip.name} after worker restart")
        await self._start_clip(clip)

    async def _start_cue(self, frequency_hz: float, reason: str) -> None:
        # Mode confirmation shares the speaker lease and preempts any clip, so
        # only one R1 speaker stream is active at a time.
        await self._cancel_active(reason)
        if not self._can_play():
            return
        self._active_clip = None
        self._active_task = asyncio.create_task(
            self._run_cue(frequency_hz), name="preset-mode-cue"
        )

    async def _run_cue(self, frequency_hz: float) -> None:
        try:
            if not await self._acquire_lease():
                self._write_status("cue_failed")
                return
            played = await self._stream_pcm(_tone_pcm(frequency_hz), 90)
            if not played and self._can_play():
                await asyncio.sleep(self._config.retry_interval_s)
                if self._can_play():
                    played = await self._stream_pcm(_tone_pcm(frequency_hz), 90)
            if not played:
                self._write_status("cue_failed")
        finally:
            if self._active_task is asyncio.current_task():
                self._active_task = None
            await self._release_lease()

    async def _acquire_lease(self) -> bool:
        if not self._can_play():
            return False
        for _ in range(16):
            self._send_lease(True)
            await asyncio.sleep(0.05)
            if self._status and self._status.preset_busy:
                self._lease_owned = True
                if not self._lease_task:
                    self._lease_task = asyncio.create_task(self._refresh_lease())
                return True
        return False

    async def _refresh_lease(self) -> None:
        try:
            while self._lease_owned and not self._stopping:
                self._send_lease(True)
                await asyncio.sleep(0.25)
        finally:
            self._lease_task = None

    async def _release_lease(self) -> None:
        self._lease_owned = False
        task = self._lease_task
        if task and task is not asyncio.current_task():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self._lease_task = None
        self._send_lease(False)

    def _send_lease(self, busy: bool) -> None:
        if not self._control:
            return
        message = f"v=1 role=preset busy={int(busy)}".encode()
        try:
            self._control.sendto(message, str(self._control_socket_path))
        except OSError:
            # Coordinator may be restarting. The active clip retries only if it
            # was not canceled by remote loss/high BUSY when status returns.
            pass

    async def _cancel_active(self, reason: str) -> None:
        task = self._active_task
        if not task:
            return
        label = self._active_clip.name if self._active_clip else "voice preset"
        print(f"voice_presets stop {label}: {reason}")
        task.cancel()
        try:
            await asyncio.wait_for(task, timeout=CANCEL_TIMEOUT_S)
        except (asyncio.CancelledError, asyncio.TimeoutError):
            # The worker is deliberately bounded. Process cleanup below and
            # an explicit remote PlayStop handle a bridge blocked in RPC.
            pass
        if self._active_task is task:
            self._active_task = None
            self._active_clip = None
        await self._stop_processes()
        await self._stop_remote_stream()

    async def _stop_remote_stream(self) -> None:
        """Issue PlayStop even if the streaming bridge was killed mid-RPC."""
        if not self._bridge_path.is_file():
            return
        process: asyncio.subprocess.Process | None = None
        try:
            # An empty speaker invocation exits through the bridge's normal
            # PlayStop path without sending PCM or changing the global volume.
            process = await asyncio.create_subprocess_exec(
                str(self._bridge_path), "speaker", self._network_interface,
                SPEAKER_APP_NAME,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            await asyncio.wait_for(process.wait(), REMOTE_STOP_TIMEOUT_S)
        except (OSError, asyncio.TimeoutError) as error:
            print(f"voice_presets remote stop failed: {error}")
            if process and process.returncode is None:
                process.kill()
                await process.wait()

    async def _stream_file(self, clip: PresetClip) -> bool:
        assert clip.path is not None
        ffmpeg = await asyncio.create_subprocess_exec(
            "ffmpeg", "-nostdin", "-v", "error", "-i", str(clip.path),
            "-map", "0:a:0", "-vn",
            "-f", "s16le", "-acodec", "pcm_s16le", "-ac", "1", "-ar", str(SAMPLE_RATE),
            "pipe:1", stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
        )
        bridge = await self._start_bridge(clip.volume_percent)
        self._active_processes = (ffmpeg, bridge)
        try:
            assert ffmpeg.stdout and bridge.stdin
            # ffmpeg can decode a multi-minute MP3 much faster than real time.
            # Pacing the pipe prevents PlayStream from accumulating a large
            # remote queue, which otherwise sounds like skips/reordering and
            # also makes a later stop appear ineffective.
            playback_deadline = asyncio.get_running_loop().time()
            while data := await ffmpeg.stdout.read(STREAM_CHUNK_BYTES):
                bridge.stdin.write(data)
                await bridge.stdin.drain()
                playback_deadline += len(data) / PCM_BYTES_PER_SECOND
                delay = playback_deadline - asyncio.get_running_loop().time()
                if delay > 0:
                    await asyncio.sleep(delay)
            # PlayStream acknowledges enqueueing PCM, not that the R1 speaker
            # has finished it. Keep the bridge open briefly after the final
            # chunk so its later PlayStop cannot cut the end of a clip that is
            # still in the robot-side queue. Abort this grace period instantly
            # when the operator cancels or a gate condition changes.
            tail_deadline = asyncio.get_running_loop().time() + self._config.end_tail_s
            while self._can_play() and self._active_clip == clip:
                remaining = tail_deadline - asyncio.get_running_loop().time()
                if remaining <= 0:
                    break
                await asyncio.sleep(min(0.05, remaining))
            bridge.stdin.close()
            await bridge.stdin.wait_closed()
            await ffmpeg.wait()
            await bridge.wait()
            return ffmpeg.returncode == 0 and bridge.returncode == 0
        finally:
            await self._stop_processes()

    async def _stream_pcm(self, pcm: bytes, volume_percent: int) -> bool:
        bridge = await self._start_bridge(volume_percent)
        self._active_processes = (bridge,)
        try:
            assert bridge.stdin
            bridge.stdin.write(pcm)
            await bridge.stdin.drain()
            bridge.stdin.close()
            await bridge.stdin.wait_closed()
            await bridge.wait()
            return bridge.returncode == 0
        finally:
            await self._stop_processes()

    async def _start_bridge(self, volume_percent: int) -> asyncio.subprocess.Process:
        return await asyncio.create_subprocess_exec(
            str(self._bridge_path), "speaker", self._network_interface, SPEAKER_APP_NAME,
            str(volume_percent), stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
        )

    async def _stop_processes(self) -> None:
        processes, self._active_processes = self._active_processes, ()
        for process in processes:
            if process.returncode is None:
                process.terminate()

        async def wait_or_kill(process: asyncio.subprocess.Process) -> None:
            if process.returncode is None:
                try:
                    await asyncio.wait_for(process.wait(), timeout=PROCESS_STOP_TIMEOUT_S)
                except asyncio.TimeoutError:
                    if process.returncode is None:
                        process.kill()
                        await process.wait()

        # ffmpeg and the bridge are independent children. Waiting for them in
        # parallel keeps cancellation bounded instead of adding two timeouts.
        await asyncio.gather(*(wait_or_kill(process) for process in processes))

    def _write_status(self, state: str) -> None:
        try:
            self._status_file.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._status_file.with_suffix(".tmp")
            tmp.write_text(
                "\n".join(
                    (
                        f"state={state}",
                        f"mode_enabled={int(self._controller.enabled)}",
                        f"active={self._active_clip.name if self._active_clip else ''}",
                        f"updated_monotonic_ms={int(time.monotonic() * 1000)}",
                    )
                ) + "\n",
                encoding="utf-8",
            )
            os.chmod(tmp, 0o644)
            tmp.replace(self._status_file)
        except OSError as error:
            print(f"voice_presets cannot write status: {error}")

    def _load_pending_replay(self) -> str | None:
        try:
            values = {
                key: value
                for line in self._replay_file.read_text(encoding="utf-8").splitlines()
                if "=" in line
                for key, value in (line.split("=", 1),)
            }
        except FileNotFoundError:
            return None
        except OSError as error:
            print(f"voice_presets cannot read replay state: {error}")
            return None
        key = values.get("key")
        if key in self._config.clips and self._config.clips[key].path:
            return key
        self._clear_pending_replay()
        return None

    def _remember_pending_replay(self, key: str) -> None:
        self._pending_replay_key = key
        try:
            self._replay_file.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._replay_file.with_suffix(".tmp")
            tmp.write_text(f"key={key}\n", encoding="utf-8")
            os.chmod(tmp, 0o600)
            tmp.replace(self._replay_file)
        except OSError as error:
            print(f"voice_presets cannot persist replay state: {error}")

    def _clear_pending_replay(self) -> None:
        self._pending_replay_key = None
        try:
            self._replay_file.unlink(missing_ok=True)
        except OSError as error:
            print(f"voice_presets cannot clear replay state: {error}")


def _tone_pcm(frequency_hz: float, duration_s: float = 0.35) -> bytes:
    samples = bytearray()
    frames = int(SAMPLE_RATE * duration_s)
    fade = max(1, int(SAMPLE_RATE * 0.012))
    for index in range(frames):
        envelope = min(1.0, index / fade, (frames - 1 - index) / fade)
        value = int(
            3500 * envelope * math.sin(2.0 * math.pi * frequency_hz * index / SAMPLE_RATE)
        )
        samples.extend(value.to_bytes(2, "little", signed=True))
    return bytes(samples)


async def _main_async(args: argparse.Namespace) -> int:
    config = PresetConfig.load(args.config)
    if args.check_config:
        print(f"PRESET_CONFIG_OK {config.source_path}")
        return 0
    player = PresetPlayer(config)
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, lambda: asyncio.create_task(player.stop()))
    await player.run()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="HB R1 offline voice preset player")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--check-config", action="store_true")
    args = parser.parse_args()
    try:
        return asyncio.run(_main_async(args))
    except PresetConfigError as error:
        print(f"PRESET_CONFIG_ERROR: {error}")
        return 2
