"""Async ownership checks for the independent preset player."""

from __future__ import annotations

import asyncio
import time
import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from voice_presets.app import CoordinatorStatus, PresetPlayer
from voice_presets.config import PresetConfig
from voice_presets.controller import ControllerEvent


class PresetPlayerTest(unittest.IsolatedAsyncioTestCase):
    def _paths(self) -> tuple[Path, Path]:
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        return root / "status.env", root / "replay.env"

    @staticmethod
    def _ready_status(
        *, remote_alive: bool = True, high_armed: bool = False
    ) -> CoordinatorStatus:
        return CoordinatorStatus(
            remote_alive=remote_alive,
            buttons=0,
            remote_sequence=1,
            high_armed=high_armed,
            high_busy=False,
            preset_busy=False,
            cancel_epoch=0,
        )

    async def test_mode_cue_runs_outside_status_receiver(self) -> None:
        """The lease acknowledgement must still be readable while a cue starts."""
        status_file, replay_file = self._paths()
        player = PresetPlayer(
            PresetConfig.load(), status_file=status_file, replay_file=replay_file
        )
        player._status = self._ready_status()
        started = asyncio.Event()
        release = asyncio.Event()

        async def acquire() -> bool:
            return True

        async def stream(_: bytes, __: int) -> bool:
            started.set()
            await release.wait()
            return True

        player._acquire_lease = acquire  # type: ignore[method-assign]
        player._stream_pcm = stream  # type: ignore[method-assign]

        await player._handle_event(ControllerEvent("mode", True))
        task = player._active_task
        self.assertIsNotNone(task)
        await asyncio.wait_for(started.wait(), timeout=0.2)
        self.assertIs(player._active_task, task)

        release.set()
        await task
        self.assertIsNone(player._active_task)

    async def test_transport_error_retries_the_same_clip(self) -> None:
        config = replace(PresetConfig.load(), retry_interval_s=0.001)
        status_file, replay_file = self._paths()
        player = PresetPlayer(config, status_file=status_file, replay_file=replay_file)
        player._status = self._ready_status()
        clip = config.clips["UP"]
        attempts = 0

        async def acquire() -> bool:
            return True

        async def stream(_):
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise BrokenPipeError("bridge restarted")
            return True

        player._acquire_lease = acquire  # type: ignore[method-assign]
        player._stream_file = stream  # type: ignore[method-assign]
        player._active_clip = clip
        task = asyncio.create_task(player._run_clip(clip))
        player._active_task = task

        await task
        self.assertEqual(attempts, 2)

    async def test_stream_file_keeps_bridge_open_for_configured_end_tail(self) -> None:
        config = replace(PresetConfig.load(), end_tail_s=0.02)
        status_file, replay_file = self._paths()
        player = PresetPlayer(config, status_file=status_file, replay_file=replay_file)
        player._status = self._ready_status()
        clip = config.clips["UP"]
        written_at = 0.0
        closed_at = 0.0

        class FakeStdout:
            def __init__(self) -> None:
                self._read = False

            async def read(self, _: int) -> bytes:
                if self._read:
                    return b""
                self._read = True
                return b"x" * 320

        class FakeStdin:
            def write(self, _: bytes) -> None:
                nonlocal written_at
                written_at = time.monotonic()

            async def drain(self) -> None:
                return None

            def close(self) -> None:
                nonlocal closed_at
                closed_at = time.monotonic()

            async def wait_closed(self) -> None:
                return None

        class FakeProcess:
            def __init__(self, *, stdout=None, stdin=None) -> None:
                self.stdout = stdout
                self.stdin = stdin
                self.returncode = 0

            async def wait(self) -> int:
                return 0

        ffmpeg = FakeProcess(stdout=FakeStdout())
        bridge = FakeProcess(stdin=FakeStdin())

        async def create_ffmpeg(*_: object, **__: object) -> FakeProcess:
            return ffmpeg

        async def start_bridge(_: int) -> FakeProcess:
            return bridge

        async def no_stop_processes() -> None:
            return None

        player._start_bridge = start_bridge  # type: ignore[method-assign]
        player._stop_processes = no_stop_processes  # type: ignore[method-assign]
        player._active_clip = clip
        with patch("voice_presets.app.asyncio.create_subprocess_exec", create_ffmpeg):
            self.assertTrue(await player._stream_file(clip))

        self.assertGreaterEqual(closed_at - written_at, config.end_tail_s * 0.8)

    async def test_remote_loss_discards_crash_replay_request(self) -> None:
        config = PresetConfig.load()
        clips = dict(config.clips)
        clips["UP"] = replace(clips["UP"], path=Path("/tmp/up.wav"))
        config = replace(config, clips=clips)
        status_file, replay_file = self._paths()
        replay_file.write_text("key=UP\n", encoding="utf-8")
        player = PresetPlayer(config, status_file=status_file, replay_file=replay_file)

        await player._on_status(self._ready_status(remote_alive=False))

        self.assertIsNone(player._pending_replay_key)
        self.assertFalse(replay_file.exists())

    async def test_repeating_active_direction_cancels_without_b(self) -> None:
        status_file, replay_file = self._paths()
        config = PresetConfig.load()
        player = PresetPlayer(config, status_file=status_file, replay_file=replay_file)
        player._status = self._ready_status()
        clip = config.clips["UP"]
        player._active_clip = clip
        player._active_task = asyncio.create_task(asyncio.sleep(60))
        remote_stop_called = False

        async def stop_remote() -> None:
            nonlocal remote_stop_called
            remote_stop_called = True

        player._stop_remote_stream = stop_remote  # type: ignore[method-assign]

        await player._handle_event(ControllerEvent("play", "UP"))

        self.assertIsNone(player._active_clip)
        self.assertIsNone(player._active_task)
        self.assertTrue(remote_stop_called)
        self.assertEqual(status_file.read_text(encoding="utf-8").splitlines()[0], "state=cancelled")

    async def test_quick_repeat_chord_is_ignored_then_later_repeat_cancels(self) -> None:
        status_file, replay_file = self._paths()
        config = PresetConfig.load()
        player = PresetPlayer(config, status_file=status_file, replay_file=replay_file)
        player._status = self._ready_status()
        clip = config.clips["UP"]
        player._active_clip = clip
        player._active_task = asyncio.create_task(asyncio.sleep(60))
        player._last_direction_press_at["UP"] = time.monotonic()

        remote_stop_calls = 0

        async def stop_remote() -> None:
            nonlocal remote_stop_calls
            remote_stop_calls += 1

        player._stop_remote_stream = stop_remote  # type: ignore[method-assign]

        await player._handle_event(ControllerEvent("play", "UP"))
        self.assertIs(player._active_clip, clip)
        self.assertEqual(remote_stop_calls, 0)

        player._last_direction_press_at["UP"] = time.monotonic() - 2.0
        await player._handle_event(ControllerEvent("play", "UP"))
        self.assertIsNone(player._active_clip)
        self.assertEqual(remote_stop_calls, 1)

    async def test_armed_high_level_can_play_when_not_busy(self) -> None:
        status_file, replay_file = self._paths()
        player = PresetPlayer(
            PresetConfig.load(), status_file=status_file, replay_file=replay_file
        )
        player._status = self._ready_status(high_armed=True)
        self.assertTrue(player._can_play())

    async def test_armed_high_level_busy_still_blocks_playback(self) -> None:
        status_file, replay_file = self._paths()
        player = PresetPlayer(
            PresetConfig.load(), status_file=status_file, replay_file=replay_file
        )
        player._status = replace(
            self._ready_status(high_armed=True), high_busy=True
        )
        self.assertFalse(player._can_play())

    async def test_service_stop_wakes_socket_receiver(self) -> None:
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        player = PresetPlayer(
            PresetConfig.load(),
            status_socket=root / "status.sock",
            control_socket=root / "control.sock",
            status_file=root / "status.env",
            replay_file=root / "replay.env",
        )
        player.validate_runtime = lambda: None  # type: ignore[method-assign]
        run_task = asyncio.create_task(player.run())
        await asyncio.sleep(0.02)
        await asyncio.wait_for(player.stop(), timeout=0.5)
        await asyncio.wait_for(run_task, timeout=0.5)
