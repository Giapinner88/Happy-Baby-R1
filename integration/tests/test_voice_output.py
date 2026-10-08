"""Tests for Unitree speaker playback lifecycle signaling."""

import asyncio
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, Mock

from pipecat.frames.frames import TTSStoppedFrame
from pipecat.processors.frame_processor import FrameDirection

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice"))

from hb_voice.gate import GateSnapshot  # noqa: E402
from hb_voice.output import UnitreeSpeakerBridge, mode_cue_pcm  # noqa: E402


class _RecordingGate:
    """Gate stand-in whose release can run a callback, like the real fan-out."""

    def __init__(self):
        self.speaking = None
        self.on_release = None
        self.add_callback = Mock()
        self.snapshot = GateSnapshot(coordinator_speaker=True)

    async def set_voice_speaking(self, speaking: bool) -> None:
        self.speaking = speaking
        if not speaking and self.on_release:
            hook, self.on_release = self.on_release, None
            await hook()


class UnitreeSpeakerBridgeTest(unittest.IsolatedAsyncioTestCase):
    """Validate signals required by streaming TTS services."""

    async def test_tts_stop_does_not_clear_buffered_playback(self):
        """TTS generation can stop before queued robot-speaker PCM is silent."""
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "voice_speaking"
            marker.touch()
            speaker = UnitreeSpeakerBridge(
                bridge_path="/does/not/matter",
                network_interface="eth10",
                response_volume_percent=100,
                response_gain=1.0,
                speaking_marker_path=marker,
            )
            speaker._bot_speaking = True
            speaker.push_frame = AsyncMock()
            speaker.broadcast_frame = AsyncMock()

            await speaker.process_frame(
                TTSStoppedFrame(),
                FrameDirection.DOWNSTREAM,
            )
            self.assertTrue(marker.exists())

    async def test_marker_survives_until_estimated_playback_finishes(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "voice_speaking"
            marker.touch()
            speaker = UnitreeSpeakerBridge(
                bridge_path="/does/not/matter",
                network_interface="eth10",
                response_volume_percent=100,
                response_gain=1.0,
                speaking_marker_path=marker,
            )
            speaker._marker_refresh_s = 0.01
            speaker._playback_tail_s = 0.01
            speaker._playback_deadline = time.monotonic() + 0.05

            task = asyncio.create_task(speaker._clear_voice_after_idle())
            await asyncio.sleep(0.02)
            self.assertTrue(marker.exists())
            await task
            self.assertFalse(marker.exists())

    async def test_audio_during_release_keeps_one_owner_of_voice_speaking(self):
        """A late PCM chunk must not leave voice_speaking latched with no owner.

        Releasing the flag takes several awaits and the idle task only frees its
        slot afterwards, so audio landing in that window used to set
        voice_speaking back to True while the task that would clear it was
        already returning.  The gate then muted the microphone forever and the
        robot went deaf until the session restarted.
        """
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "voice_speaking"
            gate = _RecordingGate()
            speaker = UnitreeSpeakerBridge(
                bridge_path="/does/not/matter",
                network_interface="eth10",
                response_volume_percent=100,
                response_gain=1.0,
                gate=gate,
                speaking_marker_path=marker,
            )
            speaker._marker_refresh_s = 0.01
            speaker._playback_tail_s = 0.0
            speaker._bot_speaking = True
            speaker.broadcast_frame = AsyncMock()
            speaker._playback_deadline = time.monotonic()  # queued PCM just ran out

            async def audio_lands_mid_release():
                # Exactly what _write_audio does for a chunk that arrives while
                # the flag is being handed back.
                await speaker._mark_voice_speaking(320)

            gate.on_release = audio_lands_mid_release

            task = asyncio.create_task(speaker._clear_voice_after_idle())
            speaker._voice_idle_task = task
            await asyncio.wait_for(task, timeout=2.0)

            self.assertFalse(gate.speaking)
            self.assertIsNone(speaker._voice_idle_task)
            self.assertFalse(marker.exists())

    def test_pcm_bytes_extend_playback_deadline(self):
        speaker = UnitreeSpeakerBridge(
            bridge_path="/does/not/matter",
            network_interface="eth10",
            response_volume_percent=100,
            response_gain=1.0,
        )
        before = time.monotonic()
        speaker._extend_playback_deadline(32000)  # 1 s of 16-kHz mono s16le PCM
        self.assertGreaterEqual(speaker._playback_deadline, before + 0.99)

    def test_speaking_marker_touch_and_clear(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "voice_speaking"
            speaker = UnitreeSpeakerBridge(
                bridge_path="/does/not/matter",
                network_interface="eth10",
                response_volume_percent=100,
                response_gain=1.0,
                speaking_marker_path=marker,
            )
            speaker._touch_speaking_marker()
            self.assertTrue(marker.exists())
            speaker._clear_speaking_marker()
            self.assertFalse(marker.exists())

    def test_mode_cues_are_local_and_have_distinct_directions(self):
        enabled = mode_cue_pcm(True)
        disabled = mode_cue_pcm(False)
        self.assertEqual(len(enabled), len(disabled))
        self.assertGreater(len(enabled), 0)
        self.assertNotEqual(enabled, disabled)

    async def test_mode_cue_respects_speaker_gate(self):
        gate = Mock()
        gate.add_callback = Mock()
        gate.snapshot = GateSnapshot(coordinator_speaker=True)
        speaker = UnitreeSpeakerBridge(
            bridge_path="/does/not/matter",
            network_interface="eth10",
            response_volume_percent=100,
            response_gain=1.0,
            gate=gate,
        )
        speaker._write_audio = AsyncMock()
        self.assertTrue(await speaker.play_mode_cue(True))
        speaker._write_audio.assert_awaited_once()

        gate.snapshot = GateSnapshot(coordinator_speaker=False)
        self.assertFalse(await speaker.play_mode_cue(False))
        speaker._write_audio.assert_awaited_once()


if __name__ == "__main__":
    unittest.main()
