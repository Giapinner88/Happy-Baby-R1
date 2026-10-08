import asyncio
import socket
import sys
import tempfile
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice"))

from hb_voice.gate import AudioGate  # noqa: E402


class AudioGateTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "gate.sock"
        self.gate = AudioGate(self.path, stale_after_s=0.15)
        await self.gate.start()

    async def asyncTearDown(self):
        await self.gate.stop()
        self.temp.cleanup()

    def send(self, text: str):
        sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            sender.sendto(text.encode(), str(self.path))
        finally:
            sender.close()

    async def test_fail_closed_ptt_and_voice_echo_gate(self):
        self.assertFalse(self.gate.snapshot.mic_allowed)
        self.assertFalse(self.gate.snapshot.speaker_allowed)

        self.send(
            "v=1 high_alive=1 high_busy=0 remote_alive=1 ptt=1 conv=1 rearm=0 mic=1 speaker=1"
        )
        await asyncio.sleep(0.03)
        self.assertTrue(self.gate.snapshot.mic_allowed)
        self.assertTrue(self.gate.snapshot.speaker_allowed)
        self.assertTrue(self.gate.snapshot.conv_mode)

        await self.gate.set_voice_speaking(True)
        self.assertFalse(self.gate.snapshot.mic_allowed)
        self.assertTrue(self.gate.snapshot.speaker_allowed)

        self.send(
            "v=1 high_alive=1 high_busy=1 remote_alive=1 ptt=1 rearm=1 mic=0 speaker=0"
        )
        await asyncio.sleep(0.03)
        self.assertFalse(self.gate.snapshot.mic_allowed)
        self.assertFalse(self.gate.snapshot.speaker_allowed)

    def test_missing_conversation_token_defaults_to_disabled(self):
        snapshot = self.gate._parse(
            "v=1 high_alive=1 high_busy=0 remote_alive=1 ptt=0 rearm=0 mic=0 speaker=1",
            False,
        )
        self.assertFalse(snapshot.conv_mode)

    def test_gesture_dev_requires_the_complete_run_r1_locomotion_state(self):
        base = (
            "v=1 high_alive=1 high_busy=0 high_armed=1 high_state=LOCOMOTION "
            "remote_alive=1 ptt=0 rearm=0 mic=0 speaker=1"
        )
        # Old coordinators and built-in firmware Dev must not expose motor tools.
        self.assertFalse(self.gate._parse(base, False).gesture_dev_ready)
        self.assertTrue(self.gate._parse(base + " gesture_dev=1", False).gesture_dev_ready)
        self.assertFalse(
            self.gate._parse(
                (base + " gesture_dev=1").replace("high_state=LOCOMOTION", "high_state=UNKNOWN"),
                False,
            ).gesture_dev_ready
        )
        self.assertFalse(
            self.gate._parse(
                (base + " gesture_dev=1").replace("remote_alive=1", "remote_alive=0"),
                False,
            ).gesture_dev_ready
        )

    def test_missing_mic_source_token_defaults_to_the_external_mic(self):
        base = "v=1 high_alive=1 high_busy=0 remote_alive=1 ptt=0 rearm=0 mic=0 speaker=1"
        # An older coordinator that does not send micext must keep running the
        # configured input.source rather than silently landing on the robot mic.
        self.assertTrue(self.gate._parse(base, False).mic_external)
        self.assertFalse(self.gate._parse(base + " micext=0", False).mic_external)
        self.assertTrue(self.gate._parse(base + " micext=1", False).mic_external)

    async def test_stale_coordinator_keeps_the_selected_microphone(self):
        """Fail-closed covers permissions, not which mic the operator chose.

        Resetting the source on a coordinator hiccup would swap the microphone
        mid-sentence with nobody touching F2.
        """
        self.send(
            "v=1 high_alive=1 high_busy=0 remote_alive=1 ptt=0 rearm=0 mic=0 "
            "speaker=1 micext=0"
        )
        await asyncio.sleep(0.03)
        self.assertFalse(self.gate.snapshot.mic_external)
        self.assertTrue(self.gate.snapshot.speaker_allowed)

        await asyncio.sleep(0.4)
        self.assertFalse(self.gate.snapshot.speaker_allowed)
        self.assertFalse(self.gate.snapshot.mic_external)

    async def test_stale_coordinator_fails_closed(self):
        self.send(
            "v=1 high_alive=1 high_busy=0 remote_alive=1 ptt=0 rearm=0 mic=0 speaker=1"
        )
        await asyncio.sleep(0.03)
        self.assertTrue(self.gate.snapshot.speaker_allowed)
        await asyncio.sleep(0.4)
        self.assertFalse(self.gate.snapshot.speaker_allowed)
        self.assertTrue(self.gate.snapshot.high_busy)

    async def test_activation_modes_follow_high_armed_state(self):
        disarmed_only = AudioGate(
            Path(self.temp.name) / "disarmed.sock",
            activation_mode="high_disarmed",
        )
        armed_only = AudioGate(
            Path(self.temp.name) / "armed.sock",
            activation_mode="high_armed",
        )
        disarmed_message = (
            "v=1 high_alive=1 high_busy=0 high_armed=0 remote_alive=1 "
            "ptt=1 rearm=0 mic=1 speaker=1"
        )
        armed_message = disarmed_message.replace("high_armed=0", "high_armed=1")
        self.assertTrue(disarmed_only._parse(disarmed_message, False).mic_allowed)
        self.assertFalse(disarmed_only._parse(armed_message, False).mic_allowed)
        self.assertFalse(armed_only._parse(disarmed_message, False).speaker_allowed)
        self.assertTrue(armed_only._parse(armed_message, False).speaker_allowed)

    async def test_startup_grace_allows_ptt_then_fails_closed(self):
        path = Path(self.temp.name) / "startup.sock"
        gate = AudioGate(
            path,
            stale_after_s=0.15,
            allow_during_startup=True,
            startup_grace_s=0.1,
        )
        await gate.start()
        try:
            message = (
                "v=1 high_alive=0 high_busy=1 high_armed=0 remote_alive=1 "
                "ptt=1 rearm=0 mic=0 speaker=0"
            )
            self.assertTrue(gate._parse(message, False).mic_allowed)
            await asyncio.sleep(0.12)
            self.assertFalse(gate._parse(message, False).mic_allowed)
        finally:
            await gate.stop()

    async def test_startup_grace_allows_hands_free_not_only_ptt(self):
        """Double-F1 before high-level is up must actually open the microphone.

        The startup override used to test `ptt` alone, so the rising mode cue
        played and Semantic VAD armed while every frame was still dropped: the
        robot said it was listening and then heard nothing for the whole grace
        window.
        """
        gate = AudioGate(
            Path(self.temp.name) / "startup_conv.sock",
            allow_during_startup=True,
            startup_grace_s=10,
        )
        await gate.start()
        try:
            conv = (
                "v=1 high_alive=0 high_busy=1 high_armed=0 remote_alive=1 "
                "ptt=0 conv=1 rearm=0 mic=0 speaker=0"
            )
            self.assertTrue(gate._parse(conv, False).mic_allowed)
            # Still fail-closed on the remote: the coordinator drops conv_mode
            # when the remote goes away, and the override must not outlive it.
            no_remote = conv.replace("remote_alive=1", "remote_alive=0")
            self.assertFalse(gate._parse(no_remote, False).mic_allowed)
            # And the robot's own voice still shuts the mic.
            self.assertFalse(gate._parse(conv, True).mic_allowed)
        finally:
            await gate.stop()

    async def test_high_level_loss_never_reopens_startup_override(self):
        gate = AudioGate(
            Path(self.temp.name) / "latch.sock",
            allow_during_startup=True,
            startup_grace_s=10,
        )
        await gate.start()
        try:
            alive = (
                "v=1 high_alive=1 high_busy=0 high_armed=0 remote_alive=1 "
                "ptt=0 rearm=0 mic=0 speaker=1"
            )
            lost = alive.replace("high_alive=1", "high_alive=0").replace(
                "speaker=1", "speaker=0"
            )
            self.assertTrue(gate._parse(alive, False).speaker_allowed)
            self.assertFalse(gate._parse(lost, False).speaker_allowed)
        finally:
            await gate.stop()


if __name__ == "__main__":
    unittest.main()
