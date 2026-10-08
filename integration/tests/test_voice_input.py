"""Tests for the microphone bridge: process arguments and turn teardown."""

import shutil
import sys
import time
import unittest
from array import array
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice"))

from hb_voice import input as input_module  # noqa: E402
from hb_voice.gate import GateSnapshot  # noqa: E402
from hb_voice.input import (  # noqa: E402
    ALSA_NO_CARD_DEVICE,
    AlsaMicBridge,
    RobotMicFallback,
    UnitreeMicBridge,
    usb_capture_cards,
)


def _bridge(**kwargs) -> UnitreeMicBridge:
    return UnitreeMicBridge(
        bridge_path="/opt/hb/r1_bridge",
        network_interface="eth10",
        **kwargs,
    )


def _fallback() -> RobotMicFallback:
    return RobotMicFallback(
        bridge_path=Path(__file__),  # any existing file: only presence is checked
        network_interface="eth10",
        gain_db=6.0,
        after_failures=3,
        recover_check_s=20.0,
    )


def _alsa(device: str | None = "plughw:CARD=Audio,DEV=0", **kwargs) -> AlsaMicBridge:
    gate = SimpleNamespace(add_callback=lambda cb: cb, snapshot=GateSnapshot())
    bridge = AlsaMicBridge(
        device=device,
        sample_rate=48000,
        gate=gate,
        **kwargs,
    )
    # Presence of the capture binary is a precondition of switching back, and it
    # must not depend on whether the machine running the tests has alsa-utils.
    bridge._alsa_bridge_path = Path(__file__)
    return bridge


def _silence() -> bytes:
    """PCM at the dither floor a dead capture chain actually produces."""
    return array("h", [1, -1] * 256).tobytes()


class MicBridgeArgumentsTest(unittest.TestCase):
    """`r1_bridge mic` reads its arguments by position, so gaps are fatal."""

    def test_defaults_pass_nothing_beyond_the_interface(self):
        self.assertEqual(
            ["/opt/hb/r1_bridge", "mic", "eth10"],
            _bridge()._mic_bridge_args(),
        )

    def test_group_ip_fills_the_seconds_slot_it_would_otherwise_shift_into(self):
        # Without the placeholder the IP landed in `seconds` (std::stoi keeps
        # the leading 239), the port in `group_ip`, and "raw" in `port`, where
        # std::stoi threw out of main and aborted the bridge.  That is the
        # documented rollback to the PC1 multicast mic, so it has to be exact.
        args = _bridge(
            mic_group_ip="239.168.123.161",
            mic_port=5555,
            mic_payload_mode="raw",
        )._mic_bridge_args()
        self.assertEqual(
            ["/opt/hb/r1_bridge", "mic", "eth10", "0", "239.168.123.161", "5555", "raw"],
            args,
        )

    def test_group_ip_alone_still_keeps_the_seconds_slot(self):
        self.assertEqual(
            ["/opt/hb/r1_bridge", "mic", "eth10", "0", "239.168.123.161"],
            _bridge(mic_group_ip="239.168.123.161")._mic_bridge_args(),
        )

    def test_payload_mode_without_a_port_still_gets_the_default_port(self):
        self.assertEqual(
            ["/opt/hb/r1_bridge", "mic", "eth10", "0", "239.168.123.161", "5555", "rtp"],
            _bridge(
                mic_group_ip="239.168.123.161", mic_payload_mode="rtp"
            )._mic_bridge_args(),
        )


class DiscardedTurnTest(unittest.IsolatedAsyncioTestCase):
    """Audio already handed to the Realtime API has to be retracted."""

    async def test_streamed_turn_is_cleared_when_the_mic_closes_under_ptt(self):
        cleared = []
        bridge = _bridge()
        bridge.set_turn_discard_hook(lambda: _record(cleared))
        bridge._pipeline_ready = True
        bridge._turn_audio.begin()
        bridge._turn_active = True

        held = GateSnapshot(coordinator_mic=True, ptt=True)
        speaking = GateSnapshot(coordinator_mic=True, ptt=True, voice_speaking=True)
        await bridge._on_gate_change(held, speaking)

        self.assertEqual(1, len(cleared))
        self.assertFalse(bridge._turn_active)
        self.assertFalse(bridge._turn_audio.capturing)

    async def test_turn_that_never_streamed_leaves_the_buffer_alone(self):
        # Nothing was pushed downstream, so nothing is queued server-side and a
        # clear here could only discard somebody else's audio.
        cleared = []
        bridge = _bridge()
        bridge.set_turn_discard_hook(lambda: _record(cleared))
        bridge._pipeline_ready = True
        bridge._turn_audio.begin()

        held = GateSnapshot(coordinator_mic=True, ptt=True)
        speaking = GateSnapshot(coordinator_mic=True, ptt=True, voice_speaking=True)
        await bridge._on_gate_change(held, speaking)

        self.assertEqual([], cleared)
        self.assertFalse(bridge._turn_audio.capturing)

    async def test_switching_to_hands_free_clears_a_half_sent_turn(self):
        # Semantic VAD would otherwise commit the leftover fragment on its own
        # and the robot would answer something nobody said.
        cleared = []
        bridge = _bridge()
        bridge.set_turn_discard_hook(lambda: _record(cleared))
        bridge._pipeline_ready = True
        bridge._turn_audio.begin()
        bridge._turn_active = True

        await bridge._on_gate_change(GateSnapshot(), GateSnapshot(conv_mode=True))

        self.assertEqual(1, len(cleared))
        self.assertTrue(bridge._mode_transition_pending)

    async def test_a_failing_clear_never_propagates(self):
        bridge = _bridge()

        async def explode():
            raise RuntimeError("websocket is closed")

        bridge.set_turn_discard_hook(explode)
        bridge._turn_active = True
        await bridge._discard_active_turn()
        self.assertFalse(bridge._turn_active)


class ManualMicSwitchTest(unittest.IsolatedAsyncioTestCase):
    """F2 owns the microphone source; health never moves it."""

    async def test_manual_mode_leaves_a_dead_external_mic_selected(self):
        # A source that changes by itself mid-event is worse than one that is
        # broken but predictable: on 2026-08-06 the two health detectors traded
        # the mic three times in three minutes and neither one was usable.
        bridge = _alsa(silence_s=20.0, fallback=_fallback(), switch_mode="manual")
        bridge._last_sound_at = time.monotonic() - 999
        bridge._note_audio_content(_silence())
        self.assertFalse(bridge._silent_stall)
        self.assertEqual("alsa_usb", bridge.active_source)

    async def test_auto_mode_still_switches_on_silence(self):
        bridge = _alsa(silence_s=20.0, fallback=_fallback(), switch_mode="auto")
        bridge._last_sound_at = time.monotonic() - 999
        bridge._note_audio_content(_silence())
        self.assertTrue(bridge._silent_stall)

    async def test_manual_mode_never_counts_its_way_into_a_switch(self):
        bridge = _alsa(fallback=_fallback(), switch_mode="manual")
        for _ in range(10):
            await bridge._on_attempt_finished(produced_audio=False)
        self.assertEqual("alsa_usb", bridge.active_source)
        self.assertIsNone(bridge._recover_task)

    async def test_f2_moves_the_source_both_ways(self):
        bridge = _alsa(fallback=_fallback(), switch_mode="manual")
        await bridge.request_source(False)
        self.assertEqual("r1_multicast", bridge.active_source)
        self.assertEqual(16000, bridge._source_sample_rate)
        self.assertEqual(6.0, bridge._input_gain_db)

        await bridge.request_source(True)
        self.assertEqual("alsa_usb", bridge.active_source)
        self.assertEqual(48000, bridge._source_sample_rate)
        self.assertEqual(0.0, bridge._input_gain_db)

    async def test_f2_refuses_when_there_is_no_robot_mic_configured(self):
        bridge = _alsa(switch_mode="manual")
        await bridge.request_source(False)
        self.assertEqual("alsa_usb", bridge.active_source)

    async def test_gate_mic_external_edge_drives_the_switch(self):
        bridge = _alsa(fallback=_fallback(), switch_mode="manual")
        bridge._pipeline_ready = True
        await bridge._on_gate_change(
            GateSnapshot(mic_external=True), GateSnapshot(mic_external=False)
        )
        self.assertEqual("r1_multicast", bridge.active_source)

    async def test_unrelated_gate_traffic_does_not_touch_the_source(self):
        bridge = _alsa(fallback=_fallback(), switch_mode="manual")
        bridge._pipeline_ready = True
        await bridge._on_gate_change(
            GateSnapshot(mic_external=True),
            GateSnapshot(mic_external=True, coordinator_speaker=True),
        )
        self.assertEqual("alsa_usb", bridge.active_source)


class UsbCardDetectionTest(unittest.TestCase):
    """The card name belongs to the mic model, so it is discovered, not pinned.

    Swapping the TTGK receiver for a BOYALINK (2026-08-18) renamed the ALSA card
    from `Audio` to `BOYALINK`, and the pinned ALSA_DEVICE then failed with
    `No such device` - indistinguishable in the log from a mic that fell off the
    USB bus.
    """

    BOYALINK = (
        " 0 [BOYALINK       ]: USB-Audio - BOYALINK\n"
        "                      Shenzhen jiayz BOYALINK at usb-3610000.xhci-3, full speed\n"
        " 1 [HDA            ]: tegra-hda - NVIDIA Jetson Orin NX HDA\n"
        "                      NVIDIA Jetson Orin NX HDA at 0x3518000 irq 112\n"
        " 2 [APE            ]: tegra-ape - NVIDIA Jetson Orin NX APE\n"
        "                      NVIDIA Jetson Orin NX APE\n"
    )
    TTGK = (
        " 0 [HDA            ]: tegra-hda - NVIDIA Jetson Orin NX HDA\n"
        " 1 [Audio          ]: USB-Audio - USB Audio\n"
        "                      Generic USB Audio at usb-3610000.xhci-3, full speed\n"
    )

    def setUp(self):
        self._root = TemporaryDirectory()
        self.addCleanup(self._root.cleanup)
        self.root = Path(self._root.name)
        self._restore = (input_module.SOUND_CARDS_PATH, input_module.SOUND_CARD_ROOT)
        input_module.SOUND_CARD_ROOT = self.root
        input_module.SOUND_CARDS_PATH = self.root / "cards"
        self.addCleanup(self._put_back)
        self.cards("")

    def _put_back(self):
        input_module.SOUND_CARDS_PATH, input_module.SOUND_CARD_ROOT = self._restore

    def cards(self, text: str, capture=(0,)):
        """Write a `/proc/asound` tree; `capture` lists cards that can record."""
        shutil.rmtree(self.root, ignore_errors=True)
        self.root.mkdir()
        (self.root / "cards").write_text(text)
        for index in capture:
            (self.root / f"card{index}" / "pcm0c").mkdir(parents=True)

    def test_usb_capture_card_is_found_next_to_the_jetson_cards(self):
        self.cards(self.BOYALINK)
        self.assertEqual([(0, "BOYALINK")], usb_capture_cards())

    def test_playback_only_usb_audio_is_not_a_microphone(self):
        # A USB speaker or dock is a USB-Audio card too; without the capture
        # check the mic would be "found" on a device that cannot record.
        self.cards(self.BOYALINK, capture=())
        self.assertEqual([], usb_capture_cards())

    def test_auto_device_follows_a_mic_swap_without_a_restart(self):
        self.cards(self.BOYALINK)
        bridge = _alsa(device=None)
        self.assertEqual("plughw:CARD=BOYALINK,DEV=0", bridge._resolve_alsa_device())
        self.cards(self.TTGK, capture=(1,))
        self.assertEqual("plughw:CARD=Audio,DEV=0", bridge._resolve_alsa_device())

    def test_no_card_yields_a_device_that_fails_instead_of_no_device(self):
        # The read loop ends for good if a spawn is skipped, taking the standby
        # mic with it, so an unpluggged mic has to keep failing attempts.
        self.cards(self.BOYALINK)
        bridge = _alsa(device=None)
        bridge._resolve_alsa_device()
        self.cards("", capture=())
        self.assertEqual(ALSA_NO_CARD_DEVICE, bridge._resolve_alsa_device())
        self.assertFalse(bridge._external_card_present())
        self.cards(self.BOYALINK)
        self.assertTrue(bridge._external_card_present())

    def test_explicit_device_is_never_overridden_by_detection(self):
        self.cards(self.BOYALINK)
        bridge = _alsa(device="plughw:CARD=Audio,DEV=0")
        self.assertEqual("plughw:CARD=Audio,DEV=0", bridge._resolve_alsa_device())
        # Pinned means pinned: a different card being present is not this mic.
        self.assertFalse(bridge._external_card_present())

    def test_pinned_device_still_rejects_the_jetson_virtual_card(self):
        with self.assertRaisesRegex(ValueError, "APE virtual card"):
            _alsa(device="plughw:CARD=APE,DEV=0")

    def test_pinned_device_still_rejects_a_numeric_card_index(self):
        # Index shifts with plug order: the BOYALINK took card 0 and pushed the
        # Jetson HDA to 1.
        with self.assertRaisesRegex(ValueError, "stable card name"):
            _alsa(device="hw:1,0")


async def _record(sink: list) -> None:
    sink.append(True)


if __name__ == "__main__":
    unittest.main()
