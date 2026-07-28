import sys
import tempfile
import unittest
from array import array
from pathlib import Path
from types import SimpleNamespace

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice_r1"))

from hb_voice.resilience import (  # noqa: E402
    PttTurnAudioBuffer,
    VoiceRuntimeStatus,
    apply_pcm16_gain,
    is_realtime_connection_error,
    pcm_bytes_for_ms,
    realtime_service_state,
)


class PttTurnAudioBufferTest(unittest.TestCase):
    def test_short_turn_is_dropped_without_emitting_frames(self):
        audio = PttTurnAudioBuffer(minimum_bytes=10)
        audio.begin()
        self.assertEqual([], audio.add("short", 9))
        self.assertEqual((False, 9), audio.finish())
        self.assertFalse(audio.capturing)

    def test_threshold_flushes_preroll_and_allows_commit(self):
        audio = PttTurnAudioBuffer(minimum_bytes=10)
        audio.begin()
        self.assertEqual([], audio.add("first", 4))
        self.assertEqual(["first", "second"], audio.add("second", 6))
        self.assertEqual(["third"], audio.add("third", 3))
        self.assertEqual((True, 13), audio.finish())

    def test_100ms_s16_mono_at_24khz_is_4800_bytes(self):
        self.assertEqual(4800, pcm_bytes_for_ms(24000, 100))


class PcmGainTest(unittest.TestCase):
    def test_plus_6db_approximately_doubles_amplitude(self):
        source = array("h", [1000, -1000]).tobytes()
        output = array("h")
        output.frombytes(apply_pcm16_gain(source, 6.0))
        self.assertEqual([1995, -1995], output.tolist())

    def test_gain_saturates_instead_of_overflowing(self):
        source = array("h", [30000, -30000]).tobytes()
        output = array("h")
        output.frombytes(apply_pcm16_gain(source, 12.0))
        self.assertEqual([32767, -32768], output.tolist())


class RealtimeConnectionErrorTest(unittest.TestCase):
    def test_keepalive_timeout_requires_reconnect(self):
        llm = object()
        frame = SimpleNamespace(
            processor=llm,
            error="sent 1011 keepalive ping timeout; no close frame received",
            exception=None,
        )
        self.assertTrue(is_realtime_connection_error(frame, llm))

    def test_validation_error_does_not_restart_session(self):
        llm = object()
        frame = SimpleNamespace(
            processor=llm,
            error="input_audio_buffer_commit_empty: buffer too small",
            exception=None,
        )
        self.assertFalse(is_realtime_connection_error(frame, llm))

    def test_error_from_another_processor_is_ignored(self):
        frame = SimpleNamespace(
            processor=object(),
            error="keepalive ping timeout",
            exception=None,
        )
        self.assertFalse(is_realtime_connection_error(frame, object()))

    def test_initial_errno_113_connect_failure_requires_reconnect(self):
        llm = object()
        frame = SimpleNamespace(
            processor=llm,
            error=(
                "Error connecting: Multiple exceptions: "
                "[Errno 113] Connect call failed ('172.66.0.243', 443)"
            ),
            exception=None,
        )
        self.assertTrue(is_realtime_connection_error(frame, llm))

    def test_dns_and_refused_connections_require_reconnect(self):
        for details in (
            "Temporary failure in name resolution",
            "[Errno 111] Connection refused",
        ):
            with self.subTest(details=details):
                frame = SimpleNamespace(processor=None, error=details, exception=None)
                self.assertTrue(is_realtime_connection_error(frame))


class RealtimeServiceStateTest(unittest.TestCase):
    def test_ready_service_with_stopped_receive_loop_is_unusable(self):
        service = SimpleNamespace(
            _api_session_ready=True,
            _receive_task=SimpleNamespace(done=lambda: True),
            _websocket=object(),
        )
        self.assertEqual((True, True), realtime_service_state(service))

    def test_connecting_service_is_not_misclassified_as_stopped(self):
        service = SimpleNamespace(
            _api_session_ready=False,
            _receive_task=None,
            _websocket=None,
        )
        self.assertEqual((False, False), realtime_service_state(service))


class VoiceRuntimeStatusTest(unittest.TestCase):
    def test_status_is_atomic_and_contains_no_free_form_text(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "voice_status.env"
            status = VoiceRuntimeStatus(path)
            status.update(
                "reconnecting now",
                attempt=3,
                reason="DNS failed; secret must not be here",
            )
            text = path.read_text()
        self.assertIn("state=reconnecting_now", text)
        self.assertIn("attempt=3", text)
        self.assertIn("last_reason=DNS_failed_secret_must_not_be_here", text)


if __name__ == "__main__":
    unittest.main()
