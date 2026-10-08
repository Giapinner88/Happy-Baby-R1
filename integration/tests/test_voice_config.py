import os
import re
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice"))

from hb_voice.config import VoiceConfig, VoiceConfigError  # noqa: E402


class VoiceConfigTest(unittest.TestCase):
    def load(self, **extra_env):
        env = {
            "OPENAI_API_KEY": "sk-test-only",
            "ELEVENLABS_API_KEY": "elevenlabs-test-only",
            "ELEVENLABS_VOICE_ID": "voice-test-only",
            "UNITREE_NETWORK_INTERFACE": "eth10",
            "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
            "ALSA_SAMPLE_RATE": "48000",
            **extra_env,
        }
        with patch.dict(os.environ, env, clear=True):
            return VoiceConfig.load()

    def load_path(self, path, **extra_env):
        """Load an edited copy of the checked-in tuning file."""
        env = {
            "OPENAI_API_KEY": "sk-test-only",
            "ELEVENLABS_API_KEY": "elevenlabs-test-only",
            "ELEVENLABS_VOICE_ID": "voice-test-only",
            "UNITREE_NETWORK_INTERFACE": "eth10",
            "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
            "ALSA_SAMPLE_RATE": "48000",
            **extra_env,
        }
        with patch.dict(os.environ, env, clear=True):
            return VoiceConfig.load(path)

    def test_checked_in_tuning_is_valid(self):
        config = self.load()
        self.assertTrue(config.model.startswith("gpt-realtime"), config.model)
        self.assertEqual("sage", config.voice)
        self.assertEqual("elevenlabs", config.tts_provider)
        self.assertEqual("eleven_flash_v2_5", config.elevenlabs_model)
        self.assertEqual("vi", config.elevenlabs_language)
        self.assertEqual("voice-test-only", config.elevenlabs_voice_id)
        self.assertGreaterEqual(config.response_volume_percent, 0)
        self.assertLessEqual(config.response_volume_percent, 100)
        self.assertEqual("alsa_usb", config.mic_source)
        self.assertEqual("plughw:CARD=Audio,DEV=0", config.alsa_device)
        self.assertEqual(48000, config.alsa_sample_rate)
        self.assertEqual("near_field", config.resolved_noise_reduction)
        self.assertEqual(0.0, config.input_gain_db)
        self.assertEqual("both", config.activation_mode)
        self.assertTrue(config.allow_during_startup)
        self.assertEqual(12, config.connect_timeout_s)
        self.assertEqual("medium", config.conversation_vad_eagerness)
        self.assertTrue(config.conversation_require_external_mic)
        # Chat must continue without advertising a motor-action tool by default.
        self.assertEqual("off", config.gesture_mode)
        self.assertEqual(Path("/run/hb/gesture_owner.sock"), config.gesture_socket)
        self.assertEqual(1.0, config.gesture_timeout_s)
        self.assertEqual(20, config.mic_silence_s)
        self.assertEqual("low", config.conversation_fallback_vad_eagerness)
        # `mic_fallback_enabled` is an operator choice that flips per event, so
        # only the pairing is invariant: the robot mic sits beside the speaker,
        # so running it needs a tail long enough to outlast the DDS hop.  The
        # external mic sits at the speaker's mouth and barely echoes at all.
        if config.mic_fallback_enabled:
            self.assertGreaterEqual(config.echo_tail_s, 0.5)
        else:
            self.assertGreater(config.echo_tail_s, 0.0)
        # A drifting clone voice reads as the robot swapping speakers.
        self.assertGreaterEqual(config.elevenlabs_stability, 0.7)
        self.assertTrue(config.load_prompt())

    def test_absurd_echo_tail_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = re.sub(r"echo_tail_s: [0-9.]+", "echo_tail_s: 9.0", tuning, count=1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(VoiceConfigError, "audio.echo_tail_s"):
                self.load_path(path)

    def test_runtime_settings_follow_the_live_microphone(self):
        config = self.load()
        self.assertEqual("near_field", config.noise_reduction_for("alsa_usb"))
        self.assertEqual("far_field", config.noise_reduction_for("r1_multicast"))
        self.assertEqual("medium", config.vad_eagerness_for("alsa_usb"))
        self.assertEqual("low", config.vad_eagerness_for("r1_multicast"))
        self.assertEqual(config.input_gain_db, config.input_gain_db_for("alsa_usb"))
        self.assertEqual(
            config.mic_fallback_gain_db, config.input_gain_db_for("r1_multicast")
        )

    def test_fallback_is_rejected_for_the_multicast_primary(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('source: "alsa_usb"', 'source: "r1_multicast"', 1)
        tuning = tuning.replace(
            "require_external_mic: true", "require_external_mic: false", 1
        )
        # The operator may have the standby switched off; this test is about the
        # combination being rejected, so force it on regardless.
        tuning = tuning.replace(
            "fallback_to_robot_mic: false", "fallback_to_robot_mic: true", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(
                VoiceConfigError, "input.fallback_to_robot_mic"
            ):
                self.load_path(path)

    def test_checked_in_tuning_switches_microphones_by_hand(self):
        config = self.load()
        self.assertEqual("manual", config.mic_switch_mode)
        # F2 needs somewhere to switch to, and something to confirm it landed.
        self.assertTrue(config.mic_fallback_enabled)
        self.assertTrue(config.mic_source_cue)

    def test_gesture_mode_must_be_supported(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('mode: "off"', 'mode: "always"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(VoiceConfigError, "gesture_control.mode"):
                self.load_path(path)

    def test_gesture_socket_must_be_absolute(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace(
            'socket: "/run/hb/gesture_owner.sock"',
            'socket: "gesture_owner.sock"',
            1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(VoiceConfigError, "gesture_control.socket"):
                self.load_path(path)

    def test_manual_switching_needs_a_second_microphone(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace(
            "fallback_to_robot_mic: true", "fallback_to_robot_mic: false", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(VoiceConfigError, "input.mic_switch=manual"):
                self.load_path(path)

    def test_invalid_mic_switch_mode_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace("mic_switch: manual", "mic_switch: sometimes", 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(VoiceConfigError, "input.mic_switch"):
                self.load_path(path)

    def test_invalid_fallback_vad_eagerness_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace(
            'fallback_vad_eagerness: "low"', 'fallback_vad_eagerness: "instant"', 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with self.assertRaisesRegex(
                VoiceConfigError, "conversation.fallback_vad_eagerness"
            ):
                self.load_path(path)

    def test_usb_auto_noise_reduction_uses_near_field(self):
        config = self.load()
        self.assertEqual("near_field", config.resolved_noise_reduction)

    def test_r1_multicast_auto_noise_reduction_uses_far_field(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('source: "alsa_usb"', 'source: "r1_multicast"', 1)
        tuning = tuning.replace(
            "require_external_mic: true", "require_external_mic: false", 1
        )
        tuning = tuning.replace(
            "fallback_to_robot_mic: true", "fallback_to_robot_mic: false", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                    "ELEVENLABS_VOICE_ID": "voice-test-only",
                    "UNITREE_NETWORK_INTERFACE": "eth10",
                },
                clear=True,
            ):
                config = VoiceConfig.load(path)
        self.assertEqual("far_field", config.resolved_noise_reduction)

    def test_hands_free_must_start_on_the_external_microphone(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('source: "alsa_usb"', 'source: "r1_multicast"', 1)
        tuning = tuning.replace(
            "fallback_to_robot_mic: true", "fallback_to_robot_mic: false", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                    "ELEVENLABS_VOICE_ID": "voice-test-only",
                },
                clear=True,
            ):
                with self.assertRaisesRegex(
                    VoiceConfigError, "hands-free F1 must start on the PC2 external mic"
                ):
                    VoiceConfig.load(path)

    def test_external_mic_works_without_alsa_device(self):
        # The card name is the mic model's, not the robot's: pinning it made a
        # mic swap look exactly like a mic that fell off the USB bus, so an
        # unset ALSA_DEVICE now means "find the USB capture card at spawn time".
        with patch.dict(
            os.environ,
            {
                "OPENAI_API_KEY": "sk-test-only",
                "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                "ELEVENLABS_VOICE_ID": "voice-test-only",
            },
            clear=True,
        ):
            self.assertIsNone(VoiceConfig.load().alsa_device)

    def test_external_mic_rejects_jetson_ape_virtual_card(self):
        with self.assertRaisesRegex(VoiceConfigError, "APE virtual card"):
            self.load(ALSA_DEVICE="plughw:CARD=APE,DEV=0")

    def test_api_key_is_required(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(VoiceConfigError, "OPENAI_API_KEY"):
                VoiceConfig.load()

    def test_elevenlabs_credentials_are_required_for_clone_provider(self):
        with patch.dict(
            os.environ,
            {"OPENAI_API_KEY": "sk-test-only"},
            clear=True,
        ):
            with self.assertRaisesRegex(VoiceConfigError, "ELEVENLABS_API_KEY"):
                VoiceConfig.load()

    def test_openai_provider_does_not_require_elevenlabs_credentials(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('provider: "elevenlabs"', 'provider: "openai"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
                },
                clear=True,
            ):
                config = VoiceConfig.load(path)
        self.assertEqual("openai", config.tts_provider)
        self.assertIsNone(config.elevenlabs_api_key)
        self.assertIsNone(config.elevenlabs_voice_id)

    def test_invalid_activation_mode_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('mode: "both"', 'mode: "always_open"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                    "ELEVENLABS_VOICE_ID": "voice-test-only",
                },
                clear=True,
            ):
                with self.assertRaisesRegex(VoiceConfigError, "activation.mode"):
                    VoiceConfig.load(path)

    def test_invalid_tts_provider_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace(
            'provider: "elevenlabs"',
            'provider: "unsupported"',
            1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {"OPENAI_API_KEY": "sk-test-only"},
                clear=True,
            ):
                with self.assertRaisesRegex(VoiceConfigError, "tts.provider"):
                    VoiceConfig.load(path)

    def test_invalid_conversation_vad_eagerness_is_rejected(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('vad_eagerness: "medium"', 'vad_eagerness: "fast"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                    "ELEVENLABS_VOICE_ID": "voice-test-only",
                },
                clear=True,
            ):
                with self.assertRaisesRegex(VoiceConfigError, "conversation.vad_eagerness"):
                    VoiceConfig.load(path)

if __name__ == "__main__":
    unittest.main()
