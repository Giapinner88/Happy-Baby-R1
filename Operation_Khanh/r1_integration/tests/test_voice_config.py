import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice_r1"))

from hb_voice.config import VoiceConfig, VoiceConfigError  # noqa: E402


class VoiceConfigTest(unittest.TestCase):
    def load(self, **extra_env):
        env = {
            "OPENAI_API_KEY": "sk-test-only",
            "UNITREE_NETWORK_INTERFACE": "eth10",
            **extra_env,
        }
        with patch.dict(os.environ, env, clear=True):
            return VoiceConfig.load()

    def test_checked_in_tuning_is_valid(self):
        config = self.load()
        self.assertEqual("gpt-realtime-1.5", config.model)
        self.assertEqual("marin", config.voice)
        self.assertGreaterEqual(config.response_volume_percent, 0)
        self.assertLessEqual(config.response_volume_percent, 100)
        self.assertEqual("far_field", config.resolved_noise_reduction)
        self.assertEqual(6.0, config.input_gain_db)
        self.assertEqual("both", config.activation_mode)
        self.assertTrue(config.allow_during_startup)
        self.assertEqual(12, config.connect_timeout_s)
        self.assertTrue(config.load_prompt())

    def test_usb_auto_noise_reduction_uses_near_field(self):
        tuning = (HB_ROOT / "voice_r1" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('source: "r1_multicast"', 'source: "alsa_usb"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "UNITREE_NETWORK_INTERFACE": "eth10",
                    "ALSA_DEVICE": "plughw:CARD=RobotMic,DEV=0",
                },
                clear=True,
            ):
                config = VoiceConfig.load(path)
        self.assertEqual("near_field", config.resolved_noise_reduction)

    def test_api_key_is_required(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(VoiceConfigError, "OPENAI_API_KEY"):
                VoiceConfig.load()

    def test_invalid_activation_mode_is_rejected(self):
        tuning = (HB_ROOT / "voice_r1" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('mode: "both"', 'mode: "always_open"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning)
            with patch.dict(
                os.environ,
                {"OPENAI_API_KEY": "sk-test-only"},
                clear=True,
            ):
                with self.assertRaisesRegex(VoiceConfigError, "activation.mode"):
                    VoiceConfig.load(path)


if __name__ == "__main__":
    unittest.main()
