"""tts.provider=elevenlabs must not take the voice down while the ElevenLabs keys are not set yet."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from hb_voice.config import DEFAULT_TUNING_PATH, VoiceConfig, VoiceConfigError


def tuning_with(provider):
    text = Path(DEFAULT_TUNING_PATH).read_text(encoding="utf-8")
    lines = [('  provider: "%s"' % provider) if line.strip().startswith("provider:") else line for line in text.splitlines()]
    handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    handle.write("\n".join(lines) + "\n")
    handle.close()
    return handle.name


class TtsFallbackTests(unittest.TestCase):
    def setUp(self):
        self.path = tuning_with("elevenlabs")
        self.addCleanup(os.unlink, self.path)

    def test_missing_keys_fall_back_to_openai_voice(self):
        with patch.dict(os.environ, {"ELEVENLABS_API_KEY": "", "ELEVENLABS_VOICE_ID": "", "HB_TTS_STRICT": ""}):
            self.assertEqual(VoiceConfig.load(self.path).tts_provider, "openai")
        with patch.dict(os.environ, {"ELEVENLABS_API_KEY": "your-elevenlabs-api-key-here", "ELEVENLABS_VOICE_ID": "v" * 20}):
            self.assertEqual(VoiceConfig.load(self.path).tts_provider, "openai")

    def test_keys_present_use_elevenlabs(self):
        with patch.dict(os.environ, {"ELEVENLABS_API_KEY": "k" * 32, "ELEVENLABS_VOICE_ID": "v" * 20}):
            config = VoiceConfig.load(self.path)
        self.assertEqual(config.tts_provider, "elevenlabs")
        self.assertEqual(config.elevenlabs_voice_id, "v" * 20)

    def test_strict_mode_still_fails_without_keys(self):
        with patch.dict(os.environ, {"ELEVENLABS_API_KEY": "", "ELEVENLABS_VOICE_ID": "", "HB_TTS_STRICT": "1"}):
            with self.assertRaises(VoiceConfigError):
                VoiceConfig.load(self.path)

    def test_repository_default_selects_elevenlabs(self):
        with patch.dict(os.environ, {"ELEVENLABS_API_KEY": "k" * 32, "ELEVENLABS_VOICE_ID": "v" * 20}):
            self.assertEqual(VoiceConfig.load().tts_provider, "elevenlabs")


if __name__ == "__main__":
    unittest.main()
