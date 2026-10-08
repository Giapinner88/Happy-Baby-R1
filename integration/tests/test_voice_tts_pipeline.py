"""Validate provider selection for the production voice pipeline."""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, patch

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT / "voice"))

from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService  # noqa: E402

from hb_voice.app import (  # noqa: E402
    ServerTurnRealtimeLLMService,
    _create_external_tts,
    _openai_session_properties,
)
from hb_voice.config import VoiceConfig  # noqa: E402
from hb_voice.gesture_tools import GestureToolController  # noqa: E402


class VoiceTtsPipelineTest(unittest.TestCase):
    """Check native OpenAI and external ElevenLabs synthesis configurations."""

    def test_elevenlabs_uses_text_only_openai_session(self):
        """Route OpenAI text into the configured ElevenLabs cloned voice."""
        with patch.dict(
            os.environ,
            {
                "OPENAI_API_KEY": "sk-test-only",
                "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                "ELEVENLABS_VOICE_ID": "voice-test-only",
                "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
            },
            clear=True,
        ):
            config = VoiceConfig.load()
            session = _openai_session_properties(config)
            tts = _create_external_tts(config)

        self.assertEqual(["text"], session.output_modalities)
        self.assertIsNone(session.audio.output)
        self.assertIsNotNone(tts)
        self.assertEqual("voice-test-only", tts._settings.voice)
        self.assertEqual("eleven_flash_v2_5", tts._settings.model)
        self.assertEqual("vi", tts._settings.language)

    def test_openai_provider_restores_native_audio(self):
        """Keep a configuration-only rollback path to OpenAI native audio."""
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        tuning = tuning.replace('provider: "elevenlabs"', 'provider: "openai"', 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.yaml"
            path.write_text(tuning, encoding="utf-8")
            with patch.dict(
                os.environ,
                {
                    "OPENAI_API_KEY": "sk-test-only",
                    "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
                },
                clear=True,
            ):
                config = VoiceConfig.load(path)
                session = _openai_session_properties(config)
                tts = _create_external_tts(config)

        self.assertIsNone(session.output_modalities)
        self.assertEqual("sage", session.audio.output.voice)
        self.assertIsNone(tts)

    def test_default_voice_chat_has_no_gesture_tool(self):
        """Default voice keeps normal TTS/chat while leaving motor tools off."""
        with patch.dict(
            os.environ,
            {
                "OPENAI_API_KEY": "sk-test-only",
                "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                "ELEVENLABS_VOICE_ID": "voice-test-only",
                "ALSA_DEVICE": "plughw:CARD=Audio,DEV=0",
            },
            clear=True,
        ):
            config = VoiceConfig.load()

        controller = GestureToolController(
            mode=config.gesture_mode,
            socket_path=config.gesture_socket,
            timeout_s=config.gesture_timeout_s,
        )
        session = _openai_session_properties(config, tools=controller.tools)

        self.assertEqual("off", config.gesture_mode)
        self.assertEqual([], controller.tools)
        self.assertIsNone(session.tools)
        self.assertIsNone(session.tool_choice)
        self.assertEqual(["text"], session.output_modalities)


class DuplicateAnswerTest(unittest.IsolatedAsyncioTestCase):
    """One committed turn must produce exactly one answer.

    Hands-free sessions ask OpenAI to answer every turn it commits
    (`SemanticTurnDetection(create_response=True)`).  Pipecat asks for an answer
    of its own when it first sees a context frame, and that frame arrives with
    the first user message - so on 2026-08-18 the opening question of every
    session was answered twice, once with both ElevenLabs streams overlapping.
    """

    def _service(self) -> ServerTurnRealtimeLLMService:
        with patch.dict(
            os.environ,
            {
                "OPENAI_API_KEY": "sk-test-only",
                "ELEVENLABS_API_KEY": "elevenlabs-test-only",
                "ELEVENLABS_VOICE_ID": "voice-test-only",
            },
            clear=True,
        ):
            config = VoiceConfig.load()
        return ServerTurnRealtimeLLMService(
            api_key=config.api_key,
            settings=OpenAIRealtimeLLMService.Settings(
                model=config.model,
                system_instruction="test",
                session_properties=_openai_session_properties(config),
            ),
        )

    async def test_hands_free_first_context_does_not_ask_for_a_second_answer(self):
        service = self._service()
        service.set_server_creates_responses(True)
        context = object()
        with patch.object(
            OpenAIRealtimeLLMService, "_create_response", new_callable=AsyncMock
        ) as create_response, patch.object(
            OpenAIRealtimeLLMService,
            "_process_completed_function_calls",
            new_callable=AsyncMock,
        ) as process_calls:
            await service._handle_context(context)

        create_response.assert_not_awaited()
        # The context still has to be adopted, or every later turn would be
        # treated as the first one.
        self.assertIs(context, service._context)
        process_calls.assert_awaited_once_with(send_new_results=False)

    async def test_push_to_talk_still_gets_its_only_answer(self):
        # With turn_detection=False nothing server-side answers, so suppressing
        # Pipecat's request here would leave the robot mute.
        service = self._service()
        service.set_server_creates_responses(False)
        with patch.object(
            OpenAIRealtimeLLMService, "_create_response", new_callable=AsyncMock
        ) as create_response, patch.object(
            OpenAIRealtimeLLMService,
            "_process_completed_function_calls",
            new_callable=AsyncMock,
        ):
            await service._handle_context(object())

        create_response.assert_awaited_once()

    async def test_suppression_lasts_only_for_the_initial_context(self):
        service = self._service()
        service.set_server_creates_responses(True)
        with patch.object(
            OpenAIRealtimeLLMService, "_create_response", new_callable=AsyncMock
        ) as create_response, patch.object(
            OpenAIRealtimeLLMService,
            "_process_completed_function_calls",
            new_callable=AsyncMock,
        ):
            await service._handle_context(object())
            # A tool result later in the session legitimately needs a response.
            await service._create_response()

        create_response.assert_awaited_once()


if __name__ == "__main__":
    unittest.main()
