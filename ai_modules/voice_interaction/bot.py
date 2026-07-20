#
# Robot Hanh Phuc R1 voice app.
#

import os
import sys
from pathlib import Path

from dotenv import load_dotenv
from loguru import logger

from pipecat.observers.loggers.transcription_log_observer import TranscriptionLogObserver
from pipecat.pipeline.pipeline import Pipeline
from pipecat.pipeline.worker import PipelineParams, PipelineWorker
from pipecat.processors.aggregators.llm_context import LLMContext
from pipecat.processors.aggregators.llm_response_universal import (
    AssistantTurnStoppedMessage,
    LLMContextAggregatorPair,
    UserTurnMessageAddedMessage,
    UserTurnStoppedMessage,
)
from pipecat.runner.types import RunnerArguments
from pipecat.runner.utils import create_transport
from pipecat.services.llm_service import FunctionCallParams
from pipecat.services.openai.realtime.events import (
    AudioConfiguration,
    AudioOutput,
    AudioInput,
    InputAudioNoiseReduction,
    InputAudioTranscription,
    SemanticTurnDetection,
    SessionProperties,
)
from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService
from pipecat.transports.base_transport import BaseTransport, TransportParams
from pipecat.turns.user_stop import BaseUserTurnStopStrategy
from pipecat.workers.runner import WorkerRunner
from unitree_actions import UnitreeActionBridge
from unitree_mic import UnitreeMicBridge
from unitree_speaker import UnitreeSpeakerBridge

load_dotenv(override=True)

APP_DIR = Path(__file__).resolve().parent
PROMPT_PATH = APP_DIR / "prompt_robot_hanh_phuc_r1.txt"
DEFAULT_UNITREE_BRIDGE_PATH = APP_DIR / "unitree_bridge" / "build" / "r1_bridge"
DEFAULT_OPENAI_REALTIME_MODEL = "gpt-realtime-1.5"
OPENAI_REALTIME_MODEL_ALIASES = {
    "gpt_4o-audio": "gpt-4o-realtime-preview",
    "gpt-4o-audio": "gpt-4o-realtime-preview",
    "gpt-4o-audio-preview": "gpt-4o-realtime-preview",
    "gpt_4o-realtime": "gpt-4o-realtime-preview",
}

_unitree_action_bridge: UnitreeActionBridge | None = None


def load_system_prompt() -> str:
    return PROMPT_PATH.read_text(encoding="utf-8").strip()


def validate_openai_config() -> None:
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key or api_key == "sk-your-openai-key-here":
        raise RuntimeError(
            "OPENAI_API_KEY is missing or still uses the placeholder value. "
            "Edit robot-hanh-phuc-r1/.env and set a valid OpenAI API key."
        )


def env_enabled(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes", "on"}


def unitree_bridge_path() -> Path:
    return Path(os.getenv("UNITREE_BRIDGE_PATH", str(DEFAULT_UNITREE_BRIDGE_PATH))).expanduser()


def unitree_network_interface() -> str:
    return os.getenv("UNITREE_NETWORK_INTERFACE", "eth0")


def openai_realtime_model() -> str:
    model = os.getenv("OPENAI_REALTIME_MODEL", DEFAULT_OPENAI_REALTIME_MODEL).strip()
    if not model:
        return DEFAULT_OPENAI_REALTIME_MODEL

    resolved_model = OPENAI_REALTIME_MODEL_ALIASES.get(model, model)
    if resolved_model != model:
        logger.warning(
            "OPENAI_REALTIME_MODEL={} is not a Realtime model ID; using {} instead.",
            model,
            resolved_model,
        )
    return resolved_model


async def robot_action(params: FunctionCallParams, action: str):
    """Run a safe high-level Robot Hanh Phuc R1 action.

    Args:
        action: One of "stand_up", "stop_move", "damp", "start", or "zero_torque".
    """
    if not _unitree_action_bridge:
        await params.result_callback(
            {"ok": False, "reason": "Unitree action bridge is not enabled."}
        )
        return

    ok = _unitree_action_bridge.run(action)
    await params.result_callback({"ok": ok, "action": action})


transport_params = {
    "webrtc": lambda: TransportParams(
        audio_in_enabled=True,
        audio_out_enabled=True,
    ),
}


async def run_bot(transport: BaseTransport, runner_args: RunnerArguments):
    global _unitree_action_bridge

    validate_openai_config()
    logger.info("Starting Robot Hanh Phuc R1 voice bot")

    tools = []
    if env_enabled("UNITREE_ACTIONS_ENABLED"):
        _unitree_action_bridge = UnitreeActionBridge(
            bridge_path=unitree_bridge_path(),
            network_interface=unitree_network_interface(),
            cooldown_secs=float(os.getenv("UNITREE_ACTION_COOLDOWN_SECS", "2.0")),
        )
        tools.append(robot_action)

    llm = OpenAIRealtimeLLMService(
        api_key=os.environ["OPENAI_API_KEY"],
        settings=OpenAIRealtimeLLMService.Settings(
            model=openai_realtime_model(),
            system_instruction=load_system_prompt(),
            session_properties=SessionProperties(
                audio=AudioConfiguration(
                    input=AudioInput(
                        transcription=InputAudioTranscription(
                            language="vi",
                            prompt="Tiếng Việt trong sự kiện mẹ và bé. Robot Hạnh Phúc R1.",
                        ),
                        turn_detection=SemanticTurnDetection(
                            eagerness=os.getenv("OPENAI_TURN_EAGERNESS", "medium"),
                            create_response=True,
                            interrupt_response=env_enabled("OPENAI_INTERRUPT_RESPONSE"),
                        ),
                        noise_reduction=InputAudioNoiseReduction(type="near_field"),
                    ),
                    output=AudioOutput(
                        voice=os.getenv("OPENAI_VOICE", "alloy"),
                        speed=float(os.getenv("OPENAI_VOICE_SPEED", "1.0")),
                    ),
                ),
                tools=tools or None,
            ),
        ),
    )

    context = LLMContext()

    user_aggregator, assistant_aggregator = LLMContextAggregatorPair(
        context,
        realtime_service_mode=True,
    )

    pipeline_steps = [transport.input()]

    if env_enabled("UNITREE_MIC_ENABLED"):
        pipeline_steps.append(
            UnitreeMicBridge(
                bridge_path=unitree_bridge_path(),
                network_interface=unitree_network_interface(),
                segment_seconds=int(os.getenv("UNITREE_MIC_SEGMENT_SECS", "5")),
                mic_group_ip=os.getenv("UNITREE_MIC_GROUP_IP") or None,
                mic_port=(
                    int(os.environ["UNITREE_MIC_PORT"])
                    if os.getenv("UNITREE_MIC_PORT")
                    else None
                ),
                mic_payload_mode=(
                    os.getenv("UNITREE_MIC_PAYLOAD_MODE")
                    or ("rtp" if env_enabled("UNITREE_MIC_RTP") else None)
                ),
                audio_debug=env_enabled("UNITREE_AUDIO_DEBUG"),
                drop_browser_audio=env_enabled("UNITREE_DROP_BROWSER_AUDIO"),
            )
        )

    pipeline_steps.extend(
        [
            user_aggregator,
            llm,
        ]
    )

    if env_enabled("UNITREE_SPEAKER_ENABLED"):
        pipeline_steps.append(
            UnitreeSpeakerBridge(
                bridge_path=unitree_bridge_path(),
                network_interface=unitree_network_interface(),
                app_name=os.getenv("UNITREE_AUDIO_APP_NAME", "pipecat"),
                audio_debug=env_enabled("UNITREE_AUDIO_DEBUG"),
            )
        )

    pipeline_steps.extend(
        [
            transport.output(),
            assistant_aggregator,
        ]
    )

    pipeline = Pipeline(pipeline_steps)

    worker = PipelineWorker(
        pipeline,
        params=PipelineParams(
            enable_metrics=True,
            enable_usage_metrics=True,
        ),
        idle_timeout_secs=runner_args.pipeline_idle_timeout_secs,
        observers=[TranscriptionLogObserver()],
    )

    @transport.event_handler("on_client_connected")
    async def on_client_connected(transport, client):
        logger.info("Client connected")

    @transport.event_handler("on_client_disconnected")
    async def on_client_disconnected(transport, client):
        logger.info("Client disconnected")
        await worker.cancel()

    @user_aggregator.event_handler("on_user_turn_stopped")
    async def on_user_turn_stopped(
        aggregator,
        strategy: BaseUserTurnStopStrategy,
        message: UserTurnStoppedMessage,
    ):
        logger.info(f"User turn stopped at {message.timestamp}")

    @user_aggregator.event_handler("on_user_turn_message_added")
    async def on_user_turn_message_added(aggregator, message: UserTurnMessageAddedMessage):
        timestamp = f"[{message.timestamp}] " if message.timestamp else ""
        logger.info(f"Transcript: {timestamp}user: {message.content}")

    @assistant_aggregator.event_handler("on_assistant_turn_stopped")
    async def on_assistant_turn_stopped(aggregator, message: AssistantTurnStoppedMessage):
        timestamp = f"[{message.timestamp}] " if message.timestamp else ""
        logger.info(f"Transcript: {timestamp}assistant: {message.content}")

    runner = WorkerRunner(handle_sigint=runner_args.handle_sigint)

    await runner.add_workers(worker)
    await runner.run()


async def bot(runner_args: RunnerArguments):
    transport = await create_transport(runner_args, transport_params)
    await run_bot(transport, runner_args)


if __name__ == "__main__":
    from pipecat.runner.run import main

    if not {"-h", "--help"}.intersection(sys.argv):
        validate_openai_config()
    main()
