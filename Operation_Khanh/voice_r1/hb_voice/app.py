"""OpenAI Realtime pipeline for the headless Robot Hanh Phuc R1 voice app."""

import asyncio
import time

from loguru import logger

from pipecat.observers.loggers.transcription_log_observer import (
    TranscriptionLogObserver,
)
from pipecat.pipeline.pipeline import Pipeline
from pipecat.pipeline.worker import PipelineParams, PipelineWorker
from pipecat.processors.aggregators.llm_context import LLMContext
from pipecat.processors.aggregators.llm_response_universal import (
    AssistantTurnStoppedMessage,
    LLMContextAggregatorPair,
    UserTurnMessageAddedMessage,
    UserTurnStoppedMessage,
)
from pipecat.services.openai.realtime.events import (
    AudioConfiguration,
    AudioInput,
    AudioOutput,
    InputAudioBufferClearEvent,
    InputAudioNoiseReduction,
    InputAudioTranscription,
    ResponseCancelEvent,
    SessionProperties,
)
from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService
from pipecat.transports.base_transport import BaseTransport
from pipecat.turns.user_stop import BaseUserTurnStopStrategy
from pipecat.workers.runner import WorkerRunner

from .config import VoiceConfig
from .gate import AudioGate, GateSnapshot
from .input import AlsaMicBridge, UnitreeMicBridge
from .output import UnitreeSpeakerBridge
from .resilience import (
    VoiceRuntimeStatus,
    is_realtime_connection_error,
    realtime_service_state,
)


async def run_bot(
    transport: BaseTransport,
    config: VoiceConfig,
    runtime_status: VoiceRuntimeStatus,
    attempt: int,
) -> bool:
    logger.info(f"Starting Robot Hanh Phuc R1 voice: {config.safe_summary()}")
    gate = AudioGate(
        config.gate_socket,
        activation_mode=config.activation_mode,
        allow_during_startup=config.allow_during_startup,
        startup_grace_s=config.startup_grace_s,
    )
    await gate.start()

    noise_reduction = (
        InputAudioNoiseReduction(type=config.resolved_noise_reduction)
        if config.resolved_noise_reduction
        else None
    )
    llm = OpenAIRealtimeLLMService(
        api_key=config.api_key,
        settings=OpenAIRealtimeLLMService.Settings(
            model=config.model,
            system_instruction=config.load_prompt(),
            session_properties=SessionProperties(
                audio=AudioConfiguration(
                    input=AudioInput(
                        transcription=InputAudioTranscription(
                            language=config.language,
                        ),
                        turn_detection=False,
                        noise_reduction=noise_reduction,
                    ),
                    output=AudioOutput(
                        voice=config.voice,
                        speed=config.speed,
                    ),
                ),
                tools=None,
                max_output_tokens=config.max_response_tokens,
            ),
        ),
    )

    context = LLMContext()
    user_aggregator, assistant_aggregator = LLMContextAggregatorPair(
        context,
        realtime_service_mode=True,
    )

    if config.mic_source == "r1_multicast":
        mic_processor = UnitreeMicBridge(
            bridge_path=config.bridge_path,
            network_interface=config.network_interface,
            mic_group_ip=config.mic_group_ip,
            mic_port=config.mic_port,
            mic_payload_mode=config.mic_payload_mode,
            input_gain_db=config.input_gain_db,
            audio_debug=config.audio_debug,
            gate=gate,
        )
    else:
        assert config.alsa_device is not None
        mic_processor = AlsaMicBridge(
            device=config.alsa_device,
            sample_rate=config.alsa_sample_rate,
            gate=gate,
            input_gain_db=config.input_gain_db,
            audio_debug=config.audio_debug,
        )

    speaker_processor = UnitreeSpeakerBridge(
        bridge_path=config.bridge_path,
        network_interface=config.network_interface,
        response_volume_percent=config.response_volume_percent,
        response_gain=config.response_gain,
        audio_debug=config.audio_debug,
        gate=gate,
    )

    pipeline = Pipeline(
        [
            transport.input(),
            mic_processor,
            user_aggregator,
            llm,
            speaker_processor,
            transport.output(),
            assistant_aggregator,
        ]
    )
    worker = PipelineWorker(
        pipeline,
        params=PipelineParams(enable_metrics=True, enable_usage_metrics=True),
        idle_timeout_secs=0,
        observers=[TranscriptionLogObserver()],
    )

    @worker.event_handler("on_pipeline_error")
    async def on_pipeline_error(worker, frame):
        if not is_realtime_connection_error(frame, llm):
            return
        runtime_status.update(
            "reconnecting",
            mic_ready=mic_processor.health_ready,
            attempt=attempt,
            reason="transport_error",
        )
        logger.error(
            "OpenAI Realtime connection is unusable; ending this session "
            "so the supervisor can reconnect"
        )
        await worker.cancel(reason="openai_realtime_connection_lost")

    async def drain_audio_before_ptt_commit() -> bool:
        started_at = time.monotonic()
        drained = await worker.flush_pipeline(timeout=3.0)
        elapsed_ms = (time.monotonic() - started_at) * 1000
        if drained:
            logger.info(f"PTT audio pipeline drained in {elapsed_ms:.0f}ms")
            return True

        logger.error(
            "PTT audio pipeline did not drain within 3s; cancelling the voice session"
        )
        await worker.cancel(reason="ptt_audio_flush_timeout")
        return False

    mic_processor.set_turn_commit_barrier(drain_audio_before_ptt_commit)

    @transport.event_handler("on_client_connected")
    async def on_client_connected(transport, client):
        logger.info("Headless voice transport connected")

    @transport.event_handler("on_client_disconnected")
    async def on_client_disconnected(transport, client):
        logger.info("Headless voice transport disconnected")
        await worker.cancel()

    @gate.add_callback
    async def on_high_level_preempt(old: GateSnapshot, new: GateSnapshot):
        if not old.speaker_allowed or new.speaker_allowed:
            return
        logger.warning("High-level audio preempted voice; clearing input and response")
        try:
            await llm.send_client_event(InputAudioBufferClearEvent())
            if old.voice_speaking:
                await llm.send_client_event(ResponseCancelEvent())
        except Exception as error:
            logger.warning(f"OpenAI preempt request was not active/connected: {error}")

    @user_aggregator.event_handler("on_user_turn_stopped")
    async def on_user_turn_stopped(
        aggregator,
        strategy: BaseUserTurnStopStrategy,
        message: UserTurnStoppedMessage,
    ):
        logger.info(f"User turn stopped at {message.timestamp}")

    @user_aggregator.event_handler("on_user_turn_message_added")
    async def on_user_turn_message_added(
        aggregator,
        message: UserTurnMessageAddedMessage,
    ):
        timestamp = f"[{message.timestamp}] " if message.timestamp else ""
        logger.info(f"Transcript: {timestamp}user: {message.content}")

    @assistant_aggregator.event_handler("on_assistant_turn_stopped")
    async def on_assistant_turn_stopped(
        aggregator,
        message: AssistantTurnStoppedMessage,
    ):
        timestamp = f"[{message.timestamp}] " if message.timestamp else ""
        logger.info(f"Transcript: {timestamp}assistant: {message.content}")

    runner = WorkerRunner(
        handle_sigint=False,
        handle_sigterm=False,
    )
    ever_ready = False

    async def connection_watchdog() -> None:
        nonlocal ever_ready
        started_at = time.monotonic()
        while True:
            openai_ready, receive_stopped = realtime_service_state(llm)
            mic_ready = mic_processor.health_ready
            if receive_stopped:
                runtime_status.update(
                    "reconnecting",
                    mic_ready=mic_ready,
                    attempt=attempt,
                    reason="receive_loop_stopped",
                )
                logger.error("OpenAI Realtime receive loop stopped; reconnecting")
                await worker.cancel(reason="openai_receive_loop_stopped")
                return
            if openai_ready:
                if not ever_ready:
                    logger.info("OpenAI Realtime session is ready")
                ever_ready = True
                runtime_status.update(
                    "ready",
                    openai_ready=True,
                    mic_ready=mic_ready,
                    attempt=attempt,
                )
            else:
                runtime_status.update(
                    "connecting",
                    mic_ready=mic_ready,
                    attempt=attempt,
                )
                if time.monotonic() - started_at >= config.connect_timeout_s:
                    runtime_status.update(
                        "reconnecting",
                        mic_ready=mic_ready,
                        attempt=attempt,
                        reason="connect_timeout",
                    )
                    logger.error(
                        "OpenAI Realtime was not ready within "
                        f"{config.connect_timeout_s:.1f}s; reconnecting"
                    )
                    await worker.cancel(reason="openai_connect_timeout")
                    return
            await asyncio.sleep(config.watchdog_interval_s)

    watchdog_task: asyncio.Task | None = None
    try:
        await runner.add_workers(worker)
        watchdog_task = asyncio.create_task(
            connection_watchdog(), name="hb-openai-watchdog"
        )
        await runner.run()
    finally:
        if watchdog_task:
            watchdog_task.cancel()
            await asyncio.gather(watchdog_task, return_exceptions=True)
        await gate.stop()
        runtime_status.update(
            "reconnecting",
            attempt=attempt,
        )
    return ever_ready
