"""
Pipeline OpenAI Realtime dành cho ứng dụng giọng nói (voice app) của Robot Hanh Phuc R1 (chế độ headless).
Quản lý luồng âm thanh đầu vào/đầu ra, tích hợp với hệ thống điều khiển của robot.
"""

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
    LLMAssistantAggregator,
    LLMUserAggregator,
    UserTurnMessageAddedMessage,
    UserTurnStoppedMessage,
)
from pipecat.frames.frames import LLMUpdateSettingsFrame
from pipecat.processors.frame_processor import FrameDirection
from pipecat.services.elevenlabs.tts import ElevenLabsTTSService
from pipecat.services.openai.realtime.events import (
    AudioConfiguration,
    AudioInput,
    AudioOutput,
    InputAudioBufferClearEvent,
    InputAudioNoiseReduction,
    InputAudioTranscription,
    ResponseCancelEvent,
    SemanticTurnDetection,
    SessionProperties,
)
from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService
from pipecat.transcriptions.language import Language
from pipecat.transports.base_transport import BaseTransport
from pipecat.turns.user_stop import BaseUserTurnStopStrategy
from pipecat.workers.runner import WorkerRunner

from .config import VoiceConfig
from .gate import AudioGate, GateSnapshot
from .gesture_tools import GESTURE_TOOL_INSTRUCTIONS, GestureToolController
from .gesture_response import GestureResponseGuard
from .input import AlsaMicBridge, RobotMicFallback, UnitreeMicBridge
from .output import UnitreeSpeakerBridge
from .resilience import (
    VoiceRuntimeStatus,
    is_realtime_connection_error,
    realtime_service_state,
)


def _openai_session_properties(
    config: VoiceConfig,
    *,
    conversation_mode: bool = False,
    mic_source: str | None = None,
    tools: list | None = None,
) -> SessionProperties:
    """
    Xây dựng cấu hình phiên (SessionProperties) cho OpenAI Realtime, hỗ trợ tổng hợp giọng nói nội bộ hoặc từ bên thứ 3.

    `mic_source` là micro đang được sử dụng ở thời điểm hiện tại, có thể khác với micro mặc định trong cấu hình: 
    chức năng phát hiện giọng nói (turn detection) và giảm ồn (noise reduction) phải bám theo micro dự phòng 
    khi micro ngoài bị mất kết nối.
    """
    live_mic_source = mic_source or config.mic_source
    output_modalities = None
    audio_output = None
    if config.tts_provider == "elevenlabs":
        output_modalities = ["text"]
    else:
        audio_output = AudioOutput(
            voice=config.voice,
            speed=config.speed,
        )

    return SessionProperties(
        output_modalities=output_modalities,
        audio=AudioConfiguration(
            input=AudioInput(
                transcription=InputAudioTranscription(
                    language=config.language,
                ),
                turn_detection=(
                    SemanticTurnDetection(
                        eagerness=config.vad_eagerness_for(live_mic_source),
                        create_response=True,
                        interrupt_response=False,
                    )
                    if conversation_mode
                    else False
                ),
                noise_reduction=(
                    InputAudioNoiseReduction(
                        type=config.noise_reduction_for(live_mic_source)
                    )
                    if config.noise_reduction_for(live_mic_source)
                    else None
                ),
            ),
            output=audio_output,
        ),
        tools=tools,
        tool_choice="auto" if tools else None,
        max_output_tokens=config.max_response_tokens,
    )


class ServerTurnRealtimeLLMService(GestureResponseGuard, OpenAIRealtimeLLMService):
    """Realtime service that never answers a turn the server already answered.

    In hands-free mode the session runs `SemanticTurnDetection(create_response=True)`,
    so OpenAI produces a reply for every turn it commits on its own.  Pipecat asks
    for a reply of its own the first time it sees a context frame (the initial
    branch of `_handle_context`), and that frame arrives together with the first
    user message of the connection.  Both replies then answer the same question:
    on 2026-08-18 every session's opening question was answered twice, once with
    the two ElevenLabs streams overlapping (`unable to append audio to context`),
    which is heard as the robot switching voices mid-answer.

    Suppressing the base class's request - rather than reimplementing its
    bookkeeping - keeps a future Pipecat that renames these internals falling back
    to the old duplicate-answer behaviour instead of leaving the robot mute.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Push-to-talk sessions carry `turn_detection=False`, where nothing
        # server-side creates a response and Pipecat's request is the only one
        # there is.  Off until hands-free mode is confirmed applied.
        self._server_creates_responses = False
        self._suppress_context_response = False

    def set_server_creates_responses(self, enabled: bool) -> None:
        """Track whether the live session has OpenAI creating turns for us."""
        self._server_creates_responses = enabled

    async def _handle_evt_session_updated(self, evt):
        await super()._handle_evt_session_updated(evt)
        if self._service_tools():
            advertised = evt.session.tools or []
            names = [tool.get("name") for tool in advertised if isinstance(tool, dict)]
            logger.info("Realtime session confirmed tools={}", names)
            if "perform_gesture" not in names:
                logger.error("Realtime session is missing perform_gesture; voice gestures cannot run")

    async def _handle_context(self, context) -> None:
        first_context = self._context is None
        if not first_context or not self._server_creates_responses:
            await super()._handle_context(context)
            return
        self._suppress_context_response = True
        try:
            await super()._handle_context(context)
        finally:
            self._suppress_context_response = False

    async def _create_response(self):
        if self._suppress_context_response:
            logger.info(
                "Skipping the initial-context response: hands-free turn detection "
                "already asked OpenAI to answer this turn"
            )
            return
        await super()._create_response()


class ToolResultSafeAssistantAggregator(LLMAssistantAggregator):
    """Keep deferred Realtime tool results queued across transcript updates.

    Pipecat 1.5.x clears its deferred-result flag for every context frame. A
    short assistant transcript emitted before a tool call must not cancel the
    pending tool result that should be sent after audio playback finishes.
    """

    async def push_context_frame(
        self, direction: FrameDirection = FrameDirection.DOWNSTREAM
    ):
        deferred_tool_result = self._push_context_on_bot_stopped_speaking
        await super().push_context_frame(direction)
        if direction is FrameDirection.DOWNSTREAM and deferred_tool_result:
            self._push_context_on_bot_stopped_speaking = True

    async def push_aggregation(self) -> str:
        deferred_tool_result = self._push_context_on_bot_stopped_speaking
        aggregation = await super().push_aggregation()
        if deferred_tool_result:
            self._push_context_on_bot_stopped_speaking = True
        return aggregation


def _create_external_tts(config: VoiceConfig) -> ElevenLabsTTSService | None:
    """Khởi tạo dịch vụ ElevenLabs streaming TTS nếu nhà cung cấp giọng clone được chọn."""
    if config.tts_provider != "elevenlabs":
        return None

    assert config.elevenlabs_api_key
    assert config.elevenlabs_voice_id
    return ElevenLabsTTSService(
        api_key=config.elevenlabs_api_key,
        settings=ElevenLabsTTSService.Settings(
            voice=config.elevenlabs_voice_id,
            model=config.elevenlabs_model,
            language=Language(config.elevenlabs_language),
            stability=config.elevenlabs_stability,
            similarity_boost=config.elevenlabs_similarity_boost,
            speed=config.elevenlabs_speed,
            use_speaker_boost=config.elevenlabs_use_speaker_boost,
        ),
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

    # Gesture availability is a runtime Dev-state decision from integration,
    # never a boot-time voice setting. This keeps normal conversation free of
    # motor tools until run_r1 owns LOCOMOTION.
    gesture_controller = GestureToolController(
        mode="off",
        socket_path=config.gesture_socket,
        timeout_s=config.gesture_timeout_s,
    )
    if config.gesture_mode != "off":
        logger.warning(
            "gesture_control.mode={} is ignored at startup; Dev locomotion owns gesture availability",
            config.gesture_mode,
        )
    gesture_tools: list = []
    base_system_instruction = config.load_prompt()
    system_instruction = base_system_instruction

    llm = ServerTurnRealtimeLLMService(
        guard_gesture_text=False,
        api_key=config.api_key,
        settings=OpenAIRealtimeLLMService.Settings(
            model=config.model,
            system_instruction=system_instruction,
            session_properties=_openai_session_properties(config, tools=gesture_tools),
        ),
    )
    external_tts = _create_external_tts(config)

    context = LLMContext(tools=gesture_tools)
    user_aggregator = LLMUserAggregator(
        context,
        _realtime_service_mode=True,
    )
    assistant_aggregator = ToolResultSafeAssistantAggregator(
        context,
        _realtime_service_mode=True,
        _paired_user_aggregator=user_aggregator,
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
            min_speech_peak=config.min_speech_peak,
        )
    else:
        # Unset ALSA_DEVICE means "find the USB capture card at spawn time",
        # which is the normal setup: the card name belongs to the mic model.
        mic_device_label = config.alsa_device or "auto-detected USB capture card"
        mic_fallback = (
            RobotMicFallback(
                bridge_path=config.bridge_path,
                network_interface=config.network_interface,
                gain_db=config.mic_fallback_gain_db,
                after_failures=config.mic_fallback_after_failures,
                recover_check_s=config.mic_fallback_recover_check_s,
                mic_group_ip=config.mic_group_ip,
                mic_port=config.mic_port,
                mic_payload_mode=config.mic_payload_mode,
            )
            if config.mic_fallback_enabled
            else None
        )
        mic_processor = AlsaMicBridge(
            device=config.alsa_device,
            sample_rate=config.alsa_sample_rate,
            gate=gate,
            input_gain_db=config.input_gain_db,
            audio_debug=config.audio_debug,
            min_speech_peak=config.min_speech_peak,
            silence_s=config.mic_silence_s,
            fallback=mic_fallback,
            switch_mode=config.mic_switch_mode,
        )
        if config.conversation_require_external_mic:
            logger.info(
                "Double-F1 hands-free input starts on the PC2 ALSA mic: "
                f"device={mic_device_label} sample_rate={config.alsa_sample_rate}"
            )
        if mic_fallback and config.mic_switch_mode == "manual":
            logger.info(
                "Microphone source switching is MANUAL: press F2 to move between the "
                f"external mic ({mic_device_label}) and the PC1 multicast mic. "
                "Nothing switches on its own - a dead external mic is reported in the "
                "log and stays selected. Hands-free drops to eagerness="
                f"{config.conversation_fallback_vad_eagerness} while the robot mic runs"
            )
        elif mic_fallback:
            logger.info(
                "Robot mic standby is armed: the PC1 multicast mic takes over after "
                f"{mic_fallback.after_failures} failed external-mic starts or "
                f"{config.mic_silence_s:.0f}s of silence, and hands-free drops to "
                f"eagerness={config.conversation_fallback_vad_eagerness} while it runs"
            )
        else:
            logger.warning(
                "Robot mic standby is OFF (input.fallback_to_robot_mic=false): the "
                "external mic is the only input. A dead mic will be reported in the "
                "log but the robot will not hear anything until it is fixed"
            )

    speaker_processor = UnitreeSpeakerBridge(
        bridge_path=config.bridge_path,
        network_interface=config.network_interface,
        response_volume_percent=config.response_volume_percent,
        response_gain=config.response_gain,
        audio_debug=config.audio_debug,
        gate=gate,
        playback_tail_s=config.echo_tail_s,
    )
    vad_ready = False
    active_mic_source = config.mic_source
    runtime_status.set_mic_source(active_mic_source)
    realtime_settings_lock = asyncio.Lock()
    gesture_session_enabled = False

    async def sync_conversation_mode(*, force: bool = False) -> bool:
        """
        Đồng bộ chế độ nhận diện giọng nói (VAD) của Realtime với chế độ mới nhất từ hệ thống điều phối (coordinator).

        Bridge micro sẽ giữ trạng thái đóng an toàn (fail-closed) cho luồng âm thanh rảnh tay cho đến khi coroutine này hoàn tất.
        Việc cập nhật cấu hình thông qua LLM frame giúp đồng bộ trạng thái VAD thủ công/server cục bộ của Pipecat với OpenAI.

        `force`: Ép buộc gửi lại cấu hình ngay cả khi chế độ được yêu cầu không thay đổi nhưng nội dung bên trong thay đổi 
        (ví dụ: khi thực hiện chuyển đổi micro).
        """
        nonlocal vad_ready
        async with realtime_settings_lock:
            requested = gate.snapshot.conv_mode
            openai_ready, _ = realtime_service_state(llm)
            if not openai_ready:
                vad_ready = False
                if not requested:
                    await mic_processor.set_conversation_mode(False)
                logger.warning(
                    "Conversation mode requested={} but Realtime is not ready; "
                    "hands-free audio remains closed",
                    requested,
                )
                return False

            try:
                if not requested:
                    await llm.send_client_event(InputAudioBufferClearEvent())
                if requested != vad_ready or force:
                    await llm.process_frame(
                        LLMUpdateSettingsFrame(
                            delta=OpenAIRealtimeLLMService.Settings(
                                session_properties=_openai_session_properties(
                                    config,
                                    conversation_mode=requested,
                                    mic_source=active_mic_source,
                                    tools=gesture_tools,
                                )
                            ),
                            service=llm,
                        ),
                        FrameDirection.DOWNSTREAM,
                    )
                vad_ready = requested
                # Hands-free means OpenAI creates a response for every turn it
                # commits, so Pipecat must not ask for one of its own.
                llm.set_server_creates_responses(vad_ready)
                await mic_processor.set_conversation_mode(vad_ready)
                logger.info(
                    "Conversation VAD is now {}",
                    "enabled" if vad_ready else "disabled",
                )
                return True
            except Exception as error:
                vad_ready = False
                try:
                    await llm.send_client_event(InputAudioBufferClearEvent())
                except Exception:
                    pass
                if not requested:
                    # F1 just switched back to PTT.  Even if OpenAI rejects
                    # the settings update, clear the local transition latch;
                    # otherwise every later Select hold stays blocked until a
                    # full voice-service restart.
                    try:
                        await mic_processor.set_conversation_mode(False)
                    except Exception as reset_error:
                        logger.error(
                            "Could not restore local PTT path after F1 disable: {}",
                            reset_error,
                        )
                logger.error(
                    f"Conversation VAD update failed; hands-free audio remains closed: {error}"
                )
                return False

    async def sync_gesture_dev_mode(*, force: bool = False) -> bool:
        """Advertise gesture tools only while run_r1 owns Dev locomotion."""
        nonlocal gesture_tools, system_instruction, gesture_session_enabled
        async with realtime_settings_lock:
            requested = gate.snapshot.gesture_dev_ready
            if not force and requested == gesture_session_enabled:
                return requested

            if requested:
                gesture_controller.set_mode("execute")
                next_tools = gesture_controller.tools
                # Handler registration must precede the remote session update:
                # the first post-Dev turn may call a function immediately.
                llm._register_advertised_tool_handlers(next_tools)
            else:
                await gesture_controller.disable_for_dev_exit()
                next_tools = []

            gesture_tools = next_tools
            context.set_tools(next_tools)
            system_instruction = (
                f"{base_system_instruction}\n\n{GESTURE_TOOL_INSTRUCTIONS}"
                if next_tools
                else base_system_instruction
            )
            llm.set_gesture_text_guard(
                bool(next_tools) and config.tts_provider == "elevenlabs"
            )
            try:
                await llm.process_frame(
                    LLMUpdateSettingsFrame(
                        delta=OpenAIRealtimeLLMService.Settings(
                            system_instruction=system_instruction,
                            session_properties=_openai_session_properties(
                                config,
                                conversation_mode=gate.snapshot.conv_mode,
                                mic_source=active_mic_source,
                                tools=next_tools,
                            ),
                        ),
                        service=llm,
                    ),
                    FrameDirection.DOWNSTREAM,
                )
                # Pipecat does not remove old local function handlers until it
                # receives a tool frame. We update the shared context above and
                # prune explicitly so Dev exit cannot leave a callable handler.
                llm._sync_registered_tool_handlers(next_tools)
            except Exception as error:
                logger.error("Gesture Dev session update failed; disabling tools: {}", error)
                gesture_controller.set_mode("off")
                gesture_tools = []
                context.set_tools([])
                system_instruction = base_system_instruction
                llm.set_gesture_text_guard(False)
                try:
                    await llm.process_frame(
                        LLMUpdateSettingsFrame(
                            delta=OpenAIRealtimeLLMService.Settings(
                                system_instruction=base_system_instruction,
                                session_properties=_openai_session_properties(
                                    config,
                                    conversation_mode=gate.snapshot.conv_mode,
                                    mic_source=active_mic_source,
                                    tools=[],
                                ),
                            ),
                            service=llm,
                        ),
                        FrameDirection.DOWNSTREAM,
                    )
                except Exception as rollback_error:
                    logger.error("Could not publish gesture disable rollback: {}", rollback_error)
                llm._sync_registered_tool_handlers([])
                gesture_session_enabled = False
                return False

            gesture_session_enabled = requested
            logger.info(
                "Gesture tools {} for Dev locomotion: tools={} owner={}",
                "enabled" if requested else "disabled",
                [tool.name for tool in next_tools],
                config.gesture_socket,
            )
            return requested

    async def on_mic_source_changed(source: str) -> None:
        """Keep the Realtime session honest about which mic is live.

        Hands-free stays available on the robot mic, but that mic sits on the
        robot body where servo and speaker noise are loud, so turn detection
        and noise reduction both move with the source.
        """
        nonlocal active_mic_source
        if source == active_mic_source:
            return
        active_mic_source = source
        runtime_status.set_mic_source(source)
        logger.warning(
            "Voice input switched to {} (vad_eagerness={}, noise_reduction={})",
            source,
            config.vad_eagerness_for(source),
            config.noise_reduction_for(source),
        )
        if vad_ready:
            await sync_conversation_mode(force=True)
        if config.mic_source_cue:
            # One note so the operator knows F2 landed without reading the log.
            await speaker_processor.play_mic_source_cue(source != "r1_multicast")

    if isinstance(mic_processor, AlsaMicBridge):
        mic_processor.set_source_change_callback(on_mic_source_changed)

    pipeline_steps = [
        transport.input(),
        mic_processor,
        user_aggregator,
        llm,
    ]
    if external_tts:
        pipeline_steps.append(external_tts)
    pipeline_steps.extend(
        [
            speaker_processor,
            transport.output(),
            assistant_aggregator,
        ]
    )
    pipeline = Pipeline(pipeline_steps)
    worker = PipelineWorker(
        pipeline,
        params=PipelineParams(enable_metrics=True, enable_usage_metrics=True),
        idle_timeout_secs=0,
        observers=[TranscriptionLogObserver()],
    )

    @worker.event_handler("on_pipeline_error")
    async def on_pipeline_error(worker, frame):
        provider = None
        if is_realtime_connection_error(frame, llm):
            provider = "openai"
        elif external_tts and is_realtime_connection_error(frame, external_tts):
            provider = "elevenlabs"
        if provider is None:
            return
        runtime_status.update(
            "reconnecting",
            mic_ready=mic_processor.health_ready,
            attempt=attempt,
            reason=f"{provider}_transport_error",
        )
        logger.error(
            f"{provider} connection is unusable; ending this session "
            "so the supervisor can reconnect"
        )
        await worker.cancel(reason=f"{provider}_connection_lost")

    async def drain_audio_before_ptt_commit() -> bool:
        started_at = time.monotonic()
        drained = await worker.flush_pipeline(timeout=3.0)
        elapsed_ms = (time.monotonic() - started_at) * 1000
        if drained:
            logger.info(f"PTT audio pipeline drained in {elapsed_ms:.0f}ms")
            return True

        logger.error("PTT audio pipeline did not drain within 3s; cancelling the voice session")
        await worker.cancel(reason="ptt_audio_flush_timeout")
        return False

    mic_processor.set_turn_commit_barrier(drain_audio_before_ptt_commit)

    async def drop_uncommitted_turn_audio() -> None:
        """Retract a PTT turn's audio from the Realtime input buffer.

        Audio the microphone already pushed downstream has been appended
        server-side.  Nothing else removes it, so without this a discarded turn
        is silently prepended to whatever is committed next.
        """
        await llm.send_client_event(InputAudioBufferClearEvent())
        logger.info("Discarded PTT turn audio was cleared from the Realtime buffer")

    mic_processor.set_turn_discard_hook(drop_uncommitted_turn_audio)

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
            await llm.broadcast_interruption()
        except Exception as error:
            logger.warning(f"Voice preempt request was not active/connected: {error}")

    @gate.add_callback
    async def on_double_tap_interrupt(old: GateSnapshot, new: GateSnapshot):
        # Double-tap Select trên tay cầm -> ngắt NGAY lượt hiện tại (bot im + xoá input),
        # nhưng phiên voice vẫn sống, nói lại được liền. Chỉ kích ở cạnh lên của pulse.
        if not new.interrupt or old.interrupt:
            return
        logger.warning("Double-tap Select: ngắt lượt hiện tại (interrupt)")
        try:
            await llm.send_client_event(InputAudioBufferClearEvent())
            # Select is the unconditional voice-stop gesture.  A response can
            # still be generating before the speaker bridge marks it as
            # speaking, so do not gate cancellation on that local marker.
            await llm.send_client_event(ResponseCancelEvent())
            await llm.broadcast_interruption()
        except Exception as error:
            logger.warning(f"Interrupt request was not active/connected: {error}")

    @gate.add_callback
    async def on_conversation_gate_change(old: GateSnapshot, new: GateSnapshot):
        if old.conv_mode == new.conv_mode:
            return
        logger.info("Conversation mode requested={}", new.conv_mode)
        applied = await sync_conversation_mode()
        # Âm báo phát theo trạng thái THẬT sau khi đồng bộ, không theo yêu cầu:
        # nếu OpenAI từ chối bật chế độ auto thì người vận hành phải nghe tiếng
        # đi xuống (vẫn đang ở chế độ giữ nút), không phải tiếng đi lên.
        if config.conversation_mode_cue:
            await speaker_processor.play_mode_cue(applied and new.conv_mode)

    @gate.add_callback
    async def on_gesture_dev_gate_change(old: GateSnapshot, new: GateSnapshot):
        if old.gesture_dev_ready == new.gesture_dev_ready:
            return
        logger.info(
            "Gesture Dev eligibility changed: {} (armed={} state={} remote_alive={})",
            new.gesture_dev_ready,
            new.high_armed,
            new.high_state,
            new.remote_alive,
        )
        await sync_gesture_dev_mode()

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
                if gate.snapshot.conv_mode and not vad_ready:
                    await sync_conversation_mode()
                if gate.snapshot.gesture_dev_ready != gesture_session_enabled:
                    await sync_gesture_dev_mode()
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
        # A coordinator datagram can arrive before callbacks are registered.
        # Reconcile once after the pipeline is attached.
        await sync_gesture_dev_mode(force=True)
        watchdog_task = asyncio.create_task(connection_watchdog(), name="hb-openai-watchdog")
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
