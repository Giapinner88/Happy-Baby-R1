"""Load and validate voice tuning plus robot-specific environment settings."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

VOICE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TUNING_PATH = VOICE_ROOT / "config" / "tuning.yaml"
DEFAULT_PROMPT_PATH = VOICE_ROOT / "config" / "prompt.txt"
DEFAULT_BRIDGE_PATH = VOICE_ROOT / "unitree_bridge" / "build" / "r1_bridge"
DEFAULT_GATE_SOCKET = Path("/run/hb/voice_gate.sock")
DEFAULT_GESTURE_SOCKET = Path("/run/hb/gesture_owner.sock")

SUPPORTED_VOICES = frozenset(
    {
        "alloy",
        "ash",
        "ballad",
        "coral",
        "echo",
        "sage",
        "shimmer",
        "verse",
        "marin",
        "cedar",
    }
)
SUPPORTED_NOISE_REDUCTION = frozenset({"auto", "near_field", "far_field", "off"})
SUPPORTED_ACTIVATION_MODES = frozenset({"both", "high_disarmed", "high_armed"})
SUPPORTED_TTS_PROVIDERS = frozenset({"elevenlabs", "openai"})
SUPPORTED_CONVERSATION_VAD_EAGERNESS = frozenset({"low", "medium", "high"})
SUPPORTED_MIC_SWITCH_MODES = frozenset({"manual", "auto"})
SUPPORTED_GESTURE_MODES = frozenset({"off", "shadow", "execute"})


class VoiceConfigError(ValueError):
    """Raised when voice configuration is incomplete or unsafe."""


@dataclass(frozen=True)
class VoiceConfig:
    api_key: str
    model: str
    voice: str
    speed: float
    language: str
    max_response_tokens: int
    tts_provider: str
    elevenlabs_api_key: str | None
    elevenlabs_voice_id: str | None
    elevenlabs_model: str
    elevenlabs_language: str
    elevenlabs_stability: float
    elevenlabs_similarity_boost: float
    elevenlabs_speed: float
    elevenlabs_use_speaker_boost: bool
    response_volume_percent: int
    response_gain: float
    echo_tail_s: float
    input_gain_db: float
    min_speech_peak: int
    noise_reduction: str
    audio_debug: bool
    mic_source: str
    network_interface: str
    bridge_path: Path
    alsa_device: str | None
    alsa_sample_rate: int
    mic_group_ip: str | None
    mic_port: int | None
    mic_payload_mode: str | None
    prompt_path: Path
    activation_mode: str
    allow_during_startup: bool
    startup_grace_s: float
    connect_timeout_s: float
    reconnect_initial_s: float
    reconnect_max_s: float
    watchdog_interval_s: float
    conversation_require_external_mic: bool
    conversation_vad_eagerness: str
    conversation_fallback_vad_eagerness: str
    conversation_mode_cue: bool
    mic_fallback_enabled: bool
    mic_switch_mode: str
    mic_source_cue: bool
    mic_fallback_gain_db: float
    mic_fallback_after_failures: int
    mic_fallback_recover_check_s: float
    mic_silence_s: float
    gesture_mode: str
    gesture_socket: Path
    gesture_timeout_s: float
    gate_socket: Path = DEFAULT_GATE_SOCKET

    @classmethod
    def load(cls, tuning_path: str | Path | None = None) -> "VoiceConfig":
        path = Path(tuning_path or os.getenv("VOICE_TUNING_PATH", DEFAULT_TUNING_PATH))
        if not path.is_absolute():
            path = VOICE_ROOT / path
        try:
            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except FileNotFoundError as error:
            raise VoiceConfigError(f"Voice tuning file not found: {path}") from error
        except yaml.YAMLError as error:
            raise VoiceConfigError(f"Invalid YAML in {path}: {error}") from error

        if not isinstance(raw, dict):
            raise VoiceConfigError(f"Voice tuning root must be a mapping: {path}")

        openai = _section(raw, "openai", path)
        tts = _section(raw, "tts", path)
        input_config = _section(raw, "input", path)
        audio = _section(raw, "audio", path)
        logging = _section(raw, "logging", path)
        activation = _section(raw, "activation", path)
        resilience = _section(raw, "resilience", path)
        conversation = _section(raw, "conversation", path)
        gesture_control = raw.get("gesture_control", {})
        if not isinstance(gesture_control, dict):
            raise VoiceConfigError(f"gesture_control must be a mapping in {path}")

        gesture_mode = str(gesture_control.get("mode", "off")).strip()
        if gesture_mode not in SUPPORTED_GESTURE_MODES:
            choices = ", ".join(sorted(SUPPORTED_GESTURE_MODES))
            raise VoiceConfigError(
                f"gesture_control.mode={gesture_mode!r} is unsupported; choose: {choices}"
            )
        gesture_socket = Path(str(gesture_control.get("socket", DEFAULT_GESTURE_SOCKET)))
        if not gesture_socket.is_absolute():
            raise VoiceConfigError("gesture_control.socket must be an absolute path")
        timeout_value = gesture_control.get("timeout_s", 1.0)
        if isinstance(timeout_value, bool) or not isinstance(timeout_value, (int, float)):
            raise VoiceConfigError("gesture_control.timeout_s must be a number")
        gesture_timeout_s = float(timeout_value)
        _range("gesture_control.timeout_s", gesture_timeout_s, 0.05, 5.0)

        api_key = os.getenv("OPENAI_API_KEY", "").strip()
        if not api_key or api_key == "sk-your-openai-key-here":
            raise VoiceConfigError(
                "OPENAI_API_KEY is missing/placeholder; set it in /etc/hb/stack.env"
            )

        model = _text(openai, "model", path)
        voice = _text(openai, "voice", path)
        if voice not in SUPPORTED_VOICES:
            choices = ", ".join(sorted(SUPPORTED_VOICES))
            raise VoiceConfigError(f"openai.voice={voice!r} is unsupported; choose: {choices}")

        speed = _number(openai, "speed", path)
        _range("openai.speed", speed, 0.25, 1.5)
        language = _text(openai, "language", path)
        max_response_tokens = _integer(openai, "max_response_tokens", path)
        _range("openai.max_response_tokens", max_response_tokens, 1, 4096)

        tts_provider = _text(tts, "provider", path)
        if tts_provider not in SUPPORTED_TTS_PROVIDERS:
            choices = ", ".join(sorted(SUPPORTED_TTS_PROVIDERS))
            raise VoiceConfigError(
                f"tts.provider={tts_provider!r} is unsupported; choose: {choices}"
            )

        elevenlabs_api_key = os.getenv("ELEVENLABS_API_KEY", "").strip() or None
        elevenlabs_voice_id = os.getenv("ELEVENLABS_VOICE_ID", "").strip() or None
        if tts_provider == "elevenlabs":
            if not elevenlabs_api_key or elevenlabs_api_key == "your-elevenlabs-api-key-here":
                raise VoiceConfigError(
                    "ELEVENLABS_API_KEY is missing/placeholder when "
                    "tts.provider=elevenlabs; "
                    "set it in /etc/hb/stack.env"
                )
            if not elevenlabs_voice_id or elevenlabs_voice_id == "your-elevenlabs-voice-id-here":
                raise VoiceConfigError(
                    "ELEVENLABS_VOICE_ID is missing/placeholder when "
                    "tts.provider=elevenlabs; "
                    "set it in /etc/hb/stack.env"
                )

        elevenlabs_model = _text(tts, "model", path)
        elevenlabs_language = _text(tts, "language", path)
        elevenlabs_stability = _number(tts, "stability", path)
        _range("tts.stability", elevenlabs_stability, 0.0, 1.0)
        elevenlabs_similarity_boost = _number(tts, "similarity_boost", path)
        _range("tts.similarity_boost", elevenlabs_similarity_boost, 0.0, 1.0)
        elevenlabs_speed = _number(tts, "speed", path)
        _range("tts.speed", elevenlabs_speed, 0.7, 1.2)
        elevenlabs_use_speaker_boost = _boolean(tts, "use_speaker_boost", path)

        response_volume_percent = _integer(audio, "response_volume_percent", path)
        _range("audio.response_volume_percent", response_volume_percent, 0, 100)
        response_gain = _number(audio, "response_gain", path)
        _range("audio.response_gain", response_gain, 0.0, 1.0)
        echo_tail_s = _number(audio, "echo_tail_s", path)
        _range("audio.echo_tail_s", echo_tail_s, 0.0, 3.0)
        input_gain_db = _number(input_config, "gain_db", path)
        _range("input.gain_db", input_gain_db, -12.0, 12.0)
        min_speech_peak = _integer(input_config, "min_speech_peak", path)
        _range("input.min_speech_peak", min_speech_peak, 0, 8000)
        mic_source = _text(input_config, "source", path)
        if mic_source not in {"r1_multicast", "alsa_usb"}:
            raise VoiceConfigError(
                f"input.source={mic_source!r} is unsupported; use r1_multicast or alsa_usb"
            )
        noise_reduction = _text(input_config, "noise_reduction", path)
        if noise_reduction not in SUPPORTED_NOISE_REDUCTION:
            choices = ", ".join(sorted(SUPPORTED_NOISE_REDUCTION))
            raise VoiceConfigError(
                f"input.noise_reduction={noise_reduction!r} is unsupported; choose: {choices}"
            )

        mic_fallback_enabled = _boolean(input_config, "fallback_to_robot_mic", path)
        if mic_fallback_enabled and mic_source != "alsa_usb":
            raise VoiceConfigError(
                "input.fallback_to_robot_mic=true only applies to input.source=alsa_usb; "
                "the PC1 multicast mic is already the fallback source"
            )
        mic_switch_mode = _text(input_config, "mic_switch", path)
        if mic_switch_mode not in SUPPORTED_MIC_SWITCH_MODES:
            choices = ", ".join(sorted(SUPPORTED_MIC_SWITCH_MODES))
            raise VoiceConfigError(
                f"input.mic_switch={mic_switch_mode!r} is unsupported; choose: {choices}"
            )
        # Only meaningful for the external primary: with input.source=r1_multicast
        # there is a single source and nothing for F2 to move between, so the knob
        # is simply inert rather than wrong.
        if (
            mic_source == "alsa_usb"
            and mic_switch_mode == "manual"
            and not mic_fallback_enabled
        ):
            raise VoiceConfigError(
                "input.mic_switch=manual needs input.fallback_to_robot_mic=true: the F2 "
                "key switches between the external mic and the robot mic, and that "
                "second source is what fallback_to_robot_mic configures. Set it true "
                "(nothing switches by itself in manual mode), or use mic_switch=auto"
            )
        mic_source_cue = _boolean(input_config, "mic_source_cue", path)
        mic_fallback_gain_db = _number(input_config, "fallback_gain_db", path)
        _range("input.fallback_gain_db", mic_fallback_gain_db, -12.0, 12.0)
        mic_fallback_after_failures = _integer(input_config, "fallback_after_failures", path)
        _range("input.fallback_after_failures", mic_fallback_after_failures, 1, 20)
        mic_fallback_recover_check_s = _number(input_config, "fallback_recover_check_s", path)
        _range("input.fallback_recover_check_s", mic_fallback_recover_check_s, 5.0, 300.0)
        mic_silence_s = _number(input_config, "mic_silence_s", path)
        if mic_silence_s != 0.0:
            _range("input.mic_silence_s", mic_silence_s, 5.0, 300.0)

        audio_debug = _boolean(logging, "audio_debug", path)
        activation_mode = _text(activation, "mode", path)
        if activation_mode not in SUPPORTED_ACTIVATION_MODES:
            choices = ", ".join(sorted(SUPPORTED_ACTIVATION_MODES))
            raise VoiceConfigError(
                f"activation.mode={activation_mode!r} is unsupported; choose: {choices}"
            )
        allow_during_startup = _boolean(activation, "allow_during_startup", path)
        startup_grace_s = _number(activation, "startup_grace_s", path)
        _range("activation.startup_grace_s", startup_grace_s, 0.0, 300.0)

        connect_timeout_s = _number(resilience, "connect_timeout_s", path)
        reconnect_initial_s = _number(resilience, "reconnect_initial_s", path)
        reconnect_max_s = _number(resilience, "reconnect_max_s", path)
        watchdog_interval_s = _number(resilience, "watchdog_interval_s", path)
        _range("resilience.connect_timeout_s", connect_timeout_s, 3.0, 120.0)
        _range("resilience.reconnect_initial_s", reconnect_initial_s, 0.5, 60.0)
        _range("resilience.reconnect_max_s", reconnect_max_s, 0.5, 300.0)
        _range("resilience.watchdog_interval_s", watchdog_interval_s, 0.2, 10.0)
        if reconnect_max_s < reconnect_initial_s:
            raise VoiceConfigError("resilience.reconnect_max_s must be >= reconnect_initial_s")

        conversation_require_external_mic = _boolean(
            conversation, "require_external_mic", path
        )
        if conversation_require_external_mic and mic_source != "alsa_usb":
            raise VoiceConfigError(
                "conversation.require_external_mic=true requires input.source=alsa_usb; "
                "hands-free F1 must start on the PC2 external mic. Losing that mic at "
                "runtime is covered by input.fallback_to_robot_mic, not by this setting"
            )
        conversation_vad_eagerness = _text(conversation, "vad_eagerness", path)
        if conversation_vad_eagerness not in SUPPORTED_CONVERSATION_VAD_EAGERNESS:
            choices = ", ".join(sorted(SUPPORTED_CONVERSATION_VAD_EAGERNESS))
            raise VoiceConfigError(
                "conversation.vad_eagerness="
                f"{conversation_vad_eagerness!r} is unsupported; choose: {choices}"
            )
        conversation_mode_cue = _boolean(conversation, "mode_cue", path)
        conversation_fallback_vad_eagerness = _text(
            conversation, "fallback_vad_eagerness", path
        )
        if conversation_fallback_vad_eagerness not in SUPPORTED_CONVERSATION_VAD_EAGERNESS:
            choices = ", ".join(sorted(SUPPORTED_CONVERSATION_VAD_EAGERNESS))
            raise VoiceConfigError(
                "conversation.fallback_vad_eagerness="
                f"{conversation_fallback_vad_eagerness!r} is unsupported; choose: {choices}"
            )
        network_interface = os.getenv("UNITREE_NETWORK_INTERFACE", "eth10").strip()
        if not network_interface:
            raise VoiceConfigError("UNITREE_NETWORK_INTERFACE cannot be empty")

        configured_bridge = Path(
            os.getenv("UNITREE_BRIDGE_PATH", str(DEFAULT_BRIDGE_PATH))
        ).expanduser()
        bridge_path = (
            configured_bridge if configured_bridge.is_absolute() else VOICE_ROOT / configured_bridge
        )

        alsa_device = os.getenv("ALSA_DEVICE", "").strip() or None
        alsa_sample_rate = _env_int("ALSA_SAMPLE_RATE", 48000)
        if alsa_sample_rate <= 0:
            raise VoiceConfigError("ALSA_SAMPLE_RATE must be greater than 0")
        if mic_source == "alsa_usb" and alsa_device:
            # Optional: left unset, the runtime finds the USB capture card by
            # itself, so changing the mic model does not mean editing a file.
            # Set it only to pin one of several USB mics.
            if "CARD=APE" in alsa_device:
                raise VoiceConfigError(
                    "ALSA_DEVICE cannot use the Jetson APE virtual card for the external mic; "
                    "connect a USB Audio capture device on PC2"
                )
            if alsa_device.startswith("hw:") and "CARD=" not in alsa_device:
                raise VoiceConfigError(
                    "ALSA_DEVICE must use a stable card name, for example "
                    "plughw:CARD=Audio,DEV=0"
                )

        mic_port = _optional_env_int("UNITREE_MIC_PORT")
        mic_payload_mode = os.getenv("UNITREE_MIC_PAYLOAD_MODE", "").strip() or None
        if mic_payload_mode not in {None, "raw", "rtp", "rtp-l16be"}:
            raise VoiceConfigError("UNITREE_MIC_PAYLOAD_MODE must be raw, rtp, or rtp-l16be")

        return cls(
            api_key=api_key,
            model=model,
            voice=voice,
            speed=speed,
            language=language,
            max_response_tokens=max_response_tokens,
            tts_provider=tts_provider,
            elevenlabs_api_key=elevenlabs_api_key,
            elevenlabs_voice_id=elevenlabs_voice_id,
            elevenlabs_model=elevenlabs_model,
            elevenlabs_language=elevenlabs_language,
            elevenlabs_stability=elevenlabs_stability,
            elevenlabs_similarity_boost=elevenlabs_similarity_boost,
            elevenlabs_speed=elevenlabs_speed,
            elevenlabs_use_speaker_boost=elevenlabs_use_speaker_boost,
            response_volume_percent=response_volume_percent,
            response_gain=response_gain,
            echo_tail_s=echo_tail_s,
            input_gain_db=input_gain_db,
            min_speech_peak=min_speech_peak,
            noise_reduction=noise_reduction,
            audio_debug=audio_debug,
            mic_source=mic_source,
            network_interface=network_interface,
            bridge_path=bridge_path.resolve(),
            alsa_device=alsa_device,
            alsa_sample_rate=alsa_sample_rate,
            mic_group_ip=os.getenv("UNITREE_MIC_GROUP_IP", "").strip() or None,
            mic_port=mic_port,
            mic_payload_mode=mic_payload_mode,
            prompt_path=DEFAULT_PROMPT_PATH,
            activation_mode=activation_mode,
            allow_during_startup=allow_during_startup,
            startup_grace_s=startup_grace_s,
            connect_timeout_s=connect_timeout_s,
            reconnect_initial_s=reconnect_initial_s,
            reconnect_max_s=reconnect_max_s,
            watchdog_interval_s=watchdog_interval_s,
            conversation_require_external_mic=conversation_require_external_mic,
            conversation_vad_eagerness=conversation_vad_eagerness,
            conversation_fallback_vad_eagerness=conversation_fallback_vad_eagerness,
            conversation_mode_cue=conversation_mode_cue,
            mic_fallback_enabled=mic_fallback_enabled,
            mic_switch_mode=mic_switch_mode,
            mic_source_cue=mic_source_cue,
            mic_fallback_gain_db=mic_fallback_gain_db,
            mic_fallback_after_failures=mic_fallback_after_failures,
            mic_fallback_recover_check_s=mic_fallback_recover_check_s,
            mic_silence_s=mic_silence_s,
            gesture_mode=gesture_mode,
            gesture_socket=gesture_socket,
            gesture_timeout_s=gesture_timeout_s,
        )

    @property
    def resolved_noise_reduction(self) -> str | None:
        return self.noise_reduction_for(self.mic_source)

    def noise_reduction_for(self, mic_source: str) -> str | None:
        """Noise reduction for the mic that is live right now.

        The runtime source can differ from ``input.source`` once the external
        mic drops and the fallback takes over, so ``auto`` has to follow the
        live source instead of the configured one.
        """
        if self.noise_reduction == "off":
            return None
        if self.noise_reduction != "auto":
            return self.noise_reduction
        return "far_field" if mic_source == "r1_multicast" else "near_field"

    def vad_eagerness_for(self, mic_source: str) -> str:
        """Hands-free stays on the fallback mic, but listens less eagerly.

        The PC1 mic sits on the robot body, so servo and speaker noise reach it
        far louder than they reach the external mic.
        """
        if mic_source == "r1_multicast":
            return self.conversation_fallback_vad_eagerness
        return self.conversation_vad_eagerness

    def input_gain_db_for(self, mic_source: str) -> float:
        return (
            self.mic_fallback_gain_db
            if mic_source == "r1_multicast"
            else self.input_gain_db
        )

    def load_prompt(self) -> str:
        try:
            prompt = self.prompt_path.read_text(encoding="utf-8").strip()
        except FileNotFoundError as error:
            raise VoiceConfigError(f"Voice prompt file not found: {self.prompt_path}") from error
        if not prompt:
            raise VoiceConfigError(f"Voice prompt file is empty: {self.prompt_path}")
        return prompt

    def safe_summary(self) -> str:
        tts_summary = (
            f"tts={self.tts_provider}"
            if self.tts_provider == "openai"
            else f"tts=elevenlabs/{self.elevenlabs_model} clone_configured=1"
        )
        return (
            f"model={self.model} {tts_summary} "
            f"openai_fallback_voice={self.voice} speed={self.speed} "
            f"response_volume={self.response_volume_percent}% "
            f"response_gain={self.response_gain} mic={self.mic_source} "
            f"input_gain={self.input_gain_db}dB "
            f"noise_reduction={self.resolved_noise_reduction} "
            f"activation={self.activation_mode} "
            f"startup_voice={self.allow_during_startup} "
            f"conversation_external_mic={self.conversation_require_external_mic} "
            f"conversation_vad={self.conversation_vad_eagerness} "
            f"mic_fallback={self.mic_fallback_enabled} "
            f"mic_switch={self.mic_switch_mode} "
            f"fallback_vad={self.conversation_fallback_vad_eagerness} "
            f"gesture_mode={self.gesture_mode}"
        )


def _section(raw: dict[str, Any], name: str, path: Path) -> dict[str, Any]:
    value = raw.get(name)
    if not isinstance(value, dict):
        raise VoiceConfigError(f"Missing/invalid section {name!r} in {path}")
    return value


def _text(section: dict[str, Any], key: str, path: Path) -> str:
    value = section.get(key)
    if not isinstance(value, str) or not value.strip():
        raise VoiceConfigError(f"{key!r} must be a non-empty string in {path}")
    return value.strip()


def _number(section: dict[str, Any], key: str, path: Path) -> float:
    value = section.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VoiceConfigError(f"{key!r} must be a number in {path}")
    return float(value)


def _integer(section: dict[str, Any], key: str, path: Path) -> int:
    value = section.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise VoiceConfigError(f"{key!r} must be an integer in {path}")
    return value


def _boolean(section: dict[str, Any], key: str, path: Path) -> bool:
    value = section.get(key)
    if not isinstance(value, bool):
        raise VoiceConfigError(f"{key!r} must be true or false in {path}")
    return value


def _range(name: str, value: float, minimum: float, maximum: float) -> None:
    if not minimum <= value <= maximum:
        raise VoiceConfigError(f"{name} must be between {minimum} and {maximum}")


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    try:
        return int(raw) if raw else default
    except ValueError as error:
        raise VoiceConfigError(f"{name} must be an integer") from error


def _optional_env_int(name: str) -> int | None:
    raw = os.getenv(name, "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError as error:
        raise VoiceConfigError(f"{name} must be an integer") from error
