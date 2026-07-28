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
    response_volume_percent: int
    response_gain: float
    input_gain_db: float
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
        input_config = _section(raw, "input", path)
        audio = _section(raw, "audio", path)
        logging = _section(raw, "logging", path)
        activation = _section(raw, "activation", path)
        resilience = _section(raw, "resilience", path)

        api_key = os.getenv("OPENAI_API_KEY", "").strip()
        if not api_key or api_key == "sk-your-openai-key-here":
            raise VoiceConfigError(
                "OPENAI_API_KEY is missing/placeholder; set it in /etc/hb/stack.env"
            )

        model = _text(openai, "model", path)
        voice = _text(openai, "voice", path)
        if voice not in SUPPORTED_VOICES:
            choices = ", ".join(sorted(SUPPORTED_VOICES))
            raise VoiceConfigError(
                f"openai.voice={voice!r} is unsupported; choose: {choices}"
            )

        speed = _number(openai, "speed", path)
        _range("openai.speed", speed, 0.25, 1.5)
        language = _text(openai, "language", path)
        max_response_tokens = _integer(openai, "max_response_tokens", path)
        _range("openai.max_response_tokens", max_response_tokens, 1, 4096)

        response_volume_percent = _integer(audio, "response_volume_percent", path)
        _range("audio.response_volume_percent", response_volume_percent, 0, 100)
        response_gain = _number(audio, "response_gain", path)
        _range("audio.response_gain", response_gain, 0.0, 1.0)
        input_gain_db = _number(input_config, "gain_db", path)
        _range("input.gain_db", input_gain_db, -12.0, 12.0)
        mic_source = _text(input_config, "source", path)
        if mic_source not in {"r1_multicast", "alsa_usb"}:
            raise VoiceConfigError(
                f"input.source={mic_source!r} is unsupported; "
                "use r1_multicast or alsa_usb"
            )
        noise_reduction = _text(input_config, "noise_reduction", path)
        if noise_reduction not in SUPPORTED_NOISE_REDUCTION:
            choices = ", ".join(sorted(SUPPORTED_NOISE_REDUCTION))
            raise VoiceConfigError(
                f"input.noise_reduction={noise_reduction!r} is unsupported; choose: {choices}"
            )

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
            raise VoiceConfigError(
                "resilience.reconnect_max_s must be >= reconnect_initial_s"
            )
        network_interface = os.getenv("UNITREE_NETWORK_INTERFACE", "eth10").strip()
        if not network_interface:
            raise VoiceConfigError("UNITREE_NETWORK_INTERFACE cannot be empty")

        configured_bridge = Path(
            os.getenv("UNITREE_BRIDGE_PATH", str(DEFAULT_BRIDGE_PATH))
        ).expanduser()
        bridge_path = (
            configured_bridge
            if configured_bridge.is_absolute()
            else VOICE_ROOT / configured_bridge
        )

        alsa_device = os.getenv("ALSA_DEVICE", "").strip() or None
        alsa_sample_rate = _env_int("ALSA_SAMPLE_RATE", 16000)
        if alsa_sample_rate <= 0:
            raise VoiceConfigError("ALSA_SAMPLE_RATE must be greater than 0")
        if mic_source == "alsa_usb":
            if not alsa_device:
                raise VoiceConfigError(
                    "ALSA_DEVICE is required when input.source=alsa_usb"
                )
            if alsa_device.startswith("hw:") and "CARD=" not in alsa_device:
                raise VoiceConfigError(
                    "ALSA_DEVICE must use a stable card name, for example "
                    "plughw:CARD=RobotMic,DEV=0"
                )

        mic_port = _optional_env_int("UNITREE_MIC_PORT")
        mic_payload_mode = os.getenv("UNITREE_MIC_PAYLOAD_MODE", "").strip() or None
        if mic_payload_mode not in {None, "raw", "rtp", "rtp-l16be"}:
            raise VoiceConfigError(
                "UNITREE_MIC_PAYLOAD_MODE must be raw, rtp, or rtp-l16be"
            )

        return cls(
            api_key=api_key,
            model=model,
            voice=voice,
            speed=speed,
            language=language,
            max_response_tokens=max_response_tokens,
            response_volume_percent=response_volume_percent,
            response_gain=response_gain,
            input_gain_db=input_gain_db,
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
        )

    @property
    def resolved_noise_reduction(self) -> str | None:
        if self.noise_reduction == "off":
            return None
        if self.noise_reduction != "auto":
            return self.noise_reduction
        return "far_field" if self.mic_source == "r1_multicast" else "near_field"

    def load_prompt(self) -> str:
        try:
            prompt = self.prompt_path.read_text(encoding="utf-8").strip()
        except FileNotFoundError as error:
            raise VoiceConfigError(
                f"Voice prompt file not found: {self.prompt_path}"
            ) from error
        if not prompt:
            raise VoiceConfigError(f"Voice prompt file is empty: {self.prompt_path}")
        return prompt

    def safe_summary(self) -> str:
        return (
            f"model={self.model} voice={self.voice} speed={self.speed} "
            f"response_volume={self.response_volume_percent}% "
            f"response_gain={self.response_gain} mic={self.mic_source} "
            f"input_gain={self.input_gain_db}dB "
            f"noise_reduction={self.resolved_noise_reduction} "
            f"activation={self.activation_mode} "
            f"startup_voice={self.allow_during_startup}"
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
