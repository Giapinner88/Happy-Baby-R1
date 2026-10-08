"""Strict, small configuration surface for the independent preset player."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import yaml


class PresetConfigError(ValueError):
    """Raised before runtime when an operator-editable preset config is unsafe."""


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "presets.yaml"
ASSET_DIR = ROOT / "assets"
ALLOWED_SUFFIXES = {".wav", ".mp3", ".ogg", ".flac", ".m4a"}
VOICE_KEYS = ("UP", "RIGHT", "DOWN", "LEFT")


@dataclass(frozen=True)
class ModeConfig:
    default_enabled: bool
    toggle_hold_s: float
    enabled_cue_hz: float
    disabled_cue_hz: float


@dataclass(frozen=True)
class PresetClip:
    key: str
    name: str
    path: Path | None
    volume_percent: int


@dataclass(frozen=True)
class PresetConfig:
    mode: ModeConfig
    cancel_key: str
    clips: dict[str, PresetClip]
    retry_interval_s: float
    repeat_cancel_delay_s: float
    end_tail_s: float
    source_path: Path

    @classmethod
    def load(cls, path: str | Path | None = None) -> "PresetConfig":
        requested = path or os.getenv("HB_VOICE_PRESETS_CONFIG") or DEFAULT_CONFIG
        source = Path(requested).expanduser().resolve()
        if not source.is_file():
            raise PresetConfigError(f"preset config does not exist: {source}")
        try:
            parsed = yaml.safe_load(source.read_text(encoding="utf-8"))
        except yaml.YAMLError as error:
            raise PresetConfigError(f"invalid YAML in {source}: {error}") from error
        if not isinstance(parsed, dict):
            raise PresetConfigError("preset config root must be a mapping")

        mode_raw = _mapping(parsed, "mode")
        mode = ModeConfig(
            default_enabled=_bool(mode_raw, "default_enabled"),
            toggle_hold_s=_number(mode_raw, "toggle_hold_s", lower=1.0, upper=15.0),
            enabled_cue_hz=_number(mode_raw, "enabled_cue_hz", lower=100.0, upper=4000.0),
            disabled_cue_hz=_number(mode_raw, "disabled_cue_hz", lower=100.0, upper=4000.0),
        )

        cancel_raw = _mapping(parsed, "cancel")
        cancel_key = _text(cancel_raw, "key")
        if cancel_key != "B":
            raise PresetConfigError("cancel.key must be B; do not reuse Select/F1/F2")

        retry_interval_s = _number(
            _mapping(parsed, "playback"), "retry_interval_s", lower=0.2, upper=15.0
        )
        repeat_cancel_delay_s = _number(
            _mapping(parsed, "playback"),
            "repeat_cancel_delay_s",
            lower=0.2,
            upper=5.0,
        )
        end_tail_s = _number(
            _mapping(parsed, "playback"), "end_tail_s", lower=0.1, upper=2.0
        )
        clips_raw = _mapping(parsed, "bindings")
        unknown = set(clips_raw) - set(VOICE_KEYS)
        missing = set(VOICE_KEYS) - set(clips_raw)
        if unknown or missing:
            raise PresetConfigError(
                "bindings must contain exactly UP, RIGHT, DOWN, LEFT "
                f"(unknown={sorted(unknown)}, missing={sorted(missing)})"
            )

        clips: dict[str, PresetClip] = {}
        seen_names: set[str] = set()
        for key in VOICE_KEYS:
            item = clips_raw[key]
            if not isinstance(item, dict):
                raise PresetConfigError(f"bindings.{key} must be a mapping")
            name = _text(item, "name")
            if name in seen_names:
                raise PresetConfigError(f"duplicate preset name: {name}")
            seen_names.add(name)
            file_value = _optional_text(item, "file")
            clip_path = _asset_path(file_value) if file_value else None
            volume_percent = _integer(item, "volume_percent", lower=0, upper=100)
            clips[key] = PresetClip(
                key=key, name=name, path=clip_path, volume_percent=volume_percent
            )
        return cls(
            mode=mode,
            cancel_key=cancel_key,
            clips=clips,
            retry_interval_s=retry_interval_s,
            repeat_cancel_delay_s=repeat_cancel_delay_s,
            end_tail_s=end_tail_s,
            source_path=source,
        )


def _mapping(parent: dict, key: str) -> dict:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise PresetConfigError(f"{key} must be a mapping")
    return value


def _text(parent: dict, key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PresetConfigError(f"{key} must be a non-empty string")
    return value.strip()


def _optional_text(parent: dict, key: str) -> str:
    value = parent.get(key, "")
    if value is None:
        return ""
    if not isinstance(value, str):
        raise PresetConfigError(f"{key} must be a string or empty")
    return value.strip()


def _bool(parent: dict, key: str) -> bool:
    value = parent.get(key)
    if not isinstance(value, bool):
        raise PresetConfigError(f"{key} must be true or false")
    return value


def _number(parent: dict, key: str, *, lower: float, upper: float) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PresetConfigError(f"{key} must be a number")
    value = float(value)
    if not lower <= value <= upper:
        raise PresetConfigError(f"{key} must be in [{lower}, {upper}]")
    return value


def _integer(parent: dict, key: str, *, lower: int, upper: int) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise PresetConfigError(f"{key} must be an integer")
    if not lower <= value <= upper:
        raise PresetConfigError(f"{key} must be in [{lower}, {upper}]")
    return value


def _asset_path(value: str) -> Path:
    candidate = (ROOT / value).resolve()
    assets = ASSET_DIR.resolve()
    try:
        candidate.relative_to(assets)
    except ValueError as error:
        raise PresetConfigError("preset file must stay under voice_presets/assets") from error
    if candidate.suffix.lower() not in ALLOWED_SUFFIXES:
        choices = ", ".join(sorted(ALLOWED_SUFFIXES))
        raise PresetConfigError(f"unsupported preset file type {candidate.suffix!r}; use {choices}")
    if not candidate.is_file():
        raise PresetConfigError(f"preset file does not exist: {candidate}")
    return candidate
