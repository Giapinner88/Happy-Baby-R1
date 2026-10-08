#!/usr/bin/env python3
"""Load teleop/config/teleop.yaml and emit safe shell assignments.

install_robot_runtime.sh and deploy_stack.sh are shell scripts; parsing the YAML
here keeps one operator-editable config without a YAML dependency in shell.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import sys

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised by deployment preflight
    raise SystemExit("PyYAML is required to read teleop/config/teleop.yaml") from exc


def _scalar(value: object, *, path: Path) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (str, int, float)):
        text = os.path.expandvars(os.path.expanduser(str(value)))
        return text
    raise ValueError(f"{path}: expected scalar, got {type(value).__name__}")


def _get(data: dict, section: str, key: str, *, path: Path) -> object:
    value = data.get(section)
    if not isinstance(value, dict) or key not in value:
        raise ValueError(f"{path}: missing {section}.{key}")
    return value[key]


def load(path: Path) -> dict[str, str]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ValueError(f"{path}: schema_version must be 2")

    values: dict[str, str] = {}
    mapping = {
        "HB_TELEOP_ROBOT_INTERFACE": ("robot_runtime", "robot_interface"),
        "HB_TELEOP_RUNTIME_ENABLED": ("robot_runtime", "enabled"),
        "HB_TELEOP_RUNTIME_HOST_INTERFACE": ("robot_runtime", "host_interface"),
        "HB_TELEOP_RUNTIME_HOSTNAME": ("robot_runtime", "hostname"),
        "HB_TELEOP_RUNTIME_PORT": ("robot_runtime", "port"),
        "HB_TELEOP_RUNTIME_IK_BACKEND": ("robot_runtime", "ik_backend"),
        "HB_TELEOP_RUNTIME_CERT_FILE": ("robot_runtime", "cert_file"),
        "HB_TELEOP_RUNTIME_KEY_FILE": ("robot_runtime", "key_file"),
        "HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY": ("robot_runtime", "joint_limits_only"),
        "HB_TELEOP_RUNTIME_BRIDGE_HZ": ("robot_runtime", "bridge_hz"),
        "HB_TELEOP_RUNTIME_CONTROL_HZ": ("robot_runtime", "control_hz"),
        "HB_TELEOP_RUNTIME_SEND_HZ": ("robot_runtime", "send_hz"),
        "HB_TELEOP_RUNTIME_DURATION_S": ("robot_runtime", "duration_s"),
        "HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S": ("robot_runtime", "first_input_timeout_s"),
        "HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S": ("robot_runtime", "source_timeout_s"),
        "HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S": ("robot_runtime", "release_debounce_s"),
        "HB_TELEOP_RUNTIME_MAX_OFFSET_RAD": ("robot_runtime", "max_offset_rad"),
        "HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD": ("robot_runtime", "head_yaw_max_rad"),
        "HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD": ("robot_runtime", "head_pitch_max_rad"),
    }
    for env_name, (section, key) in mapping.items():
        values[env_name] = _scalar(_get(raw, section, key, path=path), path=path)

    # Normalize relative paths against the teleop directory.
    for env_name in ("HB_TELEOP_RUNTIME_CERT_FILE", "HB_TELEOP_RUNTIME_KEY_FILE"):
        value = Path(values[env_name])
        if not value.is_absolute():
            values[env_name] = str((path.parent.parent / value).resolve())
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--shell", action="store_true")
    args = parser.parse_args()
    try:
        values = load(args.config.expanduser().resolve())
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"[FAIL] teleop YAML: {exc}", file=sys.stderr)
        return 2
    if not args.shell:
        for key, value in values.items():
            print(f"{key}={value}")
        return 0
    for key, value in values.items():
        print(f"export {key}={shlex.quote(value)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
