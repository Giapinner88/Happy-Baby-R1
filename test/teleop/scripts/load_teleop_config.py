#!/usr/bin/env python3
"""Load the HB teleop YAML and emit safe shell assignments.

The launcher is shell-based because it joins several isolated Python
environments. Keeping YAML parsing here gives operators one editable config
without adding a YAML dependency to every pipeline stage.
"""
from __future__ import annotations

import argparse
import os
import math
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
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError(f"{path}: schema_version must be 1")

    values: dict[str, str] = {}
    mapping = {
        "ROBOT": ("network", "robot"),
        "HB_TELEOP_HOST_IP": ("network", "host_ip"),
        "HB_TELEOP_HOST_INTERFACE": ("network", "host_interface"),
        "HB_TELEOP_ROBOT_INTERFACE": ("network", "robot_interface"),
        "HB_TELEOP_CERT_FILE": ("network", "cert_file"),
        "HB_TELEOP_KEY_FILE": ("network", "key_file"),
        "HB_TELEOP_VUER_PORT": ("network", "vuer_port"),
        "HB_TELEOP_VUER_ENV": ("environments", "vuer"),
        "HB_TELEOP_IK_ENV": ("environments", "ik"),
        "HB_TELEOP_VENDOR_PYTHON": ("environments", "vendor_python"),
        "HB_TELEOP_DURATION_S": ("bridge", "duration_s"),
        "HB_TELEOP_BRIDGE_FREQUENCY_HZ": ("bridge", "frequency_hz"),
        "HB_TELEOP_DEADMAN_SOURCE": ("bridge", "deadman_source"),
        "HB_TELEOP_TRIGGER_VALUE_THRESHOLD": ("bridge", "trigger_value_threshold"),
        "HB_TELEOP_IK_BACKEND": ("ik", "backend"),
        "HB_TELEOP_STREAM_CONTRACT": ("ik", "stream_contract"),
        "HB_TELEOP_JOINT_LIMITS_ONLY": ("ik", "joint_limits_only"),
        "HB_TELEOP_RELEASE_DEBOUNCE_S": ("ik", "release_debounce_s"),
        "HB_TELEOP_CONTROL_HZ": ("ik", "control_hz"),
        "HB_TELEOP_SOURCE_TIMEOUT_S": ("ik", "source_timeout_s"),
        "HB_TELEOP_MAX_JOINT_VELOCITY_RAD_S": ("ik", "max_joint_velocity_rad_s"),
        "HB_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2": ("ik", "max_joint_acceleration_rad_s2"),
        "HB_TELEOP_MAX_OFFSET_RAD": ("limits", "max_offset_rad"),
        "HB_TELEOP_HEAD_YAW_MAX_RAD": ("limits", "head_yaw_max_rad"),
        "HB_TELEOP_HEAD_PITCH_MAX_RAD": ("limits", "head_pitch_max_rad"),
        "HB_TELEOP_UDP_HOST": ("sidecar", "udp_host"),
        "HB_TELEOP_UDP_PORT": ("sidecar", "udp_port"),
        "HB_TELEOP_FIRST_INPUT_TIMEOUT_S": ("sidecar", "first_input_timeout_s"),
        "HB_TELEOP_INPUT_TIMEOUT_S": ("sidecar", "input_timeout_s"),
        "HB_TELEOP_STATE_TIMEOUT_S": ("sidecar", "state_timeout_s"),
        "HB_TELEOP_SEND_HZ": ("sidecar", "send_hz"),
        "HB_TELEOP_EXPECTED_MODE_MACHINE": ("sidecar", "expected_mode_machine"),
        "CONFIRM_SUSPENDED_WITH_ESTOP": ("safety", "confirm_suspended_with_estop"),
        "HB_TELEOP_CONFIRM_DEV_MODE": ("safety", "confirm_dev_mode"),
        "HB_TELEOP_REQUIRE_LOCOMOTION": ("safety", "require_locomotion"),
        "HB_TELEOP_LOCOMOTION_WAIT_S": ("safety", "locomotion_wait_s"),
        "HB_TELEOP_RESULTS_DIR": ("paths", "results_dir"),
        "HB_TELEOP_RUNTIME_ENABLED": ("robot_runtime", "enabled"),
        "HB_TELEOP_RUNTIME_HOST_INTERFACE": ("robot_runtime", "host_interface"),
        "HB_TELEOP_RUNTIME_HOSTNAME": ("robot_runtime", "hostname"),
        "HB_TELEOP_RUNTIME_PORT": ("robot_runtime", "port"),
        "HB_TELEOP_RUNTIME_IK_BACKEND": ("robot_runtime", "ik_backend"),
        "HB_TELEOP_RUNTIME_CERT_FILE": ("robot_runtime", "cert_file"),
        "HB_TELEOP_RUNTIME_KEY_FILE": ("robot_runtime", "key_file"),
        "HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY": ("robot_runtime", "joint_limits_only"),
        "HB_TELEOP_RUNTIME_DURATION_S": ("robot_runtime", "duration_s"),
        "HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S": ("robot_runtime", "first_input_timeout_s"),
        "HB_TELEOP_RUNTIME_INITIAL_STATE_TIMEOUT_S": ("robot_runtime", "initial_state_timeout_s"),
        "HB_TELEOP_RUNTIME_CONTROL_HZ": ("robot_runtime", "control_hz"),
        "HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S": ("robot_runtime", "source_timeout_s"),
        "HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S": ("robot_runtime", "release_debounce_s"),
        "HB_TELEOP_RUNTIME_MAX_OFFSET_RAD": ("robot_runtime", "max_offset_rad"),
        "HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD": ("robot_runtime", "head_yaw_max_rad"),
        "HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD": ("robot_runtime", "head_pitch_max_rad"),
        "HB_TELEOP_RUNTIME_CPU_AFFINITY": ("robot_runtime", "cpu_affinity"),
        "HB_TELEOP_RUNTIME_CAMERA_ENABLED": ("robot_runtime", "camera_enabled"),
        "HB_TELEOP_RUNTIME_CAMERA_INTERFACE": ("robot_runtime", "camera_interface"),
        "HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT": ("robot_runtime", "camera_media_port"),
        "HB_TELEOP_RUNTIME_CAMERA_CAPTURE_HZ": ("robot_runtime", "camera_capture_hz"),
        "HB_TELEOP_RUNTIME_CAMERA_WIDTH": ("robot_runtime", "camera_width"),
        "HB_TELEOP_RUNTIME_CAMERA_HEIGHT": ("robot_runtime", "camera_height"),
        "HB_TELEOP_RUNTIME_CAMERA_STARTUP_TIMEOUT_S": ("robot_runtime", "camera_startup_timeout_s"),
        "HB_TELEOP_RUNTIME_CAMERA_ENABLE_MAX_AGE_S": ("robot_runtime", "camera_enable_max_age_s"),
        "HB_TELEOP_RUNTIME_CAMERA_FALLBACK_MAX_AGE_S": ("robot_runtime", "camera_fallback_max_age_s"),
    }
    for env_name, (section, key) in mapping.items():
        values[env_name] = _scalar(_get(raw, section, key, path=path), path=path)

    runtime = raw["robot_runtime"]
    for name in ("camera_capture_hz", "camera_startup_timeout_s", "camera_enable_max_age_s", "camera_fallback_max_age_s"):
        value = float(runtime[name])
        if not math.isfinite(value) or value < 0 or (name != "camera_capture_hz" and value == 0):
            raise ValueError(f"{path}: robot_runtime.{name} is invalid")
    if float(runtime["camera_fallback_max_age_s"]) < float(runtime["camera_enable_max_age_s"]):
        raise ValueError(f"{path}: camera fallback age must be >= enable age")
    for name in ("port", "camera_media_port", "camera_width", "camera_height"):
        if type(runtime[name]) is not int or not 1 <= runtime[name] <= 65535:
            raise ValueError(f"{path}: robot_runtime.{name} must be a positive integer <=65535")
    if runtime["port"] == runtime["camera_media_port"]:
        raise ValueError(f"{path}: camera media and WebXR ports must differ")

    # Normalize relative paths against the HB/teleop directory. Paths in the
    # YAML can still use ~ or $HOME for secrets intentionally kept local.
    for env_name in ("HB_TELEOP_CERT_FILE", "HB_TELEOP_KEY_FILE", "HB_TELEOP_VENDOR_PYTHON",
                     "HB_TELEOP_RUNTIME_CERT_FILE", "HB_TELEOP_RUNTIME_KEY_FILE"):
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
