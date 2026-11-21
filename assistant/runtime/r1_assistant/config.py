"""Explicit robot paths and bounded task settings; no implicit motor authority."""
import json
import math
from pathlib import Path

GESTURES = ("wave", "heart", "handshake", "cool_pose", "determined", "give_gift")


def load(path):
    config = json.loads(Path(path).read_text())
    if config.get("schema") != "r1_assistant.v1":
        raise ValueError("unsupported assistant configuration")
    if config.get("mode") not in ("shadow", "execute"):
        raise ValueError("mode must be shadow or execute")
    if config.get("host") != "127.0.0.1":
        raise ValueError("assistant must bind robot loopback")
    if type(config.get("port")) is not int or not 1024 <= config["port"] <= 65535:
        raise ValueError("invalid port")
    for name in ("task_timeout_s", "poll_s", "camera_lease_s"):
        value = config.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid " + name)
    if config["task_timeout_s"] > 300 or not .02 <= config["poll_s"] <= 1:
        raise ValueError("invalid task timing")
    for name in ("runs_dir", "instance_lock", "camera_lock", "gesture_socket",
                 "hb_root", "app_root", "approach_config", "search_python",
                 "approach_python", "unitree_sdk_python_root", "unitree_python_site"):
        if not isinstance(config.get(name), str) or not Path(config[name]).is_absolute():
            raise ValueError("absolute path required: " + name)
    return config


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    durable_write(path, json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def durable_write(path, text):
    """Write via temp file + fsync + rename + directory fsync, so a power cut leaves the old or the new file,
    never a zero-length one (seen 2026-11-21 12:51: units, enabled marker and voice drop-in were empty after a reset)."""
    import os
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "w") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)
    directory = os.open(str(path.parent), os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
