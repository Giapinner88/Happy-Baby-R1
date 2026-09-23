#!/usr/bin/env python3
"""Own one SSH-forwarded R1 camera gateway process and its readiness gate."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any
from urllib.request import urlopen


@dataclass(frozen=True)
class CameraTransportConfig:
    robot: str
    local_port: int = 8765
    remote_port: int = 8765
    remote_record_dir: str = "/home/unitree/HB/teleop/logs/camera/manual"
    remote_pythonpath: str = "/home/unitree/HB/teleop/src"
    interface: str = "eth10"
    capture_hz: float = 15.0
    preview_width: int = 640
    preview_height: int = 360
    preview_quality: int = 60

    def __post_init__(self) -> None:
        if not self.robot or self.robot.startswith("-") or any(c.isspace() for c in self.robot):
            raise ValueError("robot SSH target must be non-empty and contain no whitespace")
        for port in (self.local_port, self.remote_port):
            if not 1 <= int(port) <= 65535:
                raise ValueError("camera ports must be in [1, 65535]")
        record_dir = PurePosixPath(self.remote_record_dir)
        if not record_dir.is_absolute() or ".." in record_dir.parts:
            raise ValueError("remote recording directory must be an absolute normalized path")
        pythonpath = PurePosixPath(self.remote_pythonpath)
        if not pythonpath.is_absolute() or ".." in pythonpath.parts:
            raise ValueError("remote PYTHONPATH must be an absolute normalized path")
        if not self.interface or any(c.isspace() for c in self.interface):
            raise ValueError("camera interface must be non-empty and contain no whitespace")
        if not math.isfinite(self.capture_hz) or not 0 < self.capture_hz <= 15:
            raise ValueError("capture_hz must be finite and in (0, 15]")


class CameraTransportExited(RuntimeError):
    def __init__(self, returncode: int) -> None:
        super().__init__(f"camera SSH process exited before ready with code {returncode}")
        self.returncode = int(returncode)


def assert_local_port_free(host: str, port: int) -> None:
    """Fail without touching the process that already owns the requested port."""

    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind((host, int(port)))
    except OSError as exc:
        raise OSError(f"local camera port {host}:{port} is already in use") from exc
    finally:
        probe.close()


def build_ssh_command(config: CameraTransportConfig) -> list[str]:
    remote_tokens = [
        "exec",
        "env",
        f"PYTHONPATH={config.remote_pythonpath}",
        "python3",
        "-m",
        "teleop.hardware.r1_camera_gateway",
        "--interface",
        config.interface,
        "--record-dir",
        config.remote_record_dir,
        "--host",
        "127.0.0.1",
        "--port",
        str(config.remote_port),
        "--capture-hz",
        str(config.capture_hz),
        "--preview-width",
        str(config.preview_width),
        "--preview-height",
        str(config.preview_height),
        "--preview-quality",
        str(config.preview_quality),
    ]
    remote_command = " ".join(shlex.quote(token) for token in remote_tokens)
    return [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ExitOnForwardFailure=yes",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=3",
        "-L",
        f"127.0.0.1:{config.local_port}:127.0.0.1:{config.remote_port}",
        "--",
        config.robot,
        remote_command,
    ]


def _fetch_health(url: str, timeout_s: float) -> dict[str, object]:
    with urlopen(url, timeout=timeout_s) as response:
        if int(response.status) != 200:
            raise ConnectionError(f"camera health returned HTTP {response.status}")
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError("camera health response must be a JSON object")
    return payload


def wait_until_ready(
    url: str,
    process: Any,
    timeout_s: float,
    *,
    fetch_health: Callable[[str, float], dict[str, object]] = _fetch_health,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    request_timeout_s: float = 0.4,
    max_frame_age_ms: float = 500.0,
    expected_record_dir: str | None = None,
) -> dict[str, object]:
    deadline = clock() + timeout_s
    last_error: str | None = None
    while True:
        returncode = process.poll()
        if returncode is not None:
            raise CameraTransportExited(int(returncode))
        if clock() >= deadline:
            suffix = f": {last_error}" if last_error else ""
            raise TimeoutError(f"camera transport was not ready within {timeout_s:.1f}s{suffix}")
        try:
            health = fetch_health(url, request_timeout_s)
            age = health.get("latest_frame_age_ms")
            age_ms = float(age) if age is not None else math.inf
            if (
                health.get("ready") is True
                and math.isfinite(age_ms)
                and 0.0 <= age_ms <= max_frame_age_ms
                and (expected_record_dir is None or health.get("record_dir") == expected_record_dir)
            ):
                returncode = process.poll()
                if returncode is not None:
                    raise CameraTransportExited(int(returncode))
                return health
            last_error = (
                f"health not ready for this run: ready={health.get('ready')} "
                f"age_ms={age} record_dir={health.get('record_dir')}"
            )
        except CameraTransportExited:
            raise
        except Exception as exc:  # noqa: BLE001 - retry until bounded deadline
            last_error = f"{type(exc).__name__}: {exc}"
        sleep(0.1)


def terminate_process(process: Any, *, timeout_s: float = 5.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=timeout_s)


def write_json_atomic(path: Path, payload: Mapping[str, object]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    )
    temporary = Path(handle.name)
    try:
        with handle:
            json.dump(dict(payload), handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def validate_recording_directory(root: Path) -> dict[str, object]:
    root = Path(root).resolve()
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    gateway_summary = json.loads(
        (root / "gateway_summary.json").read_text(encoding="utf-8")
    )
    if int(gateway_summary.get("exit_code", 1)) != 0:
        raise ValueError("camera gateway exit status is not complete")
    if summary.get("queue_drop_count") != 0:
        raise ValueError("camera recording contains queue drops")
    if summary.get("write_error_count") != 0:
        raise ValueError("camera recording contains write errors")
    if summary.get("accepted_frame_count") != summary.get("written_frame_count"):
        raise ValueError("camera recording accepted/written counts differ")

    records = [
        json.loads(line)
        for line in (root / "manifest.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not records:
        raise ValueError("camera recording contains no original frames")
    if len(records) != int(summary.get("written_frame_count", -1)):
        raise ValueError("camera manifest count does not match summary")
    seen_sequences: set[int] = set()
    for record in records:
        relative = Path(str(record["file"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("camera manifest contains an unsafe file path")
        image_path = (root / relative).resolve()
        if root not in image_path.parents:
            raise ValueError("camera manifest file escapes recording directory")
        payload = image_path.read_bytes()
        if len(payload) != int(record["bytes"]):
            raise ValueError("camera frame byte count does not match manifest")
        if hashlib.sha256(payload).hexdigest() != record["sha256"]:
            raise ValueError("camera frame hash does not match manifest")
        sequence = int(record["sequence"])
        if sequence in seen_sequences:
            raise ValueError("camera manifest contains duplicate sequences")
        seen_sequences.add(sequence)
    return {"status": "complete", "frame_count": len(records)}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--local-port", type=int, default=8765)
    parser.add_argument("--remote-port", type=int, default=8765)
    parser.add_argument("--remote-record-dir", required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--status-file", type=Path, required=True)
    parser.add_argument("--ready-timeout-s", type=float, default=20.0)
    parser.add_argument("--interface", default="eth10")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.ready_timeout_s) or args.ready_timeout_s <= 0:
        raise SystemExit("--ready-timeout-s must be finite and positive")
    if args.ready_file.exists() or args.status_file.exists():
        raise SystemExit("ready/status output already exists")
    config = CameraTransportConfig(
        robot=args.robot,
        local_port=args.local_port,
        remote_port=args.remote_port,
        remote_record_dir=args.remote_record_dir,
        interface=args.interface,
    )
    assert_local_port_free("127.0.0.1", config.local_port)
    command = build_ssh_command(config)
    process = subprocess.Popen(command)
    stop = threading.Event()
    previous: dict[int, Any] = {}

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, request_stop)

    result = 0
    stop_reason = "ssh_exited"
    ready_health: dict[str, object] | None = None
    try:
        health_url = f"http://127.0.0.1:{config.local_port}/health"
        ready_health = wait_until_ready(
            health_url, process, args.ready_timeout_s,
            expected_record_dir=config.remote_record_dir,
        )
        write_json_atomic(
            args.ready_file,
            {
                "ready": True,
                "local_preview_url": f"http://127.0.0.1:{config.local_port}/preview.jpg",
                "health": ready_health,
                "ssh_pid": process.pid,
            },
        )
        while not stop.is_set():
            returncode = process.poll()
            if returncode is not None:
                result = int(returncode)
                break
            stop.wait(0.1)
        if stop.is_set():
            stop_reason = "signal_requested"
            terminate_process(process)
            result = 0
    except CameraTransportExited as exc:
        result = exc.returncode if exc.returncode != 0 else 1
        stop_reason = "ssh_exited_before_ready"
    except Exception as exc:  # noqa: BLE001 - write status for launcher evidence
        stop_reason = f"startup_failed:{type(exc).__name__}:{exc}"
        terminate_process(process)
        result = 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        write_json_atomic(
            args.status_file,
            {
                "returncode": result,
                "stop_reason": stop_reason,
                "ready_health": ready_health,
            },
        )
    return result


if __name__ == "__main__":
    raise SystemExit(main())
