from __future__ import annotations

import hashlib
import json
from pathlib import Path
import socket

import pytest

from scripts.teleop.run_r1_camera_transport import (
    CameraTransportConfig,
    CameraTransportExited,
    assert_local_port_free,
    build_ssh_command,
    terminate_process,
    validate_recording_directory,
    wait_until_ready,
    write_json_atomic,
)


class FakeProcess:
    def __init__(self, poll_values: list[int | None]) -> None:
        self._poll_values = iter(poll_values)
        self.returncode: int | None = None
        self.terminate_count = 0
        self.kill_count = 0
        self.wait_count = 0

    def poll(self):
        try:
            value = next(self._poll_values)
        except StopIteration:
            value = self.returncode
        if value is not None:
            self.returncode = value
        return value

    def terminate(self) -> None:
        self.terminate_count += 1
        self.returncode = -15

    def kill(self) -> None:
        self.kill_count += 1
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        self.wait_count += 1
        return 0 if self.returncode is None else self.returncode


def test_build_ssh_command_is_loopback_foreground_and_shell_quoted() -> None:
    config = CameraTransportConfig(
        robot="unitree@100.82.165.36",
        local_port=18765,
        remote_port=8765,
        remote_record_dir="/home/unitree/HB/teleop/logs/camera/run with spaces;safe",
    )

    command = build_ssh_command(config)

    assert command[:2] == ["ssh", "-o"]
    assert "BatchMode=yes" in command
    assert "ExitOnForwardFailure=yes" in command
    assert "ServerAliveInterval=5" in command
    assert "ServerAliveCountMax=3" in command
    assert "127.0.0.1:18765:127.0.0.1:8765" in command
    assert "unitree@100.82.165.36" in command
    remote = command[-1]
    assert remote.startswith("exec env PYTHONPATH=/home/unitree/HB/teleop/src python3 -m ")
    assert "teleop.hardware.r1_camera_gateway" in remote
    assert "'/home/unitree/HB/teleop/logs/camera/run with spaces;safe'" in remote


def test_assert_local_port_free_never_terminates_the_port_owner() -> None:
    owner = socket.socket()
    owner.bind(("127.0.0.1", 0))
    owner.listen(1)
    port = owner.getsockname()[1]
    try:
        with pytest.raises(OSError, match="already in use"):
            assert_local_port_free("127.0.0.1", port)
        assert owner.fileno() >= 0
    finally:
        owner.close()


def test_wait_until_ready_ignores_not_ready_then_returns_fresh_health() -> None:
    responses = iter(
        [
            {"ready": False, "latest_frame_age_ms": None},
            {"ready": True, "latest_frame_age_ms": 25.0, "latest_sequence": 4},
        ]
    )
    ticks = iter([0.0, 0.0, 0.1, 0.1])

    health = wait_until_ready(
        "http://127.0.0.1:8765/health",
        FakeProcess([None, None]),
        1.0,
        fetch_health=lambda _url, _timeout: next(responses),
        clock=lambda: next(ticks),
        sleep=lambda _delay: None,
    )

    assert health["latest_sequence"] == 4


def test_wait_until_ready_propagates_ssh_exit_before_ready() -> None:
    with pytest.raises(CameraTransportExited) as error:
        wait_until_ready(
            "http://127.0.0.1:8765/health",
            FakeProcess([7]),
            1.0,
            fetch_health=lambda _url, _timeout: {"ready": False},
        )
    assert error.value.returncode == 7


def test_wait_until_ready_rejects_health_from_another_recording_session() -> None:
    stale_health = {
        "ready": True,
        "latest_frame_age_ms": 30.0,
        "latest_sequence": 50000,
        "record_dir": "/home/unitree/HB/teleop/logs/camera/manual_camera_old",
    }
    process = FakeProcess([None, None, 1])
    with pytest.raises(CameraTransportExited) as error:
        wait_until_ready(
            "http://127.0.0.1:8765/health",
            process,
            2.0,
            expected_record_dir="/home/unitree/HB/teleop/logs/camera/new_run",
            fetch_health=lambda _url, _timeout: stale_health,
            sleep=lambda _delay: None,
        )
    assert error.value.returncode == 1


def test_wait_until_ready_times_out_when_health_never_becomes_fresh() -> None:
    ticks = iter([0.0, 0.0, 0.6, 1.1])
    with pytest.raises(TimeoutError):
        wait_until_ready(
            "http://127.0.0.1:8765/health",
            FakeProcess([None, None, None]),
            1.0,
            fetch_health=lambda _url, _timeout: {
                "ready": True,
                "latest_frame_age_ms": 700.0,
            },
            clock=lambda: next(ticks),
            sleep=lambda _delay: None,
        )


def test_terminate_process_targets_only_the_supplied_child() -> None:
    process = FakeProcess([None])

    terminate_process(process, timeout_s=0.1)

    assert process.terminate_count == 1
    assert process.wait_count == 1
    assert process.kill_count == 0


def test_write_json_atomic_replaces_ready_file_without_temp_residue(tmp_path: Path) -> None:
    path = tmp_path / "ready.json"
    write_json_atomic(path, {"ready": True, "latest_sequence": 3})

    assert json.loads(path.read_text()) == {"latest_sequence": 3, "ready": True}
    assert list(tmp_path.iterdir()) == [path]


def _recording_dir(tmp_path: Path) -> Path:
    root = tmp_path / "recording"
    frames = root / "frames"
    frames.mkdir(parents=True)
    payload = b"\xff\xd8original\xff\xd9"
    image = frames / "frame_00000001.jpg"
    image.write_bytes(payload)
    record = {
        "sequence": 1,
        "received_monotonic_s": 2.0,
        "bytes": len(payload),
        "source_width": 1280,
        "source_height": 720,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "file": "frames/frame_00000001.jpg",
    }
    (root / "manifest.jsonl").write_text(json.dumps(record) + "\n")
    summary = {
        "accepted_frame_count": 1,
        "written_frame_count": 1,
        "queue_drop_count": 0,
        "write_error_count": 0,
        "last_write_error": None,
    }
    (root / "summary.json").write_text(json.dumps(summary))
    (root / "gateway_summary.json").write_text(
        json.dumps({"exit_code": 0, "recording": summary, "gateway": {}})
    )
    return root


def test_validate_recording_directory_checks_files_hashes_and_zero_drops(tmp_path: Path) -> None:
    root = _recording_dir(tmp_path)

    result = validate_recording_directory(root)

    assert result == {"frame_count": 1, "status": "complete"}


def test_validate_recording_directory_rejects_tamper_or_drops(tmp_path: Path) -> None:
    root = _recording_dir(tmp_path)
    (root / "frames/frame_00000001.jpg").write_bytes(b"\xff\xd8tampered\xff\xd9")
    with pytest.raises(ValueError, match="hash"):
        validate_recording_directory(root)

    root = _recording_dir(tmp_path / "second")
    summary_path = root / "summary.json"
    summary = json.loads(summary_path.read_text())
    summary["queue_drop_count"] = 1
    summary_path.write_text(json.dumps(summary))
    with pytest.raises(ValueError, match="drop"):
        validate_recording_directory(root)
