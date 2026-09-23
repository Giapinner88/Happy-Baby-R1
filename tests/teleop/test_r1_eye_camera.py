from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/teleop/test_r1_eye_camera.py"

# Minimal complete JPEG with a SOF0 marker declaring 1280x720. Pixel decoding
# is outside this probe's contract; the robot supplies real JPEGs.
JPEG_1280X720 = bytes.fromhex(
    "ffd8ffc000110802d0050003011100021100031100ffd9"
)


def _fake_sdk(tmp_path: Path, responses: list[tuple[int, bytes]]) -> dict[str, str]:
    package = tmp_path / "fake_sdk"
    response_literal = repr([(code, list(payload)) for code, payload in responses])
    files = {
        "unitree_sdk2py/__init__.py": "",
        "unitree_sdk2py/core/__init__.py": "",
        "unitree_sdk2py/core/channel.py": (
            "def ChannelFactoryInitialize(domain, interface):\n"
            "    if domain < 0 or not interface:\n"
            "        raise ValueError('invalid DDS configuration')\n"
        ),
        "unitree_sdk2py/go2/__init__.py": "",
        "unitree_sdk2py/go2/video/__init__.py": "",
        "unitree_sdk2py/go2/video/video_client.py": (
            f"RESPONSES = {response_literal}\n"
            "class VideoClient:\n"
            "    def __init__(self):\n"
            "        self.index = 0\n"
            "    def SetTimeout(self, timeout):\n"
            "        self.timeout = timeout\n"
            "    def Init(self):\n"
            "        return None\n"
            "    def GetImageSample(self):\n"
            "        code, payload = RESPONSES[min(self.index, len(RESPONSES) - 1)]\n"
            "        self.index += 1\n"
            "        return code, payload\n"
        ),
    }
    for relative, source in files.items():
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding="utf-8")
    env = dict(os.environ)
    env["PYTHONPATH"] = str(package)
    return env


def test_probe_saves_valid_videohub_jpegs_and_reports_dds_metadata(tmp_path: Path) -> None:
    env = _fake_sdk(tmp_path, [(0, JPEG_1280X720), (0, JPEG_1280X720 + b"\x00")])
    output_dir = tmp_path / "capture"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(output_dir),
            "--interface",
            "eth10",
            "--domain",
            "0",
            "--timeout-s",
            "0.1",
            "--requests",
            "2",
        ],
        text=True,
        capture_output=True,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert (output_dir / "frame_0000.jpg").read_bytes() == JPEG_1280X720
    assert (output_dir / "frame_0001.jpg").read_bytes() == JPEG_1280X720 + b"\x00"
    report = json.loads((output_dir / "report.json").read_text(encoding="utf-8"))
    assert report["status"] == "completed"
    assert report["source"] == "unitree_videohub_rpc"
    assert report["dds"] == {"domain": 0, "interface": "eth10"}
    assert report["successful_frames"] == 2
    assert report["frames"][0]["dimensions"] == {"width": 1280, "height": 720}


def test_probe_records_videohub_timeout_without_writing_a_frame(tmp_path: Path) -> None:
    env = _fake_sdk(tmp_path, [(3104, b"")])
    output_dir = tmp_path / "capture"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(output_dir),
            "--requests",
            "1",
        ],
        text=True,
        capture_output=True,
        env=env,
    )

    assert result.returncode == 1
    assert not list(output_dir.glob("*.jpg"))
    report = json.loads((output_dir / "report.json").read_text(encoding="utf-8"))
    assert report["status"] == "failed"
    assert report["successful_frames"] == 0
    assert report["failures_by_code"] == {"3104": 1}


def test_probe_rejects_truncated_videohub_jpeg(tmp_path: Path) -> None:
    env = _fake_sdk(tmp_path, [(0, JPEG_1280X720[:-2])])
    output_dir = tmp_path / "capture"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(output_dir),
            "--requests",
            "1",
        ],
        text=True,
        capture_output=True,
        env=env,
    )

    assert result.returncode == 1
    report = json.loads((output_dir / "report.json").read_text(encoding="utf-8"))
    assert report["failures_by_code"] == {"truncated_or_non_jpeg": 1}


def test_probe_refuses_to_overwrite_an_existing_output_directory(tmp_path: Path) -> None:
    output_dir = tmp_path / "capture"
    output_dir.mkdir()

    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--output-dir", str(output_dir)],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "Refusing to overwrite" in result.stderr
