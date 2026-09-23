from __future__ import annotations

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "scripts/teleop/run_r1_quest3_hardware.sh"


def _dry_run(**overrides: str) -> str:
    environment = os.environ.copy()
    environment.update(
        {
            "HB_TELEOP_WIRING_DRY_RUN": "1",
            "RUN_ID": "camera-wiring-test",
            "ROBOT": "unitree@100.82.165.36",
            "HOST_IP": "100.95.122.105",
        }
    )
    environment.update(overrides)
    result = subprocess.run(
        ["bash", str(LAUNCHER)],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=True,
    )
    return result.stdout


def test_hardware_launcher_enables_builtin_camera_transport_by_default() -> None:
    output = _dry_run()

    assert "camera_transport=enabled" in output
    assert "scripts/teleop/run_r1_camera_transport.py" in output
    assert "--robot-camera-preview-url http://127.0.0.1:8765/preview.jpg" in output
    assert "camera_ready_gate=before_control_pipeline" in output


def test_hardware_launcher_allows_explicit_builtin_camera_opt_out() -> None:
    output = _dry_run(HB_ROBOT_CAMERA="0")

    assert "camera_transport=disabled" in output
    assert "run_r1_camera_transport.py" not in output
    assert "--robot-camera-preview-url" not in output


def test_external_webrtc_remains_compatible_and_takes_precedence() -> None:
    output = _dry_run(ROBOT_CAMERA_WEBRTC_URL="https://robot.example/offer")

    assert "camera_transport=external_webrtc" in output
    assert "--robot-camera-webrtc-url https://robot.example/offer" in output
    assert "run_r1_camera_transport.py" not in output


def test_make_help_explains_builtin_camera_operator_contract() -> None:
    output = subprocess.run(
        ["make", "help"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    ).stdout

    assert "HB_ROBOT_CAMERA=0" in output
    assert "right A" in output
    assert "full-field" in output
    assert "stale" in output
    assert "robot_camera_original" in output
    assert "no separate camera IP/port" in output


def test_make_hardware_forwards_builtin_camera_configuration() -> None:
    output = subprocess.run(
        [
            "make",
            "-n",
            "teleop-hardware",
            "HOST_IP=100.95.122.105",
            "ROBOT=unitree@100.82.165.36",
            "HB_ROBOT_CAMERA=0",
            "ROBOT_CAMERA_LOCAL_PORT=18765",
            "ROBOT_CAMERA_REMOTE_PORT=28765",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    ).stdout

    assert 'HB_ROBOT_CAMERA="0"' in output
    assert 'ROBOT_CAMERA_LOCAL_PORT="18765"' in output
    assert 'ROBOT_CAMERA_REMOTE_PORT="28765"' in output


def test_hardware_launcher_uses_real_isaac_sim_python_for_mirror() -> None:
    output = _dry_run(
        ISAAC_SIM_PYTHON="/opt/isaac-sim/python.sh",
        ISAACLAB_ROOT="/opt/IsaacLab",
    )

    assert "sim_mirror=enabled" in output
    assert "sim_python=/opt/isaac-sim/python.sh" in output
    assert "isaaclab_root=/opt/IsaacLab" in output


def test_make_hardware_forwards_isaac_runtime_configuration() -> None:
    output = subprocess.run(
        [
            "make",
            "-n",
            "teleop-hardware",
            "HOST_IP=100.95.122.105",
            "ROBOT=unitree@100.82.165.36",
            "ISAAC_SIM_PYTHON=/opt/isaac-sim/python.sh",
            "ISAACLAB_ROOT=/opt/IsaacLab",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    ).stdout

    assert 'ISAAC_SIM_PYTHON="/opt/isaac-sim/python.sh"' in output
    assert 'ISAACLAB_ROOT="/opt/IsaacLab"' in output
