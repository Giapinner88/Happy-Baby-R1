"""Kiểm tra tĩnh: bản deploy phải fail-closed và truy được nguồn.

Các test này cố ý không import runtime teleop: chúng chạy được trên máy dev lẫn
trên robot, kể cả khi chưa cài dependency của simulator.
"""
from __future__ import annotations

import ast
import configparser
import io
from pathlib import Path

import pytest

TELEOP_DIR = Path(__file__).resolve().parents[1]


def _unit() -> configparser.ConfigParser:
    text = (TELEOP_DIR / "systemd" / "hb_teleop.service.in").read_text(encoding="utf-8")
    parser = configparser.ConfigParser(strict=False, allow_no_value=True)
    parser.read_file(io.StringIO(text))
    return parser


def test_service_does_not_autostart_at_boot() -> None:
    """Teleop không được tự bật khi robot khởi động."""
    assert not _unit()["Install"].get("wantedby")


def test_read_only_service_does_not_conflict_with_high_level_owner() -> None:
    """Read-only preflight must not stop the sole command owner."""
    assert "hb_high_level.service" not in _unit()["Unit"].get("conflicts", "")


def test_service_does_not_restart_on_failure() -> None:
    """Tự khởi động lại một vòng teleop đang lỗi là hành vi nguy hiểm."""
    assert _unit()["Service"].get("restart", "no") == "no"


def test_service_imports_the_deployed_src_package() -> None:
    assert _unit()["Service"].get("environment", "").endswith("/teleop/src")


def test_install_script_never_enables_the_service() -> None:
    script = (TELEOP_DIR / "scripts" / "install_service.sh").read_text(encoding="utf-8")
    assert "systemctl enable" not in script


def test_high_level_sidecar_defaults_to_closed() -> None:
    example = (TELEOP_DIR / "config" / "teleop.env.example").read_text(encoding="utf-8")
    assert "HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=0" in example
    assert "HB_TELEOP_ALLOW_MOTOR_WRITE" not in example


def test_no_real_secret_is_committed() -> None:
    """File .env thật không được nằm trong package."""
    for path in TELEOP_DIR.rglob("*.env"):
        pytest.fail(f"File env thật không được commit: {path}")


def test_deploy_excludes_env_files() -> None:
    script = (TELEOP_DIR / "scripts" / "deploy_teleop.sh").read_text(encoding="utf-8")
    assert "--exclude '*.env'" in script


def test_deploy_never_deletes_remote_runtime_artifacts() -> None:
    script = (TELEOP_DIR / "scripts" / "deploy_teleop.sh").read_text(encoding="utf-8")
    command_lines = [line for line in script.splitlines() if line.lstrip().startswith("rsync ")]
    assert command_lines
    assert all("--delete" not in line for line in command_lines)


def test_deploy_does_not_start_or_enable_anything() -> None:
    """`deploy` chỉ được copy file."""
    script = (TELEOP_DIR / "scripts" / "deploy_teleop.sh").read_text(encoding="utf-8")
    for forbidden in ("systemctl start", "systemctl enable", "systemctl restart"):
        assert forbidden not in script, forbidden


def test_deploy_uses_the_repo_local_robot_finder() -> None:
    """Package sống trong Happy-Baby-R1, không phải cây HB, nên không được
    source _find_robot.sh của high_level_2 (không tồn tại ở đây)."""
    script = (TELEOP_DIR / "scripts" / "deploy_teleop.sh").read_text(encoding="utf-8")
    assert "high_level_2" not in script
    assert (TELEOP_DIR / "scripts" / "_find_robot.sh").is_file()


def test_preflight_requires_the_service_hardware_entrypoint() -> None:
    """Preflight must not call a simulation-only source sync deploy-ready."""
    script = (TELEOP_DIR / "scripts" / "preflight.sh").read_text(encoding="utf-8")
    assert "src/teleop/hardware/run_teleop.py" in script
    assert "src/teleop/hardware/high_level_sidecar.py" in script
    assert "ChannelPublisher" in script  # static rejection check


def _hardware_runtime_source() -> str:
    return (TELEOP_DIR / "src" / "teleop" / "hardware" / "run_teleop.py").read_text(
        encoding="utf-8"
    )


def test_hardware_preflight_is_read_only() -> None:
    source = _hardware_runtime_source()
    assert "ChannelSubscriber" in source
    assert "ChannelPublisher" not in source
    assert "COMMAND_TOPICS" not in source
    for forbidden in ("--execute-pilot", "--stream-stdin", "HB_TELEOP_ALLOW_MOTOR_WRITE"):
        assert forbidden not in source


def test_sidecar_has_relative_envelope_and_watchdogs() -> None:
    source = (TELEOP_DIR / "src/teleop/hardware/high_level_sidecar.py").read_text(
        encoding="utf-8"
    )
    assert "--max-offset-rad" in source
    assert "--input-timeout-s" in source
    assert "--state-timeout-s" in source
    assert 'stop_reason = "input_watchdog"' in source
    assert '"home_aborted_stream_closed"' in source
    assert '"home_aborted_input_watchdog"' in source
    assert '"last_input_age_s"' in source
    assert '"accepted_input_count"' in source
    assert "relative_session_target" in source
    assert "value - zero" in source
    assert "HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP" in source
    assert "--confirm-suspended-with-estop" in source
    assert "--confirm-dev-mode" in source
    assert "ChannelPublisher" not in source


def test_hardware_launcher_persists_each_pipeline_stage_failure() -> None:
    launcher = TELEOP_DIR.parents[1] / "scripts/teleop/run_r1_quest3_hardware.sh"
    if not launcher.is_file():
        pytest.skip("workstation launcher is not deployed in robot package")
    source = launcher.read_text(encoding="utf-8")
    for artifact in (
        "bridge.stderr.log",
        "upstream_solver.stderr.log",
        "upstream_solver_stats.json",
        "hardware_targets.stderr.log",
        "ssh.stderr.log",
        "pipeline_status.json",
    ):
        assert artifact in source
    assert 'PIPELINE_STATUS=("${PIPESTATUS[@]}")' in source


def test_high_level_owner_logs_why_the_udp_stream_became_inactive() -> None:
    receiver = TELEOP_DIR.parent / "high_level_lock/src/input/TeleopReceiver.hpp"
    if not receiver.is_file():
        pytest.skip("owner source is outside robot sidecar package")
    source = receiver.read_text(encoding="utf-8")
    assert "stream ACTIVE" in source
    assert "stream INACTIVE" in source
    for reason in ("explicit_stop", "udp_watchdog", "operator_not_ready", "no_packet"):
        assert reason in source


def test_workspace_sync_preserves_hardware_adapter() -> None:
    source = (TELEOP_DIR / "scripts" / "sync_from_workspace.sh").read_text(encoding="utf-8")
    assert "--exclude 'hardware/'" in source


def test_hardware_runtime_scope_is_r1_a5_arms_head_only() -> None:
    """The runtime may address the arms and the head, and nothing else.

    Read the resolved value rather than the literal: the indices are now built
    from named head constants, and a scope check that only understands literals
    would fail on a rename while still missing a real widening of the set.
    """

    import importlib.util

    path = TELEOP_DIR / "src/teleop/hardware/high_level_sidecar.py"
    spec = importlib.util.spec_from_file_location("high_level_sidecar_scope", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert set(module.MOTOR_INDICES) == {15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30}
    assert tuple(module.ARM_MOTOR_INDICES) == (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
    # UTL1 carries the head semantically as (yaw, pitch); the R1-A5 IDL puts
    # pitch at 29 and yaw at 30. Pinned because getting it backwards drives the
    # wrong head joint with a plausible-looking number.
    assert module.MOTOR_INDICES[-2:] == (30, 29)
    assert (module.HEAD_YAW_IDL, module.HEAD_PITCH_IDL) == (30, 29)


@pytest.mark.skipif(not (TELEOP_DIR / "src" / "teleop").is_dir(), reason="chưa sync src/teleop")
def test_synced_source_is_traceable() -> None:
    source = TELEOP_DIR / "src" / "SOURCE.txt"
    assert source.is_file(), "thiếu SOURCE.txt: bản deploy không truy được nguồn"
    assert "commit:" in source.read_text(encoding="utf-8")


@pytest.mark.skipif(not (TELEOP_DIR / "src" / "teleop").is_dir(), reason="chưa sync src/teleop")
def test_synced_teleop_has_no_dds_or_hardware_import() -> None:
    """Bản mô phỏng R1 phải giữ nguyên ranh giới: không import DDS/SDK."""
    forbidden = ("unitree_sdk", "unitree_dds", "cyclonedds", "rclpy")
    offenders = []
    for path in (TELEOP_DIR / "src" / "teleop" / "r1").rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for name in names:
                if any(name.startswith(f) for f in forbidden):
                    offenders.append(f"{path.name}: {name}")
    assert not offenders, offenders
