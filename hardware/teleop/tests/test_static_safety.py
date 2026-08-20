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


def test_service_conflicts_with_high_level() -> None:
    """Hai bên cùng ghi một nhóm khớp thì không được chạy song song."""
    assert "hb_high_level.service" in _unit()["Unit"].get("conflicts", "")


def test_service_does_not_restart_on_failure() -> None:
    """Tự khởi động lại một vòng teleop đang lỗi là hành vi nguy hiểm."""
    assert _unit()["Service"].get("restart", "no") == "no"


def test_service_imports_the_deployed_src_package() -> None:
    assert _unit()["Service"].get("environment", "").endswith("/teleop/src")


def test_install_script_never_enables_the_service() -> None:
    script = (TELEOP_DIR / "scripts" / "install_service.sh").read_text(encoding="utf-8")
    assert "systemctl enable" not in script


def test_motor_write_defaults_to_closed() -> None:
    example = (TELEOP_DIR / "config" / "teleop.env.example").read_text(encoding="utf-8")
    assert "HB_TELEOP_ALLOW_MOTOR_WRITE=0" in example


def test_no_real_secret_is_committed() -> None:
    """File .env thật không được nằm trong package."""
    for path in TELEOP_DIR.rglob("*.env"):
        pytest.fail(f"File env thật không được commit: {path}")


def test_deploy_excludes_env_files() -> None:
    script = (TELEOP_DIR / "scripts" / "deploy_teleop.sh").read_text(encoding="utf-8")
    assert "--exclude '*.env'" in script


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
    assert "teleop.hardware.run_teleop" in script


def _hardware_runtime_source() -> str:
    return (TELEOP_DIR / "src" / "teleop" / "hardware" / "run_teleop.py").read_text(
        encoding="utf-8"
    )


def test_hardware_runtime_defaults_to_arm_sdk_and_guards_lowcmd() -> None:
    source = _hardware_runtime_source()
    assert 'COMMAND_TOPICS = {"arm_sdk": "rt/arm_sdk", "lowcmd": "rt/lowcmd"}' in source
    assert 'default="arm_sdk"' in source
    assert "--confirm-dev-mode" in source
    assert 'args.transport == "lowcmd" and not args.confirm_dev_mode' in source


def test_hardware_runtime_requires_motor_write_interlocks() -> None:
    source = _hardware_runtime_source()
    assert "--execute-pilot" in source
    assert "--stream-stdin" in source
    assert "--confirm-suspended-with-estop" in source
    assert 'HB_TELEOP_ALLOW_MOTOR_WRITE", "0"' in source


def test_stream_receiver_has_relative_envelope_and_watchdogs() -> None:
    source = _hardware_runtime_source()
    assert "--stream-max-offset-rad" in source
    assert "--stream-max-rate-rad-s" in source
    assert "--stream-input-timeout-s" in source
    assert 'stop_reason = "input_watchdog"' in source
    assert "relative = latest_source[offset] - source_zero[offset]" in source


def test_workspace_sync_preserves_hardware_adapter() -> None:
    source = (TELEOP_DIR / "scripts" / "sync_from_workspace.sh").read_text(encoding="utf-8")
    assert "--exclude 'hardware/'" in source


def test_hardware_runtime_scope_is_r1_a5_arms_head_only() -> None:
    source = _hardware_runtime_source()
    tree = ast.parse(source)
    assignments = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in {"ARM_INDICES", "HEAD_INDICES", "WAIST_HOLD_INDICES"}
    }
    assert assignments["ARM_INDICES"] == (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
    assert assignments["HEAD_INDICES"] == (29, 30)
    assert assignments["WAIST_HOLD_INDICES"] == (12, 13)


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
