"""Kiểm tra tĩnh cho teleop chạy trên robot: fail-closed và đúng hợp đồng.

Các test này không import runtime teleop nên chạy được cả trên máy dev lẫn robot,
kể cả khi chưa cài môi trường WebXR/IK.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

TELEOP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = TELEOP_DIR.parent
RUNTIME = TELEOP_DIR / "scripts" / "run_robot_teleop.sh"
SIDECAR = TELEOP_DIR / "src" / "teleop" / "hardware" / "high_level_sidecar.py"
PREFLIGHT = TELEOP_DIR / "src" / "teleop" / "hardware" / "run_teleop.py"


def _stage_flags(module_marker: str) -> set[str]:
    """Flags run_robot_teleop.sh passes to the pipeline stage named by marker."""
    script = RUNTIME.read_text(encoding="utf-8")
    start = script.index(module_marker)
    end = script.find("\n|", start)
    stage = script[start : end if end != -1 else len(script)]
    flags = set(re.findall(r"(?<![\w-])(--[a-z][a-z0-9-]*)", stage))
    if '"${LIMIT_ARGS[@]}"' in stage:
        flags.add("--joint-limits-only")
    return flags


def _accepted_flags(path: Path) -> set[str]:
    return set(re.findall(r'add_argument\(\s*"(--[a-z][a-z0-9-]*)"', path.read_text(encoding="utf-8")))


def test_every_runtime_stage_accepts_the_flags_the_service_passes() -> None:
    """A stage that rejects a flag kills hb_teleop_runtime at start-up."""
    stages = {
        "teleop/scripts/teleop/quest_bridge.py": TELEOP_DIR / "scripts/teleop/quest_bridge.py",
        "teleop/scripts/teleop/run_r1_vendor_ik_stream.py": TELEOP_DIR / "scripts/teleop/run_r1_vendor_ik_stream.py",
        "teleop/scripts/teleop/run_r1_vendor_targets.py": TELEOP_DIR / "scripts/teleop/run_r1_vendor_targets.py",
        "-m teleop.hardware.high_level_sidecar": SIDECAR,
    }
    for marker, path in stages.items():
        missing = _stage_flags(marker) - _accepted_flags(path)
        assert not missing, f"{path.name} does not accept {sorted(missing)}"


def test_runtime_uses_the_h4_contract_required_by_the_controller() -> None:
    script = RUNTIME.read_text(encoding="utf-8")
    assert script.count("--stream-contract h4") == 2  # targets producer + sidecar
    assert '"h4"' in SIDECAR.read_text(encoding="utf-8")
    application = (REPO_DIR / "controller/src/app/Application.cpp").read_text(encoding="utf-8")
    assert "h4_teleop_approved_" in application


def test_runtime_reaches_motors_only_through_the_loopback_sidecar() -> None:
    script = RUNTIME.read_text(encoding="utf-8")
    assert "--udp-host 127.0.0.1" in script
    assert "--udp-port 5560" in script
    assert "teleop.hardware.run_teleop" not in script
    assert "HB_TELEOP_ALLOW_MOTOR_WRITE" not in script
    assert "ChannelPublisher" not in SIDECAR.read_text(encoding="utf-8")


def test_lowstate_check_is_read_only() -> None:
    source = PREFLIGHT.read_text(encoding="utf-8")
    assert "ChannelSubscriber" in source
    assert "ChannelPublisher" not in source
    for forbidden in ("--execute-pilot", "--stream-stdin", "HB_TELEOP_ALLOW_MOTOR_WRITE", "rt/arm_sdk"):
        assert forbidden not in source


def test_sidecar_has_relative_envelope_and_watchdogs() -> None:
    source = SIDECAR.read_text(encoding="utf-8")
    assert '"--max-offset-rad"' in source
    assert '"--input-timeout-s"' in source
    assert '"--state-timeout-s"' in source
    assert "source_zero" in source


def test_sidecar_scope_is_r1_a5_arms_and_head_only() -> None:
    tree = ast.parse(SIDECAR.read_text(encoding="utf-8"))
    values = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            try:
                values[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    assert values["ARM_MOTOR_INDICES"] == (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
    # UTL1 carries the head as (yaw, pitch); the R1-A5 IDL has pitch at 29, yaw at 30.
    assert (values["HEAD_YAW_IDL"], values["HEAD_PITCH_IDL"]) == (30, 29)


def test_controller_logs_why_the_udp_stream_became_inactive() -> None:
    receiver = (REPO_DIR / "controller/src/input/TeleopReceiver.hpp").read_text(encoding="utf-8")
    assert "stream ACTIVE" in receiver
    assert "stream INACTIVE" in receiver
    for reason in ("explicit_stop", "udp_watchdog", "operator_not_ready", "no_packet", "h4_deadman_not_authorized"):
        assert reason in receiver


def test_no_real_secret_is_committed() -> None:
    for path in TELEOP_DIR.rglob("*.env"):
        raise AssertionError(f"real env file must not be committed: {path}")


def _sync_teleop_function() -> str:
    """Teleop reaches the robot from the dev machine only through deploy_stack.sh."""
    script = (REPO_DIR / "integration/scripts/deploy_stack.sh").read_text(encoding="utf-8")
    start = script.index("sync_teleop() {")
    return script[start : script.index("\n}\n", start)]


def test_deploy_excludes_env_files_and_never_deletes_remote_artifacts() -> None:
    sync = _sync_teleop_function()
    assert "--exclude '*.env'" in sync
    assert "--delete" not in sync


def test_deploy_sync_does_not_start_anything() -> None:
    sync = _sync_teleop_function()
    for forbidden in ("systemctl", "install_robot_runtime", "run_robot_teleop"):
        assert forbidden not in sync, forbidden


def test_teleop_r1_has_no_dds_or_hardware_import() -> None:
    forbidden = ("unitree_sdk", "unitree_dds", "cyclonedds", "rclpy")
    offenders = []
    for path in (TELEOP_DIR / "src" / "teleop" / "r1").rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            offenders += [f"{path.name}: {n}" for n in names if n.startswith(forbidden)]
    assert not offenders, offenders
