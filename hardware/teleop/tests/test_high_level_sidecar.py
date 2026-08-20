from __future__ import annotations

import importlib.util
from pathlib import Path


TELEOP_DIR = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[3]
SIDECAR_PATH = TELEOP_DIR / "src/teleop/hardware/high_level_sidecar.py"


def _sidecar():
    spec = importlib.util.spec_from_file_location("high_level_sidecar_test", SIDECAR_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sidecar_protocol_and_joint_order() -> None:
    import pytest

    sidecar = _sidecar()
    payload = {
        "sequence_id": 4,
        "joint_names": sidecar.JOINT_NAMES,
        "positions_rad": [index / 10 for index in range(12)],
    }
    import json

    sequence, positions = sidecar.parse_target(json.dumps(payload), 3)
    assert sequence == 4
    assert positions[-2:] == [1.0, 1.1]  # head_pitch, head_yaw
    encoded = sidecar.PACKET.unpack(sidecar.encode_target(9, positions))
    assert len(sidecar.encode_target(9, positions)) == 60
    assert encoded[:6] == (sidecar.TELEOP_MAGIC, 9, 1, 1, 1, 0)
    assert encoded[6:16] == pytest.approx(tuple(positions[:10]))
    assert encoded[16:] == pytest.approx((positions[11], positions[10]))  # UDP: yaw, pitch
    stopped = sidecar.PACKET.unpack(sidecar.encode_stop(10))
    assert stopped[:6] == (sidecar.TELEOP_MAGIC, 10, 0, 0, 0, 0)


def test_sidecar_is_not_a_dds_motor_publisher() -> None:
    source = SIDECAR_PATH.read_text(encoding="utf-8")
    assert "ChannelPublisher" not in source
    assert "HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP" in source
    assert "--confirm-suspended-with-estop" in source
    assert "--confirm-dev-mode" in source


def test_hardware_launcher_requires_active_high_level_owner() -> None:
    launcher = ROOT / "scripts/teleop/run_r1_quest3_hardware.sh"
    if not launcher.is_file():
        import pytest
        pytest.skip("workspace launcher is not part of the deployed robot package")
    source = launcher.read_text(encoding="utf-8")
    assert 'systemctl is-active hb_high_level.service' in source
    assert '= active' in source
    assert 'ss -H -lun "sport = :5560"' in source
    assert "127.0.0.1:5560" in source
    assert "teleop.hardware.high_level_sidecar" in source
    assert "HB_TELEOP_ALLOW_MOTOR_WRITE" not in source
    assert "teleop.hardware.run_teleop" not in source


def test_high_level_is_the_only_lowcmd_owner_and_head_mapping_matches_vendor() -> None:
    high_level = ROOT / "hardware/high_level/src"
    if not high_level.is_dir():
        import pytest
        pytest.skip("high-level source is owned by a separate deployed directory")
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in high_level.rglob("*")
        if path.suffix in {".cpp", ".hpp"}
    )
    publisher_files = [
        path.relative_to(high_level).as_posix()
        for path in high_level.rglob("*")
        if path.suffix in {".cpp", ".hpp"}
        and "ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>" in path.read_text(encoding="utf-8")
    ]
    assert publisher_files == ["robot/LowCmdSender.hpp"]
    spec = (high_level / "config/RobotSpec.hpp").read_text(encoding="utf-8")
    assert "kHeadPitchIdl = 29" in spec
    assert "kHeadYawIdl   = 30" in spec
