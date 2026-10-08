from __future__ import annotations

import importlib.util
from pathlib import Path


TELEOP_DIR = Path(__file__).resolve().parents[1]
HB_DIR = TELEOP_DIR.parent
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
    assert positions[-2:] == [1.0, 1.1]  # head_yaw, head_pitch
    assert sidecar.MOTOR_INDICES[-2:] == (30, 29)  # SDK physical yaw, pitch
    encoded = sidecar.PACKET.unpack(sidecar.encode_target(9, positions))
    assert len(sidecar.encode_target(9, positions)) == 60
    assert encoded[:6] == (sidecar.TELEOP_MAGIC, 9, 1, 1, 1, 0)
    assert encoded[6:16] == pytest.approx(tuple(positions[:10]))
    assert encoded[16:] == pytest.approx((positions[10], positions[11]))  # UDP: yaw, pitch
    stopped = sidecar.PACKET.unpack(sidecar.encode_stop(10))
    assert stopped[:6] == (sidecar.TELEOP_MAGIC, 10, 0, 0, 0, 0)


def test_sidecar_is_not_a_dds_motor_publisher() -> None:
    source = SIDECAR_PATH.read_text(encoding="utf-8")
    assert "ChannelPublisher" not in source
    assert "HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP" in source
    assert "--confirm-suspended-with-estop" in source
    assert "--confirm-dev-mode" in source


def test_high_level_is_the_only_lowcmd_owner_and_head_mapping_matches_vendor() -> None:
    high_level = HB_DIR / "controller/src"
    assert high_level.is_dir()
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

    receiver = (high_level / "input/TeleopReceiver.hpp").read_text(encoding="utf-8")
    assert "htonl(INADDR_LOOPBACK)" in receiver
    assert "&& Valid(buf)" in receiver
    assert "static_assert(sizeof(Packet) == 60" in receiver
