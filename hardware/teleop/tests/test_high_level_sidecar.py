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
        "schema_version": 1,
        "sequence_id": 4,
        "joint_names": sidecar.JOINT_NAMES,
        "positions_rad": [index / 10 for index in range(12)],
    }
    import json

    sequence, positions, head_valid, target_mode, rehome = sidecar.parse_target(json.dumps(payload), 3)
    assert sequence == 4
    assert head_valid is True
    assert target_mode == "relative_source"
    assert positions[-2:] == [1.0, 1.1]  # head_yaw, head_pitch -- the wire order
    encoded = sidecar.PACKET.unpack(sidecar.encode_target(9, positions, head_valid))
    assert len(sidecar.encode_target(9, positions, head_valid)) == 60
    assert encoded[:6] == (sidecar.TELEOP_MAGIC, 9, 1, 1, 1, 0)
    assert encoded[6:16] == pytest.approx(tuple(positions[:10]))
    # The stream already arrives as (yaw, pitch), so the packet carries it through.
    assert encoded[16:] == pytest.approx((positions[10], positions[11]))
    stopped = sidecar.PACKET.unpack(sidecar.encode_stop(10))
    assert stopped[:6] == (sidecar.TELEOP_MAGIC, 10, 0, 0, 0, 0)


def test_arm_only_stream_clears_the_head_flag() -> None:
    """Ten joints means the head is not being driven, and the packet says so.

    The head floats limp in that case, so the flag has to reach the owner: a
    stale head target applied while nobody is commanding the head is exactly
    what the flag exists to prevent.
    """

    import json

    sidecar = _sidecar()
    payload = {
        "schema_version": 1,
        "sequence_id": 4,
        "joint_names": sidecar.ARM_JOINT_NAMES,
        "positions_rad": [0.0] * 10,
    }
    sequence, positions, head_valid, _, _ = sidecar.parse_target(json.dumps(payload), 3)
    assert (sequence, head_valid, len(positions)) == (4, False, 10)
    encoded = sidecar.PACKET.unpack(sidecar.encode_target(9, positions, head_valid))
    assert encoded[4] == 0
    assert encoded[16:] == (0.0, 0.0)


def test_sidecar_rejects_an_unknown_schema_or_target_mode() -> None:
    """Version 2 is refused; a missing version is now read as version 1.

    That default is a real loosening against the version this repository shipped,
    which refused a line with no `schema_version` at all. It came in with the
    robot-side sidecar and is pinned here so it stays a decision rather than a
    surprise -- `docs/hardware_gate.md` carries it as an open item.
    """

    import json

    sidecar = _sidecar()
    payload = {
        "sequence_id": 4,
        "joint_names": sidecar.JOINT_NAMES,
        "positions_rad": [0.0] * 12,
    }
    assert sidecar.parse_target(json.dumps(payload), 3) is not None
    payload["schema_version"] = 2
    assert sidecar.parse_target(json.dumps(payload), 3) is None
    payload["schema_version"] = 1
    payload["target_mode"] = "whatever"
    assert sidecar.parse_target(json.dumps(payload), 3) is None


def test_absolute_targets_stay_inside_the_session_envelope() -> None:
    """`absolute_robot` mode is bounded by the same envelope as relative mode.

    The tolerance exists because LeRobot and this process sample the encoder
    anchor a few ticks apart and would otherwise stop each other over rounding.
    It must widen the accept/reject boundary only -- the packet that reaches the
    owner still has to be clamped to the exact envelope.
    """

    import pytest

    sidecar = _sidecar()
    start = [0.0] * 12
    bounded, worst_index, worst_offset, clamped = sidecar.constrain_absolute_target(
        [0.10] * 11 + [0.1501], start, 0.15
    )
    assert worst_index == 11
    assert worst_offset == pytest.approx(0.1501)
    assert clamped is True
    assert max(abs(value) for value in bounded) <= 0.15 + 1e-12

    with pytest.raises(ValueError):
        sidecar.constrain_absolute_target([0.0] * 11 + [0.2], start, 0.15)


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
    # Điều kiện là ĐÚNG MỘT chủ rt/lowcmd đang giữ 5560, không phải "service
    # active": bản cô lập high_level_lock chạy foreground và cố ý dừng service.
    # Kiểm theo service sẽ chặn đúng một cấu hình hợp lệ, và bỏ lọt hai chủ cùng
    # chạy -- đúng thứ D003 cấm.
    assert "pgrep -x run_r1" in source
    assert "/proc/$p/exe" in source
    assert '"$n" -gt 1' in source
    assert 'hb_teleop.service' in source
    assert 'ss -H -lun "sport = :5560"' in source
    assert "127.0.0.1:5560" in source
    assert "teleop.hardware.high_level_sidecar" in source
    assert "HB_TELEOP_ALLOW_MOTOR_WRITE" not in source
    assert "teleop.hardware.run_teleop" not in source


def test_hardware_target_adapter_refuses_projected_ik() -> None:
    adapter = ROOT / "scripts/teleop/run_r1_quest3_hardware_targets.py"
    if not adapter.is_file():
        import pytest
        pytest.skip("workstation target adapter is not in the robot-only package")
    source = adapter.read_text(encoding="utf-8")
    assert "allow_projected_position_solution=False" in source


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


def test_homing_ramp_is_rate_limited_and_converges() -> None:
    """Bước homing bị chặn tốc độ và tới đúng tư thế đã nêu tên.

    Chuyển động lớn nhất trên robot thật là khớp elbow, 1.36 rad. Ở đây kiểm
    đúng khoảng đó: từng bước không được vượt trần, và chuỗi bước phải hội tụ
    chứ không dao động quanh đích.
    """

    sidecar = _sidecar()
    goal = [0.0] * 12
    current = [0.0] * 3 + [1.36] + [0.0] * 8
    step = 0.15 / 100.0            # home_rate_rad_s / send_hz
    previous = list(current)
    for _ in range(2000):
        current = sidecar.ramp_toward(current, goal, step)
        assert max(abs(a - b) for a, b in zip(current, previous)) <= step + 1e-12
        previous = list(current)
        if max(abs(g - c) for g, c in zip(goal, current)) <= 0.02:
            break
    else:
        raise AssertionError("homing ramp did not converge")
    # 1.36 rad ở 0.15 rad/s là khoảng 9 giây; số bước phải nằm quanh đó.
    assert max(abs(g - c) for g, c in zip(goal, current)) <= 0.02


def test_homing_ramp_never_overshoots_the_named_pose() -> None:
    sidecar = _sidecar()
    goal = [0.5] * 12
    current = [0.0] * 12
    for _ in range(1000):
        current = sidecar.ramp_toward(current, goal, 0.1)
        assert all(c <= g + 1e-12 for c, g in zip(current, goal))
    assert all(abs(c - g) < 1e-9 for c, g in zip(current, goal))


def test_homing_arguments_are_bounded() -> None:
    """Tốc độ homing phải chậm hơn teleop và tư thế home không nhận số lạ."""

    import pytest

    sidecar = _sidecar()
    base = [
        "--confirm-suspended-with-estop", "--confirm-dev-mode",
        "--home-to-nominal",
    ]
    import os

    os.environ["HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP"] = "1"
    parser = sidecar.build_parser()

    ok = parser.parse_args(base)
    sidecar.validate_args(ok)
    assert ok.home_pose_rad == [0.0] * 12          # nominal arms_head của sim
    assert ok.home_rate_rad_s <= 0.30              # không nhanh hơn trần teleop

    for extra in (
        ["--home-rate-rad-s", "0.9"],
        ["--home-pose-rad", *(["0.0"] * 11), "2.0"],
        ["--home-timeout-s", "1"],
    ):
        with pytest.raises(SystemExit):
            sidecar.validate_args(parser.parse_args(base + extra))


def test_head_gate_thresholds() -> None:
    sidecar = _sidecar()
    assert sidecar.head_outside_gate(2.007, 0.0, 0.60, 0.35) is True     # đầu ở chặn cơ khí
    assert sidecar.head_outside_gate(0.0, 0.50, 0.60, 0.35) is True
    assert sidecar.head_outside_gate(0.59, 0.34, 0.60, 0.35) is False
    assert sidecar.head_outside_gate(0.0, 0.0, 0.60, 0.35) is False


def test_head_gate_moves_behind_homing_when_homing_is_on() -> None:
    """Bật homing thì đầu lệch không được chặn phiên TRƯỚC khi kịp sửa.

    Gate có mặt vì phiên là tương đối, nên đầu lệch lúc chốt sẽ lệch cả phiên.
    Homing đưa đầu về nominal trước khi chốt nên tiền đề đó biến mất; giữ gate ở
    trước tức là chặn đúng cái cơ chế sửa được vấn đề, và với đầu đang tì vào
    chặn cơ khí thì đi về giữa còn là hướng rời khỏi giới hạn.
    """

    source = SIDECAR_PATH.read_text(encoding="utf-8")
    pre, post = source.split("--- Pha homing ---", 1)
    assert "not args.home_to_nominal and head_outside_gate" in pre
    assert "head_not_neutral_after_home" in post
    # Sau homing phải kiểm ENCODER, không phải giá trị vừa ra lệnh: ramp là vòng
    # hở và owner còn kẹp lệnh đầu ở teleop_head_yaw_max trước khi slew.
    assert "latest_state.motor_state[i].q" in post


def test_rehome_flag_rides_on_the_command_stream() -> None:
    """Cò trái đi kèm chính command, không qua kênh phụ.

    Nhờ vậy yêu cầu về-nominal tới receiver đúng thứ tự với dòng lệnh nó đi
    cùng, và một dòng cũ không thể kích hoạt lại homing sau khi đã xử lý.
    """

    import json

    sidecar = _sidecar()
    base = {
        "schema_version": 1,
        "sequence_id": 4,
        "joint_names": sidecar.JOINT_NAMES,
        "positions_rad": [0.0] * 12,
    }
    *_, rehome = sidecar.parse_target(json.dumps(base), 3)
    assert rehome is False

    asked = dict(base, rehome=True)
    *_, rehome = sidecar.parse_target(json.dumps(asked), 3)
    assert rehome is True


def test_session_envelope_accepts_the_widened_range() -> None:
    """±1.0 rad được nhận, quá 1.0 thì không.

    Phiên 2026-08-29 bão hoà 10/12 khớp ở 0.15 rad, nên trần cũ 0.30 là thứ giữ
    robot trong một hộp 8.6 độ. 1.0 rad vẫn là bao an toàn thật.
    """

    import os

    import pytest

    os.environ["HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP"] = "1"
    sidecar = _sidecar()
    parser = sidecar.build_parser()
    base = ["--confirm-suspended-with-estop", "--confirm-dev-mode"]

    for value in ("0.15", "0.5", "1.0"):
        sidecar.validate_args(parser.parse_args(base + ["--max-offset-rad", value]))

    for value in ("1.01", "3.2", "0.01"):
        with pytest.raises(SystemExit):
            sidecar.validate_args(parser.parse_args(base + ["--max-offset-rad", value]))

