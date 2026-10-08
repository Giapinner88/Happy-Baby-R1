"""Deadman-aware H4 transport; duplicate adapter keepalives cannot extend authority."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import queue
import time
from types import SimpleNamespace

import pytest


SIDECAR_PATH = Path(__file__).resolve().parents[1] / "src/teleop/hardware/high_level_sidecar.py"


def _sidecar():
    spec = importlib.util.spec_from_file_location("h4_sidecar_test", SIDECAR_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_h4_stream_requires_explicit_deadman_and_marks_wire_packet() -> None:
    sidecar = _sidecar()
    payload = {
        "sequence_id": 4,
        "upstream_sequence_id": 91,
        "enabled": True,
        "joint_names": sidecar.JOINT_NAMES,
        "positions_rad": [0.1] * 12,
    }
    assert sidecar.parse_h4_target(json.dumps({k: v for k, v in payload.items() if k != "enabled"}), 3) is None
    parsed = sidecar.parse_h4_target(json.dumps(payload), 3)
    assert parsed is not None and parsed.enabled
    assert parsed.upstream_sequence_id == 91
    encoded = sidecar.PACKET.unpack(sidecar.encode_target(4, parsed.positions_rad, explicit_deadman=True))
    assert encoded[:6] == (sidecar.TELEOP_MAGIC, 4, 1, 1, 1, 1)
    assert len(sidecar.encode_target(4, parsed.positions_rad, explicit_deadman=True)) == 60


def test_h4_release_is_stop_not_park_target() -> None:
    sidecar = _sidecar()
    released = sidecar.parse_h4_target(
        json.dumps({"sequence_id": 5, "upstream_sequence_id": 92, "enabled": False}), 4
    )
    assert released is not None and not released.enabled
    assert sidecar.PACKET.unpack(sidecar.encode_stop(5, explicit_deadman=True))[:6] == (
        sidecar.TELEOP_MAGIC, 5, 0, 0, 0, 1
    )


def test_replayed_source_sequence_does_not_refresh_deadman() -> None:
    sidecar = _sidecar()
    watchdog = sidecar.SourceSequenceWatchdog(timeout_s=0.3)
    assert watchdog.observe(91, now_s=10.0)
    assert watchdog.observe(91, now_s=10.2)
    assert not watchdog.observe(91, now_s=10.31)
    assert watchdog.observe(92, now_s=10.32)
    assert not watchdog.observe(91, now_s=10.33)


def test_h4_session_reanchors_after_release_and_blocks_replay_after_timeout() -> None:
    sidecar = _sidecar()
    session = sidecar.H4SourceSession(max_offset_rad=0.2, source_timeout_s=0.3)
    observed = [0.4] * 12
    target = lambda seq, source_seq, enabled, q: sidecar.H4Target(
        seq, source_seq, enabled, [q] * 12 if enabled else None
    )
    assert session.consume(target(1, 1, False, 0.0), observed, 10.0) is None
    assert session.consume(target(2, 2, True, 1.0), observed, 10.1) == [0.4] * 12
    assert session.consume(target(3, 3, True, 1.1), observed, 10.2) == pytest.approx([0.5] * 12)
    assert session.consume(target(4, 3, True, 1.1), observed, 10.51) is None
    assert session.consume(target(5, 4, True, 1.2), observed, 10.52) is None
    assert session.consume(target(6, 5, False, 0.0), observed, 10.53) is None
    observed = [0.6] * 12
    assert session.consume(target(7, 6, True, 1.5), observed, 10.54) == [0.6] * 12


def test_rereading_lowstate_does_not_refresh_its_watchdog(monkeypatch: pytest.MonkeyPatch) -> None:
    sidecar = _sidecar()
    monkeypatch.setattr(sidecar.time, "monotonic", lambda: 10.0)
    states = sidecar.LatestLowState()
    sample = object()
    states.receive(sample)
    assert states.snapshot() == (sample, 10.0)
    monkeypatch.setattr(sidecar.time, "monotonic", lambda: 10.4)
    assert states.snapshot() == (sample, 10.0)


def test_h4_frame_encoder_sends_stop_on_release_and_stale_source() -> None:
    sidecar = _sidecar()
    session = sidecar.H4SourceSession(max_offset_rad=0.2, source_timeout_s=0.3)
    observed = [0.0] * 12
    target = lambda seq, upstream, enabled: sidecar.H4Target(
        seq, upstream, enabled, [0.1] * 12 if enabled else None
    )
    first = sidecar.encode_h4_frame(session, target(1, 1, True), observed, 10.0)
    assert sidecar.PACKET.unpack(first)[:6] == (sidecar.TELEOP_MAGIC, 1, 1, 1, 1, 1)
    released = sidecar.encode_h4_frame(session, target(2, 2, False), observed, 10.1)
    assert sidecar.PACKET.unpack(released)[:6] == (sidecar.TELEOP_MAGIC, 2, 0, 0, 0, 1)
    sidecar.encode_h4_frame(session, target(3, 3, True), observed, 10.2)
    stale = sidecar.encode_h4_frame(session, target(4, 3, True), observed, 10.51)
    assert sidecar.PACKET.unpack(stale)[2] == 0


def test_h4_stream_pump_requires_release_after_input_timeout() -> None:
    sidecar = _sidecar()
    pump = sidecar.H4StreamPump(max_offset_rad=0.2, source_timeout_s=0.3,
                                input_timeout_s=0.4)
    observed = [0.0] * 12
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.0))[:6] == (
        sidecar.TELEOP_MAGIC, 1, 0, 0, 0, 1
    )
    enabled = lambda seq: json.dumps({
        "sequence_id": seq, "upstream_sequence_id": seq, "enabled": True,
        "joint_names": sidecar.JOINT_NAMES, "positions_rad": [0.1] * 12,
    })
    released = lambda seq: json.dumps({
        "sequence_id": seq, "upstream_sequence_id": seq, "enabled": False,
    })
    assert pump.accept(enabled(1), 10.1)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.1))[2] == 1
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.51))[2] == 0
    assert pump.accept(enabled(2), 10.52)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.52))[2] == 0
    assert pump.accept(released(3), 10.53)
    assert pump.accept(enabled(4), 10.54)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.54))[2] == 0
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.55))[2] == 1


def test_h4_pump_keeps_release_edge_across_batched_input() -> None:
    sidecar = _sidecar()
    pump = sidecar.H4StreamPump(max_offset_rad=0.2, source_timeout_s=0.3,
                                input_timeout_s=0.4)
    observed = [0.0] * 12
    def line(seq: int, enabled: bool) -> str:
        payload = {"sequence_id": seq, "upstream_sequence_id": seq, "enabled": enabled}
        if enabled:
            payload.update(joint_names=sidecar.JOINT_NAMES, positions_rad=[0.1] * 12)
        return json.dumps(payload)

    assert pump.accept(line(1, True), 10.0)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.0))[2] == 1
    assert pump.accept(line(2, False), 10.01)
    assert pump.accept(line(3, True), 10.02)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.02))[2] == 0
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.03))[2] == 1


def test_h4_sidecar_route_is_explicit_opt_in() -> None:
    sidecar = _sidecar()
    assert sidecar.build_parser().parse_args([]).stream_contract == "legacy"
    assert sidecar.build_parser().parse_args(["--stream-contract", "h4"]).stream_contract == "h4"


def test_h4_stream_loop_sends_start_and_exit_stop_without_dds_writer(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sidecar = _sidecar()
    packets: list[bytes] = []

    class FakeSocket:
        def connect(self, address: tuple[str, int]) -> None:
            assert address == ("127.0.0.1", 5560)

        def send(self, packet: bytes) -> None:
            packets.append(packet)

        def close(self) -> None:
            pass

    monkeypatch.setattr(sidecar.socket, "socket", lambda *args: FakeSocket())
    state = SimpleNamespace(mode_machine=1, motor_state=[SimpleNamespace(q=0.0) for _ in range(35)])
    states = SimpleNamespace(snapshot=lambda: (state, time.monotonic()))
    lines: queue.Queue[str | None] = queue.Queue()
    lines.put(json.dumps({
        "sequence_id": 1, "upstream_sequence_id": 1, "enabled": True,
        "joint_names": sidecar.JOINT_NAMES, "positions_rad": [0.1] * 12,
    }))
    args = SimpleNamespace(max_offset_rad=0.2, source_timeout_s=0.3,
                           input_timeout_s=0.5, head_yaw_max_rad=0.6,
                           head_pitch_max_rad=0.35, udp_host="127.0.0.1",
                           udp_port=5560, send_hz=100.0, duration_s=0.025,
                           first_input_timeout_s=1.0, state_timeout_s=0.2,
                           expected_mode_machine=1)
    assert sidecar.run_h4_stream(args, states, lines, {}, tmp_path / "meta.json",
                                 tmp_path / "samples.jsonl") == 0
    decoded = [sidecar.PACKET.unpack(packet) for packet in packets]
    assert decoded[0][2:6] == (0, 0, 0, 1)
    assert any(packet[2:6] == (1, 1, 1, 1) for packet in decoded[1:-1])
    assert decoded[-1][2:6] == (0, 0, 0, 1)
