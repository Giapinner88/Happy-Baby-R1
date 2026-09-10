"""Execute the real sidecar loop with a clock, DDS callback and UDP recorder.

No DDS/UDP transport is opened. In particular, a synchronous Read must never
be called: losing lowstate must stop promptly rather than blocking the loop.
"""
import importlib.util
import json
import queue
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.mark.parametrize("fault,expected", [
    (None, "duration_elapsed"),
    ("state", "home_aborted_lowstate"),
    ("input", "home_aborted_input_watchdog"),
    ("eof", "home_aborted_stream_closed"),
    ("rehome_state", "lowstate_watchdog"),
])
def test_loop_homes_at_100hz_and_stops_on_actual_loss(monkeypatch, tmp_path, fault, expected):
    path = Path(__file__).parents[1] / "src/teleop/hardware/high_level_sidecar.py"
    spec = importlib.util.spec_from_file_location("sidecar_loop", path)
    sidecar = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sidecar)
    clock = SimpleNamespace(now=100.0, callback=None)
    state = SimpleNamespace(mode_machine=1, motor_state=[SimpleNamespace(q=0.0) for _ in range(35)])
    # Reproduce reported 0.714 rad initial ramp; head remains inside its gate.
    state.motor_state[18].q = 0.714
    sends = []

    def sleep(dt):
        clock.now += dt
        cutoff = 105.0 if fault == "rehome_state" else 101.0
        if clock.callback and (fault not in ("state", "rehome_state") or clock.now < cutoff):
            clock.callback(state)

    monkeypatch.setattr(sidecar, "time", SimpleNamespace(
        monotonic=lambda: clock.now, sleep=sleep, gmtime=lambda: None,
        strftime=lambda *args: "test_run",
    ))

    class Subscriber:
        def __init__(self, *args): pass
        def Init(self, handler):
            clock.callback = handler
            handler(state)
        def Read(self, *args):
            pytest.fail("blocking DDS Read entered control loop")

    channel = ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *args: None
    channel.ChannelSubscriber = Subscriber
    idl = ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    idl.LowState_ = object
    monkeypatch.setitem(sys.modules, channel.__name__, channel)
    monkeypatch.setitem(sys.modules, idl.__name__, idl)

    class Input:
        def __init__(self): self.next_at, self.sequence = 100.0, 0
        def get(self, **kwargs): return self.get_nowait()
        def get_nowait(self):
            if fault == "eof" and clock.now >= 101.0: return None
            if (fault == "input" and clock.now >= 101.0) or clock.now < self.next_at:
                raise queue.Empty
            self.next_at = clock.now + 0.1
            self.sequence += 1
            return json.dumps({
                "sequence_id": self.sequence, "joint_names": sidecar.JOINT_NAMES,
                "positions_rad": [0.0] * 12,
                "rehome": fault == "rehome_state" and 104.8 <= clock.now < 104.95,
            })

    monkeypatch.setattr(sidecar, "queue", SimpleNamespace(Queue=Input, Empty=queue.Empty))
    monkeypatch.setattr(sidecar, "threading", SimpleNamespace(
        Lock=__import__("threading").Lock,
        Thread=lambda **kwargs: SimpleNamespace(start=lambda: None),
    ))
    transport = SimpleNamespace(
        connect=lambda *args: None, close=lambda: None,
        send=lambda packet: sends.append((clock.now, sidecar.PACKET.unpack(packet))),
    )
    monkeypatch.setattr(sidecar, "socket", SimpleNamespace(
        AF_INET=0, SOCK_DGRAM=0, socket=lambda *args: transport,
    ))
    monkeypatch.setenv("HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP", "1")
    sidecar.main([
        "--confirm-suspended-with-estop", "--confirm-dev-mode", "--home-to-source",
        "--duration-s", "1", "--log-dir", str(tmp_path),
    ])
    report = json.loads(next(tmp_path.glob("*/metadata.json")).read_text())
    assert report["stop_reason"] == expected
    active = [(t, p) for t, p in sends if p[2]]
    assert active
    assert max(b[0] - a[0] for a, b in zip(active, active[1:])) < 0.011
    assert sends[-1][1][2] == 0  # explicit stop, never fake freshness
    if fault is None:
        assert report["home"]["reached"]
        assert report["home"]["elapsed_s"] == pytest.approx(4.62, abs=0.03)
    elif fault == "state":
        assert clock.now < 101.22
    elif fault == "rehome_state":
        assert clock.now < 105.22
    elif fault == "input":
        assert clock.now < 101.77


def test_repeated_snapshot_does_not_refresh_receipt_time(monkeypatch):
    path = Path(__file__).parents[1] / "src/teleop/hardware/high_level_sidecar.py"
    spec = importlib.util.spec_from_file_location("sidecar_mailbox", path)
    sidecar = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sidecar)
    mailbox = sidecar.LatestLowState()
    mailbox.receive(object())
    first = mailbox.snapshot()
    assert mailbox.snapshot() == first
