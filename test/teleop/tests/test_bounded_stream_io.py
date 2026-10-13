"""Fault injection for local backlog, slow evidence disk and stream closure."""
import io
import importlib.util
import json
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from teleop.hardware import high_level_sidecar as sidecar
from teleop.hardware.stream_io import (
    AsyncFileLog, ControlInputError, LatestControlInbox, MAX_LINE_BYTES,
    control_batch, read_control_stream,
)


def frame(seq, enabled=True, reset=False):
    return json.dumps(dict(sequence_id=seq, enabled=enabled, reset_requested=reset))


def test_flood_keeps_constant_size_and_latest_pose_and_stop_reset():
    inbox = LatestControlInbox('enabled')
    for seq in range(10000):
        inbox.put(frame(seq, enabled=seq != 200, reset=seq == 400))
        assert inbox.qsize() <= 3
    batch = [json.loads(line) for line in inbox.take()]
    assert [p['sequence_id'] for p in batch] == [200, 400, 9999]
    assert batch[0]['enabled'] is False
    assert batch[1]['reset_requested'] is True
    assert inbox.compacted == 9997
    assert inbox.take() == []


@pytest.mark.parametrize('bad', ['{}', '[]', 'not json', '{"sequence_id": true,"enabled":true}'])
def test_bad_envelope_is_sticky_and_cannot_be_hidden_by_new_pose(bad):
    inbox = LatestControlInbox('enabled')
    inbox.put(frame(1))
    inbox.put(bad)
    inbox.put(frame(2))
    with pytest.raises(ControlInputError):
        inbox.take()
    assert inbox.qsize() == 0


def test_dropped_duplicate_sequence_still_faults():
    inbox = LatestControlInbox('enabled')
    inbox.put(frame(2))
    inbox.put(frame(2))
    inbox.put(frame(3))
    with pytest.raises(ControlInputError, match='non-increasing'):
        inbox.take()


def test_reader_eof_and_oversized_input_are_bounded():
    inbox = LatestControlInbox('enabled')
    read_control_stream(inbox, io.StringIO(frame(1)+'\n'))
    batch = inbox.take()
    assert len(batch) == 2 and batch[-1] is None
    assert batch[0].received_monotonic_s <= time.monotonic()
    read_control_stream(inbox := LatestControlInbox('enabled'), io.StringIO('x'*(MAX_LINE_BYTES+100)))
    with pytest.raises(ControlInputError, match='oversized'):
        inbox.take()


def test_atomic_batch_does_not_chase_producer_forever():
    inbox = LatestControlInbox('enabled')
    inbox.put(frame(1))
    snapshot = control_batch(inbox)
    inbox.put(frame(2))
    assert json.loads(snapshot[0])['sequence_id'] == 1
    assert json.loads(control_batch(inbox)[0])['sequence_id'] == 2
    fifo = queue.Queue()
    for _ in range(1000): fifo.put('sample')
    assert len(control_batch(fifo)) == 64


def test_slow_disk_does_not_block_log_offer_and_shutdown(monkeypatch, tmp_path):
    entered, release = threading.Event(), threading.Event()
    class SlowDisk:
        def write(self, text):
            entered.set()
            release.wait(3)
        def flush(self): pass
        def close(self): pass
    monkeypatch.setattr(Path, 'open', lambda *a, **k: SlowDisk())
    log = AsyncFileLog(tmp_path/'slow.jsonl', capacity=4)
    log.write('first\n')
    assert entered.wait(1)
    before = time.monotonic()
    for _ in range(10000): log.write('more\n')
    log.close(timeout_s=.01)
    assert time.monotonic() - before < .2
    assert log.stats()['pending'] <= 4
    assert log.stats()['dropped'] >= 9996
    release.set()
    log._worker.join(1)


def test_disk_write_error_is_reported_without_crashing_control(monkeypatch, tmp_path):
    class FullDisk:
        def write(self, text): raise OSError('disk full')
        def flush(self): pass
        def close(self): pass
    monkeypatch.setattr(Path, 'open', lambda *a, **k: FullDisk())
    log = AsyncFileLog(tmp_path/'full.jsonl')
    log.write('sample\n')
    log.close()
    assert log.stats()['error'] == 'disk full'
    log.write('new sample\n')
    assert log.stats()['dropped'] == 1


def wire(seq, enabled=True, at=None):
    value = dict(sequence_id=seq, upstream_sequence_id=seq, enabled=enabled)
    if enabled:
        value.update(joint_names=sidecar.JOINT_NAMES, positions_rad=[.1]*12)
    if at is not None: value['source_monotonic_s'] = at
    return json.dumps(value)


def test_old_queued_source_cannot_gain_new_lease_and_requires_release():
    pump = sidecar.H4StreamPump(None, .6, .75)
    observed = [.0]*12
    assert pump.accept(wire(1, at=9.0), 10.0)
    assert not sidecar.PACKET.unpack(pump.frame(observed, 10.0))[2]
    assert pump.last_stop_reason == 'source_timestamp_watchdog'
    assert pump.accept(wire(2, at=10.0), 10.01)
    assert not sidecar.PACKET.unpack(pump.frame(observed, 10.01))[2]
    assert pump.accept(wire(3, False), 10.02)
    pump.frame(observed, 10.02)
    assert pump.accept(wire(4, at=10.03), 10.03)
    assert sidecar.PACKET.unpack(pump.frame(observed, 10.03))[2]


def test_sidecar_eof_stops_immediately_with_duration_zero(monkeypatch, tmp_path):
    packets = []
    class Socket:
        def connect(self, address): pass
        def send(self, data): packets.append(sidecar.PACKET.unpack(data))
        def close(self): pass
    monkeypatch.setattr(sidecar.socket, 'socket', lambda *a: Socket())
    # No state may be consumed after an EOF; final STOP must be immediate.
    states = SimpleNamespace(snapshot=lambda: pytest.fail('read lowstate after EOF'))
    inbox = LatestControlInbox('enabled')
    inbox.put(wire(1))
    inbox.put(None)
    args = SimpleNamespace(joint_limits_only=True, max_offset_rad=.2, source_timeout_s=.6,
                           input_timeout_s=.75, head_yaw_max_rad=.65, head_pitch_max_rad=.64,
                           udp_host='127.0.0.1', udp_port=5560, send_hz=100., duration_s=0,
                           first_input_timeout_s=0, state_timeout_s=.2, expected_mode_machine=1)
    before = time.monotonic()
    assert sidecar.run_h4_stream(args, states, inbox, {}, tmp_path/'meta.json', tmp_path/'samples.jsonl') == 0
    assert time.monotonic()-before < .3
    assert all(p[2] == 0 for p in packets)
    assert json.loads((tmp_path/'meta.json').read_text())['stop_reason'] == 'stream_closed'


def test_sidecar_sends_through_slow_sample_writer(monkeypatch, tmp_path):
    entered, release = threading.Event(), threading.Event()
    original_open = Path.open
    class SlowDisk:
        def write(self, text):
            entered.set()
            release.wait(3)
        def flush(self): pass
        def close(self): pass
    def open_file(path, *args, **kwargs):
        return SlowDisk() if path.name == 'samples.jsonl' else original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_file)
    times, packets = [], []
    class Socket:
        def connect(self, address): pass
        def send(self, data):
            times.append(time.monotonic())
            packets.append(sidecar.PACKET.unpack(data))
        def close(self): pass
    monkeypatch.setattr(sidecar.socket, 'socket', lambda *a: Socket())
    state = SimpleNamespace(mode_machine=1, motor_state=[SimpleNamespace(q=0.) for _ in range(35)])
    states = SimpleNamespace(snapshot=lambda: (state, time.monotonic()))
    inbox = LatestControlInbox('enabled')
    inbox.put(wire(1, at=time.monotonic()))
    args = SimpleNamespace(joint_limits_only=True, max_offset_rad=.2, source_timeout_s=.6,
                           input_timeout_s=.75, head_yaw_max_rad=.65, head_pitch_max_rad=.64,
                           udp_host='127.0.0.1', udp_port=5560, send_hz=100., duration_s=.12,
                           first_input_timeout_s=0, state_timeout_s=.2, expected_mode_machine=1)
    try:
        assert sidecar.run_h4_stream(args, states, inbox, {}, tmp_path/'meta.json', tmp_path/'samples.jsonl') == 0
        assert entered.is_set()
        assert len([p for p in packets if p[2]]) >= 8
        # Exclude final STOP/metadata shutdown; steady send loop remains fast.
        assert max(b-a for a,b in zip(times[1:-2],times[2:-1])) < .1
        assert packets[-1][2] == 0
    finally:
        release.set()


def test_ik_timing_memory_is_bounded_but_lifetime_counters_are_exact():
    path = Path(__file__).resolve().parents[1]/'scripts/teleop/run_r1_vendor_ik_stream.py'
    spec = importlib.util.spec_from_file_location('bounded_ik_stats', path)
    ik = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ik)
    stats = ik.SolveTimingStats()
    for value in range(10000): stats.append(float(value))
    assert len(stats.samples) == 4096
    assert stats.count == 10000
    assert stats.total_ms == sum(range(10000))
    assert stats.max_ms == 9999
    assert stats.samples[0] == 10000-4096


def test_ik_invalid_input_does_not_accumulate_while_solver_is_busy():
    path = Path(__file__).resolve().parents[1]/'scripts/teleop/run_r1_vendor_ik_stream.py'
    spec = importlib.util.spec_from_file_location('bounded_ik_faults', path)
    ik = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ik)
    compacted = []
    for i in range(10000):
        compacted = ik.compact_live_commands(compacted+['not-json-'+str(i)])
        assert len(compacted) <= 2
    assert compacted[0] == 'not-json-0'
    assert compacted[-1] == 'not-json-9999'
