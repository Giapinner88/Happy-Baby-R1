"""Reproduce H4 revocation in the actual C++ receiver, on an isolated UDP port."""
from pathlib import Path
import json
import shutil
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from teleop.hardware import high_level_sidecar as sidecar
from teleop.hardware.stream_io import LatestControlInbox


def test_actual_cpp_head_rejection_requires_rearm_but_clamping_keeps_authority(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    source = tmp_path/'head_envelope.cpp'
    source.write_text(r'''
#include "input/TeleopReceiver.hpp"
#include <cassert>
#include <iostream>

void exercise(bool clamp) {
    int tx = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(tx, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    socklen_t size = sizeof(address);
    assert(getsockname(tx, reinterpret_cast<sockaddr*>(&address), &size) == 0);
    int port = ntohs(address.sin_port);
    close(tx);
    tx = socket(AF_INET, SOCK_DGRAM, 0);
    TeleopReceiver receiver;
    std::array<float, 10> neutral{};
    receiver.Init(neutral, true, port, 300.f, .05f, .05f, 8.f, .65f, .64f);
    TeleopReceiver::Packet packet{};
    packet.magic = TeleopReceiver::kMagic;
    packet.pad = 1;
    auto send = [&]() {
        ++packet.seq;
        assert(sendto(tx, &packet, sizeof(packet), 0,
                      reinterpret_cast<sockaddr*>(&address), sizeof(address)) == sizeof(packet));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };
    send(); // explicit STOP
    packet.enable = packet.arm_valid = packet.head_valid = 1;
    packet.arm_q[0] = 1.2f;
    packet.head_yaw = .2f;
    send();
    assert(receiver.H4Authorized());
    // Valid arms and a continuous stream, but head exceeds the receiver cap.
    packet.head_yaw = clamp ? .65f : 2.0047855f;
    for (int i=0; i<40; ++i) send();
    assert(receiver.H4Authorized() == clamp);
    packet.head_yaw = .2f;
    send();
    assert(receiver.H4Authorized() == clamp);
    if (!clamp) {
        packet.enable = packet.arm_valid = packet.head_valid = 0;
        send();
        packet.enable = packet.arm_valid = packet.head_valid = 1;
        send();
        assert(receiver.H4Authorized());
    }
    close(tx);
    std::cout << (clamp ? "CLAMPED_STREAM_STAYS_AUTHORIZED" : "HEAD_REJECTION_REPRODUCES_RELEASE_REQUIRED") << '\n';
}
int main() { exercise(false); exercise(true); }
''')
    include = Path(__file__).resolve().parents[2]/'controller/src'
    binary = tmp_path/'head_envelope'
    subprocess.run([compiler, '-std=c++17', '-pthread', '-O2', '-I', str(include),
                    str(source), '-o', str(binary)], check=True, capture_output=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=5)
    assert 'HEAD_REJECTION_REPRODUCES_RELEASE_REQUIRED' in result.stdout
    assert 'CLAMPED_STREAM_STAYS_AUTHORIZED' in result.stdout


def test_actual_cpp_receiver_retains_authority_during_blocked_disk(monkeypatch, tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    source = tmp_path/'slow_disk_receiver.cpp'
    source.write_text(r'''
#include "input/TeleopReceiver.hpp"
#include <cassert>
#include <iostream>
int main() {
    int probe=socket(AF_INET,SOCK_DGRAM,0);
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(probe,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0);
    socklen_t size=sizeof(addr); getsockname(probe,reinterpret_cast<sockaddr*>(&addr),&size);
    const int port=ntohs(addr.sin_port); close(probe);
    TeleopReceiver receiver; std::array<float,10> neutral{};
    receiver.Init(neutral,true,port,300.f,.05f,.05f,8.f,.65f,.64f);
    std::cout << port << std::endl;
    auto start=std::chrono::steady_clock::now(), first=start;
    bool activated=false; int checks=0;
    while(std::chrono::steady_clock::now()-start < std::chrono::seconds(3)) {
        const bool granted=receiver.H4Authorized();
        auto now=std::chrono::steady_clock::now();
        if(granted && !activated) { activated=true; first=now; }
        if(activated && now-first < std::chrono::milliseconds(1800)) {
            assert(granted); ++checks;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(activated && checks > 200);
    std::cout << "SLOW_DISK_STREAM_STAYS_AUTHORIZED" << std::endl;
}
''')
    include = Path(__file__).resolve().parents[2]/'controller/src'
    binary = tmp_path/'slow_disk_receiver'
    subprocess.run([compiler, '-std=c++17', '-pthread', '-O2', '-I', str(include),
                    str(source), '-o', str(binary)], check=True, capture_output=True, timeout=30)
    receiver = subprocess.Popen([str(binary)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stop_producer, release_disk, writing_disk = threading.Event(), threading.Event(), threading.Event()
    original_open = Path.open
    class BlockedDisk:
        def write(self, text):
            writing_disk.set()
            release_disk.wait(5)
        def flush(self): pass
        def close(self): pass
    def open_file(path, *args, **kwargs):
        return BlockedDisk() if path.name == 'samples.jsonl' else original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_file)
    inbox = LatestControlInbox('enabled')
    def producer():
        seq = 0
        while not stop_producer.is_set():
            seq += 1
            inbox.put(json.dumps(dict(sequence_id=seq, upstream_sequence_id=seq, enabled=True,
                                      joint_names=sidecar.JOINT_NAMES, positions_rad=[1.2]*10+[2.0,-.7],
                                      source_monotonic_s=time.monotonic())))
            stop_producer.wait(1/30)
    thread = threading.Thread(target=producer, daemon=True)
    try:
        port = int(receiver.stdout.readline())
        thread.start()
        state = SimpleNamespace(mode_machine=1, motor_state=[SimpleNamespace(q=0.) for _ in range(35)])
        states = SimpleNamespace(snapshot=lambda: (state, time.monotonic()))
        args = SimpleNamespace(joint_limits_only=True, max_offset_rad=.2, source_timeout_s=.6,
                               input_timeout_s=.75, head_yaw_max_rad=.65, head_pitch_max_rad=.64,
                               udp_host='127.0.0.1', udp_port=port, send_hz=100., duration_s=2.2,
                               first_input_timeout_s=1., state_timeout_s=.2, expected_mode_machine=1)
        assert sidecar.run_h4_stream(args, states, inbox, {}, tmp_path/'meta.json', tmp_path/'samples.jsonl') == 0
        assert writing_disk.is_set()
        output, error = receiver.communicate(timeout=4)
        assert receiver.returncode == 0, error
        assert 'SLOW_DISK_STREAM_STAYS_AUTHORIZED' in output
    finally:
        stop_producer.set()
        release_disk.set()
        if thread.ident is not None: thread.join(1)
        if receiver.poll() is None:
            receiver.terminate()
            receiver.communicate(timeout=2)


def test_position_pause_stops_and_recovers_through_actual_h4_receiver(tmp_path):
    """Real gate, sender and C++ header; synthetic joints/DDS, no motors or IK."""
    from test_position_pause_recovery import tracked, estimated
    from quest_tracking import TrackingGate
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    source = tmp_path / 'position_pause.cpp'
    source.write_text(r'''
#include "input/TeleopReceiver.hpp"
#include <cassert>
#include <iostream>
int main() {
    int probe=socket(AF_INET,SOCK_DGRAM,0);
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(probe,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0);
    socklen_t size=sizeof(addr); getsockname(probe,reinterpret_cast<sockaddr*>(&addr),&size);
    int port=ntohs(addr.sin_port); close(probe); assert(port!=5560);
    TeleopReceiver receiver; std::array<float,10> neutral{};
    receiver.Init(neutral,true,port,300.f,.05f,.05f,8.f,.65f,.64f);
    std::cout << port << std::endl;
    auto start=std::chrono::steady_clock::now();
    bool active=false, paused=false, resumed=false;
    while(std::chrono::steady_clock::now()-start < std::chrono::seconds(3)) {
        bool granted=receiver.H4Authorized();
        if(granted && !active) active=true;
        else if(active && !granted) paused=true;
        else if(paused && granted) resumed=true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(active && paused && resumed);
    std::cout << "PAUSE_STOP_AND_FRESH_RESUME_ACCEPTED" << std::endl;
}
''')
    binary = tmp_path / 'position_pause'
    include = Path(__file__).resolve().parents[2] / 'controller/src'
    subprocess.run([compiler, '-std=c++17', '-pthread', '-O2', '-I', str(include),
                    str(source), '-o', str(binary)], check=True, capture_output=True, timeout=30)
    receiver = subprocess.Popen([str(binary)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stop = threading.Event()
    inbox = LatestControlInbox('enabled')
    evidence = []

    def produce():
        gate, sequence, start = TrackingGate(), 0, time.monotonic()
        while not stop.is_set():
            sequence += 1
            now = time.monotonic()
            elapsed = now - start
            packet = (estimated(sequence) if .4 <= elapsed < .9 else
                      tracked(sequence, held=elapsed >= .1))
            normalized = gate.update(packet, now)
            enabled = normalized is not None and normalized['rightState']['trigger']
            evidence.append((elapsed, gate.reason, enabled, gate.release_required))
            inbox.put(json.dumps(dict(sequence_id=sequence, upstream_sequence_id=sequence,
                                      enabled=enabled, joint_names=sidecar.JOINT_NAMES,
                                      positions_rad=[.1]*10+[0., 0.], source_monotonic_s=now)))
            stop.wait(1 / 30)

    thread = threading.Thread(target=produce, daemon=True)
    try:
        port = int(receiver.stdout.readline())
        assert port != 5560
        thread.start()
        state = SimpleNamespace(mode_machine=1, motor_state=[SimpleNamespace(q=0.) for _ in range(35)])
        states = SimpleNamespace(snapshot=lambda: (state, time.monotonic()))
        args = SimpleNamespace(joint_limits_only=True, max_offset_rad=.2, source_timeout_s=.6,
                               input_timeout_s=.75, head_yaw_max_rad=.65, head_pitch_max_rad=.64,
                               udp_host='127.0.0.1', udp_port=port, send_hz=100., duration_s=2.2,
                               first_input_timeout_s=1., state_timeout_s=.2, expected_mode_machine=1)
        assert sidecar.run_h4_stream(args, states, inbox, {}, tmp_path/'meta.json', tmp_path/'samples.jsonl') == 0
        output, error = receiver.communicate(timeout=4)
        assert receiver.returncode == 0, error
        assert 'PAUSE_STOP_AND_FRESH_RESUME_ACCEPTED' in output
        pauses = [entry for entry in evidence if entry[1] == 'right_position_paused']
        assert pauses and all(not entry[2] and not entry[3] for entry in pauses)
        assert any(elapsed > 1.1 and enabled for elapsed, reason, enabled, release in evidence)
    finally:
        stop.set()
        if thread.ident is not None:
            thread.join(1)
        if receiver.poll() is None:
            receiver.terminate()
            receiver.communicate(timeout=2)
