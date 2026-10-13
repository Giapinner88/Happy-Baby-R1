"""Fresh stationary XR input, neck branches and actual logging child shutdown."""
from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
from teleop.r1 import QuestCommandBridge, QuestTransportSample

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts/teleop'
sys.path.insert(0, str(SCRIPTS))
from quest_tracking import valid_snapshot
from probe_robot_camera import probe
from test_head_video_regressions import ik, rot


def stationary(**kwargs):
    identity = np.eye(4).tolist()
    return QuestTransportSample(True, identity, identity, identity, True, **kwargs)


def test_stationary_held_for_two_minutes_with_fresh_observations():
    bridge = QuestCommandBridge()
    for i in range(3601):
        at = i / 30
        command = bridge.build(stationary(tracking_received_monotonic_s=at), at)
        assert command is not None and command.deadman_enabled
    assert bridge.state.disconnect_count == 0


def test_tracking_loss_emits_stop_without_killing_pipeline_and_recovers():
    bridge = QuestCommandBridge()
    assert bridge.build(stationary(tracking_received_monotonic_s=1), 1).deadman_enabled
    for at in (1.6, 2, 20, 120):
        command = bridge.build(stationary(tracking_received_monotonic_s=1), at)
        assert command is not None and not command.deadman_enabled
    assert not bridge.build(stationary(tracking_received_monotonic_s=121,
                                      tracking_valid=False, reset_requested=True), 121).deadman_enabled
    assert bridge.build(stationary(tracking_received_monotonic_s=122), 122).deadman_enabled


@pytest.mark.parametrize('yaw_deg', [-120, -110, -100, -90, -80, 80, 90, 100, 110, 120])
@pytest.mark.parametrize('pitch', [-.3, 0, .3])
def test_large_neck_yaw_preserves_pitch_and_turn_direction(yaw_deg, pitch, monkeypatch):
    yaw = np.deg2rad(yaw_deg)
    neutral = np.eye(4)
    neutral[:3, :3] = rot('z', 2.3) @ rot('y', .13)
    matrix = neutral.copy()
    matrix[:3, :3] = neutral[:3, :3] @ rot('y', pitch) @ rot('z', yaw)
    monkeypatch.setattr(ik, 'pose_to_matrix', lambda _: matrix)
    limits = ((-.62832, .62832), (-2.0071, 2.0071))
    assert ik.relative_head_angles({}, neutral, limits) == pytest.approx((pitch, np.clip(yaw, *limits[1])))


@pytest.mark.parametrize('yaw_deg', [-110, -100, -80, 80, 100, 110])
def test_head_roll_does_not_create_fake_pitch(yaw_deg):
    yaw = np.deg2rad(yaw_deg)
    rotation = rot('y', .2) @ rot('z', yaw) @ rot('x', .4)
    assert ik.neck_angles_from_rotation(rotation) == pytest.approx((.2, yaw))


def test_complete_tracking_snapshot_rejects_missing_poses_and_trigger():
    matrix = np.eye(4).flatten(order='F').tolist()
    payload = dict(valid=True, camera=dict(matrix=matrix), left=matrix, right=matrix,
                   leftState=dict(trigger=False, triggerValue=0),
                   rightState=dict(trigger=True, triggerValue=1))
    assert valid_snapshot(payload)
    for update in ({'valid': False}, {'left': None}, {'camera': None}, {'rightState': {}},
                   {'right': [float('nan')] * 16}, {'right': [0] * 16},
                   {'rightState': dict(trigger=True, triggerValue=1, thumbstickValue=[0])}):
        assert not valid_snapshot({**payload, **update})


def test_camera_retries_transient_invalid_frame(monkeypatch):
    import probe_robot_camera as module
    calls = []
    class Client:
        def GetImageSample(self):
            calls.append(1)
            return (1, b'') if len(calls) < 3 else (0, b'valid')
    monkeypatch.setattr(module, 'decode_jpeg', lambda *args: None)
    assert probe(Client())[0] and len(calls) == 3
    class Offline:
        def GetImageSample(self):
            return (100, b'')
    ready, errors = probe(Offline())
    assert not ready and errors.count('videohub code=100') == 3


def test_sigterm_cleans_real_logging_mp_child_while_stdin_idle(tmp_path):
    pytest.importorskip('logging_mp')
    script = f'''
import importlib.util, sys
spec=importlib.util.spec_from_file_location("ik_shutdown", {str(SCRIPTS / 'run_r1_vendor_ik_stream.py')!r})
ik=importlib.util.module_from_spec(spec); spec.loader.exec_module(ik)
def solver():
    import logging_mp
    logging_mp.getLogger("shutdown-test")
    print("CHILD=" + str(logging_mp._internal_manager._listener_process.pid), flush=True)
    return object()
ik.load_solver=solver
sys.argv=["ik"]
ik.main()
'''
    process = subprocess.Popen([sys.executable, '-c', script], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=tmp_path)
    try:
        assert select.select([process.stdout], [], [], 8)[0]
        child = int(process.stdout.readline().decode().strip().removeprefix('CHILD='))
        time.sleep(.1)
        started = time.monotonic()
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=4)
        assert process.returncode == 0, process.stderr.read().decode()
        assert time.monotonic() - started < 3
        assert not Path(f'/proc/{child}').exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()


@pytest.mark.parametrize('late_api', [False, True])
def test_browser_sends_stationary_xr_frames_and_stops_on_tracking_loss(late_api):
    quickjs = pytest.importorskip('quickjs')
    context = quickjs.Context()
    context.eval("""
const matrix = [1,0,0,0,0,1,0,0,0,0,1,0,0,1.5,0,1];
const messages = [], reports = [], timers=[];
let at=0, missing=false;
const performance = {now: () => at};
const location = {host:'hb-r1.local:8012',protocol:'https:',href:'https://hb-r1.local:8012/'};
const fetch = (url, request) => { reports.push(JSON.parse(request.body)); return Promise.resolve({}); };
const setTimeout = fn => timers.push(fn);
class WebSocket {
 static OPEN=1;
 constructor(url) {this.url=url;this.readyState=1;this.bufferedAmount=0;globalThis.socket=this;}
 send(data) {messages.push(JSON.parse(data));}
 close() {this.readyState=3;if(this.onclose)this.onclose({code:1000});}
}
const buttons = [{pressed:true,value:1},{pressed:false,value:0},{},{},{},{}];
const session={visibilityState:'visible',handlers:{},
 inputSources:['left','right'].map(side => ({handedness:side,targetRayMode:'tracked-pointer',
 targetRaySpace:{},gamepad:{buttons:side==='left'?[{pressed:false,value:0}]:buttons,axes:[0,0,0,0]}})),
 addEventListener(name,cb) {this.handlers[name]=cb;}};
class Frame {
 constructor() {this.session=session;}
 getViewerPose(reference) {return {transform:{matrix},views:[{},{}]};}
 getPose(space,reference) {return missing?null:{transform:{matrix}};}
}
// An immutable requestSession cannot prevent the frame observer from working.
const navigator={xr:{}};
Object.defineProperty(navigator.xr,'requestSession',{value:()=>session,writable:false});
const frame=new Frame();
""")
    if not late_api:
        context.eval('globalThis.XRFrame=Frame')
    context.eval((SCRIPTS / 'quest_tracking.js').read_text())
    if late_api:
        context.eval('globalThis.XRFrame=Frame; timers.shift()()')
    context.eval('for(let i=0;i<=3600;i++){at=i*40;frame.getViewerPose({});}')
    messages = json.loads(context.eval('JSON.stringify(messages)'))
    assert len(messages) == 3601 and all(m['valid'] for m in messages)
    assert all(m['rightState']['trigger'] for m in messages)
    assert messages[-1]['sequence'] == 3601
    assert 'installed' in json.loads(context.eval('JSON.stringify(reports.map(r=>r.stage))'))
    context.eval("session.inputSources=session.inputSources.filter(s=>s.handedness==='right');at=144040;frame.getViewerPose({})")
    partial = json.loads(context.eval('JSON.stringify(messages[messages.length-1])'))
    assert partial['valid'] and partial['arm_tracking'] == {'left': False, 'right': True}
    assert partial['left'] is None and partial['rightState']['trigger']
    context.eval('missing=true;at=144080;frame.getViewerPose({})')
    assert json.loads(context.eval('JSON.stringify(messages[messages.length-1])'))['reason'] == 'right_pose_missing_or_emulated'
    context.eval("session.visibilityState='hidden';session.handlers.visibilitychange()")
    assert not json.loads(context.eval('JSON.stringify(messages[messages.length-1])'))['valid']
    context.eval("session.visibilityState='visible';socket.bufferedAmount=17000;at=144120;frame.getViewerPose({})")
    assert context.eval('socket.readyState') == 3
    context.eval('session.handlers.end()')


def test_side_facing_neck_uses_previous_pitch_instead_of_roll():
    rotation = rot('y', .2) @ rot('z', np.pi / 2) @ rot('x', .4)
    assert ik.neck_angles_from_rotation(rotation, previous_pitch=.2) == pytest.approx((.2, np.pi / 2))


@pytest.mark.parametrize('report,exit_code,red', [
    ('[RUNTIME PREFLIGHT] DEGRADED Camera robot unavailable: videohub code=3104', 2, True),
    ('[RUNTIME PREFLIGHT] OK R1 videohub camera read on eth10', 0, False),
    ('[RUNTIME PREFLIGHT] READY', 2, True),
])
def test_deploy_camera_report_uses_current_invocation_and_red_errors(report, exit_code, red):
    import pty
    # Teleop reaches the robot through integration/scripts/deploy_stack.sh.
    script_path = SCRIPTS.parents[2] / 'integration' / 'scripts' / 'deploy_stack.sh'
    script = script_path.read_text()
    start = script.index('    # Read only this invocation')
    end = script.index('\n}\n', start)
    # Execute the actual report branch (function body tail) with a mock SSH
    # journal and terminal; `return` becomes `exit` outside the function.
    branch = script[start:end].replace('|| return 2', '|| exit 2')
    setup = '''
set -euo pipefail
RED=$'\\033[31m'; RESET=$'\\033[0m'; GREEN=$'\\033[32m'
ROBOT=test; runtime_camera_enabled=1; runtime_port=8012; camera_deploy_error=0
SSH=(ssh)
teleop_fail() { printf '%s[FAIL]%s %s\\n' "$RED" "$RESET" "$*" >&2; }
teleop_ok() { printf '%s[OK]%s %s\\n' "$GREEN" "$RESET" "$*"; }
ssh() {
 case "$*" in
  *InvocationID*) printf 'current-run\\n' ;;
  *journalctl*)
   [[ "$*" == *'_SYSTEMD_INVOCATION_ID=current-run'* ]] || exit 90
   printf '%s\\n' "$TEST_REPORT" ;;
  *) exit 91 ;;
 esac
}
'''
    master, slave = pty.openpty()
    process = subprocess.Popen(['bash', '-c', setup + branch], stdout=slave, stderr=slave,
                               env={**os.environ, 'TEST_REPORT': report})
    os.close(slave)
    output = b''
    try:
        while True:
            try:
                part = os.read(master, 4096)
                if not part:
                    break
                output += part
            except OSError:
                break
        process.wait(timeout=3)
        assert process.returncode == exit_code, output.decode()
        assert (b'\x1b[31m' in output) == red
        assert b'Quest bookmark:' in output
    finally:
        os.close(master)
        if process.poll() is None:
            process.kill()
            process.wait()
