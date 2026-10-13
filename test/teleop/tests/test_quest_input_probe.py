"""An isolated native WebXR measurement page never opens robot control."""
import json
from pathlib import Path
import pytest

PAGE = Path(__file__).resolve().parents[1] / 'scripts/teleop/quest_input_probe.html'


def probe_context(supported=True):
    context = pytest.importorskip('quickjs').Context()
    context.eval('''
const reports=[],colors=[],calls=[],canvases=[],start={disabled:true},status={},side={value:'right'};
let at=0,requested=null,sessionFailure=null,glFailure=null,resolveSession=null,deferSession=false;
globalThis.isSecureContext=true;
let supported=true;
const performance={now:()=>at};
const fetch=(url,request)=>{reports.push({url,body:JSON.parse(request.body)});return Promise.resolve({});};
const gl={makeXRCompatible(){calls.push('compatible');return glFailure?Promise.reject(glFailure):Promise.resolve();},
 isContextLost:()=>false,bindFramebuffer(){},enable(){},viewport(){},scissor(){},
 clearColor(...color){colors.push(color);},clear(){}};
const document={getElementById:name=>name==='start'?start:name==='status'?status:side,
 body:{appendChild(){}},createElement(){calls.push('canvas');const canvas={removed:false,
 getContext(){calls.push('context');return gl;},remove(){this.removed=true;}};canvases.push(canvas);return canvas;}};
const trigger={value:1,pressed:true,touched:true};
const right={handedness:'right',targetRayMode:'tracked-pointer',targetRaySpace:{},profiles:['meta-quest-touch-plus'],
 gamepad:{connected:true,mapping:'xr-standard',buttons:[trigger,{value:0,pressed:false}]}};
const session={inputSources:[right],visibilityState:'visible',enabledFeatures:['viewer','local-floor'],handlers:{},
 updateRenderState(){},requestReferenceSpace:()=>Promise.resolve({}),
 requestAnimationFrame(fn){this.nextFrame=fn;},addEventListener(name,fn){this.handlers[name]=fn;},
 end(){calls.push('end');this.handlers.end();return Promise.resolve();}};
const navigator={userAgent:'test Quest',xr:{isSessionSupported:()=>Promise.resolve(supported),requestSession(mode,options){calls.push('request');requested={mode,options};
 if(sessionFailure)return Promise.reject(sessionFailure);
 if(deferSession)return new Promise(resolve=>{resolveSession=resolve;});return Promise.resolve(session);}}};
function XRWebGLLayer(){this.framebuffer={};this.getViewport=()=>({x:0,y:0,width:100,height:100});}
const frame={getViewerPose:()=>({views:[{},{}]}),getPose:()=>({emulatedPosition:false})};
''')
    script = PAGE.read_text().split('<script>', 1)[1].split('</script>', 1)[0]
    context.eval(f'supported={json.dumps(supported)}')
    context.eval(script)
    drain(context)
    return context


def drain(context):
    while context.execute_pending_job():
        pass


@pytest.mark.parametrize('hand', ['left', 'right'])
def test_probe_native_frames_and_button_events_without_control_transport(hand):
    context = probe_context()
    context.eval(f'side.value={json.dumps(hand)};right.handedness={json.dumps(hand)}')
    context.eval('start.onclick()')
    # User activation is still present: requestSession must happen before yielding.
    assert json.loads(context.eval('JSON.stringify(calls)')) == ['request']
    drain(context)
    assert json.loads(context.eval('JSON.stringify(calls)'))[:4] == ['request', 'canvas', 'context', 'compatible']
    requested = json.loads(context.eval('JSON.stringify(requested)'))
    assert requested == {'mode': 'immersive-vr', 'options': {'requiredFeatures': ['local-floor']}}
    context.eval('session.nextFrame(0,frame);at=1000;trigger.value=0;trigger.pressed=false;'
                 'session.handlers.selectend({inputSource:right,isTrusted:true});session.nextFrame(1000,frame)')
    reports = json.loads(context.eval('JSON.stringify(reports)'))
    assert all(row['url'] == '/tracking/diagnostic' for row in reports)
    assert reports[0]['body']['frame']['init_stage'] == 'page_loaded'
    held = next(row['body']['frame'] for row in reports if row['body']['frame'].get('state') == 'held')
    assert held['probe_hand'] == hand
    assert reports[-1]['body']['frame']['state'] == 'released'
    assert any(event['type'] == 'selectend' and event['trusted'] for event in reports[-1]['body']['events'])
    assert all('deadman_enabled' not in row['body'] for row in reports)
    context.eval('at=2000;session.inputSources=[];session.nextFrame(2000,frame)')
    assert json.loads(context.eval('JSON.stringify(reports[reports.length-1].body)'))['frame']['state'] == 'missing'
    context.eval('session.end()')
    assert not context.eval('start.disabled')


@pytest.mark.parametrize('failure,stage', [('sessionFailure', 'request_session'), ('glFailure', 'make_xr_compatible')])
def test_probe_reports_exact_startup_failure_and_cleans_owned_session(failure, stage):
    context = probe_context()
    context.eval(f'{failure}=new Error("not usable");{failure}.name="InvalidStateError";start.onclick()')
    drain(context)
    report = json.loads(context.eval('JSON.stringify(reports[reports.length-1].body)'))
    assert report['stage'] == 'input_probe_error'
    assert report['frame']['init_stage'] == stage
    assert report['frame']['error']['name'] == 'InvalidStateError'
    assert stage in context.eval('status.textContent')
    assert not context.eval('start.disabled')
    calls = json.loads(context.eval('JSON.stringify(calls)'))
    assert calls.count('end') == (1 if failure == 'glFailure' else 0)
    assert context.eval('canvases.every(canvas=>canvas.removed)')
    context.eval(f'{failure}=null;start.onclick()')
    drain(context)
    assert context.eval('typeof session.nextFrame') == 'function'
    assert context.eval('canvases.length') == (2 if failure == 'glFailure' else 1)


def test_probe_ignores_double_start_during_pending_session_request():
    context = probe_context()
    context.eval('deferSession=true;start.onclick();start.onclick()')
    assert json.loads(context.eval('JSON.stringify(calls)')) == ['request']
    context.eval('resolveSession(session)')
    drain(context)
    assert context.eval('canvases.length') == 1


def test_probe_reports_unsupported_desktop_without_attempting_immersive_session():
    context = probe_context(supported=False)
    assert 'Quest Browser' in context.eval('status.textContent')
    assert context.eval('start.disabled')
    context.eval('start.onclick()')
    drain(context)
    assert json.loads(context.eval('JSON.stringify(calls)')) == []
    report = json.loads(context.eval('JSON.stringify(reports[reports.length-1].body)'))
    assert report['capabilities']['immersive_vr_supported'] is False


def test_probe_has_no_vendor_bootstrap_camera_or_command_routes():
    page = PAGE.read_text()
    for forbidden in ('quest_tracking.js', 'new WebSocket', '/camera/offer', 'deadman_enabled', 'teleop.r1'):
        assert forbidden not in page
    assert "fetch('/tracking/diagnostic'" in page
