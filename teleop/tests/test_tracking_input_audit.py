"""Browser audit preserves control semantics and records short input transitions."""
import json
from pathlib import Path
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/teleop/quest_tracking.js'


@pytest.fixture
def browser():
    context = pytest.importorskip('quickjs').Context()
    context.eval('''
const reports=[], messages=[], timers=[], lifecycle={};
let at=0;
const performance={now:()=>at};
const location={host:'robot:8012',protocol:'https:',href:'https://robot:8012/'};
const fetch=(url,request)=>{reports.push(JSON.parse(request.body));return Promise.resolve({});};
const setTimeout=(fn,ms)=>{timers.push({fn,ms});return timers.length;};
globalThis.addEventListener=(name,fn)=>{lifecycle[name]=fn;};
const document={visibilityState:'visible',addEventListener:(name,fn)=>{lifecycle['document_'+name]=fn;}};
class WebSocket {
 static OPEN=1;
 constructor(){this.readyState=1;this.bufferedAmount=0;globalThis.socket=this;}
 send(data){messages.push(JSON.parse(data));}
 close(){this.readyState=3;}
}
const buttons=[{value:1,pressed:true,touched:true},{value:0,pressed:false}];
const right={handedness:'right',targetRayMode:'tracked-pointer',targetRaySpace:{},gripSpace:{},
 profiles:['oculus-touch-v3','generic-trigger-squeeze-thumbstick'],
 gamepad:{mapping:'xr-standard',connected:true,buttons,axes:[0,0,0,0]}};
const session={inputSources:[right],visibilityState:'visible',enabledFeatures:['local-floor','hand-tracking'],
 handlers:{},addEventListener(name,fn){this.handlers[name]=fn;}};
const promise=Promise.resolve(session);
class XRSystem {requestSession(mode,options){globalThis.nativeRequest={mode,options};return promise;}}
const navigator={xr:new XRSystem()};
const matrix=[1,0,0,0,0,1,0,0,0,0,1,0,0,1.5,0,1];
class XRFrame {
 constructor(){this.session=session;}
 getViewerPose(){return {views:[{},{}],transform:{matrix}};}
 getPose(){return {transform:{matrix}};}
}
const mediaPromise=Promise.resolve();
class RTCPeerConnection {
 constructor(){this.signalingState='stable';this.connectionState='connected';}
 setRemoteDescription(value){this.description=value;return mediaPromise;}
 getStats(){return Promise.resolve(new Map([['video',{type:'inbound-rtp',kind:'video',
   framesDecoded:120,framesDropped:2,freezeCount:1,totalDecodeTime:.4}]]));}
}
const frame=new XRFrame();
Object.assign(globalThis,{XRFrame,XRSystem,RTCPeerConnection});
''')
    context.eval(SCRIPT.read_text())
    return context


def audit_flush(context):
    context.eval('if(timers.length)timers.shift().fn()')
    return json.loads(context.eval('JSON.stringify(reports[reports.length-1])'))


def test_original_session_and_media_promises_unchanged(browser):
    assert browser.eval("navigator.xr.requestSession('immersive-ar',{optionalFeatures:['hand-tracking']})===promise")
    assert browser.eval("new RTCPeerConnection().setRemoteDescription({type:'answer'})===mediaPromise")
    browser.eval('frame.getViewerPose({})')
    report = audit_flush(browser)  # media first timer
    while browser.execute_pending_job():
        pass
    report = audit_flush(browser)
    assert report['audit_revision'] == 'input-audit-4'
    assert any(e['type'] == 'request_session' and e['optional'] == ['hand-tracking'] for e in report['events'])
    assert report['media']['framesDecoded'] == 120
    assert report['media']['freezeCount'] == 1


def test_raw_trigger_grip_and_source_loss_order_preserved(browser):
    browser.eval('frame.getViewerPose({});at=40;buttons[0].value=0;buttons[0].pressed=false;'
                 'buttons[1].value=1;buttons[1].pressed=true;frame.getViewerPose({});'
                 "at=80;session.inputSources=[];session.handlers.inputsourceschange({added:[],removed:[right]});"
                 'frame.getViewerPose({});at=90;session.visibilityState="visible-blurred";session.handlers.visibilitychange()')
    report = audit_flush(browser)
    events = report['events']
    states = [e['source'] for e in events if e['type'] == 'right_buttons' and e['source']]
    assert states[0]['trigger']['v'] == 1
    assert states[1]['trigger']['v'] == 0 and states[1]['grip']['v'] == 1
    change = next(e for e in events if e['type'] == 'input_sources_change')
    assert change['removed'][0]['side'] == 'right' and change['ms'] == 80
    assert events[-1]['type'] == 'stop' and events[-1]['reason'] == 'session_visible-blurred'
    packets = json.loads(browser.eval('JSON.stringify(messages)'))
    assert packets[0]['valid'] and packets[0]['rightState']['triggerValue'] == 1
    assert packets[1]['valid'] and packets[1]['rightState']['triggerValue'] == 0
    assert not packets[2]['valid'] and packets[2]['reason'] == 'right_input_source_missing'


def test_diagnostics_bounded_and_inspection_error_does_not_stop_control(browser):
    browser.eval("Object.defineProperty(right,'profiles',{get(){throw new Error('audit only');}});"
                 'for(let i=0;i<200;i++){at=i*40;buttons[0].value=i%2;frame.getViewerPose({});}')
    report = audit_flush(browser)
    assert len(json.dumps(report, ensure_ascii=False)) < 4096
    assert len(report['events']) <= 16
    assert browser.eval('messages.length') == 200
    assert browser.eval('messages.every(packet=>packet.valid)')


def test_document_visibility_and_frame_stall_are_recorded(browser):
    browser.eval('frame.getViewerPose({});at=350;frame.getViewerPose({});'
                 'document.visibilityState="hidden";lifecycle.document_visibilitychange();lifecycle.blur()')
    report = audit_flush(browser)
    assert report['frame']['max_gap_ms'] == 350
    assert any(e['type'] == 'frame_gap' and e['gap_ms'] == 350 for e in report['events'])
    assert any(e['type'] == 'document_visibility' and e['state'] == 'hidden' for e in report['events'])
    assert any(e['type'] == 'blur' for e in report['events'])


def test_browser_input_events_distinguished_from_script_dispatch(browser):
    browser.eval('frame.getViewerPose({});session.handlers.selectend({inputSource:right,isTrusted:true});'
                 'session.handlers.selectstart({inputSource:right,isTrusted:false})')
    report = audit_flush(browser)
    events = {e['type']: e for e in report['events']}
    assert events['selectend']['trusted'] is True
    assert events['selectstart']['trusted'] is False


def test_controller_profile_requires_explicit_query_and_preserves_original_options(browser):
    browser.eval("globalThis.original={requiredFeatures:['local-floor'],optionalFeatures:['hand-tracking','layers']};"
                 "navigator.xr.requestSession('immersive-ar',original)")
    assert browser.eval('nativeRequest.options===original')
    browser.eval("location.href='https://robot:8012/?hb_controller_only=1&ws=wss://robot:8012';"
                 "original.requiredFeatures.push('hand-tracking');"
                 "original.optionalFeatures.push('mesh-detection','plane-detection','hit-test','anchors','body-tracking')")
    assert browser.eval("navigator.xr.requestSession('immersive-ar',original)===promise")
    request = json.loads(browser.eval('JSON.stringify(nativeRequest)'))
    assert request == {'mode': 'immersive-ar', 'options': {
        'requiredFeatures': ['local-floor'], 'optionalFeatures': ['layers']}}
    assert json.loads(browser.eval('JSON.stringify(original.requiredFeatures)')) == ['local-floor', 'hand-tracking']
    browser.eval('frame.getViewerPose({})')
    report = audit_flush(browser)
    assert report['session_profile'] == 'controller-only'
    assert any(e['type'] == 'request_session' and e['profile'] == 'controller-only' for e in report['events'])


@pytest.mark.parametrize('url', ['https://robot:8012/?hb_controller_only=10',
                              'https://robot:8012/?ws=wss://robot:8012#hb_controller_only=1'])
def test_controller_profile_does_not_match_partial_or_fragment_flags(browser, url):
    browser.eval(f'location.href={json.dumps(url)};')
    browser.eval("globalThis.original={optionalFeatures:['hand-tracking']};navigator.xr.requestSession('immersive-vr',original)")
    assert browser.eval('nativeRequest.options===original')


def test_browser_heap_and_context_loss_audit_preserves_control(browser):
    browser.eval('performance.memory={usedJSHeapSize:1000000,totalJSHeapSize:2000000,jsHeapSizeLimit:100000000};'
                 'frame.getViewerPose({});lifecycle.document_webglcontextlost()')
    report = audit_flush(browser)
    assert report['browser_memory']['used_js_heap_bytes'] == 1000000
    assert any(e['type'] == 'webglcontextlost' for e in report['events'])
    assert browser.eval('messages.every(packet=>packet.valid)')
    browser.eval("Object.defineProperty(performance,'memory',{get(){throw new Error('unavailable');}});"
                 'at=1000;frame.getViewerPose({})')
    assert audit_flush(browser)['browser_memory'] == {'available': False}
    assert browser.eval('messages.every(packet=>packet.valid)')


def test_emulated_position_sends_only_stop_and_live_button_proof(browser):
    browser.eval('frame.getViewerPose({});'
                 'XRFrame.prototype.getPose=function(){return {emulatedPosition:true,transform:{matrix}};};'
                 'at=40;frame.getViewerPose({});')
    packets = json.loads(browser.eval('JSON.stringify(messages)'))
    assert packets[0]['rightInputId'] == packets[1]['rightInputId']
    assert packets[0]['rightInputConnected'] is True
    stop = packets[1]
    assert stop['valid'] is False
    assert stop['reason'] == 'right_pose_missing_or_emulated'
    assert stop['rightState'] == {'trigger': True, 'triggerValue': 1}
    assert stop['positionPauseProof']['viewer_tracked'] is True
    assert 'right' not in stop and 'left' not in stop
    # An absent pose has no proof, even while the button remains held.
    browser.eval('XRFrame.prototype.getPose=function(){return null;};at=80;frame.getViewerPose({});')
    assert 'positionPauseProof' not in json.loads(browser.eval('JSON.stringify(messages[2])'))
