#!/usr/bin/env python3
"""Capture/replay -> OpenAI bbox -> coordinated head+arm target -> optional owner stream."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import uuid

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT))
from ai_modules.vision import openai_grounding as grounding
from control import runtime_info,limits,validate_target
from vision_geometry import combined_target,validate_config,camera_pose

CAPTURE_SCRIPT=ROOT/'experiments/r1_object_grounding/p2_look_at_dryrun/capture_observation.py'
BBOX_SCRIPT=ROOT/'experiments/r1_object_grounding/p2_bbox_baseline/run.py'


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def validate_robot(value):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:-]*',value):
        raise ValueError('invalid SSH robot address')
    return value


def verify_capture(directory):
    capture=json.loads((directory/'capture.json').read_text())
    image=directory/'frame.jpg'
    if hashlib.sha256(image.read_bytes()).hexdigest()!=capture['image_sha256']:
        raise ValueError('capture image SHA256 mismatch')
    return capture,image


def read_bbox(directory,capture):
    result=json.loads((directory/'result.json').read_text())
    observation=json.loads((directory/'observation.json').read_text())
    if result['observation_id']!=observation['observation_id']:
        raise ValueError('bbox observation ID mismatch')
    if observation['source_sha256']!=capture['image_sha256']:
        raise ValueError('bbox and capture refer to different source images')
    if hashlib.sha256((directory/'frame.png').read_bytes()).hexdigest()!=observation['image_sha256']:
        raise ValueError('submitted image SHA256 mismatch')
    proposal=grounding.validate_proposal(result['proposal'])
    if proposal['decision']!=result['decision']:raise ValueError('bbox decision mismatch')
    if proposal['decision']=='selected':
        if not result.get('target') or result['target']['observation_id']!=observation['observation_id']:
            raise ValueError('selected target missing/mismatched observation')
        if result['target']['bbox_normalized_1000']!=proposal['bbox']:
            raise ValueError('target bbox differs from proposal')
    return result,observation,proposal


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--capture-dir',type=Path,help='Existing matching frame.jpg + capture.json')
    source.add_argument('--robot',help='Fresh read-only capture over SSH, e.g. unitree@10.42.0.114')
    p.add_argument('--query',help='One requested object, Vietnamese supported')
    p.add_argument('--bbox-run',type=Path,help='Replay matching bbox without API call')
    p.add_argument('--control-config',type=Path,default=HERE/'config.json')
    p.add_argument('--vision-config',type=Path,default=HERE/'vision_config.json')
    p.add_argument('--bbox-config',type=Path,default=ROOT/'experiments/r1_object_grounding/p2_bbox_baseline/config.json')
    p.add_argument('--runtime-root',type=Path,default=ROOT/'third_party/Happy-Baby-R1-develop')
    p.add_argument('--assumed-range-m',type=float)
    p.add_argument('--interface',default='eth10')
    p.add_argument('--execute',action='store_true',help='Fresh --robot only: copy bounded target client and stream to existing robot owner')
    p.add_argument('--robot-runtime-root',default='/home/unitree/HB')
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.robot:validate_robot(a.robot)
    if a.robot and a.bbox_run:p.error('fresh capture cannot reuse an older bbox')
    if not a.bbox_run and not a.query:p.error('--query required for an API call')
    if not a.bbox_run and not os.environ.get('OPENAI_API_KEY','').strip():
        p.error('OPENAI_API_KEY is missing on workstation; no capture or motion started')
    if a.execute and not a.robot:p.error('--execute requires fresh --robot capture; replay is offline')
    control=json.loads(a.control_config.read_text());vision=json.loads(a.vision_config.read_text())
    if a.assumed_range_m is not None:vision['assumed_range_m']=a.assumed_range_m
    validate_config(vision)
    info=runtime_info(a.runtime_root,control)
    camera_pose(info['urdf'],[0.,0.],vision)
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    save(out/'resolved_config.json',{'control':control,'vision':vision,'runtime_root':str(a.runtime_root.resolve()),
         'bbox_config':json.loads(a.bbox_config.read_text()),'mode':'fresh_capture' if a.robot else 'existing_capture',
         'api_called':a.bbox_run is None,'execute_requested':a.execute,'query':a.query,
         'interface':a.interface,'robot':a.robot,'robot_runtime_root':a.robot_runtime_root})
    save(out/'runtime_reference.json',info)
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    files=[Path(__file__),HERE/'control.py',HERE/'vision_geometry.py',HERE/'run.py',HERE/'state.py',
           Path(grounding.__file__),CAPTURE_SCRIPT,BBOX_SCRIPT,a.control_config,a.vision_config,a.bbox_config]
    snap=out/'source_snapshot';snap.mkdir()
    for i,src in enumerate(files):shutil.copy2(src,snap/f'{i:02d}_{src.name}')
    save(out/'source_sha256.json',{str(src.resolve()):hashlib.sha256(src.read_bytes()).hexdigest() for src in files})
    stages=[]
    def command(argv,name):
        record={'stage':name,'command':shlex.join(argv),'returncode':None}
        stages.append(record);save(out/'stages.json',stages)
        try:
            process=subprocess.run(argv,capture_output=True,text=True,timeout=180)
        except subprocess.TimeoutExpired as exc:
            def text(value):return value.decode(errors='replace') if isinstance(value,bytes) else (value or '')
            (out/(name+'.log')).write_text(text(exc.stdout)+text(exc.stderr))
            record['failure']='timeout';save(out/'stages.json',stages)
            raise RuntimeError(f'{name} timeout; remote command may have started') from None
        (out/(name+'.log')).write_text(process.stdout+process.stderr)
        record['returncode']=process.returncode;save(out/'stages.json',stages)
        if process.returncode:raise RuntimeError(f'{name} failed; see {name}.log')
    token='r1_vision_'+uuid.uuid4().hex
    def execute_on_robot(target_file):
        work='/home/unitree/r1_target_control_runs/'+token
        command(['ssh',a.robot,shlex.join(['mkdir','-p',work+'/client'])],'robot_prepare')
        client_files=[HERE/'run.py',HERE/'control.py',HERE/'state.py']
        command(['scp',*map(str,client_files),a.robot+':'+work+'/client/'],'robot_copy_client')
        command(['scp',str(a.control_config),a.robot+':'+work+'/client/config.json'],'robot_copy_config')
        command(['scp',str(target_file),a.robot+':'+work+'/target.json'],'robot_copy_target')
        argv=['python3',work+'/client/run.py','--config',work+'/client/config.json','--runtime-root',a.robot_runtime_root,
              'stream','--target',work+'/target.json','--interface',a.interface,'--output-dir',work+'/stream']
        command(['ssh',a.robot,shlex.join(argv)],'robot_stream')
        local=out/'robot_stream';local.mkdir()
        command(['scp',a.robot+':'+work+'/stream/result.json',a.robot+':'+work+'/stream/trace.csv',str(local)+'/'],'robot_fetch_stream')
        return json.loads((local/'result.json').read_text())
    try:
        capture_dir=out/'capture';capture_dir.mkdir()
        if a.robot:
            remote_script='/tmp/'+token+'_capture.py';remote_dir='/tmp/'+token
            command(['scp',str(CAPTURE_SCRIPT),a.robot+':'+remote_script],'copy_capture_script')
            command(['ssh',a.robot,shlex.join(['python3',remote_script,'--interface',a.interface,'--output-dir',remote_dir])],'capture')
            command(['scp',a.robot+':'+remote_dir+'/frame.jpg',a.robot+':'+remote_dir+'/capture.json',str(capture_dir)+'/'],'fetch_capture')
        else:
            verify_capture(a.capture_dir)
            for name in ['frame.jpg','capture.json']:shutil.copy2(a.capture_dir/name,capture_dir/name)
        capture,image=verify_capture(capture_dir)
        captured_q=capture['state_near_rpc_midpoint_proxy']['q_rad']
        validate_target({name:captured_q[name] for name in ['head_pitch_joint','head_yaw_joint']},limits(info['urdf'],control))
        bbox=out/'bbox'
        if a.bbox_run:
            # Verify source linkage before copying/reusing a model result.
            read_bbox(a.bbox_run,capture)
            stored_query=json.loads((a.bbox_run/'resolved_config.json').read_text())['query']
            if a.query is not None and a.query!=stored_query:raise ValueError('replay query differs from recorded query')
            bbox.mkdir()
            for name in ['result.json','observation.json','frame.png','annotated.png','resolved_config.json']:
                shutil.copy2(a.bbox_run/name,bbox/name)
            for name in ['response.json','provenance.json']:
                if (a.bbox_run/name).is_file():shutil.copy2(a.bbox_run/name,bbox/name)
            query=stored_query
        else:
            query=a.query
            command([sys.executable,str(BBOX_SCRIPT),'--image',str(image),'--query',query,
                     '--config',str(a.bbox_config),'--output-dir',str(bbox)],'bbox')
        result,observation,proposal=read_bbox(bbox,capture)
        source_note={'bbox_run':str(a.bbox_run.resolve()) if a.bbox_run else str(bbox),
                     'observation_id':observation['observation_id'],'source_image_sha256':capture['image_sha256'],
                     'query':query,'api_called_this_run':a.bbox_run is None}
        if proposal['decision']!='selected':
            summary={**source_note,'status':'needs_clarification' if proposal['decision']=='ask' else 'target_not_found',
                     'clarification':proposal['clarification'],'target':None,'motor_commands_sent':False}
        else:
            # Recompute from the validated bbox and recorded calibration, not model-provided robot coordinates.
            cfg=json.loads((bbox/'resolved_config.json').read_text())
            cal=Path(cfg['calibration']) if cfg.get('calibration') else None
            if cal is None:raise ValueError('bbox calibration required')
            provenance_path=(a.bbox_run/'provenance.json') if a.bbox_run else bbox/'provenance.json'
            if provenance_path.is_file():
                expected=json.loads(provenance_path.read_text()).get('calibration_sha256')
                if expected and hashlib.sha256(cal.read_bytes()).hexdigest()!=expected:
                    raise ValueError('calibration changed since bbox run')
            shutil.copy2(cal,out/'calibration.npz')
            save(out/'calibration_reference.json',{'source':str(cal.resolve()),'sha256':hashlib.sha256(cal.read_bytes()).hexdigest()})
            from PIL import Image
            with Image.open(bbox/'frame.png') as im:wh=im.size
            if tuple(observation['image_size_wh'])!=wh:raise ValueError('bbox image dimension mismatch')
            pixel=grounding.pixel_box(proposal['bbox'],*wh)
            ray=grounding.optical_ray(pixel['center_uv'],wh,cal)
            save(out/'camera_ray.json',ray)
            combined=combined_target(ray['unit_xyz'],capture,info['urdf'],control,vision)
            save(out/'geometry.json',combined)
            summary={**source_note,'status':combined['status'],'target':None,'motor_commands_sent':False}
            if combined['status']=='planned':
                detail=combined['details']
                target={'schema':'r1_target_control.v1','kind':'vision_head_and_arm_approx',
                        'joints_rad':combined['joints_rad'],'details':detail,
                        'policy_sha256':info['policy_sha256'],'created_utc':datetime.now(timezone.utc).isoformat(),
                        'collision_checked':False,'balance_validated':False,'label':proposal['label'],
                        'target_id':result['target']['target_id'],'observation_id':observation['observation_id'],
                        'source_image_sha256':capture['image_sha256'],'preferred_arm':detail['preferred_arm'],
                        'selected_arm':detail['selected_arm'],'assumed_range_m':vision['assumed_range_m'],
                        'range_is_measured':False}
                save(out/'target.json',target)
                summary.update(target='target.json',preferred_arm=detail['preferred_arm'],selected_arm=detail['selected_arm'],
                               arm_fallback_used=detail['arm_fallback_used'],head_pitch_yaw_deg=detail['head']['candidate_pitch_yaw_deg'])
                if a.execute:
                    stream=execute_on_robot(out/'target.json')
                    summary.update(status='stream_finished_check_tracking',motor_commands_sent=stream['packets_sent']>0,
                                   robot_stream_result=stream)
        save(out/'pipeline.json',summary);print(json.dumps({'output_dir':str(out),**summary},ensure_ascii=False,indent=2))
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        save(out/'failure.json',{'status':'cancelled' if isinstance(exc,KeyboardInterrupt) else 'failed',
             'message':str(exc),'execute_requested':a.execute,
             'motor_commands_may_have_been_sent':any(s['stage']=='robot_stream' for s in stages),
             'note':'Check stages and logs; no fallback to an invented target.'})
        raise


if __name__=='__main__':raise SystemExit(main())
