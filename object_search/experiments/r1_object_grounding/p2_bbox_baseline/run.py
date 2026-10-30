#!/usr/bin/env python3
"""Image + request -> OpenAI bbox proposal -> overlay + optional camera ray. No robot I/O."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from ai_modules.vision import openai_grounding as grounding


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--query',required=True)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    parser.add_argument('--model',help='Override Responses model; Realtime model IDs do not use this endpoint')
    parser.add_argument('--calibration',type=Path)
    parser.add_argument('--output-dir',type=Path,required=True)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare-only',action='store_true',help='Validate/save input without calling API')
    mode.add_argument('--proposal-json',type=Path,help='Replay a saved proposal offline; does not call API')
    args=parser.parse_args()
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw
    cfg=json.loads(args.config.read_text())
    if args.model:cfg['model']=args.model
    cal=args.calibration or (ROOT/cfg['calibration'] if cfg.get('calibration') else None)
    required={'model','reasoning_effort','image_detail','max_output_tokens','timeout_s'}
    if not required <= cfg.keys():parser.error('incomplete config')
    if cfg['image_detail'] not in ('auto','low','high') or cfg['timeout_s']<=0 or cfg['max_output_tokens']<=0:
        parser.error('invalid detail, timeout or token budget')
    if not args.query.strip():parser.error('query is empty')
    if not args.prepare_only and args.proposal_json is None and not os.environ.get('OPENAI_API_KEY','').strip():
        parser.error('OPENAI_API_KEY is missing; use --prepare-only for offline preparation')
    source=args.image.read_bytes()
    if len(source)>20*1024*1024:parser.error('input exceeds this pilot limit of 20 MiB')
    with Image.open(args.image) as image:
        if image.getexif().get(274,1)!=1:
            parser.error('EXIF rotation present; prepare an explicitly oriented image first')
        image=image.convert('RGB');width,height=image.size
        if width*height>20_000_000:parser.error('input exceeds this pilot limit of 20 megapixels')
        png_path_image=image.copy()
    if cal:
        # Check geometry before any paid request. No 3D or base-frame transform is inferred.
        grounding.optical_ray([width/2,height/2],(width,height),cal)
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    def save(name,value):
        (out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    png_path_image.save(out/'frame.png')
    submitted=(out/'frame.png').read_bytes();sha=hashlib.sha256(submitted).hexdigest()
    observation='obs_'+sha[:16]
    request=grounding.build_request(submitted,args.query,(width,height),cfg)
    save('resolved_config.json',{**cfg,'calibration':str(cal.resolve()) if cal else None,
         'source_image':str(args.image.resolve()),'query':args.query,
         'mode':'prepare_only' if args.prepare_only else ('offline_replay' if args.proposal_json else 'openai_responses')})
    save('observation.json',{'observation_id':observation,'image_sha256':sha,
         'source_sha256':hashlib.sha256(source).hexdigest(),'image_size_wh':[width,height],
         'image':'frame.png','loaded_at_utc':datetime.now(timezone.utc).isoformat(),
         'captured_at_utc':None,'head_state':None,'body_pose':None,
         'note':'Load time is not capture time; no temporal or robot-frame validity is claimed.'})
    (out/'prompt.txt').write_text(grounding.INSTRUCTIONS+'\n\n'+args.query+'\n')
    save('schema.json',grounding.SCHEMA)
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    sources=[Path(__file__),Path(grounding.__file__),args.config]
    snap=out/'source_snapshot';snap.mkdir()
    for i,path in enumerate(sources):shutil.copy2(path,snap/f'{i}_{path.name}')
    git_head=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,text=True,capture_output=True)
    save('provenance.json',{'python':sys.version,'opencv':cv2.__version__,'numpy':np.__version__,
         'git_head':git_head.stdout.strip() if git_head.returncode==0 else None,
         'source_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
         'calibration_sha256':hashlib.sha256(cal.read_bytes()).hexdigest() if cal else None})
    if args.prepare_only:
        save('result.json',{'status':'prepared_only','observation_id':observation,'motor_execution_enabled':False})
        print(f'Prepared offline: {out}');return 0
    try:
        started=time.monotonic()
        if args.proposal_json:
            save('replayed_proposal.json',json.loads(args.proposal_json.read_text()))
            proposal=grounding.validate_proposal(json.loads(args.proposal_json.read_text()))
            api={'source':'offline_replay','api_called':False}
        else:
            response=grounding.query_openai(request,cfg['timeout_s'])
            save('response.json',response)
            proposal=grounding.extract_proposal(response)
            api={'source':'openai_responses','api_called':True,'response_id':response.get('id'),
                 'returned_model':response.get('model'),'usage':response.get('usage'),
                 'request_duration_ms':round((time.monotonic()-started)*1000,3)}
        target=None
        overlay=png_path_image.copy();draw=ImageDraw.Draw(overlay)
        if proposal['decision']=='selected':
            box=grounding.pixel_box(proposal['bbox'],width,height)
            target={'target_id':'target_'+hashlib.sha256((observation+'\n'+args.query).encode()).hexdigest()[:16],
                    'observation_id':observation,'label':proposal['label'],'bbox_normalized_1000':proposal['bbox'],
                    'bbox_pixels_xyxy':box['xyxy'],'center_uv':box['center_uv'],
                    'visual_localization_verified':False,'robot_position_m':None,
                    'camera_ray':grounding.optical_ray(box['center_uv'],(width,height),cal) if cal else None}
            draw.rectangle(box['xyxy'],outline='red',width=3)
            u,v=box['center_uv'];draw.ellipse((u-5,v-5,u+5,v+5),fill='yellow')
            # ASCII ID avoids font-dependent rendering of Vietnamese labels.
            draw.text((max(0,box['xyxy'][0]),max(0,box['xyxy'][1]-15)),target['target_id'],fill='red')
        elif proposal['decision']=='ask':
            for index,candidate in enumerate(proposal['candidates']):
                box=grounding.pixel_box(candidate['bbox'],width,height)
                draw.rectangle(box['xyxy'],outline='orange',width=3)
                draw.text((box['xyxy'][0],max(0,box['xyxy'][1]-15)),str(index+1),fill='orange')
        overlay.save(out/'annotated.png')
        save('result.json',{'status':'proposal_only','decision':proposal['decision'],'proposal':proposal,
             'observation_id':observation,'target':target,'api':api,'motor_execution_enabled':False,
             'verification_required':'Inspect annotated.png; schema and calibration do not prove correct localization.'})
        print(json.dumps({'output_dir':str(out),'decision':proposal['decision'],'target':target},ensure_ascii=False,indent=2))
        return 0
    except Exception as exc:
        # Do not log request headers, environment or base64 image payloads.
        save('failure.json',{'status':'failed_no_target','error_type':type(exc).__name__,
             'message':str(exc).replace(os.environ.get('OPENAI_API_KEY','\0'),'[REDACTED]'),
             'motor_execution_enabled':False})
        raise


if __name__=='__main__':
    raise SystemExit(main())
