#!/usr/bin/env python3
"""P4a/P4b bounded search: offline replay or explicit external process adapter."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import sys
from backends import Replay,FeedbackReplay,ProcessBackend,PipelineBackend
from search import Finished,Search,validate_config

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--replay',type=Path)
    mode.add_argument('--robot',help='Live SSH bridge to prepared P4 owner/agent')
    mode.add_argument('--adapter-argv',type=Path,help='JSON argv array for your hardware adapter')
    p.add_argument('--config',type=Path,default=HERE/'config.json')
    p.add_argument('--query',required=True)
    p.add_argument('--vision-config',type=Path,help='Override existing head/arm vision config')
    p.add_argument('--runtime-root',type=Path,help='Runtime root for vision geometry and policy hash')
    p.add_argument('--bbox-config',type=Path,help='BBox config, including camera calibration path')
    p.add_argument('--body-turns',type=int,help='Limit number of body offsets, e.g. 1 for first hardware turn')
    p.add_argument('--body-step-deg',type=float,help='Scan stage in degrees; up to 180 per body command')
    p.add_argument('--interactive',action='store_true',help='Ask clarification in terminal and continue, up to 3 rounds')
    p.add_argument('--head-only',action='store_true')
    p.add_argument('--batch-head-capture',action='store_true',help='Capture all three head views before processing; one selected view is enough')
    p.add_argument('--resume-run',type=Path,help='Reuse two candidate boxes and capture from a prior clarification')
    p.add_argument('--candidate-index',type=int,choices=(0,1))
    p.add_argument('--cancel-file',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.resume_run and a.candidate_index is None:p.error('--resume-run requires --candidate-index')
    if not a.replay and not a.resume_run and not os.environ.get('OPENAI_API_KEY','').strip():
        p.error('OPENAI_API_KEY is missing on workstation; no robot commands sent. Export it before live search.')
    c=json.loads(a.config.read_text())
    if a.interactive and a.replay:p.error('--interactive requires live backend')
    if a.body_turns is not None:
        if a.body_turns<1:p.error('--body-turns must be >=1')
        c['body_offsets_deg']=c['body_offsets_deg'][:a.body_turns]
    if a.body_step_deg is not None:
        if not 0<a.body_step_deg<=180:p.error('--body-step-deg must be in (0,180]')
        c['body_offsets_deg']=[a.body_step_deg*(i+1) for i in range(len(c['body_offsets_deg']))]
    if a.head_only:c['enable_body']=False
    if a.batch_head_capture:c['batch_head_capture']=True
    validate_config(c)
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    def save(name,value):(out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    if a.robot:
        bridge=HERE/'hardware/workstation_bridge.py'
        a.adapter_argv=out/'adapter_argv.json'
        save('adapter_argv.json',[sys.executable,str(bridge),'--robot',a.robot,'--cache-dir',str(out/'captures')])
    save('resolved_config.json',{'search':c,'query':a.query,'mode':'replay' if a.replay else 'external_adapter',
                                'resume_run':str(a.resume_run.resolve()) if a.resume_run else None,
                                'candidate_index':a.candidate_index,
                                'input':str((a.replay or a.adapter_argv).resolve()),
                                'vision_config_override':str(a.vision_config.resolve()) if a.vision_config else None,
                                'runtime_root_override':str(a.runtime_root.resolve()) if a.runtime_root else None,
                                'bbox_config_override':str(a.bbox_config.resolve()) if a.bbox_config else None})
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    sources=([HERE/'hardware/workstation_bridge.py',HERE/'hardware/robot_agent.py',HERE/'hardware/SearchYawReceiver.hpp',HERE/'hardware/prepare_owner.py'] if a.robot else [])+[v for v in [a.vision_config,a.bbox_config] if v]+[HERE/n for n in ['run.py','search.py','backends.py','feedback.py']]+[a.config,a.replay or a.adapter_argv]
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for i,f in enumerate(sources):(snapshot/(str(i)+'_'+f.name)).write_bytes(f.read_bytes())
    save('source_sha256.json',{str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
    if a.replay:
        b=FeedbackReplay(json.loads(a.replay.read_text()))
    else:
        transport=ProcessBackend(json.loads(a.adapter_argv.read_text()),a.cancel_file,c['adapter_timeout_s'])
        args=['--vision-config',str(a.vision_config.resolve())] if a.vision_config else []
        if a.runtime_root:args+=['--runtime-root',str(a.runtime_root.resolve())]
        if a.bbox_config:args+=['--bbox-config',str(a.bbox_config.resolve())]
        b=PipelineBackend(transport,out,args)
    original=b.cancelled;cancelled=[False]
    b.cancelled=lambda:cancelled[0] or (a.cancel_file is not None and a.cancel_file.exists()) or original()
    handlers={}
    for sig in [signal.SIGINT,signal.SIGTERM]:
        handlers[sig]=signal.signal(sig,lambda *_:cancelled.__setitem__(0,True))
    try:
        with (out/'events.jsonl').open('w') as log:
            def emit(event):log.write(json.dumps({'round':round_index+1,**event},ensure_ascii=False)+'\n');log.flush()
            episodes=[];query=a.query;round_index=0
            if a.resume_run:
                prior=json.loads((a.resume_run/'result.json').read_text())
                pending=prior.get('pending_candidates')
                if prior.get('status')!='needs_clarification' or not pending or len(pending.get('candidates',[]))!=2:
                    raise ValueError('prior run has no reusable candidates')
                engine=Search(b,c,emit)
                try:
                    selected=b.resume_candidate(pending,a.candidate_index,query)
                    action=engine.point_or_recenter(query,selected)
                    result=engine.result('found' if action else 'failed',None,action)
                except Finished as done:
                    result=engine.result(done.status,done.reason,done.action)
                except Exception as exc:
                    result=engine.result('failed',type(exc).__name__+': '+str(exc),None)
                finally:
                    try:b.stop()
                    except Exception as exc:result=engine.result('failed','stop failed: '+str(exc),None)
                episodes.append({'query':query,'result':result});save('episodes.json',episodes)
            else:
                for round_index in range(3):
                    result=Search(b,c,emit).run(query)
                    episodes.append({'query':query,'result':result})
                    save('episodes.json',episodes)
                    if not a.interactive or result['status']!='needs_clarification' or round_index==2:break
                    print(result['reason'] or 'Bạn muốn chọn vật nào?',flush=True)
                    try:reply=input('Mô tả rõ vật (Enter để dừng): ').strip()
                    except (EOFError,KeyboardInterrupt):break
                    if not reply:break
                    query=a.query+'; Làm rõ yêu cầu: '+reply
        answer={'found':'Đã tìm thấy mục tiêu và đã thực hiện bước nhìn/chỉ; xem kết quả tracking.',
                'not_found_in_observed_views':'Chưa tìm thấy vật trong các hướng đã quan sát.',
                'needs_clarification':result.get('reason'),
                'needs_body_turn':result.get('reason'),
                'target_unreachable':result.get('reason'),
                'target_lost_after_recenter':result.get('reason'),
                'cancelled':'Đã hủy tìm kiếm.', 'failed':'Tác vụ chưa hoàn thành: '+str(result.get('reason'))}
        result['answer']=answer[result['status']]
        result['mode']='replay' if a.replay else 'external_adapter'
        save('result.json',result);print(json.dumps({'output_dir':str(out),**result},ensure_ascii=False,indent=2))
        return 1 if result['status']=='failed' else 0
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)


if __name__=='__main__':raise SystemExit(main())
