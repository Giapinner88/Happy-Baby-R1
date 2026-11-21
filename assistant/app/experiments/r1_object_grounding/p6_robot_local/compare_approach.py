"""Prepare or explicitly execute A/B approach workflows; no automatic retries."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(ROOT),str(HERE)]
from ai_modules.navigation.approach import Config
from runtime import build_navigation


def variant_config(base,definition,strategy,execute=False):
    v=definition['variants'][strategy]
    if type(v['segments']) is not int or not 1<=v['segments']<=10:raise ValueError('invalid segment count')
    if not math.isfinite(v['max_advance_m']) or not 0<v['max_advance_m']<=3:raise ValueError('invalid segment distance')
    if v['max_advance_m']*v['segments']>3:raise ValueError('total distance exceeds experiment budget')
    c=copy.deepcopy(base)
    if c.get('approach_mode')!='initial_target_then_odom':raise ValueError('requires initial estimate then odometry')
    c['planner']['controller'].update(max_advance_m=v['max_advance_m'],standoff_m=definition['standoff_m'],
                                    goal_tolerance_m=definition['goal_tolerance_m'])
    c['gait_approach']['max_duration_s']=definition['control_budget_s']
    c['task_duration_s']=definition['outer_task_s']
    if not 0<definition['control_budget_s']<definition['outer_task_s']<=120:raise ValueError('invalid task budgets')
    c['execute']=execute;c['clear_corridor_confirmed']=execute
    c['planner']['corridor']['operator_confirmed']=execute
    c['gait_approach']['execute_enabled']=execute
    Config(**c['planner']['controller']);build_navigation(c)
    return c


def wait_still(read,definition,*,clock=time.monotonic,wait=time.sleep):
    timeout=definition['settle_timeout_s'];duration=definition['settle_min_s'];speed=definition['settle_leg_dq_radps']
    if not 0<duration<timeout<=20 or not 0<speed<=.2:raise ValueError('invalid measured settle limits')
    deadline=clock()+timeout;since=None;records=[]
    while clock()<deadline:
        state=read();now=clock();records.append(dict(state,local_monotonic_s=now))
        if state['max_abs_leg_dq_rad_s']<=speed:
            if since is None:since=now
            if now-since>=duration:return records
        else:since=None
        wait(.05)
    raise TimeoutError('legs did not settle; no next segment')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--definition',type=Path,default=HERE/'approach_comparison/config.json')
    p.add_argument('--strategy',choices=['A','B'],required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--query')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-clear-corridor',action='store_true')
    a=p.parse_args()
    if a.execute and (os.geteuid()!=0 or not a.confirm_clear_corridor):p.error('execute requires sudo and --confirm-clear-corridor')
    definition=json.loads(a.definition.read_text())
    base=json.loads((a.definition.parent/definition['base_config']).read_text())
    config=variant_config(base,definition,a.strategy,a.execute)
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'resolved_definition.json').write_text(json.dumps(definition,indent=2)+'\n')
    (out/'effective_config.json').write_text(json.dumps(config,indent=2)+'\n')
    sources=[Path(__file__),ROOT/'ai_modules/navigation/approach.py',HERE/'runtime.py']
    report={'strategy':a.strategy,'variant':definition['variants'][a.strategy],
            'mode':'execute' if a.execute else 'prepare_only','command':sys.argv,
            'physical_arrival_verified':False,'segments':[],
            'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}}
    if not a.execute:
        report['status']='prepared; no camera, SDK, cloud, service or motor calls'
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2));return
    # Read-only subscriber, never a second LowCmd owner.
    from bounded_trial import Feedback
    runs=ROOT/config['runs_dir']
    wrapper=HERE/'run_bounded_live.sh'
    if not wrapper.exists():wrapper=ROOT.parent/'run_bounded_live.sh'
    try:
        feedback=Feedback(config['interface'])
        for i in range(definition['variants'][a.strategy]['segments']):
            settle=wait_still(feedback.read,definition)
            (out/('settle_%02d.json'%i)).write_text(json.dumps(settle,indent=2)+'\n')
            before={d.name for d in runs.iterdir() if d.is_dir()}
            suffix='compare_%s_%d_%d'%(a.strategy,time.monotonic_ns(),i)
            launched=time.monotonic()
            subprocess.run(['bash',str(wrapper),'--execute',str(out/'effective_config.json'),suffix,a.query or definition['query']],check=True)
            created=[d for d in runs.iterdir() if d.is_dir() and d.name not in before]
            if len(created)!=1:raise RuntimeError('cannot identify unique new task; no next segment')
            result=json.loads((created[0]/'result.json').read_text())
            control=[json.loads(l) for l in (created[0]/'control.jsonl').read_text().splitlines()] if (created[0]/'control.jsonl').exists() else []
            acks=[r['ack']['received_monotonic_s'] for r in control if r.get('ack') and
                  (r['ack'].get('vx_mps',0)>0 or r['ack'].get('wz_radps',0)!=0)]
            gait_acks=[r['ack']['received_monotonic_s'] for r in control if r.get('ack') and
                       (r['ack'].get('vx_mps',0)>.15 or abs(r['ack'].get('wz_radps',0))>.15)]
            report['segments'].append({'task_dir':str(created[0]),'result':result,
                'wall_s':time.monotonic()-launched,
                'first_nonzero_ack_after_launch_s':min(acks)-launched if acks else None,
                'first_gait_request_ack_after_launch_s':min(gait_acks)-launched if gait_acks else None})
            if result['reason']!='goal_reached_kinematic' or not (result.get('stop') or {}).get('stop_acknowledged'):
                report['status']='stopped after segment; no retry';break
            # B starts a completely new task: fresh seed/grounding and3depth
            # median. No bbox or target ID is carried across stopped segments.
        else:report['status']='segment estimates completed; physical outcome requires operator observation'
    except BaseException as exc:
        report['status']='aborted';report['error']=repr(exc)
        raise
    finally:
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
