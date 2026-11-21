"""Offline policy replay from saved measurements; no camera/GPU/cloud/motor."""
import argparse
import collections
import copy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path[:0]=[str(ROOT),str(Path(__file__).parent)]
from observation_policy import ObservationPolicy
from ai_modules.vision.range_window import RangeWindow
from ai_modules.navigation.target_odom import TargetOdom
from ai_modules.navigation.approach_shadow import ApproachShadow

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True);p.add_argument('--observations',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
a=p.parse_args();c=json.loads(a.config.read_text());a.output_dir.mkdir(parents=True,exist_ok=False)
raw=[json.loads(x) for x in a.observations.read_text().splitlines()]
window=RangeWindow(c['temporal']);odom=TargetOdom(c['odom']);rebuilt=[]
for old in raw:
    e=copy.deepcopy(old);stamp=e['local_requested_monotonic_s'];now=e['local_finished_monotonic_s']
    if e['capture'].get('ok') and (e.get('tracker') or {}).get('status')=='tracked':
        e['temporal']=window.update(stamp,now,(e.get('depth') or {}).get('range_m'))
        e['target_odom_proposal']=odom.register(e,(e['capture'].get('odom_sync') or {}).get('poses',[]))
    rebuilt.append(e)
outputs={}
for label,records in [('original',raw),('single_frame_recovery',rebuilt)]:
    engine=ApproachShadow(c['planner']);policy=ObservationPolicy(c['observation_recovery']);i=0;current=None;trace=[]
    now=records[0]['local_finished_monotonic_s'];end=records[-1]['local_finished_monotonic_s']+.55
    while now<=end:
        incoming=None
        while i<len(records) and records[i]['local_finished_monotonic_s']<=now:
            incoming=records[i];i+=1
            if label=='single_frame_recovery':current=policy.update(incoming,now)
            else:current=incoming
        if label=='single_frame_recovery':current=policy.update(None,now)
        result=engine.update(current,now)
        if label=='single_frame_recovery':result=policy.limit(result)
        result.update(replay_monotonic_s=now,observation_recovery=(current or {}).get('observation_recovery'))
        trace.append(result)
        if result['state'] in ('ABORTED','DONE'):break
        now+=.05
    (a.output_dir/(label+'.jsonl')).write_text(''.join(json.dumps(x)+'\n' for x in trace))
    outputs[label]={'states':dict(collections.Counter(x['state'] for x in trace)),
                    'last_reason':trace[-1]['reason'],'last_source_age_s':trace[-1].get('source_age_s'),
                    'held_ticks':sum(bool((x.get('observation_recovery') or {}).get('held_after_camera_timeout')) for x in trace)}
summary={'read_only':True,'motor_calls':0,'kind':'counterfactual policy replay; measured depth/pose retained; not hardware validation',
         'observations':str(a.observations),'command':sys.argv,'results':outputs}
(a.output_dir/'resolved_config.json').write_text(json.dumps(c,indent=2)+'\n')
(a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
