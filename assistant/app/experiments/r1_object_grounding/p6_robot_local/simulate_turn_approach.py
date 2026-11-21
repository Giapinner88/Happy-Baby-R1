"""Ideal planar kinematic test, no physics/SDK/network or hardware readiness claim."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from ai_modules.navigation.turn_approach import TurnThenApproach,wrap
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
p.add_argument('--angles-deg',nargs='+',type=float,default=[-179,-90,-30,-3.44,0,3.44,30,90,179])
a=p.parse_args();c=json.loads(a.config.read_text());a.output_dir.mkdir(parents=True,exist_ok=False)
(a.output_dir/'resolved_config.json').write_text(json.dumps(c,indent=2)+'\n')
summary=[];dt=.05
for case,bearing_deg in enumerate(a.angles_deg):
    if not math.isfinite(bearing_deg) or abs(bearing_deg)>180:raise ValueError('angle must be within±180')
    bearing=math.radians(bearing_deg);g=TurnThenApproach(c['execution'],c['orientation'])
    x=y=yaw=0.;records=[];reason='simulation_timeout';aligned_at=None;settled_error=None
    for i in range(800):
        now=10+i*dt
        s={'state':'APPROACH','declared_corridor_confirmed':True,'source_age_s':0.,'measurement_stamp_monotonic_s':now,
           'expires_at_local_monotonic_s':now+.1,'pose_xyyaw':[x,y,yaw],
           'goal_xy_m':[math.cos(bearing),math.sin(bearing)],'target_id':'object','proposal':{'vx_mps':.16,'wz_radps':0.}}
        try:cmd=g.command(s,now)
        except ValueError as exc:reason=str(exc);break
        if cmd['phase']=='FORWARD' and aligned_at is None:
            aligned_at=now-10;settled_error=wrap(math.atan2(math.sin(bearing)-y,math.cos(bearing)-x)-yaw)
        records.append({'time_s':now-10,'x_m':x,'y_m':y,'yaw_rad':yaw,**cmd})
        x+=cmd['vx_mps']*math.cos(yaw)*dt;y+=cmd['vx_mps']*math.sin(yaw)*dt;yaw=wrap(yaw+cmd['wz_radps']*dt)
    keys=sorted(set(k for r in records for k in r))
    with (a.output_dir/('case_%02d.csv'%case)).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(records)
    result={'bearing_deg':bearing_deg,'aligned_at_s':aligned_at,'settled_heading_error_rad':settled_error,
            'last_reason':reason,'distance_m':math.hypot(x,y),'simulated_ticks':len(records),
            'simultaneous_turn_and_forward':any(r['vx_mps']>0 and r['wz_radps']!=0 for r in records)}
    if aligned_at is None or abs(settled_error)>.05 or result['simultaneous_turn_and_forward']:raise ValueError(str(result))
    summary.append(result)
meta={'read_only':True,'model':'ideal zero-latency planar integration, not dynamics or hardware validation','dt_s':dt,
      'command':sys.argv,'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),ROOT/'ai_modules/navigation/turn_approach.py',ROOT/'ai_modules/navigation/execution_gate.py']},'cases':summary}
(a.output_dir/'summary.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(summary))
