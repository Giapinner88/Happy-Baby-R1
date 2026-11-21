"""Exploratory planar relay plant, not R1 dynamics or camera validation."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from ai_modules.navigation.approach import Approach, Config, Grid, Pose, Target
from ai_modules.navigation.gait_approach import GaitAwareApproach
from ai_modules.navigation.turn_approach import wrap


def simulate(config, bearing_deg, *, dt=.05, observation_period_s=.2,
             observation_delay_s=.2, actuator_lag_s=.15, yaw_bias_radps=0.):
    planner = Approach(Config(**config['planner']['controller']))
    gate = GaitAwareApproach(config['gait_approach'])
    bearing = math.radians(bearing_deg)
    target = (2*math.cos(bearing), 2*math.sin(bearing))
    rows = tuple((0,)*100 for _ in range(100))
    x = y = yaw = vx = wz = 0.
    moving = False
    history = []
    records = []
    measured = None
    next_sample = 0.
    reason = 'simulation_timeout'
    done_at = None
    fault = None
    for i in range(math.ceil(config['task_duration_s']/dt)):
        t = i*dt
        history.append((t, x, y, yaw))
        if t+1e-8 >= next_sample:
            available = [r for r in history if r[0] <= t-observation_delay_s+1e-8]
            if available:
                measured = available[-1]
            next_sample += observation_period_s
        if measured is None:
            continue
        stamp, mx, my, myaw = measured
        now = 10+t
        grid = Grid(rows, .1, (-5., -5.), 10+stamp)
        cmd = planner.step(now, Pose(mx,my,myaw,10+stamp),
                           Target('object',*target,10+stamp),grid,
                           metric_valid=True,authority=True)
        status = {'state':cmd.state, 'source_age_s':t-stamp,
                  'measurement_stamp_monotonic_s':10+stamp,
                  'expires_at_local_monotonic_s':min(cmd.expires_at,10+stamp+config['planner']['controller']['max_age_s']),
                  'pose_xyyaw':[mx,my,myaw], 'goal_xy_m':planner.goal,
                  'target_id':'object','proposal':asdict(cmd),
                  'declared_corridor_confirmed':True}
        if cmd.state == 'DONE':
            if done_at is None:
                done_at, reason = t, cmd.reason
            control = {'vx_mps':0.,'wz_radps':0.,'phase':'DONE'}
        elif cmd.state == 'ABORTED':
            reason = cmd.reason
            break
        else:
            try:
                control = gate.command(status,now)
            except ValueError as exc:
                reason = str(exc)
                fault = {'time_s':t,'source_age_s':t-stamp,'status':status}
                break
        cv, cw = control['vx_mps'], control['wz_radps']
        # Strict move threshold and stop hysteresis from policy metadata.
        if abs(cv) > .15 or abs(cw) > .15:
            moving = True
        elif abs(cv) <= .10 and abs(cw) <= .10:
            moving = False
        desired_v, desired_w = (cv, cw) if moving else (0., 0.)
        alpha = 1. if actuator_lag_s == 0 else min(1., dt/actuator_lag_s)
        vx += alpha*(desired_v-vx)
        wz += alpha*(desired_w-wz)
        actual_w = wz + (yaw_bias_radps if moving else 0.)
        records.append({'time_s':t,'x_m':x,'y_m':y,'yaw_rad':yaw,
                        'actual_vx_mps':vx,'actual_wz_radps':actual_w,
                        'gait_moving':moving,'source_age_s':t-stamp,**control})
        x += vx*math.cos(yaw)*dt
        y += vx*math.sin(yaw)*dt
        yaw = wrap(yaw+actual_w*dt)
        if done_at is not None and t-done_at >= 1.:
            break
    return records, {'bearing_deg':bearing_deg,'reason':reason,
                     'done':done_at is not None,'done_at_s':done_at,
                     'fault':fault,
                     'final_goal_error_m':math.dist((x,y),planner.goal) if planner.goal else None,
                     'final_target_distance_m':math.dist((x,y),target),
                     'max_path_m':gate.path,
                     'arc_ticks':sum(r['vx_mps']>.15 and abs(r['wz_radps'])>1e-6 for r in records),
                     'turn_ticks':sum(r['phase']=='TURN' for r in records)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--angles-deg',nargs='+',type=float,default=[-179,-90,-30,-3.44,0,3.44,30,90,179])
    p.add_argument('--observation-period-s',type=float,default=.2)
    p.add_argument('--observation-delay-s',type=float,default=.2)
    p.add_argument('--actuator-lag-s',type=float,default=.15)
    p.add_argument('--yaw-bias-radps',type=float,default=0.)
    p.add_argument('--dt-s',type=float,default=.05)
    a = p.parse_args()
    if not 0 < a.dt_s <= .1 or not a.dt_s <= a.observation_period_s <= .5 or not 0 <= a.observation_delay_s <= .5 or not 0 <= a.actuator_lag_s <= 1 or not math.isfinite(a.yaw_bias_radps):
        p.error('invalid plant timing/bias')
    if any(not math.isfinite(v) or abs(v)>180 for v in a.angles_deg):p.error('invalid bearing')
    config = json.loads(a.config.read_text())
    a.output_dir.mkdir(parents=True,exist_ok=False)
    (a.output_dir/'resolved_config.json').write_text(json.dumps(config,indent=2)+'\n')
    summary = []
    for n, angle in enumerate(a.angles_deg):
        records, result = simulate(config,angle,dt=a.dt_s,observation_period_s=a.observation_period_s,
                                   observation_delay_s=a.observation_delay_s,
                                   actuator_lag_s=a.actuator_lag_s,yaw_bias_radps=a.yaw_bias_radps)
        summary.append(result)
        keys = sorted({k for r in records for k in r})
        with (a.output_dir/('case_%02d.csv'%n)).open('w') as f:
            writer = csv.DictWriter(f,fieldnames=keys)
            writer.writeheader();writer.writerows(records)
    sources = [Path(__file__),ROOT/'ai_modules/navigation/gait_approach.py',ROOT/'ai_modules/navigation/approach.py',ROOT/'ai_modules/navigation/turn_approach.py']
    (a.output_dir/'summary.json').write_text(json.dumps({
        'model':__doc__,'python':platform.python_version(),
        'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'command':sys.argv,'plant':{'dt_s':a.dt_s,'observation_period_s':a.observation_period_s,
        'observation_delay_s':a.observation_delay_s,'actuator_lag_s':a.actuator_lag_s,'yaw_bias_radps':a.yaw_bias_radps},
        'source_sha256':{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
        'cases':summary},indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    if any(not r['done'] or r['final_goal_error_m']>config['planner']['controller']['goal_tolerance_m'] for r in summary):
        raise SystemExit('some cases failed; preserved all traces')


if __name__ == '__main__':
    main()
