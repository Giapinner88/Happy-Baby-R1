"""Live-observation approach proposals only, no motor transport or authority.

Local monotonic clock for freshness; remote vendor odom coordinates for pose.
Declared map is in a task frame anchored at the first valid robot pose.
"""
from ai_modules.vision.range_window import range_ready
from dataclasses import asdict
import math
from ai_modules.navigation.approach import Approach, Config, Grid, Pose, Target


class ApproachShadow:
    def __init__(self, config):
        self.config=config
        self.engine=Approach(Config(**config['controller']))
        self.anchor=None

    def update(self, event, now, cancelled=False):
        result={'mode':'shadow','hardware_authority':False,'navigation_ready':False,
                'hardware_command':{'vx_mps':0.,'wz_radps':0.},
                'state':'WAITING','reason':'waiting_for_fresh_target','proposal':{'vx_mps':0.,'wz_radps':0.},
                'path_xy_m':[],'goal_xy_m':None,'pose_xyyaw':None,'target_xy_m':None,
                'expires_at_local_monotonic_s':now}
        if cancelled:
            self.engine._stop(now,'cancelled')
        if event and (event.get('temporal',{}).get('status')=='lost_latched' or
                      (event.get('tracker') or {}).get('status')=='lost_latched'):
            self.engine._stop(now,'target_lost_latched')
        if self.engine.state in ('ABORTED','DONE'):
            result.update(state=self.engine.state,reason=self.engine.reason)
            return result
        try:
            if not event or 'local_requested_monotonic_s' not in event:
                raise ValueError('perception_unavailable')
            stamp=event['local_requested_monotonic_s'];age=now-stamp
            result['source_age_s']=age
            if not math.isfinite(age) or not 0<=age<=self.engine.config.max_age_s:
                raise ValueError('perception_stale_or_future')
            if not range_ready(event.get('temporal',{})):
                raise ValueError('observation_'+event.get('temporal',{}).get('status','unavailable'))
            p=event.get('target_odom_proposal') or {}
            if p.get('status')!='diagnostic_registered':raise ValueError('odom_'+p.get('reason','unavailable'))
            pose=p['robot_pose_xyyaw'];target=p['pinned_point_odom_m'][:2]
            if not all(math.isfinite(v) for v in [*pose,*target]):raise ValueError('invalid_geometry')
            if self.anchor is None:self.anchor=list(pose)
            ox,oy,yaw=self.anchor;c,s=math.cos(yaw),math.sin(yaw)
            def local(x,y):return (c*(x-ox)+s*(y-oy),-s*(x-ox)+c*(y-oy))
            xy=local(*pose[:2]);txy=local(*target)
            m=self.config['corridor'];width,height=m['width'],m['height']
            if type(width) is not int or type(height) is not int or not 1<=width*height<=10000:raise ValueError('invalid map size')
            rows=[[-1]*width for _ in range(height)]
            for x0,y0,x1,y1 in m['declared_free_cells']:
                if not 0<=x0<x1<=width or not 0<=y0<y1<=height:raise ValueError('invalid corridor')
                for j in range(y0,y1):
                    for i in range(x0,x1):rows[j][i]=0
            # Target/nearby support exclusion is declared, not detected obstacle coverage.
            for j in range(height):
                for i in range(width):
                    x=m['origin_m'][0]+(i+.5)*m['resolution_m'];y=m['origin_m'][1]+(j+.5)*m['resolution_m']
                    if math.hypot(x-txy[0],y-txy[1])<=m['target_exclusion_radius_m']:rows[j][i]=1
            grid=Grid(tuple(tuple(r) for r in rows),m['resolution_m'],tuple(m['origin_m']),stamp)
            cmd=self.engine.step(now,Pose(*xy,pose[2]-yaw,stamp),Target(p['target_id'],*txy,stamp),grid,
                                 metric_valid=True,authority=True)
            # The flags above exercise diagnostic controller math only. There is
            # intentionally no hardware output path in this module.
            result.update(state=cmd.state,reason=cmd.reason,proposal=asdict(cmd),
                          expires_at_local_monotonic_s=min(stamp+self.engine.config.max_age_s,cmd.expires_at),
                          path_xy_m=list(self.engine.path),goal_xy_m=self.engine.goal,
                          pose_xyyaw=[*xy,pose[2]-yaw],target_xy_m=list(txy),
                          target_id=p['target_id'],corridor=m,
                          declared_corridor_confirmed=m['operator_confirmed'],
                          geometry_validation='skipped/exploratory; proposal only')
        except (KeyError,TypeError,ValueError) as exc:
            if self.anchor is not None:
                self.engine._stop(now,str(exc));result.update(state='ABORTED',reason=str(exc))
            else:result['reason']=str(exc)
        return result
