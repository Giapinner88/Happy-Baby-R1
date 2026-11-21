"""Validate a live proposal for bounded straight execution; no transport."""
import math


class ExecutionGate:
    def __init__(self, config, *, require_corridor=True):
        self.require_corridor=require_corridor
        self.config=config;self.start=None;self.origin=None;self.identity=None;self.last_time=None;self.vx=0.;self.active=False
        limits={'max_v_mps':.16,'max_accel_mps2':.1,'max_duration_s':2.5,'max_displacement_m':.1,'max_heading_error_rad':.05,'max_source_age_s':.6}
        for key,cap in limits.items():
            value=config[key]
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=cap:raise ValueError('invalid '+key)
        if config['gait_activation_mps']!=.15:raise ValueError('gait contract mismatch')

    def command(self,status,now):
        if not math.isfinite(now):raise ValueError('invalid local clock')
        if status['state'] not in ('APPROACH','ALIGN'):raise ValueError('controller '+status['state'])
        if self.require_corridor and status.get('declared_corridor_confirmed') is not True:raise ValueError('clear corridor not confirmed')
        if not 0<=status['source_age_s']<=self.config['max_source_age_s'] or now>=status['expires_at_local_monotonic_s']:raise ValueError('stale proposal')
        pose=status['pose_xyyaw'];goal=status['goal_xy_m'];proposal=status['proposal']
        if not all(math.isfinite(x) for x in [*pose,*goal,proposal['vx_mps'],proposal['wz_radps']]):raise ValueError('nonfinite geometry')
        heading=math.atan2(goal[1]-pose[1],goal[0]-pose[0])-pose[2]
        heading=math.atan2(math.sin(heading),math.cos(heading))
        if abs(heading)>self.config['max_heading_error_rad']:raise ValueError('turn required; yaw execution unverified')
        if self.start is None:
            self.start=now;self.origin=pose[:2];self.identity=status['target_id'];self.last_time=now
        if status['target_id']!=self.identity:raise ValueError('target changed')
        if not 0<=now-self.start<self.config['max_duration_s']:raise ValueError('bounded execution window complete')
        if math.dist(pose[:2],self.origin)>self.config['max_displacement_m']:raise ValueError('displacement guard')
        dt=now-self.last_time
        if not 0<=dt<=.1:raise ValueError('control gap')
        requested=proposal['vx_mps']
        if not 0<=requested<=self.config['max_v_mps']+1e-6:raise ValueError('proposal exceeds envelope')
        # Never boost a slow planner command to cross the gait threshold.
        if self.active and requested<=self.config['gait_activation_mps']:raise ValueError('planner below active gait envelope')
        vx=min(requested,self.config['max_v_mps'],self.vx+self.config['max_accel_mps2']*dt)
        self.active=self.active or vx>self.config['gait_activation_mps']
        self.last_time=now;self.vx=vx
        return {'vx_mps':vx,'wz_radps':0.}
