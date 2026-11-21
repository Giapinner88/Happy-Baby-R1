"""Measured-odom turn → fresh visual settle → bounded straight approach.

SI units, CCW-positive yaw, shortest wrapped bearing. No transport or arming.
Preview explicitly skips corridor authorization; hardware callers require it.
"""
import math
from ai_modules.navigation.execution_gate import ExecutionGate


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


class TurnThenApproach:
    def __init__(self,drive_config,turn_config,*,preview=False):
        self.config=dict(turn_config);self.drive_config=dict(drive_config);self.preview=preview
        self.drive=ExecutionGate(drive_config,require_corridor=not preview)
        limits={'max_w_radps':.15,'max_measured_yaw_rate_radps':.5,'max_yaw_accel_radps2':.3,'max_turn_duration_s':30.,
                'max_yaw_travel_rad':3.5,'max_turn_displacement_m':.1,'heading_tolerance_rad':.05,
                'settle_yaw_rate_radps':.05,'settle_min_s':1.,'heading_gain':2.}
        for key,cap in limits.items():
            v=self.config[key]
            if type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=cap:raise ValueError('invalid turn '+key)
        if not self.config['heading_tolerance_rad']<=drive_config['max_heading_error_rad']:raise ValueError('turn tolerance exceeds drive gate')
        if type(self.config['settle_samples']) is not int or not 3<=self.config['settle_samples']<=10:raise ValueError('settle samples3..10')
        self.phase='ORIENT';self.start=None;self.last_time=None;self.origin=None;self.identity=None
        self.previous_stamp=None;self.previous_yaw=None;self.travel=0.;self.wz=0.;self.good=0;self.settle_start=None
        self.last_fresh_stamp=None

    def command(self,status,now):
        if not math.isfinite(now):raise ValueError('invalid clock')
        if status['state'] not in ('APPROACH','ALIGN'):raise ValueError('controller '+status['state'])
        if not self.preview and status.get('declared_corridor_confirmed') is not True:raise ValueError('clear corridor not confirmed')
        age=status['source_age_s']
        if not math.isfinite(age) or not 0<=age<=self.drive_config['max_source_age_s'] or now>=status['expires_at_local_monotonic_s']:raise ValueError('stale orientation proposal')
        pose=status['pose_xyyaw'];goal=status['goal_xy_m'];stamp=status.get('measurement_stamp_monotonic_s',now-age)
        if not math.isfinite(stamp) or not 0<=now-stamp<=self.drive_config['max_source_age_s']:raise ValueError('stale orientation measurement')
        if len(pose)!=3 or len(goal)!=2:raise ValueError('invalid pose/goal dimensions')
        if not all(math.isfinite(v) for v in [*pose,*goal]):raise ValueError('nonfinite orientation geometry')
        heading=wrap(math.atan2(goal[1]-pose[1],goal[0]-pose[0])-pose[2])
        if self.start is None:
            self.start=now;self.origin=pose[:2];self.identity=status['target_id'];self.last_time=now
        if status['target_id']!=self.identity:raise ValueError('orientation target changed')
        dt=now-self.last_time
        if not 0<=dt<=.1:raise ValueError('orientation control gap')
        self.last_time=now
        if math.dist(self.origin,pose[:2])>self.drive_config['max_displacement_m']:raise ValueError('orientation displacement guard')
        fresh=self.previous_stamp is None or stamp>self.previous_stamp+1e-7
        if self.previous_stamp is not None and stamp<self.previous_stamp-1e-7:raise ValueError('orientation out-of-order measurement')
        yaw_rate=None
        if fresh:
            if self.previous_stamp is not None:
                delta=wrap(pose[2]-self.previous_yaw);self.travel+=abs(delta)
                yaw_rate=delta/(stamp-self.previous_stamp)
                if abs(yaw_rate)>self.config['max_measured_yaw_rate_radps']:raise ValueError('measured yaw rate guard')
            self.previous_stamp=stamp;self.previous_yaw=pose[2]
        if self.phase=='ALIGNED':return {'vx_mps':0.,'wz_radps':0.,'phase':'ALIGNED','heading_error_rad':heading}
        if self.phase=='FORWARD':
            # Forward gate preserves its original heading, duration and gait rules.
            out=self.drive.command(status,now)
            return dict(out,phase='FORWARD',heading_error_rad=heading,yaw_travel_rad=self.travel)
        c=self.config
        if now-self.start>=c['max_turn_duration_s']:raise ValueError('orientation timeout')
        if self.travel>c['max_yaw_travel_rad']:raise ValueError('orientation yaw travel guard')
        if math.dist(self.origin,pose[:2])>c['max_turn_displacement_m']:raise ValueError('turn translation guard')
        if abs(heading)>c['heading_tolerance_rad']:
            self.phase='ORIENT';self.good=0;self.settle_start=None;self.last_fresh_stamp=None
            desired=max(-c['max_w_radps'],min(c['max_w_radps'],c['heading_gain']*heading))
            self.wz=max(self.wz-c['max_yaw_accel_radps2']*dt,min(self.wz+c['max_yaw_accel_radps2']*dt,desired))
        else:
            if self.phase!='SETTLE':
                self.phase='SETTLE';self.settle_start=now;self.good=0;self.last_fresh_stamp=stamp
            self.wz=0.
            # Held or pre-stop images never count as new confirmation.
            if fresh and stamp>self.settle_start and stamp>self.last_fresh_stamp:
                self.last_fresh_stamp=stamp
                if yaw_rate is not None and abs(yaw_rate)<=c['settle_yaw_rate_radps']:self.good+=1
                else:self.good=0
            if self.good>=c['settle_samples'] and now-self.settle_start>=c['settle_min_s']:
                self.phase='ALIGNED' if c.get('orientation_only',False) else 'FORWARD';self.wz=0.
        return {'vx_mps':0.,'wz_radps':self.wz,'phase':self.phase,'heading_error_rad':heading,
                'settle_samples':self.good,'yaw_travel_rad':self.travel,'requires_fresh_after_turn':self.phase=='SETTLE'}
