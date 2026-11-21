"""Segmented visual approach in pelvis coordinates; no DDS or motor transport.

Distances integrated here describe commands, never measured foot displacement.
Transition images retain capture time. Without calibrated motion compensation,
an image captured after zero command is required before starting another segment.
"""
import math
from ai_modules.vision.range_window import range_ready


class VisualSegments:
    def __init__(self, config, controller, gait):
        self.config=dict(config);self.controller=dict(controller);self.gait=dict(gait)
        limits={'segment_commanded_m':.3,'max_drive_s':5.,'recheck_timeout_s':3.,
                'max_total_commanded_m':3.,'max_duration_s':120.}
        for key,cap in limits.items():
            value=config.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=cap:
                raise ValueError('invalid visual segments '+key)
        # 'all_commands' (legacy): segment length includes the sub-threshold ramp.
        # 'above_gait_threshold': only command integral with vx > gait threshold counts.
        # Both are command integrals, never measured displacement.
        self.segment_basis=config.get('segment_basis','all_commands')
        if self.segment_basis not in ('all_commands','above_gait_threshold'):raise ValueError('invalid segment_basis')
        self.gait_threshold=gait.get('gait_threshold_mps',.15)
        if type(self.gait_threshold) not in (int,float) or not .1<=self.gait_threshold<.16:raise ValueError('invalid gait_threshold_mps')
        self.max_segments=config.get('max_segments')
        if self.max_segments is not None and (type(self.max_segments) is not int or not 1<=self.max_segments<=20):
            raise ValueError('invalid max_segments')
        for key,cap in [('max_v_mps',.16),('max_w_radps',.3),('max_accel_mps2',.1),
                        ('max_yaw_accel_radps2',.3),('max_source_age_s',.6)]:
            value=gait[key]
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=cap:
                raise ValueError('invalid visual segment envelope '+key)
        # Segment-only freshness limit; operator-approved 2026-11-19 for testing at 0.8 and 1.0 s (hard cap 1.0).
        # Falls back to the gait envelope (0.6). Cost: open-loop travel on an older image, <= v*age
        # (0.13 m at 0.8 s, 0.16 m at 1.0 s with v=0.16). No obstacle sensing in this flow.
        self.max_age=config.get('max_source_age_s',gait['max_source_age_s'])
        if type(self.max_age) not in (int,float) or not math.isfinite(self.max_age) or not 0<self.max_age<=1.:
            raise ValueError('invalid visual segments max_source_age_s')
        if not self.gait_threshold<gait['max_v_mps']<=.16:raise ValueError('visual segment gait threshold')
        self.phase='ACQUIRE';self.start=self.last_time=self.phase_start=None
        self.identity=None;self.measurement=None;self.last_stamp=None
        self.vx=self.wz=self.segment_m=self.total_m=0.
        self.segment_started=None;self.zero_since=None;self.segment_index=0
        self.transition_stamp=None;self.early_sequence=None;self.early_measurement=None;self.capture_epoch=0
        self.terminal=None;self.reset_segment_stats()
        # Opt-in 'look once' mode (operator-selected 2026-11-19, exploratory): one valid look at ACQUIRE sets a
        # bounded planned length; ADVANCE/BRAKE then ignore perception (no freshness, tracker or goal checks).
        # Still enforced: planned length, max_drive_s, max_total_commanded_m, duration, cancel. No obstacle sensing.
        self.look_once=bool(config.get('look_once',False));self.blind=False;self.plan_m=None
        self.look_once_fraction=config.get('look_once_fraction',1.);self.look_once_max_m=config.get('look_once_max_m',.3)
        # Ramp+brake also move the robot (0.116+0.124 m measured, ruler/total-command ~1.0 in 3 runs); not part of plan_m.
        self.look_once_overhead_m=config.get('look_once_overhead_m',0.);self.look_once_min_plan_m=.02
        # Plan from the median of K distinct fresh frames taken while stationary (first look scatters +-7-12 %).
        self.look_once_frames=config.get('look_once_frames',1);self.look_buf=[];self.look_buf_stamp=None
        # Turn in place toward the target before walking when |bearing| exceeds this tolerance (0 disables).
        # Yaw is a command integral (no yaw-displacement validation); each turn ends with a stationary re-look.
        self.align_tol=config.get('look_once_align_tolerance_rad',0.);self.align_target=None;self.yaw_done=0.;self.align_count=0
        # Look-once never aborts on perception after a first valid look: a lost tracker, perception error or missing
        # stationary frames switch to dead reckoning from the last look (remaining = look distance - commands since).
        self.look_distance=None;self.look_total=0.;self.vision_lost=None
        if self.look_once:
            if type(self.look_once_fraction) not in (int,float) or not 0<self.look_once_fraction<=1.:raise ValueError('invalid look_once_fraction')
            if type(self.look_once_max_m) not in (int,float) or not 0<self.look_once_max_m<=.5:raise ValueError('invalid look_once_max_m')
            if type(self.look_once_overhead_m) not in (int,float) or not 0<=self.look_once_overhead_m<=.4:raise ValueError('invalid look_once_overhead_m')
            if type(self.look_once_frames) is not int or not 1<=self.look_once_frames<=9:raise ValueError('invalid look_once_frames')
            if type(self.align_tol) not in (int,float) or not 0<=self.align_tol<=self.gait['arc_heading_rad']:raise ValueError('invalid look_once_align_tolerance_rad')

    def reset_segment_stats(self):
        self.stats={'ramp_s':0.,'ramp_commanded_m':0.,'above_gait_s':0.,'above_gait_commanded_m':0.,
                    'brake_s':0.,'brake_commanded_m':0.,'above_gait_ticks':0}

    def accumulate(self,dt):
        step=self.vx*dt
        if self.phase=='BRAKE':key='brake'
        elif self.phase!='ADVANCE' or self.vx==0:return
        elif self.vx>self.gait_threshold:key='above_gait';self.stats['above_gait_ticks']+=1
        else:key='ramp'
        self.stats[key+'_s']+=dt;self.stats[key+'_commanded_m']+=step

    def start_segment(self,now):
        self.transition('ADVANCE',now);self.segment_started=now;self.segment_m=0.;self.segment_index+=1
        self.reset_segment_stats()

    def collect_look(self,now,after=None):
        """Add the newest distinct fresh stationary measurement; return (median_distance, median_bearing) once K are held, else None."""
        m=self.measurement
        if m is not None and m['stamp']!=self.look_buf_stamp and 0<=now-m['stamp']<=self.max_age and (after is None or m['stamp']>after):
            x,y,_=m['point'];self.look_buf_stamp=m['stamp'];self.look_buf.append((math.hypot(x,y),math.atan2(y,x)))
        if len(self.look_buf)<self.look_once_frames:return None
        return self.median_buf()

    def median_buf(self):
        d=sorted(v[0] for v in self.look_buf);b=sorted(v[1] for v in self.look_buf);mid=len(d)//2
        return (d[mid],b[mid]) if len(d)%2 else ((d[mid-1]+d[mid])/2,(b[mid-1]+b[mid])/2)

    def begin_look_once(self,look,now):
        distance,bearing=look
        if self.align_tol>0 and abs(bearing)>self.align_tol and distance>self.controller['standoff_m']+self.controller['goal_tolerance_m']:
            if self.align_count>=3:
                if abs(bearing)>self.gait['arc_heading_rad']:self.terminal=('ABORTED','look_once alignment not converged');return
            else:
                self.look_buf=[];self.look_distance=distance;self.align_target=bearing;self.yaw_done=0.;self.align_count+=1
                self.blind=True;self.transition('ALIGN',now);return
        try:self.plan_m=self.plan_look_once(distance,bearing)
        except ValueError as exc:self.terminal=('ABORTED',str(exc));return
        self.look_buf=[];self.look_distance=distance;self.look_total=self.total_m
        if self.plan_m is None:self.terminal=('DONE','estimated_visual_goal')
        else:self.blind=True;self.start_segment(now)

    def dead_reckon(self,now,why):
        """Continue without vision from the last look; abort only if there never was one."""
        self.vision_lost=self.vision_lost or why;self.look_buf=[]
        if self.look_distance is None:self.terminal=('ABORTED',why);return
        remaining=self.look_distance-(self.total_m-self.look_total)
        plan=min(self.look_once_max_m,self.look_once_fraction*(remaining-self.controller['standoff_m'])-self.look_once_overhead_m)
        if plan<self.look_once_min_plan_m:self.terminal=('DONE','estimated_goal_dead_reckoned');return
        self.plan_m=plan;self.look_distance=remaining;self.look_total=self.total_m;self.blind=True;self.start_segment(now)

    def plan_look_once(self,distance,bearing):
        """Planned commanded length from one look; None means no motion is warranted."""
        if abs(bearing)>self.gait['arc_heading_rad']:raise ValueError('look_once target not aligned')
        plan=min(self.look_once_max_m,self.look_once_fraction*(distance-self.controller['standoff_m'])-self.look_once_overhead_m)
        return plan if plan>=self.look_once_min_plan_m else None

    def segment_progress(self):
        if self.segment_basis=='above_gait_threshold':return self.stats['above_gait_commanded_m']
        return self.segment_m

    def transition(self, phase, now):
        self.phase=phase;self.phase_start=now
        if phase in ('BRAKE','RECHECK'):
            self.capture_epoch+=1
        if phase=='BRAKE':
            self.transition_stamp=now;self.early_sequence=None;self.early_measurement=None
        if phase=='RECHECK':self.zero_since=now

    def observe(self,event,now):
        if not event:return
        if event.get('error'):raise ValueError(event['error'])
        track=event.get('tracker') or {};body=event.get('body_frame_proposal') or {}
        if track.get('status')=='lost_latched':raise ValueError('target_lost_latched')
        if not ((event.get('capture') or {}).get('ok') is True and track.get('status')=='tracked'
                and range_ready(event.get('temporal',{})) and body.get('status')=='diagnostic_proposal'
                and body.get('frame')=='pelvis_link'):
            return
        stamp=event.get('local_requested_monotonic_s')
        point=body.get('estimated_point_body_m')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):raise ValueError('invalid visual stamp')
        if not 0<=now-stamp<=self.max_age:return
        if not isinstance(point,(list,tuple)) or len(point)!=3 or not all(type(v) in (float,int) and math.isfinite(v) for v in point):
            raise ValueError('invalid visual body point')
        identity=track.get('target_id')
        if not isinstance(identity,str) or not identity:raise ValueError('missing visual target identity')
        if self.identity is not None and identity!=self.identity:raise ValueError('target identity changed')
        if self.last_stamp is not None and stamp<self.last_stamp:raise ValueError('out of order visual frame')
        if stamp==self.last_stamp:return
        self.identity=identity;self.last_stamp=stamp
        self.measurement={'stamp':stamp,'sequence':event.get('sequence'),'point':list(point)}
        if self.early_measurement is None and self.transition_stamp is not None and stamp>=self.transition_stamp and self.phase in ('BRAKE','RECHECK'):
            self.early_sequence=event.get('sequence')
            self.early_measurement=dict(self.measurement)

    def update(self,event,now,cancelled=False):
        if not math.isfinite(now):raise ValueError('invalid control clock')
        if self.start is None:self.start=self.last_time=self.phase_start=now
        dt=now-self.last_time
        if not 0<=dt<=(.25 if self.look_once else .1):raise ValueError('visual segment control gap')
        # This is an integral of previously emitted commands, not foot odometry.
        if self.vx>0:self.accumulate(dt)
        self.total_m+=self.vx*dt;self.segment_m+=self.vx*dt;self.last_time=now;self.yaw_done+=self.wz*dt
        reason='visual_segment_'+self.phase.lower()
        try:
            if not self.blind and not self.vision_lost:
                try:self.observe(event,now)
                except ValueError as exc:
                    if not self.look_once:raise
                    self.vision_lost=str(exc);self.measurement=None
            if cancelled:raise ValueError('cancelled')
            if now-self.start>=self.config['max_duration_s']:raise ValueError('visual segment time budget')
            if self.total_m>=self.config['max_total_commanded_m']:raise ValueError('commanded distance budget')
        except ValueError as exc:self.terminal=('ABORTED',str(exc))
        recovering=(event or {}).get('tracker') or {}
        if not self.terminal and not self.blind and recovering.get('status')=='recovering' and self.phase not in ('ACQUIRE','RECHECK'):
            self.vx=self.wz=0.;self.transition('RECHECK',now)
        fresh=self.measurement is not None and 0<=now-self.measurement['stamp']<=self.max_age
        distance=bearing=None
        if self.measurement:
            x,y,_=self.measurement['point'];distance=math.hypot(x,y);bearing=math.atan2(y,x)
        goal=fresh and distance<=self.controller['standoff_m']+self.controller['goal_tolerance_m']
        if not self.terminal:
            if self.phase=='ACQUIRE':
                look=self.collect_look(now) if self.look_once and not self.vision_lost else None
                if look is not None:self.begin_look_once(look,now)
                elif self.look_once and not self.terminal:
                    if self.vision_lost:self.dead_reckon(now,self.vision_lost)
                    elif now-self.phase_start>=self.config['recheck_timeout_s']:self.terminal=('ABORTED','initial visual measurement timeout')
                elif fresh:
                    if goal:self.terminal=('DONE','estimated_visual_goal')
                    else:self.start_segment(now)
                elif now-self.phase_start>=self.config['recheck_timeout_s']:self.terminal=('ABORTED','initial visual measurement timeout')
            elif self.phase=='ALIGN':
                remaining=abs(self.align_target)-abs(self.yaw_done)
                stop_yaw=self.wz*self.wz/(2*self.gait['max_yaw_accel_radps2'])
                if remaining<=stop_yaw or now-self.phase_start>=self.config['max_drive_s']:self.transition('BRAKE',now)
            elif self.phase=='ADVANCE':
                if self.blind:
                    if self.segment_progress()>=self.plan_m or now-self.segment_started>=self.config['max_drive_s']:self.transition('BRAKE',now)
                elif not fresh:
                    # Expired perception never renews moving commands. Reacquire at zero.
                    self.vx=self.wz=0.;self.transition('RECHECK',now);reason='visual_expired_zero_command'
                elif goal or self.segment_progress()>=self.config['segment_commanded_m'] or now-self.segment_started>=self.config['max_drive_s']:
                    self.transition('BRAKE',now)
            elif self.phase=='RECHECK':
                if self.max_segments is not None and self.segment_index>=self.max_segments:
                    self.terminal=('DONE','segment_trial_complete')
                elif self.look_once:
                    look=None if self.vision_lost else self.collect_look(now,after=self.zero_since)
                    if look is not None:self.begin_look_once(look,now)
                    elif self.vision_lost:self.dead_reckon(now,self.vision_lost)
                    elif now-self.phase_start>=self.config['recheck_timeout_s']:
                        if self.look_buf:self.look_once_frames_short=len(self.look_buf);self.begin_look_once(self.median_buf(),now)
                        else:self.dead_reckon(now,'visual recheck timeout')
                elif fresh and self.measurement['stamp']>self.zero_since:
                    if goal:self.terminal=('DONE','estimated_visual_goal')
                    else:self.start_segment(now)
                elif now-self.phase_start>=self.config['recheck_timeout_s']:self.terminal=('ABORTED','visual recheck timeout')
        if self.terminal:
            self.vx=self.wz=0.;state,reason=self.terminal
        elif self.phase=='ALIGN':
            state='ALIGN'
            self.vx=0.;target=math.copysign(self.gait['max_w_radps'],self.align_target)
            self.wz=self.slew(self.wz,target,self.gait['max_yaw_accel_radps2']*dt)
        elif self.phase=='ADVANCE':
            state='APPROACH'
            if self.blind:desired_v,desired_w=self.gait['max_v_mps'],0.  # blind segment: straight, no stale bearing
            else:
                aligned=abs(bearing)<=self.gait['arc_heading_rad']
                desired_v=self.gait['max_v_mps'] if aligned else 0.
                desired_w=max(-self.gait['max_w_radps'],min(self.gait['max_w_radps'],self.gait['heading_gain']*bearing))
            self.vx=self.slew(self.vx,desired_v,self.gait['max_accel_mps2']*dt)
            self.wz=self.slew(self.wz,desired_w,self.gait['max_yaw_accel_radps2']*dt)
        elif self.phase=='BRAKE':
            state='APPROACH'
            if not fresh and not self.blind:self.vx=self.wz=0.
            else:
                self.vx=self.slew(self.vx,0.,self.gait['max_accel_mps2']*dt)
                self.wz=self.slew(self.wz,0.,self.gait['max_yaw_accel_radps2']*dt)
            if abs(self.vx)<1e-8 and abs(self.wz)<1e-8:
                self.vx=self.wz=0.;self.transition('RECHECK',now);self.blind=False
        else:state='WAITING';self.vx=self.wz=0.
        if not self.terminal and reason!='visual_expired_zero_command':reason='visual_segment_'+self.phase.lower()
        if not self.terminal and recovering.get('status')=='recovering':reason='tracking_recovery_zero_command'
        return {'state':state,'reason':reason,'phase':self.phase,'proposal':{'vx_mps':self.vx,'wz_radps':self.wz},
                'hardware_authority':False,'navigation_ready':False,'physical_arrival_verified':False,
                'source_age_s':None if not self.measurement else now-self.measurement['stamp'],
                'measurement_stamp_monotonic_s':None if not self.measurement else self.measurement['stamp'],
                'target_id':self.identity,'visual_range_m':distance,'visual_bearing_rad':bearing,
                'segment_index':self.segment_index,'segment_commanded_m':self.segment_m,
                'total_commanded_m':self.total_m,'segment_basis':self.segment_basis,'freshness_limit_s':self.max_age,
                'gait_threshold_mps':self.gait_threshold,'look_once':self.look_once,'blind_advance':self.blind,'plan_m':self.plan_m,'look_once_overhead_m':self.look_once_overhead_m,'look_once_frames':self.look_once_frames,'look_distance_m':getattr(self,'look_distance',None),'align_target_rad':self.align_target,'yaw_commanded_rad':self.yaw_done,'align_count':self.align_count,'vision_lost':self.vision_lost,'segment_command_profile':dict(self.stats),'distance_source':'command_integral_not_measured_steps',
                'capture_epoch':self.capture_epoch,'transition_stamp_s':self.transition_stamp,
                'transition_image_sequence':self.early_sequence,'transition_measurement':self.early_measurement,
                'zero_command_since_s':self.zero_since,
                'stationary_verified':False,'position_source':'live_camera_body_point'}

    @staticmethod
    def slew(value,target,step):return max(value-step,min(value+step,target))
