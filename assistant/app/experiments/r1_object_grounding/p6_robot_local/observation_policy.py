"""Bounded hold for timeout/skipped old input; no retimestamp or extrapolation."""
import math
from ai_modules.vision.range_window import range_ready


class ObservationPolicy:
    def __init__(self,config):
        self.config=dict(config);self.good=None;self.current=None;self.held=False;self.failure=None;self.hold_reason=None
        c=self.config
        if not 0<c['slow_after_s']<c['max_age_s']<=.6:raise ValueError('invalid observation age limits')
        if not .15<c['slow_floor_mps']<=c['max_v_mps']<=.16:raise ValueError('invalid gait-aware speed limits')

    def update(self,incoming,now):
        if incoming is not None:
            stamp=incoming.get('local_requested_monotonic_s',float('nan'))
            age=now-stamp
            clock_ok=math.isfinite(age) and 0<=age<=self.config['max_age_s']
            timeout=((incoming.get('capture') or {}).get('ok') is False
                     and (incoming.get('capture') or {}).get('code')==3104 and incoming.get('tracker') is None
                     and incoming.get('depth') is None and incoming.get('body_frame_proposal') is None)
            skipped=((incoming.get('capture') or {}).get('ok') is True
                     and incoming.get('temporal',{}).get('status')=='stale_input_skipped'
                     and incoming.get('tracker') is None and incoming.get('depth') is None
                     and incoming.get('body_frame_proposal') is None)
            usable=(clock_ok and (incoming.get('capture') or {}).get('ok') is True
                    and ((incoming.get('tracker') or {}).get('status')=='tracked' or
                         (incoming.get('capture',{}).get('event')=='odom' and incoming.get('temporal',{}).get('status')=='pinned_target_odometry'))
                    and range_ready(incoming.get('temporal',{}))
                    and (incoming.get('target_odom_proposal') or {}).get('status')=='diagnostic_registered')
            old_age=now-self.good['local_requested_monotonic_s'] if self.good else float('inf')
            if usable:
                duplicate=(self.good and stamp==self.good['local_requested_monotonic_s']
                           and incoming.get('capture',{}).get('event')=='odom'
                           and incoming.get('target_odom_proposal',{}).get('held_duplicate_odom') is True)
                if duplicate:
                    self.current=self.good;self.held=True;self.failure=incoming.get('sequence');self.hold_reason='duplicate_odom'
                elif self.good and stamp<=self.good['local_requested_monotonic_s']:
                    self.good=None;self.current={'local_requested_monotonic_s':stamp,'temporal':{'status':'out_of_order'}}
                else:self.good=incoming;self.current=incoming
                if not duplicate:self.held=False;self.failure=None;self.hold_reason=None
            elif (timeout or skipped) and clock_ok and self.good and 0<=old_age:
                self.current=self.good;self.held=True;self.failure=incoming.get('sequence')
                self.hold_reason='camera_timeout' if timeout else 'stale_input_skipped'
            else:
                self.current=incoming;self.good=None;self.held=False;self.failure=None;self.hold_reason=None
        if self.current is None:return None
        result=dict(self.current)
        result['observation_recovery']={'held_after_camera_timeout':self.held and self.hold_reason=='camera_timeout',
                                       'held_after_stale_input':self.held and self.hold_reason=='stale_input_skipped','failed_sequence':self.failure,
                                       'held_after_duplicate_odom':self.held and self.hold_reason=='duplicate_odom',
                                       'measurement_age_s':now-self.current['local_requested_monotonic_s'],
                                       'expired':now-self.current['local_requested_monotonic_s']>self.config['max_age_s']}
        return result

    def limit(self,status):
        result=dict(status);age=result.get('source_age_s');c=self.config
        result['age_speed_limit_mps']=None
        if result['state'] in ('APPROACH','ALIGN') and age is not None and age>c['slow_after_s']:
            fraction=max(0.,min(1.,(age-c['slow_after_s'])/(c['max_age_s']-c['slow_after_s'])))
            cap=c['max_v_mps']-fraction*(c['max_v_mps']-c['slow_floor_mps'])
            proposal=dict(result['proposal']);proposal['vx_mps']=min(proposal['vx_mps'],cap)
            result.update(proposal=proposal,age_speed_limit_mps=cap)
        return result
