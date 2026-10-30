"""Finite search orchestration. Angles in degrees, positive left; heading is unwrapped.
Backend state timestamps use the same monotonic clock as backend.now().
No motor publisher or cloud client in this module.
"""
import math


def validate_config(c):
    positive = ['head_limit_deg','usable_hfov_deg','max_observations','max_duration_s',
                'max_body_travel_deg','settle_samples','settle_timeout_s','body_command_timeout_s','poll_s',
                'state_max_age_s','max_observation_age_s','adapter_timeout_s','head_tolerance_deg','body_tolerance_deg',
                'waist_tolerance_deg','speed_tolerance_deg_s']
    for k in positive:
        if type(c[k]) not in (float,int) or not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError('invalid '+k)
    for k in ['max_observations','settle_samples']:
        if type(c[k]) is not int: raise ValueError(k+' must be integer')
    if type(c['enable_body']) is not bool: raise ValueError('enable_body must be bool')
    if type(c.get('fast_recenter_without_recapture',False)) is not bool:
        raise ValueError('fast_recenter_without_recapture must be bool')
    if type(c.get('batch_head_capture',False)) is not bool:
        raise ValueError('batch_head_capture must be bool')
    if type(c.get('recenter_attempts',2)) is not int or not 0<=c.get('recenter_attempts',2)<=4:
        raise ValueError('invalid recenter_attempts')
    step=c.get('recenter_step_deg',60)
    if type(step) not in (int,float) or not math.isfinite(step) or not 0<step<=90:raise ValueError('invalid recenter_step_deg')
    if not 0 <= c['overlap_fraction'] < 1 or not 0 <= c['confirm_iou_min'] <= 1:
        raise ValueError('invalid overlap/IoU')
    for k in ['head_yaws_deg','body_offsets_deg']:
        if not isinstance(c[k],list) or not c[k] or any(type(x) not in (int,float) or not math.isfinite(x) for x in c[k]):
            raise ValueError('invalid '+k)
    if c['head_yaws_deg'][0] != 0 or any(abs(x)>c['head_limit_deg'] for x in c['head_yaws_deg']):
        raise ValueError('head scan must start at zero and respect envelope')
    step=c['usable_hfov_deg']*(1-c['overlap_fraction'])
    if any(b-a>step for a,b in zip(sorted(c['head_yaws_deg']),sorted(c['head_yaws_deg'])[1:])):
        raise ValueError('head scan leaves gaps under configured FOV')


def iou(a,b):
    def area(x): return max(0,x[2]-x[0])*max(0,x[3]-x[1])
    intersection=area([max(a[0],b[0]),max(a[1],b[1]),min(a[2],b[2]),min(a[3],b[3])])
    return intersection/max(1e-12,area(a)+area(b)-intersection)


class Finished(Exception):
    def __init__(self,status,reason=None,action=None):
        self.status=status;self.reason=reason;self.action=action


class Search:
    """Backend: now/sleep/state, move_head/move_body/stop/rest_arms,
    observe(query), act(observation), cancelled(). observe returns selected/ask/not_found,
    observation_id, image_sha256, pose, and selected requires bbox+identity_key.
    act must re-use this exact observation for existing bbox/head/arm pipeline.
    """
    def __init__(self,backend,config,emit=lambda event:None):
        validate_config(config)
        self.b=backend; self.c=config; self.emit=emit; self.events=[]; self.views=[]
        self.seen=set(); self.count=0; self.capture_count=0; self.travel=0; self.start=backend.now()
        self.pending=None

    def log(self,state,**data):
        event={'time_s':self.b.now()-self.start,'state':state,**data}
        self.events.append(event);self.emit(event)

    def check(self):
        if self.b.cancelled(): raise InterruptedError('cancelled')
        if self.b.now()-self.start > self.c['max_duration_s']: raise TimeoutError('search deadline')

    def state(self):
        self.check();s=self.b.state()
        keys=['head_yaw_deg','body_heading_deg','head_speed_deg_s','body_speed_deg_s',
              'waist_yaw_deg','waist_speed_deg_s','time_s']
        if any(not math.isfinite(s[k]) for k in keys):raise ValueError('nonfinite state')
        age=self.b.now()-s['time_s']
        if not 0<=age<=self.c['state_max_age_s']:raise TimeoutError('stale state')
        return s

    def settle(self,head,body):
        begin=self.b.now();good=0;last=None;last_log=-float('inf');diagnostic=None
        while self.b.now()-begin<=self.c['settle_timeout_s']:
            s=self.state()
            # Repeated reads of the same packet do not count as fresh samples.
            if s['time_s'] != last:
                last=s['time_s']
                ok=(abs(s['head_yaw_deg']-head)<=self.c['head_tolerance_deg'] and
                    (body is None or abs(s['body_heading_deg']-body)<=self.c['body_tolerance_deg']) and
                    abs(s['waist_yaw_deg'])<=self.c['waist_tolerance_deg'] and
                    max(abs(s['head_speed_deg_s']),abs(s['body_speed_deg_s']),abs(s['waist_speed_deg_s']))<=self.c['speed_tolerance_deg_s'])
                good=good+1 if ok else 0
                diagnostic={'head_target_deg':head,'body_target_deg':body,
                    'head_actual_deg':s['head_yaw_deg'],'body_actual_deg':s['body_heading_deg'],
                    'head_error_deg':s['head_yaw_deg']-head,'body_error_deg':None if body is None else s['body_heading_deg']-body,
                    'head_speed_deg_s':s['head_speed_deg_s'],'body_speed_deg_s':s['body_speed_deg_s'],
                    'waist_yaw_deg':s['waist_yaw_deg'],'waist_speed_deg_s':s['waist_speed_deg_s'],
                    'consecutive_good_samples':good}
                if self.b.now()-last_log>=.5:
                    self.log('SETTLE_SAMPLE',**diagnostic);last_log=self.b.now()
                if good>=self.c['settle_samples']:return s
            self.b.sleep(self.c['poll_s'])
        self.log('SETTLE_TIMEOUT',last_sample=diagnostic,head_target_deg=head,body_target_deg=body)
        raise TimeoutError('motion did not settle; last sample='+str(diagnostic))

    def turn_body(self,target):
        """Send one bounded yaw goal, then use the stationary measured IMU yaw."""
        actual=self.state()['body_heading_deg']
        self.check()
        delta=target-actual
        command=actual+max(-180,min(180,delta))
        if self.travel+abs(command-actual)>self.c['max_body_travel_deg']:
            raise TimeoutError('body travel budget')
        self.log('BODY_TURN_COMMAND',from_heading_deg=actual,target_heading_deg=command)
        self.b.move_body(command)
        began=self.b.now();reached=False
        while self.b.now()-began<=self.c['body_command_timeout_s']:
            s=self.state()
            if abs(s['body_heading_deg']-command)<=4.5:
                reached=True;break
            self.b.sleep(self.c['poll_s'])
        self.b.stop()
        stationary=self.settle(0,None)
        new_actual=stationary['body_heading_deg']
        progress=abs(new_actual-actual)
        self.travel+=progress
        self.log('BODY_TURN_RESULT',requested_heading_deg=command,actual_heading_deg=new_actual,
                 measured_progress_deg=progress,reached_window=reached)
        if not reached or progress<1:
            raise TimeoutError('body turn did not make sufficient progress')
        return new_actual

    def capture_staged(self,head,body):
        self.settle(head,body);self.check()
        captured=self.b.capture_view();self.capture_count+=1
        pose=captured['pose'];stamp=captured['captured_at_s']
        if not all(math.isfinite(x) for x in (stamp,pose['time_s'],pose['head_yaw_deg'],pose['body_heading_deg'])):
            raise ValueError('nonfinite staged capture pose')
        if not 0<=self.b.now()-stamp<=self.c['max_observation_age_s']:
            raise ValueError('staged capture expired')
        if abs(stamp-pose['time_s'])>self.c['state_max_age_s']:
            raise ValueError('stale staged capture pose')
        if abs(pose['head_yaw_deg']-head)>self.c['head_tolerance_deg'] or abs(pose['body_heading_deg']-body)>self.c['body_tolerance_deg']:
            raise ValueError('staged capture differs from settled pose')
        self.log('CAPTURED_VIEW',head_yaw_deg=head,body_heading_deg=body,
                 image_sha256=captured.get('image_sha256'))
        return captured

    def observe(self,query,head,body,captured=None):
        if captured is None:self.settle(head,body)
        self.check()
        if self.count>=self.c['max_observations']:raise TimeoutError('observation budget')
        o=self.b.process_view(query,captured) if captured is not None else self.b.observe(query)
        self.count+=1;self.check()
        if o['observation_id'] in self.seen:raise ValueError('reused observation ID')
        self.seen.add(o['observation_id'])
        if not o.get('image_sha256'):raise ValueError('missing image hash')
        pose=o['pose'];captured_at=o['captured_at_s']
        if not all(math.isfinite(x) for x in [captured_at,pose['time_s'],pose['head_yaw_deg'],pose['body_heading_deg']]):
            raise ValueError('nonfinite observation pose')
        age=self.b.now()-captured_at
        if not 0<=age<=self.c['max_observation_age_s']:raise ValueError('expired observation')
        if abs(captured_at-pose['time_s'])>self.c['state_max_age_s']:raise ValueError('stale capture pose')
        if captured is not None and (o['image_sha256']!=captured.get('image_sha256') or
                                     abs(captured_at-captured['captured_at_s'])>self.c['state_max_age_s']):
            raise ValueError('processed view differs from staged capture')
        current=self.state()
        if (captured is None and abs(current['head_yaw_deg']-head)>self.c['head_tolerance_deg']) or abs(current['body_heading_deg']-body)>self.c['body_tolerance_deg']:
            raise ValueError('robot moved during perception')
        if abs(pose['head_yaw_deg']-head)>self.c['head_tolerance_deg'] or abs(pose['body_heading_deg']-body)>self.c['body_tolerance_deg']:
            raise ValueError('capture pose differs from settled pose')
        if o['decision'] not in ['selected','ask','not_found']:raise ValueError('invalid decision')
        if o['decision']=='selected':
            box=o['bbox']
            if len(box)!=4 or not all(math.isfinite(x) for x in box) or not (0<=box[0]<box[2]<=1 and 0<=box[1]<box[3]<=1) or not o.get('identity_key'):
                raise ValueError('invalid selected target')
        bearing=pose['head_yaw_deg']+pose['body_heading_deg']
        self.views.append({'observation_id':o['observation_id'],'bearing_deg':bearing,
                           'interval_deg':[bearing-self.c['usable_hfov_deg']/2,bearing+self.c['usable_hfov_deg']/2]})
        self.log('OBSERVATION',observation=o)
        return o

    def point_or_recenter(self,query,observation):
        for attempt in range(self.c.get('recenter_attempts',2)+1):
            self.check();self.log('LOOK_AND_POINT',observation_id=observation['observation_id'])
            try:return self.b.act(observation)
            except ValueError as exc:
                if 'unreachable_head' not in str(exc):raise
                self.log('TARGET_OUTSIDE_HEAD_RANGE',reason=str(exc))
            heading=observation.get('target_heading_deg')
            if heading is None:return None  # Legacy/replay adapter: retain scan behavior.
            if not self.c['enable_body']:
                raise Finished('needs_body_turn','Đã thấy vật nhưng cần bật xoay thân để hướng đầu và chỉ tay tới vật.')
            if attempt>=self.c.get('recenter_attempts',2):
                raise Finished('target_unreachable','Đã thấy vật nhưng chưa đưa được mục tiêu vào tầm đầu sau khi xoay thân.')
            if not math.isfinite(heading):raise ValueError('nonfinite target heading')
            body=self.state()['body_heading_deg']
            delta=(heading-body+180)%360-180
            step=self.c.get('recenter_step_deg',60)
            target=body+max(-step,min(step,delta))
            travel=abs(target-body)
            if travel<self.c['body_tolerance_deg']:
                raise Finished('target_unreachable','Hướng ngang đã phù hợp nhưng tư thế đầu vẫn ngoài tầm; cần kiểm tra độ cao vật hoặc mapping camera.')
            if self.travel+travel>self.c['max_body_travel_deg']:raise TimeoutError('body travel budget')
            self.b.rest_arms();self.b.move_head(0);self.settle(0,body)
            self.log('TARGET_RECENTER',attempt=attempt+1,target_heading_deg=target,source_observation_id=observation['observation_id'])
            actual=self.turn_body(target)
            if self.c.get('fast_recenter_without_recapture',False) and hasattr(self.b,'act_after_turn'):
                self.log('FAST_RECENTER_POINT',source_observation_id=observation['observation_id'],
                         measured_heading_deg=actual)
                try:return self.b.act_after_turn(observation,actual)
                except ValueError as exc:
                    if 'unreachable_head' not in str(exc):raise
                    self.log('FAST_RECENTER_NEEDS_MORE_TURN',reason=str(exc))
                    continue
            # The slower path acquires a new view after rotation.
            first=self.observe(query,0,actual)
            if first['decision']=='ask':raise Finished('needs_clarification',first.get('clarification'))
            if first['decision']!='selected':raise Finished('target_lost_after_recenter','Chưa thấy lại vật sau khi xoay thân; vui lòng mô tả lại mục tiêu.')
            second=self.observe(query,0,actual)
            if second['decision']=='ask':raise Finished('needs_clarification',second.get('clarification'))
            if second['decision']!='selected' or first['identity_key']!=second.get('identity_key') or iou(first['bbox'],second['bbox'])<self.c['confirm_iou_min']:
                raise Finished('target_lost_after_recenter','Chưa xác nhận ổn định mục tiêu sau khi xoay thân.')
            observation=second

    def run(self,query):
        status='failed';action=None;reason=None
        try:
            self.check();self.log('REST_ARMS');self.b.rest_arms();initial=self.state()['body_heading_deg'];body=initial
            offsets=[0]+(self.c['body_offsets_deg'] if self.c['enable_body'] else [])
            for offset in offsets:
                self.check();target=initial+offset
                if offset!=0:
                    self.log('CENTER_HEAD');self.b.move_head(0);self.settle(0,body)
                    travel=abs(target-body)
                    if self.travel+travel>self.c['max_body_travel_deg']:raise TimeoutError('body travel budget')
                    self.log('BODY_TURN',target_heading_deg=target)
                    body=self.turn_body(target)
                if self.c.get('batch_head_capture',False):
                    if not all(hasattr(self.b,name) for name in ('capture_view','process_view')):
                        raise ValueError('backend does not support staged capture')
                    staged=[]
                    for head in self.c['head_yaws_deg']:
                        self.log('HEAD_SCAN',target_yaw_deg=head);self.b.move_head(head)
                        staged.append((head,self.capture_staged(head,body)))
                    self.log('PROCESS_STAGED_VIEWS',capture_count=len(staged))
                    for head,captured in staged:
                        o=self.observe(query,head,body,captured)
                        if o['decision']=='ask':
                            self.pending=o if len(o.get('candidates',[]))==2 else None
                            raise Finished('needs_clarification',o.get('clarification'))
                        if o['decision']!='selected':continue
                        action=self.point_or_recenter(query,o)
                        if action is not None:raise Finished('found',action=action)
                    self.log('SCAN_ROUND_RESULT',body_offset_deg=offset,decision='not_found',
                             views_processed=len(staged))
                    continue
                for head in self.c['head_yaws_deg']:
                    self.log('HEAD_SCAN',target_yaw_deg=head);self.b.move_head(head)
                    o=self.observe(query,head,body)
                    if o['decision']=='ask':
                        self.pending=o if len(o.get('candidates',[]))==2 else None
                        raise Finished('needs_clarification',o.get('clarification'))
                    if o['decision']!='selected':continue
                    # Confirm from the same settled viewpoint: IoU is meaningful here.
                    # Do not compare pixel boxes across head/body rotations.
                    self.log('CONFIRM',candidate=o['observation_id'])
                    confirm=self.observe(query,head,body)
                    if confirm['decision']=='ask':raise Finished('needs_clarification',confirm.get('clarification'))
                    if confirm['decision']!='selected' or confirm['identity_key']!=o['identity_key'] or iou(confirm['bbox'],o['bbox'])<self.c['confirm_iou_min']:
                        self.log('CONFIRM_REJECTED');continue
                    action=self.point_or_recenter(query,confirm)
                    if action is None:continue
                    raise Finished('found',action=action)
                self.log('SCAN_ROUND_RESULT',body_offset_deg=offset,decision='not_found',
                         views_processed=len(self.c['head_yaws_deg']))
            status='not_found_in_observed_views'
        except Finished as done:status=done.status;reason=done.reason;action=done.action
        except InterruptedError as exc:status='cancelled';reason=str(exc)
        except Exception as exc:status='failed';reason=type(exc).__name__+': '+str(exc)
        finally:
            # Stop is attempted for every exit, including errors/cancel; never masks evidence.
            try:self.b.stop();self.log('STOP')
            except Exception as exc:
                self.log('STOP_FAILED',error=str(exc));status='failed';reason='stop failed: '+str(exc)
        return self.result(status,reason,action)

    def coverage_summary(self):
        intervals=[]
        for view in self.views:
            low,high=view['interval_deg'];width=high-low
            if width>=360:return {'observed_intervals_deg':[[0,360]],'unobserved_intervals_deg':[]}
            low=low%360;high=low+width
            intervals.extend([[low,min(high,360)]]+([[0,high-360]] if high>360 else []))
        merged=[]
        for low,high in sorted(intervals):
            if merged and low<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],high)
            else:merged.append([low,high])
        gaps=[];end=0
        for low,high in merged:
            if low>end:gaps.append([end,low])
            end=high
        if end<360:gaps.append([end,360])
        return {'observed_intervals_deg':merged,'unobserved_intervals_deg':gaps}

    def result(self,status,reason,action):
        return {'status':status,'reason':reason,'action':action,'observations':self.count,
                'pending_candidates':self.pending,
                'captures':self.capture_count,
                'body_travel_deg':self.travel,'coverage':self.views,'angular_coverage':self.coverage_summary(),
                'coverage_assumed_fov':True,'hardware_validated':False}
