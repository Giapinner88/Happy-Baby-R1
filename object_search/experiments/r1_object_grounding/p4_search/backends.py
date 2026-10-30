"""Deterministic replay and process adapter; no robot connection by default."""
import json
import subprocess
import time
from pathlib import Path
import math


def rotate_point_into_heading(point,delta_heading_deg):
    """Transform an old waist-frame target into a new yaw-only body frame."""
    if len(point)!=3 or not all(math.isfinite(v) for v in point) or not math.isfinite(delta_heading_deg):
        raise ValueError('finite 3D point and heading required')
    angle=math.radians(delta_heading_deg)
    return [math.cos(angle)*point[0]+math.sin(angle)*point[1],
            -math.sin(angle)*point[0]+math.cos(angle)*point[1],point[2]]


class Replay:
    def __init__(self,scenario):
        self.scenario=scenario;self.t=0.;self.head=0.;self.body=0.;self.index=0;self.stops=0
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds
    def cancelled(self):return self.t>=self.scenario.get('cancel_at_s',float('inf'))
    def state(self):
        return {'time_s':self.t,'head_yaw_deg':self.head,'body_heading_deg':self.body,
                'head_speed_deg_s':0.,'body_speed_deg_s':0.,
                'waist_yaw_deg':8. if self.t<self.scenario.get('waist_unstable_until_s',-1) else 0.,
                'waist_speed_deg_s':4. if self.t<self.scenario.get('waist_unstable_until_s',-1) else 0.}
    def move_head(self,yaw):
        if not self.scenario.get('stuck_head'):self.head=yaw
    def move_body(self,heading):
        if not self.scenario.get('stuck_body'):self.body=heading
    def stop(self):self.stops+=1
    def rest_arms(self):pass
    def observe(self,query):
        records=self.scenario.get('observations',[])
        if self.index>=len(records):raise ValueError('replay exhausted; missing observation')
        record=dict(records[self.index]);self.index+=1
        expected=record.pop('expected_pose',None)
        if expected and any(abs(self.state()[k]-v)>1e-6 for k,v in expected.items()):
            raise ValueError('replay view does not match requested pose')
        return {'observation_id':'replay_'+str(self.index),'image_sha256':'synthetic_replay_no_image',
                'pose':self.state(),'captured_at_s':self.now(),**record}
    def capture_view(self):
        return {'pose':self.state(),'captured_at_s':self.now(),
                'image_sha256':'synthetic_replay_no_image'}
    def process_view(self,query,captured):
        records=self.scenario.get('observations',[])
        if self.index>=len(records):raise ValueError('replay exhausted; missing observation')
        record=dict(records[self.index]);self.index+=1
        expected=record.pop('expected_pose',None)
        if expected and any(abs(captured['pose'][k]-v)>1e-6 for k,v in expected.items()):
            raise ValueError('replay view does not match captured pose')
        return {'observation_id':'replay_'+str(self.index),'image_sha256':captured['image_sha256'],
                'pose':captured['pose'],'captured_at_s':captured['captured_at_s'],**record}
    def act(self,observation):
        return {'status':'replay_action_requested','observation_id':observation['observation_id'],
                'identity_key':observation['identity_key'],'motor_commands_sent':False}


class ProcessBackend:
    """One JSON request stdin/response stdout per process invocation.
    External adapter owns feedback control, image capture and pipeline execution.
    argv is a list; shell=False. A failed RPC ends search, then attempts stop RPC.
    """
    def __init__(self,argv,cancel_file=None,timeout_s=30):
        if not argv or not all(isinstance(x,str) for x in argv):raise ValueError('adapter argv required')
        self.argv=argv;self.cancel_file=Path(cancel_file) if cancel_file else None;self.timeout=timeout_s
    def rpc(self,method,**params):
        p=subprocess.run(self.argv,input=json.dumps({'method':method,'params':params}),
                         text=True,capture_output=True,timeout=self.timeout,check=True)
        result=json.loads(p.stdout)
        if result.get('error'):raise RuntimeError(str(result['error']))
        return result['result']
    def now(self):return time.monotonic()
    def sleep(self,s):time.sleep(s)
    def cancelled(self):return self.cancel_file is not None and self.cancel_file.exists()
    def state(self):return self.rpc('state')
    def move_head(self,yaw):return self.rpc('move_head',yaw_deg=yaw)
    def move_body(self,heading):return self.rpc('move_body',heading_deg=heading)
    def rest_arms(self):return self.rpc('rest_arms')
    def observe(self,query):return self.rpc('observe',query=query)
    def act(self,o):return self.rpc('act',observation=o)
    def stop(self):return self.rpc('stop')


class FeedbackReplay(Replay):
    """Kinematic feedback fixture only; does not model locomotion or balance."""
    def __init__(self,scenario):
        super().__init__(scenario)
        self.head_goal=0.;self.body_goal=0.;self.hspeed=0.;self.bspeed=0.
    def move_head(self,yaw):self.head_goal=yaw
    def move_body(self,heading):self.body_goal=heading
    def stop(self):
        super().stop();self.body_goal=self.body;self.bspeed=0.
    def state(self):
        s=super().state();s.update(head_speed_deg_s=self.hspeed,body_speed_deg_s=self.bspeed);return s
    def sleep(self,seconds):
        from feedback import yaw_rate
        self.hspeed=0. if self.scenario.get('stuck_head') else yaw_rate(self.head_goal,self.head,5,35,.05)
        self.bspeed=0. if self.scenario.get('stuck_body') else yaw_rate(self.body_goal,self.body,2,15,.05)
        self.head+=self.hspeed*seconds;self.body+=self.bspeed*seconds;self.t+=seconds


class PipelineBackend:
    """Real vision orchestration around an explicitly supplied motion/capture adapter.
    Adapter capture returns a LOCAL capture_dir plus pose and captured_at_s in
    workstation monotonic clock. No movement or cloud calls during construction.
    """
    def __init__(self,transport,output_dir,pipeline_args=None):
        self.transport=transport;self.out=Path(output_dir);self.index=0
        self.args=pipeline_args or [];self.plans={}
    def __getattr__(self,name):return getattr(self.transport,name)
    def capture_view(self):
        captured=self.transport.rpc('capture')
        directory=Path(captured['capture_dir']).resolve()
        capture=json.loads((directory/'capture.json').read_text())
        captured['image_sha256']=capture['image_sha256']
        return captured
    def observe(self,query):
        return self.process_view(query,self.capture_view())
    def process_view(self,query,captured):
        import sys
        directory=Path(captured['capture_dir']).resolve()
        self.index+=1;out=self.out/('view_%03d'%self.index)
        script=Path(__file__).resolve().parents[2]/'r1_target_control/pipeline.py'
        cmd=[sys.executable,str(script),'--capture-dir',str(directory),
             '--query',query,'--output-dir',str(out),*self.args]
        try:
            process=subprocess.run(cmd,text=True,capture_output=True,timeout=120)
        except subprocess.TimeoutExpired:
            raise TimeoutError('vision pipeline timeout') from None
        (self.out/('view_%03d.log'%self.index)).write_text(process.stdout+process.stderr)
        if process.returncode:raise RuntimeError('vision pipeline failed; inspect view log')
        proposal=json.loads((out/'bbox/result.json').read_text())
        observation=json.loads((out/'bbox/observation.json').read_text())
        # Link adapter pose to the same image consumed by the vision pipeline.
        capture=json.loads((directory/'capture.json').read_text())
        if capture['image_sha256']!=captured.get('image_sha256'):
            raise ValueError('staged capture changed before perception')
        if observation['source_sha256']!=capture['image_sha256']:
            raise ValueError('capture changed during perception')
        q=capture['state_near_rpc_midpoint_proxy']['q_rad']
        if abs(math_degrees(q['head_yaw_joint'])-captured['pose']['head_yaw_deg'])>1:
            raise ValueError('adapter pose differs from capture encoder')
        result={'decision':proposal['decision'],'observation_id':proposal['observation_id'],
                'image_sha256':observation['source_sha256'],'pose':captured['pose'],
                'captured_at_s':captured['captured_at_s']}
        p=proposal['proposal']
        if proposal['decision']=='selected':
            box=p['bbox'];result.update(identity_key=query,
                bbox=[box[n]/1000 for n in ['x_min','y_min','x_max','y_max']])
            geometry=json.loads((out/'geometry.json').read_text())
            point=geometry['details']['approximate_target_waist_xyz_m']
            import math
            # Geometry already includes captured head pose and camera mounting.
            # Adding head yaw here again would double-count it.
            result['target_heading_deg']=captured['pose']['body_heading_deg']+math.degrees(math.atan2(point[1],point[0]))
            self.plans[result['observation_id']]=out
        else:
            result['clarification']=p.get('clarification')
            if proposal['decision']=='ask' and len(p.get('candidates',[]))==2:
                result['candidates']=p['candidates']
                result['capture_dir']=str(directory)
                result['bbox_run']=str(out/'bbox')
        return result

    def resume_candidate(self,prior,choice,query):
        """Plan one saved ambiguous box using the original capture, with no API call."""
        import sys
        candidate=prior['candidates'][choice]
        directory=Path(prior['capture_dir']).resolve()
        old_bbox=Path(prior['bbox_run']).resolve()
        if not (directory/'capture.json').is_file() or not old_bbox.is_dir():
            raise ValueError('cached capture unavailable')
        capture=json.loads((directory/'capture.json').read_text())
        if capture['image_sha256']!=prior['image_sha256']:
            raise ValueError('cached image hash differs')
        age=self.now()-prior['captured_at_s']
        if not 0<=age<=60:
            raise ValueError('cached capture expired')
        state=self.state()
        if abs(state['body_heading_deg']-prior['pose']['body_heading_deg'])>5:
            raise ValueError('robot heading changed since ambiguous capture')
        proposal={'decision':'selected','label':candidate['label'],'bbox':candidate['bbox'],
                  'clarification':None,'candidates':[]}
        proposal_path=self.out/'cached_selection.json'
        proposal_path.write_text(json.dumps(proposal,ensure_ascii=False,indent=2)+'\n')
        bbox=self.out/'cached_bbox'
        script=Path(__file__).resolve().parents[1]/'p2_bbox_baseline/run.py'
        cfg=json.loads((old_bbox/'resolved_config.json').read_text())
        # The original resolved config records the exact calibration and model settings.
        saved_cfg=self.out/'cached_bbox_config.json'
        saved_cfg.write_text(json.dumps({k:cfg[k] for k in ('model','reasoning_effort','image_detail','max_output_tokens','timeout_s','calibration')}))
        cmd=[sys.executable,str(script),'--image',str(directory/'frame.jpg'),
             '--query',query,'--config',str(saved_cfg),'--proposal-json',str(proposal_path),
             '--output-dir',str(bbox)]
        process=subprocess.run(cmd,text=True,capture_output=True,timeout=30)
        (self.out/'cached_bbox.log').write_text(process.stdout+process.stderr)
        if process.returncode:raise RuntimeError('cached bbox replay failed')
        out=self.out/'cached_view'
        pipeline=Path(__file__).resolve().parents[2]/'r1_target_control/pipeline.py'
        cmd=[sys.executable,str(pipeline),'--capture-dir',str(directory),'--bbox-run',str(bbox),
             '--query',query,'--output-dir',str(out),*self.args]
        process=subprocess.run(cmd,text=True,capture_output=True,timeout=60)
        (self.out/'cached_pipeline.log').write_text(process.stdout+process.stderr)
        if process.returncode:raise RuntimeError('cached target planning failed')
        result=json.loads((bbox/'result.json').read_text())
        geometry=json.loads((out/'geometry.json').read_text())
        point=geometry['details']['approximate_target_waist_xyz_m']
        observation={**prior,'decision':'selected','observation_id':result['observation_id'],
            'bbox':[candidate['bbox'][k]/1000 for k in ('x_min','y_min','x_max','y_max')],
            'identity_key':query,
            'target_heading_deg':prior['pose']['body_heading_deg']+math.degrees(math.atan2(point[1],point[0]))}
        self.plans[observation['observation_id']]=out
        return observation
    def act(self,observation):
        out=self.plans[observation['observation_id']]
        summary=json.loads((out/'pipeline.json').read_text())
        if summary['status']!='planned':raise ValueError('no reachable coordinated target: '+summary['status'])
        target=json.loads((out/'target.json').read_text())
        if target['observation_id']!=observation['observation_id']:raise ValueError('target observation mismatch')
        result=self.transport.rpc('execute_target',target=target)
        if not isinstance(result,dict) or result.get('status') in ['failed','cancelled']:
            raise RuntimeError('target execution failed')
        return result

    def act_after_turn(self,observation,actual_heading_deg):
        """Replan from the old image ray after a measured body turn, without another API call."""
        import sys
        from pathlib import Path
        out=self.plans[observation['observation_id']]
        geometry=json.loads((out/'geometry.json').read_text())
        old=geometry['details']['approximate_target_waist_xyz_m']
        old_heading=observation['pose']['body_heading_deg']
        point=rotate_point_into_heading(old,actual_heading_deg-old_heading)
        root=Path(__file__).resolve().parents[2]/'r1_target_control'
        if str(root) not in sys.path:sys.path.insert(0,str(root))
        from control import point_arm,limits,validate_target
        from vision_geometry import solve_head_point,preferred_arm
        def option(flag,default):
            return Path(self.args[self.args.index(flag)+1]) if flag in self.args else default
        runtime=option('--runtime-root',Path(__file__).resolve().parents[3]/'third_party/Happy-Baby-R1-develop')
        urdf=runtime/'teleop/src/assets/R1.urdf'
        control=json.loads((root/'config.json').read_text())
        vision=json.loads(option('--vision-config',root/'vision_config.json').read_text())
        capture=json.loads((out/'capture/capture.json').read_text())
        pitch=capture['state_near_rpc_midpoint_proxy']['q_rad']['head_pitch_joint']
        head=solve_head_point(urdf,point,[pitch,0.],control,vision)
        if not head['solved']:raise ValueError('unreachable_head after body turn')
        side,_=preferred_arm(head['joints_rad']['head_yaw_joint'],vision['arm_side_deadband_deg'])
        arm=point_arm(urdf,side,point,control)
        if not arm['solved']:raise ValueError('unreachable_arm after body turn')
        joints={**head['joints_rad'],**arm['joints_rad']}
        validate_target(joints,limits(urdf,control))
        original=json.loads((out/'bbox/result.json').read_text())
        target={'schema':'r1_target_control.v1','kind':'vision_head_and_arm_approx',
                'joints_rad':joints,'policy_sha256':control['policy_sha256'],
                'observation_id':observation['observation_id'],
                'source_image_sha256':observation['image_sha256'],
                'target_id':original['target']['target_id'],
                'selected_arm':side,'preferred_arm':side,
                'replanned_from_old_view':True,'body_heading_at_capture_deg':old_heading,
                'body_heading_after_turn_deg':actual_heading_deg,
                'approximate_target_waist_xyz_m':point,
                'range_is_measured':False,'collision_checked':False,'balance_validated':False}
        (out/'target_after_turn.json').write_text(json.dumps(target,ensure_ascii=False,indent=2)+'\n')
        result=self.transport.rpc('execute_target',target=target)
        if not isinstance(result,dict) or result.get('status') in ('failed','cancelled'):
            raise RuntimeError('target execution failed after body turn')
        return result


def math_degrees(value):
    import math
    return math.degrees(value)
