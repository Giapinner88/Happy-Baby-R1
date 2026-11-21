"""Robot-local single-task voice/vision/approach service. Shadow by default."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import collections
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from ai_modules.navigation.latest_slot import LatestSlot
from ai_modules.vision.camera_shadow_source import read_packet


def pack(header,jpeg):
    data=json.dumps(header).encode();return struct.pack('!I',len(data))+data+jpeg


def unpack(value):
    size=struct.unpack('!I',value[:4])[0]
    return json.loads(value[4:4+size]),value[4+size:]


def attach_pose(header,history):
    # Never borrow another image's pose or change its original capture time.
    match=next((h for h in history if h.get('sequence')==header.get('sequence')
                and h.get('remote_requested_monotonic_s')==header.get('remote_requested_monotonic_s')),None)
    result=dict(header)
    if match:
        for key in ('pose_sync','odom_sync','metadata_wait_s'):
            if key in match:result[key]=match[key]
    return result


def stale_input(stamp,now,max_age_s):
    """Drop old work before decode/tracking/DAC, without changing its clock."""
    if not math.isfinite(stamp) or not math.isfinite(now) or now<stamp:
        raise ValueError('invalid input capture clock')
    return now-stamp>max_age_s


def atomic(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,allow_nan=False)+'\n');temp.replace(path)


def handoff_zero(command,controller):
    """Owner Hello establishes a new slew clock; first command must be zero."""
    controller.vx=0.
    if hasattr(controller,'wz'):controller.wz=0.
    return dict(command,vx_mps=0.,wz_radps=0.)


class Sensor:
    def __init__(self,config,slot,poses,cancel,log,target_ready=None,capture_now=None):
        self.capture_now=capture_now
        self.target_ready=target_ready
        self.config=config;self.slot=slot;self.poses=poses;self.cancel=cancel;self.error=None;self.history=[]
        cmd=['/usr/bin/python3','-u',str(ROOT/config['sensor_worker']),'--interface',config['interface'],'--rpc-timeout-s',str(config['capture_rpc_timeout_s']),'--split-pose','--state-enabled']
        if not config.get('visual_segments',{}).get('enabled'):
            cmd+=['--odom-enabled','--odom-topic','rt/lf/odommodestate']
        self.child=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log)
        try:
            ready,_=read_packet(self.child.stdout.fileno(),5.)
            if ready.get('event')!='ready':raise ValueError('sensor handshake')
        except BaseException:
            self.child.terminate();self.child.wait(timeout=1)
            self.child.stdin.close();self.child.stdout.close();raise
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def loop(self):
        timeout_streak=0;timeouts_total=0;previous_image_received=None
        try:
            while not self.cancel.is_set():
                priority=bool(getattr(self,'capture_now',None) is not None and self.capture_now.is_set())
                if priority:self.capture_now.clear()
                odom_only=bool(self.config.get('approach_mode')=='initial_target_then_odom' and self.target_ready is not None and self.target_ready.is_set())
                started=time.monotonic();self.child.stdin.write(b'odom\n' if odom_only else b'capture\n');self.child.stdin.flush()
                header,jpeg=read_packet(self.child.stdout.fileno(),1.)
                received=time.monotonic()
                header['local_requested_monotonic_s']=started
                header['transition_priority_capture']=priority
                timeout=not odom_only and header.get('ok') is False and header.get('code')==3104
                timeout_streak=timeout_streak+1 if timeout else 0
                timeouts_total+=int(timeout)
                header.update(camera_timeout_streak=timeout_streak,camera_timeouts_total=timeouts_total,
                              local_received_monotonic_s=received,request_to_receive_s=received-started)
                if header.get('event')=='image' and header.get('ok'):
                    header['image_receive_gap_s']=None if previous_image_received is None else received-previous_image_received
                    previous_image_received=received
                self.slot.publish(pack(header,jpeg))
                if header.get("event")=="image":
                    metadata,_=read_packet(self.child.stdout.fileno(),1.)
                    if metadata.get("event")!="pose" or metadata.get("sequence")!=header.get("sequence"):
                        raise ValueError("sensor metadata sequence mismatch")
                    metadata['metadata_wait_s']=time.monotonic()-received
                    self.history=(self.history+[metadata])[-4:]
                    self.poses.publish(json.dumps(self.history).encode())
                hz=self.config.get('odom_hz',20.) if odom_only else capture_hz(self.config,self.target_ready is not None and self.target_ready.is_set())
                # Only the first timeout in an outage gets an immediate retry.
                # Requests remain serial; persistent failure returns to normal cadence.
                retry=timeout_streak==1 and self.config.get('capture_retry_on_timeout',True)
                delay=0 if retry else max(0,1/hz-(time.monotonic()-started))
                if getattr(self,'capture_now',None) is None:self.cancel.wait(delay)
                else:
                    deadline=time.monotonic()+delay
                    while not self.cancel.is_set() and not self.capture_now.is_set():
                        remaining=deadline-time.monotonic()
                        if remaining<=0:break
                        self.cancel.wait(min(.01,remaining))
        except Exception as exc:self.error=repr(exc)

    def close(self):
        self.cancel.set();self.thread.join(timeout=1.2)
        if self.child.poll() is None:self.child.terminate()
        try:self.child.wait(timeout=1)
        except subprocess.TimeoutExpired:self.child.kill();self.child.wait()
        self.child.stdin.close();self.child.stdout.close()


def run_branches(track,depth,pool=None,*,skip_depth=False):
    """Join one frame before admitting the next; no pending-frame FIFO."""
    def timed(call):
        start=time.perf_counter();result=call()
        return result,time.perf_counter()-start
    start=time.perf_counter()
    future=pool.submit(timed,track) if pool and not skip_depth else None
    if future is None:tracked,tracking_s=timed(track)
    result=None;depth_s=None
    if not skip_depth and (future is not None or tracked[1] is not None):result,depth_s=timed(depth)
    if future is not None:tracked,tracking_s=future.result()
    return tracked,result,{'tracking_s':tracking_s,'dac_pipeline_s':depth_s,'branches_wall_s':time.perf_counter()-start}


def capture_hz(config,pinned):
    return config.get('tracking_camera_hz',config['camera_hz']) if pinned else config['camera_hz']


def wait_seed_frame(frames,cancel,timeout_s=2.,max_age_s=.2):
    """Allow transient SDK timeouts during startup, without accepting stale images."""
    deadline=time.monotonic()+timeout_s;previous=0
    while not cancel.is_set():
        remaining=deadline-time.monotonic()
        if remaining<=0:raise TimeoutError('no valid camera seed before deadline')
        sample=frames.read(previous,min(.1,remaining))
        if sample is None:continue
        previous,payload=sample;header,jpeg=unpack(payload)
        if header.get('ok'):
            if stale_input(header['local_requested_monotonic_s'],time.monotonic(),max_age_s):continue
            return header,jpeg
        if header.get('code')!=3104:raise ValueError('camera capture failed: '+str(header.get('error',header.get('code'))))
    raise ValueError('cancelled during camera startup')


def matched_pose(header,poses,cancel,timeout_s=.12):
    deadline=time.monotonic()+timeout_s;previous=0
    while not cancel.is_set():
        value=poses.read(previous,min(.005,max(0,deadline-time.monotonic())))
        if value:
            previous=value[0];result=attach_pose(header,json.loads(value[1]))
            if 'pose_sync' in result and 'odom_sync' in result:return result
        if time.monotonic()>=deadline:return header
    return header


OWNER_STATUS='/run/hb/status.env'


def read_owner_status(path=OWNER_STATUS):
    """Key=value snapshot of the owner's status file (diagnostic only; never gates motion)."""
    try:
        values={}
        for line in Path(path).read_text().splitlines():
            key,sep,value=line.partition('=')
            if sep:values[key.strip()]=value.strip()
        return values
    except Exception as exc:
        return {'error':type(exc).__name__}


def perception(config,frames,poses,observations,cancel,seed_path,seed_box,ready,task_id,target_ready=None):
    # Dedicated process: GPU work and Python image processing cannot block control.
    os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
    import cv2
    import numpy as np
    from dac_runtime import Geometry,TensorRT
    from ai_modules.vision.feature_target import FeatureTarget
    from ai_modules.vision.color_target import masked_range
    from ai_modules.vision.range_window import RangeWindow
    from ai_modules.vision.camera_frames import CameraFrames
    from ai_modules.navigation.target_odom import TargetOdom
    cv2.setNumThreads(1);model=None;tracking_pool=None
    try:
        geometry=Geometry(ROOT/config['model_dir']/'geometry.npz',config['preprocess_backend'])
        model=TensorRT(ROOT/config['model_dir']/'dac.engine')
        warm=np.load(ROOT/config['model_dir']/'reference_input.npy')
        for _ in range(config['gpu_warmup_runs']):
            if cancel.is_set():return
            model.infer(warm)
        ready.set()
        seed=None
        while not cancel.is_set() and seed is None:seed=seed_box.read(timeout=.1)
        if seed is None:return
        box=json.loads(seed[1])
        tracker=FeatureTarget(config['tracker'],cv2.imread(str(seed_path)),box)
        if config['parallel_tracking']:tracking_pool=ThreadPoolExecutor(max_workers=1)
        window=RangeWindow(config['temporal']);body=CameraFrames(config['body'],ROOT);odom=TargetOdom(config['odom'])
        previous=0;next_depth_at=0.
        recent=collections.deque(maxlen=8);lost_dumped=False   # last JPEGs, saved once if the tracker is lost (diagnosis)
        while not cancel.is_set():
            if time.monotonic()<next_depth_at:
                cancel.wait(min(.1,next_depth_at-time.monotonic()));continue
            value=frames.read(previous,.1)
            if value is None:continue
            sequence,payload=value;header,jpeg=unpack(payload);stamp=header['local_requested_monotonic_s']
            event={'task_id':task_id,'sequence':sequence,'dropped_input_frames':max(0,sequence-previous-1),'local_requested_monotonic_s':stamp,'capture':header,'tracker':None,'depth':None,'body_frame_proposal':None}
            previous=sequence
            if config.get('approach_mode')=='initial_target_then_odom' and odom.pinned is not None and header.get('event')!='odom':
                continue  # Discard a final image already in flight when pin completed.
            if header.get('event')=='odom':
                if config.get('approach_mode')!='initial_target_then_odom':raise ValueError('unexpected odometry-only packet')
                event=odometry_event(odom,header,task_id,sequence,time.monotonic())
                stamp=event['local_requested_monotonic_s'];temporal=event['temporal']
            elif not header.get('ok'):
                temporal=({'status':'capture_timeout','range_m':None,'samples':len(window.values),'navigation_ready':False}
                          if header.get('code')==3104 else window.clear('capture_failed'))
            elif stale_input(stamp,time.monotonic(),config.get('max_input_age_s',config['tracker']['max_age_s'])):
                # Keep the previous valid observation while a fresh image arrives.
                # Do not call the tracker: stale input would latch target loss.
                temporal={'status':'stale_input_skipped','navigation_ready':False}
            else:
                decode_started=time.perf_counter()
                event['input_age_s']=time.monotonic()-stamp
                image=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
                event['decode_s']=time.perf_counter()-decode_started
                if image is None:raise ValueError('invalid camera JPEG')
                stage=time.perf_counter();depth_cycle_started=time.monotonic()
                # Both branches read this same immutable decoded frame.
                def depth_frame():
                    x,_=geometry.prepare(image);prediction=model.infer(x)
                    return geometry.restore(prediction)
                tracking_only=bool(config.get('tracking_only_after_pin',False) and odom.pinned is not None)
                (track,mask),depth_result,timing=run_branches(
                    lambda:tracker.observe(image,stamp,time.monotonic()),depth_frame,tracking_pool,skip_depth=tracking_only)
                event.update(tracker=track,**timing)
                recent.append((sequence,stamp,jpeg))
                if track.get('status')=='lost' and not lost_dumped:
                    lost_dumped=True
                    try:
                        folder=Path(seed_path).parent/'tracker_lost';folder.mkdir(exist_ok=True)
                        for number,time_s,data in recent:(folder/('seq%05d_%.3f.jpg'%(number,time_s))).write_bytes(data)
                        (folder/'tracker.json').write_text(json.dumps({'sequence':sequence,'tracker':track,'frames':[n for n,_,_ in recent]},default=str))
                    except Exception:pass   # diagnostics must never affect control
                if depth_result is not None:ranges,valid=depth_result
                if mask is None:temporal=window.clear(track.get('reason',track['status']))
                elif tracking_only:
                    header=matched_pose(header,poses,cancel);event['capture']=header
                    event['body_frame_proposal']=body.pose_only(header.get('pose_sync'))
                    temporal={'status':'tracked_pinned_target','measurement_mode':'tracking_only','has_pinned_target':True,
                              'depth_measured':False,'navigation_ready':False}
                    event.update(depth_computed=False,inference_total_s=time.perf_counter()-stage)
                    next_depth_at=0.
                else:
                    t=time.monotonic()
                    metadata=poses.read()
                    header=attach_pose(header,json.loads(metadata[1]) if metadata else [])
                    event['capture']=header
                    estimate=masked_range(ranges,valid,mask,config['tracker']['min_depth_valid_fraction']);event['depth']=estimate
                    temporal=window.update(stamp,time.monotonic(),estimate['range_m'])
                    next_depth_at=depth_cycle_started+config['min_depth_interval_s']
                    support=mask & valid & np.isfinite(ranges)
                    if estimate['status']=='estimated':
                        point=np.median(geometry.rays[support]*ranges[support,None],axis=0).tolist()
                        event['raw_point_camera_optical_m']=point;event['body_frame_proposal']=body.project(point,header.get('pose_sync'))
                    event['postprocess_s']=time.monotonic()-t
                    event['inference_total_s']=time.perf_counter()-stage
            finished=time.monotonic();event.update(temporal=temporal,request_to_publish_s=finished-stamp,local_finished_monotonic_s=finished)
            if finished-stamp>config['temporal']['max_age_s']:event['temporal']=window.clear('processing_deadline')
            if header.get('event')!='odom' and not config.get('visual_segments',{}).get('enabled'):
                event['target_odom_proposal']=odom.register(event,(header.get('odom_sync') or {}).get('poses',[]))
            if target_ready is not None and odom.pinned is not None:target_ready.set()
            event.update(local_finished_monotonic_s=time.monotonic(),request_to_publish_s=time.monotonic()-stamp)
            observations.publish(json.dumps(event,allow_nan=False).encode())
    except BaseException as exc:
        observations.publish(json.dumps({'task_id':task_id,'error':repr(exc)}).encode())
    finally:
        if tracking_pool is not None:tracking_pool.shutdown(wait=False)
        if model is not None:model.close()


def odometry_event(odom,header,task_id,sequence,now):
    sample=header.get('odom_sample') or {}
    registration=odom.follow_pin(sample,now) if header.get('ok') is True else {'status':'unavailable','reason':'odometry source unavailable'}
    ok=registration['status']=='diagnostic_registered'
    return {'task_id':task_id,'sequence':sequence,'capture':header,'tracker':None,'depth':None,
            'body_frame_proposal':None,'local_requested_monotonic_s':sample.get('stamp',now),
            'temporal':{'status':'pinned_target_odometry' if ok else 'invalid_odometry',
                        'measurement_mode':'odometry_only','has_pinned_target':odom.pinned is not None,
                        'visual_revalidation':False},'target_odom_proposal':registration,
            'position_source':'initial_pin_and_fresh_odom','initial_target_visual_stamp_s':odom.initial[-1]['stamp'] if odom.initial else None}


def wait_model_ready(worker,ready,cancel,timeout_s,observations):
    deadline=time.monotonic()+timeout_s
    while not ready.wait(.05):
        if cancel.is_set():raise ValueError('cancelled during model startup')
        value=observations.read()
        if value:
            event=json.loads(value[1])
            if event.get('error'):raise ValueError(event['error'])
        if not worker.is_alive():raise ValueError('GPU worker stopped before ready')
        if time.monotonic()>=deadline:raise TimeoutError('GPU ready deadline')
    if cancel.is_set():raise ValueError('cancelled during model startup')
    if not worker.is_alive():raise ValueError('GPU worker stopped at ready')


class APV:
    """Robot-local APV1, no SSH or external-clock renewal."""
    def __init__(self,release_after_stop=False):
        import socket,secrets
        self.release_after_stop=release_after_stop
        from bounded_trial import owner_ready
        self.ready=owner_ready;self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.connect(('127.0.0.1',5573));self.socket.settimeout(.025);self.nonce=secrets.randbits(63)+1;self.sequence=0;self.nonzero_sent_count=0;self.forward_sent_count=0;self.yaw_sent_count=0
        try:self.send(4);self.send(1)
        except BaseException as exc:
            exc.apv_cleanup=self.close();raise

    def send(self,kind,vx=0.,wz=0.):
        if kind not in (3,5):self.ready()
        wire=struct.Struct('<IHHQQQff');self.sequence+=1;now=time.monotonic_ns()//1000
        self.socket.send(wire.pack(0x31565041,1,kind,self.nonce,self.sequence,now+100000,vx,wz))
        if kind==2:
            self.nonzero_sent_count=getattr(self,'nonzero_sent_count',0)+int(vx>0 or wz!=0)
            self.forward_sent_count=getattr(self,'forward_sent_count',0)+int(vx>0)
            self.yaw_sent_count=getattr(self,'yaw_sent_count',0)+int(wz!=0)
        data=self.socket.recv(128)
        if len(data)!=40:raise ValueError('owner ACK size')
        magic,version,status,nonce,sequence,stamp,rv,rw=wire.unpack(data)
        if (magic,version,status,nonce,sequence)!=(0x31565041,1,{1:1,2:2,3:3,4:4,5:5}[kind],self.nonce,self.sequence) or not math.isfinite(rv) or not math.isfinite(rw) or abs(rv-vx)>1e-6 or abs(rw-wz)>1e-6:
            raise ValueError('owner ACK rejected: status=%s requested=(%.6f,%.6f) echoed=(%.6f,%.6f)'%(status,vx,wz,rv,rw))
        return {'kind':kind,'vx_mps':vx,'wz_radps':wz,'status':status,'received_monotonic_s':time.monotonic()}

    def close(self):
        result={'stop_acknowledged':False}
        try:result['ack']=self.send(3);result['stop_acknowledged']=True
        except Exception as exc:result['error']=repr(exc)
        if getattr(self,'release_after_stop',False):
            result['release_acknowledged']=False
            if result['stop_acknowledged']:
                try:result['release_ack']=self.send(5);result['release_acknowledged']=True
                except Exception as exc:result['release_error']=repr(exc)
        self.socket.close();return result


class Manager:
    def __init__(self,config):
        self.config=config;self.lock=threading.Lock();self.active=False;self.cancel=None
        self.status={'state':'IDLE','mode':'execute' if config['execute'] else 'shadow','answer':'Chưa có tác vụ.'}
        self.root=ROOT/config['runs_dir'];self.root.mkdir(parents=True,exist_ok=True)
        self.marker=Path(config['active_marker']);self.marker.parent.mkdir(parents=True,exist_ok=True)
        self.marker.unlink(missing_ok=True) if sys.version_info>=(3,8) else None

    def submit(self,payload):
        query=payload.get('query','')
        if not isinstance(query,str) or not 1<=len(query.strip())<=200:return 400,{'status':'invalid_query'}
        with self.lock:
            if self.active:return 409,{'status':'busy','answer':'Robot đang xử lý một tác vụ, không xếp hàng yêu cầu mới.'}
            from voice_tools import claim_camera
            try:self.camera_lease=claim_camera(self.config['camera_lock'])
            except BlockingIOError:return 409,{'status':'busy','answer':'Camera đang được tác vụ khác sử dụng.'}
            self.active=True;self.cancel=mp.get_context('spawn').Event();task_id=uuid.uuid4().hex
            self.status={'state':'STARTING','task_id':task_id,'mode':'execute' if self.config['execute'] else 'shadow','query':query}
            self.marker.write_text(task_id+'\n')
        self.task_thread=threading.Thread(target=self.run,args=(task_id,query,payload),daemon=True)
        self.task_thread.start()
        return 202,{'status':'accepted','task_id':task_id,'mode':self.status['mode'],'answer':'Đã nhận yêu cầu.' if self.config['execute'] else 'Đang quan sát mục tiêu ở chế độ shadow, robot chưa đi.'}

    def snapshot(self):
        with self.lock:return dict(self.status,active=self.active)

    def run(self,task_id,query,payload):
        from ai_modules.navigation.approach_shadow import ApproachShadow
        from ai_modules.navigation.execution_gate import ExecutionGate
        from ai_modules.navigation.turn_approach import TurnThenApproach
        from ai_modules.vision.openai_grounding import build_request,query_openai,extract_proposal,pixel_box
        ctx=mp.get_context('spawn');frames=LatestSlot(5*1024*1024,ctx);poses=LatestSlot(65536,ctx);observations=LatestSlot(65536,ctx);seed_box=LatestSlot(4096,ctx);ready=ctx.Event();target_ready=ctx.Event();capture_now=ctx.Event()
        hybrid=None;capture_epoch=0
        if self.config.get('visual_segments',{}).get('enabled'):
            from ai_modules.navigation.visual_segments import VisualSegments
            hybrid=VisualSegments(self.config['visual_segments'],self.config['planner']['controller'],self.config['gait_approach'])
        out=self.root/task_id;sensor=None;worker=None;apv=None;commands=0;reason='task_timeout';stop_result=None;log=None;result_state='STOPPED';answer=None
        try:
            out.mkdir();atomic(out/'resolved_config.json',self.config);atomic(out/'request.json',{'task_id':task_id,'query':query})
            log=(out/'sensor.log').open('w')
            seed=out/'seed.jpg';startup=time.monotonic()
            worker=ctx.Process(target=perception,args=(self.config,frames,poses,observations,self.cancel,str(seed),seed_box,ready,task_id,target_ready));worker.start()
            wait_model_ready(worker,ready,self.cancel,self.config['gpu_ready_timeout_s'],observations)
            atomic(out/'gpu_ready.json',{'ready':True,'warmup_runs':self.config['gpu_warmup_runs'],'startup_s':time.monotonic()-startup,'ready_monotonic_s':time.monotonic()})
            sensor=Sensor(self.config,frames,poses,self.cancel,log,target_ready,capture_now)
            header,jpeg=wait_seed_frame(frames,self.cancel,max_age_s=self.config.get('max_input_age_s',self.config['tracker']['max_age_s']))
            import cv2
            import numpy as np
            seed=out/'seed.jpg';seed.write_bytes(jpeg)
            image=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('invalid seed')
            # Explicit test seed is CLI-only, unavailable through voice HTTP.
            box=payload.get('_test_bbox')
            if box is None:
                request=build_request(jpeg,query,(image.shape[1],image.shape[0]),self.config['grounding'])
                proposal=extract_proposal(query_openai(request,self.config['grounding_timeout_s']))
                atomic(out/'grounding.json',proposal)
                if proposal.get('decision')!='selected':
                    result_state='NEEDS_CLARIFICATION' if proposal.get('decision')=='ask' else 'NOT_FOUND'
                    answer=proposal.get('clarification') or 'Chưa tìm thấy mục tiêu rõ ràng trong ảnh hiện tại.'
                    raise ValueError('grounding '+proposal.get('decision','unavailable'))
                box=pixel_box(proposal['bbox'],image.shape[1],image.shape[0])['xyxy']
            if self.cancel.is_set():raise ValueError('cancelled')
            seed_box.publish(json.dumps(box).encode())
            engine=ApproachShadow(self.config['planner']);gate=ExecutionGate(self.config['execution'])
            orient=None if hybrid is not None else build_navigation(self.config)
            from observation_policy import ObservationPolicy
            recovery=ObservationPolicy(self.config['observation_recovery'])
            previous=0;event=None;began=time.monotonic();next_tick=began
            # Look-once: perception faults hand control to dead reckoning in VisualSegments instead of ending the task.
            look_once=hybrid is not None and hybrid.look_once;perception_fault_sent=False;worker_fault=False
            owner_trace=collections.deque(maxlen=60);trace_tick=0
            with (out/'control.jsonl').open('w') as trace, (out/'observations.jsonl').open('w') as diagnostic:
                while time.monotonic()-began<self.config['task_duration_s']:
                    now=time.monotonic();value=observations.read(previous);incoming=None
                    if value is not None:
                        previous,data=value;incoming=json.loads(data)
                        diagnostic.write(json.dumps(incoming)+'\n')
                        if incoming.get('task_id')!=task_id:raise ValueError('old task result')
                        if incoming.get('error') and not look_once:raise ValueError(incoming['error'])
                    if sensor.error:
                        if not look_once:raise ValueError(sensor.error)
                        if not perception_fault_sent:incoming={'error':'sensor: '+str(sensor.error)};perception_fault_sent=True
                    if worker_fault and not perception_fault_sent:incoming={'error':'inference worker stopped'};perception_fault_sent=True
                    event=recovery.update(incoming,now)
                    computed=time.perf_counter()
                    status=hybrid.update(incoming,now,self.cancel.is_set()) if hybrid is not None else recovery.limit(engine.update(event,now,self.cancel.is_set()))
                    if hybrid is not None and status['capture_epoch']!=capture_epoch:
                        capture_epoch=status['capture_epoch'];capture_now.set()
                    status.update(task_id=task_id,mode='execute' if self.config['execute'] else 'shadow',ack=None,
                                  control_compute_s=time.perf_counter()-computed,
                                  measurement_stamp_monotonic_s=status.get('measurement_stamp_monotonic_s',(event or {}).get('local_requested_monotonic_s')),
                                  perception_sequence=(event or {}).get('sequence'),
                                  inference_total_s=(event or {}).get('inference_total_s'),
                                  request_to_publish_s=(event or {}).get('request_to_publish_s'),
                                  camera_rpc_s=((event or {}).get('capture') or {}).get('rpc_latency_s'),
                                  dropped_input_frames=(event or {}).get('dropped_input_frames'),
                                  observation_recovery=(event or {}).get('observation_recovery'),
                                  observation_status=((event or {}).get('temporal') or {}).get('status'))
                    command=None
                    if hybrid is not None:
                        command=dict(status['proposal'],phase=status['phase'])
                        status.update(visual_segments=True,navigation_strategy='visual_segments',orientation=command)
                    if orient is not None and status['state'] in ('APPROACH','ALIGN'):
                        command=orient.command(status,time.monotonic())
                        status['orientation']=command
                        status['navigation_strategy']=self.config.get('navigation_strategy','turn_then_approach')
                        if command['phase']=='ALIGNED':
                            reason='orientation_only_complete';result_state='ORIENTATION_COMPLETE'
                            trace.write(json.dumps(status)+'\n');break
                    if self.config['execute'] and (status['state'] in ('APPROACH','ALIGN') or (hybrid is not None and apv is not None and status['state']=='WAITING')):
                        if not self.config['clear_corridor_confirmed']:raise ValueError('operator corridor confirmation missing')
                        if hybrid is not None and self.config['planner']['corridor']['operator_confirmed'] is not True:raise ValueError('visual segments corridor not confirmed')
                        if command is None:command=gate.command(status,time.monotonic())
                        permission=self.config['gait_approach'] if self.config.get('navigation_strategy')=='gait_aware_arc' else self.config['orientation']
                        if hybrid is not None and not self.config['gait_approach']['execute_enabled']:raise ValueError('visual segments execution disabled')
                        if orient is not None and not permission['execute_enabled']:raise ValueError('navigation execution disabled pending hardware validation')
                        if apv is None:
                            apv=APV(release_after_stop=True) if self.config.get('release_owner_after_stop',False) else APV()
                            command=handoff_zero(command,hybrid if hybrid is not None else (orient if orient is not None else gate))
                            status['owner_handoff_zero']=True
                        status['ack']=apv.send(2,command['vx_mps'],command['wz_radps']);commands+=command['vx_mps']>0 or command['wz_radps']!=0
                        status.update(hardware_authority=True,hardware_command={'vx_mps':command['vx_mps'],'wz_radps':command['wz_radps']})
                    trace_tick+=1
                    if apv is not None and trace_tick%2==0:owner_trace.append((round(time.monotonic(),3),status.get('phase'),read_owner_status()))
                    with self.lock:self.status=status
                    trace.write(json.dumps(status)+'\n')
                    if status['state'] in ('ABORTED','DONE'):reason=status['reason'];break
                    if not worker.is_alive():
                        if not look_once:raise ValueError('inference worker stopped')
                        worker_fault=True
                    next_tick+=.05;self.cancel.wait(max(0,next_tick-time.monotonic()))
        except BaseException as exc:
            reason=repr(exc)
            if 'owner ACK rejected' in reason:
                try:
                    atomic(out/'owner_status_trace.json',{'reason':reason,'at_reject':read_owner_status(),'at_reject_monotonic_s':round(time.monotonic(),3),
                                                           'samples':list(owner_trace) if 'owner_trace' in locals() else [],
                                                           'note':'owner_status = /run/hb/status.env snapshots (high_alive/armed/state/mode/busy, remote_alive...), newest last'})
                except Exception:pass
            if apv is None:stop_result=getattr(exc,'apv_cleanup',None)
            if 'status' in locals():atomic(out/'last_status.json',status)
        finally:
            if apv is not None:stop_result=apv.close()
            self.cancel.set()
            if sensor is not None:sensor.close()
            if worker is not None:
                worker.join(timeout=1)
                if worker.is_alive():worker.terminate();worker.join(timeout=1)
                if worker.is_alive():worker.kill();worker.join()
            if log is not None:log.close()
            result={'state':result_state,'task_id':task_id,'reason':reason,'mode':'execute' if self.config['execute'] else 'shadow','motion_requests_sent':apv.nonzero_sent_count if apv else 0,'motion_requests_acknowledged':commands,'forward_requests_sent':apv.forward_sent_count if apv else 0,'yaw_requests_sent':apv.yaw_sent_count if apv else 0,'stop':stop_result,'physical_arrival_verified':False,'answer':answer or 'Tác vụ đã kết thúc; chưa xác nhận đã tới vật.'}
            try:atomic(out/'result.json',result)
            except OSError as exc:result['persistence_error']=repr(exc)
            with self.lock:
                self.status=result;self.active=False;self.marker.unlink(missing_ok=True);self.camera_lease.close()


def build_navigation(config):
    strategy=config.get('navigation_strategy','turn_then_approach')
    if strategy=='gait_aware_arc':
        from ai_modules.navigation.gait_approach import GaitAwareApproach
        if type(config['gait_approach']['execute_enabled']) is not bool:raise ValueError('invalid navigation permission')
        if not config['planner']['controller'].get('cruise_until_goal',False):raise ValueError('gait strategy requires explicit cruise preset')
        for planner_key,gate_key in [('max_v_mps','max_v_mps'),('max_age_s','max_source_age_s'),('turn_threshold_rad','arc_heading_rad')]:
            if config['planner']['controller'][planner_key]!=config['gait_approach'][gate_key]:raise ValueError('inconsistent gait strategy '+gate_key)
        return GaitAwareApproach(config['gait_approach'],preview=not config['execute'])
    if strategy!='turn_then_approach':raise ValueError('unknown navigation strategy')
    from ai_modules.navigation.turn_approach import TurnThenApproach
    return TurnThenApproach(config['execution'],config['orientation'],preview=not config['execute']) if config['orientation']['enabled'] else None


def validate_config(config):
    # Shadow is deployment default; execute requires both configuration and owner opt-in.
    if type(config['execute']) is not bool or config['camera_hz']>20 or not 1<=config['camera_hz']:raise ValueError('invalid runtime config')
    if type(config.get('tracking_only_after_pin',False)) is not bool:raise ValueError('invalid tracking-only flag')
    if type(config.get('release_owner_after_stop',False)) is not bool:raise ValueError('invalid ownership release flag')
    if type(config.get('capture_retry_on_timeout',True)) is not bool:raise ValueError('invalid camera retry flag')
    segments=config.get('visual_segments',{})
    if type(segments.get('enabled',False)) is not bool:raise ValueError('invalid visual segments flag')
    if segments.get('enabled'):
        if config.get('approach_mode','visual_tracking')!='visual_tracking' or config.get('tracking_only_after_pin',False) or config['odom'].get('depth_usage')!='continuous_validation':
            raise ValueError('visual segments requires continuous visual measurements')
        from ai_modules.navigation.visual_segments import VisualSegments
        VisualSegments(segments,config['planner']['controller'],config['gait_approach'])
    if not 1<=config.get('tracking_camera_hz',config['camera_hz'])<=config['camera_hz']:raise ValueError('invalid tracking camera cadence')
    if config.get('tracking_only_after_pin') and config['odom'].get('depth_usage')!='initial_median_then_diagnostic':raise ValueError('tracking-only requires initial median mode')
    if config.get('approach_mode','visual_tracking') not in ('visual_tracking','initial_target_then_odom'):raise ValueError('invalid approach mode')
    if config.get('approach_mode')=='initial_target_then_odom':
        if config['odom'].get('depth_usage')!='initial_median_then_diagnostic' or not 10<=config.get('odom_hz',0)<=20:raise ValueError('invalid initial-target odometry mode')
    if type(config.get('gpu_warmup_runs')) is not int or not 1<=config['gpu_warmup_runs']<=10:raise ValueError('invalid GPU warmup runs')
    if not 1<=config.get('gpu_ready_timeout_s',0)<=60:raise ValueError('invalid GPU ready timeout')
    if not .05<=config['capture_rpc_timeout_s']<=.2 or not .1<=config['min_depth_interval_s']<=.3:raise ValueError('invalid capture/depth timing')
    if not 0<config.get('max_input_age_s',config['tracker']['max_age_s'])<=config['tracker']['max_age_s']:raise ValueError('invalid fresh input budget')
    from observation_policy import ObservationPolicy
    ObservationPolicy(config['observation_recovery'])
    if any(config['observation_recovery']['max_age_s']!=v for v in [config['planner']['controller']['max_age_s'],config['execution']['max_source_age_s'],config['temporal']['max_age_s'],config['odom']['max_observation_age_s']]):raise ValueError('inconsistent freshness limits')
    from ai_modules.navigation.turn_approach import TurnThenApproach
    if any(type(config['orientation'][key]) is not bool for key in ['enabled','execute_enabled','orientation_only']):raise ValueError('invalid orientation flags')
    TurnThenApproach(config['execution'],config['orientation'],preview=True)
    build_navigation(config)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--test-query');p.add_argument('--test-bbox',type=int,nargs=4);a=p.parse_args()
    config=json.loads(a.config.read_text())
    validate_config(config)
    instance=open(config['instance_lock'],'a')
    fcntl.flock(instance,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manager=Manager(config)
    if a.test_query:
        manager.submit({'query':a.test_query,'_test_bbox':a.test_bbox})
        while manager.active:time.sleep(.1)
        print(json.dumps(manager.snapshot(),ensure_ascii=False));return
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def do_POST(self):
            try:
                self.connection.settimeout(2.)
                if self.headers.get('Content-Type')!='application/json':raise ValueError('JSON required')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4096:raise ValueError('invalid payload')
                payload=json.loads(self.rfile.read(size))
                if self.path=='/approach':code,value=manager.submit({'query':payload.get('query','')})
                elif self.path=='/cancel':
                    if manager.cancel:manager.cancel.set()
                    code,value=200,{'status':'cancel_requested','answer':'Đã yêu cầu dừng; chờ xác nhận từ robot.'}
                elif self.path=='/status':code,value=200,manager.snapshot()
                else:code,value=404,{'status':'unknown_endpoint'}
            except Exception as exc:code,value=400,{'status':'invalid_request','reason':type(exc).__name__}
            body=json.dumps(value,ensure_ascii=False).encode();self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    server=ThreadingHTTPServer(('127.0.0.1',5594),Handler)
    def shutdown(*_):
        if manager.cancel:manager.cancel.set()
        threading.Thread(target=server.shutdown,daemon=True).start()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,shutdown)
    try:server.serve_forever(poll_interval=.1)
    finally:
        if manager.cancel:manager.cancel.set()
        if getattr(manager,'task_thread',None):manager.task_thread.join(timeout=3.)
        server.server_close()


if __name__=='__main__':main()
