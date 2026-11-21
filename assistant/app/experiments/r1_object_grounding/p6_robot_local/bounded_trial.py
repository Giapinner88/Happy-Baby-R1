"""Single bounded forward trial via APV1. Default dry-run, no motor publisher.

Requires independently enabled owner, operator floor/E-stop setup and --execute.
No mode/arming/service changes; no replay target or autonomous approach.
"""
import argparse
import json
import math
import secrets
import signal
import socket
import struct
import threading
import time
from pathlib import Path


def validate(config):
    profile=config.get('profile','interface_probe')
    if profile not in ('interface_probe','gait_activation'):raise ValueError('unknown trial profile')
    limits=({'speed_mps':.16,'duration_s':2.5,'acceleration_mps2':.1,
             'max_displacement_m':.1,'max_tilt_deg':12.,'max_yaw_change_deg':5.}
            if profile=='gait_activation' else
            {'speed_mps':.03,'duration_s':1.,'acceleration_mps2':.05,
             'max_displacement_m':.05,'max_tilt_deg':12.,'max_yaw_change_deg':5.})
    for key,limit in limits.items():
        if type(config[key]) not in (int,float) or not math.isfinite(config[key]) or not 0<config[key]<=limit:
            raise ValueError('invalid/excessive '+key)
    if config['rate_hz']!=20:raise ValueError('trial requires 20Hz')


def trial(config,request,feedback,clock=time.monotonic,sleep=time.sleep):
    validate(config);events=[];start_pose=None;began=None;previous=None;stopped=False
    try:
        feedback()  # must have fresh state before Hello
        if request(4,0.)!=4:raise RuntimeError('Ping rejected')
        if request(1,0.)!=1:raise RuntimeError('Hello rejected')
        began=clock();previous=began;next_tick=began
        while True:
            now=clock();elapsed=now-began
            if elapsed>=config['duration_s']:break
            if not 0<=now-previous<=.1:raise TimeoutError('control gap')
            state=feedback()
            if not all(math.isfinite(v) for v in state.values()):raise ValueError('nonfinite feedback')
            if max(abs(state['roll_deg']),abs(state['pitch_deg']))>config['max_tilt_deg']:raise ValueError('tilt guard')
            if start_pose is None:start_pose=state
            displacement=math.hypot(state['x_m']-start_pose['x_m'],state['y_m']-start_pose['y_m'])
            yaw=math.degrees(math.atan2(math.sin(state['yaw_rad']-start_pose['yaw_rad']),math.cos(state['yaw_rad']-start_pose['yaw_rad'])))
            if displacement>config['max_displacement_m'] or abs(yaw)>config['max_yaw_change_deg']:raise ValueError('motion bound')
            vx=min(config['speed_mps'],config['acceleration_mps2']*elapsed)
            if request(2,vx)!=2:raise RuntimeError('command not stored')
            events.append({'elapsed_s':elapsed,'requested_vx_mps':vx,'requested_wz_radps':0.,'feedback':state,'displacement_m':displacement})
            previous=now;next_tick+=1/config['rate_hz'];sleep(max(0,next_tick-clock()))
        if request(3,0.)!=3:raise RuntimeError('STOP not acknowledged')
        stopped=True
        return {'status':'bounded_requests_completed','events':events,'stop_acknowledged':True,
                'motor_response_verified':False,'note':'ACK is receiver receipt; feedback saved for analysis, not automatic success claim'}
    except BaseException as exc:
        # Preserve observations collected before a guard/ACK failure.
        exc.trial_events=events
        raise
    finally:
        if not stopped:
            try:request(3,0.)
            except Exception:pass  # owner deadline/watchdog remains independent


class Feedback:
    def __init__(self,interface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        ChannelFactoryInitialize(0,interface)
        self.lock=threading.Lock();self.low=None;self.odom=None
        def low(msg):
            with self.lock:self.low=(time.monotonic(),list(msg.imu_state.quaternion),list(msg.imu_state.gyroscope),
                                     [float(msg.motor_state[i].q) for i in range(12)],
                                     [float(msg.motor_state[i].dq) for i in range(12)])
        def odom(msg):
            with self.lock:self.odom=(time.monotonic(),list(msg.position),list(msg.imu_state.quaternion))
        self.subs=[ChannelSubscriber('rt/lowstate',LowState_),ChannelSubscriber('rt/lf/odommodestate',SportModeState_)]
        self.subs[0].Init(low,1);self.subs[1].Init(odom,1)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            with self.lock:ready=self.low is not None and self.odom is not None
            if ready:return
            time.sleep(.01)
        raise TimeoutError('no feedback before trial')

    def read(self):
        with self.lock:low,odom=self.low,self.odom
        now=time.monotonic()
        if low is None or odom is None or not 0<=now-low[0]<=.1 or not 0<=now-odom[0]<=.15:raise TimeoutError('feedback stale')
        q=low[1];oq=odom[2]
        if len(q)!=4 or len(oq)!=4 or not all(math.isfinite(v) for v in q+oq+low[2]+odom[1]+low[3]+low[4]):raise ValueError('invalid feedback')
        for values in (q,oq):
            if not .98<=math.sqrt(sum(v*v for v in values))<=1.02:raise ValueError('invalid quaternion')
        w,x,y,z=q;ow,ox,oy,oz=oq
        if math.sqrt(sum(v*v for v in low[2]))>.8:raise ValueError('gyro guard')
        state={'x_m':float(odom[1][0]),'y_m':float(odom[1][1]),
                'roll_deg':math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
                'pitch_deg':math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))),
                'yaw_rad':math.atan2(2*(ow*oz+ox*oy),1-2*(oy*oy+oz*oz))}
        state.update({'leg_q_%02d_rad'%i:value for i,value in enumerate(low[3])})
        state['max_abs_leg_dq_rad_s']=max(abs(value) for value in low[4])
        return state


def owner_ready():
    status=dict(line.split('=',1) for line in Path('/run/hb/status.env').read_text().splitlines() if '=' in line)
    if not 0<=time.monotonic()-int(status['updated_monotonic_ms'])/1000<=.3:raise ValueError('owner status stale')
    if any(status.get(k)!=v for k,v in {'high_alive':'1','high_armed':'1','remote_alive':'1','high_state':'LOCOMOTION','high_mode':'walk'}.items()):raise ValueError('owner gate')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    config=json.loads(a.config.read_text());validate(config);out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
    (out/'resolved_config.json').write_text(json.dumps(config,indent=2)+'\n')
    if not a.execute:
        (out/'result.json').write_text(json.dumps({'status':'dry_run','motion_sent':False,'maximum_requested_distance_m':config['speed_mps']*config['duration_s']},indent=2)+'\n');print('Dry-run only; no socket or SDK created');return
    def interrupt(*_):raise KeyboardInterrupt()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,interrupt)
    try:owner_ready();feedback=Feedback(config['interface'])
    except BaseException as exc:
        (out/'result.json').write_text(json.dumps({'status':'initialization_failed','error':repr(exc),'motion_sent':False},indent=2)+'\n');raise
    wire=struct.Struct('<IHHQQQff');nonce=secrets.randbits(63)+1;sequence=0;sent=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.connect(('127.0.0.1',5573));sock.settimeout(.08)
        def request(kind,vx):
            nonlocal sequence
            if kind!=3:owner_ready()
            sequence+=1;now=time.monotonic_ns()//1000
            sock.send(wire.pack(0x31565041,1,kind,nonce,sequence,now+100000,vx,0.));sent.append({'kind':kind,'sequence':sequence,'vx':vx,'wz':0.,'stamp':now})
            data=sock.recv(128)
            if len(data)!=40:raise ValueError('ACK size')
            magic,version,status,session,seq,stamp,rv,rw=wire.unpack(data)
            if (magic,version,session,seq)!=(0x31565041,1,nonce,sequence):raise ValueError('ACK identity')
            if not math.isfinite(rv) or not math.isfinite(rw) or abs(rv)>config['speed_mps']+1e-6 or rw!=0:raise ValueError('ACK velocity')
            return status
        try:result=trial(config,request,feedback.read)
        except BaseException as exc:
            result={'status':'failed','error':repr(exc),'events':getattr(exc,'trial_events',[]),'motor_response_verified':False};raise
        finally:
            result['sent']=sent;(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
