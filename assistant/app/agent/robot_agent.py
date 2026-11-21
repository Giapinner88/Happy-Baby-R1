#!/usr/bin/env python3
"""Robot-local loopback RPC + bounded UTL1/head stream + measured-heading yaw.
No LowCmd publisher. Existing run_r1 remains the only motor owner.
"""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
import uuid

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent));sys.path.insert(0,str(HERE/'target_control'))
from control import (JOINTS,HEAD_JOINTS,ARM_JOINTS,IDL,packet,limits,validate_target,
                     runtime_info,ramp_duration,interpolate)
from feedback import unwrap,yaw_rate


def quaternion_yaw(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q):raise ValueError('invalid quaternion')
    norm=math.sqrt(sum(v*v for v in q))
    if norm<.5 or norm>1.5:raise ValueError('invalid quaternion norm')
    w,x,y,z=[v/norm for v in q]
    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def check_runtime_processes(runtime_root,proc_root=Path('/proc')):
    """Preflight only: inspect process metadata; never stop another controller."""
    expected=(Path(runtime_root)/'controller/build/run_r1').resolve()
    owners=[];senders=[]
    conflicts={'run_r1_vendor_targets.py','run_r1_vendor_ik_stream.py',
               'run_r1_policy_rollout.py','run_r1_upstream_ik_stream.py'}
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():continue
        try:
            args=proc.joinpath('cmdline').read_bytes().split(bytes([0]))
            args=[a.decode(errors='replace') for a in args if a]
            if not args:continue
            if Path(args[0]).name=='run_r1':owners.append((proc.name,(proc/'exe').resolve()))
            if any(Path(a).name in conflicts for a in args):senders.append(proc.name)
        except (OSError,ValueError):continue
    if len(owners)!=1 or owners[0][1]!=expected:
        raise RuntimeError('P4 owner required as sole run_r1; observed '+str([(pid,str(path)) for pid,path in owners]))
    if senders:raise RuntimeError('competing Quest/policy sender detected, PID '+','.join(senders)+'; stop its service before P4')
    return {'owner_pid':owners[0][0],'owner':str(expected)}


def require_walk_search_mode(path=Path('/run/hb/status.env'), now_s=None):
    """R2+A only: never compete with the R2+X Quest/teleop UDP sender."""
    try:
        values=dict(line.split('=',1) for line in path.read_text().splitlines() if '=' in line)
        stamp=int(values['updated_monotonic_ms'])
    except (OSError,ValueError,KeyError) as exc:
        raise RuntimeError('HB owner status unavailable for search') from exc
    now_ms=int(1000*(time.monotonic() if now_s is None else now_s))
    if stamp>now_ms+100 or now_ms-stamp>2000:
        raise RuntimeError('HB owner status stale for search')
    if (values.get('high_alive')!='1' or values.get('high_armed')!='1' or
        values.get('remote_alive')!='1' or values.get('high_state')!='LOCOMOTION' or
        values.get('high_mode')!='walk'):
        raise RuntimeError('object search requires armed R2+A locomotion; release R2+X/Quest')
    return values


class RobotState:
    def __init__(self,interface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.lock=threading.Lock();self.item=None
        def update(msg):
            try:
                now=time.monotonic();q=[float(msg.motor_state[i].q) for i in IDL]
                dq=[float(msg.motor_state[i].dq) for i in IDL]
                if not all(math.isfinite(v) for v in q+dq):return
                wrapped=quaternion_yaw(list(msg.imu_state.quaternion))
                with self.lock:
                    old=self.item
                    if old and old['tick']==int(msg.tick):return
                    heading=unwrap(old['body_heading_deg'] if old else None,wrapped)
                    dt=now-old['time_s'] if old else 0
                    speed=(heading-old['body_heading_deg'])/dt if dt>1e-4 else 0
                    self.item={'time_s':now,'tick':int(msg.tick),'q':q,'dq':dq,
                        'head_yaw_deg':math.degrees(q[10]),'head_speed_deg_s':math.degrees(dq[10]),
                        'body_heading_deg':heading,'body_speed_deg_s':speed,
                        'waist_yaw_rad':float(msg.motor_state[13].q),
                        'waist_yaw_deg':math.degrees(float(msg.motor_state[13].q)),
                        'waist_speed_deg_s':math.degrees(float(msg.motor_state[13].dq))}
            except (ValueError,IndexError,AttributeError):return
        ChannelFactoryInitialize(0,interface)
        self.sub=ChannelSubscriber('rt/lowstate',LowState_);self.sub.Init(update,10)
    def latest(self):
        with self.lock:s=dict(self.item) if self.item else None
        if not s or time.monotonic()-s['time_s']>.2:raise TimeoutError('no fresh LowState')
        if not all(math.isfinite(s[k]) for k in ('waist_yaw_rad','waist_yaw_deg','waist_speed_deg_s')):
            raise ValueError('nonfinite waist yaw state')
        return s
    def require_safe_waist(self,s):
        waist=s['waist_yaw_rad']
        if abs(waist)>math.radians(10):
            raise ValueError('waist yaw outside +/-10deg search assumption: measured=%.3fdeg' % math.degrees(waist))
        return s


class Agent:
    def __init__(self,a):
        self.a=a;self.cfg=json.loads((HERE/'target_control/config.json').read_text())
        self.info=runtime_info(Path(a.runtime_root),self.cfg);self.bounds=limits(self.info['urdf'],self.cfg)
        self.lockfile=open('/tmp/r1_target_control.sender.lock','w');fcntl.flock(self.lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.reader=RobotState(a.interface);self.lock=threading.RLock();self.cancel=threading.Event()
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.seq=0
        self.goal=None;self.body_goal=None;self.last_rpc=time.monotonic();self.last_error=None
        self.enabled_at=None;self.abort=threading.Event();self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
    def set_joints(self,joints):
        require_walk_search_mode()
        validate_target(joints,self.bounds);s=self.reader.require_safe_waist(self.reader.latest());self.abort.clear()
        with self.lock:
            start=s['q'];validate_target(dict(zip(JOINTS,start)),self.bounds)
            end=[joints.get(n,q) for n,q in zip(JOINTS,start)]
            validate_target(dict(zip(JOINTS,end)),self.bounds)
            if self.enabled_at is None:self.enabled_at=time.monotonic()+.4
            self.goal=(start,end,time.monotonic(),ramp_duration(start,end,self.cfg))
            self.last_error=None
        return end
    def yaw(self,rate):
        p=struct.pack('<IIfI',0x31593450,self.seq&0xffffffff,math.radians(rate),1)
        self.sock.sendto(p,('127.0.0.1',self.a.yaw_port));self.seq+=1
    def stop(self):
        with self.lock:self.body_goal=None;self.yaw(0)
    def release(self):
        self.abort.set()
        with self.lock:
            self.body_goal=None;self.goal=None;self.enabled_at=None;self.yaw(0)
        for _ in range(5):
            self.sock.sendto(packet(self.seq,[0.]*12,False),('127.0.0.1',self.cfg['udp_port']));self.seq+=1;time.sleep(.01)
        with self.lock:self.goal=None;self.enabled_at=None
    def loop(self):
        while not self.cancel.wait(1/self.cfg['stream_hz']):
            try:
                with self.lock:
                    now=time.monotonic();state=self.reader.latest()
                    if self.body_goal is not None or self.goal is not None:
                        require_walk_search_mode(now_s=now)
                    if self.body_goal is not None:
                        target,began=self.body_goal
                        if now-self.last_rpc>.75 or now-began>45:raise TimeoutError('body turn lease/timeout')
                        self.yaw(yaw_rate(target,state['body_heading_deg'],2.,45.,1.))
                    if now-self.last_rpc>35:raise TimeoutError('search RPC lease expired')
                    if self.goal:
                        start,end,t0,duration=self.goal
                        values=interpolate(start,end,max(0,now-t0-.4),duration)
                        self.sock.sendto(packet(self.seq,values,now>=self.enabled_at),('127.0.0.1',self.cfg['udp_port']));self.seq+=1
            except Exception as exc:
                with self.lock:
                    self.last_error=str(exc);self.body_goal=None
                    try:
                        self.yaw(0)
                        self.sock.sendto(packet(self.seq,[0.]*12,False),('127.0.0.1',self.cfg['udp_port']));self.seq+=1
                    except OSError:pass
                    self.goal=None;self.enabled_at=None
    def wait_motion_window(self,duration_s):
        began=time.monotonic()
        while time.monotonic()-began<duration_s:
            if self.abort.is_set():raise InterruptedError('target cancelled')
            self.last_rpc=time.monotonic()
            self.reader.require_safe_waist(self.reader.latest())
            if self.last_error:raise RuntimeError(self.last_error)
            time.sleep(.03)
        self.reader.require_safe_waist(self.reader.latest())
        if self.last_error:raise RuntimeError(self.last_error)
    def request(self,method,params):
        self.last_rpc=time.monotonic()
        if method=='clock':return {'time_s':time.monotonic()}
        if method=='stop':self.release();return {'status':'released'}
        if method=='release':self.release();return {'status':'released'}
        if self.last_error and method not in ['rest_arms','state']:raise RuntimeError(self.last_error)
        if method=='state':
            # A previous motion/lease/state fault must not hide fresh telemetry.
            # latest() still enforces freshness and the waist envelope. Motion
            # faults remain latched until rest_arms/set_joints establishes a goal.
            return self.reader.latest()
        if method=='rest_arms':
            check_runtime_processes(self.a.runtime_root)
            require_walk_search_mode()
            self.stop();j=dict(zip(ARM_JOINTS,self.cfg['default_arm_rad']));j.update(head_yaw_joint=0.,head_pitch_joint=0.)
            # Neutral arm placement is preparation, not evidence that pointing
            # succeeded. Give the owner a short command window, then continue
            # regardless of residual arm error. Pointing still uses the strict
            # encoder tracking check in execute_target below.
            self.set_joints(j)
            began=time.monotonic()
            while time.monotonic()-began<self.cfg.get('rest_command_s',1.0):
                if self.abort.is_set():raise InterruptedError('target cancelled')
                self.last_rpc=time.monotonic()
                if self.last_error:raise RuntimeError(self.last_error)
                time.sleep(.03)
            return {'status':'rest_commanded_without_tracking_gate'}
        if method=='move_head':
            self.set_joints({'head_yaw_joint':math.radians(params['yaw_deg'])});return None
        if method=='move_body':
            require_walk_search_mode()
            target=float(params['heading_deg']);s=self.reader.latest()
            delta=target-s['body_heading_deg']
            if not math.isfinite(target) or abs(delta)>181:
                raise ValueError('single turn exceeds 180deg plus 1deg telemetry drift')
            # Search uses an earlier IMU sample. Clamp transport-time drift so
            # the command actually sent to the owner never exceeds 180deg.
            bounded=s['body_heading_deg']+max(-180.,min(180.,delta))
            with self.lock:self.body_goal=(bounded,time.monotonic())
            return None
        if method=='capture':
            require_walk_search_mode()
            self.reader.require_safe_waist(self.reader.latest())
            # Each retry records a new RPC/image/state pair; never re-use the
            # timestamp or encoder sample of an unsuccessful JPEG request.
            for attempt in range(3):
                directory='/tmp/r1_p4_capture_'+uuid.uuid4().hex
                process=subprocess.run([sys.executable,str(HERE/'capture_observation.py'),'--interface',self.a.interface,'--output-dir',directory],capture_output=True,text=True,timeout=10)
                if not process.returncode:break
                if 'camera did not return a complete JPEG' not in process.stderr:break
            if process.returncode:raise RuntimeError('capture failed after %d attempt(s): ' % (attempt+1)+process.stderr[-1000:])
            record=json.loads((Path(directory)/'capture.json').read_text());s=record['state_near_rpc_midpoint_proxy']
            waist=s['q_rad'].get('waist_yaw_joint')
            if waist is None or not math.isfinite(waist) or abs(waist)>math.radians(10):
                measured=math.degrees(waist) if waist is not None and math.isfinite(waist) else float('nan')
                raise ValueError('capture waist yaw outside +/-10deg search assumption: measured=%.3fdeg' % measured)
            heading=quaternion_yaw(s['imu_quaternion_wxyz'])
            heading=unwrap(self.reader.latest()['body_heading_deg'],heading)
            return {'capture_dir':directory,'captured_at_s':record['rpc_midpoint_proxy_monotonic_s'],
                'pose':{'time_s':s['received_monotonic_s'],'head_yaw_deg':math.degrees(s['q_rad']['head_yaw_joint']),
                        'body_heading_deg':heading}}
        if method=='execute_target':
            self.stop();t=params['target']
            if t['policy_sha256']!=self.info['policy_sha256']:raise ValueError('policy mismatch')
            self.set_joints(t['joints_rad'])
            self.wait_motion_window(self.goal[3]+self.cfg['hold_s']+.4)
            if self.abort.is_set():raise InterruptedError('target cancelled')
            self.set_joints(dict(zip(ARM_JOINTS,self.cfg['default_arm_rad'])))
            self.wait_motion_window(self.goal[3]+.4)
            return {'status':'command_completed','encoder_tracking_verified':False,
                    'owner_policy_ack_available':False}
        raise ValueError('unknown method')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['serve','rpc'])
    p.add_argument('--port',type=int,default=5572);p.add_argument('--yaw-port',type=int,default=5571)
    p.add_argument('--interface',default='eth10');p.add_argument('--runtime-root',default='/home/unitree/HB')
    a=p.parse_args()
    if a.mode=='rpc':
        request=json.load(sys.stdin)
        with socket.create_connection(('127.0.0.1',a.port),3) as sock:
            sock.settimeout(55);sock.sendall((json.dumps(request)+'\n').encode())
            response=sock.makefile('r').readline(2000000)
            if not response:raise RuntimeError('agent disconnected')
            sys.stdout.write(response)
        return
    agent=Agent(a)
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                request=json.loads(self.rfile.readline(2000000));result=agent.request(request['method'],request.get('params',{}))
                response={'result':result}
            except Exception as exc:
                agent.release();response={'error':str(exc)}
            self.wfile.write((json.dumps(response)+'\n').encode())
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address=True
        daemon_threads=True
    server=Server(('127.0.0.1',a.port),Handler)
    print('P4 agent listening on loopback; arm owner manually with R2+A then START.',flush=True)
    def shutdown(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,shutdown)
    try:server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:pass
    finally:
        agent.cancel.set();agent.thread.join(2);agent.release();agent.sock.close();server.server_close()


if __name__=='__main__':main()
