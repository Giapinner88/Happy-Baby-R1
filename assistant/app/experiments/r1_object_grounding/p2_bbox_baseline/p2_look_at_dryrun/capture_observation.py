#!/usr/bin/env python3
"""One VideoHub JPEG plus nearby LowState samples. Read-only, Python 3.8 robot."""
import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',default='eth10')
    p.add_argument('--domain',type=int,default=0)
    p.add_argument('--timeout-s',type=float,default=5.)
    p.add_argument('--rpc-timeout-s',type=float,default=.5)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.timeout_s<=0 or a.rpc_timeout_s<=0 or a.domain<0:p.error('invalid timeouts/domain')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.go2.video.video_client import VideoClient
    names={12:'waist_roll_joint',13:'waist_yaw_joint',29:'head_pitch_joint',30:'head_yaw_joint'}
    arm_parts=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll')
    for side,indices in [('left',(15,16,17,18,19)),('right',(22,23,24,25,26))]:
        names.update({index:f'{side}_{part}_joint' for index,part in zip(indices,arm_parts)})
    samples=deque(maxlen=10000);lock=threading.Lock()
    def callback(message):
        if len(message.motor_state)<=30:return
        state={'received_monotonic_s':time.monotonic(),'robot_tick':int(message.tick),
               'q_rad':{name:float(message.motor_state[i].q) for i,name in names.items()},
               'dq_rad_s':{name:float(message.motor_state[i].dq) for i,name in names.items()}}
        imu=getattr(message,'imu_state',None)
        if imu is not None:
            state['imu_quaternion_wxyz']=list(imu.quaternion)
            state['imu_gyroscope_rad_s']=list(imu.gyroscope)
        with lock:samples.append(state)
    ChannelFactoryInitialize(a.domain,a.interface)
    sub=ChannelSubscriber('rt/lowstate',LowState_);sub.Init(callback,10)
    deadline=time.monotonic()+a.timeout_s
    while True:
        with lock:ready=bool(samples)
        if ready:break
        if time.monotonic()>deadline:raise RuntimeError('no LowState; no image/state record produced')
        time.sleep(.01)
    client=VideoClient();client.SetTimeout(a.rpc_timeout_s);client.Init()
    start=time.monotonic();utc=datetime.now(timezone.utc).isoformat()
    code,data=client.GetImageSample();end=time.monotonic()
    payload=bytes(data) if data is not None else b''
    if code!=0 or not payload.startswith(b'\xff\xd8') or not payload.rstrip(b'\0').endswith(b'\xff\xd9'):
        raise RuntimeError('camera did not return a complete JPEG; no retry')
    while time.monotonic()<deadline:
        with lock:after=bool(samples) and samples[-1]['received_monotonic_s']>=end
        if after:break
        time.sleep(.005)
    with lock:states=list(samples)
    midpoint=(start+end)/2
    nearest=min(states,key=lambda v:abs(v['received_monotonic_s']-midpoint))
    before=[s for s in states if s['received_monotonic_s']<=start]
    after=[s for s in states if s['received_monotonic_s']>=end]
    report={'schema':'happy_baby_r1.image_state_observation','schema_version':1,'read_only':True,
        'motor_commands_sent':False,'joint_mapping':'develop RobotSpec: waist roll12/yaw13, head pitch29/yaw30','image':'frame.jpg','image_sha256':hashlib.sha256(payload).hexdigest(),
        'host_utc_at_rpc_request':utc,'rpc_start_monotonic_s':start,'rpc_end_monotonic_s':end,
        'rpc_midpoint_proxy_monotonic_s':midpoint,'exposure_timestamp':None,
        'state_near_rpc_midpoint_proxy':nearest,'state_before_rpc':before[-1] if before else None,
        'state_after_rpc':after[0] if after else None,
        'state_rpc_proxy_gap_s':abs(nearest['received_monotonic_s']-midpoint),
        'state_sample_count':len(states),'interface':a.interface,
        'limit':'Receiver timestamps only, not synchronized exposure; camera buffering unknown. Waist roll slot availability depends on model.'}
    out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
    (out/'frame.jpg').write_bytes(payload)
    (out/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output_dir':str(out),'image_sha256':report['image_sha256'],
          'q_rad':nearest['q_rad'],'state_rpc_proxy_gap_s':report['state_rpc_proxy_gap_s']},indent=2))


if __name__=='__main__':main()
