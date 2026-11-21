#!/usr/bin/env python3
"""Separate lab client for existing HB locomotion owner: inspect, plan, stream."""
import argparse
import csv
from datetime import datetime,timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import shlex
import signal
import socket
import sys
import time

from control import (ARM_JOINTS,HEAD_JOINTS,JOINTS,interpolate,limits,packet,
                     point_arm,ramp_duration,runtime_info,validate_target)

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def dump(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=HERE/'config.json')
    p.add_argument('--runtime-root',type=Path,default=ROOT/'third_party/Happy-Baby-R1-develop')
    commands=p.add_subparsers(dest='command',required=True)
    commands.add_parser('inspect')
    plan=commands.add_parser('plan')
    request=plan.add_mutually_exclusive_group(required=True)
    request.add_argument('--joints',type=Path,help='JSON mapping joint names to absolute radians')
    request.add_argument('--point',type=float,nargs=3,metavar=('X','Y','Z'),help='Metric target in waist_yaw_link')
    request.add_argument('--look-plan',type=Path,help='Import dry-run head proposal for offline inspection')
    plan.add_argument('--side',choices=['left','right'],default='right')
    plan.add_argument('--seed',type=float,nargs=5)
    plan.add_argument('--output-dir',type=Path,required=True)
    stream=commands.add_parser('stream')
    stream.add_argument('--target',type=Path,required=True)
    stream.add_argument('--interface',default='eth10')
    stream.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();cfg=json.loads(a.config.read_text())
    for key in ['stream_hz','hold_s','ramp_min_s','max_velocity_rad_s','max_acceleration_rad_s2','state_timeout_s','initial_state_timeout_s']:
        if type(cfg[key]) not in (int,float) or not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid '+key)
    if not 10<=cfg['stream_hz']<=100:raise ValueError('stream_hz must be in [10,100]')
    info=runtime_info(a.runtime_root,cfg);joint_limits=limits(info['urdf'],cfg)
    if a.command=='inspect':
        print(json.dumps({**info,'joint_limits_rad':joint_limits,'udp_endpoint':['127.0.0.1',cfg['udp_port']],
             'sim_h4_teleop_parity':False,'owns_locomotion':False,'creates_lowcmd_publisher':False},indent=2));return
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    dump(out/'resolved_config.json',cfg);dump(out/'runtime_reference.json',info)
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    snap=out/'source_snapshot';snap.mkdir()
    for source in [HERE/'run.py',HERE/'control.py',HERE/'state.py',a.config]:
        (snap/source.name).write_bytes(source.read_bytes())
    dump(out/'source_sha256.json',{str(s):hashlib.sha256(s.read_bytes()).hexdigest() for s in [HERE/'run.py',HERE/'control.py',HERE/'state.py',a.config]})
    if a.command=='plan':
        detail=None;kind='manual_joints'
        if a.joints:joints=json.loads(a.joints.read_text())
        elif a.point is not None:
            detail=point_arm(info['urdf'],a.side,a.point,cfg,a.seed);joints=detail['joints_rad'];kind='pointing_ik'
            if not detail['solved']:
                dump(out/'ik_failure.json',detail);raise ValueError('pointing IK did not reach angular tolerance')
        else:
            source=json.loads(a.look_plan.read_text());geometry=source.get('geometry')
            if not geometry or not geometry['within_angular_tolerance']:raise ValueError('no usable dry-run head geometry')
            pitch,yaw=geometry['candidate_pitch_yaw_rad'];joints={HEAD_JOINTS[0]:yaw,HEAD_JOINTS[1]:pitch}
            detail={'source_plan':str(a.look_plan.resolve()),'execution_blockers':source.get('execution_blockers',[])};kind='head_joints_from_look_at_proposal'
        validate_target(joints,joint_limits)
        target={'schema':'r1_target_control.v1','kind':kind,'joints_rad':joints,'details':detail,
                'policy_sha256':info['policy_sha256'],'created_utc':datetime.now(timezone.utc).isoformat(),
                'collision_checked':False,'balance_validated':False}
        dump(out/'target.json',target);print(json.dumps(target,indent=2));return
    target=json.loads(a.target.read_text())
    if target['policy_sha256']!=info['policy_sha256']:raise ValueError('target uses a different policy reference')
    joints=validate_target(target['joints_rad'],joint_limits)
    # Existing receiver has no sender ID. This lock prevents this client's own
    # duplicate streams; Quest/other external senders must be stopped by operator.
    lock=open('/tmp/r1_target_control.sender.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from state import StateReader
    reader=StateReader(a.interface,cfg['initial_state_timeout_s']);initial=reader.latest(cfg['state_timeout_s'])
    start=initial['positions_rad'];end=[joints.get(n,q) for n,q in zip(JOINTS,start)]
    arms=any(n in joints for n in ARM_JOINTS)
    head=any(n in joints for n in HEAD_JOINTS)
    # UTL1 arm_valid covers both arms. The untouched arm is held at measured
    # start pose; head-only keeps arm_valid=0 so locomotion retains both arms.
    if arms:
        validate_target(dict(zip(ARM_JOINTS,start[:10])),joint_limits)
        validate_target(dict(zip(ARM_JOINTS,end[:10])),joint_limits)
    if head:validate_target(dict(zip(HEAD_JOINTS,end[10:])),joint_limits)
    # Receiver validates all head fields even when head_valid=0.
    wire_end=end.copy();wire_start=start.copy()
    if not head:wire_start[10:]=wire_end[10:]=[0.,0.]
    validate_target(dict(zip(HEAD_JOINTS,wire_start[10:])),joint_limits)
    duration=ramp_duration(wire_start,wire_end,cfg)
    dump(out/'initial_state.json',initial);dump(out/'target_snapshot.json',target)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);seq=0;dst=('127.0.0.1',cfg['udp_port'])
    def send(values,enabled):
        nonlocal seq
        sock.sendto(packet(seq,values,enabled,arms,head),dst);seq+=1
    status='stream_ended';error=None;last=None
    def cancel_signal(signum,frame):raise KeyboardInterrupt()
    previous_handlers={s:signal.signal(s,cancel_signal) for s in (signal.SIGTERM,signal.SIGHUP)}
    try:
        # STOP before ENABLE is the controller's existing H4 session contract.
        for _ in range(math.ceil(.4*cfg['stream_hz'])):
            send(wire_start,False);time.sleep(1/cfg['stream_hz'])
        t0=time.monotonic();next_tick=t0
        with (out/'trace.csv').open('w') as trace:
            writer=csv.writer(trace);writer.writerow(['elapsed_s','state_age_s','seq','max_requested_joint_error_rad',*['command_'+n for n in JOINTS],*['measured_'+n for n in JOINTS]])
            while time.monotonic()-t0<duration+cfg['hold_s']:
                now=time.monotonic();state=reader.latest(cfg['state_timeout_s'])
                values=interpolate(wire_start,wire_end,now-t0,duration);send(values,True)
                actual=state['positions_rad'];diff=max(abs(actual[JOINTS.index(n)]-joints[n]) for n in joints)
                last=diff
                writer.writerow([now-t0,now-state['received_monotonic_s'],seq,diff,*values,*actual])
                next_tick+=1/cfg['stream_hz'];time.sleep(max(0.,next_tick-time.monotonic()))
    except KeyboardInterrupt:status='cancelled'
    except Exception as exc:status='failed';error=str(exc)
    finally:
        for _ in range(5):
            send(wire_start,False);time.sleep(1/cfg['stream_hz'])
        sock.close();lock.close()
        for signum,handler in previous_handlers.items():signal.signal(signum,handler)
        dump(out/'result.json',{'status':status,'error':error,'packets_sent':seq,
             'ramp_duration_s':duration,'last_measured_target_error_rad':last,
             'last_tracking_within_tolerance':last is not None and last<=cfg['tracking_tolerance_rad'],
             'owner_acknowledgement_available':False,'note':'UDP send does not prove owner accepted target or policy active. No lowcmd publisher here.'})
    if error:raise RuntimeError(error)


if __name__=='__main__':main()
