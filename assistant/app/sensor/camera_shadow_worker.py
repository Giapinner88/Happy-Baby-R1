"""Robot-side read-only camera RPC over stdin/stdout; no motor APIs.

stdout wire: uint32 network-order JSON length, JSON header, JPEG bytes.
All SDK/library console output is diverted to stderr before imports.
"""
import argparse
import hashlib
import json
import os
import struct
import sys
import time
import math


def encode_for_wire(payload, quality):
    if quality is None:return payload
    import cv2
    import numpy as np
    image=cv2.imdecode(np.frombuffer(payload,dtype=np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError('JPEG decode failed')
    ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,quality])
    if not ok:raise ValueError('JPEG encode failed')
    return encoded.tobytes()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface', required=True)
    p.add_argument('--rpc-timeout-s', type=float, required=True)
    p.add_argument('--split-pose', action='store_true', help='Send image first, then exact-sequence pose metadata packet')
    p.add_argument('--state-enabled', action='store_true')
    p.add_argument('--odom-enabled', action='store_true')
    p.add_argument('--odom-topic', choices=['rt/odommodestate', 'rt/lf/odommodestate'], default='rt/lf/odommodestate')
    p.add_argument('--odom-wait-s', type=float, default=.08)
    p.add_argument('--state-timeout-s', type=float, default=3.)
    p.add_argument('--state-after-wait-s', type=float, default=.03)
    p.add_argument('--jpeg-quality',type=int,help='Explicit optional same-resolution wire re-encoding')
    a = p.parse_args()
    if a.jpeg_quality is not None and not 50<=a.jpeg_quality<=95:p.error('jpeg quality must be50..95')
    if any(not math.isfinite(v) or v <= 0 for v in
           (a.rpc_timeout_s, a.state_timeout_s, a.state_after_wait_s, a.odom_wait_s)):
        p.error('timeout must be positive')
    wire = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    if a.jpeg_quality is not None:
        # Initialize codec before DDS/RPC subscriptions. Bound CPU parallelism
        # so encoding cannot oversubscribe the embedded owner/camera cores.
        os.environ['OPENBLAS_NUM_THREADS']='1'
        os.environ['OMP_NUM_THREADS']='1'
        import cv2
        import numpy as np
        cv2.setNumThreads(1)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    ChannelFactoryInitialize(0, a.interface)
    state_buffer = None
    subscriber = None
    odom_buffer = None
    odom_subscriber = None
    if a.odom_enabled:
        from odom_sync import OdomBuffer
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        odom_buffer = OdomBuffer(topic=a.odom_topic)
        odom_subscriber = ChannelSubscriber(a.odom_topic, SportModeState_)
        odom_subscriber.Init(odom_buffer.add_message, 10)
    if a.state_enabled:
        from state_sync import StateBuffer, JOINT_IDS
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        state_buffer = StateBuffer()
        def callback(message):
            try:
                sample = {'received_monotonic_s': time.monotonic(),
                          'robot_tick': int(message.tick),
                          'q_rad': {name: float(message.motor_state[i].q) for name, i in JOINT_IDS.items()},
                          'dq_rad_s': {name: float(message.motor_state[i].dq) for name, i in JOINT_IDS.items()},
                          'imu_quaternion_wxyz': [float(v) for v in message.imu_state.quaternion],
                          'imu_gyroscope_rad_s': [float(v) for v in message.imu_state.gyroscope]}
                state_buffer.add(sample)
            except (AttributeError, IndexError, TypeError, ValueError):
                with state_buffer.condition:
                    state_buffer.rejected += 1
        subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        subscriber.Init(callback, 10)
        if not state_buffer.wait_ready(a.state_timeout_s):
            raise RuntimeError('no finite LowState before timeout; no motion commands sent')
    client = VideoClient()
    client.SetTimeout(a.rpc_timeout_s)
    client.Init()
    ready = json.dumps({'ok': False, 'sequence': 0, 'bytes': 0,
                        'read_only': True, 'event': 'ready', 'state_enabled': a.state_enabled}).encode('utf-8')
    wire.write(struct.pack('!I', len(ready))+ready)
    sequence = 0
    for line in sys.stdin:
        if line.strip() == 'close':
            break
        if line.strip()=='odom':
            sequence+=1
            sample=odom_buffer.latest() if odom_buffer is not None else None
            header={'ok':sample is not None,'sequence':sequence,'bytes':0,'event':'odom',
                    'read_only':True,'sha256':hashlib.sha256(b'').hexdigest(),'odom_sample':sample}
            packed=json.dumps(header).encode('utf-8');wire.write(struct.pack('!I',len(packed))+packed)
            continue
        if line.strip() != 'capture':
            break
        sequence += 1
        started = time.monotonic()
        try:
            code, data = client.GetImageSample()
            payload = bytes(data) if data is not None else b''
            received = time.monotonic()
            ok = code == 0 and payload.startswith(b'\xff\xd8') and len(payload) <= 4*1024*1024
            if not ok:
                payload = b''
            original_bytes=len(payload);original_sha=hashlib.sha256(payload).hexdigest()
            if ok:payload=encode_for_wire(payload,a.jpeg_quality)
            header = {'ok': ok, 'sequence': sequence, 'code': code,
                      'remote_requested_monotonic_s': started,
                      'remote_received_monotonic_s': received,
                      'rpc_latency_s': received-started,
                      'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest(),
                      'read_only': True, 'exposure_timestamp_verified': False}
            header.update(original_bytes=original_bytes,original_sha256=original_sha,jpeg_quality=a.jpeg_quality,
                          jpeg_cpu_threads=1 if a.jpeg_quality is not None else None)
            if a.split_pose:
                packed = json.dumps(dict(header, event='image')).encode('utf-8')
                wire.write(struct.pack('!I',len(packed))+packed+payload)
                payload=b''
                header=dict(header,event='pose',bytes=0,sha256=hashlib.sha256(b'').hexdigest())
            if state_buffer is not None:
                header['pose_sync'] = state_buffer.capture_record(started, received, a.state_after_wait_s)
            if odom_buffer is not None:
                header['odom_sync'] = odom_buffer.snapshot(started, received, a.odom_wait_s)
        except Exception as exc:
            payload = b''
            header = {'ok': False, 'sequence': sequence, 'bytes': 0, 'error': repr(exc),
                      'read_only': True, 'exposure_timestamp_verified': False}
        packed = json.dumps(header).encode('utf-8')
        wire.write(struct.pack('!I', len(packed)))
        wire.write(packed)
        wire.write(payload)


if __name__ == '__main__':
    main()
