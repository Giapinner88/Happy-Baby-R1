"""Bounded read-only vendor odometry receiver on robot Python 3.8.

Vendor frame is assumed pelvis for exploratory integration, not calibrated.
Receive time shares the camera worker clock; sensor stamp preserved separately.
"""
from collections import deque
import math
import threading
import time


class OdomBuffer:
    def __init__(self, capacity=100, topic='rt/lf/odommodestate'):
        self.topic = topic
        self.samples = deque(maxlen=capacity)
        self.lock = threading.Condition()
        self.rejected = 0

    def add_message(self, message):
        try:
            p = [float(v) for v in message.position]
            q = [float(v) for v in message.imu_state.quaternion]
            if len(p) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in p+q) or not .98 <= math.sqrt(sum(v*v for v in q)) <= 1.02:
                raise ValueError('invalid vendor pose')
            w,x,y,z = q
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            sample = {'stamp': time.monotonic(), 'x_m': p[0], 'y_m': p[1],
                      'z_m': p[2], 'yaw_rad': yaw, 'frame': 'vendor_odom',
                      'child_frame': 'pelvis_link', 'frame_verified': False,
                      'clock': 'robot_receive_monotonic',
                      'source': self.topic, 'source_type': 'unitree_go/SportModeState',
                      'vendor_stamp': {'sec': int(message.stamp.sec), 'nanosec': int(message.stamp.nanosec)},
                      'imu_quaternion_wxyz': q}
            with self.lock:
                self.samples.append(sample)
                self.lock.notify_all()
        except (AttributeError, IndexError, TypeError, ValueError):
            with self.lock:
                self.rejected += 1

    def latest(self):
        with self.lock:
            return None if not self.samples else dict(self.samples[-1])

    def snapshot(self, start, end, wait_s=0.):
        deadline = time.monotonic()+wait_s
        with self.lock:
            midpoint = (start+end)/2
            while not self.samples or self.samples[-1]['stamp'] <= midpoint:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    break
                self.lock.wait(remaining)
            samples = list(self.samples)
            before = [s for s in samples if s['stamp'] <= midpoint]
            after = [s for s in samples if s['stamp'] > midpoint]
            # Two samples suffice for interpolation; do not send a 500-Hz
            # trajectory through the camera transport on every frame.
            return {'poses': before[-1:]+after[:1], 'rejected': self.rejected,
                    'frame_verified': False, 'exposure_synchronized': False}
