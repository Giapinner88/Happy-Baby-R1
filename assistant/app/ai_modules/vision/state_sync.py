"""Read-only LowState receive-time buffer, usable on robot Python 3.8.

Samples share the camera worker's clock. RPC midpoint is a proxy, not the
camera exposure time. Joint IDs follow the repository R1 RobotSpec mapping.
"""
from collections import deque
import math
import threading
import time

JOINT_IDS = {'waist_roll_joint': 12, 'waist_yaw_joint': 13,
             'head_pitch_joint': 29, 'head_yaw_joint': 30}


class StateBuffer:
    def __init__(self, capacity=500):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('capacity must be a positive integer')
        self.samples = deque(maxlen=capacity)
        self.condition = threading.Condition()
        self.rejected = 0

    def add(self, sample):
        try:
            if type(sample['robot_tick']) is not int or not 0 <= sample['robot_tick'] < 2**32:
                raise ValueError('invalid robot tick')
            values = [sample['received_monotonic_s']]
            for key in ('q_rad', 'dq_rad_s'):
                values.extend(sample[key][name] for name in JOINT_IDS)
            values.extend(sample['imu_quaternion_wxyz'])
            values.extend(sample['imu_gyroscope_rad_s'])
            if (len(sample['imu_quaternion_wxyz']) != 4 or len(sample['imu_gyroscope_rad_s']) != 3
                    or not all(math.isfinite(v) for v in values)):
                raise ValueError('invalid sample')
        except (KeyError, TypeError, ValueError):
            with self.condition:
                self.rejected += 1
            return False
        with self.condition:
            if self.samples and (sample['robot_tick'] == self.samples[-1]['robot_tick'] or
                                 sample['received_monotonic_s'] <= self.samples[-1]['received_monotonic_s']):
                self.rejected += 1
                return False
            self.samples.append(sample)
            self.condition.notify_all()
        return True

    def wait_ready(self, timeout_s):
        deadline = time.monotonic()+timeout_s
        with self.condition:
            while not self.samples:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return False
                self.condition.wait(remaining)
        return True

    def capture_record(self, start, end, wait_after_s):
        deadline = time.monotonic()+wait_after_s
        with self.condition:
            while not self.samples or self.samples[-1]['received_monotonic_s'] < end:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            samples = list(self.samples)
            rejected = self.rejected
        midpoint = (start+end)/2
        before = [s for s in samples if s['received_monotonic_s'] <= start]
        after = [s for s in samples if s['received_monotonic_s'] >= end]
        near = min(samples, key=lambda s: abs(s['received_monotonic_s']-midpoint)) if samples else None
        return {'rpc_start_monotonic_s': start, 'rpc_end_monotonic_s': end,
                'rpc_midpoint_proxy_monotonic_s': midpoint,
                'state_near_rpc_midpoint_proxy': near,
                'state_before_rpc': before[-1] if before else None,
                'state_after_rpc': after[0] if after else None,
                'state_rpc_proxy_gap_s': abs(near['received_monotonic_s']-midpoint) if near else None,
                'buffer_samples': len(samples), 'rejected_samples': rejected,
                'joint_mapping': JOINT_IDS, 'exposure_synchronized': False,
                'timestamp_basis': 'remote Python receive monotonic, not sensor/exposure timestamps'}
