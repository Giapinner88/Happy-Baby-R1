"""Camera surface points -> named R1 body frame using measured joints.

Reuses project URDF joint parsing/FK. Nominal mount results remain explicitly
diagnostic; no world/odom translation or exposure synchronization is invented.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from teleop.r1.kinematics import load_joint_geometry, joint_transform


class CameraFrames:
    def __init__(self, config, root):
        self.config = dict(config)
        self.path = Path(root)/config['urdf']
        tree = ET.parse(self.path)
        previous = config['base_frame']
        self.joints = []
        for name in config['joint_chain']:
            element = tree.find(".//joint[@name='%s']" % name)
            if element is None or element.find('parent').get('link') != previous:
                raise ValueError('disconnected camera chain: '+name)
            previous = element.find('child').get('link')
            self.joints.append(load_joint_geometry(name, self.path))
        if previous != config['mount_parent_frame']:
            raise ValueError('camera mount parent differs from chain endpoint')
        R = np.asarray(config['optical_to_mount_rotation'], float)
        t = np.asarray(config['camera_origin_in_mount_m'], float)
        if (R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all() or
                not np.isfinite(t).all() or not np.allclose(R.T@R, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(R), 1.)):
            raise ValueError('mount must be a finite rigid transform')
        if type(config['mount_verified']) is not bool:
            raise ValueError('mount_verified must be boolean')
        for key in ('max_state_gap_s', 'max_bracket_span_s', 'max_joint_speed_rad_s'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid '+key)
        self.mount = np.eye(4)
        self.mount[:3, :3], self.mount[:3, 3] = R, t

    def transform(self, q):
        T = np.eye(4)
        for joint in self.joints:
            value = q[joint.name]
            if not math.isfinite(value) or not joint.lower_rad <= value <= joint.upper_rad:
                raise ValueError('joint outside finite URDF limits: '+joint.name)
            T = T@joint_transform(joint, value)
        return T@self.mount

    def pose_only(self, sync):
        # Reuse identical timestamp/joint/frame checks, without a depth target.
        result=self.project([0.,0.,1.],sync)
        if result['status']=='diagnostic_proposal':
            result['status']='diagnostic_pose'
            for key in ('estimated_point_body_m','bearing_body_rad','planar_range_body_xy_m','body_range_m'):
                result.pop(key,None)
            result['note']='Camera/body pose validation only; no new target depth or point.'
        return result

    def project(self, point, sync):
        result = {'status': 'unavailable', 'frame': self.config['base_frame'],
                  'estimated_point_body_m': None, 'base_position_m': None,
                  'odom_position_m': None, 'navigation_ready': False,
                  'mount_verified': self.config['mount_verified'],
                  'exposure_synchronized': False}
        try:
            if not sync:
                raise ValueError('missing LowState/camera association')
            before, after = sync['state_before_rpc'], sync['state_after_rpc']
            state = sync['state_near_rpc_midpoint_proxy']
            if before is None or after is None or state is None:
                raise ValueError('missing bracketing state')
            start, end = sync['rpc_start_monotonic_s'], sync['rpc_end_monotonic_s']
            midpoint = (start+end)/2
            times = [start, end, state['received_monotonic_s'],
                     before['received_monotonic_s'], after['received_monotonic_s']]
            if not all(math.isfinite(v) for v in times) or end < start:
                raise ValueError('invalid camera/state timestamps')
            if not before['received_monotonic_s'] <= start <= end <= after['received_monotonic_s']:
                raise ValueError('state does not bracket RPC')
            if not before['received_monotonic_s'] <= state['received_monotonic_s'] <= after['received_monotonic_s']:
                raise ValueError('nearest state outside bracket')
            gap = abs(state['received_monotonic_s']-midpoint)
            if gap > self.config['max_state_gap_s']:
                raise ValueError('state too far from RPC midpoint proxy')
            bracket_span = after['received_monotonic_s']-before['received_monotonic_s']
            if bracket_span > self.config['max_bracket_span_s']:
                raise ValueError('state bracket span too large')
            speeds = [state['dq_rad_s'][j.name] for j in self.joints]
            if not all(math.isfinite(v) and abs(v) <= self.config['max_joint_speed_rad_s'] for v in speeds):
                raise ValueError('joint speed exceeds capture assumption')
            quaternion = np.asarray(state['imu_quaternion_wxyz'], float)
            if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or not .98 <= np.linalg.norm(quaternion) <= 1.02:
                raise ValueError('invalid IMU quaternion')
            point = np.asarray(point, float)
            if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0:
                raise ValueError('invalid forward optical point')
            T = self.transform(state['q_rad'])
            transformed = T[:3, :3]@point+T[:3, 3]
            result.update(status='diagnostic_proposal', estimated_point_body_m=transformed.tolist(),
                          T_body_camera=T.tolist(), state_rpc_proxy_gap_s=gap,
                          state_bracket_span_s=bracket_span,
                          bearing_body_rad=math.atan2(transformed[1], transformed[0]),
                          planar_range_body_xy_m=float(np.hypot(*transformed[:2])),
                          body_range_m=float(np.linalg.norm(transformed)),
                          joint_q_rad={j.name: state['q_rad'][j.name] for j in self.joints},
                          imu_quaternion_wxyz=quaternion.tolist(),
                          note='Receiver-time association only; IMU recorded but not used as odometry. Base output withheld until independent validation.')
        except (KeyError, TypeError, ValueError) as exc:
            result['reason'] = str(exc)
        return result
