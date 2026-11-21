"""Read-only SE(2) target registration, metres/radians, CCW yaw.

Pose samples must describe pelvis_link in odom, on the remote camera clock.
No IMU dead reckoning, clock conversion, frame aliasing or motor authority.
"""
from ai_modules.vision.range_window import range_ready
import math
import numpy as np


class TargetOdom:
    def __init__(self, config):
        self.config = dict(config)
        for key in ('max_pose_gap_s', 'max_pose_bracket_s', 'max_target_jump_m', 'max_observation_age_s'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid '+key)
        if config['body_frame'] != 'pelvis_link' or config['clock'] != 'robot_receive_monotonic':
            raise ValueError('requires pelvis_link and robot_receive_monotonic')
        self.pinned = None
        self.last_stamp = None
        self.follow_pose=None
        self.initial=[]
        self.depth_usage=config.get('depth_usage','continuous_validation')
        if self.depth_usage not in ('continuous_validation','initial_median_then_diagnostic'):raise ValueError('invalid depth usage')
        if self.depth_usage=='initial_median_then_diagnostic':
            if type(config.get('initial_samples')) is not int or config['initial_samples']<3 or config['initial_samples']>9 or config['initial_samples']%2!=1:raise ValueError('initial_samples must be odd3..9')
            if not math.isfinite(config.get('initial_max_span_s',float('nan'))) or not 0<config['initial_max_span_s']<=3:raise ValueError('invalid initial span')

    def follow_pin(self,sample,now):
        """Static initial target + current measured odom, no camera/IK update."""
        result={'status':'unavailable','navigation_ready':False}
        try:
            if self.pinned is None:raise ValueError('initial target not pinned')
            if sample['frame']!=self.config['odom_frame'] or sample['child_frame']!=self.config['body_frame'] or sample['clock']!=self.config['clock']:raise ValueError('odom frame or clock mismatch')
            if not all(math.isfinite(v) for v in [now,sample['stamp'],sample['x_m'],sample['y_m'],sample['yaw_rad']]):raise ValueError('invalid odom sample')
            if not 0<=now-sample['stamp']<=self.config['max_observation_age_s']:raise ValueError('odom expired')
            signature=(sample['x_m'],sample['y_m'],sample['yaw_rad'])
            duplicate=self.last_stamp is not None and sample['stamp']==self.last_stamp
            if self.last_stamp is not None and sample['stamp']<self.last_stamp:raise ValueError('nonmonotonic odom')
            if duplicate and signature!=self.follow_pose:raise ValueError('conflicting duplicate odom')
            self.last_stamp=sample['stamp']
            self.follow_pose=signature
            result.update(status='diagnostic_registered',target_id=self.pinned['identity'],
                          frame=self.config['odom_frame'],pinned_point_odom_m=list(self.pinned['point']),
                          estimated_point_odom_m=None,robot_pose_xyyaw=[sample['x_m'],sample['y_m'],sample['yaw_rad']],
                          position_source='initial_pin_and_fresh_odom',visual_revalidation=False,
                          held_duplicate_odom=duplicate,
                          initial_median_range_m=self.pinned.get('median_range_m'),
                          initial_sample_stamps=[s['stamp'] for s in self.initial])
        except (KeyError,TypeError,ValueError) as exc:result['reason']=str(exc)
        return result

    def register(self, event, poses):
        result = {'status': 'unavailable', 'navigation_ready': False,
                  'frame': self.config['odom_frame'], 'estimated_point_odom_m': None,
                  'pinned_point_odom_m': None if self.pinned is None else self.pinned['point'],
                  'target_id': None if self.pinned is None else self.pinned['identity']}
        try:
            track = event['tracker'] or {}
            body = event['body_frame_proposal'] or {}
            tracking_only=event['temporal'].get('status')=='tracked_pinned_target'
            if tracking_only and (self.pinned is None or self.depth_usage!='initial_median_then_diagnostic'):
                raise ValueError('tracking-only requires initial pinned target')
            if track.get('status') != 'tracked' or not range_ready(event['temporal']):
                raise ValueError('target observation unavailable')
            age = event['request_to_publish_s']
            if not math.isfinite(age) or not 0 <= age <= self.config['max_observation_age_s']:
                raise ValueError('observation expired')
            identity = track['target_id']
            if not isinstance(identity, str) or not identity:
                raise ValueError('missing identity')
            if self.pinned is not None and identity != self.pinned['identity']:
                raise ValueError('identity changed; explicit reset required')
            if self.initial and identity!=self.initial[0]['identity']:raise ValueError('initial identity changed')
            if body.get('status') != ('diagnostic_pose' if tracking_only else 'diagnostic_proposal') or body.get('frame') != self.config['body_frame']:
                raise ValueError('body transform unavailable or frame mismatch')
            capture = event['capture']
            stamp = (capture['remote_requested_monotonic_s']+capture['remote_received_monotonic_s'])/2
            if not math.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
                raise ValueError('nonmonotonic observation')
            point = None if tracking_only else np.asarray(body['estimated_point_body_m'], float)
            if not tracking_only and (point.shape != (3,) or not np.isfinite(point).all()):
                raise ValueError('invalid body point')
            # Only a bounded buffer is supplied by the caller; no extrapolation.
            before = after = None
            previous = None
            for pose in poses:
                if (pose['frame'] != self.config['odom_frame'] or pose['child_frame'] != self.config['body_frame']
                        or pose['clock'] != self.config['clock']):
                    raise ValueError('pose frame or clock mismatch')
                values = [pose[k] for k in ('stamp', 'x_m', 'y_m', 'z_m', 'yaw_rad')]
                if not all(math.isfinite(v) for v in values) or (previous is not None and pose['stamp'] <= previous):
                    raise ValueError('invalid or unordered pose')
                previous = pose['stamp']
                if pose['stamp'] <= stamp:
                    before = pose
                if pose['stamp'] >= stamp and after is None:
                    after = pose
            if before is None or after is None:
                raise ValueError('pose does not bracket camera')
            span = after['stamp']-before['stamp']
            gap = min(stamp-before['stamp'], after['stamp']-stamp)
            if span > self.config['max_pose_bracket_s'] or gap > self.config['max_pose_gap_s']:
                raise ValueError('stale pose bracket')
            alpha = (stamp-before['stamp'])/span if span else 0.
            delta = math.atan2(math.sin(after['yaw_rad']-before['yaw_rad']), math.cos(after['yaw_rad']-before['yaw_rad']))
            yaw = before['yaw_rad']+alpha*delta
            translation = np.array([before[k]+alpha*(after[k]-before[k]) for k in ('x_m', 'y_m', 'z_m')])
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            world = None if tracking_only else rotation@point+translation
            innovation=None if self.pinned is None or world is None else float(np.linalg.norm(world-np.array(self.pinned['point'])))
            if self.depth_usage=='continuous_validation' and innovation is not None and innovation > self.config['max_target_jump_m']:
                raise ValueError('target innovation exceeds limit')
            if self.pinned is None:
                if self.depth_usage=='initial_median_then_diagnostic':
                    measured_range=event['depth']['range_m']
                    if not math.isfinite(measured_range) or measured_range<=0:raise ValueError('invalid initial range')
                    self.initial=[s for s in self.initial if stamp-s['stamp']<=self.config['initial_max_span_s']]
                    self.initial.append({'identity':identity,'stamp':stamp,'world':world.tolist(),
                                         'range_m':measured_range})
                    self.last_stamp=stamp
                    if len(self.initial)<self.config['initial_samples']:
                        result.update(status='initializing_target',reason='initial_median_warming_up',
                                      initial_samples=len(self.initial),required_samples=self.config['initial_samples'])
                        return result
                    self.pinned={'identity':identity,'point':np.median([s['world'] for s in self.initial],axis=0).tolist(),
                                 'median_range_m':float(np.median([s['range_m'] for s in self.initial]))}
                else:self.pinned = {'identity': identity, 'point': world.tolist()}
            self.last_stamp = stamp
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, translation
            result.update(status='diagnostic_registered', target_id=identity,
                          T_odom_body=transform.tolist(), robot_pose_xyyaw=[float(translation[0]), float(translation[1]), yaw],
                          estimated_point_odom_m=None if world is None else world.tolist(), pinned_point_odom_m=self.pinned['point'],
                          pose_bracket_span_s=span, pose_gap_s=gap,
                          depth_usage=self.depth_usage,depth_innovation_m=innovation,
                          live_depth_diagnostic_only=self.depth_usage=='initial_median_then_diagnostic',
                          initial_median_range_m=self.pinned.get('median_range_m'),
                          initial_sample_stamps=[s['stamp'] for s in self.initial],
                          position_source='initial_pin_and_fresh_odom' if tracking_only else 'live_depth',
                          note='Pinned diagnostic only; planar pelvis pose assumed, roll/pitch ignored; no map or motor authority')
        except (KeyError, TypeError, ValueError) as exc:
            result['reason'] = str(exc)
        return result
