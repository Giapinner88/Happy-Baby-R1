"""Gait-aware arc approach; large errors use measured turn/settle feedback.

Consumes collision-checked Approach proposals. No transport or authority.
Coordinates are odom metres and CCW radians. Deadbands come from the active
flat_plus_gait policy; crossing them is a request, never proof of motion.
"""
import math
from ai_modules.navigation.turn_approach import wrap


class GaitAwareApproach:
    def __init__(self, config, *, preview=False):
        self.config = dict(config)
        self.preview = preview
        caps = {'max_v_mps': .16, 'max_w_radps': .3,
                'max_accel_mps2': .1, 'max_yaw_accel_radps2': .3,
                'max_source_age_s': .6, 'max_duration_s': 120.,
                'max_path_m': 5., 'heading_gain': 2., 'arc_heading_rad': .3,
                'turn_tolerance_rad': .05, 'settle_min_s': 2.,
                'settle_rate_radps': .1, 'max_measured_yaw_rate_radps': .5,
                'max_turn_duration_s': 60.}
        for key, cap in caps.items():
            v = config[key]
            if type(v) not in (float, int) or not math.isfinite(v) or not 0 < v <= cap:
                raise ValueError('invalid gait approach ' + key)
        if config['linear_move_threshold_mps'] != .15 or config['angular_move_threshold_radps'] != .15:
            raise ValueError('active policy gait contract mismatch')
        if not .15 < config['turn_speed_radps'] <= config['max_w_radps']:
            raise ValueError('turn speed must exceed strict gait threshold')
        if config['max_v_mps'] <= .15:
            raise ValueError('forward cap cannot activate gait')
        if not 0 < config['turn_tolerance_rad'] < config['arc_heading_rad']:
            raise ValueError('invalid heading hysteresis')
        if type(config['settle_samples']) is not int or not 3 <= config['settle_samples'] <= 10:
            raise ValueError('invalid settle samples')
        self.start = self.last_time = self.stamp = self.pose = self.identity = None
        self.phase = 'DRIVE'
        self.vx = self.wz = self.path = 0.
        self.settle_start = self.turn_start = None
        self.good = 0

    def command(self, status, now):
        c = self.config
        if status['state'] not in ('APPROACH', 'ALIGN'):
            raise ValueError('controller ' + status['state'])
        if not self.preview and status.get('declared_corridor_confirmed') is not True:
            raise ValueError('clear corridor not confirmed')
        age = status['source_age_s']
        stamp = status['measurement_stamp_monotonic_s']
        pose, goal = status['pose_xyyaw'], status['goal_xy_m']
        if len(pose) != 3 or len(goal) != 2 or not all(math.isfinite(v) for v in [now, age, stamp, *pose, *goal, status['expires_at_local_monotonic_s']]):
            raise ValueError('invalid approach geometry/clock')
        if not 0 <= age <= c['max_source_age_s'] or not 0 <= now-stamp <= c['max_source_age_s'] or now >= status['expires_at_local_monotonic_s']:
            raise ValueError('stale approach observation')
        if self.start is None:
            self.start = self.last_time = now
            self.identity = status['target_id']
        if status['target_id'] != self.identity:
            raise ValueError('target changed')
        dt = now-self.last_time
        if not 0 <= dt <= .1:
            raise ValueError('control gap')
        if now-self.start >= c['max_duration_s']:
            raise ValueError('task budget exhausted')
        fresh = self.stamp is None or stamp > self.stamp+1e-7
        rate = None
        if self.stamp is not None and stamp < self.stamp-1e-7:
            raise ValueError('out-of-order measurement')
        if fresh:
            if self.pose is not None:
                self.path += math.dist(pose[:2], self.pose[:2])
                rate = wrap(pose[2]-self.pose[2])/(stamp-self.stamp)
                if abs(rate) > c['max_measured_yaw_rate_radps']:
                    raise ValueError('measured yaw rate guard')
            self.stamp, self.pose = stamp, list(pose)
        if self.path > c['max_path_m']:
            raise ValueError('path budget exhausted')
        heading = wrap(math.atan2(goal[1]-pose[1], goal[0]-pose[0])-pose[2])
        if self.phase == 'DRIVE' and abs(heading) > c['arc_heading_rad']:
            self.phase = 'TURN'
            self.turn_start = now
        vx = wz = 0.
        if self.phase == 'TURN':
            if now-self.turn_start >= c['max_turn_duration_s']:
                raise ValueError('turn budget exhausted')
            if abs(heading) <= c['turn_tolerance_rad']:
                self.phase = 'SETTLE'
                self.settle_start, self.good = now, 0
            else:
                # Constant pulse amplitude avoids sub-threshold P-control stall.
                desired = math.copysign(c['turn_speed_radps'], heading)
                step = c['max_yaw_accel_radps2']*dt
                wz = max(self.wz-step, min(self.wz+step, desired))
        if self.phase == 'SETTLE':
            if fresh and stamp > self.settle_start:
                self.good = self.good+1 if rate is not None and abs(rate) <= c['settle_rate_radps'] else 0
            if self.good >= c['settle_samples'] and now-self.settle_start >= c['settle_min_s']:
                if abs(heading) > c['arc_heading_rad']:
                    self.phase, self.turn_start = 'TURN', now
                else:
                    self.phase = 'DRIVE'
        elif self.phase == 'DRIVE':
            requested = status['proposal']['vx_mps']
            if not math.isfinite(requested) or not 0 <= requested <= c['max_v_mps']+1e-6:
                raise ValueError('proposal exceeds envelope')
            # Never increase a collision/age-limited speed to activate gait.
            if self.vx > .15 and requested <= .15:
                raise ValueError('safe proposal below gait envelope')
            vx = min(requested, c['max_v_mps'], self.vx+c['max_accel_mps2']*dt)
            desired = max(-c['max_w_radps'], min(c['max_w_radps'], c['heading_gain']*heading))
            step = c['max_yaw_accel_radps2']*dt
            wz = max(self.wz-step, min(self.wz+step, desired))
        self.last_time, self.vx, self.wz = now, vx, wz
        return {'vx_mps': vx, 'wz_radps': wz, 'phase': self.phase,
                'heading_error_rad': heading, 'path_m': self.path,
                'settle_samples': self.good}
