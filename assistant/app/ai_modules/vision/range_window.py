"""Bounded temporal range consistency for observation, never motion authority.

Thresholds are experimental configuration, not a calibrated accuracy model.
No previous estimate is emitted after stale/invalid/outlier observations.
"""
from collections import deque
import math
import statistics


def range_ready(value):
    if value.get('status')=='pinned_target_odometry':
        return value.get('measurement_mode')=='odometry_only' and value.get('has_pinned_target') is True
    if value.get('status')=='tracked_pinned_target':
        return value.get('measurement_mode')=='tracking_only' and value.get('has_pinned_target') is True
    if value.get('status')=='consistent_window':return True
    distance=value.get('range_m')
    return (value.get('status')=='single_frame_estimate' and value.get('measurement_mode')=='single_frame'
            and value.get('samples')==1 and type(distance) in (int,float)
            and math.isfinite(distance) and distance>0)


class RangeWindow:
    def __init__(self, config):
        self.config = dict(config)
        if type(config.get('reject_jump',True)) is not bool:raise ValueError('reject_jump must be boolean')
        self.single=config.get('mode','temporal')=='single_frame'
        if config.get('mode','temporal') not in ('temporal','single_frame'):raise ValueError('invalid range mode')
        if type(config['samples']) is not int or (config['samples']!=1 if self.single else config['samples']<2):
            raise ValueError('samples must be integer >=2')
        for key in ('max_age_s', 'max_gap_s', 'max_window_span_m', 'max_jump_m'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid '+key)
        self.values = deque(maxlen=config['samples'])
        self.last_stamp = None

    def clear(self, reason):
        self.values.clear()
        return {'status': reason, 'range_m': None, 'samples': 0,
                'navigation_ready': False}

    def update(self, stamp, now, value):
        c = self.config
        if not math.isfinite(stamp) or not math.isfinite(now) or not 0 <= now-stamp <= c['max_age_s']:
            return self.clear('stale_or_future')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return self.clear('out_of_order')
        if self.last_stamp is not None and stamp-self.last_stamp > c['max_gap_s']:
            self.values.clear()
        self.last_stamp = stamp
        if value is None or not math.isfinite(value) or value <= 0:
            return self.clear('invalid_depth')
        if c.get('reject_jump',True) and self.values and abs(value-statistics.median(v for _, v in self.values)) > c['max_jump_m']:
            return self.clear('range_jump')
        self.values.append((stamp, value))
        values = [v for _, v in self.values]
        span = max(values)-min(values)
        result = {'status': 'warming_up', 'range_m': None, 'samples': len(values),
                  'raw_window_span_m': span, 'window_age_s': now-self.values[0][0],
                  'measurement_age_s': now-stamp, 'navigation_ready': False}
        if self.single:
            result.update(status='single_frame_estimate',range_m=value,measurement_mode='single_frame',temporal_consistency_verified=False)
            return result
        if len(values) == c['samples']:
            if span > c['max_window_span_m']:
                result['status'] = 'unstable_window'
            else:
                result['status'] = 'consistent_window'
                result['range_m'] = statistics.median(values)
                result['median_absolute_deviation_m'] = statistics.median(
                    abs(v-result['range_m']) for v in values)
        return result
