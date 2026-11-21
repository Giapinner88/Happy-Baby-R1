"""Seeded red-region association baseline; not a semantic object detector.

Experimental offline mask tracking for the confirmed red extinguisher. Does
not authorize navigation or infer that any arbitrary red region is that object.
"""
import math
import cv2
import numpy as np


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x1-x0)*max(0, y1-y0)
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union else 0.


class RedTarget:
    def __init__(self, config):
        self.config = dict(config)
        self.box = tuple(config['seed_bbox_xyxy_pixels'])
        if len(self.box) != 4 or self.box[0] >= self.box[2] or self.box[1] >= self.box[3]:
            raise ValueError('invalid seed box')
        self.last_stamp = None
        self.lost = False
        for name in ('min_area_px', 'max_center_step_px', 'max_age_s', 'max_gap_s'):
            if not math.isfinite(config[name]) or config[name] <= 0:
                raise ValueError('invalid '+name)
        if not 0 < config['min_iou'] <= 1 or not 0 <= config['ambiguity_iou_gap'] <= 1:
            raise ValueError('invalid IoU thresholds')
        for name in ('close_kernel_px', 'erode_kernel_px'):
            if type(config[name]) is not int or config[name] < 1 or config[name]%2 != 1:
                raise ValueError('kernel must be positive odd integer')
        if not 0 <= config['hue_low_max'] < config['hue_high_min'] <= 179:
            raise ValueError('invalid hue thresholds')
        if any(not 0 <= config[name] <= 255 for name in ('saturation_min', 'value_min')):
            raise ValueError('invalid saturation/value threshold')

    def observe(self, image, stamp, now, quality_ok=True):
        c = self.config
        if self.lost:
            return {'status': 'lost_latched'}, None
        reason = None
        if not quality_ok:
            reason = 'input_quality'
        elif not math.isfinite(stamp) or not math.isfinite(now) or not 0 <= now-stamp <= c['max_age_s']:
            reason = 'stale_or_future'
        elif self.last_stamp is not None and not 0 < stamp-self.last_stamp <= c['max_gap_s']:
            reason = 'out_of_order_or_gap'
        if reason:
            self.lost = True
            return {'status': 'lost', 'reason': reason}, None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = (((hsv[..., 0] <= c['hue_low_max']) | (hsv[..., 0] >= c['hue_high_min'])) &
                (hsv[..., 1] >= c['saturation_min']) & (hsv[..., 2] >= c['value_min'])).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((c['close_kernel_px'],)*2, np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        candidates = []
        for label in range(1, count):
            x, y, w, h, area = stats[label]
            box = (int(x), int(y), int(x+w), int(y+h))
            shift = math.hypot((box[0]+box[2]-self.box[0]-self.box[2])/2,
                               (box[1]+box[3]-self.box[1]-self.box[3])/2)
            overlap = iou(box, self.box)
            if area >= c['min_area_px'] and shift <= c['max_center_step_px'] and overlap >= c['min_iou']:
                candidates.append((overlap, label, box))
        candidates.sort(reverse=True)
        if not candidates or (len(candidates) > 1 and candidates[0][0]-candidates[1][0] <= c['ambiguity_iou_gap']):
            self.lost = True
            return {'status': 'lost', 'reason': 'missing_or_ambiguous_region'}, None
        score, label, box = candidates[0]
        interior = cv2.erode((labels == label).astype(np.uint8),
                             np.ones((c['erode_kernel_px'],)*2, np.uint8)).astype(bool)
        if interior.sum() < c['min_area_px']:
            self.lost = True
            return {'status': 'lost', 'reason': 'insufficient_interior'}, None
        self.box, self.last_stamp = box, stamp
        return {'status': 'tracked', 'target_id': c['target_id'], 'bbox_xyxy_pixels': list(box),
                'association_iou': score, 'mask_pixels': int(interior.sum()),
                'identity_basis': 'seeded_color_and_overlap_only', 'navigation_ready': False}, interior


def masked_range(ranges, valid, mask, min_valid_fraction):
    if ranges.shape != valid.shape or valid.shape != mask.shape:
        raise ValueError('registered arrays must have identical shape')
    if not 0 < min_valid_fraction <= 1 or not mask.any():
        raise ValueError('invalid mask or support threshold')
    support = mask & valid & np.isfinite(ranges) & (ranges > 0)
    fraction = float(support.sum()/mask.sum())
    if fraction < min_valid_fraction:
        return {'status': 'invalid_depth', 'valid_fraction': fraction, 'range_m': None}
    values = ranges[support]
    return {'status': 'estimated', 'valid_fraction': fraction,
            'range_m': float(np.median(values)),
            'p10_m': float(np.percentile(values, 10)), 'p90_m': float(np.percentile(values, 90))}
