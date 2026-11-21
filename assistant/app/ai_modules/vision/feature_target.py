"""Grounding-seeded, colour-independent local feature tracking.

Pins the selected instance ID. LK + forward/backward + RANSAC + fixed
appearance checks gate updates. Optional bounded flow recovery preserves the
last verified frame; hard failures still latch until explicit reselection.
The depth support is an inset bbox proxy, NOT semantic segmentation.
"""
import math
import cv2
import numpy as np


class FeatureTarget:
    def __init__(self, config, reference, box):
        self.config = dict(config)
        self.box = tuple(map(int, box))
        h, w = reference.shape[:2]
        x0, y0, x1, y1 = self.box
        if not (0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h):
            raise ValueError('seed box outside reference image')
        for key in ('min_features', 'max_features', 'lk_window_px', 'lk_levels'):
            if type(config[key]) is not int or config[key] < 1:
                raise ValueError('invalid '+key)
        if config['min_features'] > config['max_features']:
            raise ValueError('min_features exceeds max_features')
        for key in ('feature_quality', 'min_inlier_ratio', 'min_template_score'):
            if not 0 < config[key] <= 1:
                raise ValueError('invalid '+key)
        for key in ('feature_distance_px', 'max_fb_error_px', 'ransac_error_px',
                    'max_step_px', 'max_age_s', 'max_gap_s'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid '+key)
        if not 0 < config['scale_min'] <= 1 <= config['scale_max']:
            raise ValueError('invalid scale bounds')
        if not 0 <= config['depth_inset_fraction'] < .5:
            raise ValueError('invalid depth inset')
        self.previous = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        self.template = self.previous[y0:y1, x0:x1].copy()
        xx, yy = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        self.template_coordinates = np.stack((xx, yy, np.ones_like(xx)), axis=-1).astype(np.float32)
        self.cumulative = np.eye(3)
        if self.template.std() < 2:
            raise ValueError('reference object lacks texture')
        self.corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
        self.points = self.features(self.previous, self.mask(reference.shape[:2], inset=0.))
        if self.points is None or len(self.points) < config['min_features']:
            raise ValueError('insufficient reference features')
        self.last_stamp = None
        self.lost = False
        self.recovery_frames = config.get('recovery_max_frames',0)
        self.recovery_s = config.get('recovery_max_s',min(.6,config['max_gap_s']))
        if type(self.recovery_frames) is not int or not 0<=self.recovery_frames<=5:
            raise ValueError('invalid recovery_max_frames')
        if type(self.recovery_s) not in (int,float) or not math.isfinite(self.recovery_s) or not 0<self.recovery_s<=config['max_gap_s']:
            raise ValueError('invalid recovery_max_s')
        self.recovery_count=0;self.recovery_start=None;self.last_input_stamp=None
        # Optional drift-bounded template refresh (0 = off, original behaviour). The fixed seed template decays
        # as the object grows/changes view while walking (score 1.0 -> 0.67 in 3/6 hardware runs). When the match to
        # the CURRENT template is >= refresh score the current template is replaced by the aligned patch; identity is
        # still anchored by a hard floor on the NCC to the ORIGINAL seed template. Refresh is not semantic re-ID.
        self.refresh_score=config.get('template_refresh_score',0.)
        self.seed_floor=config.get('seed_floor_score',.3)
        if type(self.refresh_score) not in (int,float) or not (self.refresh_score==0 or .8<=self.refresh_score<=1):
            raise ValueError('invalid template_refresh_score')
        if type(self.seed_floor) not in (int,float) or not 0<=self.seed_floor<1:raise ValueError('invalid seed_floor_score')
        self.seed_template=self.template.copy();self.frames_since_refresh=0;self.refreshes=0
        # Treat a single geometry-gate failure as recoverable (bounded by recovery_max_frames/recovery_max_s).
        self.recover_geometry=bool(config.get('recover_geometry',False))

    def features(self, gray, mask):
        return cv2.goodFeaturesToTrack(gray, maxCorners=self.config['max_features'],
                                      qualityLevel=self.config['feature_quality'],
                                      minDistance=self.config['feature_distance_px'], mask=mask)

    def mask(self, shape, inset=None):
        if inset is None:
            inset = self.config['depth_inset_fraction']
        centre = self.corners.mean(axis=0)
        polygon = centre+(self.corners-centre)*(1-2*inset)
        mask = np.zeros(shape, np.uint8)
        cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
        return mask

    def fail(self, reason, **diagnostic):
        self.lost = True
        return {'status': 'lost', 'reason': reason, 'target_id': self.config['target_id'],
                'navigation_ready': False, **diagnostic}, None

    def observe(self, image, stamp, now, quality_ok=True):
        if not self.recovery_frames:return self._observe(image,stamp,now,quality_ok)
        if self.lost:return self._observe(image,stamp,now,quality_ok)
        if self.last_input_stamp is not None and (not math.isfinite(stamp) or stamp<=self.last_input_stamp):
            return self.fail('out_of_order_input')
        self.last_input_stamp=stamp
        # Never commit geometry/features from an unverified frame, including
        # failures detected after updating the candidate corners.
        saved=(self.previous,self.points,self.corners,self.cumulative,self.last_stamp,self.box)
        result,mask=self._observe(image,stamp,now,quality_ok)
        if result['status']=='tracked':
            result['recovered_after_frames']=self.recovery_count
            self.recovery_count=0;self.recovery_start=None
            return result,mask
        soft=result.get('reason') in ('forward_flow_failed','backward_flow_failed',
                                     'insufficient_consistent_features','insufficient_features_after_update') or (
                                     self.recover_geometry and result.get('reason')=='geometry_gate')
        if not soft:return result,mask
        self.previous,self.points,self.corners,self.cumulative,self.last_stamp,self.box=saved
        if self.recovery_start is None:self.recovery_start=stamp
        self.recovery_count+=1
        result.update(recovery_attempt=self.recovery_count,recovery_age_s=stamp-self.recovery_start)
        if self.recovery_count<=self.recovery_frames and stamp-self.recovery_start<=self.recovery_s:
            self.lost=False;result['status']='recovering'
        else:result['recovery_exhausted']=True
        return result,None

    def _observe(self, image, stamp, now, quality_ok=True):
        c = self.config
        if self.lost:
            return {'status': 'lost_latched', 'target_id': c['target_id'], 'navigation_ready': False}, None
        if not quality_ok:
            return self.fail('input_quality')
        if not math.isfinite(stamp) or not math.isfinite(now) or not 0 <= now-stamp <= c['max_age_s']:
            return self.fail('stale_or_future')
        if self.last_stamp is not None and not 0 < stamp-self.last_stamp <= c['max_gap_s']:
            return self.fail('out_of_order_or_gap')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if gray.shape != self.previous.shape:
            return self.fail('image_size_changed')
        options = dict(winSize=(c['lk_window_px'],)*2, maxLevel=c['lk_levels'],
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        forward, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None, **options)
        if forward is None:
            return self.fail('forward_flow_failed')
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, forward, None, **options)
        if backward is None:
            return self.fail('backward_flow_failed')
        old, new = self.points.reshape(-1, 2), forward.reshape(-1, 2)
        fb = np.linalg.norm(backward.reshape(-1, 2)-old, axis=1)
        valid = ((status.ravel() == 1) & (back_status.ravel() == 1) &
                 np.isfinite(new).all(axis=1) & np.isfinite(fb) & (fb <= c['max_fb_error_px']))
        if valid.sum() < c['min_features']:
            return self.fail('insufficient_consistent_features',consistent_features=int(valid.sum()),
                             required_features=c['min_features'],input_features=int(len(old)))
        matrix, inliers = cv2.estimateAffinePartial2D(old[valid], new[valid], method=cv2.RANSAC,
                                                     ransacReprojThreshold=c['ransac_error_px'])
        if matrix is None or not np.isfinite(matrix).all() or inliers is None:
            return self.fail('affine_estimation_failed')
        ratio = float(inliers.mean())
        scale = math.hypot(matrix[0, 0], matrix[1, 0])
        proposed = self.corners@matrix[:, :2].T+matrix[:, 2]
        shift = float(np.linalg.norm(proposed.mean(axis=0)-self.corners.mean(axis=0)))
        if ratio < c['min_inlier_ratio'] or not c['scale_min'] <= scale <= c['scale_max'] or shift > c['max_step_px']:
            return self.fail('geometry_gate', inlier_ratio=ratio, scale_step=scale, centre_step_px=shift,
                             gap_s=None if self.last_stamp is None else stamp-self.last_stamp, inliers=int(inliers.sum()), tracked_points=int(valid.sum()))
        x0, y0 = np.floor(proposed.min(axis=0)).astype(int)
        x1, y1 = np.ceil(proposed.max(axis=0)).astype(int)
        h, w = gray.shape
        if not (0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h):
            return self.fail('object_outside_image')
        # Align the current image to the fixed reference, rather than resizing
        # a rounded bbox (one pixel of rounding can destroy texture correlation).
        step_transform = np.eye(3)
        step_transform[:2] = matrix
        cumulative = step_transform@self.cumulative
        mapped = self.template_coordinates@cumulative[:2].T
        patch = cv2.remap(gray, mapped[..., 0].astype(np.float32), mapped[..., 1].astype(np.float32),
                          cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        score = float(cv2.matchTemplate(patch, self.template, cv2.TM_CCOEFF_NORMED)[0, 0])
        if not math.isfinite(score) or patch.std() < 2 or score < c['min_template_score']:
            return self.fail('appearance_mismatch', template_score=score if math.isfinite(score) else None, scale_step=scale, centre_step_px=shift)
        seed_score = None
        if self.refresh_score:
            seed_score = float(cv2.matchTemplate(patch, self.seed_template, cv2.TM_CCOEFF_NORMED)[0, 0])
            if not math.isfinite(seed_score) or seed_score < self.seed_floor:
                return self.fail('appearance_mismatch', seed_template_score=seed_score)
        self.corners = proposed.astype(np.float32)
        self.cumulative = cumulative
        depth_mask = self.mask(gray.shape).astype(bool)
        self.points = self.features(gray, self.mask(gray.shape, inset=0.))
        if self.points is None or len(self.points) < c['min_features']:
            return self.fail('insufficient_features_after_update')
        self.previous, self.last_stamp, self.box = gray, stamp, (int(x0), int(y0), int(x1), int(y1))
        self.frames_since_refresh += 1
        if self.refresh_score and score >= self.refresh_score and self.frames_since_refresh >= 3:
            self.template = patch.copy();self.frames_since_refresh = 0;self.refreshes += 1
        return {'status': 'tracked', 'seed_template_score': seed_score, 'template_refreshes': self.refreshes, 'target_id': c['target_id'], 'bbox_xyxy_pixels': list(self.box),
                'feature_count': int(len(self.points)), 'inlier_ratio': ratio,
                'template_score': score, 'scale_step': scale, 'centre_step_px': shift,
                'mask_pixels': int(depth_mask.sum()), 'mask_kind': 'bbox_interior_proxy_not_segmentation',
                'identity_basis': 'grounding_seed_and_local_features_not_semantic_reidentification',
                'navigation_ready': False}, depth_mask
