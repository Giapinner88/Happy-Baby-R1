"""Official Depth Anything V2 metric backend with explicit fisheye handling.

Native metric output is optical-axis Z (see upstream depth_to_pointcloud).
Map it to original fisheye pixels, then range=Z/unit_ray_z. This geometric
conversion does not validate learned metric scale or camera intrinsics.
"""
import inspect
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from ai_modules.vision.dac_depth import camera_rays


class FisheyeView:
    def __init__(self, size, matrix, distortion):
        self.size = size
        self.rays = camera_rays(size, matrix, distortion)
        self.map_x, self.map_y = cv2.fisheye.initUndistortRectifyMap(
            matrix, distortion, np.eye(3), matrix, size, cv2.CV_32FC1)
        self.inverse_x = (matrix[0, 0]*self.rays[..., 0]/self.rays[..., 2]+matrix[0, 2]).astype(np.float32)
        self.inverse_y = (matrix[1, 1]*self.rays[..., 1]/self.rays[..., 2]+matrix[1, 2]).astype(np.float32)
        width, height = size
        self.rect_valid = ((self.map_x >= 0) & (self.map_x < width-1) &
                           (self.map_y >= 0) & (self.map_y < height-1)).astype(np.float32)

    def rectify(self, image):
        return cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def restore(self, z, rectified):
        if rectified:
            z = cv2.remap(z, self.inverse_x, self.inverse_y, cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)
            support = cv2.remap(self.rect_valid, self.inverse_x, self.inverse_y,
                                cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            valid = support > .999
        else:
            valid = np.ones(z.shape, bool)
        valid &= np.isfinite(z) & (z > 0) & np.isfinite(self.rays).all(axis=-1) & (self.rays[..., 2] > 0)
        z = z.astype(np.float32).copy()
        z[~valid] = np.nan
        return z, (z/self.rays[..., 2]).astype(np.float32), valid


class DAV2Depth:
    def __init__(self, upstream, model_config, checkpoint, calibration, device, rectified):
        import torch
        self.device = torch.device(device)
        if self.device.type == 'cuda':
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA unavailable')
            # Upstream image2tensor uses the current CUDA device.
            torch.cuda.set_device(self.device)
        self.effective = json.loads(Path(model_config).read_text())
        self.rectified = rectified
        if type(rectified) is not bool:
            raise ValueError('rectified must be boolean')
        metric_root = (Path(upstream)/'metric_depth').resolve()
        sys.path.insert(0, str(metric_root))
        from depth_anything_v2.dpt import DepthAnythingV2
        if not Path(inspect.getfile(DepthAnythingV2)).resolve().is_relative_to(metric_root):
            raise RuntimeError('imported relative-depth model instead of metric variant')
        data = np.load(calibration, allow_pickle=False)
        if str(data['camera_model']) != 'fisheye':
            raise ValueError('expected OpenCV fisheye calibration')
        self.size = tuple(int(v) for v in data['image_size_wh'])
        self.view = FisheyeView(self.size, data['camera_matrix'], data['dist_coeffs'])
        self.model = DepthAnythingV2(**{k: self.effective[k] for k in
                                      ('encoder', 'features', 'out_channels', 'max_depth')})
        self.model.load_state_dict(torch.load(checkpoint, map_location='cpu', weights_only=True), strict=True)
        self.model = self.model.to(self.device).eval()
        self.effective['rectified'] = rectified
        self.effective['rectified_camera_matrix'] = data['camera_matrix'].tolist()

    def sync(self):
        import torch
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)

    def infer(self, image):
        import torch
        import torch.nn.functional as F
        if (image.shape[1], image.shape[0]) != self.size:
            raise ValueError('image/calibration size mismatch')
        started = time.perf_counter()
        view = self.view.rectify(image) if self.rectified else image
        tensor, shape = self.model.image2tensor(view, self.effective['input_size'])
        tensor = tensor.to(self.device)
        self.sync()
        prepared = time.perf_counter()
        with torch.inference_mode():
            depth = self.model(tensor)
        self.sync()
        inferred = time.perf_counter()
        z_view = F.interpolate(depth[:, None], shape, mode='bilinear', align_corners=True)[0, 0].float().cpu().numpy()
        if not np.isfinite(z_view).all():
            raise ValueError('model emitted nonfinite Z')
        z, ranges, valid = self.view.restore(z_view, self.rectified)
        return {'range_m': ranges, 'optical_z_m': z, 'valid_mask': valid,
                'inference_depth_z_m': z_view, 'model_input_rgb': cv2.cvtColor(view, cv2.COLOR_BGR2RGB),
                'timing_s': {'preprocess': prepared-started, 'inference': inferred-prepared,
                             'total': time.perf_counter()-started}}
