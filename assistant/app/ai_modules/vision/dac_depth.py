"""DAC indoor inference on calibrated OpenCV fisheye images; offline estimates.

Matches upstream demo: camera -> ERP patch -> resize -> IDiscERP -> scale.
Depth is Euclidean camera range in metres, NOT optical Z. No motor transport.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import types
import sys
import time
import subprocess


def inspect_jpeg(path):
    """Catch native libjpeg warnings which PIL.verify() may not report.

    Separate process keeps decoder stderr attributable to one source frame.
    This is an offline input gate, not a real-time decoder implementation.
    Decoder success still does not rule out visible exposure/encoding seams.
    """
    code = ('import cv2,json,sys; im=cv2.imread(sys.argv[1]); '
            'print(json.dumps({"decoded":im is not None,"shape":None if im is None else list(im.shape)}))')
    result = subprocess.run([sys.executable, '-c', code, str(path)],
                            capture_output=True, text=True, timeout=20)
    try:
        record = json.loads(result.stdout)
    except ValueError:
        record = {'decoded': False, 'shape': None}
    record['decoder_stderr'] = result.stderr.strip()
    record['accepted'] = result.returncode == 0 and record['decoded'] and not result.stderr.strip()
    return record


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_model(upstream, config, checkpoint, device):
    """Use upstream FPN branch; omit its unused CUDA-extension import.

    Upstream imports the deformable decoder even when attn_dec=false. Load
    an in-memory copy of that class with ONLY this unused import removed.
    Never substitute a decoder implementation; attn_dec=true is rejected.
    The vendor checkout stays untouched; full checkpoint load is strict.
    """
    import torch
    if config['model_name'] != 'IDiscERP' or config['model']['attn_dec'] is not False:
        raise ValueError('adapter supports IDiscERP FPN checkpoints only')
    sys.path.insert(0, str(Path(upstream).resolve()))
    source = Path(upstream)/'dac/models/idisc_erp.py'
    text = source.read_text()
    unused = 'from dac.models.defattn_decoder import MSDeformAttnPixelDecoder'
    if text.count(unused) != 1:
        raise ValueError('upstream import changed; review adapter compatibility')
    module = types.ModuleType('r1_dac_fpn')
    exec(compile(text.replace(unused, '# unused deformable decoder import omitted'),
                 str(source), 'exec'), module.__dict__)
    effective = copy.deepcopy(config)
    effective['model']['pixel_encoder']['pretrained'] = False
    model = module.IDiscERP.build(effective)
    state = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if 'model' in state:
        state = state['model']
    model.load_state_dict({k.removeprefix('module.'): v for k, v in state.items()}, strict=True)
    return model.to(device).eval(), effective


def camera_rays(size, matrix, distortion):
    import cv2
    import numpy as np
    width, height = size
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    pixels = np.stack((x, y), axis=-1).astype(np.float64).reshape(-1, 1, 2)
    points = cv2.fisheye.undistortPoints(pixels, matrix, distortion).reshape(height, width, 2)
    rays = np.concatenate((points, np.ones((height, width, 1))), axis=-1)
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    return rays


def remap_range(prediction, active, rays, lat_range, lon_range):
    """Inverse angular mapping consistent with upstream endpoint ERP grids."""
    import cv2
    import numpy as np
    height, width = prediction.shape
    longitude = np.arctan2(rays[..., 0], rays[..., 2])
    latitude = np.arctan2(rays[..., 1], np.hypot(rays[..., 0], rays[..., 2]))
    mx = ((longitude-lon_range[0])/(lon_range[1]-lon_range[0])*(width-1)).astype(np.float32)
    my = ((latitude-lat_range[0])/(lat_range[1]-lat_range[0])*(height-1)).astype(np.float32)
    valid = (np.isfinite(mx) & np.isfinite(my) & (mx >= 0) & (mx <= width-1)
             & (my >= 0) & (my <= height-1) & (rays[..., 2] > 0))
    values = cv2.remap(prediction, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    support = cv2.remap(active.astype(np.float32), mx, my, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT)
    valid &= (support > .999) & np.isfinite(values) & (values > 0)
    values[~valid] = np.nan
    return values, valid


class DACDepth:
    def __init__(self, upstream, model_config, checkpoint, calibration,
                 device='cuda:0', horizontal_fov_deg=120., forward_size=(500, 750)):
        import numpy as np
        import torch
        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable; use explicit --device cpu for CPU probe')
        self.config = json.loads(Path(model_config).read_text())
        data = np.load(calibration, allow_pickle=False)
        if str(data['camera_model']) != 'fisheye':
            raise ValueError('requires OpenCV fisheye calibration')
        self.matrix, self.distortion = data['camera_matrix'], data['dist_coeffs']
        self.size = tuple(int(v) for v in data['image_size_wh'])
        if (self.matrix.shape != (3, 3) or self.distortion.size != 4 or
                not np.isfinite(self.matrix).all() or not np.isfinite(self.distortion).all()):
            raise ValueError('invalid calibration')
        if not 0 < horizontal_fov_deg < 180 or len(forward_size) != 2 or min(forward_size) <= 0:
            raise ValueError('invalid ERP crop or inference size')
        self.forward_size = tuple(forward_size)
        self.cano = self.config['cano_sz'][0]
        self.patch_w = int(self.cano*horizontal_fov_deg/180)
        self.patch_h = int(self.patch_w*self.forward_size[0]/self.forward_size[1])
        self.rays = camera_rays(self.size, self.matrix, self.distortion)
        self.model, self.effective = load_model(upstream, self.config, checkpoint, self.device)

    def sync(self):
        import torch
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)

    def infer(self, bgr):
        import cv2
        import numpy as np
        import torch
        import torchvision.transforms.functional as TF
        from dac.utils.erp_geometry import cam_to_erp_patch_fast
        from dac.dataloders.dataset import resize_for_input
        if (bgr.shape[1], bgr.shape[0]) != self.size:
            raise ValueError('image size does not match calibration')
        started = time.perf_counter()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32)/255.
        zeros = np.zeros((*bgr.shape[:2], 1), np.float32)
        d = self.distortion.ravel()
        cam = {'camera_model': 'OPENCV_FISHEYE', 'fl_x': self.matrix[0, 0],
               'fl_y': self.matrix[1, 1], 'cx': self.matrix[0, 2], 'cy': self.matrix[1, 2],
               **{f'k{i+1}': float(v) for i, v in enumerate(d)}}
        erp, depth, _, active, lat, lon = cam_to_erp_patch_fast(
            rgb, zeros, zeros, 0., np.array(0., np.float32), self.patch_h, self.patch_w,
            self.cano, self.cano*2, cam, np.array(0., np.float32))
        erp, _, pad, scale, active = resize_for_input(
            (erp*255).astype(np.uint8), depth, self.forward_size, None,
            list(erp.shape[:2]), 1., padding_rgb=[0, 0, 0], mask=active)
        if any(pad):
            raise ValueError('padded ERP crops require explicit angular remapping; choose matching aspect')
        lat_range = [float(np.min(lat)), float(np.max(lat))]
        lon_range = [float(np.min(lon)), float(np.max(lon))]
        image = TF.normalize(TF.to_tensor(erp), [0.485, .456, .406], [.229, .224, .225])
        image = image.unsqueeze(0).to(self.device)
        latitude = torch.tensor([lat_range], dtype=torch.float32, device=self.device)
        longitude = torch.tensor([lon_range], dtype=torch.float32, device=self.device)
        self.sync()
        prepared = time.perf_counter()
        with torch.inference_mode():
            prediction, _, _ = self.model(image, latitude, longitude)
        self.sync()
        inferred = time.perf_counter()
        prediction = prediction[0, 0].float().cpu().numpy()*scale
        if not np.isfinite(prediction).all() or (prediction <= 0).any():
            raise ValueError('model emitted invalid range')
        ranges, valid = remap_range(prediction, active, self.rays, lat_range, lon_range)
        return {'range_m': ranges, 'optical_z_m': ranges*self.rays[..., 2],
                'valid_mask': valid, 'erp_range_m': prediction, 'erp_rgb': erp,
                'lat_range': lat_range, 'lon_range': lon_range, 'scale': scale,
                'timing_s': {'preprocess': prepared-started, 'inference': inferred-prepared,
                             'total': time.perf_counter()-started}}
