"""Export fixed-camera DAC FPN plus cached ERP geometry; vendor stays unchanged."""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))


def main():
    import cv2
    import numpy as np
    import torch
    import torchvision.transforms.functional as TF
    from ai_modules.vision.dac_depth import DACDepth,sha256
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--opset',type=int,choices=[16,17],default=16);a=p.parse_args()
    c=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(c['cpu_threads'])
    depth=DACDepth(ROOT/c['upstream'],ROOT/c['model_config'],ROOT/c['checkpoint'],ROOT/c['calibration'],c['device'],c['horizontal_fov_deg'],c['forward_size'])
    image=cv2.imread(str(a.reference));reference=depth.infer(image)
    from dac.utils import erp_geometry as geo
    # Record the exact upstream static sample grid, only adding a return value.
    source=inspect.getsource(geo.cam_to_erp_patch_fast)
    old='return erp_img, erp_depth, erp_mask_valid_depth, mask_active, lat_grid, lon_grid'
    assert source.count(old)==1
    env=dict(geo.__dict__);exec(compile(source.replace(old,old+', new_grid.numpy()'),'<capture_static_erp_grid>','exec'),env)
    cam={'camera_model':'OPENCV_FISHEYE','fl_x':depth.matrix[0,0],'fl_y':depth.matrix[1,1],'cx':depth.matrix[0,2],'cy':depth.matrix[1,2],**{'k%d'%(i+1):float(v) for i,v in enumerate(depth.distortion.ravel())}}
    rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB).astype(np.float32)/255.;zeros=np.zeros((*image.shape[:2],1),np.float32)
    result=env['cam_to_erp_patch_fast'](rgb,zeros,zeros,0.,np.array(0.,np.float32),depth.patch_h,depth.patch_w,depth.cano,depth.cano*2,cam,np.array(0.,np.float32))
    grid=result[-1][0];h,w=image.shape[:2]
    np.savez_compressed(out/'geometry.npz',grid=grid,active=reference['valid_mask'],erp_active=cv2.resize(result[3],(depth.forward_size[1],depth.forward_size[0]),interpolation=cv2.INTER_NEAREST),rays=depth.rays.astype(np.float32),lat_range=reference['lat_range'],lon_range=reference['lon_range'],scale=np.array(reference['scale'],np.float32),source_size=np.array([w,h]),forward_size=np.array(depth.forward_size))
    x=TF.normalize(TF.to_tensor(reference['erp_rgb']),[.485,.456,.406],[.229,.224,.225]).unsqueeze(0).to(depth.device)
    lat=torch.tensor([reference['lat_range']],device=depth.device,dtype=torch.float32);lon=torch.tensor([reference['lon_range']],device=depth.device,dtype=torch.float32)
    class Export(torch.nn.Module):
        def __init__(self):
            super().__init__();self.network=depth.model;self.register_buffer('latitude',lat);self.register_buffer('longitude',lon)
        def forward(self,image):return self.network(image,self.latitude,self.longitude)[0]
    wrapper=Export().eval()
    with torch.inference_mode():expected=wrapper(x).float().cpu().numpy()
    # ERP positional encodings depend only on fixed spatial shapes/angular
    # ranges, not pixel content. Materialize exact upstream values once.
    encodings={};hooks=[]
    for name,module in list(wrapper.named_modules()):
        if module.__class__.__name__=='PositionEmbeddingSineERP':
            hooks.append(module.register_forward_hook(lambda m,i,o,name=name:encodings.__setitem__(name,o.detach().clone())))
    with torch.inference_mode():wrapper(x)
    for hook in hooks:hook.remove()
    class FixedPE(torch.nn.Module):
        def __init__(self,value):super().__init__();self.register_buffer('value',value)
        def forward(self,x,latitude=None,longitude=None):return self.value
    for name,value in encodings.items():
        parent,_,child=name.rpartition('.')
        setattr(wrapper.get_submodule(parent),child,FixedPE(value))
    with torch.inference_mode():frozen=wrapper(x).float().cpu().numpy()
    if not np.allclose(expected,frozen,rtol=1e-6,atol=1e-6):raise ValueError('cached PE parity failed')
    np.save(out/'reference_input.npy',x.cpu().numpy());np.save(out/'reference_output.npy',expected)
    cv2.imwrite(str(out/'reference.jpg'),image)
    np.save(out/'reference_erp.npy',reference['erp_rgb'])
    torch.onnx.export(wrapper,x,str(out/'dac.onnx'),input_names=['image'],output_names=['range'],opset_version=a.opset,dynamo=False,do_constant_folding=True)
    import onnx
    model=onnx.load(str(out/'dac.onnx'));onnx.checker.check_model(model)
    from onnxsim import simplify
    model,valid=simplify(model)
    if not valid:raise ValueError('ONNX simplification validation failed')
    onnx.save(model,str(out/'dac.onnx'))
    import onnxruntime as ort
    session=ort.InferenceSession(str(out/'dac.onnx'),providers=['CPUExecutionProvider'])
    actual=session.run(None,{'image':x.cpu().numpy()})[0]
    relative=np.abs(actual-expected)/np.maximum(np.abs(expected),1e-3)
    parity={'max_relative_error':float(relative.max()),'mean_relative_error':float(relative.mean())}
    (out/'export_parity.json').write_text(json.dumps(parity,indent=2)+'\n')
    if parity['max_relative_error']>.005:raise ValueError('export parity failed')
    (out/'signature.json').write_text(json.dumps({'checkpoint_sha256':sha256(ROOT/c['checkpoint']),'calibration_sha256':sha256(ROOT/c['calibration']),'model_config_sha256':sha256(ROOT/c['model_config']),'onnx_sha256':sha256(out/'dac.onnx'),'input':{'name':'image','shape':list(x.shape),'dtype':'float32','normalization':'RGB ImageNet, cached upstream ERP'},'output':{'name':'range','shape':list(expected.shape),'units':'unscaled ERP Euclidean range metres; multiply geometry.scale before camera remap'},'geometry_sha256':sha256(out/'geometry.npz'),'source_config':c,'export_torch':torch.__version__},indent=2)+'\n')
    print('Exported',out,flush=True)


if __name__=='__main__':main()
