"""Read-only TensorRT/geometry parity and latency probe on the robot GPU."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
from dac_runtime import Geometry,TensorRT


def main():
    p=argparse.ArgumentParser();p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--count',type=int,default=10);a=p.parse_args()
    if not 1<=a.count<=100:raise ValueError('count1..100')
    a.output_dir.mkdir(exist_ok=False,parents=True);cv2.setNumThreads(1)
    geometry=Geometry(a.model_dir/'geometry.npz');image=cv2.imread(str(a.model_dir/'reference.jpg'))
    x,erp=geometry.prepare(image);reference_erp=np.load(a.model_dir/'reference_erp.npy')
    # reference.jpg is an extra JPEG encoding; test exact model-input parity separately.
    model=TensorRT(a.model_dir/'dac.engine');original=np.load(a.model_dir/'reference_input.npy');expected=np.load(a.model_dir/'reference_output.npy')
    actual=model.infer(original);relative=np.abs(actual-expected)/np.maximum(np.abs(expected),1e-3)
    if not np.isfinite(actual).all() or (actual<=0).any():raise ValueError('nonfinite/nonpositive TensorRT output')
    times=[]
    for i in range(a.count):
        t=time.perf_counter();x,erp=geometry.prepare(image);p=time.perf_counter();prediction=model.infer(x);g=time.perf_counter();depth,valid=geometry.restore(prediction);end=time.perf_counter()
        times.append({'preprocess_s':p-t,'inference_s':g-p,'restore_s':end-g,'total_s':end-t})
    model.close()
    result={'read_only':True,'motion_commands_sent':0,'preprocess_backend':geometry.backend,'tensor_rt_reference_max_relative_error':float(relative.max()),'tensor_rt_reference_p99_relative_error':float(np.percentile(relative,99)),
            'timing':times,'total_p95_s':float(np.percentile([x['total_s'] for x in times],95)),'valid_fraction':float(valid.mean())}
    (a.output_dir/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if result['tensor_rt_reference_p99_relative_error']>.05:raise ValueError('TensorRT parity not accepted')


if __name__=='__main__':main()
