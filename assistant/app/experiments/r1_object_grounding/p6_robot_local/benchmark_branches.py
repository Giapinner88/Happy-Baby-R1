"""Matched cached-image branch benchmark. No camera, cloud or motor calls."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from concurrent.futures import ThreadPoolExecutor

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--app-root',type=Path,required=True)
p.add_argument('--seed-run',type=Path,required=True)
p.add_argument('--output-dir',type=Path,required=True)
p.add_argument('--count',type=int,default=15)
a=p.parse_args()
assert 2<=a.count<=100
sys.path[:0]=[str(a.app_root),str(a.app_root/'experiments/r1_object_grounding/p6_robot_local')]
import cv2
import numpy as np
from runtime import run_branches
from dac_runtime import Geometry,TensorRT
from ai_modules.vision.feature_target import FeatureTarget
from ai_modules.vision.openai_grounding import pixel_box
cv2.setNumThreads(1)
c=json.loads((a.app_root/'experiments/r1_object_grounding/p6_robot_local/config.json').read_text())
a.output_dir.mkdir(parents=True,exist_ok=False)
raw=(a.seed_run/'seed.jpg').read_bytes();seed=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
proposal=json.loads((a.seed_run/'grounding.json').read_text());box=pixel_box(proposal['bbox'],seed.shape[1],seed.shape[0])['xyxy']
g=Geometry(Path(c['model_dir'])/'geometry.npz',c['preprocess_backend']);model=TensorRT(Path(c['model_dir'])/'dac.engine')
for _ in range(c['gpu_warmup_runs']):model.infer(np.load(Path(c['model_dir'])/'reference_input.npy'))
rows=[]
try:
    for batch,parallel in enumerate([False,True,True,False]):
        tracker=FeatureTarget(c['tracker'],seed,box)
        with ThreadPoolExecutor(max_workers=1) as pool:
            for i in range(a.count):
                start=time.perf_counter();image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR);stamp=time.monotonic()
                def depth():
                    x,_=g.prepare(image);return g.restore(model.infer(x))
                (track,mask),result,timing=run_branches(lambda:tracker.observe(image,stamp,time.monotonic()),depth,pool if parallel else None)
                if mask is None:raise ValueError('benchmark tracker failed: '+str(track))
                ranges,valid=result;support=mask & valid & np.isfinite(ranges)
                rows.append(dict(batch=batch,parallel=parallel,index=i,total_s=time.perf_counter()-start,**timing,
                                 range_m=float(np.median(ranges[support])),bbox=track['bbox_xyxy_pixels']))
finally:model.close()
(a.output_dir/'samples.json').write_text(json.dumps(rows,indent=2)+'\n')
summary={'read_only':True,'motion_calls':0,'cloud_calls':0,'camera_calls':0,'count_per_batch':a.count,'order':[False,True,True,False],
         'seed_sha256':hashlib.sha256(raw).hexdigest(),'seed_run':str(a.seed_run),'config':c,'command':sys.argv}
for mode in [False,True]:
    rs=[r for r in rows if r['parallel']==mode]
    summary[str(mode)]={k:{'p50':float(np.percentile([r[k] for r in rs],50)),'p95':float(np.percentile([r[k] for r in rs],95))} for k in ['total_s','tracking_s','dac_pipeline_s','branches_wall_s']}
summary['range_span_m']=max(r['range_m'] for r in rows)-min(r['range_m'] for r in rows)
(a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
