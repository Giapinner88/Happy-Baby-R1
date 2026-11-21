"""Read-only bundle/model/dependency checks; no camera, voice or motor RPC."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--require-engine',action='store_true');a=p.parse_args()
    manifest=json.loads((a.bundle/'manifest.json').read_text());root=a.bundle/'app'
    deployment=json.loads((a.bundle/'deployment_files.json').read_text())
    if 'services/hb_local_approach.service' not in deployment:raise ValueError('service manifest missing')
    for name,expected in deployment.items():
        path=Path(name)
        if path.is_absolute() or '..' in path.parts:raise ValueError('unsafe deployment path')
        if hashlib.sha256((a.bundle/path).read_bytes()).hexdigest()!=expected:raise ValueError('deployment file mismatch')
    for name,expected in manifest.items():
        path=Path(name)
        if path.is_absolute() or '..' in path.parts:raise ValueError('unsafe manifest path')
        data=(root/path).read_bytes()
        if hashlib.sha256(data).hexdigest()!=expected:raise ValueError('source hash changed: '+name)
        if name.endswith('.py'):ast.parse(data.decode(),feature_version=(3,8))
    config=json.loads((root/'experiments/r1_object_grounding/p6_robot_local/config.json').read_text())
    if config['execute'] is not False or config['clear_corridor_confirmed'] is not False:raise ValueError('deployment must start shadow')
    if config.get('orientation',{}).get('execute_enabled',False) is not False:raise ValueError('deployment must keep angular execution disabled')
    if config.get('gait_approach',{}).get('execute_enabled',False) is not False:raise ValueError('deployment must keep gait approach execution disabled')
    sig=json.loads((a.model_dir/'signature.json').read_text())
    for name,key in [('dac.onnx','onnx_sha256'),('geometry.npz','geometry_sha256')]:
        if hashlib.sha256((a.model_dir/name).read_bytes()).hexdigest()!=sig[key]:raise ValueError('model hash mismatch')
    if a.require_engine:
        engine=a.model_dir/'dac.engine';record=json.loads((a.model_dir/'engine.json').read_text())
        if hashlib.sha256(engine.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('engine mismatch')
        import cv2,numpy,tensorrt
        print(json.dumps({'bundle_verified':True,'model_verified':True,'execute':False,'numpy':numpy.__version__,'opencv':cv2.__version__,'tensorrt':tensorrt.__version__}))
    else:print('Bundle and ONNX hashes verified; no motor output')


if __name__=='__main__':main()
