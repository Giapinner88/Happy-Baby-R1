"""Create a hash-addressed robot-local app, never include credentials."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'));a=p.parse_args()
    config=json.loads(a.config.read_text())
    if config.get('execute') is not False or config.get('clear_corridor_confirmed') is not False:raise ValueError('bundle must start shadow')
    if config.get('orientation',{}).get('execute_enabled',False) is not False or config.get('gait_approach',{}).get('execute_enabled',False) is not False:raise ValueError('bundle must disable navigation execution')
    out=a.output_dir;out.mkdir(parents=True,exist_ok=False);app=out/'app'
    sources=[]
    for directory in ['ai_modules/navigation','ai_modules/vision']:
        sources.extend(Path(directory)/p.name for p in (ROOT/directory).glob('*.py'))
    sources += [Path('teleop/r1/kinematics.py'),Path('assets/R1.urdf')]
    sources += [p.relative_to(ROOT) for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('test_')]
    sources += [Path('experiments/r1_object_grounding/p6_robot_local/config.json')]
    sources += [Path('experiments/r1_object_grounding/p6_robot_local/gait_aware_arc.json')]
    sources += [Path('experiments/r1_object_grounding/p6_robot_local/approach_comparison/config.json')]
    sources += [Path('experiments/r1_object_grounding/p6_robot_local/approach_comparison/README.md')]
    for name in sources:
        dest=app/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
    (app/'experiments/r1_object_grounding/p6_robot_local/config.json').write_text(json.dumps(config,indent=2)+'\n')
    for name,source in {'camera_shadow_worker.py':'experiments/r1_camera/depth_capability_v1/camera_shadow_worker.py','state_sync.py':'ai_modules/vision/state_sync.py','odom_sync.py':'ai_modules/vision/odom_sync.py'}.items():
        (app/'sensor').mkdir(exist_ok=True);shutil.copy2(ROOT/source,app/'sensor'/name)
    shutil.copy2(ROOT/'experiments/r1_object_grounding/p6_owner_adapter/bounded_trial.py',app/'experiments/r1_object_grounding/p6_robot_local/bounded_trial.py')
    shutil.copytree(Path(__file__).parent/'services',out/'services')
    for path in Path(__file__).parent.glob('*.sh'):shutil.copy2(path,out/path.name)
    if (Path(__file__).parent/'README.md').exists():shutil.copy2(Path(__file__).parent/'README.md',out/'README.md')
    manifest={str(p.relative_to(app)):hashlib.sha256(p.read_bytes()).hexdigest() for p in app.rglob('*') if p.is_file()}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    deployment={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in out.rglob('*') if p.is_file() and 'app' not in p.relative_to(out).parts and p.name!='manifest.json'}
    (out/'deployment_files.json').write_text(json.dumps(deployment,indent=2)+'\n')
    print(out)


if __name__=='__main__':main()
