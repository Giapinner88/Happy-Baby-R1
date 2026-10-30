"""Positive/negative preflight checks of the new legs12 artifact contract."""
from pathlib import Path
import hashlib,json,os,shutil,subprocess,sys,tempfile
import numpy as np,onnx
root=Path(sys.argv[1]).resolve();binary=Path(sys.argv[2]).resolve()
name=sys.argv[3] if len(sys.argv)>3 else 'armfree_mirror_d25_15k_20261024';source=root/'policies/locomotion'/name
model=onnx.load(source/'policy.onnx');meta={x.key:x.value for x in model.metadata_props}
pd=np.array([float(x) for x in meta['joint_damping'].split(',')]);scale=np.array([float(x) for x in meta['action_scale'].split(',')])
expected_damping=2.0 if name=='armfree_nomirror_d20_15k_20261024' else 2.5
for i in range(12):
 assert pd[i]==(expected_damping if i%6 in (3,4,5) else 2.)
 assert np.isclose(scale[i],.1575 if i%6==3 else .328125 if i%6 in (4,5) else .15,atol=1e-8,rtol=0)
assert meta['deployable']=='false'
with tempfile.TemporaryDirectory(prefix='hb-legs12-contract-') as tmp:
 target=Path(tmp);shutil.copytree(root/'config',target/'config')
 # Select the candidate only in this temporary test fixture, never active config.
 config=target/'config/locomotion.yaml'
 import re
 text=config.read_text()
 for key in ('walk_policy','teleop_policy'):
  text=re.sub(r'^'+key+r':.*$',key+': '+name,text,flags=re.MULTILINE)
 config.write_text(text)
 for folder in ('motions','assets'):
  if (root/folder).exists():(target/folder).symlink_to(root/folder,target_is_directory=True)
 (target/'policies').mkdir()
 for p in (root/'policies').iterdir():
  if p.name!='locomotion':(target/'policies'/p.name).symlink_to(p,target_is_directory=p.is_dir())
 loco=target/'policies/locomotion';loco.mkdir()
 for p in (root/'policies/locomotion').iterdir():
  if p.name!=name:(loco/p.name).symlink_to(p,target_is_directory=p.is_dir())
 bundle=loco/name;bundle.mkdir()
 cases=[('valid',{},True),('wrong_leg_order',{'gait_stability_joint_names':','.join(reversed(meta['gait_stability_joint_names'].split(',')))},False),('wrong_predicate',{'gait_stability_predicate':'filtered_imu_ang_vel_norm_and_joint_vel_rms_v2'},False),('wrong_fsm',{'gait_fsm_contract':'unregistered_fsm'},False),('wrong_dwell',{'gait_t_settle_s':'1.5'},False),('wrong_joint_order',{'joint_names':','.join(reversed(meta['joint_names'].split(',')))},False)]
 for label,updates,accepted in cases:
  current=onnx.ModelProto();current.CopyFrom(model)
  for prop in current.metadata_props:
   if prop.key in updates:prop.value=updates[prop.key]
  onnx.save(current,bundle/'policy.onnx')
  sha=hashlib.sha256((bundle/'policy.onnx').read_bytes()).hexdigest()
  (bundle/'policy.yaml').write_text(f'contract: flat_plus_gait_h4_v1\nsha256: {sha}\nteleop_h4: true\n')
  result=subprocess.run([str(binary),'--preflight'],env={**os.environ,'HB_PROJECT_DIR':str(target)},text=True,capture_output=True)
  assert (result.returncode==0)==accepted,(label,result.stdout[-3000:],result.stderr)
  if not accepted:assert 'metadata mismatch' in result.stdout+result.stderr,(label,result.stdout[-1000:])
  print(label,'PASS')
print('ARMFREE_BUNDLE_CONTRACT_OK cases=6 exact_PD/action_metadata=true')
