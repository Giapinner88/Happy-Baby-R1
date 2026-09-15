#!/usr/bin/env python3
"""Replay a recorded T007 Quest trajectory through fixed-base R1 MuJoCo."""
from __future__ import annotations
import argparse,csv,hashlib,json,sys,time,xml.etree.ElementTree as ET
from dataclasses import dataclass,fields
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from evidence.run_id import allocate_run_id
from evidence.writer import write_evidence_completeness,write_experiment_config,write_json,write_metadata,write_resolved_config,write_runner_command,write_status
from teleop.r1 import R1TeleopCommand
from teleop.r1.differential_tracking import DifferentialTrackingConfig,DifferentialUpperBodyTracker
from teleop.r1.mapping import R1TeleopMapper,TeleopCalibration,TeleopLimits
from teleop.r1.upper_body_ik import UpperBodyIKTarget,so3_log
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model

DEFAULT_CONFIG=ROOT/'experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_mujoco_trajectory_replay.json'
RUN_ROOT=ROOT/'experiments/r1_teleop/quest3_sim_v1/T007/runs'

@dataclass(frozen=True)
class Packets:
 t:np.ndarray;seq:np.ndarray;lp:np.ndarray;rp:np.ndarray;lR:np.ndarray;rR:np.ndarray;hR:np.ndarray;ha:np.ndarray;vel:np.ndarray;start:int;stop:int

@dataclass
class Trace:
 name:str;t:np.ndarray;truth_lp:np.ndarray;truth_rp:np.ndarray;ref_lp:np.ndarray;ref_rp:np.ndarray;actual_lp:np.ndarray;actual_rp:np.ndarray;desired_lv:np.ndarray;desired_rv:np.ndarray;actual_lv:np.ndarray;actual_rv:np.ndarray;truth_ha:np.ndarray;actual_ha:np.ndarray;qref:np.ndarray;q:np.ndarray;dqref:np.ndarray;dq:np.ndarray;controller_ms:np.ndarray;physics_ms:np.ndarray;age:np.ndarray;sigma:np.ndarray;v_sat:np.ndarray;a_sat:np.ndarray;lead_sat:np.ndarray;limit:np.ndarray;force_sat:np.ndarray

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def summary(x):
 a=np.asarray(x,float);a=a[np.isfinite(a)]
 return {'count':int(len(a)),'mean':float(np.mean(a)) if len(a) else None,'median':float(np.median(a)) if len(a) else None,'p95':float(np.quantile(a,.95)) if len(a) else None,'max':float(np.max(a)) if len(a) else None}
def vec(p):return np.array([p.x,p.y,p.z],float)
def rot(p):
 from scipy.spatial.transform import Rotation
 q=p.orientation;return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
def longest(cmd):
 spans=[];start=None
 for i in range(len(cmd)+1):
  enabled=i<len(cmd) and cmd[i].deadman_enabled
  if enabled and start is None:start=i
  elif not enabled and start is not None:spans.append((start,i));start=None
 if not spans:raise ValueError('No deadman-enabled segment in source trace.')
 return max(spans,key=lambda s:cmd[s[1]-1].timestamp_monotonic_s-cmd[s[0]].timestamp_monotonic_s)

def map_packets(commands,model,scale,alpha,max_duration):
 start,stop=longest(commands);selected=commands[start:stop];origin=selected[0].timestamp_monotonic_s
 if max_duration is not None:selected=[c for c in selected if c.timestamp_monotonic_s-origin<=max_duration];stop=start+len(selected)
 mapper=R1TeleopMapper(TeleopCalibration(),TeleopLimits(1.0));mapped=[mapper.map(c,c.timestamp_monotonic_s) for c in selected]
 t=np.array([c.timestamp_monotonic_s-origin for c in selected]);nom=np.zeros(model.dof);fk=model.forward_kinematics(nom)
 sl=np.array([vec(x.left_wrist_target.position) for x in mapped]);sr=np.array([vec(x.right_wrist_target.position) for x in mapped])
 lp=fk.left_end_effector[:3,3]+scale*(sl-sl[0]);rp=fk.right_end_effector[:3,3]+scale*(sr-sr[0])
 sLR=np.array([rot(x.left_wrist_target) for x in mapped]);sRR=np.array([rot(x.right_wrist_target) for x in mapped])
 lR=np.array([R@sLR[0].T@fk.left_end_effector[:3,:3] for R in sLR]);rR=np.array([R@sRR[0].T@fk.right_end_effector[:3,:3] for R in sRR])
 raw=np.unwrap(np.array([[x.head_pitch_rad,x.head_yaw_rad] for x in mapped]),axis=0);ha=raw-raw[0];hs=model.head_slice
 ha[:,0]=np.clip(ha[:,0],model.lower_limits[hs.start],model.upper_limits[hs.start]);ha[:,1]=np.clip(ha[:,1],model.lower_limits[hs.start+1],model.upper_limits[hs.start+1]);hR=np.array([model.head_rotation(*x) for x in ha])
 vel=np.zeros((len(t),15))
 for i in range(1,len(t)):
  dt=t[i]-t[i-1];v=np.r_[(lp[i]-lp[i-1])/dt,(rp[i]-rp[i-1])/dt,so3_log(lR[i]@lR[i-1].T)/dt,so3_log(rR[i]@rR[i-1].T)/dt,so3_log(hR[i]@hR[i-1].T)/dt];vel[i]=alpha*v+(1-alpha)*vel[i-1]
 vel[0]=vel[1]
 return Packets(t,np.array([c.sequence_id for c in selected]),lp,rp,lR,rR,hR,ha,vel,start,stop)

def truth_grid(p,rate):
 from scipy.spatial.transform import Rotation,Slerp
 # Slerp rejects even a sub-nanosecond overshoot, so construct a grid whose
 # final tick is provably inside the immutable source interval.
 t=np.arange(int(np.floor(p.t[-1]*rate))+1,dtype=float)/rate;out={'t':t}
 for name,a in [('lp',p.lp),('rp',p.rp),('ha',p.ha)]:out[name]=np.column_stack([np.interp(t,p.t,a[:,j]) for j in range(a.shape[1])])
 for name,a in [('lR',p.lR),('rR',p.rR),('hR',p.hR)]:out[name]=Slerp(p.t,Rotation.from_matrix(a))(t).as_matrix()
 out['lv']=np.gradient(out['lp'],t,axis=0,edge_order=2);out['rv']=np.gradient(out['rp'],t,axis=0,edge_order=2);return out

def reference(p,i,age,predict,hmax):
 from scipy.spatial.transform import Rotation
 h=min(max(age,0),hmax) if predict else 0.;v=p.vel[i]
 advance=lambda R,w:Rotation.from_rotvec(w*h).as_matrix()@R
 lp=p.lp[i]+v[:3]*h;rp=p.rp[i]+v[3:6]*h
 return UpperBodyIKTarget(lp,advance(p.lR[i],v[6:9]),rp,advance(p.rR[i],v[9:12]),advance(p.hR[i],v[12:15])),v,np.r_[lp,rp]

def find(root,tag,name):
 x=root.find(f".//{tag}[@name='{name}']")
 if x is None:raise ValueError(f'Missing {tag} {name} in canonical MuJoCo XML')
 return x
def build_xml(source,out,physics_hz):
 tree=ET.parse(source);root=tree.getroot();compiler=root.find('compiler');compiler.set('meshdir',str((source.parent/'meshes').resolve()));option=root.find('option')
 if option is None:option=ET.Element('option');root.insert(1,option)
 option.set('timestep',f'{1/physics_hz:.12g}');pelvis=find(root,'body','pelvis');pelvis.remove(pelvis.find("joint[@name='floating_base_joint']"));torso=find(root,'body','torso_link')
 for g in list(torso.findall('geom')):
  if g.get('mesh') in {'head_pitch_link','head_yaw_link'}:torso.remove(g)
 pb=ET.SubElement(torso,'body',name='head_pitch_link',pos='-0.006 0.03155 0.255');ET.SubElement(pb,'inertial',pos='-0.00006956 -0.03077972 0.08058589',mass='0.50391253',fullinertia='0.00095995 0.00091344 0.00045356 -0.00000045 0.00000624 0.00000588');ET.SubElement(pb,'joint',name='head_pitch_joint',axis='0 1 0',range='-0.62832 0.62832');ET.SubElement(pb,'geom',type='mesh',density='0',contype='0',conaffinity='0',group='2',mesh='head_pitch_link',rgba='0.79216 0.81961 0.93333 1')
 yb=ET.SubElement(pb,'body',name='head_yaw_link',pos='0.0010126 -0.03155 0.1165');ET.SubElement(yb,'inertial',pos='0.02211657 -0.00004725 0.00199411',mass='0.71432240',fullinertia='0.00168182 0.00202452 0.00181448 0.0000001 0.00029553 0.0000001');ET.SubElement(yb,'joint',name='head_yaw_joint',axis='0 0 1',range='-2.0071 2.0071');ET.SubElement(yb,'geom',type='mesh',density='0',contype='0',conaffinity='0',group='2',mesh='head_yaw_link',rgba='0.79216 0.81961 0.93333 1')
 for side in ('left','right'):ET.SubElement(find(root,'body',f'{side}_wrist_roll_link'),'site',name=f'teleop_{side}_ee',pos='.2 0 0',size='.005')
 act=root.find('actuator');ET.SubElement(act,'motor',name='head_pitch',joint='head_pitch_joint',ctrlrange='-33 33');ET.SubElement(act,'motor',name='head_yaw',joint='head_yaw_joint',ctrlrange='-33 33');ET.indent(tree);tree.write(out,encoding='unicode')

def patch_actuators(model,cfg):
 import mujoco
 sim=cfg['simulation'];ids={}
 for aid in range(model.nu):
  an=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,aid);jn=f'{an}_joint';jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,jn)
  if jid<0:continue
  if an.startswith('head_'):kp,kd,eff=sim['head_actuator_stiffness_nm_rad'],sim['head_actuator_damping_nm_s_rad'],sim['head_effort_limit_nm']
  elif any(x in an for x in ('shoulder','elbow','wrist')):kp,kd,eff=sim['arm_actuator_stiffness_nm_rad'],sim['arm_actuator_damping_nm_s_rad'],max(abs(model.actuator_ctrlrange[aid]))
  else:kp,kd,eff=(40.,3.,max(abs(model.actuator_ctrlrange[aid]))) if 'ankle' in an else (100.,3.,max(abs(model.actuator_ctrlrange[aid])))
  model.actuator_dyntype[aid]=int(mujoco.mjtDyn.mjDYN_NONE);model.actuator_gaintype[aid]=int(mujoco.mjtGain.mjGAIN_FIXED);model.actuator_biastype[aid]=int(mujoco.mjtBias.mjBIAS_AFFINE);model.actuator_gainprm[aid,:]=0;model.actuator_biasprm[aid,:]=0;model.actuator_gainprm[aid,0]=kp;model.actuator_biasprm[aid,1]=-kp;model.actuator_biasprm[aid,2]=-kd;model.actuator_ctrllimited[aid]=0;model.actuator_forcelimited[aid]=1;model.actuator_forcerange[aid]=[-eff,eff];ids[jn]=aid
 return ids

def run_case(model,p,truth,cfg,case,upper,actids):
 import mujoco
 sim,tc,cc=cfg['simulation'],cfg['transport'],cfg['controller'];rate=float(sim['control_hz']);steps=round(float(sim['physics_hz'])/rate)
 accepted={f.name for f in fields(DifferentialTrackingConfig)}-{'dt_s'};dc={k:float(v) for k,v in cc.items() if k in accepted};dc.update({k:float(v) for k,v in dict(case.get('controller_overrides') or {}).items() if k in accepted});tracker=DifferentialUpperBodyTracker(upper,np.zeros(upper.dof),np.zeros(upper.dof),DifferentialTrackingConfig(dt_s=1/rate,**dc));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 names=list(upper.joint_names);jids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names];qa=np.array([model.jnt_qposadr[i] for i in jids]);va=np.array([model.jnt_dofadr[i] for i in jids]);aids=np.array([actids[n] for n in names]);ls=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'teleop_left_ee');rs=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'teleop_right_ee');pel=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'pelvis');delivery=p.t+float(tc['replay_latency_s'])
 keys=['ref','al','ar','ha','qref','q','dqref','dq','cms','pms','age','sigma','vs','as','lead','limit','force'];a={k:[] for k in keys}
 for now in truth['t']:
  st=time.perf_counter_ns();i=max(int(np.searchsorted(delivery,now,side='right')-1),0);age=max(0,now-p.t[i]);target,v,ref=reference(p,i,age,case['latency_prediction'],float(tc['prediction_horizon_limit_s']));ff_scale=float(case.get('velocity_feedforward_scale',1.0 if case['velocity_feedforward'] else 0.0));feedforward=v*ff_scale
  if case.get('velocity_feedforward_task')=='head_only':feedforward[:12]=0.0
  step=tracker.step(data.qpos[qa],target,feedforward,velocity_feedforward=ff_scale>0.0);data.ctrl[aids]=step.joint_position_reference_rad;cms=(time.perf_counter_ns()-st)*1e-6;st=time.perf_counter_ns()
  for _ in range(steps):mujoco.mj_step(model,data)
  pms=(time.perf_counter_ns()-st)*1e-6;q=data.qpos[qa].copy();fr=model.actuator_forcerange[aids];force=data.actuator_force[aids];sat=(np.abs(force-fr[:,0])<1e-5)|(np.abs(force-fr[:,1])<1e-5);base=data.xpos[pel]
  vals=[ref,data.site_xpos[ls].copy()-base,data.site_xpos[rs].copy()-base,q[upper.head_slice],step.joint_position_reference_rad,q,step.joint_velocity_reference_rad_s,data.qvel[va].copy(),cms,pms,age,step.minimum_singular_value,step.velocity_saturated,step.acceleration_saturated,step.reference_lead_clamped,step.joint_limit_active,sat]
  for k,vv in zip(keys,vals):a[k].append(vv)
 for k in a:a[k]=np.asarray(a[k])
 t=truth['t'];return Trace(str(case['name']),t,truth['lp'],truth['rp'],a['ref'][:,:3],a['ref'][:,3:],a['al'],a['ar'],truth['lv'],truth['rv'],np.gradient(a['al'],t,axis=0,edge_order=2),np.gradient(a['ar'],t,axis=0,edge_order=2),truth['ha'],a['ha'],a['qref'],a['q'],a['dqref'],a['dq'],a['cms'],a['pms'],a['age'],a['sigma'],a['vs'],a['as'],a['lead'],a['limit'],a['force'])

def rmse(e):return float(np.sqrt(np.mean(np.sum(np.asarray(e)**2,axis=1))))
def metrics(x):
 le=x.truth_lp-x.actual_lp;re=x.truth_rp-x.actual_rp;lv=x.desired_lv-x.actual_lv;rv=x.desired_rv-x.actual_rv;he=x.truth_ha-x.actual_ha
 return {'sample_count':len(x.t),'duration_s':float(x.t[-1]),'wrist_position_rmse_m':{'left':rmse(le),'right':rmse(re)},'wrist_position_error_norm_m':{'left':summary(np.linalg.norm(le,axis=1)),'right':summary(np.linalg.norm(re,axis=1))},'wrist_velocity_rmse_mps':{'left':rmse(lv),'right':rmse(rv)},'head_angle_rmse_rad':{'pitch':float(np.sqrt(np.mean(he[:,0]**2))),'yaw':float(np.sqrt(np.mean(he[:,1]**2)))},'joint_position_rmse_rad':rmse(x.qref-x.q),'joint_velocity_rmse_rad_s':rmse(x.dqref-x.dq),'controller_compute_ms':summary(x.controller_ms),'physics_compute_ms':summary(x.physics_ms),'offline_real_time_factor':float(x.t[-1]/(np.sum(x.controller_ms+x.physics_ms)*1e-3)),'controller_capacity_hz_from_mean_compute':float(1000/np.mean(x.controller_ms)),'packet_age_s':summary(x.age),'minimum_weighted_jacobian_singular_value':summary(x.sigma),'velocity_saturation_fraction':float(np.mean(x.v_sat)),'acceleration_saturation_fraction':float(np.mean(x.a_sat)),'reference_lead_clamp_fraction':float(np.mean(x.lead_sat)),'joint_limit_activation_fraction':float(np.mean(x.limit)),'actuator_force_saturation_fraction':float(np.mean(x.force_sat))}

def transport(source,p):
 dt=np.diff(p.t);rows=[]
 for i,t in enumerate(p.t):rows.append({'packet_index':i,'source_sequence_id':int(p.seq[i]),'elapsed_s':float(t),'interarrival_s':float(dt[i-1]) if i else '', 'instantaneous_rate_hz':float(1/dt[i-1]) if i else ''})
 ages=[];tp=source/'targets.json'
 if tp.exists():ages=[float(x['age_s']) for x in json.loads(tp.read_text()) if x.get('is_fresh_command') and x.get('age_s') is not None]
 gaps=np.diff(p.seq)-1;return {'selected_packet_count':len(p.t),'selected_duration_s':float(p.t[-1]),'mean_packet_rate_hz':float((len(p.t)-1)/p.t[-1]),'interarrival_s':summary(dt),'interarrival_jitter_std_s':float(np.std(dt)),'sequence_gap_missing_packet_count':int(np.sum(np.maximum(gaps,0))),'source_recorded_fresh_command_age_s':summary(ages),'interpretation':'Packet rate uses source timestamps; command age is the original same-host bridge-to-control measurement.'},rows

def plots(out,traces,primary,names,rows,age):
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 x=next(z for z in traces if z.name==primary)
 for title,dl,dr,al,ar,unit,file in [('Position',x.truth_lp,x.truth_rp,x.actual_lp,x.actual_rp,'m','cartesian_position_tracking.png'),('Velocity',x.desired_lv,x.desired_rv,x.actual_lv,x.actual_rv,'m/s','cartesian_velocity_tracking.png')]:
  fig,ax=plt.subplots(4,2,figsize=(14,11),sharex='col',layout='constrained')
  for c,(side,d,a) in enumerate((('left',dl,al),('right',dr,ar))):
   for j,label in enumerate('xyz'):ax[j,c].plot(x.t,d[:,j],label='teleop desired');ax[j,c].plot(x.t,a[:,j],label='MuJoCo actual',lw=.8);ax[j,c].set_ylabel(f'{label} ({unit})')
   ax[3,c].plot(x.t,np.linalg.norm(d-a,axis=1),color='tab:red');ax[3,c].set(xlabel='trajectory time (s)',ylabel='error norm');ax[0,c].set_title(f'{side} wrist virtual EE');ax[0,c].legend()
  fig.suptitle(f'{title} tracking — {primary}');fig.savefig(out/file,dpi=160);plt.close(fig)
 fig,ax=plt.subplots(4,3,figsize=(16,11),sharex=True,layout='constrained')
 for j,(axis,name) in enumerate(zip(ax.flat,names)):axis.plot(x.t,x.qref[:,j],label='q reference');axis.plot(x.t,x.q[:,j],label='q actual',lw=.8);axis.set_title(name.replace('_joint',''),fontsize=9);axis.grid(alpha=.2)
 ax.flat[0].legend();fig.supylabel('joint angle (rad)');fig.supxlabel('trajectory time (s)');fig.savefig(out/'joint_angle_tracking.png',dpi=160);plt.close(fig)
 fig,ax=plt.subplots(2,2,figsize=(14,8),layout='constrained')
 for z in traces:
  err=.5*(np.linalg.norm(z.truth_lp-z.actual_lp,axis=1)+np.linalg.norm(z.truth_rp-z.actual_rp,axis=1));ax[0,0].plot(z.t,err,label=z.name,lw=.7);ax[0,1].plot(z.t,np.linalg.norm(z.truth_ha-z.actual_ha,axis=1),label=z.name,lw=.7);ax[1,0].plot(z.t,z.controller_ms,label=z.name,lw=.5)
 ax[0,0].set(title='Cartesian position error',ylabel='mean wrist error (m)');ax[0,1].set(title='Head tracking error',ylabel='angle error norm (rad)');ax[1,0].set(title='Controller cost',ylabel='computation (ms)',xlabel='time (s)');ax[0,0].legend(fontsize=7);ax[0,1].legend(fontsize=7)
 pt=np.array([r['elapsed_s'] for r in rows[1:]],float);pr=np.array([r['instantaneous_rate_hz'] for r in rows[1:]],float);am=age.get('mean');txt='n/a' if am is None else f'{1000*am:.1f} ms';ax[1,1].plot(pt,pr,lw=.6);ax[1,1].set(title=f'Quest packet rate; recorded age {txt}',ylabel='Hz',xlabel='source time (s)');fig.savefig(out/'controller_comparison_and_timing.png',dpi=160);plt.close(fig)

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--config',type=Path,default=DEFAULT_CONFIG);ap.add_argument('--output-dir',type=Path);ap.add_argument('--max-duration-s',type=float);ap.add_argument('--case',action='append');args=ap.parse_args();cp=args.config.resolve();cfg=json.loads(cp.read_text());source=(ROOT/cfg['source_run']).resolve();raw=source/'raw_commands.jsonl';commands=[R1TeleopCommand.from_dict(json.loads(x)) for x in raw.read_text().splitlines() if x.strip()];maxd=args.max_duration_s if args.max_duration_s is not None else cfg['source_selection'].get('maximum_duration_s')
 out=args.output_dir.resolve() if args.output_dir else RUN_ROOT/allocate_run_id(RUN_ROOT,'t007_mujoco_replay')
 if out.exists():raise SystemExit(f'Refusing to overwrite {out}')
 out.mkdir(parents=True);import mujoco
 sim=cfg['simulation'];upper=load_r1_a5_upper_body_model((ROOT/sim['kinematic_model']).resolve(),control_waist_yaw=False);p=map_packets(commands,upper,float(cfg['calibration']['position_scale']),float(cfg['transport']['velocity_filter_alpha']),maxd);truth=truth_grid(p,float(sim['control_hz']));xml=out/'resolved_mujoco_model.xml';build_xml((ROOT/sim['canonical_model']).resolve(),xml,float(sim['physics_hz']));model=mujoco.MjModel.from_xml_path(str(xml));actids=patch_actuators(model,cfg);cases=[dict(x) for x in cfg['cases']]
 if args.case:cases=[x for x in cases if x['name'] in set(args.case)]
 primary=cfg['primary_case'] if cfg['primary_case'] in [x['name'] for x in cases] else cases[-1]['name'];traces=[];cm={}
 for case in cases:print(f"running {case['name']} ({p.t[-1]:.2f} s)",flush=True);z=run_case(model,p,truth,cfg,case,upper,actids);traces.append(z);cm[z.name]=metrics(z)
 ts,rows=transport(source,p);crit=cfg['acceptance_criteria'];pm=cm[primary];checks={'wrist_position_rmse':max(pm['wrist_position_rmse_m'].values())<=crit['wrist_position_rmse_m_max'],'wrist_velocity_rmse':max(pm['wrist_velocity_rmse_mps'].values())<=crit['wrist_velocity_rmse_mps_max'],'head_angle_rmse':max(pm['head_angle_rmse_rad'].values())<=crit['head_angle_rmse_rad_max'],'controller_compute_budget':pm['controller_compute_ms']['p95']<=crit['controller_compute_p95_ms_max'],'joint_limit_activation':pm['joint_limit_activation_fraction']<=crit['joint_limit_activation_fraction_max'],'fixed_base':True};result={'schema_version':1,'mode':'simulation_only','backend':'mujoco_cpu','source_transport':ts,'cases':cm,'primary_case':primary,'acceptance_checks':checks,'all_declared_acceptance_checks_pass':all(checks.values())};write_json(out/'metrics.json',result)
 with (out/'source_packet_transport.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 npz={'joint_names':np.asarray(upper.joint_names)}
 for z in traces:
  for fld in fields(z):
   if fld.name!='name':npz[f'{z.name}__{fld.name}']=np.asarray(getattr(z,fld.name))
 np.savez_compressed(out/'trajectory_trace.npz',**npz);fig=out/'figures';fig.mkdir();plots(fig,traces,primary,upper.joint_names,rows,ts['source_recorded_fresh_command_age_s']);resolved=json.loads(json.dumps(cfg));resolved['runtime']={'source_trace':str(raw),'source_trace_sha256':sha(raw),'selected_source_indices_half_open':[p.start,p.stop],'selected_source_sequence_ids_inclusive':[int(p.seq[0]),int(p.seq[-1])],'selected_duration_s':float(p.t[-1]),'maximum_duration_s_override':maxd,'effective_cases':cases,'effective_primary_case':primary,'resolved_mujoco_model_sha256':sha(xml),'controlled_joint_names':list(upper.joint_names)};write_experiment_config(out,cfg);write_resolved_config(out,resolved);write_json(out/'source_manifest.json',{'raw_commands':{'path':str(raw),'sha256':sha(raw)},'targets':{'path':str(source/'targets.json'),'sha256':sha(source/'targets.json')},'source_metrics':{'path':str(source/'metrics.json'),'sha256':sha(source/'metrics.json')}});write_runner_command(out);write_metadata(out,ROOT,{'protocol_id':'t007_mujoco_replay','execution_backend':'mujoco_cpu_fixed_base','hardware_command_channel':'not_opened','mujoco_version':mujoco.__version__,'configuration_path':str(cp),'configuration_sha256':sha(cp)});write_evidence_completeness(out,{'source_manifest':True,'resolved_mujoco_model':True,'trajectory_trace':True,'source_packet_transport_csv':True,'metrics':True,'plots':True,'video':{'present':False,'reason':'Quantitative headless replay; source run retains IsaacLab video.'}});outcome='pass' if all(checks.values()) else 'fail';write_status(out,'completed',outcome,'all declared simulation criteria passed' if outcome=='pass' else 'one or more declared simulation criteria failed',{'hardware_claim':'none','dds_or_hardware_called':False});print(json.dumps(result,indent=2));print(out);return 0
if __name__=='__main__':raise SystemExit(main())
