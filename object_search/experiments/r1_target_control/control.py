"""Target planning and UTL1 packets; no DDS publisher or policy reimplementation."""
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ARM_NAMES=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll')
ARM_JOINTS=tuple(f'{side}_{j}_joint' for side in ('left','right') for j in ARM_NAMES)
HEAD_JOINTS=('head_yaw_joint','head_pitch_joint')
JOINTS=ARM_JOINTS+HEAD_JOINTS
IDL=(15,16,17,18,19,22,23,24,25,26,30,29)
PACKET=struct.Struct('<II4B12f')
MAGIC=0x314C5455


def sha256(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def runtime_info(root,cfg):
    root=Path(root)
    bundle=root/'controller/policies/locomotion'/cfg['policy_bundle']
    policy=bundle/'policy.onnx';meta=bundle/'policy.yaml'
    def yaml_values(path):
        return dict(line.split(':',1) for raw in path.read_text().splitlines()
                    if ':' in (line:=raw.split('#',1)[0].strip()))
    values={k.strip():v.strip().strip('\"\'') for k,v in yaml_values(meta).items()}
    digest=sha256(policy)
    if digest!=cfg['policy_sha256'] or values.get('sha256')!=digest or values.get('contract')!=cfg['policy_contract']:
        raise ValueError('policy bundle hash/contract differs from config; no policy substitution')
    urdf=root/'teleop/src/assets/R1.urdf'
    if not urdf.is_file():raise ValueError(f'URDF unavailable: {urdf}')
    return {'runtime_root':str(root.resolve()),'policy':str(policy.resolve()),'policy_sha256':digest,
            'contract':values['contract'],'urdf':str(urdf.resolve()),'urdf_sha256':sha256(urdf),
            'controller_source_sha256':{name:sha256(root/name) for name in ['controller/src/config/RobotSpec.hpp','controller/src/input/TeleopReceiver.hpp','controller/src/app/ApplicationPolicy.cpp','controller/config/locomotion.yaml','controller/config/gestures.yaml']},
            'runtime_reference_only':True,'note':'Disk artifacts verified; active owner policy is not acknowledged by UTL1.'}


def limits(urdf,cfg):
    tree=ET.parse(urdf)
    result={}
    for name in JOINTS:
        joint=tree.find(f".//joint[@name='{name}']");limit=joint.find('limit')
        lo,hi=float(limit.get('lower')),float(limit.get('upper'))
        if name in HEAD_JOINTS:
            cap=cfg['head_yaw_limit_rad'] if name==HEAD_JOINTS[0] else cfg['head_pitch_limit_rad']
            lo,hi=max(lo,-cap),min(hi,cap)
        result[name]=(lo+cfg['joint_margin_rad'],hi-cfg['joint_margin_rad'])
    return result


def validate_target(target,joint_limits):
    if set(target)-set(JOINTS):raise ValueError('unknown joint name')
    if not target:raise ValueError('empty target')
    for name,value in target.items():
        if type(value) not in (float,int) or not math.isfinite(value):raise ValueError('joint targets must be finite numbers')
        lo,hi=joint_limits[name]
        if not lo<=value<=hi:raise ValueError(f'{name}={value:g} outside [{lo:g},{hi:g}]')
    return target


def packet(seq,positions,enable=True,arms=True,head=True):
    if len(positions)!=12 or not all(math.isfinite(v) for v in positions):raise ValueError('12 finite positions required')
    # pad=1 declares the explicit enabled/STOP source contract expected by H4.
    return PACKET.pack(MAGIC,seq&0xffffffff,int(enable),int(arms),int(head),1,*positions)


def ramp_duration(start,end,cfg):
    # Quintic 10u^3-15u^4+6u^5: max derivative 1.875, second derivative 10/sqrt(3).
    if len(start)!=len(end) or not start:raise ValueError('matching nonempty trajectory required')
    duration=cfg['ramp_min_s']
    for i,(a,b) in enumerate(zip(start,end)):
        arm=len(start)==len(JOINTS) and i<len(ARM_JOINTS)
        velocity=cfg.get('arm_max_velocity_rad_s',cfg['max_velocity_rad_s']) if arm else cfg['max_velocity_rad_s']
        acceleration=cfg.get('arm_max_acceleration_rad_s2',cfg['max_acceleration_rad_s2']) if arm else cfg['max_acceleration_rad_s2']
        if not all(math.isfinite(v) and v>0 for v in [velocity,acceleration,duration]):
            raise ValueError('trajectory limits must be finite and positive')
        distance=abs(a-b)
        if not math.isfinite(distance):raise ValueError('nonfinite trajectory')
        duration=max(duration,1.875*distance/velocity,
                     math.sqrt((10/math.sqrt(3))*distance/acceleration))
    return duration


def interpolate(start,end,t,duration):
    u=min(1.,max(0.,t/duration));blend=u*u*u*(10+u*(-15+6*u))
    return [a+(b-a)*blend for a,b in zip(start,end)]


def load_chain(urdf,names):
    """URDF revolute chain as xyz, origin R, unit axis and physical limits."""
    import numpy as np
    tree=ET.parse(urdf);chain=[];previous_child=None
    for name in names:
        j=tree.find(f".//joint[@name='{name}']")
        if j is None:raise ValueError(f'missing URDF joint {name}')
        parent=j.find('parent').get('link');child=j.find('child').get('link')
        if previous_child is not None and parent!=previous_child:raise ValueError('disconnected URDF chain')
        previous_child=child
        origin=j.find('origin');axis=j.find('axis');bound=j.find('limit')
        xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        roll,pitch,yaw=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
        cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
        R=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        axis=np.fromstring(axis.get('xyz'),sep=' ');norm=np.linalg.norm(axis)
        if norm<1e-12:raise ValueError('zero joint axis')
        chain.append((xyz,R,axis/norm,float(bound.get('lower')),float(bound.get('upper'))))
    return chain


def chain_fk(chain,q):
    import numpy as np
    if len(chain)!=len(q):raise ValueError('joint vector dimension mismatch')
    T=np.eye(4)
    for angle,(xyz,R,a,_,_) in zip(q,chain):
        x,y,z=a;S=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        A=np.eye(4);A[:3,:3]=R@(np.eye(3)+math.sin(angle)*S+(1-math.cos(angle))*(S@S));A[:3,3]=xyz
        T=T@A
    return T


def point_arm(urdf,side,target,cfg,seed=None):
    """Orientation IK to a 3D point in waist_yaw_link, not a reaching/grasping task."""
    import numpy as np
    target=np.asarray(target,dtype=float)
    if target.shape!=(3,) or not np.all(np.isfinite(target)):raise ValueError('finite xyz target required')
    chain=load_chain(urdf,[f'{side}_{part}_joint' for part in ARM_NAMES])
    margin=cfg['joint_margin_rad']
    chain=[(xyz,R,axis,lo+margin,hi-margin) for xyz,R,axis,lo,hi in chain]
    lo=np.array([j[3] for j in chain]);hi=np.array([j[4] for j in chain])
    def fk(q):
        T=chain_fk(chain,q)
        axis=T[:3,0];origin=T[:3,3]+cfg['ee_offset_m']*axis
        return origin,axis
    shoulder=chain[0][0]
    if np.linalg.norm(target-shoulder)<0.1:raise ValueError('target too close to shoulder for pointing baseline')
    def error(q):
        origin,axis=fk(q);d=target-origin
        if np.linalg.norm(d)<1e-6:raise ValueError('target coincides with pointing origin')
        return axis-d/np.linalg.norm(d)
    default = cfg['pointing_seed_left_rad'] if side=='left' else cfg['pointing_seed_right_rad']
    nominal=np.array(seed if seed is not None else default,dtype=float)
    if nominal.shape!=(5,) or not np.all(np.isfinite(nominal)):raise ValueError('5-value arm seed required')
    q=np.clip(nominal,lo,hi);nominal=q.copy();eps=cfg['ik_jacobian_step_rad']
    for _ in range(cfg['ik_iterations']):
        e=error(q);J=np.column_stack([(error(q+np.eye(5)[i]*eps)-error(q-np.eye(5)[i]*eps))/(2*eps) for i in range(5)])
        delta=np.linalg.solve(J.T@J+(cfg['ik_damping']+cfg['ik_regularization'])*np.eye(5),-J.T@e-cfg['ik_regularization']*(q-nominal))
        size=np.linalg.norm(delta)
        if size>cfg['ik_step_rad']:delta*=cfg['ik_step_rad']/size
        q=np.clip(q+delta,lo,hi)
    origin,axis=fk(q);direction=target-origin;direction/=np.linalg.norm(direction)
    angle=math.degrees(math.acos(float(np.clip(axis@direction,-1,1))))
    return {'side':side,'frame':'waist_yaw_link','target_xyz_m':target.tolist(),'pointing_origin_m':origin.tolist(),
            'pointing_axis_xyz':axis.tolist(),'angular_error_deg':angle,'solved':angle<=cfg['pointing_max_error_deg'],
            'joints_rad':{f'{side}_{part}_joint':float(v) for part,v in zip(ARM_NAMES,q)},
            'limitations':['No self/environment collision or balance validation','Requires metric target in declared waist frame; bbox alone is insufficient']}
