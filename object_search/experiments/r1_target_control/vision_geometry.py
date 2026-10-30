"""Coupled head+arm proposal from one optical ray and matching capture state."""
import math
import numpy as np
from control import (ARM_NAMES,HEAD_JOINTS,chain_fk,limits,load_chain,point_arm,validate_target)


def validate_config(vision):
    for name in ['assumed_range_m','max_state_rpc_gap_s','head_solver_step_rad',
                 'head_jacobian_step_rad','head_damping','head_max_error_deg']:
        value=vision[name]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('invalid '+name)
    preferred_arm(0,vision['arm_side_deadband_deg'])
    if type(vision['head_solver_iterations']) is not int or vision['head_solver_iterations']<=0:
        raise ValueError('invalid head_solver_iterations')
    for name in ['camera_mount_verified','allow_other_arm_if_ik_fails','try_measured_arm_seed']:
        if type(vision[name]) is not bool:raise ValueError('invalid '+name)


def preferred_arm(yaw_rad,deadband_deg):
    if not math.isfinite(yaw_rad) or not math.isfinite(deadband_deg) or not 0<=deadband_deg<90:
        raise ValueError('invalid yaw or neutral deadband')
    if yaw_rad>math.radians(deadband_deg):return 'left','head_target_left'
    if yaw_rad<-math.radians(deadband_deg):return 'right','head_target_right'
    return 'right','head_target_neutral_use_right'


def camera_pose(urdf,pitch_yaw,vision):
    q=np.asarray(pitch_yaw,dtype=float)
    R=np.asarray(vision['optical_to_head_rotation'],dtype=float)
    t=np.asarray(vision['camera_translation_in_head_m'],dtype=float)
    if q.shape!=(2,) or not np.all(np.isfinite(q)):raise ValueError('finite pitch/yaw required')
    if R.shape!=(3,3) or not np.all(np.isfinite(R)) or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1):
        raise ValueError('invalid camera mount rotation')
    if t.shape!=(3,) or not np.all(np.isfinite(t)):raise ValueError('invalid camera mount translation')
    chain=load_chain(urdf,['head_pitch_joint','head_yaw_joint'])
    T=chain_fk(chain,q);mount=np.eye(4);mount[:3,:3]=R;mount[:3,3]=t
    return T@mount


def solve_head_point(urdf,point,q0,control,vision):
    point=np.asarray(point,dtype=float);q=np.asarray(q0,dtype=float).copy()
    if point.shape!=(3,) or not np.all(np.isfinite(point)):raise ValueError('invalid target xyz')
    bounds=limits(urdf,control)
    lo=np.array([bounds['head_pitch_joint'][0],bounds['head_yaw_joint'][0]])
    hi=np.array([bounds['head_pitch_joint'][1],bounds['head_yaw_joint'][1]])
    if np.any(q<lo) or np.any(q>hi):raise ValueError('head at capture outside planning envelope')
    def error(values):
        T=camera_pose(urdf,values,vision);direction=point-T[:3,3]
        if np.linalg.norm(direction)<1e-8:raise ValueError('target at camera origin')
        return T[:3,2]-direction/np.linalg.norm(direction)
    eps=vision['head_jacobian_step_rad']
    for _ in range(vision['head_solver_iterations']):
        J=np.column_stack([(error(q+np.eye(2)[i]*eps)-error(q-np.eye(2)[i]*eps))/(2*eps) for i in range(2)])
        delta=np.linalg.solve(J.T@J+vision['head_damping']*np.eye(2),-J.T@error(q))
        n=np.linalg.norm(delta)
        if n>vision['head_solver_step_rad']:delta*=vision['head_solver_step_rad']/n
        q=np.clip(q+delta,lo,hi)
    residual=error(q)
    angle=math.degrees(2*math.asin(min(1.,float(np.linalg.norm(residual))/2)))
    return {'frame':'waist_yaw_link','captured_pitch_yaw_rad':list(map(float,q0)),
            'candidate_pitch_yaw_rad':q.tolist(),'candidate_pitch_yaw_deg':np.degrees(q).tolist(),
            'residual_deg':angle,'solved':angle<=vision['head_max_error_deg'],
            'joints_rad':{'head_pitch_joint':float(q[0]),'head_yaw_joint':float(q[1])}}


def combined_target(ray,capture,urdf,control,vision):
    validate_config(vision)
    ray=np.asarray(ray,dtype=float)
    if ray.shape!=(3,) or not np.all(np.isfinite(ray)) or ray[2]<=0:raise ValueError('invalid forward optical ray')
    ray/=np.linalg.norm(ray)
    distance=vision['assumed_range_m']
    if type(distance) not in (int,float) or not math.isfinite(distance) or distance<=0:
        raise ValueError('assumed range must be positive and finite')
    state=capture['state_near_rpc_midpoint_proxy'];q=state['q_rad']
    gap=abs(state['received_monotonic_s']-capture['rpc_midpoint_proxy_monotonic_s'])
    if not math.isfinite(gap) or gap>vision['max_state_rpc_gap_s']:raise ValueError('state too far from RPC midpoint proxy')
    q0=[q['head_pitch_joint'],q['head_yaw_joint']]
    T=camera_pose(urdf,q0,vision)
    point=T[:3,3]+T[:3,:3]@(distance*ray)
    head=solve_head_point(urdf,point,q0,control,vision)
    details={'assumed_range_m':distance,'range_source':'assumed_not_measured',
        'approximate_target_waist_xyz_m':point.tolist(),'camera_pose_at_capture':T.tolist(),
        'head':head,'camera_mount_verified':vision['camera_mount_verified'],
        'exposure_timestamp_verified':False,'state_rpc_proxy_gap_s':gap,
        'limitations':['Nominal camera extrinsics and assumed range: approximate direction only',
                      'No collision/balance validation; waist/body/scene assumed stationary after capture']}
    if not head['solved']:return {'status':'unreachable_head','joints_rad':None,'details':details}
    side,reason=preferred_arm(head['joints_rad']['head_yaw_joint'],vision['arm_side_deadband_deg'])
    details.update(preferred_arm=side,arm_selection_reason=reason,arm_side_deadband_deg=vision['arm_side_deadband_deg'])
    attempts=[]
    fallback_allowed=vision['allow_other_arm_if_ik_fails'] and reason!='head_target_neutral_use_right'
    sides=[side]+(['right' if side=='left' else 'left'] if fallback_allowed else [])
    for candidate in sides:
        measured=[q.get(f'{candidate}_{part}_joint') for part in ARM_NAMES]
        seeds=[('configured',None)]
        if vision['try_measured_arm_seed'] and all(v is not None for v in measured):seeds.insert(0,('measured_at_capture',measured))
        solved=[]
        for seed_name,seed in seeds:
            arm=point_arm(urdf,candidate,point,control,seed)
            attempts.append({'side':candidate,'seed':seed_name,'angular_error_deg':arm['angular_error_deg'],'solved':arm['solved']})
            if arm['solved']:solved.append(arm)
        if not solved:continue
        if all(v is not None for v in measured):
            arm=min(solved,key=lambda r:sum((r['joints_rad'][f'{candidate}_{part}_joint']-v)**2 for part,v in zip(ARM_NAMES,measured)))
        else:arm=min(solved,key=lambda r:r['angular_error_deg'])
        joints={**head['joints_rad'],**arm['joints_rad']};validate_target(joints,limits(urdf,control))
        details.update(selected_arm=candidate,arm_fallback_used=candidate!=side,arm=arm,ik_attempts=attempts,
                       untouched_arm_behavior='Held at fresh encoder at stream start by existing sender')
        return {'status':'planned','joints_rad':joints,'details':details}
    details['ik_attempts']=attempts
    return {'status':'unreachable_arm','joints_rad':None,'details':details}
