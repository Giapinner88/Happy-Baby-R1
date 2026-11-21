"""Bearing-only R1 head planning, no motor interface. Uses project URDF geometry."""
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'HB/teleop/src'))
from teleop.r1.kinematics import load_joint_geometry,joint_transform


def solve_head_bearing(ray_optical, current_pitch_yaw, config):
    """Align camera forward with a fixed direction in waist_yaw_link (infinite range).

    This is an orientation approximation. Head-induced camera translation cannot
    be corrected without object range and a measured camera mount translation.
    """
    R=np.asarray(config['optical_to_head_rotation'],dtype=float)
    if R.shape!=(3,3) or not np.all(np.isfinite(R)) or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1):
        raise ValueError('optical_to_head_rotation must be a proper rotation')
    ray=np.asarray(ray_optical,dtype=float)
    q=np.asarray(current_pitch_yaw,dtype=float)
    if ray.shape!=(3,) or not np.all(np.isfinite(ray)) or np.linalg.norm(ray)<1e-9 or ray[2]<=0:
        raise ValueError('invalid forward optical ray')
    if q.shape!=(2,) or not np.all(np.isfinite(q)):
        raise ValueError('current state must be finite [pitch,yaw]')
    path=ROOT/config['urdf']
    pitch=load_joint_geometry('head_pitch_joint',path)
    yaw=load_joint_geometry('head_yaw_joint',path)
    margin=float(config['joint_margin_rad'])
    if margin<0:raise ValueError('negative joint margin')
    bounds=np.array([[max(pitch.lower_rad,-config['head_pitch_limit_rad'])+margin,
                       max(yaw.lower_rad,-config['head_yaw_limit_rad'])+margin],
                     [min(pitch.upper_rad,config['head_pitch_limit_rad'])-margin,
                       min(yaw.upper_rad,config['head_yaw_limit_rad'])-margin]])
    if np.any(bounds[0]>=bounds[1]):raise ValueError('empty joint envelope')
    if np.any(q<bounds[0]) or np.any(q>bounds[1]):raise ValueError('current head state outside planning envelope')
    count=config['solver_iterations'];eps=float(config['jacobian_step_rad'])
    damping=float(config['damping']);step=float(config['solver_step_rad'])
    if type(count) is not int or count<=0 or min(eps,damping,step)<=0:
        raise ValueError('invalid solver settings')
    def head_rotation(v):
        return (joint_transform(pitch,v[0])@joint_transform(yaw,v[1]))[:3,:3]
    target=head_rotation(q)@R@(ray/np.linalg.norm(ray))
    forward=R@np.array([0.,0.,1.])
    def direction(v):return head_rotation(v)@forward
    initial=q.copy()
    for _ in range(count):
        d=direction(q)
        J=np.column_stack([(direction(q+np.eye(2)[i]*eps)-direction(q-np.eye(2)[i]*eps))/(2*eps) for i in range(2)])
        delta=np.linalg.solve(J.T@J+damping*np.eye(2),J.T@(target-d))
        norm=np.linalg.norm(delta)
        if norm>step:delta*=step/norm
        q=np.clip(q+delta,*bounds)
    angle=float(np.degrees(np.arccos(np.clip(direction(q)@target,-1,1))))
    return {'reference_frame':'waist_yaw_link','current_pitch_yaw_rad':initial.tolist(),
            'candidate_pitch_yaw_rad':q.tolist(),'delta_pitch_yaw_rad':(q-initial).tolist(),
            'candidate_pitch_yaw_deg':np.degrees(q).tolist(),'target_direction_waist_xyz':target.tolist(),
            'residual_deg':angle,'within_angular_tolerance':angle<=config['max_residual_deg'],
            'planning_bounds_pitch_yaw_rad':bounds.tolist(),
            'approximation':'Bearing only, ignores camera-origin translation; not an exact near-object look_at.'}
