"""Conservative endpoint grid from metric surface points in a named world frame.

Only observed ground endpoints mark free; no ray clearing, hole filling,
unobserved robot-footprint clearing or multi-frame accumulation. Obstacles win.
This diagnostic assumes a horizontal ground plane; it grants no authority.
"""
import math
import numpy as np
from ai_modules.navigation.approach import Grid, plan, segment_clear


def floor_support_diagnostics(points, tolerance_m, max_tilt_deg, iterations, seed):
    """Audit the exact RANSAC support gate; unrestricted fit is diagnostic only."""
    points=np.asarray(points,float)
    A=np.column_stack((points[:,:2],np.ones(len(points))));z=points[:,2]
    output={}
    for name,cap in (('configured',max_tilt_deg),('without_tilt_gate',89.9)):
        rng=np.random.default_rng(seed);best=0;tilt=None
        for _ in range(iterations):
            ids=rng.choice(len(points),3,replace=False)
            if abs(np.linalg.det(A[ids]))<1e-6:continue
            coeff=np.linalg.solve(A[ids],z[ids]);norm=math.sqrt(1+float(coeff[0]**2+coeff[1]**2))
            angle=math.degrees(math.acos(1/norm))
            if angle>cap:continue
            hits=int(np.count_nonzero(np.abs(A@coeff-z)/norm<=tolerance_m))
            if hits>best:best,tilt=hits,angle
        output[name]={'best_support_fraction':best/len(points),'tilt_deg':tilt}
    centre=points.mean(axis=0);_,_,V=np.linalg.svd(points-centre,full_matrices=False)
    residual=np.abs((points-centre)@V[-1])
    output['orthogonal_plane_residual_p95_m']=float(np.percentile(residual,95))
    return output


def fit_floor_candidate(points, tolerance_m, max_tilt_deg, min_fraction, iterations, seed):
    """RANSAC plane on an explicitly selected floor ROI, not floor truth.

    Returns T_candidate_floor_body: +z normal, x projected body-forward.
    No odometry height is used; per-view results must not be fused blindly.
    """
    points=np.asarray(points,float)
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)<30 or not np.isfinite(points).all()
            or not math.isfinite(tolerance_m) or tolerance_m<=0 or not 0<max_tilt_deg<45
            or not 0<min_fraction<=1 or type(iterations) is not int or not 1<=iterations<=1000):
        raise ValueError('invalid floor fit input')
    rng=np.random.default_rng(seed);best=None
    A=np.column_stack((points[:,:2],np.ones(len(points))));z=points[:,2]
    for _ in range(iterations):
        ids=rng.choice(len(points),3,replace=False)
        if abs(np.linalg.det(A[ids]))<1e-6:continue
        coeff=np.linalg.solve(A[ids],z[ids]);norm=math.sqrt(1+float(coeff[0]**2+coeff[1]**2))
        if math.degrees(math.acos(1/norm))>max_tilt_deg:continue
        mask=np.abs(A@coeff-z)/norm<=tolerance_m
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.mean()<min_fraction:raise ValueError('no supported floor candidate')
    coeff=np.linalg.lstsq(A[best],z[best],rcond=None)[0]
    normal=np.array([-coeff[0],-coeff[1],1.]);normal/=np.linalg.norm(normal)
    d=-float(coeff[2])*float(normal[2])
    residual=np.abs(points@normal+d);inliers=residual<=tolerance_m
    tilt=math.degrees(math.acos(float(normal[2])))
    if tilt>max_tilt_deg or inliers.mean()<min_fraction or np.min(np.ptp(points[inliers,:2],axis=0))<.2:
        raise ValueError('floor support too small or tilted')
    x=np.array([1.,0.,0.])-normal[0]*normal;x/=np.linalg.norm(x)
    T=np.eye(4);T[:3,:3]=np.array([x,np.cross(normal,x),normal]);T[2,3]=d
    return {'normal_body':normal.tolist(),'plane_d_m':d,'tilt_body_deg':tilt,
            'inlier_fraction':float(inliers.mean()),'residual_p95_m':float(np.percentile(residual[inliers],95)),
            'T_candidate_floor_body':T.tolist(),'verified':False}


class DepthGrid:
    def __init__(self, config):
        self.config = dict(config)
        for key in ('resolution_m', 'ground_tolerance_m', 'obstacle_min_height_m',
                    'obstacle_max_height_m', 'max_range_m'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid '+key)
        if not config['ground_tolerance_m'] < config['obstacle_min_height_m'] < config['obstacle_max_height_m']:
            raise ValueError('ground/obstacle bands overlap or reversed')
        for key in ('width', 'height', 'pixel_stride', 'min_ground_hits'):
            if type(config[key]) is not int or config[key] < 1:
                raise ValueError('invalid '+key)
        if config['width']*config['height'] > 10000:
            raise ValueError('diagnostic grid limited to 10000 cells')
        if not math.isfinite(config['ground_z_m']) or len(config['origin_m']) != 2 or not all(math.isfinite(x) for x in config['origin_m']):
            raise ValueError('invalid ground/origin')

    def build(self, ranges, valid, rays, transform, stamp):
        c = self.config
        ranges, valid, rays, T = np.asarray(ranges), np.asarray(valid), np.asarray(rays), np.asarray(transform, float)
        if (ranges.ndim != 2 or valid.shape != ranges.shape or rays.shape != (*ranges.shape, 3)
                or T.shape != (4,4) or not np.isfinite(T).all() or not math.isfinite(stamp)
                or not np.allclose(T[3], [0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3], np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(T[:3,:3]),1.)):
            raise ValueError('invalid depth geometry')
        stride = c['pixel_stride']
        r, v, ray = ranges[::stride,::stride], valid[::stride,::stride], rays[::stride,::stride]
        good = (v.astype(bool) & np.isfinite(r) & (r>0) & (r<=c['max_range_m'])
                & np.isfinite(ray).all(axis=-1) & np.isclose(np.linalg.norm(ray,axis=-1),1.,atol=1e-5))
        points = (ray[good]*r[good,None])@T[:3,:3].T+T[:3,3]
        shape = (c['height'], c['width'])
        ground, occupied, ambiguous = [np.zeros(shape, np.int32) for _ in range(3)]
        cell = np.floor((points[:,:2]-c['origin_m'])/c['resolution_m']).astype(int)
        inside = (cell[:,0]>=0)&(cell[:,0]<c['width'])&(cell[:,1]>=0)&(cell[:,1]<c['height'])
        cell, points = cell[inside], points[inside]
        z = points[:,2]-c['ground_z_m']
        floor = np.abs(z)<=c['ground_tolerance_m']
        obstacle = (z>=c['obstacle_min_height_m'])&(z<=c['obstacle_max_height_m'])
        # Below ground or uncertain low surfaces also forbid free classification.
        uncertain = (~floor)&(~obstacle)&(z<c['obstacle_min_height_m'])
        for hits, mask in ((ground,floor),(occupied,obstacle),(ambiguous,uncertain)):
            np.add.at(hits,(cell[mask,1],cell[mask,0]),1)
        rows = np.full(shape,-1,np.int8)
        rows[(ground>=c['min_ground_hits'])&(ambiguous==0)] = 0
        rows[occupied>0] = 1
        grid = Grid(tuple(tuple(int(v) for v in row) for row in rows),c['resolution_m'],tuple(c['origin_m']),stamp)
        return grid, {'ground_hits':ground,'obstacle_hits':occupied,'ambiguous_hits':ambiguous,
                      'point_count':int(len(points))}


def plan_standoff(grid, pose_xy, target_xy, standoff_m, radius_m):
    if not all(math.isfinite(v) for v in (*pose_xy,*target_xy,standoff_m,radius_m)) or standoff_m<=radius_m or radius_m<=0:
        raise ValueError('invalid planning geometry')
    direction = np.asarray(pose_xy)-target_xy
    distance = float(np.linalg.norm(direction))
    result = {'status':'blocked','reason':'inside_standoff','goal_xy_m':None,'path_xy_m':[],
              'navigation_ready':False,'command':{'vx_mps':0.,'wz_radps':0.,'authority':False}}
    if distance<=standoff_m:
        return result
    goal = np.asarray(target_xy)+direction/distance*standoff_m
    result['goal_xy_m'] = goal.tolist()
    if grid.clearance(pose_xy)<=radius_m:
        result['reason']='start_footprint_unknown_or_occupied'
    elif grid.clearance(goal)<=radius_m:
        result['reason']='goal_footprint_unknown_or_occupied'
    else:
        path = plan(grid,tuple(pose_xy),tuple(goal),radius_m)
        if path and all(segment_clear(grid,a,b,radius_m) for a,b in zip(path,path[1:])):
            result.update(status='diagnostic_path',reason='geometric_path_only',path_xy_m=path)
        else:
            result['reason']='no_known_path'
    return result
