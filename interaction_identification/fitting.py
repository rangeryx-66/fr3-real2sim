"""Fit line/screw-axis models from measured SE(3); no asset/GT inputs."""
import numpy as np
from scipy.spatial.transform import Rotation


def fit_articulation(poses,position_noise_m=5e-5,rotation_noise_rad=1e-4):
    poses=np.asarray(poses,float)
    if poses.ndim!=3 or poses.shape[1:]!=(4,4) or len(poses)<12:raise ValueError('at least 12 SE(3) samples required')
    p=poses[:,:3,3];R=poses[:,:3,:3];relative=R@R[0].T
    rotvec=Rotation.from_matrix(relative).as_rotvec();centre=p.mean(0)
    _,s,V=np.linalg.svd(p-centre,full_matrices=False);line_axis=V[0]
    line_pred=centre+((p-centre)@line_axis)[:,None]*line_axis
    line_pos=float(np.sqrt(np.mean(np.sum((p-line_pred)**2,axis=1))))
    line_rot=float(np.sqrt(np.mean(np.sum(rotvec**2,axis=1))))
    travel=float(np.max(np.linalg.norm(p-p[0],axis=1)))
    result={'sample_count':len(p),'travel_m':travel,'prismatic':{'axis':line_axis.tolist(),
            'point_on_axis':centre.tolist(),'position_rmse_m':line_pos,'rotation_rmse_rad':line_rot}}
    if travel<.001:
        return {**result,'joint_type':'UNOBSERVABLE','confidence':0.,'reason':'less than 1 mm observed motion'}
    _,sr,vr=np.linalg.svd(rotvec,full_matrices=False);axis=vr[0]
    theta=rotvec@axis;span=float(np.ptp(theta));axis_rot_error=rotvec-theta[:,None]*axis
    axis_rotation=Rotation.from_rotvec(theta[:,None]*axis).as_matrix()
    A=np.concatenate([np.eye(3)-r for r in axis_rotation],axis=0)
    b=np.concatenate([v-r@p[0] for v,r in zip(p,axis_rotation)])
    # Axis-point gauge: choose its intersection with the first EE orbit plane.
    A=np.vstack((A,axis[None,:]));b=np.r_[b,axis@p[0]]
    axis_point=np.linalg.lstsq(A,b,rcond=None)[0]
    circle_pred=axis_point+np.einsum('nij,j->ni',axis_rotation,p[0]-axis_point)
    circle_pos=float(np.sqrt(np.mean(np.sum((p-circle_pred)**2,axis=1))))
    circle_rot=float(np.sqrt(np.mean(np.sum(axis_rot_error**2,axis=1))))
    radius=float(np.linalg.norm(np.cross(axis,p[0]-axis_point)))
    n=len(p)
    line_cost=n*((line_pos/position_noise_m)**2+(line_rot/rotation_noise_rad)**2)+4*np.log(n)
    circle_cost=n*((circle_pos/position_noise_m)**2+(circle_rot/rotation_noise_rad)**2)+5*np.log(n)
    observable=span>.002 and sr[0]>1e-4 and radius<10.
    selected='revolute' if observable and circle_cost<line_cost else 'prismatic'
    gap=abs(circle_cost-line_cost)/max(1.,line_cost,circle_cost)
    result.update(joint_type=selected,confidence=float(min(.999,gap)),
                  confidence_definition='relative model-score gap; not a calibrated probability',
                  scores={'line':float(line_cost),'circle':float(circle_cost)},
                  revolute={'axis':axis.tolist(),'point_on_axis':axis_point.tolist(),
                            'radius_m':radius,'angle_span_rad':span,'position_rmse_m':circle_pos,
                            'rotation_rmse_rad':circle_rot,'axis_point_gauge':'first EE orbit plane'})
    return result


def evaluate(fit,ground_truth,first_position,observed_poses=None):
    """Evaluation only. The control/fitting modules never receive this input."""
    result={'type_correct':fit['joint_type']==ground_truth['joint_type']}
    if fit['joint_type'] not in ('revolute','prismatic'):return result
    model=fit[fit['joint_type']];axis=np.asarray(model['axis']);gt_axis=np.asarray(ground_truth['axis_world'])
    result['axis_angular_error_deg']=float(np.rad2deg(np.arccos(np.clip(abs(axis@gt_axis),0,1))))
    if fit['joint_type']=='revolute':
        point=np.asarray(model['point_on_axis']);origin=np.asarray(ground_truth['origin_world'])
        result['axis_line_distance_m']=float(np.linalg.norm(np.cross(gt_axis,point-origin)))
        projected=origin+gt_axis*np.dot(gt_axis,np.asarray(first_position)-origin)
        result['joint_origin_error_in_observed_plane_m']=float(np.linalg.norm(point-projected))
        result['raw_origin_point_distance_m']=float(np.linalg.norm(point-origin))
        result['origin_note']='point along axis is unidentifiable; raw point distance includes gauge difference'
    if observed_poses is not None and fit['joint_type']=='revolute':
        poses=np.asarray(observed_poses);p=poses[:,:3,3]
        angles=Rotation.from_matrix(poses[:,:3,:3]@poses[0,:3,:3].T).as_rotvec()@gt_axis
        rg=Rotation.from_rotvec(angles[:,None]*gt_axis).as_matrix()
        predicted=origin+np.einsum('nij,j->ni',rg,p[0]-origin)
        result['gt_constrained_reconstruction_rmse_m']=float(np.sqrt(np.mean(np.sum((predicted-p)**2,axis=1))))
    return result
