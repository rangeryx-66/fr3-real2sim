"""Robust SE(3) revolute calibration. No plant, joint state, or truth input.

Theta[0]=0 fixes only the angular coordinate gauge: the corresponding reference
pose is fitted, not copied from the first measurement. Axis point is constrained
to the plane through the observation centroid normal to the estimated axis.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation
from interaction_identification.fitting import fit_articulation


def unit(v):
    return np.asarray(v, float) / np.linalg.norm(v)


def basis(a):
    e = np.eye(3)[np.argmin(np.abs(a))]
    u = unit(np.cross(a, e))
    return np.column_stack((u, np.cross(a, u)))


def fit_se3(poses, position_noise_m=2e-5, rotation_noise_rad=2e-5,
            maximum_samples=180, max_nfev=160, fixed_model=None):
    poses = np.asarray(poses, float)
    if len(poses) < 12 or poses.shape[1:] != (4, 4) or not np.isfinite(poses).all():
        raise ValueError('INSUFFICIENT_OR_INVALID_MEASURED_SE3')
    indices = np.unique(np.linspace(0, len(poses)-1, min(maximum_samples, len(poses))).round().astype(int))
    T = poses[indices]; p = T[:, :3, 3]; R = T[:, :3, :3]; n = len(T)
    old = fit_articulation(T)
    seed = (fixed_model or old)['revolute']; a0 = unit(seed['axis'])
    B = basis(a0); anchor = p.mean(0); c0 = np.asarray(seed['point_on_axis'])
    c0 = c0-a0*np.dot(a0, c0-anchor)
    theta = Rotation.from_matrix(R @ R[0].T).as_rotvec() @ a0
    free = fixed_model is None
    global_n = 10 if free else 6
    x0 = np.r_[([0., 0., *(B.T@(c0-anchor))] if free else []), p[0], np.zeros(3), theta[1:]]

    def unpack(x):
        if free:
            a = unit(a0+B@x[:2]); u = unit(B[:,0]-a*np.dot(a,B[:,0])); C = np.column_stack((u,np.cross(a,u)))
            c = anchor+C@x[2:4]; j = 4
        else:
            a, c, j = a0, c0, 0
        pref = x[j:j+3]; Rref = Rotation.from_rotvec(x[j+3:j+6]).as_matrix() @ R[0]
        angles = np.r_[0., x[global_n:]]
        rot = Rotation.from_rotvec(angles[:, None]*a).as_matrix()
        pred_p = c+np.einsum('nij,j->ni', rot, pref-c)
        pred_R = rot@Rref
        return a, c, pref, Rref, angles, pred_p, pred_R

    # basis(a0) equals B. The parameterization has 2 axis + 2 line + 6 reference
    # DOFs. The first angle is a coordinate origin, not an exact observation.
    def residual(x):
        *_, pp, rr = unpack(x)
        return np.c_[(pp-p)/position_noise_m,
                     Rotation.from_matrix(rr@R.transpose(0,2,1)).as_rotvec()/rotation_noise_rad].ravel()

    sparsity = lil_matrix((6*n, len(x0)), dtype=int)
    sparsity[:, :global_n] = 1
    for i in range(1,n): sparsity[6*i:6*i+6, global_n+i-1] = 1
    result = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(), loss='soft_l1',
                           f_scale=1., x_scale='jac', max_nfev=max_nfev,
                           ftol=1e-8, xtol=1e-9, gtol=1e-7)
    a,c,pref,Rref,angles,pp,rr = unpack(result.x)
    ep = np.linalg.norm(pp-p,axis=1)
    er = np.linalg.norm(Rotation.from_matrix(rr@R.transpose(0,2,1)).as_rotvec(),axis=1)
    reference = np.eye(4);reference[:3,:3]=Rref;reference[:3,3]=pref
    J = result.jac.toarray()
    # Eliminate phase nuisance variables, then report normalized structure
    # information singular values. No GT error is an acceptance criterion.
    Js,Jn = J[:,:4],J[:,4:]
    info = Js-Jn@np.linalg.lstsq(Jn,Js,rcond=1e-10)[0] if free else np.zeros((6*n,4))
    singular = np.linalg.svd(info,compute_uv=False)
    out = {'joint_type':'revolute','sample_count':len(poses),'travel_m':float(np.max(np.linalg.norm(p-p[0],axis=1))),
           'confidence':.999 if result.success else 0.,
           'confidence_definition':'optimizer convergence flag, not calibrated probability or acceptance',
           'revolute':{'axis':a.tolist(),'point_on_axis':c.tolist(),'radius_m':float(np.linalg.norm(np.cross(a,pref-c))),
                       'angle_span_rad':float(np.ptp(angles)), 'position_rmse_m':float(np.sqrt(np.mean(ep**2))),
                       'rotation_rmse_rad':float(np.sqrt(np.mean(er**2))),
                       'axis_point_gauge':'axis-normal plane through measured-position centroid'},
           'reference_pose':reference.tolist(),'relative_angles_rad':angles.tolist(),
           'sample_indices':indices.tolist(),'optimizer':{'success':bool(result.success),'message':result.message,'nfev':result.nfev,
                'robust_cost':float(result.cost),'normalized_rmse':float(np.sqrt(np.mean(residual(result.x)**2))),
                'structure_information_singular_values':singular.tolist()},
           'input_source':'measured EE SE3 only','position_noise_m':position_noise_m,'rotation_noise_rad':rotation_noise_rad,
           'fixed_axis_validation':not free}
    return out


def modality_consistency(poses, position_noise_m=2e-5, rotation_noise_rad=2e-5):
    T = np.asarray(poses,float);p=T[:,:3,3];R=T[:,:3,:3]
    _,sp,Vp=np.linalg.svd(p-p.mean(0),full_matrices=False)
    rv=Rotation.from_matrix(R@Rotation.from_matrix(R).mean().as_matrix().T).as_rotvec()
    _,sr,Vr=np.linalg.svd(rv,full_matrices=False)
    ap,ar=Vp[-1],Vr[0]
    angle=float(np.arccos(np.clip(abs(ap@ar),0,1)))
    # Short-arc plane normal is ill-conditioned when curvature signal is at
    # sensor noise. Signal/noise based uncertainty, not a vertical-axis prior.
    curvature=sp[1]/np.sqrt(len(p));angular_span=sr[0]/np.sqrt(len(p))
    uncertainty=np.arctan2(position_noise_m,max(curvature,1e-12))+np.arctan2(rotation_noise_rad,max(angular_span,1e-12))
    return {'position_axis':ap.tolist(),'rotation_axis':ar.tolist(),'axis_disagreement_deg':float(np.rad2deg(angle)),
            'position_curvature_rms_m':float(curvature),'uncertainty_scale_deg':float(np.rad2deg(uncertainty)),
            'position_axis_observable':bool(curvature>5*position_noise_m),
            'consistent_with_noise':bool(angle<3*uncertainty),
            'interpretation':'modality disagreement is retained; no averaging or GT acceptance'}


def predict_action(poses, fit, position_noise_m=2e-5, rotation_noise_rad=2e-5):
    """Cross-action structure prediction: phase comes only from EE orientation.

    Fit only this grasp's constant reference transform. Neither axis nor center
    nor per-frame phase may move to absorb translational validation residuals.
    This is a geometric validation, separate from unseen dynamic prediction.
    """
    T=np.asarray(poses,float);p=T[:,:3,3];R=T[:,:3,:3]
    a=unit(fit['revolute']['axis']);c=np.asarray(fit['revolute']['point_on_axis'])
    Rref=Rotation.from_matrix(R).mean().as_matrix()
    theta=Rotation.from_matrix(R@Rref.T).as_rotvec()@a
    rr=Rotation.from_rotvec(theta[:,None]*a).as_matrix()
    pref=c+np.mean(np.einsum('nji,nj->ni',rr,p-c),axis=0)
    pp=c+np.einsum('nij,j->ni',rr,pref-c)
    ep=pp-p;er=Rotation.from_matrix((rr@Rref)@R.transpose(0,2,1)).as_rotvec()
    return {'position_rmse_m':float(np.sqrt(np.mean(np.sum(ep**2,axis=1)))),
            'rotation_rmse_rad':float(np.sqrt(np.mean(np.sum(er**2,axis=1)))),
            'normalized_rmse':float(np.sqrt(np.mean(np.c_[ep/position_noise_m,er/rotation_noise_rad]**2))),
            'angle_span_deg':float(np.rad2deg(np.ptp(theta))),
            'phase_source':'measured orientation projected on frozen axis; validation position never determines phase',
            'axis_center_updated':False,'reference_pose_reestimated':True}
