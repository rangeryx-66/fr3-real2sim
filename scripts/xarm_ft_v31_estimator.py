"""SG121 consistent dynamics processing; no scene, hidden dynamics or controls.

All physical terms are formed at their measured timestamps, shifted to the
fixed hinge origin, then passed through ONE common SG2-equivalent linear
operator. In particular K(sign(v)) is not sign(K(v)).
"""
from functools import lru_cache
import numpy as np
from scipy.signal import savgol_coeffs
from scipy.spatial.transform import Rotation

VERSION='V3.1-common-SG2'
DEFAULT_WINDOW=121
DEFAULT_EDGE=121


@lru_cache(maxsize=8)
def kernel(window=DEFAULT_WINDOW):
    if window<7 or window%2!=1:raise ValueError('odd SG/order3 window required')
    # Discrete inverse of central second difference, normalized at DC.
    c=savgol_coeffs(window,3,deriv=2,delta=1.,use='conv')
    k=np.roll(np.cumsum(np.cumsum(c)),1);k[0]=0.;k/=k.sum()
    k.setflags(write=False)
    return k


def common_filter(x,window=DEFAULT_WINDOW):
    x=np.asarray(x,dtype=float);pad=[(0,0)]*x.ndim;pad[0]=(window//2,window//2)
    return np.apply_along_axis(lambda v:np.convolve(v,kernel(window),mode='valid'),0,np.pad(x,pad,mode='edge'))


def second_difference(p,dt):
    result=np.empty_like(p);result[1:-1]=(p[2:]-2*p[1:-1]+p[:-2])/dt**2
    result[0]=result[1];result[-1]=result[-2]
    return result


def process(rows,prior,bias,native_velocity,window=DEFAULT_WINDOW,edge=DEFAULT_EDGE):
    n=len(rows);support=window//2+2;edge=max(edge,support)
    if n<2*edge+5:return None
    if len({r['identification_segment'] for r in rows})!=1:raise ValueError('split FIT/VALIDATION/segment boundaries before processing')
    t=np.array([r['t'] for r in rows]);dt=float(np.median(np.diff(t)))
    if np.max(abs(np.diff(t)-dt))>1e-7:raise ValueError('split sampling gaps before processing')
    if not all(r['wrist_ft']['valid'] for r in rows):raise ValueError('split invalid F/T samples before processing')
    velocity=np.asarray(native_velocity,dtype=float)
    if velocity.shape!=(n,) or not np.isfinite(velocity).all():raise ValueError('native measured velocity required; no silent derivative fallback')
    ft=[r['wrist_ft'] for r in rows];T=np.array([f['T_world_sensor'] for f in ft]);R=T[:,:3,:3];sensor=T[:,:3,3]
    raw=np.array([f['raw_wrench_sensor'] for f in ft]);unbiased=raw-np.asarray(bias)
    world=np.c_[np.einsum('nij,nj->ni',R,unbiased[:,:3]),np.einsum('nij,nj->ni',R,unbiased[:,3:])]
    gravity=np.array([f['tool_gravity_wrench_world'] for f in ft]);inertial=np.zeros((n,6))
    for j in range(len(ft[0]['body_states'])):
        bodies=[f['body_states'][j] for f in ft];mass=bodies[0]['mass_kg']
        p=np.array([b['com_world_m'] for b in bodies]);B=np.array([b['T_world_body'] for b in bodies])[:,:3,:3];I=np.array([b['inertia_world_kg_m2'] for b in bodies])
        omega=np.empty((n,3))
        omega[1:-1]=Rotation.from_matrix(B[2:]@B[:-2].transpose(0,2,1)).as_rotvec()/(2*dt)
        omega[0]=Rotation.from_matrix(B[1]@B[0].T).as_rotvec()/dt;omega[-1]=Rotation.from_matrix(B[-1]@B[-2].T).as_rotvec()/dt
        alpha=np.gradient(omega,dt,axis=0);force=mass*second_difference(p,dt)
        inertial[:,:3]+=force
        inertial[:,3:]+=np.einsum('nij,nj->ni',I,alpha)+np.cross(omega,np.einsum('nij,nj->ni',I,omega))+np.cross(p-sensor,force)
    axis=np.asarray(prior['axis_world'],dtype=float);axis/=np.linalg.norm(axis);origin=np.asarray(prior['origin_world_m'])
    def fixed_origin(w):return np.c_[w[:,:3],w[:,3:]+np.cross(sensor-origin,w[:,:3])]
    world=fixed_origin(world);gravity=fixed_origin(gravity);inertial=fixed_origin(inertial)
    # Projection is now at a FIXED reference, so filtering and projection commute.
    selector=slice(3,6) if prior['joint_type']=='revolute' else slice(0,3)
    unfiltered_torque=(world+gravity-inertial)[:,selector]@axis
    torque=common_filter(unfiltered_torque,window)
    q=np.deg2rad([r['door_angle_deg'] for r in rows]);qd=common_filter(velocity,window);qdd=common_filter(np.gradient(velocity,dt),window)
    direction=common_filter(np.sign(velocity),window);valid=np.ones(n,dtype=bool);tags=[]
    for i,r in enumerate(rows):
        reasons=[]
        if i<edge or i>=n-edge:reasons.append('segment-local derivative/filter boundary')
        if r.get('force_event',{}).get('status') not in [None,'NORMAL']:reasons.append('original protection event')
        if r.get('relative_translation_slip_m',0)>.005:reasons.append('original slip data-quality tag')
        if abs(qd[i])<1e-4:reasons.append('original stiction/uncertain direction data-quality tag')
        tags.append(reasons);valid[i]=not reasons
    return dict(t=t,q=q,qd=qd,qdd=qdd,direction=direction,native_qd=velocity,torque=torque,unfiltered_torque=unfiltered_torque,
        raw=raw,raw_world_hinge=world,gravity_world_hinge=gravity,inertial_world_hinge=inertial,
        valid=valid,tags=tags,segments=np.array([r['identification_segment'] for r in rows]),
        processing=dict(version=VERSION,window=window,dt_s=dt,boundary_exclusion_samples=edge,centered_future_support_samples=support,
            filter_lookahead_s=support*dt,kernel='SG3 second-derivative equivalent acceleration smoother; unit DC gain',
            tool_compensation_count=1,reference='fixed world hinge origin before filtering',friction_basis='K(sign(native_velocity))',
            finite_difference_error_remains=True,availability='completed segment plus worker latency; not zero-latency online state'))


def regression_samples(d):
    result=[];groups=np.floor(d['t']/.1).astype(np.int64)
    for label in dict.fromkeys(d['segments']):
        for group in np.unique(groups[(d['segments']==label)&d['valid']]):
            ix=np.flatnonzero((d['segments']==label)&d['valid']&(groups==group))
            if len(ix)<5:continue
            result.append(dict(segment=str(label),t=float(d['t'][ix].mean()),q=float(d['q'][ix].mean()),qd=float(d['qd'][ix].mean()),
                qdd=float(d['qdd'][ix].mean()),direction=float(d['direction'][ix].mean()),tau=float(d['torque'][ix].mean())))
    return result
