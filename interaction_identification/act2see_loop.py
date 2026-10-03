"""Small EE-only interaction loop; no object model or simulator truth inputs."""
from collections import deque
import json
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_identification.fitting import fit_articulation


def unit(v):
    v = np.asarray(v, float)
    return v / max(np.linalg.norm(v), 1e-12)


def bounded(v, cap):
    return v * min(1., cap / max(np.linalg.norm(v), 1e-12))


class TactileRetention:
    """Observe grasp-surface drift without treating pressure redistribution as slip.

    The full native contact manifold is a simulated tactile/proximity observation,
    not a query of object pose or joint state. Its support plane is insensitive to
    which corner carries load. This measures normal drift and surface tilt only:
    in-plane translation is unobservable and is NEVER reported as zero true slip.
    Bilateral load, aperture, contact-region and loss guards remain independent.
    """
    def __init__(self, fingers, window_s=.08):
        self.fingers=fingers;self.window_s=window_s;self.history=deque()
        self.reference=None;self.current={};self.drift_m=0.;self.planes={}

    def observe(self, t, T, contacts):
        inv=np.linalg.inv(T);patches={};self.planes={}
        for finger in self.fingers:
            rows=[c for c in contacts if c['finger']==finger and c['allowed_pad_target']
                  and c.get('contact_point_world_m') is not None]
            if len(rows)<3:continue
            points=np.array([(inv@np.r_[c['contact_point_world_m'],1])[:3] for c in rows])
            center=points.mean(0);_,singular,axes=np.linalg.svd(points-center,full_matrices=False)
            if singular[1]<1e-6:continue
            normal=axes[-1];radius=float(np.linalg.norm(points-center,axis=1).max())
            self.planes[finger]=(center,normal,radius)
            patches[finger]=center
        self.current=patches
        observed=[]
        if self.reference is not None:
            for finger,(center,normal,radius) in self.planes.items():
                old,n0,r0=self.reference[finger]
                normal_drift=abs(float((center-old)@normal))
                tilt_displacement=r0*float(np.linalg.norm(np.cross(normal,n0)))
                observed.append(normal_drift+tilt_displacement)
        self.history.append((t,max(observed,default=0.)))
        while self.history and self.history[0][0]<t-self.window_s:self.history.popleft()
        self.drift_m=float(np.mean([v for _,v in self.history]))
        return {'contact_patch_local_m':{f:p.tolist() for f,p in self.current.items()},
                'tactile_patch_drift_m':self.drift_m,
                'slip_measurement':'observed contact-plane normal drift + tilt at patch radius; not full relative slip',
                'unobservable_slip_components':['translation within parallel contact planes','rotation about their normal'],
                'contact_plane_count':len(self.planes)}

    def arm(self):
        if len(self.planes)!=2:raise RuntimeError('TACTILE_RETENTION_REFERENCE_UNAVAILABLE')
        self.reference={f:(c.copy(),n.copy(),r) for f,(c,n,r) in self.planes.items()}
        self.history.clear();self.drift_m=0.


class ConstrainedDrive:
    """Directional impedance, zero orthogonal/rotation stiffness.

    Gravity/Coriolis compensation comes only from the robot model. An implicit
    damping solve avoids explicit high-damping instability at the physics step.
    No six-dimensional pose target, object wrench, attachment or joint model.
    """
    speed_m_s = .0005
    force_cap_n = .20
    moment_cap_nm = .01
    drive_stiffness_n_m = 150.
    damping_along_ns_m = 100.
    damping_orthogonal_ns_m = 12.
    damping_rotation_nms_rad = .04

    def __init__(self, T, direction):
        self.direction = unit(direction)
        self.reference = np.asarray(T)[:3,3].copy()
        self.elapsed = 0.
        self.last = {}
        self.active = True

    def set_direction(self, direction, T):
        self.direction = unit(direction)
        self.reference = np.asarray(T)[:3,3].copy()
        self.elapsed = 0.

    def refresh_tangent(self, direction, T):
        # Update a short incremental tangent without restarting the drive ramp
        # or discarding its bounded compliant lead at every 1 mm segment.
        lead=np.clip(self.direction@(self.reference-T[:3,3]),-.002,.002)
        self.direction=unit(direction)
        self.reference=T[:3,3]+lead*self.direction

    def command(self, T, J, M, qdot, gravity, coriolis, dt):
        d = self.direction
        self.elapsed += dt
        speed = self.speed_m_s * min(1., self.elapsed/2.) if self.active else 0.
        self.reference += d * speed * dt
        error = np.clip(d @ (self.reference-T[:3,3]), -.002, .002)
        feed = np.r_[d*np.clip(self.drive_stiffness_n_m*error, -self.force_cap_n, self.force_cap_n), np.zeros(3)]
        D = np.zeros((6,6))
        D[:3,:3] = self.damping_orthogonal_ns_m*np.eye(3) + (self.damping_along_ns_m-self.damping_orthogonal_ns_m)*np.outer(d,d)
        D[3:,3:] = self.damping_rotation_nms_rad*np.eye(3)
        if not self.active:
            feed[:] = 0.
        acceleration = np.linalg.solve(M+dt*J.T@D@J+1e-9*np.eye(len(qdot)), J.T@(feed-D@J@qdot))
        wrench = feed-D@J@(qdot+dt*acceleration)
        wrench[:3] = bounded(wrench[:3], self.force_cap_n)
        wrench[3:] = bounded(wrench[3:], self.moment_cap_nm)
        tau = gravity+coriolis+J.T@wrench
        self.last = {'direction_world': d.tolist(), 'command_wrench_world': wrench.tolist(),
                     'measured_tcp_twist': (J@qdot).tolist(), 'drive_reference_world_m':self.reference.tolist(),
                     'orthogonal_stiffness_n_m':0., 'rotational_stiffness_nm_rad':0.,
                     'force_cap_n':self.force_cap_n, 'moment_cap_nm':self.moment_cap_nm}
        return tau


class InteractionMemory:
    """Probe -> observe -> fit -> update -> execute, bounded to four probes."""
    def __init__(self, T, output):
        self.output = output
        self.initial = np.asarray(T).copy()
        n = -self.initial[:3,2]
        lateral = self.initial[:3,1]
        self.directions = [unit(n), unit(n+.25*lateral), unit(n-.25*lateral), unit(lateral)]
        self.labels = ['outward', 'outward_plus_tangent', 'outward_minus_tangent', 'handle_lateral']
        self.poses = []
        self.attempts = []
        self.estimates = []
        self.estimate = None
        self.start_index = 0
        self.last_fit_count = 0
        self.angle_origin = None
        self.follow_sign = 1.
        self.save()

    def begin_attempt(self, index, t):
        self.start_index = len(self.poses)
        self.attempts.append({'index':index, 'direction_name':self.labels[index],
                              'direction_world':self.directions[index].tolist(), 'start_s':t,
                              'observation_start':self.start_index, 'result':'RUNNING'})
        self.save()
        return self.directions[index]

    def observe(self, T):
        self.poses.append(np.asarray(T).tolist())

    def travel(self, start=0):
        p = np.asarray(self.poses[start:])
        return 0. if len(p)<2 else float(np.max(np.linalg.norm(p[:,:3,3]-p[0,:3,3],axis=1)))

    def finish_attempt(self, t, reason):
        self.attempts[-1].update(end_s=t, measured_ee_travel_m=self.travel(self.start_index),
                                 observation_end=len(self.poses), result=reason)
        self.save()

    def try_fit(self):
        if len(self.poses)<12 or self.travel()<.005:
            return None
        fit = fit_articulation(self.poses)
        r = fit.get('revolute',{})
        accepted = (fit['joint_type']=='revolute' and fit['confidence']>.9
                    and r['position_rmse_m']<.0003 and r['rotation_rmse_rad']<.001
                    and r['angle_span_rad']>np.deg2rad(1.))
        record = {'fit':fit,'accepted':bool(accepted),'observation_count':len(self.poses)}
        self.estimates.append(record)
        if accepted:
            axis = np.asarray(r['axis'])
            rotations = Rotation.from_matrix(np.asarray(self.poses)[:,:3,:3]@self.initial[:3,:3].T).as_rotvec()
            self.follow_sign = 1. if (rotations[-1]-rotations[0])@axis>=0 else -1.
            self.estimate = fit
        self.save()
        return fit

    def tangent(self, T):
        if self.estimate is None:
            raise RuntimeError('NO_ESTIMATED_MODEL')
        r = self.estimate['revolute']
        return self.follow_sign*unit(np.cross(r['axis'],T[:3,3]-r['point_on_axis']))

    def consistency_error(self, T):
        if self.estimate is None:
            return None
        r=self.estimate['revolute'];axis=np.asarray(r['axis']);center=np.asarray(r['point_on_axis'])
        theta=float(Rotation.from_matrix(T[:3,:3]@self.initial[:3,:3].T).as_rotvec()@axis)
        predicted=center+Rotation.from_rotvec(theta*axis).apply(self.initial[:3,3]-center)
        return float(np.linalg.norm(T[:3,3]-predicted))

    def angle_deg(self, T):
        if self.estimate is None:
            return 0.
        v = Rotation.from_matrix(T[:3,:3]@self.initial[:3,:3].T).as_rotvec()
        return float(np.rad2deg(self.follow_sign*np.dot(v,self.estimate['revolute']['axis'])))

    def final_fit(self):
        if self.travel()<.005 or len(self.poses)<12:
            return {'joint_type':'UNOBSERVABLE','confidence':0.,'travel_m':self.travel(),
                    'sample_count':len(self.poses),'reason':'less than 5 mm useful measured EE travel'}
        return fit_articulation(self.poses)

    def save(self):
        data = {'source':'measured EE SE(3) only','estimated_articulation':self.estimate,
                'attempt_history':self.attempts,'fit_history':self.estimates,
                'supporting_observations':self.poses,'GT_inputs':False}
        path=self.output/'structured_memory.json'
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(data));temporary.replace(path)
        if self.estimate:
            r=self.estimate['revolute'];fmt=lambda v:' '.join(f'{x:.12g}' for x in v)
            (self.output/'estimated_articulation.urdf').write_text(
                '<robot name="EE_estimate"><link name="world_estimate"/><link name="moving_estimate"/>'
                '<joint name="estimated_joint" type="continuous"><parent link="world_estimate"/>'
                '<child link="moving_estimate"/><origin xyz="'+fmt(r['point_on_axis'])+'" rpy="0 0 0"/>'
                '<axis xyz="'+fmt(r['axis'])+'"/></joint></robot>')
