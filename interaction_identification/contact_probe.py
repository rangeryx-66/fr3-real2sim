"""Real-contact probe inputs: robot state, tactile loads, calibrated RGB-D.

No object URDF, simulator object pose, articulation state, or GT axis inputs.
"""
import inspect
import textwrap
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


class GroundTruthGate:
    """Scene assembly is finished; GT handles cannot be queried until evaluation."""
    def __init__(self):
        self.open = False
        self.accesses = []

    def protect(self, scene):
        gate = self
        class Protected:
            def __init__(self, name, value):
                self.name, self.value = name, value
            def __getattr__(self, attribute):
                if not gate.open:
                    raise RuntimeError('FORBIDDEN_RUNTIME_GT_ACCESS:'+self.name+'.'+attribute)
                gate.accesses.append(self.name+'.'+attribute)
                return getattr(self.value, attribute)
        for name in ('articulation','door_contacts','door_link','asset_chain'):
            scene[name] = Protected(name, scene[name])


def robot_only_model(urdf):
    # Reuse the frozen robot initialization and identical IK implementation,
    # stopping before the asset URDF/GT parser. No model.asset is constructed.
    import piper_mobile_demo.model as module
    code = inspect.getsource(module.Model.__init__)
    marker = '        self.source=source;'
    if code.count(marker) != 1:
        raise RuntimeError('FROZEN_ROBOT_INITIALIZER_CHANGED')
    namespace = {}
    exec(textwrap.dedent(code.split(marker)[0]), module.__dict__, namespace)
    model = module.Model.__new__(module.Model)
    namespace['__init__'](model, urdf, None, None)
    model.rng = np.random.default_rng(61)
    return model


class DepthMotion:
    """Marker-free local RGB-D registration, with q-based robot occlusion mask.

    The initial grasp provides only a target-region seed. ICP estimates a free
    rigid transform; it has no joint family, axis, radius, or hinge constraint.
    """
    def __init__(self, camera, camera_world, K, seed, robot_model):
        self.camera = camera
        self.camera_world = camera_world
        self.K = K
        self.seed = np.asarray(seed).copy()
        self.model = robot_model
        self.reference = None
        self.delta = np.eye(4)
        self.status = {'ready': False}
        self.frames = []
        self.optical = None

    def backproject(self, pixels, depth):
        # NumPy bilinear sampling also supports a full-resolution ROI (>32767
        # pixels), unlike OpenCV remap's signed-short destination dimensions.
        depth=np.asarray(depth,dtype=np.float32);h,w=depth.shape
        u=np.clip(pixels[:,0],0,w-1);v=np.clip(pixels[:,1],0,h-1)
        x=np.floor(u).astype(int);y=np.floor(v).astype(int)
        x1=np.minimum(x+1,w-1);y1=np.minimum(y+1,h-1)
        du=u-x;dv=v-y
        z=(1-dv)*((1-du)*depth[y,x]+du*depth[y,x1])+dv*((1-du)*depth[y1,x]+du*depth[y1,x1])
        outside=(pixels[:,0]<0)|(pixels[:,0]>w-1)|(pixels[:,1]<0)|(pixels[:,1]>h-1)
        z[outside]=np.nan
        xyz=np.column_stack(((pixels[:,0]-self.K[0,2])*z/self.K[0,0],(pixels[:,1]-self.K[1,2])*z/self.K[1,1],z))
        return xyz@self.camera_world[:3,:3].T+self.camera_world[:3,3],z

    def visible(self, xyz, poses):
        keep=np.ones(len(xyz),dtype=bool)
        for body,hull in self.model.robot_hulls.items():
            if body not in poses:continue
            T=np.asarray(poses[body]);p=(xyz-T[:3,3])@T[:3,:3]
            keep &= ~np.all((p>=hull.bounds[0]-.001)&(p<=hull.bounds[1]+.001),axis=1)
        return keep

    def initialize_grasp(self, poses, time_s):
        """Fix RGB-D correspondences after approach occlusion, before probing."""
        gray=cv2.cvtColor(np.asarray(self.camera.get_rgba())[:,:,:3],cv2.COLOR_RGB2GRAY)
        depth=np.asarray(self.camera.get_depth());v,u=np.mgrid[:depth.shape[0],:depth.shape[1]]
        pixels=np.column_stack((u.ravel(),v.ravel())).astype(np.float32)
        xyz,z=self.backproject(pixels,depth)
        seed=self.delta@self.seed;local=(xyz-seed[:3,3])@seed[:3,:3]
        mask=(np.abs(local[:,0])<.125)&(np.abs(local[:,1])<.16)&(local[:,2]>-.018)&(local[:,2]<.055)&np.isfinite(z)&(z>.02)&self.visible(xyz,poses)
        mask=(mask.reshape(depth.shape).astype(np.uint8)*255)
        corners=cv2.goodFeaturesToTrack(gray,maxCorners=1000,qualityLevel=.001,minDistance=4,mask=mask)
        if corners is None or len(corners)<25:raise RuntimeError('UNOBSERVABLE_RGBD_FEATURES')
        pixels=corners.reshape(-1,2);points,z=self.backproject(pixels,depth)
        self.optical={'gray':gray,'pixels':pixels,'points':points,'origin_delta':self.delta.copy()}
        self.status={'ready':True,'valid':True,'point_count':len(points),'rmse_m':0.,'inlier_fraction':1.,'method':'RGB-D fixed optical correspondences, free SE3'}
        self.frames.append({'t':time_s,'delta':self.delta.tolist(),**self.status})

    def update_optical(self, poses, time_s):
        gray=cv2.cvtColor(np.asarray(self.camera.get_rgba())[:,:,:3],cv2.COLOR_RGB2GRAY)
        prior=self.optical['pixels'].astype(np.float32).reshape(-1,1,2)
        nxt,ok,_=cv2.calcOpticalFlowPyrLK(self.optical['gray'],gray,prior,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.005))
        back,bok,_=cv2.calcOpticalFlowPyrLK(gray,self.optical['gray'],nxt,None,winSize=(21,21),maxLevel=3)
        pixels=nxt.reshape(-1,2);points,z=self.backproject(pixels,self.camera.get_depth())
        good=ok.ravel().astype(bool)&bok.ravel().astype(bool)&(np.linalg.norm(back.reshape(-1,2)-prior.reshape(-1,2),axis=1)<.5)&np.isfinite(z)&(z>.02)&(z<2.)&self.visible(points,poses)
        a=self.optical['points'][good];b=points[good];pixels=pixels[good]
        if len(a)<25:self.status.update(valid=False,reason='RGB-D feature loss');return
        # Track initial RGB-D 3D points by their image projections. Sampling
        # changing depth discontinuities at silhouette corners otherwise feeds
        # sub-mm depth errors directly into a very small rotation estimate.
        prior_T=np.linalg.inv(self.camera_world)@self.delta@np.linalg.inv(self.optical['origin_delta'])
        rvec=cv2.Rodrigues(prior_T[:3,:3])[0];tvec=prior_T[:3,3].reshape(3,1)
        ok,rvec,tvec,inliers=cv2.solvePnPRansac(a.astype(np.float64),pixels.astype(np.float64),self.K,None,rvec,tvec,True,iterationsCount=100,reprojectionError=.75,confidence=.99,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None or len(inliers)<25:self.status.update(valid=False,reason='RGB-D PnP correspondence loss');return
        idx=inliers.ravel();rvec,tvec=cv2.solvePnPRefineLM(a[idx].astype(np.float64),pixels[idx].astype(np.float64),self.K,None,rvec,tvec)
        camera_T=np.eye(4);camera_T[:3,:3]=cv2.Rodrigues(rvec)[0];camera_T[:3,3]=tvec.ravel();T=self.camera_world@camera_T
        projected=cv2.projectPoints(a[idx].astype(np.float64),rvec,tvec,self.K,None)[0].reshape(-1,2)
        pixel_rmse=float(np.sqrt(np.mean(np.sum((projected-pixels[idx])**2,axis=1))))
        error=np.linalg.norm(a@T[:3,:3].T+T[:3,3]-b,axis=1);trim=np.sort(error[idx])[:max(1,int(.85*len(idx)))];rmse=float(np.sqrt(np.mean(trim**2)))
        valid=len(idx)/len(a)>.5 and pixel_rmse<.5 and rmse<.001
        self.status={'ready':True,'valid':bool(valid),'point_count':len(idx),'rmse_m':rmse,'reprojection_rmse_px':pixel_rmse,'inlier_fraction':len(idx)/len(a),'method':'initial RGB-D points + optical PnP, free SE3'}
        if valid:
            self.delta=T@self.optical['origin_delta'];self.optical['gray']=gray;self.optical['pixels']=pixels;self.optical['points']=a
        self.frames.append({'t':time_s,'delta':self.delta.tolist(),**self.status})

    def points(self, poses):
        depth = self.camera.get_depth()
        if depth is None or np.asarray(depth).ndim != 2:
            return None
        depth = np.asarray(depth)
        v,u = np.mgrid[0:depth.shape[0]:3, 0:depth.shape[1]:3]
        z = depth[v,u]
        good = np.isfinite(z) & (z > .02) & (z < 2.)
        z,u,v = z[good],u[good],v[good]
        xyz = np.column_stack(((u-self.K[0,2])*z/self.K[0,0],
                               (v-self.K[1,2])*z/self.K[1,1],z))
        xyz = xyz @ self.camera_world[:3,:3].T + self.camera_world[:3,3]
        current_seed = self.delta @ self.seed
        local = (xyz-current_seed[:3,3]) @ current_seed[:3,:3]
        # Generic nearby handle/panel patch; no simulator segmentation IDs.
        keep = (np.abs(local[:,0]) < .125) & (np.abs(local[:,1]) < .16)
        keep &= (local[:,2] > -.018) & (local[:,2] < .055)
        xyz = xyz[keep]
        for body, hull in self.model.robot_hulls.items():
            if body not in poses:
                continue
            T = np.asarray(poses[body])
            p = (xyz-T[:3,3]) @ T[:3,:3]
            bounds = hull.bounds
            # Sensor visibility mask only, never collision geometry resizing.
            inside = np.all((p >= bounds[0]-.001) & (p <= bounds[1]+.001),axis=1)
            xyz = xyz[~inside]
        return xyz

    def update(self, poses, time_s):
        if self.optical is not None:
            self.update_optical(poses,time_s)
            return
        cloud = self.points(poses)
        if cloud is None or len(cloud) < 150:
            self.status = {'ready': self.reference is not None, 'valid': False,
                           'reason': 'insufficient RGB-D target points',
                           'point_count': 0 if cloud is None else len(cloud)}
            return
        if self.reference is None:
            # Bound correspondence query cost; targets retain full depth density.
            self.reference = cloud[::max(1,len(cloud)//2500)].copy()
            self.status = {'ready': True, 'valid': True, 'point_count': len(cloud),
                           'rmse_m': 0., 'inlier_fraction': 1.}
            self.frames.append({'t':time_s,'delta':self.delta.tolist(),**self.status})
            return
        tree = cKDTree(cloud)
        estimate = self.delta.copy()
        for _ in range(12):
            predicted = self.reference @ estimate[:3,:3].T + estimate[:3,3]
            distance,index = tree.query(predicted)
            good = distance < .004
            if good.sum() < 120:
                break
            cut = np.quantile(distance[good],.85)
            good &= distance <= max(cut,.00015)
            a,b = predicted[good],cloud[index[good]]
            ac,bc = a.mean(0),b.mean(0)
            U,_,V = np.linalg.svd((a-ac).T @ (b-bc))
            R = V.T @ U.T
            if np.linalg.det(R)<0:
                V[-1]*=-1;R=V.T @ U.T
            step = np.eye(4);step[:3,:3]=R;step[:3,3]=bc-R@ac
            estimate = step @ estimate
            if np.linalg.norm(step[:3,3])<1e-7 and Rotation.from_matrix(R).magnitude()<1e-6:
                break
        predicted = self.reference @ estimate[:3,:3].T + estimate[:3,3]
        distance,_ = tree.query(predicted)
        good = distance < .004
        rmse = float(np.sqrt(np.mean(distance[good]**2))) if good.any() else float('inf')
        valid = good.mean() > .45 and rmse < .001
        self.status = {'ready':True,'valid':bool(valid),'point_count':len(cloud),
                       'rmse_m':rmse,'inlier_fraction':float(good.mean())}
        if valid:
            self.delta = estimate
        self.frames.append({'t':time_s,'delta':self.delta.tolist(),**self.status})


class ObservedCompliantPull:
    """Bounded Cartesian lead following observed handle motion, not a line/arc.

    Force-limited speed and a 0.3 mm virtual lead allow the object to choose its
    motion. Rotation follows RGB-D observations. Estimated tangent replaces the
    initial pull direction only after the EE-only articulation fit is accepted.
    """
    def __init__(self, ee, observed_delta, policy):
        self.origin = np.asarray(ee).copy()
        self.reference_delta = np.asarray(observed_delta).copy()
        self.policy = policy
        self.force_limit = policy['max_pad_load_n']
        self.estimate = None
        self.command = self.origin.copy()
        self.angle_sign = 1.
        self.filtered_rotation = self.origin[:3,:3].copy()
        self.center_offset = np.zeros(3)

    def set_estimate(self, fit, current_ee):
        self.estimate = fit['revolute']
        axis = np.asarray(self.estimate['axis'])
        rotation = Rotation.from_matrix(current_ee[:3,:3] @ self.origin[:3,:3].T).as_rotvec()
        self.angle_sign = 1. if axis @ rotation >= 0 else -1.

    def angle_deg(self, ee):
        if self.estimate is None:
            return None
        rotation = Rotation.from_matrix(ee[:3,:3] @ self.origin[:3,:3].T).as_rotvec()
        return float(np.rad2deg(self.angle_sign*np.dot(self.estimate['axis'],rotation)))

    def update(self, ee, observed_delta, forces, pad_centers, dt):
        follow = observed_delta @ np.linalg.inv(self.reference_delta) @ self.origin
        # Free RGB-D orientation contains depth-edge noise. A short temporal
        # filter is joint-agnostic; it constrains neither axis nor joint family.
        dr=Rotation.from_matrix(follow[:3,:3]@self.filtered_rotation.T).as_rotvec()
        self.filtered_rotation=Rotation.from_rotvec(dr*dt/(.25+dt)).as_matrix()@self.filtered_rotation
        follow[:3,:3]=self.filtered_rotation
        direction = -follow[:3,2]
        if self.estimate is not None:
            radial = ee[:3,3]-np.asarray(self.estimate['point_on_axis'])
            direction = self.angle_sign*np.cross(self.estimate['axis'],radial)
        direction /= np.linalg.norm(direction)
        speed = .0005 * np.clip((self.force_limit-max(forces))/(self.force_limit-.5),.1,1.)
        desired = follow[:3,3] + self.center_offset + .0003*direction
        displacement = desired-ee[:3,3]
        n = np.linalg.norm(displacement)
        if n > speed*dt:
            displacement *= speed*dt/n
        # Same geometric jaw direction and centering bounds as the frozen
        # sensor-only controllers; only post-grasp interaction uses this term.
        centers=np.asarray(pad_centers);jaw=centers[0]-centers[1];jaw/=np.linalg.norm(jaw)
        balance_speed=np.clip(.003*(forces[0]-forces[1]),-.001,.001)
        delta=jaw*np.clip(balance_speed*dt,-self.policy['max_centering_step_m'],self.policy['max_centering_step_m'])
        previous=self.center_offset.copy();self.center_offset+=delta
        length=np.linalg.norm(self.center_offset);bound=self.policy['max_centering_displacement_m']
        if length>bound:self.center_offset*=bound/length
        displacement+=self.center_offset-previous
        n=np.linalg.norm(displacement)
        if n>speed*dt:displacement*=speed*dt/n
        rotation = Rotation.from_matrix(follow[:3,:3] @ ee[:3,:3].T).as_rotvec()
        r = np.linalg.norm(rotation)
        if r > .02*dt:
            rotation *= .02*dt/r
        self.command = ee.copy()
        self.command[:3,3] += displacement
        self.command[:3,:3] = Rotation.from_rotvec(rotation).as_matrix() @ ee[:3,:3]
        return self.command.copy()


def high_quality_revolute(fit):
    if fit.get('joint_type') != 'revolute':
        return False
    m = fit['revolute']
    return (fit['confidence']>.9 and fit['travel_m']>.005 and
            m['angle_span_rad']>np.deg2rad(1.) and
            m['position_rmse_m']<.0003 and m['rotation_rmse_rad']<.001)
