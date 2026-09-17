"""Optional three-body GT clutter and per-physics-step disturbance measurements."""
import numpy as np
from scipy.spatial.transform import Rotation

SIZES = np.array([[.035, .040, .060], [.035, .035, .135], [.030, .040, .055]])

def layout(seed):
    rng = np.random.default_rng(int(seed))
    angles = np.deg2rad([60., 180., 300.]) + rng.uniform(-.10, .10, 3)
    radii = np.array([.085, .085, .095]) + rng.uniform(-.008, .008, 3)
    centers = np.column_stack([.5 + radii*np.cos(angles), radii*np.sin(angles), SIZES[:,2]/2])
    yaws = rng.uniform(-.15, .15, 3)
    quats = np.column_stack([np.cos(yaws/2), np.zeros(3), np.zeros(3), np.sin(yaws/2)])
    return centers, quats

class ClutterMonitor:
    def __init__(self, world, stage, material):
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.prims import RigidPrim
        from pxr import UsdPhysics
        self.bodies=[];self.target=world.scene.get_object("box")
        positions, quats=layout(0)
        for i in range(3):
            b=world.scene.add(DynamicCuboid(f'/World/clutter_{i}',name=f'clutter_{i}',position=positions[i],orientation=quats[i],scale=SIZES[i],mass=.10,physics_material=material,color=np.array([[.12,.35,.8],[.2,.65,.35],[.65,.3,.65]][i])))
            self.bodies.append(b)
        self.filters=[str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith('/World/FR3/') and p.HasAPI(UsdPhysics.RigidBodyAPI)]+['/World/box']
        self.contacts=world.scene.add(RigidPrim(prim_paths_expr='/World/clutter_.*',name='clutter_contacts',contact_filter_prim_paths_expr=self.filters,track_contact_forces=True,prepare_contact_sensors=True))
        self.armed=False;self.seed=0;self.phase='IDLE';self.metrics={};self.events=[]
    def reset(self, seed):
        self.armed=False;self.seed=int(seed);self.phase='SETTLE'
        pos,q=layout(seed)
        for i,b in enumerate(self.bodies):
            b.set_world_pose(pos[i],q[i]);b.set_linear_velocity(np.zeros(3));b.set_angular_velocity(np.zeros(3))
        self.events=[];self.metrics={};self.last_contacts=set()
    def arm(self, now):
        p,q=self.contacts.get_world_poses();self.initial=list(zip(p.copy(),q.copy()))
        self.start=now;self.armed=True;self.phase='PLAN';self.initial_target_z=float(self.target.get_world_pose()[0][2])
        self.metrics=dict(any_contact=False,robot_contact=False,target_contact=False,max_displacement_m=0.,max_rotation_rad=0.,peak_contact_N=0.,contact_steps=0,max_target_lift_m=0.,per_object=[dict(id=f'clutter_{i}',max_displacement_m=0.,max_rotation_rad=0.,robot_peak_N=0.,target_peak_N=0.) for i in range(3)])
    def sample(self, now, target_position):
        positions,quaternions=self.contacts.get_world_poses()
        poses=list(zip(positions,quaternions))
        obstacles=[]
        for i,(p,q) in enumerate(poses):
            R=Rotation.from_quat(np.roll(q,-1)).as_matrix()
            obstacles.append(dict(id=f'clutter_{i}',position=p.tolist(),quaternion_wxyz=q.tolist(),size=SIZES[i].tolist(),aabb_size=(np.abs(R)@SIZES[i]).tolist()))
        if self.armed:
            self.metrics['max_target_lift_m']=max(self.metrics['max_target_lift_m'],float(target_position[2])-self.initial_target_z)
            forces=np.linalg.norm(np.asarray(self.contacts.get_contact_force_matrix(dt=1/240)),axis=-1)
            assert forces.shape==(3,len(self.filters)), forces.shape
            active=set()
            for i,(p,q) in enumerate(poses):
                initial_p,initial_q=self.initial[i]
                displacement=float(np.linalg.norm(p-initial_p))
                rotation=float(2*np.arccos(np.clip(abs(np.dot(q/np.linalg.norm(q),initial_q/np.linalg.norm(initial_q))),0,1)))
                item=self.metrics['per_object'][i]
                robot_peak=float(forces[i,:-1].max());target_peak=float(forces[i,-1])
                item['max_displacement_m']=max(item['max_displacement_m'],displacement)
                item['max_rotation_rad']=max(item['max_rotation_rad'],rotation)
                item['robot_peak_N']=max(item['robot_peak_N'],robot_peak)
                item['target_peak_N']=max(item['target_peak_N'],target_peak)
                self.metrics['max_displacement_m']=max(self.metrics['max_displacement_m'],displacement)
                self.metrics['max_rotation_rad']=max(self.metrics['max_rotation_rad'],rotation)
                self.metrics['peak_contact_N']=max(self.metrics['peak_contact_N'],robot_peak,target_peak)
                for j in np.where(forces[i]>.05)[0]:
                    key=(i,int(j));active.add(key)
                    if key not in self.last_contacts and len(self.events)<300:
                        self.events.append(dict(t=now-self.start,phase=self.phase,obstacle=f'clutter_{i}',other=self.filters[j],force_N=float(forces[i,j])))
                self.metrics['robot_contact']|=robot_peak>.05
                self.metrics['target_contact']|=target_peak>.05
            if active:self.metrics['contact_steps']+=1
            self.metrics['any_contact']|=bool(active)
            self.last_contacts=active
        return dict(seed=self.seed,phase=self.phase,obstacles=obstacles,metrics=self.metrics,events=self.events,armed=self.armed)
