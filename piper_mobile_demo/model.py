"""Official geometry, blind local grasp manifold and explicit collision checks."""
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib
import time
import numpy as np
import trimesh
import fcl
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from articulated_demo.kinematics import URDFChain, transform


def origin(item):
    e=item.find('origin')
    return transform() if e is None else transform(np.fromstring(e.get('xyz','0 0 0'),sep=' '),np.fromstring(e.get('rpy','0 0 0'),sep=' '))


def geometry(item, directory):
    g=item.find('geometry');m=g.find('mesh')
    if m is not None:
        path=Path(m.get('filename'));path=path if path.is_absolute() else directory/path
        mesh=trimesh.load(path,force='mesh',process=True)
        mesh.apply_scale(np.fromstring(m.get('scale','1 1 1'),sep=' '))
    elif g.find('box') is not None:mesh=trimesh.creation.box(np.fromstring(g.find('box').get('size'),sep=' '))
    elif g.find('cylinder') is not None:
        c=g.find('cylinder');mesh=trimesh.creation.cylinder(float(c.get('radius')),float(c.get('length')),sections=48)
    elif g.find('sphere') is not None:mesh=trimesh.creation.icosphere(radius=float(g.find('sphere').get('radius')))
    else:raise ValueError('unsupported collision geometry')
    mesh.apply_transform(origin(item));return mesh


def fcl_geometry(mesh):
    out=fcl.BVHModel();out.beginModel(len(mesh.vertices),len(mesh.faces));out.addSubModel(np.asarray(mesh.vertices),np.asarray(mesh.faces,dtype=np.int32));out.endModel();return out


def obj(mesh,T):
    return fcl.CollisionObject(mesh,fcl.Transform(T[:3,:3],T[:3,3]))


class Model:
    def __init__(self,urdf,asset_root,source):
        self.urdf=Path(urdf);self.chain=URDFChain(urdf);self.names=[f'joint{i}' for i in range(1,7)]
        root=ET.parse(urdf).getroot();j={x.get('name'):x for x in root.findall('joint')}
        self.limits=np.array([[float(j[n].find('limit').get(k)) for k in ['lower','upper']] for n in self.names])
        self.home=np.array([0,1.15,-1.35,0,.2,0.]);self.robot=[];self.pads={};self.pad_triangles={};self.robot_hulls={}
        path=[];name='tcp_link'
        while name!=self.chain.root:
            joint=self.chain.joints[name];path.append(joint);name=joint.parent
        self.tip_chain=list(reversed(path))
        for link in root.findall('link'):
            for item in link.findall('collision'):
                mesh=geometry(item,self.urdf.parent);name=link.get('name')
                # Match Isaac's unscaled convexHull representation.
                hull=mesh.convex_hull;self.robot.append((name,fcl_geometry(hull)));self.robot_hulls[name]=hull
                if name in ['gripper_link1','gripper_link2']:
                    inner=float(mesh.vertices[:,2].min());faces=np.max(np.abs(mesh.triangles[:,:,2]-inner),axis=1)<1e-6
                    points=mesh.triangles[faces].reshape(-1,3)
                    if len(points)<3:raise ValueError('official distal flat inner surface not found')
                    self.pads[name]=np.array([points.min(0),points.max(0)])
                    self.pad_triangles[name]=mesh.triangles[faces]
        self.allowed=set()
        # Ignore only same rigid cluster and neighbouring connected links.
        graph={link.get('name'):set() for link in root.findall('link')}
        fixed=[]
        for x in j.values():
            a=x.find('parent').get('link');b=x.find('child').get('link');graph[a].add(b);graph[b].add(a)
            if x.get('type')=='fixed':fixed.append((a,b))
        cluster={n:{n} for n in graph}
        for _ in graph:
            for a,b in fixed:cluster[a]|=cluster[b];cluster[b]|=cluster[a]
        for a in graph:
            for b in graph:
                if cluster[a]&cluster[b] or any(graph[x]&cluster[b] for x in cluster[a]):self.allowed.add(frozenset((a,b)))
        self.allowed.add(frozenset(('gripper_link1','gripper_link2')))
        self.source=source;self.manifest=__import__('json').loads((Path(asset_root)/'manifest.json').read_text())
        self.asset_urdf=Path(asset_root)/'urdf'/f"{self.manifest['asset_id']}.urdf";self.asset=URDFChain(self.asset_urdf)
        placement=source['asset_installation'];s=self.manifest['scale_source_to_meters']
        R=Rotation.from_euler('z',placement['yaw_deg'],degrees=True).as_matrix()@np.array([[0,0,1],[1,0,0],[0,1,0]])
        self.asset_T=np.eye(4);self.asset_T[:3,:3]=R;self.asset_T[:3,3]=[placement['x_m'],placement['y_m'],-s*self.manifest['static_source_bounds'][0][1]+placement['fixture_height_m']]
        self.obstacles=[];self.obstacle_meshes={}
        for link in ET.parse(self.asset_urdf).getroot().findall('link'):
            for item in link.findall('collision'):
                mesh=geometry(item,self.asset_urdf.parent)
                shape=fcl_geometry(mesh);self.obstacles.append((link.get('name'),shape,None));self.obstacle_meshes[id(shape)]=mesh
        from articulated_demo.fixture_geometry import fixture_box
        fix=fixture_box(self.manifest,self.asset,self.asset_urdf,R,self.asset_T[:3,3],placement['fixture_height_m'])
        for name,size,T in [('table',[.9,.9,.05],transform([.5,0,-.025])),('fixture',fix['size'],transform(fix['center']))]:
            if name=='fixture':T[:3,:3]=Rotation.from_quat(np.roll(fix['quaternion_wxyz'],-1)).as_matrix()
            self.obstacles.append((name,fcl_geometry(trimesh.creation.box(size)),T))
        self.rng=np.random.default_rng(61)
        self.target_tree=None

    def fk(self,q):
        values=dict(zip(self.names,q));T=np.eye(4)
        for joint in self.tip_chain:
            T=T@joint.origin
            if joint.name in values:
                x,y,z=joint.axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                angle=values[joint.name];R=np.eye(4);R[:3,:3]=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K);T=T@R
        return T

    def poses(self,q,base,width=.1,finger_q=None):
        positions=dict(zip(self.names,q));positions.update(gripper=width,gripper_joint1=width/2,gripper_joint2=-width/2)
        if finger_q is not None:positions.update(gripper_joint1=float(finger_q[0]),gripper_joint2=float(finger_q[1]))
        B=transform(base[:3],[0,0,np.deg2rad(base[3])])
        return {n:B@self.chain.root_to_link(n,positions) for n,_ in self.robot} | {'tcp_link':B@self.chain.root_to_link('tcp_link',positions)}

    def margin(self,q):return float(np.min(np.minimum(q-self.limits[:,0],self.limits[:,1]-q)))

    def pad_contact(self,*args,**kwargs):
        raise RuntimeError('Point-projection contact classification is retired; use native collider ownership and validate_piper_owned_contact.py')

    def check(self,q,base,angle=0,width=.1,allow_pad=False,clearance=False,finger_q=None):
        if time.time()>getattr(self,'deadline_timestamp',float('inf')):raise TimeoutError('CUTOFF_05_00')
        if allow_pad:return False,'OWNED_CONTACT_VALIDATION_REQUIRED',None
        if self.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
        P=self.poses(q,base,width,finger_q);robots=[(n,obj(g,P[n])) for n,g in self.robot]
        for i,(a,A) in enumerate(robots):
            for b,B in robots[:i]:
                if frozenset((a,b)) not in self.allowed and fcl.collide(A,B)>0:return False,'SELF_COLLISION:'+a+':'+b,None
        obstacles=[];collision_surfaces={}
        joint=self.manifest['joint_name']
        for n,g,T in self.obstacles:
            W=T if T is not None else self.asset_T@self.asset.root_to_link(n,{joint:angle})
            body=obj(g,W);obstacles.append((n,body))
            if id(g) in self.obstacle_meshes:collision_surfaces[id(body)]=(self.obstacle_meshes[id(g)],W)
        # Platform must clear the environment too, not merely the arm.
        scene_obstacles=list(obstacles)
        for n,size,pos in [('mobile_base',[.34,.30,.20],[base[0],base[1],-.66]),('mast',[.10,.10,base[2]+.56],[base[0],base[1],(base[2]-.56)/2])]:
            T=transform(pos,[0,0,np.deg2rad(base[3])]);platform=obj(fcl_geometry(trimesh.creation.box(size)),T)
            for target,B in scene_obstacles:
                if fcl.collide(platform,B)>0:return False,'PLATFORM_COLLISION:'+n+':'+target,None
            obstacles.append((n,platform))
        minimum=float('inf')
        for n,A in robots:
            for target,B in obstacles:
                if target in ('mobile_base','mast') and n in ('base_link','link1'):continue
                result=fcl.CollisionResult();count=fcl.collide(A,B,fcl.CollisionRequest(num_max_contacts=64,enable_contact=True),result)
                if count:
                    return False,'COLLISION:'+n+':'+target,None
                if clearance:minimum=min(minimum,float(fcl.distance(A,B)))
        return True,'FREE',minimum if clearance else None

    def ik(self,target,base,seed=None,starts=5):
        lo=self.limits[:,0]+.050001;hi=self.limits[:,1]-.050001
        B=transform(base[:3],[0,0,np.deg2rad(base[3])]);T=np.linalg.inv(B)@target
        def residual(q):
            predicted=self.fk(q)
            return np.r_[predicted[:3,3]-T[:3,3],.1*Rotation.from_matrix(predicted[:3,:3]@T[:3,:3].T).as_rotvec()]
        seeds=[self.home if seed is None else seed]+list(self.rng.uniform(lo,hi,size=(starts-1,6)))
        for q0 in seeds:
            fit=least_squares(residual,np.clip(q0,lo,hi),bounds=(lo,hi),max_nfev=90,ftol=1e-7,gtol=1e-7,xtol=1e-7)
            if np.linalg.norm(fit.fun[:3])<.0008 and np.linalg.norm(fit.fun[3:])/.1<.004:return fit.x
        return None

    def door_target(self,T,angle):
        m=self.manifest['moving_link'];j=self.manifest['joint_name']
        D0=self.asset_T@self.asset.root_to_link(m,{j:0.});D=self.asset_T@self.asset.root_to_link(m,{j:angle})
        return D@np.linalg.inv(D0)@T

    def joint_plan(self,start,goal,base,iterations=1500):
        def edge(a,b):
            return all(self.check(q,base)[0] for q in np.linspace(a,b,max(2,int(np.ceil(np.max(np.abs(a-b))/.025))+1)))
        if not self.check(start,base)[0]:return None
        if edge(start,goal):return [start.tolist(),goal.tolist()]
        nodes=[np.asarray(start)];parents=[-1]
        lo=self.limits[:,0]+.050001;hi=self.limits[:,1]-.050001
        for _ in range(iterations):
            sample=np.asarray(goal) if self.rng.random()<.15 else self.rng.uniform(lo,hi)
            index=int(np.argmin([np.linalg.norm(q-sample) for q in nodes]));delta=sample-nodes[index]
            end=nodes[index]+delta*min(1.,.18/max(np.linalg.norm(delta),1e-12))
            if not edge(nodes[index],end):continue
            nodes.append(end);parents.append(index)
            if np.linalg.norm(end-goal)<.35 and edge(end,goal):
                path=[np.asarray(goal),end];index=len(nodes)-1
                while parents[index]>=0:index=parents[index];path.append(nodes[index])
                return [q.tolist() for q in reversed(path)]
        return None

    def fingerprint(self):
        return {'urdf_sha256':hashlib.sha256(self.urdf.read_bytes()).hexdigest(),'asset_urdf_sha256':hashlib.sha256(self.asset_urdf.read_bytes()).hexdigest(),'representation':'official robot convex hulls; unscaled original asset meshes','pad_bounds_link_frame':{n:b.tolist() for n,b in self.pads.items()},'margin_rad':.05}
