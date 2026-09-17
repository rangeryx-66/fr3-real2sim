"""One GT-defined grasp per asset; MoveIt plans arm paths, no candidate selection."""
import os,sys,json,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
import numpy as np,rclpy
from scipy.spatial.transform import Rotation
from backend import Backend,pose,BASE,TCP
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import CollisionObject,AttachedCollisionObject,PlanningSceneComponents
from shape_msgs.msg import Mesh,MeshTriangle,SolidPrimitive
from geometry_msgs.msg import Point
import plant
plant.URL='http://127.0.0.1:'+os.environ.get('CALIBRATION_PORT','18765')
name=os.environ['FR3_ARENA_TARGET'];out=ROOT/'results/hand_calibration';out.mkdir(exist_ok=True,parents=True)
OFFSETS={'soup':[0,0,.03],'banana':[-.033,0,0.],'bowl':[.069,0,.017],'mug':[-.052,0,.025]}
R=np.diag([1.,-1.,-1.]) if name=='soup' else np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
meshdata=np.load(ROOT/f'assets/arena_complex/{name}_mesh.npz')
def x_hits(y,z):
    v=meshdata['vertices'][meshdata['triangles']];a=v[:,0,1:];u=v[:,1,1:]-a;w=v[:,2,1:]-a;q=np.array([y,z])-a
    den=u[:,0]*w[:,1]-u[:,1]*w[:,0];valid=abs(den)>1e-12;safe=np.where(valid,den,1.)
    b=(q[:,0]*w[:,1]-q[:,1]*w[:,0])/safe;c=(u[:,0]*q[:,1]-u[:,1]*q[:,0])/safe
    ok=valid&(b>=-1e-8)&(c>=-1e-8)&(b+c<=1+1e-8)
    xx=v[:,0,0]+b*(v[:,1,0]-v[:,0,0])+c*(v[:,2,0]-v[:,0,0])
    return np.unique(np.round(xx[ok],6))
geometry={}
if name!='soup':
    z=OFFSETS[name][2];hits=x_hits(0,z)
    pair=hits[-2:] if name=='bowl' else hits[:2] if name=='mug' else hits[[0,-1]]
    assert len(pair)==2 and 0<pair[1]-pair[0]<.08,(name,hits)
    OFFSETS[name][0]=float(pair.mean());geometry=dict(centerline_surface_x_m=pair.tolist(),all_surface_x_m=hits.tolist())
    if name in ['bowl','mug']:
        outer=lambda zz:float(x_hits(0,zz)[-1 if name=='bowl' else 0])
        slope=(outer(z+.002)-outer(z-.002))/.004
        yaxis=np.array([1.,0.,-slope]);yaxis/=np.linalg.norm(yaxis);xaxis=np.array([0.,1.,0.]);R=np.column_stack([xaxis,yaxis,np.cross(xaxis,yaxis)])
        geometry['wall_dx_dz']=slope
    else:
        center=lambda yy:float(x_hits(yy,z)[[0,-1]].mean())
        slope=(center(.005)-center(-.005))/.01
        xaxis=np.array([slope,1.,0.]);xaxis/=np.linalg.norm(xaxis);yaxis=np.array([1.,-slope,0.]);yaxis/=np.linalg.norm(yaxis);R=np.column_stack([xaxis,yaxis,np.cross(xaxis,yaxis)])
        geometry['centerline_dx_dy']=slope
def serialize(t):
 j=t.joint_trajectory
 return dict(names=list(j.joint_names),points=[dict(t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,q=list(p.positions)) for p in j.points])
def endpoint(start,t):
 s=copy.deepcopy(start)
 for n,q in zip(t.joint_trajectory.joint_names,t.joint_trajectory.points[-1].positions):s.joint_state.position[s.joint_state.name.index(n)]=q
 return s
rclpy.init();n=Backend()
plant.command(dict(op='reset'));plant.settle(1.)
initial=plant.state();O=np.eye(4);O[:3,:3]=Rotation.from_quat(np.roll(initial['box_quat'],-1)).as_matrix();O[:3,3]=initial['box']
H=np.eye(4);H[:3,:3]=O[:3,:3]@R;H[:3,3]=O[:3,3]+O[:3,:3]@OFFSETS[name]
pre=H.copy();pre[:3,3]-=H[:3,2]*.08;micro=H.copy();micro[2,3]+=.005;lift=H.copy();lift[2,3]+=.105
get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
s=n.call('get_planning_scene',get).scene
req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
for o in s.world.collision_objects:o.operation=CollisionObject.REMOVE;req.scene.world.collision_objects.append(o)
for a in s.robot_state.attached_collision_objects:a.object.operation=CollisionObject.REMOVE;req.scene.robot_state.attached_collision_objects.append(a)
acm=s.allowed_collision_matrix
if 'box' in acm.entry_names:
 i=acm.entry_names.index('box')
 for j in range(len(acm.entry_names)):acm.entry_values[i].enabled[j]=False;acm.entry_values[j].enabled[i]=False
req.scene.allowed_collision_matrix=acm
assert n.call('apply_planning_scene',req).success
req=ApplyPlanningScene.Request();req.scene.is_diff=True
table=CollisionObject();table.id='table';table.header.frame_id=BASE;table.operation=table.ADD
shape=SolidPrimitive();shape.type=shape.BOX;shape.dimensions=[1.,1.,.05];T=np.eye(4);T[:3,3]=[.5,0,-.025];table.primitives=[shape];table.primitive_poses=[pose(T)]
target=CollisionObject();target.id='box';target.header.frame_id=BASE;target.operation=target.ADD
d=np.load(ROOT/f'assets/arena_complex/{name}_mesh.npz');mesh=Mesh();mesh.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in d['vertices']];mesh.triangles=[MeshTriangle(vertex_indices=list(map(int,t))) for t in d['triangles']];target.meshes=[mesh];target.mesh_poses=[pose(O)]
req.scene.world.collision_objects=[table,target];assert n.call('apply_planning_scene',req).success
start=n.measured();gpre=n.ik(pre,start);approach=n.cartesian(gpre,H);g=endpoint(gpre,approach)
# The calibration lift replays only the arm path with independent live hand
# force control. World target removed only from the planner, never the physics.
target.operation=target.REMOVE;req.scene.world.collision_objects=[target];assert n.call('apply_planning_scene',req).success
mt=n.cartesian(g,micro);lt=n.cartesian(endpoint(g,mt),lift)
record=dict(target=name,source='manual GT mesh geometry; no AnyGrasp',GT_geometry=geometry,initial_target=initial['box'],T_B_target=O.tolist(),T_B_TCP=H.tolist(),target_local_offset=OFFSETS[name],pre_q=[gpre.joint_state.position[gpre.joint_state.name.index(f'fr3_joint{i}')] for i in range(1,8)],approach=serialize(approach),micro=serialize(mt),lift=serialize(lt),fk_check=n.check_fk())
(out/f'gt_path_{name}.json').write_text(json.dumps(record,indent=2));print('GT_PATH',name,H[:3,3],flush=True)
n.destroy_node();rclpy.shutdown()
