"""Asset geometry data adapter to frozen v2; ordering, thresholds and actions unchanged."""
import os,json,argparse,copy
from pathlib import Path
import numpy as np
import rclpy
import clutter_backend as cb
ROOT=Path(__file__).resolve().parents[1]
INVENTORY=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
TARGET=os.environ.get('FR3_ARENA_TARGET','mustard')
BOUNDS=np.array(INVENTORY[TARGET]['bounds']);SIZE=BOUNDS[1]-BOUNDS[0];CENTER=BOUNDS.mean(0)
# The pre-lift finger-width prediction previously used the fixed box dimensions.
# Substitute only this geometry input, retaining the original projection formula.
source=Path(cb.__file__).read_text();old="np.abs(grasp[:3,1]@T_B_O[:3,:3])*[.045,.045,.05]"
assert source.count(old)==1
exec(compile(source.replace(old,'np.abs(grasp[:3,1]@T_B_O[:3,:3])*ARENA_SIZE'),cb.__file__,'exec'),cb.__dict__)
cb.ARENA_SIZE=SIZE
from shape_msgs.msg import Mesh,MeshTriangle
from geometry_msgs.msg import Point
from moveit_msgs.srv import ApplyPlanningScene
meshdata=np.load(ROOT/f'assets/arena_complex/{TARGET}_mesh.npz')
mesh=Mesh();mesh.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in meshdata['vertices']]
mesh.triangles=[MeshTriangle(vertex_indices=[int(i) for i in f]) for f in meshdata['triangles']]
original_box=cb.collision_box
def collision_geometry(name,size,T,frame=cb.BASE):
    if name!='box':return original_box(name,size,T,frame)
    obj=cb.CollisionObject();obj.id=name;obj.header.frame_id=frame;obj.operation=obj.ADD
    obj.meshes=[mesh];obj.mesh_poses=[cb.pose(T)];return obj
cb.collision_box=collision_geometry
import hand_geometry
hand_geometry.SIZE=SIZE
original_geometry=hand_geometry.geometry
def geometry(H,O):
    T=np.eye(4);T[:3,3]=CENTER
    result=original_geometry(H,O@T);result['asset']=TARGET;result['GT_OBB_size_m']=SIZE.tolist()
    return result
import contact_backend
contact_backend.geometry=geometry
from workspace_mount import load_mount,transform_pose
from generalization_backend import HeldOut
class ArenaBackend(HeldOut):
    def reset_scene(self):
        super().reset_scene()
        b=np.array(INVENTORY['table']['bounds'])
        p,q=transform_pose([.5,0,-(b[1,2]-b[0,2])/2],[1,0,0,0],load_mount())
        T=cb.transform(p,q)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        req.scene.world.collision_objects=[original_box('table',b[1]-b[0],T)];self.apply(req)
    def episode(self,seed,mode='B'):
        r=super().episode(seed,mode)
        r['arena_asset']=INVENTORY[TARGET];r['target_class']=TARGET;r['geometry_adapter']=dict(size=SIZE.tolist(),center=CENTER.tolist(),target_mesh_triangles=len(mesh.triangles),target_mesh_vertices=len(mesh.vertices),attached_geometry='same GT triangle mesh in TCP frame',non_target='GT AABB + 1 mm each face',pad_metric='unchanged opposite OBB face projection')
        (self.output/f'B_seed_{seed:04d}.json').write_text(json.dumps(r,indent=2));return r
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seeds',required=True);a=p.parse_args()
    rclpy.init();n=ArenaBackend(a.output,geometry_filter=True,min_pad_coverage=.045,lift_gain_scale=2.)
    for seed in map(int,a.seeds.split(',')):
        if (n.output/f'B_seed_{seed:04d}.json').exists():continue
        result=n.episode(seed)
        if result['category']=='SYSTEM_ERROR':raise RuntimeError(result['detail'])
    n.destroy_node();rclpy.shutdown()
