"""Read-only controlled counterexample: robot-only path clear, carried box sweep blocked."""
import copy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from clutter_backend import ClutterBackend, collision_box, transform, ROOT, BASE, TCP, GROUP
from backend import pose
from moveit_msgs.srv import GetPositionFK, GetStateValidity, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
import plant
rclpy.init();n=ClutterBackend(ROOT/'results/clutter_attachment_probe')
plant.command(dict(op='reset',seed=1000));plant.settle(1.);n.mode='A';n.reset_scene()
trial=json.loads((ROOT/'results/run_1788943864166403373/trial_01.json').read_text())
s=n.measured()
for name,q in zip(trial['executions'][1]['joint_names'],trial['executions'][1]['points'][-1]['q']):s.joint_state.position[s.joint_state.name.index(name)]=q
for name in ['fr3_finger_joint1','fr3_finger_joint2']:s.joint_state.position[s.joint_state.name.index(name)]=.0225
req=GetPositionFK.Request();req.header.frame_id=BASE;req.fk_link_names=[TCP];req.robot_state=s
p=n.call('compute_fk',req).pose_stamped[0].pose
T=np.eye(4);T[:3,3]=[p.position.x,p.position.y,p.position.z];T[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
O=np.eye(4);O[:3,3]=[.5,0,.025]
n.attach(T,O);n.support_contact(True)
# A 6 mm planning-only diagnostic cube lies inside the target's vertical sweep,
# below the finger tips when the carried box passes it. It is NOT an A/B obstacle.
probe=np.eye(4);probe[:3,3]=[.5,0,.070]
a=ApplyPlanningScene.Request();a.scene.is_diff=True;a.scene.world.collision_objects=[collision_box('clutter_probe',[.006,.006,.006],probe)];n.apply(a)
lift=T.copy();lift[2,3]+=.1

def path(attached):
    req=GetCartesianPath.Request();req.header.frame_id=BASE;req.group_name=GROUP;req.link_name=TCP;req.max_step=.003;req.jump_threshold=1.5;req.avoid_collisions=True;req.waypoints=[pose(lift)]
    req.start_state=copy.deepcopy(s)
    req.start_state.is_diff=attached
    req.start_state.attached_collision_objects=[copy.deepcopy(n.attached)] if attached else []
    return n.call('compute_cartesian_path',req)
without=path(False);with_target=path(True)
assert without.fraction>.999,without.fraction
assert with_target.fraction<.999,with_target.fraction
pairs=[];endpoint=[]
points=without.solution.joint_trajectory.points
for index,point in enumerate(points):
    state=copy.deepcopy(s);state.is_diff=True;state.attached_collision_objects=[copy.deepcopy(n.attached)]
    for name,q in zip(without.solution.joint_trajectory.joint_names,point.positions):state.joint_state.position[state.joint_state.name.index(name)]=q
    req=GetStateValidity.Request();req.robot_state=state;req.group_name=GROUP
    response=n.call('check_state_validity',req)
    contacts=[(c.contact_body_1,c.contact_body_2) for c in response.contacts]
    if index in [0,len(points)-1]:endpoint.append(bool(response.valid))
    if any('box' in pair and 'clutter_probe' in pair for pair in contacts):pairs.append(dict(sample=index,contacts=contacts))
assert pairs,pairs
assert all(endpoint),endpoint
out=dict(passed=True,robot_only_fraction=without.fraction,attached_target_fraction=with_target.fraction,attached_endpoints_valid=endpoint,interior_target_collisions=pairs,probe_center=probe[:3,3].tolist(),probe_size=[.006]*3,scope='Planning-only diagnostic; no robot motion; excluded from paired episodes',robot_state_is_diff_required=True)
(ROOT/'results/clutter_attachment_probe/proof.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
a=ApplyPlanningScene.Request();a.scene.is_diff=True;remove=CollisionObject();remove.id='clutter_probe';remove.operation=CollisionObject.REMOVE;a.scene.world.collision_objects=[remove];n.apply(a);n.reset_scene()
n.destroy_node();rclpy.shutdown()
