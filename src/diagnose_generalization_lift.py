"""Post-batch saved-state PlanningScene audit of held-out seed110; no plant commands."""
import copy
import json
import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath,GetStateValidity
from clutter_backend import ClutterBackend,ROOT,BASE,TCP,GROUP,transform
from backend import pose
import plant
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from clutter_backend import collision_box
rclpy.init();n=ClutterBackend(ROOT/'results/generalization_lift110');n.mode='B';n.margin=.001
r=json.loads((ROOT/'results/generalization_clutter50/B_seed_0110.json').read_text())
req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
old=AttachedCollisionObject();old.link_name=TCP;old.object.id='box';old.object.operation=CollisionObject.REMOVE
req.scene.robot_state.attached_collision_objects=[old]
for ob in r['initial_layout']:
    T=np.eye(4);T[:3,3]=ob['position'];req.scene.world.collision_objects.append(collision_box(ob['id'],np.array(ob['aabb_size'])+.002,T))
n.apply(req)
snapshot=next(x['state'] for x in r['telemetry']['phase_starts'] if x['phase']=='LIFT')
H=transform(snapshot['tcp'],snapshot['tcp_quat']);relative=np.array(r['T_TCP_target']);n.attach(H,H@relative);n.support_contact(False)
actual=RobotState();actual.joint_state.name=snapshot['names'];actual.joint_state.position=snapshot['q'];actual=n.with_attachment(actual)
ideal=copy.deepcopy(actual);micro=r['executions'][-1]
for name,q in zip(micro['joint_names'],micro['points'][-1]['q']):ideal.joint_state.position[ideal.joint_state.name.index(name)]=q
nominal=np.array(r['telemetry']['TF']['T_B_TCP']);nominal[2,3]+=.1
vertical=H.copy();vertical[2,3]=nominal[2,3]
out=[]
cases=[('actual_to_nominal',actual,nominal),('actual_vertical',actual,vertical),('ideal_to_nominal',ideal,nominal)]
for fraction in [.25,.5,.75]:
    state=copy.deepcopy(actual);state.joint_state.position=(np.array(actual.joint_state.position)*(1-fraction)+np.array(ideal.joint_state.position)*fraction).tolist()
    cases.append((f'error_reduced_{fraction:.2f}',state,nominal))
for label,state,goal in cases:
    valid=GetStateValidity.Request();valid.robot_state=state;valid.group_name=GROUP;v=n.call('check_state_validity',valid)
    req=GetCartesianPath.Request();req.header.frame_id=BASE;req.group_name=GROUP;req.link_name=TCP;req.start_state=state;req.waypoints=[pose(goal)];req.max_step=.003;req.jump_threshold=1.5;req.avoid_collisions=True
    result=n.call('compute_cartesian_path',req)
    item=dict(case=label,start_valid=v.valid,start_contacts=[[c.contact_body_1,c.contact_body_2] for c in v.contacts],fraction=result.fraction,error_code=result.error_code.val,goal=goal.tolist());out.append(item);print(json.dumps(item),flush=True)
(n.output/'comparison.json').write_text(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
