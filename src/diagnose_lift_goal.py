"""Post-batch, planning-only audit of seed 9's stopped lift."""
import copy
import json
import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath,GetStateValidity
from clutter_backend import ClutterBackend,ROOT,BASE,TCP,GROUP,transform
from backend import pose
import plant
rclpy.init();n=ClutterBackend(ROOT/'results/contact_lift_diagnostic');n.mode='B';n.margin=.001
plant.command(dict(op='clutter_enabled',enabled=True));plant.command(dict(op='reset',seed=9));plant.settle(1.);n.reset_scene()
r=json.loads((ROOT/'results/contact_repaired/B_seed_0009.json').read_text());snapshot=next(x['state'] for x in r['telemetry']['phase_starts'] if x['phase']=='LIFT')
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
(n.output/'comparison.json').write_text(json.dumps(out,indent=2));n.reset_scene();n.destroy_node();rclpy.shutdown()
