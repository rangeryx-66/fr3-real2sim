"""Read-only MoveIt rejection probes; sends no commands to the robot."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import RobotState
import plant
rclpy.init();n=Node('fr3_negative_validation')
ik=n.create_client(GetPositionIK,'/compute_ik');valid=n.create_client(GetStateValidity,'/check_state_validity')
assert ik.wait_for_service(timeout_sec=10) and valid.wait_for_service(timeout_sec=10)
def call(c,r):
    f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=10);assert f.done();return f.result()
s=plant.state();state=RobotState();state.joint_state.name=s['names'];state.joint_state.position=s['q']
req=GetPositionIK.Request();req.ik_request.group_name='fr3_arm';req.ik_request.ik_link_name='fr3_hand_tcp';req.ik_request.robot_state=state;req.ik_request.timeout.sec=1;req.ik_request.avoid_collisions=False
req.ik_request.pose_stamped.header.frame_id='fr3_link0';req.ik_request.pose_stamped.pose.position.x=5.;req.ik_request.pose_stamped.pose.orientation.w=1.
r=call(ik,req);assert r.error_code.val==-31,r.error_code.val
out={'unreachable_pose':{'code':r.error_code.val,'passed':True}}
# Reachable geometric pose penetrating the table: an IK solution may exist,
# but a collision-aware executor must reject that state.
req.ik_request.pose_stamped.pose.position.x=.5
req.ik_request.pose_stamped.pose.position.z=-.02
req.ik_request.pose_stamped.pose.orientation.w=0.
req.ik_request.pose_stamped.pose.orientation.x=1.
r=call(ik,req);assert r.error_code.val==1,r.error_code.val
v=GetStateValidity.Request();v.group_name='fr3_arm';v.robot_state=r.solution
r=call(valid,v);pairs=[(c.contact_body_1,c.contact_body_2) for c in r.contacts]
assert not r.valid and any('table' in pair for pair in pairs),pairs
out['table_penetration']={'valid':r.valid,'contacts':pairs,'passed':True}
print(json.dumps(out,indent=2));Path(__file__).resolve().parents[1].joinpath('results/negative_checks.json').write_text(json.dumps(out,indent=2))
n.destroy_node();rclpy.shutdown()
