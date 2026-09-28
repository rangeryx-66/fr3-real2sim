"""Standard ROS actions -> Isaac physics plant; publishes measured joint states."""
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rosgraph_msgs.msg import Clock
import r1a7_plant as plant
class Bridge(Node):
    def __init__(self):
        super().__init__('isaac_r1a7_controller')
        self.group=ReentrantCallbackGroup();self.busy=threading.Lock()
        self.pub=self.create_publisher(JointState,'/joint_states',10)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.publish_group=MutuallyExclusiveCallbackGroup()
        self.create_timer(.02,self.publish,callback_group=self.publish_group)
        self.arm=ActionServer(self,FollowJointTrajectory,'/r1a7_arm_controller/follow_joint_trajectory',execute_callback=self.execute,goal_callback=self.goal,cancel_callback=self.cancel,callback_group=self.group)
        self.hand=ActionServer(self,GripperCommand,'/dex1_gripper/gripper_action',execute_callback=self.gripper,goal_callback=self.goal,cancel_callback=self.cancel,callback_group=self.group)
    def goal(self,request):return GoalResponse.ACCEPT if not self.busy.locked() else GoalResponse.REJECT
    def cancel(self,handle):
        plant.submit({'op':'stop'});return CancelResponse.ACCEPT
    def publish(self):
        try:s=plant.state('joints')
        except Exception:return
        if not all(k in s for k in ['t','names','q']):return
        c=Clock();c.clock.sec=int(s['t']);c.clock.nanosec=int((s['t']%1)*1e9);self.clock.publish(c)
        m=JointState();m.header.stamp=c.clock;m.name=s['names'];m.position=s['q'];self.pub.publish(m)
    def execute(self,h):
        result=FollowJointTrajectory.Result()
        with self.busy:
            try:
                t=h.request.trajectory
                points=[dict(t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,q=list(p.positions)) for p in t.points]
                token=plant.submit(dict(op='trajectory',names=list(t.joint_names),points=points))
                deadline=time.monotonic()+max(120,points[-1]['t']*10)
                while time.monotonic()<deadline:
                    s=plant.state('control')
                    if h.is_cancel_requested:
                        plant.command({'op':'stop'});h.canceled();result.error_code=-4;return result
                    if token in s['results']:
                        r=s['results'][token]
                        if not r['ok']:raise RuntimeError(str(r))
                        h.succeed();result.error_code=0;return result
                    time.sleep(.03)
                raise TimeoutError('physical trajectory timeout')
            except Exception as e:
                plant.submit({'op':'stop'});h.abort();result.error_code=-4;result.error_string=str(e);return result
    def gripper(self,h):
        result=GripperCommand.Result()
        with self.busy:
            try:
                opening=max(.001,min(.09,h.request.command.position))
                # Official Dex1 prismatic axes close symmetrically as q rises.
                joint_q=(.05-opening)/2.
                r=plant.command(dict(op='trajectory',names=['dex1_Joint1_1','dex1_Joint2_1'],points=[dict(t=1.,q=[joint_q,joint_q])],gripper=True))
                s=plant.state();measured=s['q'][s['names'].index('dex1_Joint1_1')]
                result.position=.05-2*measured
                result.reached_goal=abs(result.position-opening)<.003
                result.stalled=not result.reached_goal
                if not r['ok']:raise RuntimeError(str(r))
                h.succeed()
            except Exception:h.abort()
        return result
rclpy.init();node=Bridge();ex=MultiThreadedExecutor(num_threads=4);ex.add_node(node)
try:ex.spin()
finally:node.destroy_node();rclpy.shutdown()
