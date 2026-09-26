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
import plant
from robot_profile import get_profile,gripper_positions
PROFILE=get_profile()
class Bridge(Node):
    def __init__(self):
        super().__init__(f'isaac_{PROFILE.name}_controller')
        self.group=ReentrantCallbackGroup();self.busy=threading.Lock()
        self.pub=self.create_publisher(JointState,'/joint_states',10)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.publish_group=MutuallyExclusiveCallbackGroup()
        self.create_timer(.02,self.publish,callback_group=self.publish_group)
        self.arm=ActionServer(self,FollowJointTrajectory,PROFILE.arm_action,execute_callback=self.execute,goal_callback=self.goal,cancel_callback=self.cancel,callback_group=self.group)
        self.hand=ActionServer(self,GripperCommand,PROFILE.gripper_action,execute_callback=self.gripper,goal_callback=self.goal,cancel_callback=self.cancel,callback_group=self.group)
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
                width=max(0.,min(PROFILE.open_width_m,h.request.command.position*(2 if PROFILE.name=='fr3' else 1)))
                desired=gripper_positions(PROFILE,width);s=plant.state();desired={n:q for n,q in desired.items() if n in s['names']}
                r=plant.command(dict(op='trajectory',names=list(desired),points=[dict(t=1.,q=list(desired.values()))],gripper=True))
                s=plant.state();actual=float(s['q'][s['names'].index(PROFILE.command_gripper_joint)])
                result.position=actual
                goal=width/2 if PROFILE.name=='fr3' else width
                result.reached_goal=abs(actual-goal)<.002
                result.stalled=not result.reached_goal
                if not r['ok']:raise RuntimeError(str(r))
                h.succeed()
            except Exception:h.abort()
        return result
rclpy.init();node=Bridge();ex=MultiThreadedExecutor(num_threads=4);ex.add_node(node)
try:ex.spin()
finally:node.destroy_node();rclpy.shutdown()
