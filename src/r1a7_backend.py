"""Independent MoveIt 2 grasp executor. All IK/collision/planning is in MoveIt."""
import argparse
import os
import json
import subprocess
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetStateValidity, GetMotionPlan, GetCartesianPath, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, CollisionObject, AttachedCollisionObject, AllowedCollisionEntry, PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from frames import pose_values
from r1a7_frames import grasp_to_r1a7_tcp
import r1a7_plant as plant
ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'results'/('run_'+str(time.time_ns()))
RUN.mkdir(parents=True,exist_ok=True)
BASE='r1a7_world';TCP='r1a7_tcp';GROUP='r1a7_arm'
JOINTS=tuple(f'J{i}' for i in range(1,8))
TOUCH=['dex1_Link1_3','dex1_Link2_3']
FOCUS=('J5','J6','J7')
MIN_MARGIN=float(os.environ.get('R1A7_MIN_JOINT_MARGIN_RAD','0.05'))
BASE_POSE=tuple(float(v) for v in os.environ.get('R1A7_BASE_POSE','0,0,0,90').split(','))
PEDESTAL_SIZE=tuple(float(v) for v in os.environ.get('R1A7_PEDESTAL_SIZE','0.10,0.10,0.20').split(','))
if len(BASE_POSE)!=4 or len(PEDESTAL_SIZE)!=3:raise ValueError('invalid R1 installation parameters')
class Failure(RuntimeError):
    def __init__(self,category,detail=''):super().__init__(detail);self.category=category

def pose(T):
    p,q=pose_values(T);m=Pose();m.position.x,m.position.y,m.position.z=p;m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=q;return m

def stamped(T):
    m=PoseStamped();m.header.frame_id=BASE;m.pose=pose(T);return m

class R1A7Backend(Node):
    def __init__(self):
        super().__init__('r1a7_moveit_backend')
        self.rpc={}
        for name,typ in [('compute_ik',GetPositionIK),('compute_fk',GetPositionFK),('check_state_validity',GetStateValidity),('plan_kinematic_path',GetMotionPlan),('compute_cartesian_path',GetCartesianPath),('apply_planning_scene',ApplyPlanningScene),('get_planning_scene',GetPlanningScene)]:
            c=self.create_client(typ,'/'+name)
            if not c.wait_for_service(timeout_sec=60):raise Failure('NO_PLAN','service unavailable '+name)
            self.rpc[name]=c
        self.exec_client=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.hand=ActionClient(self,GripperCommand,'/dex1_gripper/gripper_action')
        self.limits={j.attrib['name']:(float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper'])) for j in ET.parse(ROOT/'config/r1a7_dex1.urdf').findall('joint') if j.attrib['type'] in ['revolute','prismatic']}
        self.rng=np.random.default_rng(20260928)
    def future(self,f,timeout=90):
        rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
        if not f.done():raise Failure('NO_PLAN','ROS operation timeout')
        return f.result()
    def call(self,name,req):return self.future(self.rpc[name].call_async(req))
    def measured(self):
        s=plant.state();r=RobotState();r.joint_state.name=s['names'];r.joint_state.position=s['q'];return r
    def action(self,client,goal,category):
        if not client.wait_for_server(timeout_sec=10):raise Failure(category,'action server unavailable')
        h=self.future(client.send_goal_async(goal))
        if not h.accepted:raise Failure(category,'action rejected')
        try:r=self.future(h.get_result_async(),180)
        except Exception:
            self.future(h.cancel_goal_async(),10);raise
        if r.status!=4:raise Failure(category,'action status '+str(r.status))
        return r.result
    def execute(self,traj,category):
        record={'stage':self.stage,'joint_names':list(traj.joint_trajectory.joint_names),'points':[{'t':p.time_from_start.sec+p.time_from_start.nanosec*1e-9,'q':list(p.positions)} for p in traj.joint_trajectory.points]}
        self.executions.append(record)
        start_t=plant.state()['t']
        g=ExecuteTrajectory.Goal();g.trajectory=traj
        r=self.action(self.exec_client,g,category)
        history=[h for h in plant.state()['history'] if h['t']>=start_t]
        record['max_arm_tracking_error_rad']=max((h.get('arm_tracking_error_rad',0.) for h in history),default=None)
        if r.error_code.val!=1:raise Failure(category,'execution code '+str(r.error_code.val))
    def gripper(self,width):
        g=GripperCommand.Goal();g.command.position=width;g.command.max_effort=20.
        return self.action(self.hand,g,'BAD_CONTACT')
    def validate(self,state):
        for n,q in zip(state.joint_state.name,state.joint_state.position):
            if n in self.limits and not self.limits[n][0]-1e-6<=q<=self.limits[n][1]+1e-6:raise Failure('JOINT_LIMIT',n)
        req=GetStateValidity.Request();req.robot_state=state;req.group_name=GROUP
        r=self.call('check_state_validity',req)
        if not r.valid:
            pairs=[(c.contact_body_1,c.contact_body_2) for c in r.contacts]
            category='TABLE_COLLISION' if any('table' in x for pair in pairs for x in pair) else 'SELF_COLLISION' if pairs and all(x.startswith(('Link','dex1_','base_link')) for pair in pairs for x in pair) else 'WORLD_COLLISION'
            raise Failure('COLLISION',category+': '+str(pairs))
    def margin(self,state):
        values=dict(zip(state.joint_state.name,state.joint_state.position))
        return min(min(values[n]-self.limits[n][0],self.limits[n][1]-values[n]) for n in FOCUS)
    def trajectory_margin(self,traj):
        names=list(traj.joint_trajectory.joint_names)
        return min((min(min(p.positions[names.index(n)]-self.limits[n][0],
                            self.limits[n][1]-p.positions[names.index(n)]) for n in FOCUS)
                    for p in traj.joint_trajectory.points),default=float('inf'))
    def require_margin(self,margin,where):
        if margin<=MIN_MARGIN:raise Failure('LOW_JOINT_MARGIN',f'{where}: {margin:.6f} <= {MIN_MARGIN:.6f} rad')
    def ik(self,T,seed):
        # KDL searches the 7D nullspace from each seed. A failed seed does not
        # imply that the target pose is outside the redundant arm's workspace.
        candidates=[seed]
        for _ in range(int(os.environ.get('R1A7_IK_RANDOM_SEEDS','24'))):
            state=RobotState();state.joint_state=JointState()
            state.joint_state.name=list(seed.joint_state.name)
            values=dict(zip(seed.joint_state.name,seed.joint_state.position))
            for name in JOINTS:
                lo,hi=self.limits[name];values[name]=float(self.rng.uniform(lo+.01,hi-.01))
            state.joint_state.position=[values[n] for n in state.joint_state.name]
            candidates.append(state)
        reasons=[];reason_categories=[]
        for candidate in candidates:
            req=GetPositionIK.Request();ik=req.ik_request;ik.group_name=GROUP;ik.ik_link_name=TCP;ik.pose_stamped=stamped(T);ik.robot_state=candidate;ik.avoid_collisions=False;ik.timeout.nanosec=250_000_000
            r=self.call('compute_ik',req)
            if r.error_code.val!=1:
                reasons.append(str(r.error_code.val));reason_categories.append('NO_IK');continue
            try:
                self.validate(r.solution)
                margin=self.margin(r.solution)
                self.require_margin(margin,'IK')
                return r.solution
            except Failure as e:
                reasons.append(f'{e.category}:{e}');reason_categories.append(e.category)
        category='COLLISION' if 'COLLISION' in reason_categories else 'LOW_JOINT_MARGIN' if 'LOW_JOINT_MARGIN' in reason_categories else 'NO_IK'
        raise Failure(category,','.join(reasons))
    def check_fk(self):
        req=GetPositionFK.Request();req.header.frame_id=BASE;req.fk_link_names=[TCP];req.robot_state=self.measured()
        r=self.call('compute_fk',req)
        if r.error_code.val!=1:raise Failure('TF_ERROR','FK service failure')
        p=r.pose_stamped[0].pose;s=plant.state()
        pos=np.array([p.position.x,p.position.y,p.position.z]);q=np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);sq=np.roll(s['tcp_quat'],-1)
        dp=float(np.linalg.norm(pos-s['tcp']));dr=float((Rotation.from_quat(q).inv()*Rotation.from_quat(sq)).magnitude())
        if dp>.003 or dr>.02:raise Failure('TF_ERROR',f'MoveIt/Isaac TCP mismatch {dp} m {dr} rad')
        return dict(position_error_m=dp,rotation_error_rad=dr)
    def cartesian(self,start,T):
        req=GetCartesianPath.Request();req.header.frame_id=BASE;req.start_state=start;req.group_name=GROUP;req.link_name=TCP;req.waypoints=[pose(T)];req.max_step=.003;req.jump_threshold=1.5;req.avoid_collisions=True
        r=self.call('compute_cartesian_path',req)
        if r.error_code.val!=1 or r.fraction<.999:raise Failure('NO_PLAN',f'Cartesian fraction={r.fraction}, code={r.error_code.val}')
        pts=r.solution.joint_trajectory.points
        if len(pts)<2 or pts[-1].time_from_start.sec+pts[-1].time_from_start.nanosec*1e-9<=0:raise Failure('NO_PLAN','untimed Cartesian path')
        self.require_margin(self.trajectory_margin(r.solution),'Cartesian path')
        # Slow Cartesian motion while preserving the computed joint path.
        for p in pts:
            t=(p.time_from_start.sec+p.time_from_start.nanosec*1e-9)*3
            p.time_from_start.sec=int(t);p.time_from_start.nanosec=int((t%1)*1e9)
            p.velocities=[v/3 for v in p.velocities];p.accelerations=[v/9 for v in p.accelerations]
        return r.solution
    def plan(self,start,goal):
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name=GROUP;r.pipeline_id='ompl';r.planner_id='RRTConnect';r.start_state=start;r.num_planning_attempts=5;r.allowed_planning_time=5.;r.max_velocity_scaling_factor=.15;r.max_acceleration_scaling_factor=.15
        con=Constraints()
        for n,q in zip(goal.joint_state.name,goal.joint_state.position):
            if n in JOINTS:
                c=JointConstraint();c.joint_name=n;c.position=q;c.tolerance_above=.001;c.tolerance_below=.001;c.weight=1.;con.joint_constraints.append(c)
        r.goal_constraints=[con]
        out=self.call('plan_kinematic_path',req).motion_plan_response
        if out.error_code.val!=1:raise Failure('NO_PLAN',str(out.error_code.val))
        self.require_margin(self.trajectory_margin(out.trajectory),'planned path')
        return out.trajectory
    def scene(self,attach=False):
        req=ApplyPlanningScene.Request();s=req.scene;s.is_diff=True;s.robot_state.is_diff=True
        if not attach:
            detach=AttachedCollisionObject();detach.object.id='box';detach.object.operation=CollisionObject.REMOVE;s.robot_state.attached_collision_objects=[detach]
        geometry=[('table',[.5,0,-.025],[.7,.7,.05]),
                  ('box',plant.state()['box'],[.045,.045,.05])]
        if BASE_POSE[2]>0:
            geometry.append(('r1a7_pedestal',[BASE_POSE[0],BASE_POSE[1],PEDESTAL_SIZE[2]/2],list(PEDESTAL_SIZE)))
        for name,center,size in geometry:
            obj=CollisionObject();obj.id=name;obj.header.frame_id=BASE;obj.operation=CollisionObject.ADD
            shape=SolidPrimitive();shape.type=SolidPrimitive.BOX;shape.dimensions=size;pp=Pose();pp.position.x,pp.position.y,pp.position.z=map(float,center);pp.orientation.w=1.;obj.primitives=[shape];obj.primitive_poses=[pp]
            if name=='box' and attach:
                live=plant.state()
                tcp_rotation=Rotation.from_quat(np.roll(live['tcp_quat'],-1))
                relative=tcp_rotation.inv().apply(np.array(center)-np.array(live['tcp']))
                relative_quat=tcp_rotation.inv().as_quat()
                obj.header.frame_id=TCP
                pp.position.x,pp.position.y,pp.position.z=map(float,relative)
                pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,relative_quat)
                att=AttachedCollisionObject();att.link_name=TCP;att.touch_links=TOUCH;att.object=obj;s.robot_state.attached_collision_objects=[att]
            else:s.world.collision_objects.append(obj)
        # Only finger-to-target contact is allowed; table and palm remain checked.
        get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm=self.call('get_planning_scene',get).scene.allowed_collision_matrix
        for name in ['box']+TOUCH:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values: row.enabled.append(False)
                row=AllowedCollisionEntry();row.enabled=[False]*len(acm.entry_names);acm.entry_values.append(row)
        bi=acm.entry_names.index('box')
        for name in TOUCH:
            fi=acm.entry_names.index(name);acm.entry_values[bi].enabled[fi]=True;acm.entry_values[fi].enabled[bi]=True
        s.allowed_collision_matrix=acm
        if not self.call('apply_planning_scene',req).success:raise Failure('NO_PLAN','planning scene update failed')
    def trial(self,index,grasps_json=None):
        result={'trial':index,'success':False,'candidates':[]}
        self.executions=[];self.stage='RESET'
        try:
            plant.command({'op':'reset'});plant.settle(1.)
            result['tf_check']=self.check_fk();self.scene();self.gripper(.09)
            self.stage='PERCEPTION'
            capture=plant.command({'op':'capture'})
            if not capture['ok'] or capture.get('object_points',0)<30:raise Failure('NO_GRASP',str(capture))
            if grasps_json is None:
                output=RUN/f'trial_{index:02d}_grasps.json'
                cmd=['/data1/home/rangeryx/.conda/envs/anygrasp/bin/python',str(ROOT/'src/infer.py'),'--input',capture['path'],'--output',str(output)]
                with open(RUN/f'infer_{index:02d}.log','w') as f:
                    r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=180,env={**os.environ,"CUDA_VISIBLE_DEVICES":"1","LD_LIBRARY_PATH":"","PYTHONPATH":"","PATH":"/data1/home/rangeryx/.conda/envs/anygrasp/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin","CONDA_PREFIX":"/data1/home/rangeryx/.conda/envs/anygrasp"})
                if r.returncode:raise Failure('NO_GRASP','inference subprocess failed; see log')
            else:
                output=Path(grasps_json)
                result['reused_anygrasp_output']=str(output)
            data=json.loads(output.read_text())
            if data['frame']!='camera_optical':raise Failure('TF_ERROR','unexpected grasp frame')
            if not data['grasps']:raise Failure('NO_GRASP','zero candidates')
            result['capture']=capture
            self.stage='CANDIDATE_CHECK'
            chosen=None
            for g in data['grasps']:
                detail={'rank':g['rank'],'score':g['score']}
                try:
                    if not 0<g['width']<=.09:raise Failure('GRIPPER_WIDTH','outside Dex1 opening')
                    pre,grasp,micro,lift=grasp_to_r1a7_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
                    current=self.measured()
                    gs=self.ik(grasp,current);ps=self.ik(pre,gs)
                    approach=self.cartesian(ps,grasp)
                    micro_check=self.cartesian(gs,micro)
                    lift_check=self.cartesian(gs,lift)
                    traj=self.plan(current,ps)
                    detail['status']='VALID';detail['T_B_TCP']=grasp.tolist()
                    detail['joint_margin_rad']=min(self.margin(gs),self.margin(ps),
                        self.trajectory_margin(approach),self.trajectory_margin(micro_check),
                        self.trajectory_margin(lift_check),self.trajectory_margin(traj))
                    chosen=(pre,grasp,micro,lift,traj,approach,g)
                except Failure as e:detail.update(status=e.category,detail=str(e))
                except ValueError as e:detail.update(status='TF_ERROR',detail=str(e))
                result['candidates'].append(detail)
                if chosen:break
            if chosen is None:raise Failure('NO_EXECUTABLE_CANDIDATE','all top-K candidates rejected')
            pre,grasp,micro,lift,traj,approach,g=chosen
            result['selected_rank']=g['rank']
            self.stage='PREGRASP'
            self.execute(traj,'NO_PLAN')
            # Recompute from measured pregrasp to prevent stale start-state execution.
            self.stage='APPROACH'
            approach=self.cartesian(self.measured(),grasp);self.execute(approach,'APPROACH_FAIL')
            z0=plant.state()['box'][2];result['initial_z']=z0
            self.stage='CLOSE'
            self.gripper(0.)
            plant.settle(.3)
            s=plant.state();recent=[h for h in s['history'] if h['t']>=s['t']-.2]
            result['close_samples']=recent
            if not recent or not all(min(h['forces'])>.1 for h in recent):raise Failure('BAD_CONTACT','no sustained bilateral finger/box contact')
            self.stage='MICRO_LIFT'
            self.scene(attach=True)
            mt=self.cartesian(self.measured(),micro)
            self.execute(mt,'NO_PLAN')
            s=plant.state()
            if s['box'][2]<z0+.012:raise Failure('CONTACT_LOSS','object did not follow 2 cm micro-lift')
            self.stage='LIFT'
            try:lt=self.cartesian(self.measured(),lift)
            except Failure as e:raise Failure('NO_PLAN',str(e))
            self.execute(lt,'NO_PLAN')
            self.stage='HOLD'
            start=plant.state()['t'];plant.settle(2.2);s=plant.state()
            samples=[h for h in s['history'] if start<=h['t']<=start+2.1]
            result['hold_samples']=samples
            if len(samples)<2 or samples[-1]['t']-samples[0]['t']<2.:raise Failure('DROP','insufficient physics samples')
            heights=[h['z'] for h in samples];result['lift_m']=max(heights)-z0
            if max(heights)<z0+.08:raise Failure('CONTACT_LOSS','object not lifted 8 cm')
            if min(heights)<z0+.07:raise Failure('DROP','object lost lift height during hold')
            if max(heights)-min(heights)>.01 or not all(min(h['forces'])>.1 for h in samples):raise Failure('CONTACT_LOSS','unstable hold or lost bilateral contact')
            result.update(success=True,category='SUCCESS')
        except Failure as e:result.update(category=e.category,detail=str(e))
        except Exception as e:result.update(category='SYSTEM_ERROR',detail=str(e),traceback=traceback.format_exc())
        try:
            measured=plant.state()
            hold=[h for h in measured['history'] if h['t']>=measured['t']-2.2]
            tcp=np.array([h['tcp'] for h in hold])
            result['drive_metrics']={'recent_max_arm_tracking_error_rad':max((h.get('arm_tracking_error_rad',0.) for h in hold),default=None),
                                     'recent_tcp_range_m':(np.ptp(tcp,axis=0).tolist() if len(tcp) else None)}
        except Exception:pass
        result['last_stage']=self.stage
        result['executions']=self.executions
        (RUN/f'trial_{index:02d}.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ['hold_samples','candidates','executions','close_samples']}),flush=True)
        return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trials',type=int,default=10);p.add_argument('--grasps-json',type=Path);p.add_argument('--grasps-dir',type=Path);a=p.parse_args()
    if a.grasps_json and a.grasps_dir:p.error('use either --grasps-json or --grasps-dir')
    rclpy.init();node=R1A7Backend()
    results=[node.trial(i+1,(a.grasps_dir/f'trial_{i+1:02d}_grasps.json' if a.grasps_dir else a.grasps_json)) for i in range(a.trials)]
    from collections import Counter
    report={'trials':len(results),'successes':sum(r['success'] for r in results),'categories':dict(Counter(r['category'] for r in results)),'passed':len(results)>=10 and sum(r['success'] for r in results[-10:])>=8}
    (RUN/'summary.json').write_text(json.dumps(report,indent=2));print(report)
    node.destroy_node();rclpy.shutdown()
