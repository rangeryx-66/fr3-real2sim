"""Independent MoveIt 2 grasp executor. All IK/collision/planning is in MoveIt."""
import argparse
import os
import json
import subprocess
import time
import traceback
import hashlib
from functools import lru_cache
from collections import Counter
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
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from frames import pose_values
from r1a7_frames import grasp_to_r1a7_tcp
from r1a7_grasp_adaptation import variants, manifold_variants, contact_geometry, surface_contact_geometry, pad_table_penetration, path_targets
import r1a7_plant as plant
ROOT=Path(__file__).resolve().parents[1]
RUN=Path(os.environ.get('R1A7_RUN_DIR',str(ROOT/'results'/('run_'+str(time.time_ns())))))
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

@lru_cache(maxsize=16)
def arena_surface(name):
    source=Path(os.environ.get('R1A7_ARENA_ASSET_DIR','/data1/home/rangeryx/fr3_moveit_grasp/assets/arena_complex'))/f'{name}_mesh.npz'
    with np.load(source) as data:
        return np.asarray(data['vertices']),np.asarray(data['triangles'])

@lru_cache(maxsize=16)
def arena_mesh(name):
    vertices,triangles=arena_surface(name)
    mesh=Mesh();mesh.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in vertices]
    mesh.triangles=[MeshTriangle(vertex_indices=[int(i) for i in f]) for f in triangles]
    return mesh

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
        self.adaptation_cache={}
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
    def ik(self,T,seed,random_seeds=None):
        # KDL searches the 7D nullspace from each seed. A failed seed does not
        # imply that the target pose is outside the redundant arm's workspace.
        candidates=[seed]
        for _ in range(int(os.environ.get('R1A7_IK_RANDOM_SEEDS','24')) if random_seeds is None else random_seeds):
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
        counts=Counter(reasons)
        raise Failure(category,'; '.join(f'{reason} x{count}' for reason,count in counts.most_common(5)))
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
        live=plant.state();target=live['object']
        if target['shape']=='arena_mesh':
            inventory=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
            tb=np.array(inventory['table']['bounds']);table_size=tb[1]-tb[0]
            geometry=[('table',[.5,0,-table_size[2]/2],table_size.tolist()),('box',live['box'],target['size'])]
            geometry.extend((ob['id'],ob['position'],(np.asarray(ob['aabb_size'])+.002).tolist())
                            for ob in live['clutter']['obstacles'])
        else:
            geometry=[('table',[.5,0,-.025],[.7,.7,.05]),('box',live['box'],target['size'])]
        if BASE_POSE[2]>0:
            geometry.append(('r1a7_pedestal',[BASE_POSE[0],BASE_POSE[1],PEDESTAL_SIZE[2]/2],list(PEDESTAL_SIZE)))
        for name,center,size in geometry:
            obj=CollisionObject();obj.id=name;obj.header.frame_id=BASE;obj.operation=CollisionObject.ADD
            shape=SolidPrimitive();shape.type=SolidPrimitive.BOX;shape.dimensions=size
            if name=='box' and target['shape']=='cylinder':
                shape.type=SolidPrimitive.CYLINDER;shape.dimensions=[float(target['height']),float(target['radius'])]
            pp=Pose();pp.position.x,pp.position.y,pp.position.z=map(float,center);pp.orientation.w=1.
            if name=='box':
                bq=live['box_quat'];pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,[bq[1],bq[2],bq[3],bq[0]])
            if name=='box' and target['shape']=='arena_mesh':
                obj.meshes=[arena_mesh(target['id'])];obj.mesh_poses=[pp]
            else:obj.primitives=[shape];obj.primitive_poses=[pp]
            if name=='box' and attach:
                tcp_rotation=Rotation.from_quat(np.roll(live['tcp_quat'],-1))
                relative=tcp_rotation.inv().apply(np.array(center)-np.array(live['tcp']))
                relative_quat=(tcp_rotation.inv()*Rotation.from_quat(np.roll(live['box_quat'],-1))).as_quat()
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
    def arena_points(self):
        live=plant.state();vertices,_=arena_surface(live['object']['id'])
        return Rotation.from_quat(np.roll(live['box_quat'],-1)).apply(vertices)+np.asarray(live['box'])
    def evaluate_variant(self,T,raw,box_center,seed,quick=False,surface_points=None):
        obj=plant.state()['object']
        if obj['shape']=='arena_mesh':
            ok,reason,contact_score=surface_contact_geometry(T,self.arena_points() if surface_points is None else surface_points)
        else:ok,reason,contact_score=contact_geometry(T,box_center,obj['size'],obj['shape'],obj['yaw'])
        if not ok:raise Failure('BAD_GRASP_GEOMETRY',reason)
        table_depth=pad_table_penetration(T)
        if table_depth>.001:raise Failure('COLLISION',f'TABLE_COLLISION: official Dex1 pad vertices penetrate {table_depth:.4f} m')
        pre,micro,lift=path_targets(T)
        gs=self.ik(T,seed,random_seeds=3 if quick else None)
        ps=self.ik(pre,gs,random_seeds=3 if quick else None)
        approach=self.cartesian(ps,T)
        micro_check=self.cartesian(gs,micro)
        lift_check=self.cartesian(gs,lift)
        min_margin=min(self.margin(gs),self.margin(ps),self.trajectory_margin(approach),
                       self.trajectory_margin(micro_check),self.trajectory_margin(lift_check))
        self.require_margin(min_margin,'grasp path')
        displacement=float(np.linalg.norm(T[:3,3]-raw[:3,3]))
        angular=float((Rotation.from_matrix(raw[:3,:3]).inv()*Rotation.from_matrix(T[:3,:3])).magnitude())
        score=4*min_margin+contact_score-10*displacement-angular
        return dict(pre=pre,grasp=T,micro=micro,lift=lift,grasp_state=gs,pre_state=ps,
                    approach=approach,margin=min_margin,contact_score=contact_score,
                    displacement_m=displacement,angular_rad=angular,score=score)
    def select_candidates(self,data,mode,box_center):
        """Keep raw candidates unchanged and rank collision-free Dex1 alternatives."""
        obj=plant.state()['object']
        key=(mode,json.dumps(data['grasps'],sort_keys=True),tuple(np.round(box_center,5)),
             obj['id'],tuple(obj['size']),round(obj['yaw'],5))
        cached=self.adaptation_cache.get(key)
        if cached is not None:return cached
        details=[];feasible=[];home=self.measured()
        surface=self.arena_points() if obj['shape']=='arena_mesh' else None
        for g in data['grasps']:
            record={'rank':g['rank'],'anygrasp_score':g['score'],'raw_grasp':g,
                    'variants_tested':0,'failures':{}}
            if not 0<g['width']<=.09:
                record.update(status='GRIPPER_WIDTH',detail='outside Dex1 opening')
                details.append(record);continue
            _,raw,_,_=grasp_to_r1a7_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
            record['raw_T_B_TCP']=raw.tolist()
            best=None
            if surface is not None:
                anchor=surface[np.argmin(np.sum((surface-raw[:3,3])**2,axis=1))]
                nearby=surface[np.sum((surface-anchor)**2,axis=1)<.09**2]
                choices=manifold_variants(raw,anchor,limit=int(os.environ.get('R1A7_MANIFOLD_LIMIT','100'))) if mode=='adapted' else [next(variants(raw))]
            else:
                nearby=None
                choices=variants(raw,limit=int(os.environ.get('R1A7_VARIANT_LIMIT','46'))) if mode=='adapted' else [next(variants(raw))]
            for v in choices:
                record['variants_tested']+=1
                try:
                    evaluated=self.evaluate_variant(v.transform,raw,box_center,home,quick=mode=='adapted',surface_points=nearby)
                    if best is None or evaluated['score']>best[0]['score']:best=(evaluated,v)
                except Failure as e:
                    record['failures'][e.category]=record['failures'].get(e.category,0)+1
                    if len(record.get('failure_examples',[]))<3:record.setdefault('failure_examples',[]).append(f'{v.label}: {e.category}: {e}')
            if best:
                evaluated,v=best
                record.update(status='KINEMATIC_PATH_VALID',variant=v.label,adapted_T_B_TCP=v.transform.tolist(),
                              joint_margin_rad=evaluated['margin'],contact_score=evaluated['contact_score'],
                              translation_m=v.translation_m,rotation_rad=v.rotation_rad,score=evaluated['score'])
                feasible.append((record,g,v.transform))
            else:record['status']=max(record['failures'],key=record['failures'].get) if record['failures'] else 'NO_IK'
            details.append(record)
            print('CANDIDATE_CHECK',json.dumps(dict(rank=g['rank'],status=record['status'],
                 variants_tested=record['variants_tested'],failures=record['failures'])),flush=True)
        feasible.sort(key=lambda item:item[0]['score'],reverse=True)
        self.adaptation_cache[key]=(details,[(r,g,T.tolist()) for r,g,T in feasible])
        return self.adaptation_cache[key]
    def verify_close(self,result,label):
        self.stage=label
        close_result=self.gripper(0.)
        plant.settle(.3)
        s=plant.state();recent=[h for h in s['history'] if h['t']>=s['t']-.2]
        finger_q={n:q for n,q in zip(s['names'],s['q']) if n.startswith('dex1_Joint')}
        bilateral_force=bool(recent and all(min(h['forces'])>.1 for h in recent))
        bilateral_stall=all(finger_q.get(n,.0245)<.018 for n in ('dex1_Joint1_1','dex1_Joint2_1'))
        evidence=dict(stage=label,bilateral_force=bilateral_force,bilateral_stall=bilateral_stall,
                      finger_q=finger_q,reported_position_m=close_result.position)
        result.setdefault('contact_checks',[]).append(evidence)
        result['contact_evidence']={'bilateral_force':bilateral_force,'bilateral_stall':bilateral_stall}
        result['close_finger_q']=finger_q;result['close_reported_position_m']=close_result.position
        result['close_samples']=recent
        if not (bilateral_force or bilateral_stall):raise Failure('BAD_CONTACT','no bilateral force or finger stall')

    def micro_lift_check(self,grasp,z0):
        self.stage='MICRO_LIFT'
        self.scene(attach=True)
        micro=path_targets(grasp)[1]
        mt=self.cartesian(self.measured(),micro)
        self.execute(mt,'NO_PLAN')
        if plant.state()['box'][2]<z0+.012:
            raise Failure('CONTACT_LOSS','object did not follow 2 cm micro-lift')

    def recover_contact(self,grasp,raw,z0,result):
        """One bounded re-close and 3 mm depth correction, then require a loaded micro-lift."""
        result.setdefault('recovery_events',[]).append('reclose')
        # A failed micro-lift leaves the hand 2 cm above the object. Detach the
        # planning proxy, reopen, and return to the checked grasp before closing.
        self.scene(attach=False)
        self.gripper(.09)
        if np.linalg.norm(np.asarray(plant.state()['tcp'])-grasp[:3,3])>.002:
            self.stage='CONTACT_RECOVERY_RETURN'
            self.execute(self.cartesian(self.measured(),grasp),'NO_PLAN')
        try:
            self.verify_close(result,'RECLOSE')
            self.micro_lift_check(grasp,z0)
            return grasp
        except Failure as error:
            if error.category not in ('BAD_CONTACT','CONTACT_LOSS'):raise
        corrected=grasp.copy();corrected[:3,3]+=.003*grasp[:3,2]
        result['recovery_events'].append('depth_correction_3mm')
        self.scene(attach=False)
        self.gripper(.09)
        box_center=plant.state()['box']
        self.evaluate_variant(corrected,raw,box_center,self.measured())
        self.stage='DEPTH_CORRECTION'
        self.execute(self.cartesian(self.measured(),corrected),'NO_PLAN')
        self.verify_close(result,'CORRECTED_CLOSE')
        self.micro_lift_check(corrected,z0)
        return corrected

    def trial(self,index,grasps_json=None,mode='adapted',scenario=None,reference_capture=None):
        result={'trial':index,'success':False,'candidates':[]}
        self.executions=[];self.stage='RESET'
        try:
            reset={'op':'reset'}
            if scenario:
                if 'seed' in scenario:reset['seed']=scenario['seed']
                else:reset.update(xy=scenario['xy'],yaw=scenario['yaw'])
            plant.command(reset);plant.settle(1.)
            result['tf_check']=self.check_fk();self.scene();self.gripper(.09)
            result['object']=plant.state()['object'];result['scenario']=scenario
            self.stage='PERCEPTION'
            if reference_capture:
                cloud=Path(reference_capture['cloud'])
                digest=hashlib.sha256(cloud.read_bytes()).hexdigest()
                if digest!=reference_capture['cloud_sha256']:
                    raise Failure('TF_ERROR','FR3 reference point cloud hash mismatch')
                with np.load(cloud) as points:
                    point_count=int(np.count_nonzero(points['mask']))
                    camera_pose=points['T_B_C'].tolist()
                capture=dict(ok=True,path=str(cloud),object_points=point_count,
                             source='fr3_reference',sha256=digest,T_B_C=camera_pose)
            else:
                capture=plant.command({'op':'capture'})
                if not capture['ok']:raise Failure('NO_GRASP',str(capture))
            result['object_points']=capture.get('object_points',0)
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
            if reference_capture:
                grasp_digest=hashlib.sha256(output.read_bytes()).hexdigest()
                if grasp_digest!=reference_capture['grasps_sha256']:
                    raise Failure('TF_ERROR','FR3 reference AnyGrasp output hash mismatch')
                if not np.allclose(data['T_B_C'],capture['T_B_C'],atol=1e-9):
                    raise Failure('TF_ERROR','FR3 reference camera transform mismatch')
                result['reference_input']=dict(cloud_sha256=digest,grasps_sha256=grasp_digest,
                                               fr3_trial=reference_capture['fr3_trial'])
            result['capture']=capture
            if capture.get('object_points',0)<30:
                raise Failure('NO_GRASP','AnyGrasp ran, but the target region has fewer than 30 visible points')
            if not data['grasps']:raise Failure('NO_GRASP','zero candidates')
            if mode=='capture':
                result.update(category='CAPTURED',grasp_count=len(data['grasps']),grasp_file=str(output))
                raise StopIteration
            self.stage='CANDIDATE_CHECK'
            box_center=plant.state()['box']
            details,feasible=self.select_candidates(data,mode,box_center)
            result['mode']=mode;result['candidates']=details
            result['candidate_counts']={'raw':len(data['grasps']),'path_valid':len(feasible),
                                        'collision_free':len(feasible)}
            result['candidate_attempts']=[]
            last_failure=None
            max_candidates=3 if mode=='adapted' else 1
            for detail,g,T_list in feasible[:max_candidates]:
                attempt=dict(rank=g['rank'],variant=detail.get('variant'),status='STARTED')
                result['candidate_attempts'].append(attempt)
                try:
                    grasp=np.asarray(T_list)
                    current=self.measured()
                    evaluated=self.evaluate_variant(grasp,np.asarray(detail['raw_T_B_TCP']),box_center,current)
                    traj=self.plan(current,evaluated['pre_state'])
                    detail['status']='PLANNED';detail['planning_margin_rad']=self.trajectory_margin(traj)
                    result['planning_succeeded']=True
                    result['selected_rank']=g['rank']
                    self.stage='PREGRASP';self.execute(traj,'NO_PLAN')
                    self.stage='APPROACH'
                    self.execute(self.cartesian(self.measured(),grasp),'NO_PLAN')
                    z0=plant.state()['box'][2];result['initial_z']=z0
                    try:
                        self.verify_close(result,'CLOSE')
                        self.micro_lift_check(grasp,z0)
                    except Failure as error:
                        if mode!='adapted' or error.category not in ('BAD_CONTACT','CONTACT_LOSS'):
                            raise
                        attempt['initial_contact_failure']=error.category
                        grasp=self.recover_contact(grasp,np.asarray(detail['raw_T_B_TCP']),z0,result)
                    self.stage='LIFT'
                    lift=path_targets(grasp)[2]
                    self.execute(self.cartesian(self.measured(),lift),'NO_PLAN')
                    self.stage='HOLD'
                    start=plant.state()['t'];plant.settle(2.2);s=plant.state()
                    samples=[h for h in s['history'] if start<=h['t']<=start+2.1]
                    result['hold_samples']=samples
                    if len(samples)<2 or samples[-1]['t']-samples[0]['t']<2.:raise Failure('DROP','insufficient physics samples')
                    heights=[h['z'] for h in samples];result['lift_m']=max(heights)-z0
                    if max(heights)<z0+.08:raise Failure('CONTACT_LOSS','object not lifted 8 cm')
                    if min(heights)<z0+.07:raise Failure('DROP','object lost lift height during hold')
                    if max(heights)-min(heights)>.01:raise Failure('CONTACT_LOSS','unstable hold height')
                    attempt['status']='SUCCESS'
                    result.update(success=True,category='SUCCESS')
                    break
                except Failure as error:
                    last_failure=error
                    attempt.update(status=error.category,detail=str(error),stage=self.stage)
                    detail.update(status=error.category,detail=str(error))
                    if mode!='adapted' or error.category not in ('BAD_CONTACT','CONTACT_LOSS','NO_PLAN'):
                        break
                    if detail is feasible[-1][0]:break
                    result.setdefault('recovery_events',[]).append('next_adapted_candidate')
                    retry_reset={'op':'reset'}
                    if scenario:
                        if 'seed' in scenario:retry_reset['seed']=scenario['seed']
                        else:retry_reset.update(xy=scenario['xy'],yaw=scenario['yaw'])
                    plant.command(retry_reset)
                    plant.settle(1.);self.scene();self.gripper(.09)
                except ValueError as error:
                    last_failure=Failure('TF_ERROR',str(error));attempt.update(status='TF_ERROR',detail=str(error))
                    break
            if not result['success']:
                if last_failure:raise last_failure
                raise Failure('NO_EXECUTABLE_CANDIDATE','all top-K candidates rejected')
        except StopIteration:pass
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
    p=argparse.ArgumentParser();p.add_argument('--trials',type=int,default=10);p.add_argument('--grasps-json',type=Path);p.add_argument('--grasps-dir',type=Path);p.add_argument('--scenarios-json',type=Path);p.add_argument('--reference-manifest',type=Path);p.add_argument('--mode',choices=['capture','raw','adapted'],default='adapted');a=p.parse_args()
    if a.grasps_json and a.grasps_dir:p.error('use either --grasps-json or --grasps-dir')
    rclpy.init();node=R1A7Backend()
    scenarios=json.loads(a.scenarios_json.read_text()) if a.scenarios_json else [None]*a.trials
    if len(scenarios)!=a.trials:p.error('scenario count must match --trials')
    references=json.loads(a.reference_manifest.read_text()) if a.reference_manifest else [None]*a.trials
    if len(references)!=a.trials:p.error('reference count must match --trials')
    results=[node.trial(i+1,(a.grasps_dir/f'trial_{i+1:02d}_grasps.json' if a.grasps_dir else a.grasps_json),a.mode,scenarios[i],references[i]) for i in range(a.trials)]
    from collections import Counter
    report={'mode':a.mode,'trials':len(results),'successes':sum(r['success'] for r in results),
            'categories':dict(Counter(r['category'] for r in results)),
            'candidate_counts':{key:sum(r.get('candidate_counts',{}).get(key,0) for r in results)
                                for key in ('raw','path_valid','collision_free')},
            'planning_successes':sum(r.get('planning_succeeded',False) for r in results),
            'contact_loss_events':sum(a.get('status')=='CONTACT_LOSS' or a.get('initial_contact_failure')=='CONTACT_LOSS' for r in results for a in r.get('candidate_attempts',[])),
            'recovery_events':dict(Counter(event for r in results for event in r.get('recovery_events',[]))),
            'approach_executions':sum(r.get('last_stage') in ('CLOSE','MICRO_LIFT','LIFT','HOLD') for r in results),
            'final_grasp_success_rate':sum(r['success'] for r in results)/len(results),
            'passed':len(results)>=10 and sum(r['success'] for r in results[-10:])>=8}
    (RUN/'summary.json').write_text(json.dumps(report,indent=2));print(report)
    node.destroy_node();rclpy.shutdown()
