"""Paired GT grasp stability experiment with local refinement and regrasp."""
from __future__ import annotations
import argparse,copy,gzip,hashlib,json,os,sys,time,traceback,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import plant
from backend import Backend,Failure,BASE,TCP,TOUCH,pose
from clutter_backend import ClutterBackend,transform,collision_box
from settling_gate import GateThresholds
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import CollisionObject,AttachedCollisionObject,AllowedCollisionEntry,PlanningSceneComponents
from shape_msgs.msg import Mesh,MeshTriangle
from geometry_msgs.msg import Point

TARGET=os.environ['FR3_ARENA_TARGET']
MOVEIT_LIMIT=12
PORT=os.environ.get('CALIBRATION_PORT','18765')
plant.URL='http://127.0.0.1:'+PORT
ASSET_DIRECTORY=Path(os.environ.get('FR3_ASSET_DIRECTORY',ROOT/'assets/arena_complex'))
INVENTORY=json.loads((ASSET_DIRECTORY/'inventory.json').read_text())
MESHDATA=np.load(ASSET_DIRECTORY/f'{TARGET}_mesh.npz')
VERTICES=np.asarray(MESHDATA['vertices'])
MESH=Mesh();MESH.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in VERTICES]
MESH.triangles=[MeshTriangle(vertex_indices=list(map(int,f))) for f in MESHDATA['triangles']]

def mesh_object(T,frame=BASE,operation=CollisionObject.ADD):
    obj=CollisionObject();obj.id='box';obj.header.frame_id=frame;obj.operation=operation
    if operation==CollisionObject.ADD:obj.meshes=[MESH];obj.mesh_poses=[pose(T)]
    return obj

def serialize_transform(T):return np.asarray(T).tolist()

class StabilityBackend(ClutterBackend):
    def __init__(self,output,thresholds):
        Backend.__init__(self);self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.thresholds=thresholds;self.executions=[];self.stage='RESET';self.attached=None;self.last_contacts=[];self.planning_seconds=0.

    def phase(self,name):
        self.stage=name;plant.command(dict(op='calibration_phase',phase=name))

    def execute(self,traj,category):
        item={'stage':self.stage,'joint_names':list(traj.joint_trajectory.joint_names),'points':[{'t':p.time_from_start.sec+p.time_from_start.nanosec*1e-9,'q':list(p.positions)} for p in traj.joint_trajectory.points]}
        self.executions.append(item)
        # MoveIt still computes and collision-checks every trajectory.  The
        # isolated calibration simulator consumes that exact joint trajectory
        # through its deterministic bridge, as the preceding GT calibration did.
        command=dict(names=item['joint_names'],points=item['points'])
        answer=plant.command(dict(op='trajectory',**command),timeout=240)
        if not answer['ok']:raise Failure(category,'Isaac trajectory bridge failed '+str(answer))

    def execute_slow(self,traj,category,minimum_duration_s):
        trajectory=copy.deepcopy(traj);points=trajectory.joint_trajectory.points
        duration=points[-1].time_from_start.sec+points[-1].time_from_start.nanosec*1e-9
        scale=max(1.,minimum_duration_s/duration)
        for point in points:
            value=(point.time_from_start.sec+point.time_from_start.nanosec*1e-9)*scale
            point.time_from_start.sec=int(value);point.time_from_start.nanosec=int((value%1)*1e9)
            point.velocities=[x/scale for x in point.velocities];point.accelerations=[x/(scale*scale) for x in point.accelerations]
        self.execute(trajectory,category)

    def target_policy(self,allowed=()):
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm=self.call('get_planning_scene',req).scene.allowed_collision_matrix
        for name in ['box',*allowed]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:row.enabled.append(False)
                row=AllowedCollisionEntry();row.enabled=[False]*len(acm.entry_names);acm.entry_values.append(row)
        i=acm.entry_names.index('box')
        for j,name in enumerate(acm.entry_names):
            enabled=name in allowed;acm.entry_values[i].enabled[j]=enabled;acm.entry_values[j].enabled[i]=enabled
        request=ApplyPlanningScene.Request();request.scene.is_diff=True;request.scene.allowed_collision_matrix=acm;self.apply(request)

    def reset_scene_at(self,O):
        self.attached=None
        get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene=self.call('get_planning_scene',get).scene
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        for a in scene.robot_state.attached_collision_objects:
            a=copy.deepcopy(a);a.object.operation=CollisionObject.REMOVE;req.scene.robot_state.attached_collision_objects.append(a)
        for item in scene.world.collision_objects:
            item=copy.deepcopy(item);item.operation=CollisionObject.REMOVE;req.scene.world.collision_objects.append(item)
        table_bounds=np.asarray(INVENTORY['table']['bounds']);T=np.eye(4);T[:3,3]=[.5,0,-(table_bounds[1,2]-table_bounds[0,2])/2]
        req.scene.world.collision_objects += [collision_box('table',table_bounds[1]-table_bounds[0],T),mesh_object(O)]
        self.apply(req);self.target_policy(())

    def attach_actual(self,H,O,hypothetical=False):
        touches=TOUCH if hypothetical else [name for name,force in zip(TOUCH,plant.state()['forces']) if force>.1]
        if len(touches)!=2:raise Failure('CONTACT_LOSS','attachment requires current bilateral contact')
        self.target_policy(touches)
        att=AttachedCollisionObject();att.link_name=TCP;att.touch_links=list(touches);att.object=mesh_object(np.linalg.inv(H)@O,TCP)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;req.scene.robot_state.attached_collision_objects=[att]
        self.apply(req);self.attached=att;return att

    def close_force(self,total=30.):
        result=plant.command(dict(op='calibration_force',force_N=total))
        if not result['ok']:raise Failure('BAD_CONTACT',str(result))
        start=plant.state()['t'];stable=None
        while True:
            s=plant.state('calibration');f=np.asarray(s['calibration']['filtered_force_N'])
            good=abs(float(f.sum())-total)<=max(2.,total*.2) and min(s['forces'])>.1
            stable=s['t'] if good and stable is None else stable if good else None
            if stable is not None and s['t']-stable>=.3:return
            if s['t']-start>5.:raise Failure('BAD_CONTACT','FORCE_UNREACHED')
            time.sleep(.01)

    def open_hand(self):
        plant.command(dict(op='calibration_position'))
        command=dict(names=['fr3_finger_joint1','fr3_finger_joint2'],
                     points=[dict(t=1.,q=[.04,.04])],gripper=True)
        answer=plant.command(dict(op='trajectory',**command))
        if not answer['ok']:raise Failure('BAD_CONTACT','gripper open failed '+str(answer))

    def stability(self,since_t,old_gate):
        last=None
        for _ in range(5):
            last=plant.command(dict(op='calibration_stability',thresholds=self.thresholds.__dict__,old_gate=old_gate,since_t=since_t))
            if last.get('ok') and 'stability' in last:return last['stability']
            plant.settle(.05)
        raise Failure('SYSTEM_ERROR','stability RPC failed '+str(last))

    def plan_candidate(self,H,O,metric):
        detail=dict(checks=[],planning_seconds=0.)
        started=time.monotonic();self.reset_scene_at(O);self.stage='CANDIDATE_CHECK'
        try:
            current=self.measured();gs=self.ik(H,current);detail['checks']+=['grasp_ik','joint_limits','self_collision','world_collision']
            pre=H.copy();pre[:3,3]-=.08*H[:3,2]
            ps=self.ik(pre,gs);detail['checks'].append('pregrasp_ik_collision')
            approach=self.cartesian(ps,H);detail['checks'].append('approach_collision')
            global_plan=self.plan(current,ps);detail['checks'].append('global_pregrasp_plan')
            self.attach_actual(H,O,hypothetical=True)
            width=metric['mesh_hand_geometry']['contact_width_m']
            for name in ['fr3_finger_joint1','fr3_finger_joint2']:
                gs.joint_state.position[gs.joint_state.name.index(name)]=min(.04,width/2)
            micro=H.copy();micro[2,3]+=.005;self.support_contact(True)
            mt=self.cartesian(gs,micro);self.support_contact(False)
            lift=H.copy();lift[2,3]+=.105;self.cartesian(self.end_state(gs,mt),lift)
            detail['checks'].append('attached_micro_lift')
            return detail,dict(pre=pre,grasp=H,global_plan=global_plan,approach=approach)
        except Failure as e:
            detail.update(category=e.category,error=str(e));return detail,None
        finally:
            detail['planning_seconds']=time.monotonic()-started
            self.reset_scene_at(O)

    def prepare(self,O,parent):
        token=f'{TARGET}_{time.time_ns()}'
        parent_path=self.output/f'.{token}_parent.json';target_path=self.output/f'.{token}_target.json';output_path=self.output/f'.{token}_refinements.json'
        parent_path.write_text(json.dumps(dict(T_B_TCP=parent.tolist())))
        target_path.write_text(json.dumps(dict(T_B_target=O.tolist())))
        env={**os.environ,'PYTHONPATH':str(ROOT/'src'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
        try:
            subprocess.run(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'src/grasp_refinement.py'),'--target',TARGET,'--parent',str(parent_path),'--target-pose',str(target_path),'--output',str(output_path)],env=env,check=True,timeout=180)
            refinements=json.loads(output_path.read_text())
        finally:
            for path in [parent_path,target_path,output_path]:path.unlink(missing_ok=True)
        by_id={x['refinement_id']:x for x in refinements['candidates']}
        queue=[0]+[x for x in refinements['moveit_queue'] if x!=0]
        checked=[];plans=[]
        for rid in queue[:MOVEIT_LIMIT]:
            metric=by_id[rid]
            if not metric['geometry_passed']:
                checked.append(dict(refinement_id=rid,category='INSUFFICIENT_PAD_OVERLAP'));continue
            detail,plan=self.plan_candidate(np.asarray(metric['T_B_TCP']),O,metric)
            checked.append(dict(refinement_id=rid,**detail))
            if plan is not None:plans.append((metric,plan))
        return refinements,checked,plans

    def execute_attempt(self,metric,plan,index):
        record=dict(attempt=index,refinement_id=metric['refinement_id'],
                    offset_translation_TCP_m=metric['offset_translation_TCP_m'],
                    offset_rotation_TCP_deg=metric['offset_rotation_TCP_deg'],
                    torque=metric['torque'],mesh_hand_geometry=metric['mesh_hand_geometry'])
        self.reset_scene_at(transform(plant.state()['box'],plant.state()['box_quat']))
        self.phase('PREGRASP');self.execute(plan['global_plan'],'NO_PLAN')
        self.phase('APPROACH');self.execute(self.cartesian(self.measured(),plan['grasp']),'APPROACH_FAIL')
        self.phase('CLOSE');plant.command(dict(op='arm_gain_scale',scale=2.));self.close_force();plant.settle(.5)
        s=plant.state();recent=[x for x in s['history'] if x['t']>=s['t']-.2]
        if not recent or not all(min(x['forces'])>.1 for x in recent):raise Failure('BAD_CONTACT','no sustained bilateral contact')
        H=transform(s['tcp'],s['tcp_quat']);O=transform(s['box'],s['box_quat']);self.attach_actual(H,O)
        plant.command(dict(op='calibration_gate_start'));self.phase('MICRO_LIFT');gate_t=plant.state()['t']
        goal=H.copy();goal[2,3]+=.005;self.support_contact(True);self.execute(self.cartesian(self.measured(),goal),'APPROACH_FAIL')
        plant.settle(.2);old=plant.command(dict(op='calibration_gate_finish'))['gate'];record['old_gate']=old
        start=plant.state()['t'];stable_since=None;observations=[];decision=None
        while plant.state()['t']-start<=1.0:
            elapsed=plant.state()['t']-start
            if elapsed>=.3:
                answer=self.stability(gate_t,old)
                observations.append(dict(t=plant.state()['t'],result=answer))
                stable_since=plant.state()['t'] if answer['passed'] and stable_since is None else stable_since if answer['passed'] else None
                if elapsed>=.5 and stable_since is not None and plant.state()['t']-stable_since>=.3:
                    decision=answer;break
            plant.settle(.05)
        if decision is None:
            decision=observations[-1]['result'];decision=dict(decision);decision['passed']=False
        # A converged object may initially lag the TCP while it rolls off the
        # table.  Give that case one measured 5 mm / >=0.5 s transport probe.
        # Continuous motion, excessive cumulative drift, or contact loss never
        # receives this extension.
        metrics=decision['metrics'];window=metrics['windows']['200ms']
        probe_eligible=(not decision['passed'] and metrics['terminal_bilateral'] and
            metrics['max_contact_gap_s']<=.05 and metrics['max_cumulative_translation_m']<=.010 and
            metrics['max_cumulative_rotation_deg']<=15 and
            window['relative_translation_velocity_m_s']<=self.thresholds.translation_velocity_m_s and
            window['relative_angular_velocity_deg_s']<=self.thresholds.angular_velocity_deg_s)
        if probe_eligible:
            probe_t=plant.state()['t'];state=plant.state();current=transform(state['tcp'],state['tcp_quat']);probe=current.copy();probe[2,3]+=.005
            self.phase('MICRO_LIFT');self.execute_slow(self.cartesian(self.measured(),probe),'APPROACH_FAIL',.5);plant.settle(.3)
            probe_result=self.stability(probe_t,old)
            record['transport_probe']=dict(start_t=probe_t,result=probe_result)
            decision=probe_result
        record.update(stability=decision,stability_observations=observations)
        return record,decision['passed']

    def safe_release(self,record):
        s=plant.state();H=transform(s['tcp'],s['tcp_quat']);O=transform(s['box'],s['box_quat'])
        vertices=VERTICES@O[:3,:3].T+O[:3,3];delta=.0005-float(vertices[:,2].min())
        goal=H.copy();goal[2,3]+=min(0.,delta)
        if abs(goal[2,3]-H[2,3])>.0005:
            self.support_contact(True);self.phase('REGRASP_LOWER');self.execute(self.cartesian(self.measured(),goal),'NO_PLAN')
        plant.settle(.3);self.phase('RELEASE');self.open_hand();plant.settle(.3)
        s=plant.state();O=transform(s['box'],s['box_quat']);self.reset_scene_at(O);record['release_target_pose']=O.tolist()
        # Once the hand is open the target is restored as a world collision
        # object.  Retreat through an open-hand Cartesian segment before
        # evaluating another grasp; otherwise the robot remains at the former
        # closed grasp pose and MoveIt quite correctly reports a world
        # collision at the start of the next regrasp.
        self.target_policy(TOUCH)
        H=transform(s['tcp'],s['tcp_quat']);H[:3,3]-=.08*H[:3,2]
        self.phase('REGRASP_RETREAT');self.execute(self.cartesian(self.measured(),H),'NO_PLAN')
        self.reset_scene_at(O)
        return O

    def final_lift(self,initial_z):
        s=plant.state();H=transform(s['tcp'],s['tcp_quat']);O=transform(s['box'],s['box_quat']);self.attach_actual(H,O)
        self.support_contact(False);goal=H.copy();goal[2,3]+=.10
        self.phase('LIFT');self.execute(self.cartesian(self.measured(),goal),'APPROACH_FAIL')
        self.phase('HOLD');start=plant.state()['t'];plant.settle(2.2);s=plant.state();samples=[x for x in s['history'] if start<=x['t']<=start+2.1]
        heights=[x['z']-initial_z for x in samples];duration=samples[-1]['t']-samples[0]['t'] if samples else 0
        if duration<2 or max(heights,default=0)<.08 or min(heights,default=0)<.08:raise Failure('DROP','height/hold criterion failed')
        if max(heights)-min(heights)>.01:raise Failure('CONTINUOUS_SLIP','hold height range exceeded')
        if not all(min(x['forces'])>.1 for x in samples):raise Failure('CONTACT_LOSS','bilateral hold contact lost')
        return dict(duration_s=duration,min_lift_m=min(heights),max_lift_m=max(heights),samples=samples)

    def episode(self,scene_seed,repeat,mode):
        label=f'{TARGET}_seed{scene_seed}_r{repeat}_{mode}';path=self.output/f'{label}.json'
        if path.exists():
            previous=json.loads(path.read_text())
            if previous.get('category')!='SYSTEM_ERROR':return previous
            aborted=self.output/'aborted_infrastructure';aborted.mkdir(exist_ok=True)
            path.replace(aborted/f'{label}_{time.time_ns()}.json')
        result=dict(id=label,target=TARGET,scene_seed=scene_seed,physics_repeat=repeat,mode=mode,
                    force_total_N=30.,mu=.7,success=False,category='SYSTEM_ERROR',attempts=[])
        tracepath=self.output/f'{label}.trace.json.gz';wall=time.monotonic();self.executions=[]
        try:
            plant.command(dict(op='reset',seed=20260911+repeat));plant.settle(.8);plant.command(dict(op='calibration_material',mu=.7));self.open_hand();plant.settle(.2)
            initial=plant.state();result['initial_target']=dict(position=initial['box'],quaternion_wxyz=initial['box_quat']);initial_z=initial['box'][2]
            O=transform(initial['box'],initial['box_quat'])
            base=json.loads((ROOT/f'results/hand_calibration/gt_path_{TARGET}.json').read_text())
            parent=O@np.linalg.inv(np.asarray(base['T_B_target']))@np.asarray(base['T_B_TCP'])
            result['parent_T_B_TCP']=parent.tolist();result['parent_definition']='frozen GT target-relative grasp'
            plant.command(dict(op='trace_start',path=str(tracepath)))
            refinements,checks,plans=self.prepare(O,parent);result['refinement_protocol']=refinements;result['moveit_checks']=checks
            if not plans:raise Failure('NO_PLAN','no refinement passed full MoveIt checks')
            selected=plans[:1] if mode=='A' else plans[:3]
            for i,(metric,plan) in enumerate(selected,1):
                # Re-express the frozen target-relative refinement at the actual
                # object pose after a physical release, then re-plan from current.
                if i>1:
                    actual=transform(plant.state()['box'],plant.state()['box_quat'])
                    relative=np.linalg.inv(O)@np.asarray(metric['T_B_TCP']);H=actual@relative
                    detail,plan=self.plan_candidate(H,actual,metric);result['moveit_checks'].append(dict(retry=i,refinement_id=metric['refinement_id'],**detail))
                    if plan is None:continue
                record,stable=self.execute_attempt(metric,plan,i);result['attempts'].append(record)
                if stable:
                    result['hold']=self.final_lift(initial_z);result.update(success=True,category='SUCCESS',successful_attempt=i,selected_refinement_id=metric['refinement_id']);break
                if i<len(selected):
                    if record['stability']['category'] in ['CONTACT_LOSS','DROP']:raise Failure(record['stability']['category'],'unsafe to regrasp')
                    self.safe_release(record)
            if not result['success']:
                result['category']=result['attempts'][-1]['stability']['category'] if result['attempts'] else 'NO_PLAN'
        except Failure as e:result.update(category=e.category,detail=str(e))
        except Exception as e:result.update(category='SYSTEM_ERROR',detail=str(e),traceback=traceback.format_exc())
        finally:
            result.update(wall_seconds=time.monotonic()-wall,executions=self.executions,last_stage=self.stage)
            try:result['trace']=plant.command(dict(op='trace_stop'))
            except Exception as e:result['trace_error']=str(e)
            path.write_text(json.dumps(result,indent=2));print(json.dumps({k:result.get(k) for k in ['id','success','category','successful_attempt','wall_seconds']}),flush=True)
        return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--scene-seed',type=int,required=True);p.add_argument('--repeats',type=int,default=3);p.add_argument('--output',required=True);p.add_argument('--thresholds',required=True);a=p.parse_args()
    thresholds=GateThresholds(**json.loads(Path(a.thresholds).read_text()));rclpy.init();node=StabilityBackend(a.output,thresholds)
    try:
        for repeat in range(a.repeats):
            order=['A','B'] if repeat%2==0 else ['B','A']
            for mode in order:
                result=node.episode(a.scene_seed,repeat,mode)
                if result['category']=='SYSTEM_ERROR':raise RuntimeError(result['detail'])
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
