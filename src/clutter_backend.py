"""Paired clutter experiment; extends the existing MoveIt executor without new planners."""
import argparse
import copy
import hashlib
import json
import os
import subprocess
import time
import traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from backend import Backend, Failure, ROOT, BASE, TCP, GROUP, TOUCH, PROFILE, pose, stamped
from moveit_msgs.srv import GetCartesianPath, GetStateValidity, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import RobotState, CollisionObject, AttachedCollisionObject, AllowedCollisionEntry, PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from frames import grasp_to_robot_tcp
import plant

def transform(p,q_wxyz):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(q_wxyz,-1)).as_matrix();T[:3,3]=p;return T

def collision_box(name,size,T,frame=BASE):
    obj=CollisionObject();obj.id=name;obj.header.frame_id=frame;obj.operation=CollisionObject.ADD
    shape=SolidPrimitive();shape.type=SolidPrimitive.BOX;shape.dimensions=list(map(float,size))
    obj.primitives=[shape];obj.primitive_poses=[pose(T)];return obj

class ClutterBackend(Backend):
    def __init__(self,output):
        super().__init__();self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.mode='B';self.margin=.001;self.attached=None;self.last_contacts=[];self.planning_seconds=0.
        if not self.hand.wait_for_server(timeout_sec=30) or not self.exec_client.wait_for_server(timeout_sec=30):raise RuntimeError('controller action servers not ready')
    def measured(self):
        state=super().measured();state.is_diff=True
        if self.attached is not None:state.attached_collision_objects=[copy.deepcopy(self.attached)]
        return state
    def phase(self,name):
        self.stage=name;plant.command(dict(op='phase',phase=name))
    def validate(self,state):
        self.last_contacts=[]
        try:super().validate(state)
        except Failure as e:
            if 'clutter_' in str(e):
                try:
                    import ast
                    self.last_contacts=ast.literal_eval(str(e))
                except Exception:pass
                raise Failure('SCENE_COLLISION',str(e))
            raise
    def apply(self,request):
        if not self.call('apply_planning_scene',request).success:raise Failure('NO_PLAN','PlanningScene update failed')
    def support_contact(self,allow):
        get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm=self.call('get_planning_scene',get).scene.allowed_collision_matrix
        for name in ['box','table']:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:row.enabled.append(False)
                row=AllowedCollisionEntry();row.enabled=[False]*len(acm.entry_names);acm.entry_values.append(row)
        i=acm.entry_names.index('box');j=acm.entry_names.index('table')
        acm.entry_values[i].enabled[j]=bool(allow);acm.entry_values[j].enabled[i]=bool(allow)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.allowed_collision_matrix=acm;self.apply(req)
    def reset_scene(self):
        self.attached=None;super().scene(attach=False)
        s=plant.state();req=ApplyPlanningScene.Request();req.scene.is_diff=True
        req.scene.world.collision_objects.append(collision_box('box',[.045,.045,.05],transform(s['box'],s['box_quat'])))
        get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        present={o.id for o in self.call('get_planning_scene',get).scene.world.collision_objects}
        for ob in s['clutter']['obstacles']:
            if self.mode=='B':
                T=np.eye(4);T[:3,3]=ob['position']
                req.scene.world.collision_objects.append(collision_box(ob['id'],np.array(ob['aabb_size'])+2*self.margin,T))
            elif ob['id'] in present:
                obj=CollisionObject();obj.id=ob['id'];obj.operation=CollisionObject.REMOVE;req.scene.world.collision_objects.append(obj)
        self.apply(req);self.support_contact(False)
        ids={o.id for o in self.call('get_planning_scene',get).scene.world.collision_objects}
        expected={o['id'] for o in s['clutter']['obstacles']} if self.mode=='B' else set()
        if {x for x in ids if x.startswith('clutter_')}!=expected:raise Failure('TF_ERROR','A/B scene membership mismatch')
        self.scene_ids=sorted(ids)
    def attach(self,T_B_H,T_B_O):
        att=AttachedCollisionObject();att.link_name=TCP;att.touch_links=TOUCH
        att.object=collision_box('box',[.045,.045,.05],np.linalg.inv(T_B_H)@T_B_O,TCP)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;req.scene.robot_state.attached_collision_objects=[att]
        self.apply(req);self.attached=att
        get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        attached=self.call('get_planning_scene',get).scene.robot_state.attached_collision_objects
        if not any(x.object.id=='box' and x.link_name==TCP for x in attached):raise Failure('TF_ERROR','attached target missing from scene')
        return att
    def with_attachment(self,state):
        state=copy.deepcopy(state);state.is_diff=True
        state.attached_collision_objects=[copy.deepcopy(self.attached)] if self.attached is not None else []
        return state
    def end_state(self,start,traj):
        s=copy.deepcopy(start)
        for name,q in zip(traj.joint_trajectory.joint_names,traj.joint_trajectory.points[-1].positions):
            s.joint_state.position[s.joint_state.name.index(name)]=q
        return self.with_attachment(s)
    def cartesian(self,start,T):
        start=self.with_attachment(start)
        try:return super().cartesian(start,T)
        except Failure as failure:
            # Recompute the same Cartesian interpolation without collision filtering,
            # then ask MoveIt which sampled state collides. This sends no robot action.
            req=GetCartesianPath.Request();req.header.frame_id=BASE;req.start_state=start;req.group_name=GROUP;req.link_name=TCP;req.waypoints=[pose(T)];req.max_step=.003;req.jump_threshold=1.5;req.avoid_collisions=False
            raw=self.call('compute_cartesian_path',req)
            for point in raw.solution.joint_trajectory.points:
                candidate=copy.deepcopy(start)
                for name,q in zip(raw.solution.joint_trajectory.joint_names,point.positions):candidate.joint_state.position[candidate.joint_state.name.index(name)]=q
                self.validate(candidate)
            raise failure
    def candidate(self,g,data):
        detail=dict(rank=g['rank'],score=g['score'],checks=[],stage='GRASP_IK')
        started=time.monotonic();self.last_contacts=[]
        try:
            if not 0<g['width']<=PROFILE.open_width_m:raise Failure('GRIPPER_WIDTH','out of range')
            pre,grasp,lift=grasp_to_robot_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'],PROFILE.grasp_tip_offset_m)
            current=self.measured();gs=self.ik(grasp,current)
            detail['checks']+=['grasp_ik','joint_limits','self_collision','robot_world_collision']
            detail['stage']='PREGRASP';ps=self.ik(pre,gs);detail['checks'].append('pregrasp_ik_collision')
            detail['stage']='APPROACH';self.cartesian(ps,grasp);detail['checks'].append('approach')
            detail['stage']='GLOBAL_PLAN';traj=self.plan(current,ps);detail['checks'].append('global_pregrasp_plan')
            detail['stage']='ATTACHED_LIFT'
            T_B_O=transform(self.initial['box'],self.initial['box_quat'])
            self.attach(grasp,T_B_O)
            width=float(np.sum(np.abs(grasp[:3,1]@T_B_O[:3,:3])*[.045,.045,.05]))
            from robot_profile import gripper_positions
            for name,value in gripper_positions(PROFILE,min(PROFILE.open_width_m,width)).items():
                if name in gs.joint_state.name:gs.joint_state.position[gs.joint_state.name.index(name)]=value
            gs=self.with_attachment(gs);micro=grasp.copy();micro[2,3]+=.005
            self.support_contact(True)
            mt=self.cartesian(gs,micro)
            self.support_contact(False)
            self.cartesian(self.end_state(gs,mt),lift)
            detail['checks'].append('attached_micro_and_lift');detail['status']='VALID'
            return detail,(pre,grasp,lift,traj,g)
        except Failure as e:
            detail.update(status=e.category,detail=str(e),contacts=self.last_contacts,attached_target_collision=any('box' in pair and any('clutter_' in x for x in pair) for pair in self.last_contacts))
            return detail,None
        except ValueError as e:
            detail.update(status='TF_ERROR',detail=str(e));return detail,None
        finally:
            self.reset_scene()
            detail['planning_seconds']=time.monotonic()-started;self.planning_seconds+=detail['planning_seconds']
    def perception(self,seed):
        cache=self.output/'inputs';cache.mkdir(exist_ok=True)
        out=cache/f'seed_{seed:04d}_grasps.json'
        if out.exists():return json.loads(out.read_text()),str(out)
        capture=plant.command(dict(op='capture'))
        if not capture['ok'] or capture.get('object_points',0)<30:raise Failure('NO_GRASP',str(capture))
        cmd=['/data1/home/rangeryx/.conda/envs/anygrasp/bin/python',str(ROOT/'src/infer.py'),'--input',capture['path'],'--output',str(out),'--top-k','20']
        env={**os.environ,'CUDA_VISIBLE_DEVICES':'1','LD_LIBRARY_PATH':'','PYTHONPATH':'','PATH':'/data1/home/rangeryx/.conda/envs/anygrasp/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin','CONDA_PREFIX':'/data1/home/rangeryx/.conda/envs/anygrasp'}
        with open(cache/f'seed_{seed:04d}_infer.log','w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=180,env=env)
        if r.returncode:raise Failure('NO_GRASP','SDK subprocess failed')
        (cache/f'seed_{seed:04d}_capture.json').write_text(json.dumps(capture,indent=2))
        return json.loads(out.read_text()),str(out)
    def episode(self,seed,mode):
        self.mode=mode;self.executions=[];self.stage='RESET';self.planning_seconds=0.;wall=time.monotonic()
        result=dict(seed=seed,mode=mode,success=False,lift_ge_8cm=False,stable_hold_ge_2s=False,selected_rank=None,candidates=[],category='SYSTEM_ERROR')
        try:
            plant.command(dict(op='reset',seed=seed));plant.settle(1.)
            self.attached=None;result['tf_check']=self.check_fk()
            self.reset_scene();self.gripper(PROFILE.open_width_m);plant.settle(.3)
            self.initial=plant.state();result['planning_scene_world_ids']=self.scene_ids;result['initial_layout']=self.initial['clutter']['obstacles'];result['initial_target']=dict(position=self.initial['box'],quaternion_wxyz=self.initial['box_quat'])
            plant.command(dict(op='arm_metrics'));self.phase('PERCEPTION')
            data,path=self.perception(seed);result['grasp_input']=path;result['grasp_sha256']=hashlib.sha256(Path(path).read_bytes()).hexdigest()
            if not data['grasps']:raise Failure('NO_GRASP','zero candidates')
            self.phase('CANDIDATE_CHECK');chosen=None
            candidates=data['grasps'][:1] if mode=='A' else data['grasps'][:20]
            for g in candidates:
                detail,plan=self.candidate(g,data);result['candidates'].append(detail)
                if chosen is None and plan is not None:chosen=plan
            result['scene_filtered_candidates']=sum(c['status']=='SCENE_COLLISION' for c in result['candidates'])
            if chosen is None:
                category='SCENE_COLLISION' if result['scene_filtered_candidates'] else result['candidates'][0]['status']
                raise Failure(category,'no executable candidate in configured candidate set')
            pre,grasp,lift,traj,g=chosen;result['selected_rank']=g['rank'];result['selected_score']=g['score']
            self.phase('PREGRASP');self.execute(traj,'NO_PLAN')
            self.phase('APPROACH');t=time.monotonic();at=self.cartesian(self.measured(),grasp);self.planning_seconds+=time.monotonic()-t;self.execute(at,'APPROACH_FAIL')
            self.phase('CLOSE');self.gripper(0.);plant.settle(.3)
            s=plant.state();recent=[h for h in s['history'] if h['t']>=s['t']-.2];result['close_samples']=recent
            if not recent or not all(min(h['forces'])>.1 for h in recent):raise Failure('BAD_CONTACT','no sustained bilateral contact')
            T_B_H=transform(s['tcp'],s['tcp_quat']);T_B_O=transform(s['box'],s['box_quat'])
            t=time.monotonic();self.attach(T_B_H,T_B_O);result['T_TCP_target']=(np.linalg.inv(T_B_H)@T_B_O).tolist()
            self.phase('MICRO_LIFT');micro=T_B_H.copy();micro[2,3]+=.005;self.support_contact(True);mt=self.cartesian(self.measured(),micro);self.planning_seconds+=time.monotonic()-t;self.execute(mt,'APPROACH_FAIL')
            self.phase('LIFT');t=time.monotonic();self.support_contact(False);lt=self.cartesian(self.measured(),lift);self.planning_seconds+=time.monotonic()-t;self.execute(lt,'APPROACH_FAIL')
            self.phase('HOLD');start=plant.state()['t'];plant.settle(2.2);s=plant.state()
            samples=[h for h in s['history'] if start<=h['t']<=start+2.1];result['hold_samples']=samples
            z0=self.initial['box'][2];result['lift_ge_8cm']=s['clutter']['metrics']['max_target_lift_m']>=.08
            if len(samples)<2 or samples[-1]['t']-samples[0]['t']<2:raise Failure('DROP','insufficient hold samples')
            heights=[h['z'] for h in samples]
            if max(heights)<z0+.08:raise Failure('BAD_CONTACT','target not lifted 8 cm')
            if min(heights)<z0+.08:raise Failure('DROP','target lost required lift height')
            if max(heights)-min(heights)>.01 or not all(min(h['forces'])>.1 for h in samples):raise Failure('SLIP','unstable hold')
            result.update(success=True,stable_hold_ge_2s=True,category='SUCCESS')
        except Failure as e:result.update(category=e.category,detail=str(e))
        except Exception as e:result.update(category='SYSTEM_ERROR',detail=str(e),traceback=traceback.format_exc())
        try:
            s=plant.state();result['physics']=s['clutter'];m=s['clutter']['metrics']
            result.update(non_target_contact=bool(m.get('any_contact',False)),non_target_disturbance=bool(m.get('max_displacement_m',0)>.002 or m.get('max_rotation_rad',0)>np.deg2rad(2)),max_displacement_m=m.get('max_displacement_m',0),max_rotation_deg=float(np.rad2deg(m.get('max_rotation_rad',0))),lift_ge_8cm=m.get('max_target_lift_m',0)>=.08)
            if result['non_target_contact'] and not result['success'] and result['category'] in ['NO_PLAN','APPROACH_FAIL']:result['underlying_failure']=result['category'];result['category']='SCENE_COLLISION'
        except Exception:pass
        result.update(planning_seconds=self.planning_seconds,wall_seconds=time.monotonic()-wall,last_stage=self.stage,executions=self.executions)
        (self.output/f'{mode}_seed_{seed:04d}.json').write_text(json.dumps(result,indent=2))
        compact={k:result.get(k) for k in ['seed','mode','success','category','last_stage','selected_rank','scene_filtered_candidates','non_target_contact','non_target_disturbance','max_displacement_m','max_rotation_deg','planning_seconds','wall_seconds','detail']}
        print(json.dumps(compact),flush=True)
        return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seeds',type=int,default=20);p.add_argument('--start-seed',type=int,default=0);p.add_argument('--mode',choices=['A','B','both'],default='both');a=p.parse_args()
    rclpy.init();node=ClutterBackend(a.output)
    for seed in range(a.start_seed,a.start_seed+a.seeds):
        order=['A','B'] if seed%2==0 else ['B','A']
        for mode in order if a.mode=='both' else [a.mode]:
            if (node.output/f'{mode}_seed_{seed:04d}.json').exists():continue
            result=node.episode(seed,mode)
            if result['category']=='SYSTEM_ERROR':raise RuntimeError(result['detail'])
    node.destroy_node();rclpy.shutdown()
