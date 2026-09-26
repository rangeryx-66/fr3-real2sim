"""Frozen executor comparison on held-out geometry with measured support state."""
from __future__ import annotations
import argparse,copy,hashlib,json,os,subprocess,time,traceback
from pathlib import Path
import numpy as np
import rclpy
from stability_backend import StabilityBackend,ROOT,TARGET,VERTICES,INVENTORY
from backend import Failure,TOUCH
from clutter_backend import transform,collision_box
from frames import grasp_to_tcp
from settling_gate import GateThresholds
from support_aware import CONFIG
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import plant

PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
ANYGRASP_ATTEMPT_BUDGET=3
FAMILY_FALLBACK_ATTEMPT_BUDGET=2

class UnseenBackend(StabilityBackend):
    def __init__(self, output, thresholds):
        super().__init__(output, thresholds)
        # Calibration-only knobs.  The frozen grasp executor does not import
        # this backend; keeping them here makes force/friction replay explicit
        # and leaves AnyGrasp/MoveIt selection unchanged.
        self.calibration_force = float(os.environ.get('CALIBRATION_FORCE_N', '30.0'))
        self.calibration_mu = float(os.environ.get('CALIBRATION_MU', '.7'))
        if not (0.0 < self.calibration_force <= 60.0):
            raise ValueError('CALIBRATION_FORCE_N must be in (0,60]')
        if not (0.0 < self.calibration_mu <= 4.0):
            raise ValueError('CALIBRATION_MU must be in (0,4.0]')

    def phase(self,name):super().phase(name)

    def execute(self,trajectory,category):
        try:super().execute(trajectory,category)
        except Failure as e:
            if 'SUPPORT_CONTACT_LOSS' in str(e):raise Failure('CONTACT_LOSS',str(e)) from e
            raise

    def target_policy(self,allowed=()):
        super().target_policy(allowed)
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm=self.call('get_planning_scene',req).scene.allowed_collision_matrix
        for i,name in enumerate(acm.default_entry_names):
            if name=='box':acm.default_entry_values[i]=False
        request=ApplyPlanningScene.Request();request.scene.is_diff=True;request.scene.allowed_collision_matrix=acm;self.apply(request)

    def reset_scene_at(self,O):
        super().reset_scene_at(O)
        s=plant.state();self.obstacles=(s.get('clutter') or {}).get('obstacles',[])
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        for ob in self.obstacles:
            H=np.eye(4);H[:3,3]=ob['position']
            req.scene.world.collision_objects.append(collision_box(ob['id'],np.asarray(ob['aabb_size'])+.002,H))
        if self.obstacles:self.apply(req)
        self.support_contact(False)

    def geometry_worker(self,request,label):
        inp=self.output/(label+'.request.json');out=self.output/(label+'.geometry.json')
        inp.write_text(json.dumps(dict(target=TARGET,**request)))
        env={**os.environ,'PYTHONPATH':str(ROOT/'src'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
        with (self.output/(label+'.geometry.log')).open('w') as log:
            subprocess.run([PY,str(ROOT/'calibration/unseen_geometry.py'),'--request',str(inp),'--output',str(out)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
        return json.loads(out.read_text())

    def plan_candidate(self,H,O,metric):
        start=time.monotonic();detail=dict(checks=[]);self.reset_scene_at(O)
        try:
            current=self.measured();gs=self.ik(H,current);detail['checks'].extend(['IK','JOINT_LIMIT','SELF_COLLISION','STRICT_OPEN_HAND_TARGET_TABLE_SCENE'])
            pre=H.copy();pre[:3,3]-=.08*H[:3,2];ps=self.ik(pre,gs)
            approach=self.cartesian(ps,H);global_plan=self.plan(current,ps)
            detail['checks'].extend(['PREGRASP','CARTESIAN_APPROACH','GLOBAL_PLAN'])
            if not metric['geometry_passed']:raise Failure('INSUFFICIENT_PAD_OVERLAP',str(metric['mesh_hand_geometry']['reasons']))
            detail['checks'].append('FROZEN_MESH_HAND_GATE')
            self.attach_actual(H,O,hypothetical=True)
            width=metric['mesh_hand_geometry']['contact_width_m']
            for name in ['fr3_finger_joint1','fr3_finger_joint2']:
                gs.joint_state.position[gs.joint_state.name.index(name)]=min(.04,width/2)
            self.support_contact(True);micro=H.copy();micro[2,3]+=.005;mt=self.cartesian(gs,micro)
            self.support_contact(False);lift=H.copy();lift[2,3]+=.105
            self.cartesian(self.end_state(gs,mt),lift);detail['checks'].append('ATTACHED_MICRO_AND_LIFT')
            detail['category']='VALID'
            return detail,dict(pre=pre,grasp=H,global_plan=global_plan,approach=approach)
        except Failure as e:
            detail.update(category=e.category,detail=str(e),contacts=self.last_contacts)
            return detail,None
        finally:
            detail['planning_seconds']=time.monotonic()-start;self.planning_seconds+=detail['planning_seconds']
            self.reset_scene_at(O)

    def support(self,since_t=None):
        answer=plant.command(dict(op='support_query',since_t=since_t))
        if not answer.get('ok'):raise Failure('SYSTEM_ERROR',str(answer))
        return answer['support']

    def update_and_step(self,dz,category='LIFT_PATH_FAIL'):
        s=plant.state();H=transform(s['tcp'],s['tcp_quat']);O=transform(s['box'],s['box_quat'])
        query=self.support();self.attach_actual(H,O);self.support_contact(not query['currently_clear'])
        goal=H.copy();goal[2,3]+=dz;t=time.monotonic()
        trajectory=self.cartesian(self.measured(),goal);self.planning_seconds+=time.monotonic()-t
        self.execute_slow(trajectory,category,abs(dz)/CONFIG['lift_speed_m_s'])
        return dict(T_TCP_target=(np.linalg.inv(H)@O).tolist(),goal_TCP=goal.tolist())

    def wait_free_stability(self,since_t=None):
        start=plant.state()['t'];answer=None
        while plant.state()['t']-start<=CONFIG['settle_max_s']:
            plant.settle(.1)
            if plant.state()['t']-start<CONFIG['settle_min_s']:continue
            answer=self.support(since_t)
            if answer['passed'] or answer['category'] in ['DROP','CONTACT_LOSS']:break
        return answer or self.support(since_t)

    def lift_closed_loop(self,record):
        s=plant.state();start_tcp_z=s['tcp'][2]
        plant.command(dict(op='support_begin',initial_z=self.initial_z))
        record['support_observations']=[];free_gate_done=False
        for step in range(150):
            status=self.support();record['support_observations'].append(status)
            if status['category'] in ['DROP','CONTACT_LOSS']:return status
            if status['clear_t'] is None:free_gate_done=False
            if status['currently_clear'] and status['clear_t'] is not None and not free_gate_done:
                self.phase('FREE_SPACE_SETTLE');status=self.wait_free_stability()
                record['support_observations'].append(status)
                if status['category']=='SUPPORTED_SETTLING':
                    free_gate_done=False;continue
                if not status['passed']:return status
                free_gate_done=True
            if free_gate_done and status['net_lift_m']>=CONFIG['lift_goal_m']:
                self.phase('HOLD');begin=plant.state()['t'];plant.settle(CONFIG['hold_s'])
                outcome=plant.command(dict(op='support_hold',since_t=begin))['outcome']
                record['hold']=outcome
                return {**outcome['stability'],'passed':outcome['success'],'category':outcome['category']}
            current_z=plant.state()['tcp'][2];remaining=CONFIG['max_tcp_travel_m']-(current_z-start_tcp_z)
            if remaining<.0005:return {**self.support(),'passed':False,'category':'INSUFFICIENT_LIFT'}
            before_t=plant.state()['t'];self.phase('LIFT' if free_gate_done else 'SUPPORTED_MICRO_LIFT')
            record.setdefault('attachments',[]).append(self.update_and_step(min(CONFIG['lift_step_m'],remaining)))
            status=self.support();record['support_observations'].append(status)
            if status['category'] in ['DROP','CONTACT_LOSS']:return status
            if status['clear_t'] is None:free_gate_done=False
            if free_gate_done:
                self.phase('FREE_SPACE_SETTLE');status=self.wait_free_stability(before_t)
                record['support_observations'].append(status)
                if status['category']=='SUPPORTED_SETTLING':
                    free_gate_done=False;continue
                if not status['passed']:return status
        raise Failure('TRACKING_ERROR','height loop failed to reach goal or actual TCP travel limit within the watchdog bound')

    def safe_release(self,record):
        plant.command(dict(op='support_pause'))
        state=plant.state()
        if max(state['forces'])<=.1:raise Failure('CONTACT_LOSS','both finger contacts lost; no retry')
        if not .15<state['box'][0]<.85 or abs(state['box'][1])>.40:raise Failure('DROP','target outside table safety region')
        self.phase('REGRASP_LOWER')
        for _ in range(52):
            state=plant.state();H=transform(state['tcp'],state['tcp_quat']);O=transform(state['box'],state['box_quat'])
            bottom=float((VERTICES@O[:3,:3].T+O[:3,3])[:,2].min())
            if bottom<=.001:break
            self.attach_actual(H,O);self.support_contact(True)
            goal=H.copy();dz=min(.005,bottom-.0005);goal[2,3]-=dz
            self.execute_slow(self.cartesian(self.measured(),goal),'NO_PLAN',dz/.01)
        else:raise Failure('NO_PLAN','could not return target to table within safe travel')
        plant.settle(.3);self.phase('RELEASE');self.open_hand();plant.settle(.3)
        state=plant.state();O=transform(state['box'],state['box_quat']);self.reset_scene_at(O)
        record['release_target_pose']=O.tolist()
        # Retreat through collision-checked open-hand space before another grasp.
        # The first open-hand sample can still overlap a finger pad by a few
        # millimetres.  Temporarily allow only the measured touch links for
        # this retreat, then restore strict world collision checking before
        # evaluating the next candidate.  This avoids treating a stale
        # release pose as a candidate failure while keeping the retreat itself
        # physically real and collision checked for all other links.
        self.target_policy(TOUCH)
        H=transform(state['tcp'],state['tcp_quat']);H[:3,3]-=.08*H[:3,2]
        self.execute(self.cartesian(self.measured(),H),'NO_PLAN')
        self.reset_scene_at(O)
        return O

    def attempt(self,metric,plan,record):
        state=plant.state();self.reset_scene_at(transform(state['box'],state['box_quat']))
        self.phase('PREGRASP');self.execute(plan['global_plan'],'NO_PLAN')
        self.phase('APPROACH');self.execute(self.cartesian(self.measured(),plan['grasp']),'APPROACH_FAIL')
        record['actual_grasp_started']=True;self.phase('CLOSE')
        plant.command(dict(op='arm_gain_scale',scale=2.));self.close_force(self.calibration_force);plant.settle(.5)
        state=plant.state();record['after_close']=dict(target=transform(state['box'],state['box_quat']).tolist(),tcp=transform(state['tcp'],state['tcp_quat']).tolist(),forces=state['forces'])
        try:answer=self.lift_closed_loop(record)
        except Failure as e:
            record['stability']={**self.support(),'passed':False,'category':e.category}
            raise
        record['stability']=answer
        return answer

    def episode(self,seed,mode,gt_path=None,family_enabled=False,replay_anygrasp=None,replay_reference=None):
        label=f'{TARGET}_seed{seed}_{mode}';path=self.output/(label+'.json');wall=time.monotonic()
        result=dict(id=label,target=TARGET,seed=seed,mode=mode,success=False,category='SYSTEM_ERROR',attempts=[],candidate_checks=[],selected_rank=None,
                    parameters=dict(force_total_N=self.calibration_force,mu=self.calibration_mu,margin_m=.001,grasp_family_expansion=bool(family_enabled)),flags={})
        self.executions=[];self.planning_seconds=0.;self.result=result
        try:
            plant.command(dict(op='reset',seed=seed));plant.command(dict(op='arm_gain_scale',scale=1.));plant.settle(.8)
            self.open_hand();plant.settle(.2)
            capture=plant.command(dict(op='capture'))
            if not capture.get('ok'):raise Failure('NO_GRASP',str(capture))
            result['capture']=capture;state=plant.state()
            O=transform(state['box'],state['box_quat']);reference=(VERTICES.min(0)+VERTICES.max(0))/2
            self.initial_z=float((O[:3,:3]@reference+O[:3,3])[2])
            result['height_reference_local_m']=reference.tolist();result['initial_height_reference_z_m']=self.initial_z
            result['initial_target']=O.tolist();result['initial_robot_q']=state['q'];result['tf_check']=self.check_fk()
            result['material_audit']=plant.command(dict(op='calibration_audit'))
            if state.get('clutter'):plant.command(dict(op='arm_metrics'))
            self.reset_scene_at(O)
            if gt_path:
                definition=json.loads(Path(gt_path).read_text());H=O@np.linalg.inv(np.asarray(definition['T_B_target']))@np.asarray(definition['T_B_TCP'])
                parents=[dict(rank=0,score=None,T_B_TCP=H.tolist())]
                result['proposal_method']='development preflight fixed GT replay'
            elif mode=='GT':
                data=self.geometry_worker(dict(operation='oracle',T_B_target=O.tolist()),label+'_oracle')
                parents=[{k:g[k] for k in ['rank','score','T_B_TCP']} for g in data['grasps']]
                result['proposal_method']=data['method'];result['oracle_counts']={k:v for k,v in data.items() if k!='grasps'}
            else:
                if replay_anygrasp:
                    if not replay_reference:raise Failure('SYSTEM_ERROR','paired AnyGrasp replay requires its reference result')
                    out=Path(replay_anygrasp);reference_result=json.loads(Path(replay_reference).read_text())
                    reference_O=np.asarray(reference_result['initial_target'])
                    result['anygrasp_replay']=dict(source_json=str(out),source_result=str(replay_reference),transport='T_B_target_current * inv(T_B_target_source) * T_B_TCP_source')
                else:
                    out=self.output/(label+'_anygrasp.json')
                    env={**os.environ,'CUDA_VISIBLE_DEVICES':os.environ.get('UNSEEN_GPU','3'),'LD_LIBRARY_PATH':'','PYTHONPATH':'','PATH':'/data1/home/rangeryx/.conda/envs/anygrasp/bin:/usr/local/bin:/usr/bin:/bin','CONDA_PREFIX':'/data1/home/rangeryx/.conda/envs/anygrasp'}
                    with (self.output/(label+'_infer.log')).open('w') as log:
                        subprocess.run(['/data1/home/rangeryx/.conda/envs/anygrasp/bin/python',str(ROOT/'src/infer.py'),'--input',capture['path'],'--output',str(out),'--top-k','20'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
                data=json.loads(out.read_text());result['anygrasp_json_sha256']=hashlib.sha256(out.read_bytes()).hexdigest()
                parents=[]
                for g in data['grasps']:
                    _,H,_=grasp_to_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
                    if replay_anygrasp:H=O@np.linalg.inv(reference_O)@H
                    parents.append(dict(rank=g['rank'],score=g['score'],T_B_TCP=H.tolist(),raw_grasp=g))
            result['candidate_count']=len(parents)
            if not parents and not family_enabled:raise Failure('NO_GRASP','proposal set empty')
            metrics=self.geometry_worker(dict(operation='evaluate',parents=parents,T_B_target=O.tolist()),label+'_parents') if parents else []
            result['mesh_gate_rejected_in_full_proposal_set']=sum(not x['geometry_passed'] for x in metrics)
            for metric in metrics:
                metric.setdefault('source','GT' if mode=='GT' else 'ANYGRASP')
                metric.setdefault('family',None);metric.setdefault('family_rank',None)
            result['candidate_order']=dict(
                family_enabled=bool(family_enabled),
                policy='ANYGRASP_FIRST_THEN_FAMILY_FALLBACK',
                anygrasp_attempt_budget=ANYGRASP_ATTEMPT_BUDGET,
                family_fallback_attempt_budget=FAMILY_FALLBACK_ATTEMPT_BUDGET,
                family_preferred=False,
                anygrasp_order_unchanged=True,
                family_generated_lazily=True,
            )
            tracepath=self.output/(label+'.trace.json.gz');plant.command(dict(op='trace_start',path=str(tracepath)))
            stage='ANYGRASP';stage_metrics=metrics;parent_index=0;queue=[];active_parent=None
            actual_grasps=0;stage_grasps={'ANYGRASP':0,'FAMILY_FALLBACK':0};last_failure=None
            last_failure_by_stage={};pending_release=False
            result['stage_transitions']=[]

            def begin_family_fallback(current_O,reason):
                nonlocal stage,stage_metrics,parent_index,queue,active_parent
                expansion=self.geometry_worker(dict(operation='family',T_B_target=current_O.tolist()),label+'_families')
                result['grasp_family_expansion']={k:v for k,v in expansion.items() if k!='candidates'}
                stage='FAMILY_FALLBACK';stage_metrics=expansion['candidates'];parent_index=0;queue=[];active_parent=None
                for candidate in stage_metrics:candidate['_reference_target_pose']=current_O.tolist()
                result['stage_transitions'].append(dict(
                    from_stage='ANYGRASP',to_stage=stage,reason=reason,
                    anygrasp_physical_attempts=stage_grasps['ANYGRASP'],
                    family_candidate_count=len(stage_metrics),
                ))

            while True:
                budget=ANYGRASP_ATTEMPT_BUDGET if stage=='ANYGRASP' else FAMILY_FALLBACK_ATTEMPT_BUDGET
                if stage_grasps[stage]>=budget:
                    if stage=='ANYGRASP' and family_enabled and last_failure not in ['DROP','CONTACT_LOSS']:
                        if pending_release:
                            current_O=self.safe_release(result['attempts'][-1]);pending_release=False
                        else:
                            state=plant.state();current_O=transform(state['box'],state['box_quat'])
                        begin_family_fallback(current_O,'ANYGRASP_REGRASP_BUDGET_EXHAUSTED')
                        continue
                    break
                state=plant.state();current_O=transform(state['box'],state['box_quat'])
                if not queue:
                    if parent_index>=len(stage_metrics):
                        if stage=='ANYGRASP' and family_enabled and last_failure not in ['DROP','CONTACT_LOSS']:
                            if pending_release:
                                current_O=self.safe_release(result['attempts'][-1]);pending_release=False
                            begin_family_fallback(current_O,'ANYGRASP_CANDIDATES_EXHAUSTED')
                            continue
                        break
                    active_parent=stage_metrics[parent_index];parent_index+=1;queue=[active_parent]
                metric=queue.pop(0);original_H=np.asarray(metric['T_B_TCP'])
                reference_O=np.asarray(metric.get('_reference_target_pose',O))
                H=current_O@np.linalg.inv(reference_O)@original_H
                if metric.get('geometry_pending'):
                    pending={k:v for k,v in metric.items() if k not in ['geometry_pending','_reference_target_pose']}
                    pending['T_B_TCP']=H.tolist()
                    updated=self.geometry_worker(dict(operation='evaluate',T_B_target=current_O.tolist(),parents=[pending]),label+f'_family_exact{len(result["candidate_checks"])}')[0]
                    metric={**metric,**updated,'geometry_pending':False}
                # Re-evaluate actual geometry after physical release; transforms only.
                elif actual_grasps:
                    updated=self.geometry_worker(dict(operation='evaluate',T_B_target=current_O.tolist(),parents=[dict(rank=active_parent.get('rank'),score=active_parent.get('score'),T_B_TCP=H.tolist())]),label+f'_actual{len(result["candidate_checks"])}')[0]
                    metric={**metric,'mesh_hand_geometry':updated['mesh_hand_geometry'],'geometry_passed':updated['geometry_passed'],'torque':updated['torque']}
                detail,plan=self.plan_candidate(H,current_O,metric)
                result['candidate_checks'].append(dict(stage=stage,source=active_parent.get('source'),family=active_parent.get('family'),family_rank=active_parent.get('family_rank'),parent_rank=active_parent.get('rank'),refinement_id=metric['refinement_id'],**detail))
                if plan is None:continue
                record=dict(attempt=actual_grasps+1,stage=stage,stage_attempt=stage_grasps[stage]+1,source=active_parent.get('source'),family=active_parent.get('family'),family_rank=active_parent.get('family_rank'),parent_rank=active_parent.get('rank'),raw_score=active_parent.get('score'),refinement_id=metric['refinement_id'],
                            offset_translation_TCP_m=metric['offset_translation_TCP_m'],offset_rotation_TCP_deg=metric['offset_rotation_TCP_deg'],torque=metric['torque'],
                            family_feasibility=metric.get('family_feasibility'),
                            T_B_TCP_commanded=H.tolist(),actual_grasp_started=False)
                result['attempts'].append(record);result['selected_rank']=active_parent.get('rank');result['selected_family']=active_parent.get('family')
                answer=self.attempt(metric,plan,record);actual_grasps+=1;stage_grasps[stage]+=1
                print('ATTEMPT_COMPLETE',label,actual_grasps,answer['category'],answer.get('net_lift_m'),flush=True)
                (self.output/(label+'.progress.json')).write_text(json.dumps(dict(attempts=result['attempts'],candidate_checks=result['candidate_checks'])))
                if answer['passed']:
                    result.update(success=True,category='SUCCESS',successful_attempt=actual_grasps,flags=answer['flags']);break
                last_failure=answer['category'];last_failure_by_stage[stage]=last_failure;result['flags']=answer['flags']
                pending_release=last_failure not in ['DROP','CONTACT_LOSS']
                if not pending_release:break
                if stage_grasps[stage]>=budget:continue
                released_O=self.safe_release(record);pending_release=False
                # Family generation already spans approach/depth variants. Do
                # not let generic local refinement bypass its retention gate.
                if metric['refinement_id']==0 and stage=='ANYGRASP':
                    parent_tag=active_parent.get('rank') if active_parent.get('rank') is not None else 'family'+str(active_parent.get('family_rank'))
                    released_H=released_O@np.linalg.inv(current_O)@H
                    refinements=self.geometry_worker(dict(operation='refine',T_B_target=released_O.tolist(),T_B_TCP=released_H.tolist()),label+f'_refine_rank{parent_tag}')
                    by_id={x['refinement_id']:x for x in refinements['candidates']}
                    queue=[dict(**by_id[x],source=active_parent.get('source'),family=active_parent.get('family'),family_rank=active_parent.get('family_rank'),rank=active_parent.get('rank'),score=active_parent.get('score'),_reference_target_pose=released_O.tolist()) for x in refinements['moveit_queue'] if x!=0][:12]
            result['attempt_budget_usage']=dict(
                anygrasp=stage_grasps['ANYGRASP'],family_fallback=stage_grasps['FAMILY_FALLBACK'],total=actual_grasps,
                limits=dict(anygrasp=ANYGRASP_ATTEMPT_BUDGET,family_fallback=FAMILY_FALLBACK_ATTEMPT_BUDGET),
            )
            if not result['success']:
                result['last_physical_failure']=last_failure
                result['last_physical_failure_by_stage']=last_failure_by_stage
                if family_enabled and stage=='FAMILY_FALLBACK' and last_failure not in ['DROP','CONTACT_LOSS']:
                    result['category']='NO_STABLE_GRASP'
                    result['candidate_exhaustion_reason']=last_failure or 'NO_EXECUTABLE_CANDIDATE'
                else:result['category']=last_failure or 'NO_EXECUTABLE_CANDIDATE'
        except Failure as e:result.update(category=e.category,detail=str(e))
        except Exception as e:result.update(category='SYSTEM_ERROR',detail=str(e),traceback=traceback.format_exc())
        finally:
            result.update(planning_seconds=self.planning_seconds,wall_seconds=time.monotonic()-wall,executions=self.executions,last_stage=self.stage)
            try:
                # An aborted grasp may remain suspended or subsequently fall.
                # Observe without attempting recovery; early abort is never SUCCESS.
                if not result['success'] and any(x['actual_grasp_started'] for x in result['attempts']):
                    result['termination_reason']=result['category']
                    if plant.state()['busy']:plant.command(dict(op='stop'))
                    self.phase('POST_FAILURE_OBSERVE');begin=plant.state()['t'];plant.settle(2.2)
                    result['post_failure_observation_s']=plant.state()['t']-begin
                state=plant.state();result['final_support']=self.support();result['flags']=result['final_support']['flags']
                if not result['success'] and result['flags']['DROP']:result['category']='DROP'
                for key in ['PICKED','CLEAR_TABLE']:
                    result['flags'][key] |= any(x.get('stability',{}).get('flags',{}).get(key,False) for x in result['attempts'])
                m=(state.get('clutter') or {}).get('metrics',{})
                result.update(non_target_contact=bool(m.get('any_contact',False)),non_target_disturbance=bool(m.get('max_displacement_m',0)>.002 or m.get('max_rotation_rad',0)>np.deg2rad(2)),non_target_metrics=m,non_target_events=(state.get('clutter') or {}).get('events',[]))
                result['trace']=plant.command(dict(op='trace_stop'))
            except Exception as e:result['telemetry_error']=str(e)
            result['scene_filtered_candidates']=sum(x['category']=='SCENE_COLLISION' for x in result['candidate_checks'])
            result['geometry_filtered_candidates']=sum(x['category']=='INSUFFICIENT_PAD_OVERLAP' for x in result['candidate_checks'])
            result['actual_grasp_count']=sum(x['actual_grasp_started'] for x in result['attempts'])
            path.write_text(json.dumps(result,indent=2));print(json.dumps({k:result.get(k) for k in ['id','success','category','actual_grasp_count','wall_seconds','flags']}),flush=True)
        return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);p.add_argument('--mode',choices=['GT','ANYGRASP'],required=True);p.add_argument('--output',required=True);p.add_argument('--gt-path');p.add_argument('--family',action='store_true');p.add_argument('--replay-anygrasp');p.add_argument('--replay-reference');a=p.parse_args()
    rclpy.init();node=UnseenBackend(a.output,GateThresholds())
    try:
        result=node.episode(a.seed,a.mode,a.gt_path,a.family,a.replay_anygrasp,a.replay_reference)
        if result['category']=='SYSTEM_ERROR':raise RuntimeError(result.get('traceback',result))
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
