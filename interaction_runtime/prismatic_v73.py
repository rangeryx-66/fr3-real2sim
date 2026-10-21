"""Isolated prismatic development: observed recovery and predictive feasibility.

Frozen revolute sources, closure, drives, safety and SG121 math are inherited.
No benchmark entry, GT grasp, integral IK or retention prototype is enabled.
"""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
from agent_command_protocol_v12 import atomic_json,receive,complete
from .prismatic_preload_v71 import PrismaticPreloadRuntime,PreloadOperations
from .prismatic_planning_v73 import PrismaticCandidateCheck,closing_envelope,predicted_aperture,reachable_interval,representable_joint_bounds,native_precision_ik_class
from .serialization import result_packet
from .policy import SafetyStop
from .segments import ExecutionBudgetExhausted,validate_action


def audit_frozen(root):
    root=Path(root);out={}
    for file,key in [('REVOLUTE_FREEZE.json','source_sha256'),('REVOLUTE_EXTERNAL_FREEZE.json','external_robot_camera_closure_uncertainty_sha256')]:
        m=json.loads((root/'docs/experiments/prismatic_transmission_v7_20261020'/file).read_text())
        bad=[p for p,h in m[key].items() if not (Path(p) if Path(p).is_absolute() else root/p).exists() or hashlib.sha256((Path(p) if Path(p).is_absolute() else root/p).read_bytes()).hexdigest()!=h]
        out[file]=dict(count=len(m[key]),mismatch=bad)
    if any(a['mismatch'] for a in out.values()):raise RuntimeError('FROZEN_REVOLUTE_DRIFT:'+str(out))
    return out


class PrismaticRuntimeV73(PrismaticPreloadRuntime):
    def __init__(self,job):
        if job.get('run_classification')!='PRISMATIC_ENGINEERING' or job.get('benchmark_registration') or job.get('preload_fixture'):
            raise ValueError('V7.3 engineering only; fresh perception, never an old registered slot')
        super().__init__(job);self.g['runtime_policy']=self.policy
        low,high=representable_joint_bounds(self.solver.original_limits,self.policy.data['joint_margin_rad'])
        self.solver.robot.lowerPositionLimit=low;self.solver.robot.upperPositionLimit=high
        atomic_json(self.output/'NATIVE_COMMAND_PRECISION.json',dict(dtype=str(self.qtarget.dtype),strict_joint_margin_rad=self.policy.data['joint_margin_rad'],IK_lower=low.tolist(),IK_upper=high.tolist(),reason='represent inclusive IK bounds strictly inside unchanged guard at actual float32 command precision; no safety relaxation'))
        previous=self.record.coordinate_fields
        def measured_motion(row):
            from .compact_v2 import pose7
            value=previous(row)
            value['moving_link_pose']=pose7(row['T_moving_link'])
            return value
        self.record.coordinate_fields=measured_motion
        from .sensor_lighting_v1 import install
        atomic_json(self.output/'LIGHTING_PROVENANCE.json',install(self.scene['stage'],self.root/'configs/sensor_lighting_v1.json'))
        original=self.Closure
        def checked_closure(T,policy):
            candidate=self.g.get('chosen',{})
            measured=np.asarray(self.robot.get_joint_positions())[self.arm]
            audit=closing_envelope(self.g,measured,predicted_aperture(candidate),policy)
            atomic_json(self.output/('CLOSING_REVALIDATION_%02d.json'%self.g.get('physical_grasps_started',0)),audit)
            if not audit['valid']:raise RuntimeError('PRECONTACT_CLOSING_ROUTE_UNAVAILABLE:'+audit['reason'])
            return original(T,policy)
        self.Closure=checked_closure

    def step(self):
        row=super().step()
        if self.tick%240==0:
            # One overwritten status packet, no extra scene query or raw copy.
            atomic_json(self.output/'LIVE_STATE.json',result_packet(dict(t_sim=row['t'],phase=row['phase'],joint_coordinate_m=row['joint_coordinate'],pad_load_n=list(row['forces_n'].values()),command_aperture_m=row['command_aperture_m'],measured_aperture_m=row['aperture_m'],relative_translation_m=row['relative_translation_slip_m'],protection=row.get('force_event'),agent_wait_is_not_allowed_while_loaded=True)))
        return row


class PrismaticOperationsV73(PreloadOperations):
    worker_script='scripts/update_prismatic_v73_model.py'
    def observe(self):
        result=super().observe();row=self.g['agent_observations'][-1];folder=Path(row['folder'])
        if not row['accepted_for_geometry']:
            sam=self.r.job['wrist_experiment']['sam3']
            env=dict(os.environ,PYTHONPATH=str(self.r.root)+':'+str(self.r.root/'scripts')+':'+sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(self.r.job['visual_gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
            with (folder/'prismatic_recovery.log').open('w') as log:
                p=subprocess.run([sam['python'],str(self.r.root/'scripts/prismatic_visual_v73.py'),str(folder),'--checkpoint',sam['checkpoint']],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            result['semantic_recovery']=dict(returncode=p.returncode,source=str(folder/'SEMANTIC_RECOVERY_V73.json'),overlay=str(folder/'handle_mask.overlay.png'))
            if p.returncode==0 and (folder/'handle_mask.npy').exists():
                result['incremental']=self.accept_geometry(folder,source='shared semantic alias; unchanged .25 SAM threshold')
        if (folder/'depth_handle_proposals.json').exists():result['observed_handle_hypotheses']=json.loads((folder/'depth_handle_proposals.json').read_text());result['hypothesis_board']=str(folder/'depth_handle_proposals.png')
        cues=json.loads((folder/'candidate_cues.json').read_text()) if (folder/'candidate_cues.json').exists() else []
        result['semantic_candidates']=[dict(c,sha256=hashlib.sha256((folder/(c['name']+'.npy')).read_bytes()).hexdigest()) for c in cues if (folder/(c['name']+'.npy')).exists()]
        atomic_json(self.r.output/'LATEST_OBSERVATION_RESULT.json',result_packet(result));return result

    def accept_geometry(self,folder,source):
        from native_g1_observed_regions import extend_observed_mask
        from native_g1_metric_rgbd_geometry import preserve,save_view
        from native_g1_agent_incremental_activation import feedback
        row=self.g['agent_observations'][-1];mask=np.load(folder/'handle_mask.npy').astype(bool);mask&=~np.load(folder/'robot_self_exclusion.npy');np.save(folder/'handle_mask.npy',mask)
        from PIL import Image
        rgb=np.asarray(Image.open(folder/'rgb.png').convert('RGB')).copy();rgb[mask]=(.45*rgb[mask]+.55*np.array([0,255,50])).astype(np.uint8)
        Image.fromarray(rgb).save(folder/'selected_handle_overlay.png')
        extend_observed_mask(folder);save_view(preserve(folder),folder/'measured_geometry')
        row.update(accepted_for_geometry=True,geometry_issue=None,prismatic_geometry_source=source)
        from native_g1_agent_inspection import inspect
        inspect(folder,self.g['agent_observations']);atomic_json(self.r.output/'AGENT_OBSERVATIONS.json',self.g['agent_observations'])
        return feedback(self.g)

    def request_plan(self,command):
        request=command.get('observed_handle_hypothesis')
        if request is not None:
            if request.get('source')=='SAM3':
                folder=Path(self.g['agent_observations'][-1]['folder']);cues=json.loads((folder/'candidate_cues.json').read_text())
                rows=[c for c in cues if c['name']==request.get('id') and not c['partial'] and c.get('eligible_for_grasp')]
                if len(rows)!=1 or not request.get('reason'):return dict(plan=False,reason='UNKNOWN_PARTIAL_OR_UNEXPLAINED_SEMANTIC_SELECTION',recoverable=True)
                path=folder/(rows[0]['name']+'.npy')
                if hashlib.sha256(path.read_bytes()).hexdigest()!=request.get('sha256'):return dict(plan=False,reason='STALE_SEMANTIC_SELECTION',recoverable=True)
                mask=np.load(path);np.save(folder/'handle_mask.npy',mask);np.save(folder/'sam3_mask_raw.npy',mask)
                atomic_json(folder/'AGENT_HANDLE_HYPOTHESIS_APPROVAL.json',dict(request=request,GT_used=False,source='Agent-inspected current SAM3 mask; unchanged threshold'))
                self.accept_geometry(folder,'Agent-selected current SAM3 hypothesis; no score/threshold change')
                return super().request_plan(command)
            folder=Path(self.g['agent_observations'][-1]['folder']);data=json.loads((folder/'depth_handle_proposals.json').read_text())
            if request.get('sha256')!=data.get('sha256') or not isinstance(request.get('reason'),str) or not request['reason'].strip():
                return dict(plan=False,reason='STALE_HYPOTHESIS_OR_MISSING_VISUAL_REASON',recoverable=True)
            rows=[r for r in data['proposals'] if r['id']==request.get('id') and not r['partial']]
            if len(rows)!=1:return dict(plan=False,reason='UNKNOWN_OR_PARTIAL_OBSERVED_REGION',recoverable=True)
            with np.load(folder/'depth_handle_proposals.npz') as a:mask=a['proposal_'+rows[0]['id']]
            np.save(folder/'handle_mask.npy',mask);np.save(folder/'sam3_mask_raw.npy',mask)
            atomic_json(folder/'AGENT_HANDLE_HYPOTHESIS_APPROVAL.json',dict(request=request,source='measured depth foreground; Agent-inspected RGB; not SAM confidence or GT',unknown_backside=True))
            self.accept_geometry(folder,'Agent-selected observed depth hypothesis, semantic uncertainty retained')
        return super().request_plan(command)

    def _update(self,ready):
        # The existing asynchronous production worker owns model coefficients.
        # This separate FIT-only packet exposes conditional-vs-contact-state
        # uncertainty; no sample is silently dropped or coefficient corrected.
        result=super()._update(ready)
        if result.get('returncode')==0:
            try:
                from prismatic_quality_v73 import quality
                atomic_json(self.r.output/'model/FIT_CONSISTENCY_V73.json',quality(self.r.output))
                result['FIT_consistency']='model/FIT_CONSISTENCY_V73.json'
            except Exception as e:result['FIT_consistency_error']=repr(e)
        return result

    def build_geometry(self):
        import native_g1_execution_candidates as executable
        import xarm7_native_planning as native
        with patch.object(executable,'CandidateCheck',PrismaticCandidateCheck),patch.object(native,'RobotPinkIK',native_precision_ik_class()):result=super().build_geometry()
        if result.get('plan'):
            from prismatic_inspection_v73 import plan_board
            result['plan_board']=plan_board(self.g)
        return result

    def reobserve(self):
        self.current_plan_ready=False;self.g.pop('recovery_candidate_catalog',None)
        self.g['observation_generations'].append(self.g['agent_observations']);self.g['agent_observations']=[];self.g['wrist_routes']=[];self.g['agent_moves']=0
        from .registered_observation_v71 import initialize
        initialize(self.r,self)
        return json.loads((self.r.output/'INITIAL_OBSERVATION_READY.json').read_text())

    def attempt(self,command):
        requested=command.get('native_g1_preload_request_n',self.r.closure_policy['preload_n'])
        if isinstance(requested,bool) or requested not in (.5,.75,1.,1.5):return dict(executed=False,reason='V71_TESTED_PRELOAD_MENU_ONLY',recoverable=True)
        result=super().attempt(command)
        try:result['contact_inspection']=self.feedback().get('contact_feedback')
        except Exception as e:result['contact_inspection_error']=repr(e)
        return result

    def rotate_segment(self,target_deg,max_sim_s=None,action=None,**kwargs):
        a=action or {};motion=kwargs.get('articulation');target=kwargs.get('target_q')
        if motion is None or motion.joint_type!='prismatic':raise ValueError('Prismatic-only operation')
        check=reachable_interval(self.r,motion,target,a);requested=float(target)
        atomic_json(self.r.output/('MEASURED_PROBE_PREFLIGHT_%02d.json'%len(self.r.record.segments)),check)
        if not check['valid']:
            prefix=check['feasible_target_q']
            if a['role'].startswith('VALIDATION') or not a.get('allow_target_truncation',False) or abs(prefix-check['actual_start_q'])<=check['stopping_distance_m']:
                return dict(ok=False,status='SAFE_PARTIAL_PROGRESS',reason='CURRENT_STATE_REACHABILITY_BOUNDARY',requested_target_reached=False,safely_stopped=True,physical_transmission_valid=False,planning=check,gain_native=0.,joint_coordinate=self.r.actual())
            kwargs['target_q']=prefix
        result=super().rotate_segment(target_deg,max_sim_s,a,**kwargs);result['current_state_preflight']=check
        if not check['valid']:
            result['original_requested_target_q']=requested;result['requested_target_reached']=False
            if result.get('ok'):result.update(ok=False,status='SAFE_PARTIAL_PROGRESS',reason='PLANNED_PROACTIVE_RELEASE_BOUNDARY')
        return result

    def decision_packet(self,seq,result=None):
        """Small precomputed references instead of repeating expensive inspection."""
        folder=Path(self.g['agent_observations'][-1]['folder']) if self.g['agent_observations'] else None
        catalog=self.g.get('recovery_candidate_catalog',{})
        model_path=self.r.output/'model/estimate.json';quality_path=self.r.output/'model/FIT_CONSISTENCY_V73.json'
        model=json.loads(model_path.read_text()) if model_path.exists() else {};quality=json.loads(quality_path.read_text()) if quality_path.exists() else {}
        selected=model.get('selected',{})
        model_summary=dict(status=model.get('status','pending'),parameters=selected.get('parameters'),conditional_std=selected.get('conditional_std'),condition=selected.get('scaled_condition'),validation_bins=model.get('validation_bins'),between_FIT_parameter_range=quality.get('between_FIT_parameter_range'),FIT_groups=[{k:r.get(k) for k in ('segment','parameters','conditional_std','velocity_rms_m_s','acceleration_rms_m_s2')} for r in quality.get('groups',[])])
        summary=None if result is None else {k:result[k] for k in ('reason','status','plan','geometry','executed','acquisition','safe_exit','candidate_family','segments','failed_contact_candidate') if k in result}
        if summary and isinstance(summary.get('safe_exit'),dict):summary['safe_exit']={k:summary['safe_exit'][k] for k in ('safe','reason') if k in summary['safe_exit']}
        if summary and 'segments' in summary:summary['segments']=[{k:s[k] for k in ('status','reason','joint_coordinate','gain_native','elapsed_sim_s','requested_target_reached') if k in s} for s in summary['segments']]
        return result_packet(dict(sequence=seq,unloaded=True,joint_coordinate_m=self.r.actual(),acquisitions=self.acquisitions,
            remaining_resources=self.r.episode_budget.remaining(self.r.tick*self.r.dt),preload_request_n=self.r.closure_policy['preload_n'],legal_preloads_n=[.5,.75,1.,1.5],
            fresh_rgb=str(folder/'rgb.png') if folder else None,RGBD_board=str(folder/'agent_inspection.png') if folder else None,
            contact_board=str(self.r.output/'CONTACT_FEEDBACK.png'),contact_geometry=str(self.r.output/'CONTACT_GEOMETRY.png'),plan_board=str(self.r.output/'AGENT_GRASP_PLAN_INSPECTION.png'),
            candidate_catalog_sha256=catalog.get('catalog_sha256'),eligible_candidates=[x for x in catalog.get('candidates',[]) if x['eligible']][:4],
            last_result=summary,model_summary=model_summary,model_reference=str(model_path),FIT_consistency_reference=str(quality_path),events=self.r.events[-8:],hard_latch=self.r.supervisor.latched,
            instruction='Inspect fresh images and feedback, choose feasible recovery promptly; low-level safety remains authoritative. No unobserved GT pose.'))


def serve_v73(r):
    ops=PrismaticOperationsV73(r);g=r.g;commands=r.output/'agent_commands';responses=r.output/'agent_responses';commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);trace=[];last=None
    try:
        from .registered_observation_v71 import initialize
        initialize(r,ops)
        for seq in range(int(r.job.get('maximum_agent_operations',40))):
            if g['recovery_state']!='UNLOADED_OBSERVATION' or r.supervisor.latched is not None:
                return dict(status='HARD_STOP_OR_NO_SAFE_DECISION_BOUNDARY',trace=trace,stop=r.supervisor.latched)
            resources=r.episode_budget.remaining(r.tick*r.dt);wait=min(float(r.job['agent_wait_timeout_s']),resources['wall_s'])
            if wait<=0:return dict(status='BUDGET_EXHAUSTED',safe_exit=ops.release_park(),trace=trace)
            atomic_json(r.output/'DECISION_PACKET.json',ops.decision_packet(seq,last));atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=True,timeout_s=wait,decision_packet=str(r.output/'DECISION_PACKET.json')))
            began=time.perf_counter();path=commands/('%03d.json'%seq)
            while not path.exists():
                if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>=wait:
                    return dict(status='ORCHESTRATION_TIMEOUT' if not r.stopped else 'OWNER_STOP',safe_exit=ops.release_park(),trace=trace)
                time.sleep(.05)
            command,ack=receive(path,seq,responses)
            if command is None:trace.append(ack);continue
            op=command['operation'];started=time.perf_counter()
            try:
                for a in command.get('segments',[]):validate_action(a);ops.articulation().validate_target(a)
                if op=='observe':result=ops.observe()
                elif op=='inspect':result=ops.decision_packet(seq)
                elif op=='evaluate_view':result=ops.evaluate_view(command['target']);ops.cache[seq]=result
                elif op=='move_and_observe':result=ops.move_and_observe(ops.cache.get(command['evaluated_command'],{}))
                elif op=='build_geometry':result=ops.request_plan(command)
                elif op=='validate_execution':result=ops.validate_execution()
                elif op=='attempt_grasp_and_probe':result=ops.attempt(command)
                elif op=='inspect_contact_result':result=ops.feedback()
                elif op in ('reobserve_and_replan','fresh_mode_observation'):result=ops.reobserve()
                elif op=='safe_release_retreat':result=ops.release_park()
                elif op=='finish':result=dict(safe_exit=ops.release_park())
                else:result=dict(executed=False,recoverable=True,reason='MODE_NOT_IMPLEMENTED_FOR_HORIZONTAL_DRAWER')
            except SafetyStop as e:
                result=dict(status='SAFETY_STOP',reason=e.reason,evidence=e.evidence)
            except ExecutionBudgetExhausted as e:
                result=dict(status='BUDGET_EXHAUSTED',reason=str(e),safe_exit=ops.release_park())
            except Exception as e:
                result=dict(operation_stopped=True,reason=repr(e),recoverable=g['recovery_state']=='UNLOADED_OBSERVATION' and r.supervisor.latched is None)
                if not result['recoverable']:result['safe_exit']=dict(safe=False,reason='NO_VERIFIED_UNLOADED_BOUNDARY')
            entry=dict(command=command,result=result,wall_s=time.perf_counter()-started,unloaded_wait_s=started-began);complete(responses,seq,result_packet(entry));trace.append(entry);last=result
            atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
            from .compact_v2 import prune_optional
            atomic_json(r.output/'STORAGE_SUMMARY.json',prune_optional(r.output))
            if op=='finish':return dict(status='SESSION_COMPLETE',trace=trace,safe_exit=result.get('safe_exit'))
            if r.supervisor.latched is not None or result.get('status')=='BUDGET_EXHAUSTED':return dict(status=result.get('status','SAFETY_STOP'),trace=trace)
        return dict(status='BOUNDED_OPERATION_BUDGET',safe_exit=ops.release_park(),trace=trace)
    finally:
        ops.close_workers()
        # The inherited compact finalizer keeps the raw RGB/depth/selected mask
        # once. Retain new semantic/hypothesis provenance before it removes
        # reconstructible mask renderings and intermediate files.
        audit=[]
        for folder in sorted((r.output/'rgbd').glob('*')):
            row=dict(checkpoint=folder.name)
            for name in ['SEMANTIC_RECOVERY_V73.json','depth_handle_proposals.json','AGENT_HANDLE_HYPOTHESIS_APPROVAL.json']:
                if (folder/name).exists():row[name]=json.loads((folder/name).read_text())
            if len(row)>1:audit.append(row)
        atomic_json(r.output/'PERCEPTION_PROVENANCE_V73.json',result_packet(audit))
