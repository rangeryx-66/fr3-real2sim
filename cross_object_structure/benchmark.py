"""Finite, immutable benchmark orchestration. Reference physics never fitted."""
import concurrent.futures
import copy
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from run_interactive_twin_benchmark import Benchmark, read, write, resolve, sha
from run_interactive_twin_recovery_benchmark import summary_safe


def lane(args):
    config,episodes,gpu=args
    b=StructureBenchmark(config)
    return [b.episode(e,gpu) for e in episodes]


def prediction_lane(args):
    config,episodes,gpu=args
    b=StructureBenchmark(config)
    from cross_object_structure.prediction import predict
    for e in episodes:
        file=b.out/'episodes'/e['episode_id']/'episode_summary.json'
        if not file.exists():continue
        s=read(file)
        if not s.get('heldout_pending'):continue
        try:
            s['selected_job']['gpu']=gpu
            s['heldout']=predict(b,s);s['heldout_pending']=False
        except Exception as error:
            s['heldout_error']=str(error)
        write(file,summary_safe(s))


def recovery_export(export):
    """Match frozen scene_at's input contract: it authors one chassis itself.

    The native planning scene already contains the same chassis. Passing it
    through would create a second collider and report chassis-vs-itself contact.
    This is the same normalization used by MobileRuntimeScene's native caller;
    neither the physical chassis nor its collision checks are disabled.
    """
    result=copy.deepcopy(export)
    result['shapes']=[s for s in result['shapes'] if s['path']!='/World/mobile_chassis']
    return result


class StructureBenchmark(Benchmark):
    def __init__(self,c):
        self.c=c
        original=read(resolve(c['original_config']))
        original.update(output=c['output'],runtime=c['runtime'],policy=c['policy'],dev=c['dev'])
        original['ranking_paths']=[str(resolve(c['output'])/'asset_preparation/ranking.json')]
        super().__init__(original)
        self.out=self.output
        self.protocol={k:c[k] for k in ('fitting','selection','refinement','validation','heldout')}

    def run(self,job):
        out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
        file=out/'report.json';jf=out/'job_private.json'
        entry=ROOT/'scripts/run_cross_object_structure_episode.py'
        digest=hashlib.sha256(json.dumps(job,sort_keys=True).encode()).hexdigest()
        identity={'job_sha256':digest,'entry_sha256':sha(entry)}
        if file.exists():
            if read(out/'identity.json')!=identity:raise RuntimeError('IMMUTABLE_JOB_CHANGED:'+str(out))
            return read(file)
        write(jf,job);write(out/'identity.json',identity)
        if self.expired():
            r={'status':'CUTOFF_05_00','success':False};write(file,r);return r
        env=dict(self.env)
        for key in list(env):
            if key.lower().endswith('_proxy'):env.pop(key)
        env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        env['PATH']=self.c['runtime']['ffmpeg_directory']+os.pathsep+env.get('PATH','')
        cmd=[self.python,str(entry),'--job',str(jf)]
        with (out/'process.log').open('w') as log:
            p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            write(out/'process.json',{'pid':p.pid,'started_unix_s':time.time(),'command':cmd})
            timeout=max(.1,min(job['wall_clock_budget_s']+45,self.deadline-time.time()))
            try:p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=min(20.,max(.1,self.deadline-time.time())))
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        if not file.exists():write(file,{'status':'NATIVE_PROCESS_FAILED','success':False,'exit_code':p.returncode})
        return read(file)

    def freeze(self):
        from interactive_twin.manifest import scan_prepared,_handle,_episodes,DEFAULT_POLICY
        from cross_object_structure.assets import prepare
        target=self.out/'frozen_asset_manifest.json'
        if target.exists():
            if read(self.out/'frozen_config.json')!=self.c:raise RuntimeError('FROZEN_CONFIG_CHANGED')
            m=read(target)
            for file,h in m['frozen_code_sha256'].items():
                if sha(ROOT/file)!=h:raise RuntimeError('FROZEN_CODE_CHANGED_REQUIRE_PROVENANCE_AND_ALL_AFFECTED_RERUN:'+file)
            self.manifest=m;return m
        prepared=prepare(self.c,self.out/'asset_preparation')
        assets=scan_prepared([a['asset_root'] for a in prepared if a.get('valid_handles')])
        by_id={a['asset_id']:a for a in prepared}
        groups=defaultdict(list)
        for a in assets:
            if a['exclusion_reasons']:continue
            handles=[_handle(r,DEFAULT_POLICY) for r in by_id[a['asset_id']]['valid_handles']]
            a['selected_handle']=max(handles,key=lambda h:(h['selection_score'],str(h['mesh'])))
            groups[(a['category'],a['object_name'])].append(a)
        for g in groups.values():g.sort(key=lambda a:(-a['selected_handle']['selection_score'],a['asset_id']))
        ordered=[]
        while any(groups.values()):
            for key in sorted(groups):
                if groups[key]:ordered.append(groups[key].pop(0))
        selected=ordered[:self.c['assets']['test_assets']]
        policy=copy.deepcopy(DEFAULT_POLICY);policy.update(self.c['policy'])
        policy.update(episodes_per_asset=2,initial_articulation_offsets_deg=[0.,1.])
        episodes=[e for a in selected for e in _episodes(a,'TEST',policy)]
        # Resolve actual initial scene and approximate proxy BEFORE any execution.
        for episode in episodes:
            folder,asset,dep=self.prepare_episode(episode)
            episode.update(prepared_proxy_root=str(asset),resolved_initial_scene=dep['source_report']['asset_installation'],
                           initial_base=dep['source_report']['robot_base_pose'],
                           prepared_proxy_sha256=sha(asset/'manifest.json'),
                           resolved_visual_sha256=sha(folder/'initial_visual_handle_world.json'))
        frozen=[]
        for pattern in ('cross_object_structure/*.py','interactive_twin/*.py','interactive_twin_recovery/*.py',
                        'interactive_twin_refinement/*.py','interaction_identification/*.py','articulated_interaction/*.py'):
            frozen.extend(ROOT.glob(pattern))
        frozen.extend([ROOT/'scripts/run_cross_object_structure_episode.py',ROOT/'scripts/run_cross_object_structure_benchmark.py',
                       ROOT/'scripts/run_interactive_twin_refinement_episode.py',ROOT/'config/semantic_interaction.json'])
        m={'schema':'cross-object-structure-manifest-v1','frozen_before_physical_execution':True,
           'selection_uses_physical_outcomes':False,'selected_assets':selected,'test_asset_ids':[a['asset_id'] for a in selected],
           'episodes':episodes,'test_denominator_assets':len(selected),'test_denominator_episodes':len(episodes),
           'missing_assets':max(0,8-len(selected)),'source_pool':str(self.out/'asset_preparation/source_pool.json'),
           'all_geometry_exclusions':str(self.out/'asset_preparation/preparation_inventory.json'),
           'excluded_exposure_ids':self.c['assets']['previously_prepared_ids'],
           'controls_not_in_denominator':['7320','45621'],'physics_calibration':False,
           'frozen_code_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sorted(set(frozen))},
           'policy':policy,'heldout':self.c['heldout'],'created_unix_s':time.time()}
        m['manifest_sha256']=hashlib.sha256(json.dumps(m,sort_keys=True).encode()).hexdigest()
        write(self.out/'frozen_config.json',self.c);write(target,m);self.manifest=m
        return m

    def controls(self,full_structure=False):
        """Rerun existing deployments/grasp once; no outcome-dependent tuning."""
        results={}
        def one(item):
            aid,spec,gpu=item;job=read(resolve(spec['source_job']))
            for key in ('import_spec','replay_commands','conditional_snapshot','initial_estimate','initial_estimate_memory'):
                job.pop(key,None)
            job.update(output=str(self.out/('controls_full_structure' if full_structure else 'controls')/aid),gpu=gpu,
                       episode_id=('structure_control_' if full_structure else 'control_')+aid,
                       mode='kinematics',continue_manipulation=True,refinement_once=full_structure,
                       refinement_target_deg=self.c['refinement']['opening_deg'] if full_structure else 5.5,
                       deadline_shanghai=self.c['runtime']['deadline_shanghai'],
                       wall_clock_budget_s=self.c['runtime']['episode_wall_clock_s'])
            if full_structure:job['structure_protocol']=self.protocol
            if job.get('mobile_route'):job['mobile_route']['deadline_unix_s']=self.deadline
            result=self.run(job)
            if full_structure:
                summary={'asset_id':aid,'control_not_in_test_denominator':True,'selected_job':job,
                         'selected_report':str(Path(job['output'])/'report.json'),'status':result['status'],
                         'fixed_base_feasible':True,'grasp_feasible':True}
                if result.get('bilateral_hold_established'):self.evaluate_episode(summary)
                write(Path(job['output'])/'episode_summary.json',summary)
                return aid,summary
            return aid,{'status':result['status'],'grasp':result.get('bilateral_hold_established',False),
                        'identification':result.get('estimate',{}).get('joint_type'),
                        'success':result.get('success',False),'evaluation':result.get('evaluation',{}),
                        'source_job':spec['source_job'],'report':str(Path(job['output'])/'report.json')}
        items=[(aid,spec,self.gpus[i]) for i,(aid,spec) in enumerate(self.c['controls'].items())]
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            for aid,r in pool.map(one,items):results[aid]=r
        write(self.out/('full_structure_controls.json' if full_structure else 'regression_controls.json'),results)
        return results

    def control_predictions(self,gpus=None):
        from cross_object_structure.prediction import predict
        file=self.out/'full_structure_controls.json'
        if not file.exists():raise RuntimeError('FULL_STRUCTURE_CONTROLS_REQUIRED')
        controls=read(file)
        for aid,summary in controls.items():
            if not summary.get('heldout_pending'):continue
            summary['heldout']=predict(self,summary,replay_gpus=(gpus or self.gpus)[:3])
            summary['heldout_pending']=False
            write(Path(summary['selected_job']['output'])/'episode_summary.json',summary)
            write(file,controls)
        return controls

    def episode(self,e,gpu):
        from interaction_identification.contact_probe import robot_only_model
        from interactive_twin_recovery.mobile import recover,eligible
        out=self.out/'episodes'/e['episode_id'];summary_file=out/'episode_summary.json'
        if summary_file.exists():return read(summary_file)
        summary={'episode_id':e['episode_id'],'asset_id':e['asset_id'],'included_in_denominator':True,
                 'fixed_base_feasible':False,'mobile_recovered':False,'attempts':[],
                 'failure_stage':'deployment','status':'STARTED'}
        try:
            if self.expired():raise RuntimeError('CUTOFF_05_00')
            asset=out/'asset';source=out/'source';plan=out/'planning/plan.json'
            job=self.job(f"episodes/{e['episode_id']}/planning",source=source,asset=asset,plan=plan,gpu=gpu,
                         mode='plan',initial_visual=str(out/'initial_visual_handle_world.json'),episode_id=e['episode_id'],
                         seed=e['seed'],initial_articulation_rad=e['initialization_only']['joint_position_rad'],
                         planning_budget_s=self.c['mobile']['fixed_plan_wall_s'],plant={},mobile_platform=True)
            r=self.run(job)
            if not plan.exists():raise RuntimeError('INITIAL_IMPORT_OR_PLANNING_FAILED:'+r['status'])
            fixed=read(plan);summary['fixed_base_feasible']=bool(fixed['trial_candidates']);summary['fixed_plan_statuses']=[r['status'] for r in fixed['rows']]
            route=None
            if not fixed['trial_candidates']:
                if not eligible(fixed):raise RuntimeError('FIXED_FAILURE_NOT_MOBILE_ELIGIBLE')
                initial=read(source/'report.json')['robot_base_pose'];visual=read(out/'initial_visual_handle_world.json')
                export=recovery_export(read(out/'planning/cooked_initial.json'))
                write(out/'mobile_scene_adapter.json',{'native_chassis_retained':True,
                    'search_chassis_authored_by':'unchanged interactive_twin_recovery.mobile.scene_at',
                    'duplicate_export_entry_removed':'/World/mobile_chassis','chassis_collision_checks_enabled':True})
                rec=recover(ROOT,export,fixed,visual,initial,
                            self.c['mobile']['search'],seed=e['seed'],deadline=self.deadline)
                write(out/'mobile_search.json',summary_safe(rec))
                if rec['selected'] is None:raise RuntimeError('NO_MOBILE_RECOVERY')
                selected=rec['selected'];summary.update(mobile_selected_pose=selected['base'],mobile_travel_m=selected['route']['translation_m'])
                plan=out/'mobile_plan.json';write(plan,summary_safe(selected['plan']))
                copied=read(source/'report.json');copied['robot_base_pose']=selected['base'];source=out/'mobile_source';write(source/'report.json',copied)
                route={**selected['route'],'initial_base':initial,'deadline_unix_s':self.deadline}
            summary['grasp_feasible']=True;summary['failure_stage']='grasp'
            trial_plan=read(plan);started=time.monotonic()
            # Same bounded candidate order and stop after the first bilateral grasp.
            for index in range(min(12,len(trial_plan['trial_candidates']))):
                if self.expired() or time.monotonic()-started>=self.c['mobile']['configuration_execution_wall_s']:break
                job=self.job(f"episodes/{e['episode_id']}/candidate_{index:02d}",source=source,asset=asset,plan=plan,gpu=gpu,
                             mode='kinematics',candidate=index,episode_id=e['episode_id'],plant={},
                             initial_articulation_rad=e['initialization_only']['joint_position_rad'],
                             mobile_platform=True,continue_manipulation=True,refinement_once=True,
                             refinement_target_deg=self.c['refinement']['opening_deg'],structure_protocol=self.protocol)
                if route:job['mobile_route']=route
                r=self.run(job);summary['attempts'].append({'output':job['output'],'status':r['status'],'bilateral':r.get('bilateral_hold_established',False)})
                if route and (Path(job['output'])/'mobile_route_result.json').exists():summary['mobile_recovered']=True
                if r.get('bilateral_hold_established'):
                    summary.update(selected_job=job,selected_report=str(Path(job['output'])/'report.json'),
                                   status=r['status'],failure_stage=r.get('failure_phase','complete'))
                    break
            if 'selected_job' not in summary:
                summary['status']=summary['attempts'][-1]['status'] if summary['attempts'] else 'EXECUTION_BUDGET_EXHAUSTED'
            else:
                self.evaluate_episode(summary)
        except Exception as error:
            import traceback
            summary.update(status=str(error),exception_trace=traceback.format_exc())
        finally:
            write(summary_file,summary_safe(summary))
        return summary

    def evaluate_episode(self,summary):
        """Called only after the native episode, fitting and control have ended."""
        import numpy as np
        from interaction_identification.fitting import evaluate
        job=summary['selected_job'];folder=Path(job['output']);r=read(folder/'report.json')
        private=read(folder/'evaluation_only.json');gt=private['ground_truth']
        rows=read(folder/'observations.json')
        selected=read(folder/'structure_selection.json') if (folder/'structure_selection.json').exists() else {}
        summary['structure_validation']=selected.get('status','NOT_REACHED')
        for name,file in [('discovery','discovery_articulation.json'),('refined','refined_articulation.json')]:
            if (folder/file).exists():
                fit=read(folder/file);poses=selected.get('support') or read(folder/'ee_probe_trajectory.json')
                if fit.get('joint_type')=='revolute' and len(poses)>1:
                    metric=evaluate(fit,gt,np.asarray(poses[0])[:3,3],poses)
                    # Axis-line distance removes the unobservable along-axis origin gauge.
                    a=np.asarray(fit['revolute']['axis']);c=np.asarray(fit['revolute']['point_on_axis']);g=np.asarray(gt['axis_world']);o=np.asarray(gt['origin_world']);anchor=np.asarray(poses)[:,:3,3].mean(0)
                    if abs(a@g)>1e-6:
                        projected=c+a*(g@(anchor-c))/(g@a);truth=o+g*(g@(anchor-o));metric['axis_line_distance_m']=float(np.linalg.norm(projected-truth))
                    summary[name+'_evaluation']=metric
        summary.update(grasp=True,probe=r.get('probe'),minimum_joint_margin_rad=r.get('minimum_joint_margin_rad'),
                       observed_slip_m=r.get('maximum_detected_contact_surface_drift_m'),post_episode_evaluation=r.get('evaluation'),
                       refined_accepted=selected.get('accepted',False),online_success=r.get('online_success',False),
                       success=bool(r.get('success') and selected.get('accepted',False)),video=str(folder/'contact_baseline.mp4'))
        if selected.get('accepted') and (folder/'heldout_completion.json').exists():
            # Complete all registered physical episodes before allocating GPUs
            # to three prediction replays of an early successful asset.
            summary['heldout_pending']=True

    def batch(self):
        m=self.freeze();groups=defaultdict(list)
        for e in m['episodes']:groups[e['asset_id']].append(e)
        lanes=[[] for _ in self.gpus]
        for i,eps in enumerate(groups.values()):lanes[i%len(lanes)].extend(eps)
        # One native world per GPU; CPU searches do not share Python's GIL.
        with concurrent.futures.ProcessPoolExecutor(max_workers=len(self.gpus)) as pool:
            result=list(pool.map(lane,[(self.c,eps,g) for eps,g in zip(lanes,self.gpus) if eps]))
        with concurrent.futures.ProcessPoolExecutor(max_workers=len(self.gpus)) as pool:
            list(pool.map(prediction_lane,[(self.c,eps,g) for eps,g in zip(lanes,self.gpus) if eps]))
        return result
