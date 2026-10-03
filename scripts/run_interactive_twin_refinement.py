"""Bounded independent structure -> physics -> held-out experiment."""
import argparse,copy,hashlib,json,os,subprocess,sys,time,signal,queue
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from run_interactive_twin_benchmark import read,write,sha,resolve

def _run_native(job):
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    identity={'job':job,'entry_sha256':sha(ROOT/'scripts/run_interactive_twin_refinement_episode.py')}
    if (out/'report.json').exists():
        if read(out/'identity.json')['job']!=job:raise RuntimeError('STALE_NATIVE_RESULT')
        # Each result retains its entry hash; diagnostic-only additions do not
        # rerun an already spent physical refinement budget.
        if read(out/'identity.json')['entry_sha256']!=identity['entry_sha256']:
            write(out/'reuse_entry_provenance.json',{'original_entry_sha256':read(out/'identity.json')['entry_sha256'],'current_entry_sha256':identity['entry_sha256'],'physical_rerun':False})
        return read(out/'report.json')
    if (out/'process.json').exists():
        p=read(out/'process.json')
        if 'exit_code' not in p:
            try:os.kill(p['pid'],0)
            except ProcessLookupError:pass
            else:raise RuntimeError('EXISTING_NATIVE_PROCESS')
    write(out/'identity.json',identity);write(out/'job_private.json',job)
    env=dict(os.environ)
    for k in ('PYTHONPATH','CUDA_VISIBLE_DEVICES','http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','all_proxy'):env.pop(k,None)
    env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'--job',str(out/'job_private.json')]
    deadline=datetime.fromisoformat(job['deadline_shanghai']).timestamp()
    if time.time()>=deadline:
        write(out/'report.json',{'status':'CUTOFF_05_00','success':False});return read(out/'report.json')
    start=time.time()
    with (out/'process.log').open('w') as f:
        p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        write(out/'process.json',{'pid':p.pid,'command':cmd,'start_time':start})
        try:p.wait(timeout=max(1,min(job['wall_clock_budget_s'],deadline-start)-30))
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=30)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    write(out/'process.json',{'pid':p.pid,'command':cmd,'start_time':start,'end_time':time.time(),'exit_code':p.returncode})
    if not (out/'report.json').exists():write(out/'report.json',{'status':'PROCESS_FAILED','exit_code':p.returncode})
    return read(out/'report.json')

_GPU_POOL=queue.Queue()
for _gpu in (1,3,5,7):_GPU_POOL.put(_gpu)

def run_native(job):
    # GPU allocation is infrastructure, not a method parameter. The actual GPU
    # remains recorded in every native job. Completed physical actions are reused.
    out=Path(job['output']);existing=out/'identity.json'
    if (out/'report.json').exists() and existing.exists():
        previous=read(existing)['job'];candidate=dict(job,gpu=previous['gpu'])
        return _run_native(candidate)
    gpu=_GPU_POOL.get()
    try:return _run_native(dict(job,gpu=gpu))
    finally:_GPU_POOL.put(gpu)

class Experiment:
    def __init__(self,c):
        self.c=c;self.out=resolve(c['output']);self.old=resolve(c['original_output']);self.recovery=resolve(c['recovery_output']);self.out.mkdir(parents=True,exist_ok=True)
        frozen=self.out/'frozen_config.json'
        if frozen.exists() and read(frozen)!=c:raise RuntimeError('FROZEN_CONFIG_CHANGED')
        if not frozen.exists():write(frozen,c)
        self.gpus=[1,3,5,7]
        manifest={'config_sha256':sha(frozen),'base_commit':c['base_commit'],'sources':{},'old_unseen_denominator_retained':12}
        for eid in c['episodes']:
            j,folder=self.source(eid)
            paths=[folder/'structured_memory.json',folder/'estimated_articulation.json',folder/'initial_scene_private.json',Path(j['plan']),Path(j['asset_root'])/'manifest.json']
            manifest['sources'][eid]={str(p):sha(p) for p in paths}
        path=self.out/'frozen_manifest.json'
        if path.exists() and read(path)!=manifest:raise RuntimeError('FROZEN_SOURCE_CHANGED')
        if not path.exists():write(path,manifest)
        transfer={'new_transfer_native_runs_started':False,'asset_parameter_sharing':True,
                  'calibration_config':c['episodes'][1],'prediction_only_configs':c['episodes'][2:],
                  'per_config_max_supplement':1,'physics_parameter_candidate_budget_per_asset':9,
                  'source_config_chosen_before_transfer_results':True,'method_thresholds_unchanged':True}
        registration=self.out/'transfer_protocol_frozen.json'
        if registration.exists():
            if any(read(registration).get(k)!=v for k,v in transfer.items()):raise RuntimeError('TRANSFER_REGISTRATION_CHANGED')
        else:write(registration,{'registered_unix_s':time.time(),**transfer})


    def source(self,eid):
        if eid.startswith('dev'):
            s=read(self.old/'episodes'/eid/'episode_summary.json');job=s['selected_job'];folder=Path(job['output'])
        else:
            s=read(self.recovery/'mobile'/eid/'summary.json');a=(s['mobile_attempts'] or s['fixed_attempts'])[-1];folder=Path(a['path']);job=read(folder/'job_private.json')
        return job,folder
    def offline(self,eid):
        from interactive_twin_refinement.fitting import fit_se3,modality_consistency
        job,folder=self.source(eid);out=self.out/eid;out.mkdir(parents=True,exist_ok=True)
        memory=read(folder/'structured_memory.json');T=memory['supporting_observations'];old=read(folder/'estimated_articulation.json')
        if memory.get('GT_inputs') is not False:raise RuntimeError('NON_SENSOR_MEMORY')
        path=out/'historical_review.json'
        if path.exists():return read(path)
        refined=fit_se3(T,**self.c['fitting']);consistency=modality_consistency(T)
        result={'old':old,'joint_fit':refined,'modality_consistency':consistency,'source':str(folder),'source_sha256':sha(folder/'structured_memory.json'),'GT_inputs':False,'supplement_reason':'single-direction short-arc data, no independent reverse validation; budgeted refinement required'}
        if eid.startswith('dev'):
            val=self.old/'episodes'/eid/'final_updated_interaction/structured_memory.json'
            result['historical_validation']={name:fit_se3(read(val)['supporting_observations'],fixed_model=f,**self.c['fitting']) for name,f in [('old',old),('joint',refined)]}
            result['validation_action_source']=str(val)
        write(path,result);write(out/'historical_joint_fit.json',refined)
        print(eid,'OFFLINE',refined['revolute'],flush=True)
        return result
    def supplement(self,eid,gpu=1):
        self.offline(eid);job,folder=self.source(eid);out=self.out/eid/'refinement_once'
        j={**job,'output':str(out),'gpu':gpu,'mode':'kinematics','continue_manipulation':True,'refinement_once':True,'refinement_target_deg':10.,'wall_clock_budget_s':2200,'deadline_shanghai':self.c['deadline_shanghai'], 'initial_estimate':str(folder/'estimated_articulation.json'),'initial_estimate_memory':str(folder/'structured_memory.json')}
        j.pop('import_spec',None)
        # Reuse selected locked deployment; no base search changes. Native mobile
        # wrapper is installed for existing mobile jobs by the new entry only.
        from interactive_twin.visual_import import compatible_urdf
        asset=Path(j['asset_root']);m=read(asset/'manifest.json');compatible_urdf(asset/'urdf'/f"{m['asset_id']}.urdf",asset/'visual_compatibility')
        if j.get('mobile_route'):j['mobile_route']['deadline_unix_s']=datetime.fromisoformat(j['deadline_shanghai']).timestamp()
        r=run_native(j);print(eid,'REFINEMENT',r['status'],flush=True);return r

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default='configs/interactive_twin_refinement.yaml');p.add_argument('--stage',choices=['offline','supplement','full'],default='full');p.add_argument('--episode',default='dev_7320_00');a=p.parse_args()
    e=Experiment(read(resolve(a.config)))
    if a.stage=='offline':e.offline(a.episode)
    elif a.stage=='supplement':e.supplement(a.episode)
    else:
        from interactive_twin_refinement.workflow import full
        full(e)
if __name__=='__main__':main()
