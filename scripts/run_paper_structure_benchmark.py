"""Existing diagnostic cohort -> observation upper bound -> bounded B0/B1/B2."""
from pathlib import Path
import sys, argparse, json, hashlib, copy, time, os, subprocess, signal, concurrent.futures
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_paper_structure_episode import source


def read(p):return json.loads(Path(p).read_text())
def write(p,x):p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def execute(item):
    job,out,c=item;folder=Path(job['output']);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'report.json').exists():
        if read(folder/'job_private.json')!=job:raise RuntimeError('REUSE_JOB_MISMATCH:'+str(folder))
        return read(folder/'report.json')
    write(folder/'job_private.json',job)
    py='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
    env=dict(os.environ);env.pop('PYTHONPATH',None);env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env.get('PATH','')
    cmd=[py,str(ROOT/'scripts/run_paper_structure_episode.py'),'--job',str(folder/'job_private.json')]
    cutoff=datetime.fromisoformat(c['deadline_shanghai']).timestamp()
    if time.time()>=cutoff:write(folder/'report.json',{'status':'CUTOFF_05_00','success':False});return
    with (folder/'process.log').open('w') as log:
        p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        write(folder/'process.json',{'pid':p.pid,'command':cmd,'started_unix_s':time.time()})
        try:p.wait(timeout=min(job['wall_clock_budget_s']+45,cutoff-time.time()))
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    if not (folder/'report.json').exists():write(folder/'report.json',{'status':'NATIVE_PROCESS_FAILED','success':False,'exit_code':p.returncode})
    return read(folder/'report.json')

def lane(args):
    jobs,out,c=args
    for job in jobs:execute((job,out,c))

def freeze(c):
    old=ROOT/c['source_run'];out=ROOT/c['output'];manifest=read(old/'frozen_test_manifest.json')
    summaries=[read(old/'episodes'/e['episode_id']/'episode_summary.json') for e in manifest['episodes']]
    shared=[]; jobs=[]
    for s in summaries:
        report=read(s['selected_report']) if s.get('selected_report') else {}
        if not report.get('bilateral_hold_established'):
            shared.append({'episode_id':s['episode_id'],'asset_id':s['asset_id'],'status':s['status'],'prior_summary':str(old/'episodes'/s['episode_id']/'episode_summary.json'),'shared_prefix_failure':True,'fixed_base_feasible':s.get('fixed_base_feasible')})
            continue
        for method in c['methods']:
            j=read(Path(s['selected_report']).parent/'job_private.json');j.update(paper_method=method,output=str(out/'comparisons'/s['episode_id']/method),episode_id=s['episode_id']+'_'+method,deadline_shanghai=c['deadline_shanghai'])
            assert j.get('active_structure') and j.get('structure_protocol'), 'INCOMPLETE_ACTUAL_JOB'
            jobs.append(j)
    obs=[]
    for eid in ['test_45385_01','test_45671_00','test_45671_01']:
        s=next(s for s in summaries if s['episode_id']==eid);j=read(Path(s['selected_report']).parent/'job_private.json')
        j.update(paper_method='LOG_ONLY',output=str(out/'observation_upper_bound'/eid),deadline_shanghai=c['deadline_shanghai']);obs.append(j)
    for aid in c['controls']:
        j=read(old/'controls_full_structure'/aid/'job_private.json');j.update(paper_method='LOG_ONLY',output=str(ROOT/c['reuse_logging_controls'][aid]) if aid in c.get('reuse_logging_controls',{}) else str(out/'observation_upper_bound'/('control_'+aid)),deadline_shanghai=c['deadline_shanghai']);obs.append(j)
    files=[]
    for pat in ['paper_structure/*.py','scripts/run_paper_structure*.py','configs/paper_structure.json']:files+=list(ROOT.glob(pat))
    inherited=read(old/'frozen_method.json')['code_sha256']
    # Every inherited method and all physical baseline sources must stay intact.
    inherited.update(manifest['frozen_code_sha256'])
    for p,h in inherited.items():
        if sha(ROOT/p)!=h:raise RuntimeError('FROZEN_BASELINE_CHANGED:'+p)
    r={'schema':c['schema'],'frozen_unix_s':time.time(),'config':c,'source_manifest_sha256':sha(old/'frozen_test_manifest.json'),'episodes':manifest['episodes'],'shared_prefix_failures':shared,'comparison_jobs':jobs,'logging_only_jobs':obs,'native_trial_budget':len(jobs)+len(obs),'inherited_sha256':inherited,'new_source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},'heldout_frozen_before_runs':c['heldout'],'method_scope':'internal ablation; not official method reproduction'}
    assert len(jobs)+len(obs)<=c['maximum_native_trials']
    p=out/'frozen_comparison_manifest.json'
    if p.exists():
        prior=read(p)
        for k in ['config','comparison_jobs','logging_only_jobs','inherited_sha256','new_source_sha256']:
            if prior[k]!=r[k]:raise RuntimeError('FROZEN_COMPARISON_CHANGED:'+k)
        return prior
    write(p,r);return r

def batch(jobs,c):
    queues=[[] for _ in c['gpus']]
    for i,job in enumerate(jobs):
        job=copy.deepcopy(job);job['gpu']=c['gpus'][i%len(queues)];queues[i%len(queues)].append(job)
    with concurrent.futures.ProcessPoolExecutor(max_workers=len(queues)) as pool:list(pool.map(lane,[(q,ROOT/c['output'],c) for q in queues if q]))

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/paper_structure.json');p.add_argument('--stage',choices=['freeze','observation','compare','report','full'],default='full');a=p.parse_args();c=read(a.config);m=freeze(c)
    print(json.dumps({'native_trials':m['native_trial_budget'],'shared_failures':len(m['shared_prefix_failures']),'stage':a.stage}),flush=True)
    if a.stage in ('observation','full'):batch(m['logging_only_jobs'],c)
    if a.stage in ('compare','full'):batch(m['comparison_jobs'],c)
    if a.stage in ('report','full'):
        from paper_structure.analysis import summarize
        summarize(ROOT/c['output'],m)

if __name__=='__main__':main()
