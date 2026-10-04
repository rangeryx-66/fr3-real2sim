"""DEV -> method freeze -> fresh TEST, without changing frozen baseline modules."""
import concurrent.futures,copy,hashlib,inspect,json,os,signal,subprocess,sys,textwrap,time
from pathlib import Path
from collections import defaultdict
from cross_object_structure.benchmark import StructureBenchmark,ROOT,read,write,resolve,sha,summary_safe

# Reuse the exact immutable-job runner with only the independent entry changed.
_run=textwrap.dedent(inspect.getsource(StructureBenchmark.run)).replace('run_cross_object_structure_episode.py','run_active_structure_episode.py')
exec(_run,globals());native_run=run


def lane(args):
    c,episodes,gpu=args;b=ActiveBenchmark(c)
    # Compute lease only: do not start a TEST world on an active DEV GPU.
    while not b.expired():
        busy=False
        for f in (b.out/'dev').glob('*/process.json'):
            d=read(f);jf=f.parent/'job_private.json'
            if jf.exists() and read(jf).get('gpu')==gpu:
                proc=Path('/proc')/str(d['pid'])/'cmdline'
                if proc.exists() and str(jf).encode() in proc.read_bytes():busy=True
        if not busy:break
        time.sleep(5.)
    return [b.episode(e,gpu) for e in episodes]


def prediction_lane(args):
    c,episodes,gpu=args;b=ActiveBenchmark(c)
    from cross_object_structure.prediction import predict
    for e in episodes:
        p=b.out/'episodes'/e['episode_id']/'episode_summary.json'
        if not p.exists():continue
        s=read(p)
        if not s.get('heldout_pending'):continue
        s['selected_job']['gpu']=gpu
        try:s['heldout']=predict(b,s);s['heldout_pending']=False
        except Exception as error:s['heldout_error']=str(error)
        write(p,summary_safe(s))


class ActiveBenchmark(StructureBenchmark):
    def __init__(self,c):
        super().__init__(c);self.protocol['active']=c['active']

    def run(self,job):
        job=copy.deepcopy(job)
        if job.get('structure_protocol'):
            job['structure_protocol']['active']=self.c['active']
        if job.get('structure_protocol') or job.get('safety_schedule'):
            job['active_structure']={**self.c['active'],'fitting':self.c['fitting']}
        return native_run(self,job)

    def controls(self,full_structure=False):
        old=self.gpus
        try:
            self.gpus=self.c.get("regression_gpus",old)
            return super().controls(full_structure)
        finally:self.gpus=old

    def dev(self):
        def one(item):
            aid,spec,gpu=item;s=read(resolve(spec['summary']));job=copy.deepcopy(s['selected_job'])
            job.update(output=str(self.out/'dev'/aid),gpu=gpu,episode_id='dev_'+aid,
                       structure_protocol=self.protocol,deadline_shanghai=self.c['runtime']['deadline_shanghai'],
                       wall_clock_budget_s=self.c['runtime']['episode_wall_clock_s'])
            if job.get('mobile_route'):job['mobile_route']['deadline_unix_s']=self.deadline
            r=self.run(job);summary={'asset_id':aid,'dev_not_test':True,'selected_job':job,
                     'selected_report':str(Path(job['output'])/'report.json'),'status':r['status'],
                     'fixed_base_feasible':s['fixed_base_feasible'],'grasp_feasible':True}
            if r.get('bilateral_hold_established'):self.evaluate_episode(summary)
            write(Path(job['output'])/'episode_summary.json',summary)
            return aid,summary
        items=[(aid,spec,self.c['diagnostic_gpus'][i]) for i,(aid,spec) in enumerate(self.c['diagnostic_dev'].items())]
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as p:results=dict(p.map(one,items))
        write(self.out/'dev_results.json',results);return results

    def freeze_method(self):
        controls=read(self.out/'regression_controls.json')
        if not all(x.get('success') for x in controls.values()):raise RuntimeError('REGRESSION_FAILED_NO_FRESH_TEST')
        entered={}
        for aid in self.c['diagnostic_dev']:
            folder=self.out/'dev'/aid;record=folder/'active_refinement.json'
            entered[aid]=bool((folder/'refinement_entered.json').exists() and record.exists() and len(read(record).get('segments',[]))>=1)
        if not all(entered.values()):raise RuntimeError('DEV_REFINEMENT_ENTRY_NOT_VALIDATED:'+str(entered))
        paths=[]
        for pattern in ['active_structure/*.py','scripts/run_active_structure*.py','configs/active_structure.yaml']:
            paths.extend(ROOT.glob(pattern))
        f={'schema':'active-structure-method-v1','frozen_unix_s':time.time(),'active':self.c['active'],
           'dev_refinement_entered':entered,'regression_controls_passed':True,
           'code_sha256':{str(p.relative_to(ROOT)):sha(p) for p in paths},'config':self.c,'GT_selection':False}
        target=self.out/'frozen_method.json'
        if target.exists():
            old=read(target)
            for k in ('active','code_sha256','config'):
                if old[k]!=f[k]:raise RuntimeError('FROZEN_METHOD_CHANGED:'+k)
            return old
        write(target,f);return f

    def freeze(self):
        method=self.freeze_method();m=super().freeze()
        target=self.out/'frozen_test_manifest.json'
        if not target.exists():
            m=copy.deepcopy(m);m.update(method_freeze=str(self.out/'frozen_method.json'),
                                       no_TEST_outcomes_before_freeze=True,discovery_protocol=self.c['active'])
            m['frozen_code_sha256'].update(method['code_sha256'])
            m['missing_assets']=max(0,self.c['assets']['test_assets']-len(m['selected_assets']))
            m['manifest_sha256']=hashlib.sha256(json.dumps({k:v for k,v in m.items() if k!='manifest_sha256'},sort_keys=True).encode()).hexdigest()
            write(self.out/'frozen_asset_manifest.json',m);write(target,m);self.manifest=m
        return m

    def batch(self):
        m=self.freeze();groups=defaultdict(list)
        for e in m['episodes']:groups[e['asset_id']].append(e)
        lanes=[[] for _ in self.gpus]
        for i,eps in enumerate(groups.values()):lanes[i%len(lanes)].extend(eps)
        with concurrent.futures.ProcessPoolExecutor(max_workers=len(self.gpus)) as p:
            result=list(p.map(lane,[(self.c,eps,g) for eps,g in zip(lanes,self.gpus) if eps]))
        with concurrent.futures.ProcessPoolExecutor(max_workers=len(self.gpus)) as p:
            list(p.map(prediction_lane,[(self.c,eps,g) for eps,g in zip(lanes,self.gpus) if eps]))
        return result
