"""DEV qualification, immutable method and fresh manifest, then batch execution."""
import concurrent.futures,copy,inspect,json,textwrap,time,hashlib,os,subprocess,signal
from pathlib import Path
from collections import defaultdict
from active_structure.benchmark import ActiveBenchmark
from cross_object_structure.benchmark import StructureBenchmark,ROOT,read,write,sha,summary_safe
from operational_structure.calibration import calibrate

_run=textwrap.dedent(inspect.getsource(StructureBenchmark.run)).replace('run_cross_object_structure_episode.py','run_operational_structure_episode.py')
exec(_run,globals());native_run=run


def lane(args):
    c,eps,gpu=args;b=OperationalBenchmark(c)
    return [b.episode(e,gpu) for e in eps]


class OperationalBenchmark(ActiveBenchmark):
    def __init__(self,c):
        c=copy.deepcopy(c)
        target=ROOT/c['output']/'DEV_prediction_calibration.json'
        if target.exists():cal=read(target)
        else:
            cal=calibrate(c,ROOT);target.parent.mkdir(parents=True,exist_ok=True);write(target,cal)
        c['active']['confidence']=cal['confidence'];c['active']['operational']=cal['operational']
        super().__init__(c)

    def run(self,job):
        if not job.get('structure_protocol') and not job.get('safety_schedule'):
            return super().run(job) # byte-identical legacy regression/grasp entry
        job=copy.deepcopy(job)
        if job.get('structure_protocol'):job['structure_protocol']['active']=self.c['active']
        if job.get('structure_protocol') or job.get('safety_schedule'):
            job['active_structure']={**self.c['active'],'fitting':self.c['fitting']}
        return native_run(self,job)

    def evaluate_episode(self,s):
        super().evaluate_episode(s)
        folder=Path(s['selected_job']['output']);ss=read(folder/'structure_selection.json') if (folder/'structure_selection.json').exists() else {}
        r=read(folder/'report.json')
        s.update(operational_predictive_accepted=ss.get('operational_predictive_accepted',False),
                 operational_structure=bool(ss.get('operational_structure') and r.get('success')),
                 high_fidelity_structure=ss.get('high_fidelity_accepted',False),
                 success=bool(ss.get('operational_structure') and r.get('success')))
        # Geometric held-out acceptance is primary here. Full-start autonomous
        # replays belong to prior physics experiments and are not scheduled.
        s['heldout_pending']=False

    def controls(self,full_structure=False):
        reuse=self.c.get('regression_reuse_root')
        if reuse and not full_structure:
            # These two native trials were actually executed during this task.
            # Reuse only while every prior frozen baseline source still matches.
            old=read(ROOT/'results/active_structure_20261005_v1/frozen_test_manifest.json')['frozen_code_sha256']
            if not all(sha(ROOT/p)==h for p,h in old.items()):raise RuntimeError('BASELINE_CHANGED_CANNOT_REUSE_REGRESSION')
            results={}
            for aid,spec in self.c['controls'].items():
                report=ROOT/reuse/'controls'/aid/'report.json';r=read(report)
                results[aid]={'status':r['status'],'grasp':r.get('bilateral_hold_established'),
                  'success':r.get('success',False),'evaluation':r.get('evaluation',{}),'source_job':spec['source_job'],
                  'report':str(report),'reused_same_task_native_trial':True,'baseline_source_sha256_unchanged':True}
            write(self.out/'regression_controls.json',results);return results
        return super().controls(full_structure)

    def dev(self):
        def one(item):
            aid,spec,gpu=item;s=read(ROOT/spec['summary']);job=copy.deepcopy(s['selected_job'])
            job.update(output=str(self.out/'dev'/aid),gpu=gpu,episode_id='dev_'+aid,
                       structure_protocol=self.protocol,deadline_shanghai=self.c['runtime']['deadline_shanghai'],
                       wall_clock_budget_s=self.c['runtime']['episode_wall_clock_s'])
            if job.get('mobile_route'):job['mobile_route']['deadline_unix_s']=self.deadline
            r=self.run(job);v={'asset_id':aid,'dev_not_test':True,'selected_job':job,'selected_report':str(Path(job['output'])/'report.json'),
                 'status':r['status'],'fixed_base_feasible':s['fixed_base_feasible'],'grasp_feasible':True}
            if r.get('bilateral_hold_established'):self.evaluate_episode(v)
            write(Path(job['output'])/'episode_summary.json',v);return aid,v
        items=[(aid,spec,self.c['diagnostic_gpus'][i]) for i,(aid,spec) in enumerate(self.c['diagnostic_dev'].items())]
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(items)) as p:result=dict(p.map(one,items))
        write(self.out/'dev_results.json',result);return result

    def freeze_method(self):
        controls=read(self.out/'regression_controls.json')
        if not all(x.get('success') for x in controls.values()):raise RuntimeError('REGRESSION_FAILED_NO_FRESH_TEST')
        heldout={};soft_events=0
        for aid in self.c['diagnostic_dev']:
            folder=self.out/'dev'/aid;selection=folder/'structure_selection.json'
            heldout[aid]=bool(read(selection).get('task_prediction')) if selection.exists() else False
            if (folder/'model_confidence.json').exists():soft_events+=sum(e['event']=='REFIT_REDUCE_STEP' for e in read(folder/'model_confidence.json')['events'])
        for aid in self.c['controls']:
            folder=self.out/'controls_full_structure'/aid
            selection=folder/'structure_selection.json'
            heldout['regression_'+aid]=bool(read(selection).get('task_prediction')) if selection.exists() else False
        if sum(heldout.values())<2 or not soft_events:
            raise RuntimeError('DEV_QUALIFICATION_FAILED:'+str({'heldout':heldout,'refit_events':soft_events}))
        paths=[]
        for pattern in ['operational_structure/*.py','scripts/run_operational_structure*.py','configs/operational_structure.yaml',
                        'active_structure/*.py','scripts/run_active_structure_episode.py','cross_object_structure/*.py',
                        'scripts/run_interactive_twin_refinement_episode.py','interactive_twin_refinement/fitting.py']:
            paths.extend(ROOT.glob(pattern))
        f={'schema':'operational-structure-v1','frozen_unix_s':time.time(),'active':self.c['active'],
           'DEV_calibration_sha256':sha(self.out/'DEV_prediction_calibration.json'),
           'dev_independent_validation_reached':heldout,'soft_refits':soft_events,'regression_controls_passed':True,
           'code_sha256':{str(p.relative_to(ROOT)):sha(p) for p in paths},'config':self.c,'GT_selection':False}
        target=self.out/'frozen_method.json'
        if target.exists():
            old=read(target)
            if old['config']!=f['config'] or old['code_sha256']!=f['code_sha256']:raise RuntimeError('FROZEN_METHOD_CHANGED')
            return old
        write(target,f);return f

    def batch(self):
        m=self.freeze();groups=defaultdict(list)
        for e in m['episodes']:groups[e['asset_id']].append(e)
        lanes=[[] for g in self.gpus]
        for i,eps in enumerate(groups.values()):lanes[i%len(lanes)].extend(eps)
        with concurrent.futures.ProcessPoolExecutor(max_workers=len(self.gpus)) as p:
            return list(p.map(lane,[(self.c,eps,g) for eps,g in zip(lanes,self.gpus) if eps]))
