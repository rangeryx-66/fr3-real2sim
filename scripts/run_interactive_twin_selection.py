"""Batch physics selection, optional native breakaway, and frozen new test."""
import sys,json,time,inspect,textwrap,argparse,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
import run_interactive_twin_conditional as parent
from run_interactive_twin_conditional import read,write,sha,absolute
from interactive_twin_conditional.workflow import replay,run_jobs,logs
from interactive_twin_observable.probes import make_tape,action_travel
from interactive_twin_selection import noise
from interactive_twin_selection.metrics import score
from interactive_twin_selection.policy import rank,extension_anchors

# Reuse the complete process/budget/GPU/cache machinery; change only the entry
# adding the optional native passive setup. The physical loop source is untouched.
_native=textwrap.dedent(inspect.getsource(parent.Experiment.native))
assert _native.count('scripts/run_interactive_twin_conditional_episode.py')==2
_namespace=dict(parent.__dict__)
exec(_native.replace('scripts/run_interactive_twin_conditional_episode.py','scripts/run_interactive_twin_selection_episode.py'),_namespace)


def key(estimate):
    if estimate['joint_type']!='revolute':raise ValueError('EXPECTED_EXISTING_REVOLUTE_ESTIMATE')
    a=np.asarray(estimate['axis_world'],dtype=float);a/=np.linalg.norm(a)
    a*=1 if a[np.argmax(abs(a))]>=0 else -1
    c=np.asarray(estimate['point_on_axis_world_m'],dtype=float);line=np.cross(c,a)
    return tuple(np.round(np.r_[a,line],13))


def public_cell(folder,phase):
    folder=Path(folder);r=read(folder/'report.json') if (folder/'report.json').exists() else {}
    log=logs(folder,(phase,)).get(phase)
    return {'safe':bool(log and r.get('status') in ('REPLAY_COMPLETE','PHYSICS_PROTOCOL_COMPLETE') and r.get('minimum_joint_margin_rad',0)>.05),
            'log':log,'status':r.get('status','MISSING'),'path':str(folder)}


class Experiment(parent.Experiment):
    native=_namespace['native']

    def __init__(self,config):
        super().__init__(config)
        files=sorted((ROOT/'interactive_twin_selection').glob('*.py'))+[
            Path(__file__).resolve(),ROOT/'scripts/run_interactive_twin_selection_episode.py',
            ROOT/'scripts/run_interactive_twin_conditional_episode.py',
            ROOT/'scripts/run_interactive_twin_conditional.py',
            ROOT/'interactive_twin_conditional/workflow.py',ROOT/'interactive_twin_observable/probes.py',
            ROOT/'interactive_twin/plant.py']
        hashes={str(p.relative_to(ROOT)):sha(p) for p in files}
        target=self.out/'method_frozen.json'
        if target.exists() and read(target)['source_hashes']!=hashes:raise RuntimeError('FROZEN_METHOD_CHANGED')
        if not target.exists():write(target,{'source_hashes':hashes,'saved_unix_s':time.time(),
                                           'base_commit':config['base_commit'],'new_test_run':False})

    def calibration(self):
        out=self.out/'calibration';out.mkdir(exist_ok=True)
        f=out/'selection_noise.json'
        if f.exists():return read(f)
        return noise.calibrate(noise.extract(ROOT,self.c['calibration'],out),self.c['calibration'],out)

    def prepare_episode(self,eid):
        out=self.out/eid;cached=out/'prepared.json'
        if cached.exists():return read(cached)
        prior=absolute(self.c['prior_observable_output'])/eid;old=read(prior/'prepared.json')
        source={**old['source_job'],'deadline_shanghai':self.c['deadline_shanghai'],
                'wall_clock_budget_s':self.c['native_wall_clock_budget_s']}
        structures=[];seen={};duplicates=[]
        sources=[('observable',prior),('historical',absolute(self.c['historical_conditional_output'])/eid)]
        for family,folder in sources:
            for model in sorted((folder/'models').glob('S*')):
                est=model/'estimated_articulation.json'
                if not est.exists():continue
                estimate=read(est);identity=key(estimate)
                if identity in seen:
                    duplicates.append({'source':str(model),'same_axis_line_as':seen[identity]});continue
                sid=family+'_'+model.name;seen[identity]=sid
                structures.append({'structure_id':sid,'asset':str(model/'T1'),
                                   'estimate_path':str(est),'estimate_sha256':sha(est),'source_family':family})
        if len(structures)>self.c['maximum_structures']:raise ValueError('STRUCTURE_BUDGET')
        candidates=[]
        for s in structures:
            for i,(c,b) in enumerate((c,b) for c in self.c['physics_grid']['tau_c'] for b in self.c['physics_grid']['b']):
                cid=s['structure_id']+f'_phi_{i:03d}'
                candidates.append({'candidate_id':cid,'structure_id':s['structure_id'],'asset':s['asset'],
                                   'tau_c':c,'b':b,'tau_s':c,'model_family':'dynamic_viscous'})
        if len(candidates)>self.c['maximum_base_parameter_points']:raise ValueError('PHYSICS_BUDGET')
        spec=self.c['heldout'];support=read(source['initial_estimate_memory'])['supporting_observations']
        test_tape=out/'commands'/f"{spec['name']}.json"
        write(test_tape,make_tape(read(source['conditional_snapshot']),read(source['initial_estimate']),support,spec))
        tapes={**old['command_tapes'],spec['name']:str(test_tape)}
        result={'source_job':source,'structures':structures,'duplicate_structures':duplicates,'candidates':candidates,
                'conditions':old['conditions'],'command_tapes':tapes,'prior_output':str(prior),
                'snapshot_sha256':sha(source['conditional_snapshot']),'new_test_tape_sha256':sha(test_tape),
                'selection_has_not_started':True,'structure_fitter_called':False,
                'candidate_source':'all pre-existing train-derived axis/axis-line candidates; no held-out-selected-only anchor'}
        write(cached,result);return result

    def job(self,data,folder,spec,candidate=None,condition=None):
        phi={k:candidate[k] for k in ('tau_c','b')} if candidate else data['conditions'][condition]
        if candidate and candidate['model_family']=='static_dynamic_viscous':phi['tau_s']=candidate['tau_s']
        result=replay(self,data['source_job'],folder,candidate['asset'] if candidate else data['source_job']['asset_root'],
                      phi,data['command_tapes'][spec['name']])
        result.update(physics_protocol=[{**spec,'active_windows_s':[w[:2] for w in spec['windows']]}],
                      role='twin' if candidate else 'reference')
        return result

    def bank(self,eid,data,candidates,extension=False):
        bank={};jobs=[];destinations={}
        for c in candidates:
            for spec in self.c['probes']:
                folder=self.out/eid/('static_bank' if extension else 'bank')/c['candidate_id']/spec['name']
                if c['structure_id'].startswith('observable_') and not extension:
                    oldcid=c['candidate_id'].removeprefix('observable_')
                    original=Path(data['prior_output'])/'bank'/oldcid/spec['name']
                    if (original/'report.json').exists():folder=original
                destinations[(c['candidate_id'],spec['name'])]=folder
                if not (folder/'report.json').exists():jobs.append(self.job(data,folder,spec,candidate=c))
        run_jobs(self,jobs)
        for c in candidates:
            bank[c['candidate_id']]={spec['name']:public_cell(destinations[(c['candidate_id'],spec['name'])],spec['probe_id']) for spec in self.c['probes']}
        write(self.out/eid/('static_bank_index.json' if extension else 'base_bank_index.json'),
              {'parameter_points':len(candidates),'new_native_trials':len(jobs),'action_count':len(self.c['probes']),
               'paths':[str(v) for v in destinations.values()],
               'failures':[{'candidate':cid,'probe':name,'status':v['status']} for cid,cells in bank.items() for name,v in cells.items() if not v['safe']]})
        return bank

    def references(self,eid,data,bank):
        # No posterior or early-response pruning is consulted. Only safety and
        # the unchanged observed 1mm rule determine reverse availability.
        result={};inventory={}
        for condition in data['conditions']:
            refs={};records=[]
            for spec in self.c['probes']:
                name=spec['name'];folder=Path(data['prior_output'])/'adaptive'/condition/'reference'/name
                if not (folder/'report.json').exists():
                    folder=self.out/eid/'reference'/condition/name
                    if name=='reverse' and not all(cells[name]['safe'] for cells in bank.values()):
                        records.append({'name':name,'status':'REVERSE_PREDICTED_UNSAFE','usable':False});continue
                    self.native(self.job(data,folder,spec,condition=condition))
                cell=public_cell(folder,spec['probe_id']);motion=action_travel(cell['log'],spec) if cell['log'] else {}
                usable=cell['safe'] and (name!='reverse' or motion['excursion_m']>=.001)
                records.append({'name':name,'path':str(folder),'status':cell['status'],'usable':usable,**motion,
                                'reason':'REVERSE_BELOW_UNCHANGED_1MM' if name=='reverse' and cell['safe'] and not usable else None})
                if usable:refs[name]=cell['log']
            result[condition]=refs;inventory[condition]=records
        write(self.out/eid/'available_probes.json',{'conditions':inventory,'early_candidate_pruning':False})
        return result

    def select(self,eid,data,bank,reference,cal):
        out=self.out/eid;f=out/'selection_frozen.json'
        if f.exists():return read(f)
        base={condition:rank(data['candidates'],bank,refs,cal,self.c['selection']) for condition,refs in reference.items()}
        write(out/'base_rankings.json',base)
        extensions={};conditions={};extra={}
        for condition,r in base.items():
            if r.get('all_candidates_validation_inadequate'):
                anchors=extension_anchors(r,self.c['static_extension'])
                for x in anchors:extra[x['candidate_id']]=x
                extensions[condition]=anchors
        # The cap applies to the union across reference resistance conditions.
        # Rank shared anchors by train-only performance across those conditions.
        if len(extra)>self.c['static_extension']['maximum_extra_parameter_points']:
            anchor_scores={}
            for r in base.values():
                for row in r['rows']:anchor_scores.setdefault(row['candidate_id'],[]).append(row['train_loss'])
            ordered=sorted(extra.values(),key=lambda x:(np.mean(anchor_scores[x['anchor_candidate']]),x['candidate_id']))
            extra={x['candidate_id']:x for x in ordered[:self.c['static_extension']['maximum_extra_parameter_points']]}
            extensions={condition:list(extra.values()) for condition in extensions}
        write(out/'static_grid_frozen.json',{'candidates':list(extra.values()),'maximum':self.c['static_extension']['maximum_extra_parameter_points'],
                                           'triggers':list(extensions),'new_test_read':False})
        extra_bank=self.bank(eid,data,list(extra.values()),True) if extra else {}
        for condition,r in base.items():
            if not r['rows']:
                conditions[condition]={'status':'NO_COMPLETE_CANDIDATE','methods':{}};continue
            prior=[x for x in r['rows'] if all(x[k]==self.c['wrong_prior'][k] for k in ('tau_c','b'))]
            fixed=next(x for x in prior if x['structure_id']=='observable_S0')
            methods={'A_wrong_prior':fixed,'B_structure_only':min(prior,key=lambda x:x['combined_loss']),'C_two_parameter':r['best']}
            extension_gate=False;ext=None
            if condition in extensions:
                ext=rank(extensions[condition],extra_bank,reference[condition],cal,self.c['selection'])
                if ext['rows']:
                    methods['D_static_extension']=ext['best']
                    extension_gate=bool(ext['model_adequate'] and ext['best']['validation_loss']<(1-self.c['selection']['extension_validation_improvement_fraction'])*r['best']['validation_loss'])
            conditions[condition]={'methods':methods,'base_status':r['status'],'base_support':r['support'],
                'base_parameter_intervals':r['parameter_intervals'],'base_retained_parameters':r['retained_parameters'],
                'base_min_validation_rms':r['minimum_validation_rms'],'base_adequate':r['model_adequate'],
                'extension_triggered':condition in extensions,'extension_validation_gate':extension_gate,
                'extension_status':None if ext is None else ext['status'],
                'extension_parameter_intervals':None if ext is None else ext.get('parameter_intervals'),
                'extension_support':None if ext is None else ext.get('support')}
            if ext:write(out/f'extension_ranking_{condition}.json',ext)
        frozen={'conditions':conditions,'saved_unix_s':time.time(),'new_test_read':False,'calibration_sha256':cal['calibration_sha256'],
                'all_usable_actions_before_selection':True,'q_in_physics_score':False}
        write(f,frozen);return frozen

    def historical_training_review(self,eid,cal):
        """Old complete P1/P2/P3 only. Never opens its P4 or success table."""
        old=absolute(self.c['historical_conditional_output'])/eid
        grid=old/'grid_frozen.json';dest=self.out/eid/'historical_training_review.json'
        if dest.exists() or not grid.exists():return
        candidates=read(grid)['candidates'];bank={};cs=[]
        mapping={'legacy_open':'P1','legacy_reverse':'P2','stop_dwell':'P3'}
        for c in candidates:
            item={**c,'tau_s':c['tau_c'],'model_family':'dynamic_viscous','asset':str(old/'models'/c['structure_id']/'T1')}
            cs.append(item);bank[c['candidate_id']]={k:public_cell(c['path'],v) for k,v in mapping.items()}
        result={}
        for folder in (old/'reference').iterdir():
            refs={k:logs(folder,(v,)).get(v) for k,v in mapping.items()}
            if not all(refs.values()):continue
            result[folder.name]=rank(cs,bank,refs,cal,self.c['selection'])
        write(dest,{'conditions':result,'P4_read':False,'weight_tuning_from_this_result':False,
                    'purpose':'training/validation reselection of historical complete candidate bank'})

    def regression(self,eid,data,selection,cal):
        """Previously seen P4 diagnostic after selection; no method feedback."""
        old=absolute(self.c['historical_conditional_output'])/eid/'common_start_heldout'
        if not (old/'results.json').exists():return
        out=self.out/eid/'regression';f=out/'results.json'
        if f.exists():return read(f)
        template=read(old/'reference/original/job_private.json')
        tape=old/'reference/original/conditional_tape.json'
        candidates={m['candidate_id']:m for s in selection['conditions'].values() for k,m in s['methods'].items() if k in ('C_two_parameter','D_static_extension')}
        jobs=[]
        for cid,m in candidates.items():
            phi={k:m[k] for k in ('tau_c','b')}
            if m['model_family']=='static_dynamic_viscous':phi['tau_s']=m['tau_s']
            j=replay(self,template,out/'predictions'/cid,m['asset'],phi,tape)
            j.update(deadline_shanghai=self.c['deadline_shanghai'],wall_clock_budget_s=self.c['native_wall_clock_budget_s'])
            jobs.append(j)
        run_jobs(self,jobs);oldrows=read(old/'results.json')['rows'];rows=[]
        for condition,s in selection['conditions'].items():
            reference=public_cell(old/'reference'/condition,'P4')
            prior=next(r for r in oldrows if r['condition']==condition and r['method']=='uncertain_structure_calibrated')
            previous=prior['test']['metrics']['ee_position_rmse_m']
            for label,m in s['methods'].items():
                if label not in ('C_two_parameter','D_static_extension'):continue
                predicted=public_cell(out/'predictions'/m['candidate_id'],'P4')
                row={'condition':condition,'model':label,'candidate_id':m['candidate_id'],'previous_success_RMSE_m':previous,'status':predicted['status']}
                if reference['safe'] and predicted['safe']:
                    row['prediction']=score(reference['log'],predicted['log'],cal,evaluation=True)
                    row['within_tolerance']=row['prediction']['metrics']['ee_position_rmse_m']<=previous+self.c['selection']['regression_absolute_tolerance_m']
                rows.append(row)
        result={'scope':'old seen P4, not the new final test','selection_changed':False,'rows':rows}
        write(f,result);return result

    def test(self,eid,data,selection,cal):
        out=self.out/eid/'heldout';f=out/'results.json'
        if f.exists():return read(f)
        selected=selection['conditions'];unique={m['candidate_id']:m for s in selected.values() for m in s['methods'].values()}
        spec=self.c['heldout'];jobs=[self.job(data,out/'reference'/c,spec,condition=c) for c,s in selected.items() if s['methods']]
        jobs.extend(self.job(data,out/'predictions'/cid,spec,candidate=m) for cid,m in unique.items())
        write(out/'registration.json',{'spec':spec,'tape_sha256':data['new_test_tape_sha256'],
              'selection_sha256':sha(self.out/eid/'selection_frozen.json'),'new_command_not_used_to_design_scoring':True})
        run_jobs(self,jobs);rows=[]
        for condition,s in selected.items():
            ref=public_cell(out/'reference'/condition,'P4');plots={}
            for label,m in s['methods'].items():
                pred=public_cell(out/'predictions'/m['candidate_id'],'P4')
                row={'episode':eid,'condition':condition,'model':label,'candidate_id':m['candidate_id'],
                     'structure_id':m['structure_id'],'parameters':{k:m[k] for k in ('tau_c','b','tau_s')},
                     'validation_rms':m['validation_loss']**.5,'native_status':pred['status'],
                     'identifiability':s['extension_status'] if label.startswith('D_') else s['base_status']}
                if ref['safe'] and pred['safe']:
                    row['test']=score(ref['log'],pred['log'],cal,evaluation=True);plots[label]=pred['log']
                rows.append(row)
            if ref['safe'] and plots:
                from interactive_twin.reporting import plot_heldout_predictions
                plot_heldout_predictions(ref['log'],plots,out/f'prediction_{condition}.png')
        decisions=[]
        for c,s in selected.items():
            models={r['model']:r for r in rows if r['condition']==c};B=models.get('B_structure_only',{}).get('test');C=models.get('C_two_parameter',{}).get('test');D=models.get('D_static_extension',{}).get('test')
            improve=lambda x,y:bool(x and y and x['metrics']['ee_position_rmse_m']<(1-self.c['selection']['heldout_improvement_fraction'])*y['metrics']['ee_position_rmse_m'])
            accept_extension=bool(s['extension_validation_gate'] and improve(D,C))
            accepted=D if accept_extension else C
            good=bool((accept_extension or s['base_adequate']) and improve(accepted,B))
            decisions.append({'condition':c,'extension_accepted':accept_extension,'accepted_family':'static_dynamic_viscous' if accept_extension else 'dynamic_viscous',
                              'physics_prediction_transfer_success':good,'updated_vs_structure_improved':improve(accepted,B),
                              'status':'VALIDATED_PREDICTOR_PARAMETERS_UNIDENTIFIABLE' if good else ('MODEL_MISMATCH' if not s['base_adequate'] else 'NO_HELDOUT_PHYSICS_IMPROVEMENT'),
                              'test_used_only_for_final_acceptance_not_parameter_reselection':True})
        result={'rows':rows,'decisions':decisions,'actual_native_test':bool(jobs),'selection_changed_after_test':False}
        write(f,result);return result

    def run_episode(self,eid,stage):
        cal=self.calibration();data=self.prepare_episode(eid)
        self.historical_training_review(eid,cal)
        if stage=='prepare':return
        bank=self.bank(eid,data,data['candidates'])
        if stage=='bank':return
        reference=self.references(eid,data,bank)
        selection=self.select(eid,data,bank,reference,cal)
        if stage=='select':return
        self.regression(eid,data,selection,cal)
        self.test(eid,data,selection,cal)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default='configs/interactive_twin_selection.yaml')
    p.add_argument('--stage',choices=['calibration','prepare','bank','select','full','summarize'],default='full');p.add_argument('--episode')
    a=p.parse_args();e=Experiment(read(absolute(a.config)))
    if a.stage=='calibration':e.calibration();return
    if a.stage!='summarize':
        for eid in e.c['episodes']:
            if a.episode and eid!=a.episode:continue
            e.run_episode(eid,a.stage)
    from interactive_twin_selection.reporting import summarize
    summarize(e)

if __name__=='__main__':main()
