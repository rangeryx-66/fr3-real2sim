"""Observability-aware bounded probes using the unchanged physical runner."""
import sys, json, time, copy, argparse
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT/'scripts'))
import numpy as np
from run_interactive_twin_conditional import Experiment, read, write, absolute, sha
from interactive_twin_conditional.workflow import run_jobs, logs, replay
from interactive_twin_observable.uncertainty import build
from interactive_twin_observable.probes import make_tape, action_travel
from interactive_twin_observable.policy import update, choose_next, finish
from interactive_twin.sysid import NoiseScales, candidate_grid
from interactive_twin.twin import write_twins
from interactive_twin.visual_import import compatible_urdf
from interactive_twin_refinement.physics import score


class ObservableExperiment(Experiment):
    def prepare_episode(self, entry):
        eid = entry['episode_id']; out = self.out/eid
        prepared = out/'prepared.json'
        if prepared.exists(): return read(prepared)
        source = read(absolute(entry['source_job']))
        if entry.get('source_job_key'): source = source[entry['source_job_key']]
        source = {**source, 'conditional_snapshot': str(absolute(entry['snapshot'])),
                  'deadline_shanghai': self.c['deadline_shanghai'],
                  'wall_clock_budget_s': self.c['native_wall_clock_budget_s'], 'continue_manipulation': False}
        source.pop('replay_commands', None); source.pop('only_probe', None)
        if entry.get('structure_actions'):
            actions = [a for a in read(absolute(entry['structure_actions']))['actions']
                       if a['phase'] in ('ESTIMATED_FOLLOW', 'REFINEMENT_REVERSE')]
        else:
            rows = read(absolute(entry['structure_observations']))
            rows = [r for i,r in enumerate(rows) if r['phase']=='ESTIMATED_FOLLOW' and i%8==0]
            actions = [{'phase':'ESTIMATED_FOLLOW', 'time_s':[r['t'] for r in rows],
                        'ee_T':[r['T_tcp'] for r in rows],
                        'observable_anomaly':[r['relative_translation_slip_m']>.003 or min(r['forces_n'].values())<=0 for r in rows]}]
        if entry.get('prior_reverse_actions'):
            reverse = next(a for a in read(absolute(entry['prior_reverse_actions']))['actions'] if a['phase']=='P2')
            actions.append({**reverse, 'phase':'PRIOR_SHORT_REVERSE'})
        center = read(absolute(entry['center_fit']))[entry['center_fit_key']] if entry.get('center_fit') else None
        write(out/'structure_input_actions.json', {'source':'measured EE only', 'actions':actions})
        space = build(actions, self.c['structure_policy'], center)
        write(out/'structure_candidates.json', space)
        if space['status'] != 'COMPLETE':
            write(out/'stop.json', {'stage':'structure_uncertainty', 'status':space['status']}); return None
        scene = read(absolute(entry['scene'])); W = np.eye(4)
        W[:3,:3] = scene['asset_rotation']; W[:3,3] = scene['asset_xyz']
        support = read(source['initial_estimate_memory'])['supporting_observations']
        structures = []
        for r in space['candidates']:
            if not r['accepted']: continue
            dest = out/'models'/r['structure_id']
            write_twins(source['asset_root'], dest, r['estimate'], W,
                        initial_physics_prior=self.c['wrong_prior'], ee_poses=support)
            asset = Path(read(dest/'twin_versions.json')['versions']['T1']['asset_root'])
            m = read(asset/'manifest.json'); compatible_urdf(asset/'urdf'/f"{m['asset_id']}.urdf", asset/'visual_compatibility')
            structures.append({'id':r['structure_id'], 'asset':str(asset), 'estimate':str(dest/'estimated_articulation.json')})
        grid = candidate_grid(self.c['physics_grid']['tau_c'], self.c['physics_grid']['b'],
                              J_eff_prior='frozen_4558a28_mass_and_inertia', budget=9)
        candidates = [{'candidate_id':s['id']+'_'+p['candidate_id'], 'structure_id':s['id'],
                       'tau_c':p['tau_c'], 'b':p['b'], 'asset':s['asset']} for s in structures for p in grid]
        snapshot = read(source['conditional_snapshot']); estimate = read(source['initial_estimate'])
        tapes = {}
        for spec in self.c['probes']+[self.c['heldout']]:
            p = out/'commands'/f"{spec['name']}.json"
            write(p, make_tape(snapshot, estimate, support, spec)); tapes[spec['name']] = str(p)
        data = {'source_job':source, 'structures':structures, 'candidates':candidates,
                'command_tapes':tapes, 'conditions':{k:v or source['plant'] for k,v in entry['conditions'].items()},
                'snapshot_sha256':sha(source['conditional_snapshot']), 'config_sha256':sha(self.out/'frozen_config.json'),
                'max_parameter_points':45, 'reference_read_for_probe_selection':False,
                'initialization':self.c['trial_semantics']}
        write(prepared, data); return data

    def job(self, data, folder, spec, candidate=None, condition=None):
        source = data['source_job']
        job = replay(self, source, folder, candidate['asset'] if candidate else source['asset_root'],
                     {k:candidate[k] for k in ('tau_c','b')} if candidate else data['conditions'][condition],
                     data['command_tapes'][spec['name']])
        job['physics_protocol'] = [{**spec, 'active_windows_s':[w[:2] for w in spec['windows']]}]
        job['role'] = 'twin' if candidate else 'reference'
        return job

    def prediction_bank(self, eid, data):
        out = self.out/eid; jobs = []
        for c in data['candidates']:
            for spec in self.c['probes']:
                jobs.append(self.job(data, out/'bank'/c['candidate_id']/spec['name'], spec, candidate=c))
        run_jobs(self, jobs)
        bank = {}
        for c in data['candidates']:
            predictions = {}; bank[c['candidate_id']] = predictions
            for spec in self.c['probes']:
                folder = out/'bank'/c['candidate_id']/spec['name']; report = read(folder/'report.json')
                records = logs(folder, (spec['probe_id'],)); log = records.get(spec['probe_id'])
                # Pass only public response and safety summaries to the policy;
                # report.evaluation / plant parameters of reference are absent.
                predictions[spec['name']] = {'log':log, 'status':report['status'],
                    'safe':bool(log and report['status']=='REPLAY_COMPLETE' and report['minimum_joint_margin_rad']>.05)}
        write(out/'prediction_bank_index.json', {'candidate_count':len(data['candidates']),
              'native_rollouts':len(jobs), 'actions_per_parameter_point':len(self.c['probes']),
              'reference_future_responses_used':False,
              'failures':[{'candidate_id':cid,'probe':name,'status':r['status']} for cid,p in bank.items() for name,r in p.items() if not r['safe']]})
        return bank

    def adaptive(self, eid, data, bank, condition, noise):
        out = self.out/eid/'adaptive'/condition
        frozen = out/'selection_frozen.json'
        if frozen.exists(): return read(frozen)
        specs = {s['name']:s for s in self.c['probes']}
        observed = {}; attempts = []; state = update(data['candidates'],bank,observed,specs,noise,self.c['selection'])
        for i in range(self.c['maximum_reference_probes']):
            decision = choose_next(state,bank,observed,specs,noise)
            # Decision is durably saved BEFORE the selected physical experiment.
            write(out/f'decision_{i:02d}.json', {**decision,'saved_unix_s':time.time(),
                  'previous_observed_actions':list(observed), 'existing_attempts':attempts})
            name = decision['selected']
            if name is None: break
            spec = specs[name]; folder = out/'reference'/name
            report = self.native(self.job(data,folder,spec,condition=condition))
            records = logs(folder,(spec['probe_id'],)); log = records.get(spec['probe_id'])
            rec = {'name':name,'status':report['status'],'path':str(folder),
                   'min_margin_rad':report.get('minimum_joint_margin_rad'),
                   'peak_load_n':report.get('peak_finger_handle_force_n'),
                   'maximum_detected_contact_drift_m':report.get('maximum_detected_contact_surface_drift_m')}
            if not log or report['status']!='REPLAY_COMPLETE':
                attempts.append(rec); write(out/'attempts.json',attempts); break
            observed[name] = log; rec.update(action_travel(log,spec))
            state = update(data['candidates'],bank,observed,specs,noise,self.c['selection'])
            rec.update(remaining_profile_count=len(state['support']),
                       structure_ids=sorted({r['structure_id'] for r in state['support']}),
                       physics_pairs=sorted({(r['tau_c'],r['b']) for r in state['support']}))
            attempts.append(rec); write(out/'attempts.json',attempts)
            write(out/f'posterior_{i:02d}.json',state)
        selected = finish(state,self.c,len(data['candidates']))
        selected.update(attempts=attempts,probe_count=len(attempts),
                        measured_probe_path_length_m=sum(r.get('path_length_m',0.) for r in attempts),
                        measured_probe_excursion_sum_m=sum(r.get('excursion_m',0.) for r in attempts),
                        saved_unix_s=time.time(), conditional_trials=True)
        write(frozen,selected); return selected

    def heldout(self, eid, data, selections, noise):
        out = self.out/eid/'heldout'; spec = self.c['heldout']
        if (out/'results.json').exists(): return read(out/'results.json')
        unique = {m['candidate_id']:m for s in selections.values() for m in s['methods'].values()}
        jobs = [self.job(data,out/'reference'/condition,spec,condition=condition) for condition,s in selections.items() if s['methods']]
        jobs += [self.job(data,out/'predictions'/cid,spec,candidate=m) for cid,m in unique.items()]
        write(out/'registration.json',{'selection_hashes':{c:sha(self.out/eid/'adaptive'/c/'selection_frozen.json') for c in selections},
               'new_action_frozen_before_training':True,'spec':spec,'source_config_sha256':sha(self.out/'frozen_config.json')})
        run_jobs(self,jobs)
        rows = []
        for condition,s in selections.items():
            ref = logs(out/'reference'/condition,('P4',))
            predictions = {}
            for method,m in s['methods'].items():
                folder = out/'predictions'/m['candidate_id']; pred = logs(folder,('P4',))
                row = {'condition':condition,'method':method,'selected':{k:m[k] for k in ('candidate_id','structure_id','tau_c','b')},
                       'identifiability':s['status'],'parameter_intervals':s.get('parameter_intervals'),
                       'status':read(folder/'report.json')['status']}
                if ref and pred:
                    row['metrics'] = score(ref['P4'],pred['P4'],noise); predictions[method] = pred['P4']
                rows.append(row)
            if ref and predictions:
                from interactive_twin.reporting import plot_heldout_predictions
                plot_heldout_predictions(ref['P4'],predictions,out/f'prediction_{condition}.png')
        result = {'rows':rows,'selection_changed_after_test':False,'actual_native_heldout':True}
        write(out/'results.json',result); return result

    def regression(self, eid, data, selections, noise):
        """Previously seen P4 diagnostic, never changes candidate selection."""
        out = self.out/eid/'regression'
        if (out/'results.json').exists(): return read(out/'results.json')
        source = data['source_job']; spec = self.c['regression_spec']
        protocol = read(ROOT/'configs/interactive_twin_conditional.yaml')['physics_protocol']
        ref = {**source,'output':str(out/'reference/original'),'role':'reference','mode':'physics_reference',
               'only_probe':'P4','physics_protocol':protocol,'plant':data['conditions']['original']}
        self.native(ref)
        tape = out/'reference/original/conditional_tape.json'
        if not tape.exists(): return None
        jobs = []
        for condition,phi in data['conditions'].items():
            if condition!='original':jobs.append(replay(self,ref,out/'reference'/condition,source['asset_root'],phi,tape)|{'role':'reference'})
        unique = {m['candidate_id']:m for s in selections.values() for m in s['methods'].values()}
        for cid,m in unique.items():jobs.append(replay(self,ref,out/'predictions'/cid,m['asset'],{k:m[k] for k in ('tau_c','b')},tape))
        run_jobs(self,jobs); rows=[]
        old = read(absolute(self.c['regression_source']))['rows']
        for condition,s in selections.items():
            reference = logs(out/'reference'/condition,('P4',))
            for method,m in s['methods'].items():
                predicted = logs(out/'predictions'/m['candidate_id'],('P4',))
                row={'condition':condition,'method':method}
                if reference and predicted:
                    metric=score(reference['P4'],predicted['P4'],noise)
                    prior=next(r for r in old if r['condition']==condition and r['method']==method)['test']['metrics']['ee_position_rmse_m']
                    row.update(metrics=metric,previous_RMSE_m=prior,
                               within_registered_tolerance=metric['metrics']['ee_position_rmse_m']<=prior+self.c['selection']['regression_absolute_tolerance_m'])
                rows.append(row)
        result={'scope':'previously seen 3e5e856 P4, regression only, no reselection','rows':rows}
        write(out/'results.json',result);return result

    def run_episode(self, entry, stage):
        eid=entry['episode_id'];data=self.prepare_episode(entry)
        if not data or stage=='prepare':return
        noise=NoiseScales(**read(absolute(self.c['noise_calibration']))['noise_scales'])
        bank=self.prediction_bank(eid,data)
        if stage=='bank':return
        selections={c:self.adaptive(eid,data,bank,c,noise) for c in data['conditions']}
        if stage=='adaptive':return
        if entry.get('center_fit'):
            self.regression(eid,data,selections,noise)
        self.heldout(eid,data,selections,noise)
        # Selected twins are references to immutable compiled model files;
        # physics attributes live in selection, applied natively by plant.py.
        write(self.out/eid/'updated_twins.json',{'methods':{c:{k:{'asset_root':v['asset'],
            'structure_id':v['structure_id'],'tau_c':v['tau_c'],'b':v['b']} for k,v in s['methods'].items()} for c,s in selections.items()},
            'parameter_uncertainty':{c:s.get('parameter_intervals') for c,s in selections.items()},
            'conditional_prediction_only':True,'full_task_replay_this_round':False})


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default='configs/interactive_twin_observable.yaml')
    p.add_argument('--stage',choices=['prepare','bank','adaptive','full','summarize'],default='full')
    p.add_argument('--episode');a=p.parse_args();e=ObservableExperiment(read(absolute(a.config)))
    if a.stage!='summarize':
        for entry in e.c['episodes']:
            if a.episode and entry['episode_id']!=a.episode:continue
            e.run_episode(entry,a.stage)
    from interactive_twin_observable.reporting import summarize
    summarize(e)

if __name__=='__main__':main()
