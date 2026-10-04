"""Truth is not consumed here: compare observable prediction and safety only."""
import csv, json
from pathlib import Path


def read(p): return json.loads(Path(p).read_text())


def plots(out):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p=out/'structure_candidates.json'
    if p.exists():
        space=read(p);samples=[s['delta'] for s in space.get('segments',[]) if 'delta' in s]
        if samples:
            V=np.asarray(samples);C=np.asarray([s['delta'] for s in space['candidates']])
            fig,axs=plt.subplots(1,2,figsize=(9,4))
            for ax,j,scale,label in [(axs[0],0,180/np.pi,'axis tangent coordinates (deg)'),
                                     (axs[1],2,1000,'axis-line gauge coordinates (mm)')]:
                ax.scatter(V[:,j]*scale,V[:,j+1]*scale,s=25,label='segment / block fit')
                ax.scatter(C[:,j]*scale,C[:,j+1]*scale,marker='x',s=80,label='bounded candidates')
                ax.set_xlabel(label+' 1');ax.set_ylabel(label+' 2');ax.grid(alpha=.25);ax.legend()
            fig.suptitle('EE-only local structure uncertainty; no GT selection')
            fig.tight_layout();fig.savefig(out/'structure_uncertainty.png');plt.close(fig)
    for folder in sorted((out/'adaptive').glob('*')) if (out/'adaptive').exists() else []:
        p=folder/'selection_frozen.json'
        if not p.exists():continue
        s=read(p);attempts=s['attempts']
        fig,axs=plt.subplots(1,2,figsize=(10,4))
        axs[0].plot(range(len(attempts)+1),[s.get('initial_candidate_count',45)]+[a.get('remaining_profile_count',0) for a in attempts],'o-')
        axs[0].set_xlabel('Completed reference probe');axs[0].set_ylabel('Training profile candidate count')
        axs[0].set_title('Profile support (not identifiability claim)');axs[0].grid(alpha=.25)
        axs[1].bar([a['name'] for a in attempts],[1000*a.get('excursion_m',0) for a in attempts])
        axs[1].tick_params(axis='x',rotation=25);axs[1].set_ylabel('Measured excursion (mm)')
        axs[1].set_title('Reverse column, if present, excludes forward prefix')
        fig.suptitle(s['status']);fig.tight_layout();fig.savefig(folder/'probe_summary.png');plt.close(fig)


def summarize(e):
    rows=[]; episodes=[]
    for entry in e.c['episodes']:
        eid=entry['episode_id'];out=e.out/eid
        record={'episode':eid,'status':'NOT_STARTED'}
        sfile=out/'structure_candidates.json'
        if sfile.exists():
            space=read(sfile);record.update(status=space['status'],unavailable_structure_actions=space.get('unavailable_actions',[]),
                                          accepted_structures=sum(c['accepted'] for c in space['candidates']))
        p=out/'heldout/results.json'
        if p.exists():
            for r in read(p)['rows']:
                s=read(out/'adaptive'/r['condition']/'selection_frozen.json')
                m=r.get('metrics',{});metrics=m.get('metrics',{});d=m.get('post_drive_drift_prediction');rd=m.get('post_drive_drift_reference')
                row={'episode':eid,'condition':r['condition'],'method':r['method'],
                     'status':r['status'],'identifiability':r['identifiability'],
                     'heldout_RMSE_mm':1000*metrics['ee_position_rmse_m'] if metrics else None,
                     'velocity_RMSE_mm_s':1000*metrics['ee_velocity_rmse_m_s'] if metrics else None,
                     'start_delay_error_s':metrics.get('start_time_error_s'),
                     'final_displacement_error_mm':1000*metrics['final_displacement_error_m'] if metrics else None,
                     'drift_error_mm':1000*abs(d['net_drift_m']-rd['net_drift_m']) if d and rd else None,
                     **r['selected'],'parameter_intervals':r['parameter_intervals'],
                     'profile_count':s.get('profile_support_count'),'feasible_count':s.get('feasible_set_count'),
                     'probe_count':s['probe_count'],'probe_order':[a['name'] for a in s['attempts']],
                     'probe_path_mm':1000*s['measured_probe_path_length_m'],
                     'probe_excursion_sum_mm':1000*s['measured_probe_excursion_sum_m']}
                rows.append(row)
            record['status']='HELDOUT_COMPLETE'
        episodes.append(record)
        if out.exists():plots(out)
    result={'mode':'SIM_TO_SIM_BLIND_SYSID','conditional_prediction_only':True,'full_task_replay_this_round':False,
            'episodes':episodes,'rows':rows,'real_friction_measurement':False}
    (e.out/'summary.json').write_text(json.dumps(result,indent=2))
    if rows:
        with (e.out/'comparison.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    return result
