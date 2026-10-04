"""Post-selection reporting; never returns metrics to parameter selection."""
import csv,json
from pathlib import Path


def summarize(e):
    rows=[];episodes=[]
    for eid in e.c['episodes']:
        out=e.out/eid;p=out/'heldout/results.json'
        record={'episode':eid,'status':'NOT_COMPLETE'}
        if p.exists():
            result=json.loads(p.read_text());record['decisions']=result['decisions']
            selection=json.loads((out/'selection_frozen.json').read_text())['conditions']
            for r in result['rows']:
                m=r.get('test',{}).get('metrics',{});s=selection[r['condition']]
                intervals=s['extension_parameter_intervals'] if r['model'].startswith('D_') else s['base_parameter_intervals']
                row={'asset':eid.split('_')[1],'condition':r['condition'],'model':r['model'],
                     'candidate_id':r['candidate_id'],'validation_residual':r['validation_rms'],
                     'new_heldout_RMSE_mm':1000*m['ee_position_rmse_m'] if m else None,
                     'start_delay_error_s':m.get('start_time_error_s'),
                     'velocity_RMSE_mm_s':1000*m['ee_velocity_rmse_m_s'] if m else None,
                     'dwell_error_mm':1000*m['dwell_incremental_position_rmse_m'] if m and m['dwell_incremental_position_rmse_m'] is not None else None,
                     'final_displacement_error_mm':1000*m['final_displacement_error_m'] if m else None,
                     'diagnostic_parameters':r['parameters'],'parameter_support_range':intervals,
                     'identifiability_status':r['identifiability'],'native_status':r['native_status']}
                rows.append(row)
            record['status']='TEST_COMPLETE' if result['rows'] and all('test' in r for r in result['rows']) else 'TEST_CENSORED'
        regression=out/'regression/results.json'
        if regression.exists():record['old_regression']=json.loads(regression.read_text())
        episodes.append(record)
    result={'mode':'SIM_TO_SIM_BLIND_SYSID','conditional_dynamics_only':True,'new_full_grasp_task_test':False,
            'rows':rows,'episodes':episodes,'no_online_GT_or_state_replay':True,'hardware_friction_identified':False}
    (e.out/'summary.json').write_text(json.dumps(result,indent=2))
    if rows:
        with (e.out/'comparison.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    return result
