"""Measured EE-only action interface; privileged diagnostics never enter fitter."""
import json
from pathlib import Path
import numpy as np

def save_actions(output, rows, report):
    actions=[]
    for i,row in enumerate(rows):
        phase=row['phase']
        if phase not in ('EXPLORATORY','ESTIMATED_FOLLOW','REFINEMENT_REVERSE','FINAL_HOLD','P1','P2','P3','P4'): continue
        if not actions or actions[-1]['phase']!=phase:
            actions.append({'phase':phase,'time_s':[],'ee_T':[],'observable_anomaly':[]})
        if i%8:continue
        a=actions[-1];a['time_s'].append(row['t']);a['ee_T'].append(row['T_tcp'])
        a['observable_anomaly'].append(bool(row.get('relative_translation_slip_m',0)>.003 or min(row['forces_n'].values())<=0))
    Path(output,'measured_actions.json').write_text(json.dumps({'source':'measured robot EE, contact-loss flags only; no moving-link trajectory','GT_inputs':False,'actions':actions,'run_status':report['status']}))

def action_poses(action, switch_exclusion_s=.5):
    t=np.asarray(action['time_s']);T=np.asarray(action['ee_T']);bad=np.asarray(action['observable_anomaly'])
    keep=(t>=t[0]+switch_exclusion_s)&(t<=t[-1]-switch_exclusion_s)&~bad
    return T[keep],{'phase':action['phase'],'total':len(t),'kept':int(keep.sum()),'switch_or_contact_excluded':int((~keep).sum())}
