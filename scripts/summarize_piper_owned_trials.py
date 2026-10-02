"""Summarize measured fixed-base trials, without interpreting previews as grasps."""
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--search',nargs='+',type=Path,required=True)
    p.add_argument('--trials',nargs='+',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();out={'searches':[],'physical_trials':[]}
    for path in a.search:
        d=json.loads(path.read_text());rows=d['rows']
        out['searches'].append({'path':str(path),'tested':len(rows),'exact_ik':sum(r['status']!='NO_IK' for r in rows),'complete_open_approach':sum('q_grasp' in r for r in rows),'pregrasp_routes':len(d.get('trial_candidates_before_stop',d.get('planned_trial_candidates_before_stop',d.get('trial_candidates',[])))),'status_counts':dict(Counter(r['status'].split(':',1)[0] for r in rows)),'base':d['base'],'bounds':d['bounds'],'search_frame':d['search_frame'],'ownership_sha256':d['ownership_sha256']})
    for parent in a.trials:
        for path in sorted(parent.glob('*/report.json')):
            d=json.loads(path.read_text());physics=json.loads((path.parent/'physics_steps.json').read_text())
            close=[s for s in physics if s['phase'] in ('CLOSE','CLOSURE_HOLD')];last=physics[-1] if physics else None
            terminal=None if last is None else {'phase':last['phase'],'actual_aperture_m':last['aperture_m'],'finger_joint_positions_m':last['q'][6:],'pad_forces_n':last['ownership']['pad_forces_n'],'forbidden_force_n':sum(c['force_n'] for c in last['ownership']['contacts'] if c['owner']!='pad' or not c['allowed_pad_target']),'active_forbidden_contacts':[c for c in last['ownership']['contacts'] if c['force_n']>0 and (c['owner']!='pad' or not c['allowed_pad_target'])]}
            audits={name:json.loads(f.read_text())['status'] for name in ('closure','pull') if (f:=path.parent/(name+'_response.json')).exists()}
            out['physical_trials'].append({'variant':d['selected']['variant'],'source':str(path),'region':d['selected'].get('region'),'pad_anchor_world':d['selected'].get('pad_anchor_world'),'raw_translation_delta_m':d['selected'].get('raw_translation_delta_m'),'raw_rotation_delta_deg':d['selected'].get('raw_rotation_delta_deg'),'offset_handle_m':d['selected']['offset_handle_m'],'rpy_handle_deg':d['selected']['rpy_handle_deg'],'status':d['status'],'actual_closure_attempted':bool(close),'legal_closure':d.get('legal_closure',False),'legal_pull':d.get('legal_pull',False),'legal_real_grasp':d.get('legal_real_grasp',False),'audit_status':audits,'terminal':terminal,'minimum_joint_margin_rad':d['minimum_joint_margin_rad'],'maximum_relative_slip_m':d['maximum_relative_slip_m'],'stage_tests':d['stage_tests'],'predicted_aperture_ranking_only_m':d['selected'].get('predicted_width_m'),'closure_terminal_aperture_m':close[-1]['aperture_m'] if close else None,'measured_pull_displacement_m':d.get('measured_pull_displacement_m'),'door_opening_arc_executed':d['door_opening_arc_executed']})
            bad=next((s for s in physics if s['ownership']['metal_contacts'] or s['ownership']['pad_target_violations']),None)
            out['physical_trials'][-1]['first_forbidden_contact']=None if bad is None else {'phase':bad['phase'],'aperture_m':bad['aperture_m'],'pad_forces_n':bad['ownership']['pad_forces_n'],'joint_margin_rad':bad['margin_rad'],'contacts':[c for c in bad['ownership']['contacts'] if c['force_n']>0 and (c['owner']!='pad' or not c['allowed_pad_target'])]}
    trials=out['physical_trials'];out['counts']={k:sum(bool(r[k]) for r in trials) for k in ['actual_closure_attempted','legal_closure','legal_pull','legal_real_grasp','door_opening_arc_executed']};out['counts']['physical_trials']=len(trials)
    out['failure_counts']=dict(Counter(r['status'].split(':',1)[0] for r in trials));out['opening_stages']={str(goal):{'attempted':sum(any(s['requested_angle_deg']==goal for s in r['stage_tests']) for r in trials),'passed':sum(any(s['requested_angle_deg']==goal and s['status']=='STAGE_PASSED' for s in r['stage_tests']) for r in trials)} for goal in [1,5,10,22]}
    errors=[r['closure_terminal_aperture_m']-r['predicted_aperture_ranking_only_m'] for r in trials if r['legal_closure'] and r['predicted_aperture_ranking_only_m'] is not None]
    out['accepted_closure_aperture_error_m']=None if not errors else {'min':min(errors),'max':max(errors),'mean':float(np.mean(errors))}
    out['note']='Rejected closures stop at their first forbidden contact; their terminal aperture is NOT a completed closure. Unattempted opening stages are not failures at those angles.'
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2));print(json.dumps({'counts':out['counts'],'failures':out['failure_counts'],'opening_stages':out['opening_stages']},indent=2))


if __name__=='__main__':main()
