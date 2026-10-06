"""Streaming effort report: operation progress and measurement validity differ."""
import json
from pathlib import Path
import numpy as np

def summarize(root):
    root=Path(root);segments=json.loads((root/'effort_segments.json').read_text());by={r['index']:r for r in segments};groups={}
    with (root/'effort_samples.jsonl').open() as stream:
        for line in stream:
            s=json.loads(line);segment=by.get(s['segment'])
            if not segment or segment.get('effort_validity')!='VALID_EFFORT_SEGMENT':continue
            k=s['segment'];a=groups.setdefault(k,{'times':[],'points':[],'forces':[],'torques':[],'full':True})
            full=s.get('full_tangential_measurement',False);a['full'] &=full
            wrench=s.get('command_wrench_world');d=np.asarray(s['direction_world']);F=s.get('effective_tangential_force_magnitude_n') if full else (None if wrench is None else float(abs(np.asarray(wrench[:3])@d)))
            a['times'].append(s['t']);a['points'].append(np.asarray(s['T_ee'])[:3,3]);a['forces'].append(float('nan') if F is None else F);a['torques'].append(s.get('estimated_axis_torque_nm'))
    rows=[]
    for index,a in groups.items():
        if len(a['times'])<3:continue
        t=np.asarray(a['times']);P=np.asarray(a['points']);F=np.asarray(a['forces']);motion=np.linalg.norm(P-P[0],axis=1);v=np.linalg.norm(np.gradient(P,t,axis=0),axis=1)
        moving=(motion>=.00025)&(v>=.00005)&(v<=.00075)&np.isfinite(F);on=np.flatnonzero(motion>=.00025);start=int(on[0]) if len(on) else None
        row=dict(by[index]);row.update(measurement='SIM_NORMAL_PLUS_FRICTION' if a['full'] else 'COMMAND_PROXY_UNCALIBRATED',motion_onset='EE displacement proxy; not independently observed object onset',mean_moving_force_n=float(F[moving].mean()) if moving.any() else None,std_moving_force_n=float(F[moving].std()) if moving.any() else None,mean_actual_EE_speed_m_s=float(v[moving].mean()) if moving.any() else None,
           opening_resistance_onset_force_range_n=[float(np.nanmin(F[:start+1])),float(np.nanmax(F[:start+1]))] if by[index].get('from_rest') and start is not None and np.isfinite(F[:start+1]).any() else None,minimum_static_friction_measured=False)
        rows.append(row)
    doc={'opening_breakaway_effort':[r for r in rows if r['opening_resistance_onset_force_range_n'] is not None],'moving_effort_vs_state':rows,'excluded_segment_count':sum(s.get('effort_validity')!='VALID_EFFORT_SEGMENT' for s in segments),'no_friction_fit':True,'onset_accuracy':'EE proxy; no precise minimum physical resistance claim','sensor_reference':'see full_contact_sensor entries in effort_samples.jsonl'}
    (root/'effort_summary.json').write_text(json.dumps(doc,indent=2));return doc
