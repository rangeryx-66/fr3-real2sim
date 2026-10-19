"""Compact post-episode metrics from the single stored numeric stream; no actor."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def summarize(p):
    index=json.loads((p/'telemetry/index.json').read_text());columns={}
    for chunk in index['chunks']:
        with np.load(p/'telemetry'/chunk['file'],allow_pickle=False) as packed:
            for key in packed.files:columns.setdefault(key,[]).append(packed[key])
    a={k:np.concatenate(v) for k,v in columns.items()};phases=np.array(index['metadata']['phases'])[a['phase']]
    dt=np.diff(a['t']);speed=np.r_[np.nan,np.linalg.norm(np.diff(a['tcp_pose'][:,:3],axis=0),axis=1)/dt]
    consecutive=np.r_[False,np.diff(a['step'])==1];speed[~consecutive]=np.nan
    config=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());arm=config['arm_indices'];names=config['native_joint_order']
    urdf=ET.parse(p/'sensor_assembly/sim_urdf.urdf').getroot();limits=np.array([[float(urdf.find("joint[@name='%s']/limit"%names[i]).get(k)) for k in ['lower','upper']] for i in arm])
    margin=np.min(np.minimum(a['q'][:,arm]-limits[:,0],limits[:,1]-a['q'][:,arm]),axis=1)
    rows=[]
    for segment in index['segments']:
        ix=a['segment']==segment['id'];steps=a['step'][ix];q=np.rad2deg(a['door_q'][ix]);v=speed[ix]
        rows.append(dict(id=segment['id'],role=segment['role'],result=segment['result'],records=int(ix.sum()),
            continuous_240Hz=bool(len(steps)>1 and np.all(np.diff(steps)==1)),duration_s=float(a['t'][ix][-1]-a['t'][ix][0]),
            start_deg=float(q[0]),end_deg=float(q[-1]),maximum_deg=float(q.max()),minimum_deg=float(q.min()),
            max_TCP_mm_s=float(np.nanmax(v)*1000),minimum_joint_margin_rad=float(margin[ix].min()),
            max_handle_relative_slip_mm=float(a['relative_slip_m'][ix].max()*1000),pad_peaks_n=a['pad_load'][ix].max(0).tolist(),
            positive_normal_unreported_fraction_at_0p05N=np.mean(a['pad_load'][ix]<=.05,axis=0).tolist()))
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text());events=json.loads((p/'events.json').read_text())
    result=dict(physics_steps=index['physics_steps_seen'],stored_records=len(a['step']),unique_stored_steps=bool(len(np.unique(a['step']))==len(a['step'])),
        all_FT_samples_valid=bool(np.all(a['ft_valid'])),recording_issues=index['issues'],segments=rows,
        maximum_actual_deg=float(np.rad2deg(a['door_q'].max())),final_deg=float(np.rad2deg(a['door_q'][-1])),
        peak_pads_n=a['pad_load'].max(0).tolist(),raw_sensor_peak_abs=np.max(abs(a['raw_ft']),axis=0).tolist(),
        interaction_minimum_joint_margin_rad=float(margin[a['segment']>0].min()),sampled_all_phase_minimum_joint_margin_rad=float(margin.min()),
        measured_loaded_TCP_max_mm_s=float(np.nanmax(speed[phases=='OPEN_CONTINUE'])*1000),
        load_warning_samples=int(a['load_warning'].sum()),hard_stop_samples=int(a['safety_stop'].sum()),
        Agent_wait_s=float(sum(e.get('unloaded_wait_s',0) for e in trace)),
        operations=[dict(operation=e['command']['operation'],wall_s=e.get('wall_s'),unloaded_wait_s=e.get('unloaded_wait_s'),reason=e['command'].get('reason')) for e in trace],
        key_events=[e for e in events if e['event'] in ['GRASP_VERIFIED','RELEASED','SAFE_PARK_REACHED','SAFETY_STOP']],
        positive_normal_report_loss_events=sum(e['event']=='PAD_CONTACT_LOST' for e in events),
        scope='one development episode, two acquisitions; missing normal report is not proven separation; no population robustness claim')
    (p/'COMPACT_METRICS.json').write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);a=parser.parse_args();print(json.dumps(summarize(a.episode),indent=2))
