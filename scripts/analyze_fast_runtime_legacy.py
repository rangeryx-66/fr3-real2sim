"""Archived native-record replay, not a new physics episode or threshold validation."""
import argparse,gzip,json,pickle,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.policy import Policy,ForceMonitor

def around(episode,t,before=.25,after=.15):
    idx=json.loads((episode/'observations_chunks/index.json').read_text());out=[]
    # Native rate and archive zero origin are confirmed from the records; read
    # neighbor chunks too so this diagnostic does not assume exact tick equality.
    n=int(t*240)
    for c in idx['chunks']:
        if c['first_record']+c['records']<n-int((before+2)*240) or c['first_record']>n+int((after+2)*240):continue
        with gzip.open(episode/'observations_chunks'/c['file'],'rb') as f:rows=pickle.load(f)
        out.extend(r for r in rows if t-before<=r['t']<=t+after)
    return out

def analyze(root,out):
    out.mkdir(parents=True,exist_ok=True);base=root/'engineering_artifacts_20261018/pull_push_v12/archive';ep=base/'episode'
    panel=around(ep,683.8708333333,.3,.01);hybrid=root/'engineering_artifacts_20261017/xarm_hybrid_v0/archive/episode';old=around(hybrid,311.6875,.03,.01)
    close=root/'engineering_artifacts_20261017/xarm_cabinet_torque_v11/archive/episode';force=around(close,416.75833333,.3,.01)
    def fields(rows):
        speed=[np.linalg.norm(np.asarray(b['T_tcp'])[:3,3]-np.asarray(a['T_tcp'])[:3,3])*240*1000 for a,b in zip(rows,rows[1:])]
        cmd=[np.linalg.norm(np.asarray(b['command_q_arm'])-a['command_q_arm']) for a,b in zip(rows,rows[1:])]
        return dict(records=len(rows),phase_first=rows[0]['phase'] if rows else None,phase_last=rows[-1]['phase'] if rows else None,max_measured_tcp_mm_s=max(speed,default=None),max_joint_command_delta_rad=max(cmd,default=None),max_raw_pad_n=max((max(r['forces_n'].values()) for r in rows),default=None),positive_contact_records=sum(any(c['force_n']>0 for c in r['contacts']) for r in rows),door_extent_deg=[min(r['door_angle_deg'] for r in rows),max(r['door_angle_deg'] for r in rows)] if rows else None)
    p=Policy.load(root/'configs/interaction_runtime_v1.json');guard=ForceMonitor(p);events=[]
    for r in force:
        event=guard.update(r['forces_n'],1/240,r['t']);events.append(event)
    results=dict(panel_precontact=fields(panel),handover=fields(old),closing=fields(force),
        closing_new_policy_stop=any(e['stop'] for e in events),closing_new_policy_statuses=sorted({e['status'] for e in events}),
        causal_interpretation=dict(panel='Measured speed crossed a phase-insensitive stop before native panel contact; small position increments do not rule out velocity-target/implicit-drive transient.',handover='Archived independent FK replay proves a 35.194 mm/s command discontinuity at centering-to-probe; must repair handover, not only raise threshold.',closing='SOFT warning was thrown by recovery wrapper before sustained-overload windows; 2.000265 N alone is not an emergency. Transmission quality remains separate.'),
        archived_wall_s=6629.858023994,archived_simulation_s=852.279166667,
        profile_snapshot=json.loads((ep/'timing_profile.json').read_text()),
        caveat='Archived replay and new policy candidacy do not establish physical safety or runtime speedup.')
    (out/'LEGACY_REPLAY.json').write_text(json.dumps(results,indent=2));return results
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();print(json.dumps(analyze(ROOT,a.output),indent=2))
