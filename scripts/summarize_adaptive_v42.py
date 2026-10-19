"""Compact derived engineering summary; raw arrays/video remain private."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns
from interaction_runtime.kinematics import FastNativeModel
from interaction_runtime.compact_v2 import storage
from interaction_runtime.serialization import result_packet

def summarize(p):
    episode=json.loads((p/'EPISODE_RESULT.json').read_text());a,idx=columns(p);job=json.loads((p/'job.json').read_text());runtime=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());arm=runtime['arm_indices']
    model=FastNativeModel(None,Path(job['asset_root']),json.loads((Path(job['source'])/'report.json').read_text()))
    points=a['tcp_pose'][:,:3];adj=np.diff(a['step'])==1;v=np.linalg.norm(np.diff(points,axis=0),axis=1)/np.diff(a['t']);rows=[]
    for s in idx['segments']:
        ix=np.flatnonzero(a['segment']==s['id']);local=np.diff(a['segment'])==0;mask=adj&local&(a['segment'][1:]==s['id'])
        rows.append(dict(id=s['id'],role=s['role'],records=len(ix),result=s.get('result'),actual_range_deg=np.rad2deg([a['door_q'][ix].min(),a['door_q'][ix].max()]).tolist(),
            pad_peaks_n=a['pad_load'][ix].max(0).tolist(),max_slip_mm=float(a['relative_slip_m'][ix].max()*1000),max_TCP_mm_s=float(v[mask].max()*1000) if mask.any() else None,
            max_joint_tracking_error_rad=float(abs(a['command_q'][ix]-a['q'][ix][:,arm]).max())))
    margins=np.minimum(a['q'][:,arm]-model.limits[:,0],model.limits[:,1]-a['q'][:,arm]);events=json.loads((p/'events.json').read_text());attempt=episode.get('attempt',{})
    hashes=[]
    for c in idx['chunks']:hashes.append(dict(file=c['file'],sha256=c['sha256'],verified=hashlib.sha256((p/'telemetry'/c['file']).read_bytes()).hexdigest()==c['sha256']))
    return result_packet(dict(private_root=str(p),classification=episode['status'],engineering_fixture_not_research=True,segments=rows,continuations=attempt.get('continuations'),acquisition=attempt.get('acquisition'),safe_exit=attempt.get('safe_exit',{}).get('safe'),
        parking=attempt.get('safe_exit',{}).get('park'),error=attempt.get('error'),minimum_joint_margin_rad=float(margins.min()),all_ft_valid=bool(a['ft_valid'].all()),hard_stops=sum(e['event']=='SAFETY_STOP' for e in events),
        records=len(a['t']),numeric_export_issues=idx['issues'],source_provenance=json.loads((p/'SOURCE_PROVENANCE.json').read_text()),profile=json.loads((p/'RUNTIME_RESULT.json').read_text())['profile'],
        completion=json.loads((p/'PROCESS_COMPLETION.json').read_text()),storage=storage(p),raw_integrity=hashes,guards=episode.get('guards'),park_before_deliberate_faults=episode.get('park_before_deliberate_guard_faults')))
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args();a.output.write_text(json.dumps(summarize(a.episode),indent=2))
