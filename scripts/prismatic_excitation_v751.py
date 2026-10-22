"""Read-only actual, qualified motion excitation from sealed recordings."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from update_probing_v7_model import load_segments
from update_prismatic_v73_model import qualify


def run(episode,output):
    p=Path(episode);seal=json.loads((p/'ONLINE_SEAL_V751.json').read_text())
    assert seal['actor_exited'] and seal['online_actions_finished']
    for name,digest in seal['sha256'].items():assert hashlib.sha256((p/name).read_bytes()).hexdigest()==digest
    pieces,_,_=load_segments(p)
    rows=[]
    for d in pieces:
        audit=qualify(d);valid=d['valid'];n=int(valid.sum())
        rows.append(dict(block=d['block_id'],qualification=audit,retained_native_samples=n,
            retained_motion_seconds=n*d['processing']['dt_s'],
            native_velocity_RMS_m_s=float(np.sqrt(np.mean(d['native_qd'][valid]**2)))if n else None,
            regression_velocity_RMS_m_s=float(np.sqrt(np.mean(d['qd'][valid]**2)))if n else None,
            regression_acceleration_RMS_m_s2=float(np.sqrt(np.mean(d['qdd'][valid]**2)))if n else None,
            supported_position_m=[float(d['q'][valid].min()),float(d['q'][valid].max())]if n else None))
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(dict(episode=p.name,post_seal_read_only=True,truth_used=False,refitting_performed=False,blocks=rows),indent=2)+'\n')
    print(p.name,len(rows))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode');ap.add_argument('--output',required=True);a=ap.parse_args();run(a.episode,a.output)
