"""Read-only prismatic traction/geometry audit. No hidden truth or pose search."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns

def audit(folder):
    p=Path(folder);a,index=columns(p);meta=index['metadata']
    prior=json.loads((p/'ARTICULATION_INTERFACE.json').read_text())['prior']
    if prior['joint_type']!='prismatic':raise ValueError('Prismatic SI only')
    axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis)
    out=dict(geometry_observation_only=True,axis_world=axis.tolist(),normal_reports_not_complete_wrench=True,segments=[])
    for s in index['segments']:
        ids=np.flatnonzero(a['segment']==s['id'])
        if not len(ids):continue
        t=a['t'][ids]-a['t'][ids[0]];q=a['joint_q'][ids];x=a['tcp_pose'][ids,:3]@axis;u=a['command_TCP_pose'][ids,:3]@axis
        nf=-a['normal_force_on_fingers_world_N'][ids]@axis;ff=-a['friction_force_on_fingers_world_N'][ids]@axis
        valid=np.all(a['friction_api_valid'][ids]>0,axis=1)&np.any(a['friction_anchor_counts'][ids]>0,axis=1)
        ff=ff.copy();ff[~valid]=np.nan # unavailable anchors are not invented zero force
        direction=1 if s['target_q']>=q[0] else -1
        row=dict(id=s['id'],role=s['role'],status=s.get('result',{}).get('status'),drawer_gain_m=float(q[-1]-q[0]),measured_TCP_gain_m=float(x[-1]-x[0]),command_TCP_gain_m=float(u[-1]-u[0]),incremental_servo_deficit_m=float((u[-1]-u[0])-(x[-1]-x[0])),actual_TCP_drawer_relative_m=float((x[-1]-x[0])-(q[-1]-q[0])),normal_rail_N_per_finger_quantiles=np.quantile(nf,[.1,.5,.9],axis=0).tolist(),native_friction_available_samples=int(valid.sum()),friction_rail_N_per_finger_quantiles=np.nanquantile(ff,[.1,.5,.9],axis=0).tolist(),jaw_start_end_m=a['aperture'][ids[[0,-1]]].tolist(),jaw_command_start_end_m=a['command_aperture'][ids[[0,-1]]].tolist(),windows=[])
        for start in np.arange(0,max(t[-1]-.5,0),.5):
            z=np.flatnonzero((t>=start)&(t<start+.5))
            if len(z)<100:continue
            j=ids[z];dx=x[z[-1]]-x[z[0]];dq=q[z[-1]]-q[z[0]]
            row['windows'].append(dict(t_s=float(start),drawer_m=float(dq),tcp_m=float(dx),tcp_drawer_m=float(dx-dq),normal_rail_N=(direction*nf[z].mean(0)).tolist(),friction_rail_N=(direction*np.nanmean(ff[z],axis=0)).tolist(),native_friction_available_samples=int(valid[z].sum()),pad_N=a['pad_load'][j].mean(0).tolist()))
        out['segments'].append(row)
    return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.write_text(json.dumps(audit(args.folder),indent=2))
