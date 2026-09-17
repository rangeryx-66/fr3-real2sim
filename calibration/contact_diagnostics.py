"""Read-only contact location, normal opposition and gravity moment diagnostics."""
import json,gzip,argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def record(path):
    result=json.loads(path.read_text());mat=result['runtime_material']['material'];mass=float(np.array(mat['target_mass_kg']).ravel()[0]);com=np.array(mat['target_COM_local']).reshape(-1,7)[0,:3]
    mesh=np.load(Path(__file__).resolve().parents[1]/f"assets/arena_complex/{result['target']}_mesh.npz")['vertices']
    rows=json.load(gzip.open(path.with_suffix('.trace.json.gz')))['records'];out={}
    close=[r for r in rows if r['phase']=='CLOSE']
    first=[next((r['t'] for r in close if r['forces'][i]>.1),None) for i in range(2)]
    out['CLOSE_ONSET']=dict(first_contact_s=first,contact_onset_gap_s=abs(first[0]-first[1]) if all(t is not None for t in first) else None,max_total_force_N=max((sum(r['hand_calibration']['normal_force_N']) for r in close),default=0.))
    for phase in ['CLOSE_HOLD','MICRO_LIFT','LIFT','HOLD']:
        selected=[r for r in rows if r['phase']==phase]
        if not selected:continue
        selected=[r for r in selected if r['t']>=selected[-1]['t']-.2];metrics=[]
        for r in selected:
            H=Rotation.from_quat(np.roll(r['tcp_quat'],-1));O=Rotation.from_quat(np.roll(r['box_quat'],-1));c=O.apply(com)+r['box'];centers=[];normals=[];fractions=[]
            for finger in r['finger_contacts']:
                f=np.array(finger['normal_force_N']);pts=np.array(finger['points_world_m']);ns=np.array(finger['normals_world'])
                if len(f)==0 or f.sum()<.1:break
                centers.append(np.average(pts,axis=0,weights=f));n=np.average(ns,axis=0,weights=f);normals.append(n/max(np.linalg.norm(n),1e-12))
                local=H.inv().apply(pts-r['tcp']);pad=(abs(local[:,0])<=.00975)&(local[:,2]>=-.010)&(local[:,2]<=.0105)
                fractions.append(float(f[pad].sum()/f.sum()))
            if len(centers)!=2:continue
            center=np.mean(centers,axis=0);moment=np.cross(c-center,[0,0,-mass*9.81])
            metrics.append(dict(gravity_moment_Nm=float(np.linalg.norm(moment)),horizontal_COM_lever_mm=float(np.linalg.norm((c-center)[:2])*1000),normal_opposition=float(-np.dot(normals[0],normals[1])),friction_weight_ratio=float(result['mu']*sum(r['hand_calibration']['normal_force_N'])/(mass*9.81)),min_pad_force_fraction=min(fractions),contact_separation_mm=float(np.linalg.norm(centers[0]-centers[1])*1000)))
        if metrics:
            out[phase]={k:float(np.mean([m[k] for m in metrics])) for k in metrics[0]}
            tail=[v for v in selected if v['t']>=selected[-1]['t']-.1]
            th=Rotation.from_quat(np.array([v['tcp_quat'] for v in tail])[:,[1,2,3,0]]);to=Rotation.from_quat(np.array([v['box_quat'] for v in tail])[:,[1,2,3,0]]);tr=th.inv().apply(np.array([v['box'] for v in tail])-np.array([v['tcp'] for v in tail]));rr=th.inv()*to
            out[phase]['tail_100ms_translation_mm']=float(np.max(np.linalg.norm(tr-tr[0],axis=1))*1000);out[phase]['tail_100ms_rotation_deg']=float(np.max((rr[0].inv()*rr).magnitude())*180/np.pi)
            r=selected[-1];rot=Rotation.from_quat(np.roll(r['box_quat'],-1));out[phase]['target_bottom_world_mm']=float((rot.apply(mesh)+r['box'])[:,2].min()*1000);out[phase]['target_origin_rise_mm']=float((r['box'][2]-result['initial'][2])*1000)
    return dict(id=result['id'],target=result['target'],category=result['category'],phases=out)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();root=Path(a.directory);rows=[]
    for f in sorted(root.glob('*/*_F*.json')):
        try:rows.append(record(f))
        except Exception as e:rows.append(dict(file=str(f),error=str(e)))
    (root/'contact_diagnostics.json').write_text(json.dumps(rows,indent=2));print('diagnosed',len(rows))
