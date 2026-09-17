"""Read-only force, contact and relative motion summaries for formal trials."""
import argparse,csv,gzip,json,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def rel(rows):
    H=Rotation.from_quat(np.array([r['tcp_quat'] for r in rows])[:,[1,2,3,0]])
    O=Rotation.from_quat(np.array([r['box_quat'] for r in rows])[:,[1,2,3,0]])
    return H.inv().apply(np.array([r['box'] for r in rows])-np.array([r['tcp'] for r in rows])),H.inv()*O

def summarize(path):
    r=json.loads(path.read_text());o={k:r.get(k) for k in ['id','target','controller','force_total_N','mu','success','category','stage','wall_seconds']}
    trace=path.with_suffix('.trace.json.gz')
    if not trace.exists():return o
    d=json.load(gzip.open(trace));rows=d['records'];names=d['names'];fi=[names.index('fr3_finger_joint1'),names.index('fr3_finger_joint2')]
    for phase in ['CLOSE','CLOSE_HOLD','MICRO_LIFT','LIFT','HOLD']:
        rr=[v for v in rows if v['phase']==phase]
        if not rr:continue
        # Settled last 0.2 s for forces/width; all phase samples for drift.
        end=[v for v in rr if v['t']>=rr[-1]['t']-.2]
        f=np.array([v['hand_calibration']['normal_force_N'] for v in end]);q=np.array([v['q'] for v in end])[:,fi]
        prefix=phase.lower()
        for i in range(2):o[f'{prefix}_force_{i+1}_N']=float(np.mean(f[:,i]));o[f'{prefix}_q_{i+1}_mm']=float(np.mean(q[:,i])*1000)
        o[prefix+'_width_mm']=float(q.sum(axis=1).mean()*1000)
        o[prefix+'_bilateral_fraction']=float(np.mean([min(v['forces'])>.1 for v in rr]))
        t,R=rel(rr);o[prefix+'_drift_mm']=float(np.max(np.linalg.norm(t-t[0],axis=1))*1000);o[prefix+'_rotation_deg']=float(np.max((R[0].inv()*R).magnitude())*180/np.pi)
    if r.get('micro_gate'):
        o.update(gate_pass=r['micro_gate']['passed'],gate_translation_mm=r['micro_gate']['max_translation_m']*1000,gate_rotation_deg=r['micro_gate']['max_rotation_deg'],gate_contact_gap_s=r['micro_gate']['max_contact_gap_s'])
    hold=[v['z']-r['initial'][2] for v in r.get('hold_samples',[])];o['min_hold_lift_mm']=min(hold)*1000 if hold else None
    material=r.get('runtime_material',{}).get('material',{});o['mass_kg']=material.get('target_mass_kg')
    return o

def main():
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();root=Path(a.directory)
    rows=[summarize(f) for f in sorted(root.glob('*/*.json')) if '_F' in f.name]
    if not rows:raise SystemExit('No trials')
    keys=list(dict.fromkeys(k for r in rows for k in r));out=root/'episodes.csv'
    with out.open('w') as f:w=csv.DictWriter(f,keys);w.writeheader();w.writerows(rows)
    groups=collections.defaultdict(list)
    for r in rows:groups[(r['target'],r['controller'],r['force_total_N'],r['mu'])].append(r)
    summary=[]
    for (target,control,force,mu),rr in sorted(groups.items()):
        summary.append(dict(target=target,controller=control,force_total_N=force,mu=mu,n=len(rr),success=sum(v['success'] for v in rr),failures=dict(collections.Counter(v['category'] for v in rr if not v['success']))))
    (root/'summary.json').write_text(json.dumps(summary,indent=2))
    for name in sorted(set(r['target'] for r in rows)):
        rr=[r for r in rows if r['target']==name];print(name,len(rr),sum(r['success'] for r in rr),dict(collections.Counter(r['category'] for r in rr)))
if __name__=='__main__':main()
