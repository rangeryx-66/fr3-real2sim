"""Validate completeness, frozen inputs, material readbacks and success criteria."""
import json,hashlib,collections
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];base=ROOT/'results/hand_calibration';report={}
for file in ['results/arena_repair_frozen/sealed_files.json','results/hand_calibration/controller_frozen.json']:
 d=json.loads((ROOT/file).read_text());bad=[p for p,h in d.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h];assert not bad,bad;report[file]=dict(files=len(d),mismatches=bad)
rows=[]
for name in ['soup','banana','bowl','mug']:
 rr=[json.loads(p.read_text()) for p in (base/'formal'/name).glob('*_F*.json')];assert len(rr)==63,(name,len(rr));rows+=rr
 expected=collections.Counter({('force',f,mu):3 for f in [10,20,30,40,60] for mu in [.3,.5,.7,1.]});expected[('position',0,.7)]=3
 assert collections.Counter((r['controller'],r['force_total_N'],r['mu']) for r in rr)==expected
 for r in rr:
  assert r['gt_path_sha256']==hashlib.sha256((base/f'gt_path_{name}.json').read_bytes()).hexdigest()
  m=r['runtime_material']['material'];assert np.allclose(np.array(m['runtime_finger_material_properties'])[:,3,:2],r['mu']);assert np.allclose(np.array(m['runtime_target_material_properties'])[...,:2],1.)
  assert Path(r['trace'].get('path','')).exists() if 'path' in r['trace'] else True
  if r['success']:
   h=r['hold_samples'];assert r['micro_gate']['passed'] and r['hold_duration_s']>=2
   assert min(x['z']-r['initial'][2] for x in h)>=.08
   assert max(x['z'] for x in h)-min(x['z'] for x in h)<=.01 and all(min(x['forces'])>.1 for x in h)
  elif r['category']=='UNSTABLE_GRASP':assert not r['micro_gate']['passed'] and r['stage']=='MICRO_LIFT' and not r.get('hold_samples')
report.update(n=len(rows),success=sum(r['success'] for r in rows),categories=dict(collections.Counter(r['category'] for r in rows)),runtime_materials_verified=True)
(base/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
