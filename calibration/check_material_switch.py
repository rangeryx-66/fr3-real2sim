"""Preflight repeated bidirectional runtime material changes; no grasp trial."""
import os,sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));import plant
plant.URL='http://127.0.0.1:'+os.environ.get('CALIBRATION_PORT','18765')
rows=[]
for mu in [.3,1.,.7,.5,.7,1.,.3,.5]*2:
    r=plant.command(dict(op='calibration_material',mu=mu));plant.settle(.15)
    a=plant.command(dict(op='calibration_audit'))['material'];f=np.array(a['runtime_finger_material_properties']);target=np.array(a['runtime_target_material_properties'])
    record=dict(mu=mu,ok=r['ok'],actual_pads=f[:,3,:].tolist(),target_unique=np.unique(target.reshape(-1,3),axis=0).tolist());rows.append(record);print(record,flush=True)
    assert r['ok'] and np.allclose(f[:,3,:2],mu) and np.allclose(target[...,:2],1.),record
out=ROOT/'results/hand_calibration'/('material_switch_'+os.environ.get('CALIBRATION_PORT','18765')+'.json');out.write_text(json.dumps(rows,indent=2))
