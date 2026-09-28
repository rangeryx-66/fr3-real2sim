"""Fixed, stratified tabletop placements for the rigid-object benchmark."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
objects=json.loads((ROOT/'config/r1a7_generalization_objects.json').read_text())
out=ROOT/'results/r1a7_generalization/scenarios'
out.mkdir(parents=True,exist_ok=True)
anchors=[(.420,-.075),(.450,.070),(.490,0.),(.540,-.060),(.570,.080)]
for oi,obj in enumerate(objects):
    rng=np.random.default_rng(20260928+oi)
    scenes=[]
    for i,(x,y) in enumerate(anchors):
        xy=np.array([x,y])+rng.uniform(-.008,.008,2)
        yaw=float(rng.uniform(-np.pi,np.pi))
        scenes.append(dict(object_id=obj['id'],placement=i+1,xy=xy.round(6).tolist(),yaw=yaw,
                           position_band='near' if i<2 else 'center' if i==2 else 'far'))
    (out/f"{obj['id']}.json").write_text(json.dumps(scenes,indent=2)+'\n')
print(out)
