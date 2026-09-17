"""Known geometry and independent FK checks for the diagnostic coordinate chain."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from hand_geometry import geometry
from analyze_contact import fk,ROOT

H=np.eye(4);O=np.eye(4)
assert abs(geometry(H,O)['min_pad_coverage']-1.)<1e-10
O[0,3]=.04
assert geometry(H,O)['min_pad_coverage']==0.
O[0,3]=.015
G=np.eye(4);G[:3,:3]=Rotation.from_euler('xyz',[.7,-.4,1.1]).as_matrix();G[:3,3]=[.4,.2,-.1]
assert abs(geometry(H,O)['min_pad_coverage']-geometry(G@H,G@O)['min_pad_coverage'])<1e-10
checked=0
for path in (ROOT/'results/contact_margin1').glob('B_seed*.json'):
    r=json.loads(path.read_text())
    for e,m in zip(r['executions'],r['telemetry']['motion_endpoints']):
        state=m['actual'];q=list(state['q']);names=state['names']
        for name,v in zip(e['joint_names'],e['points'][-1]['q']):q[names.index(name)]=v
        actual=fk(names,q);expected=np.array(m['trajectory_endpoint_EE'])
        assert np.max(np.abs(actual-expected))<1e-7,(path,e['stage'],actual,expected)
        checked+=1
assert checked>0
print('Geometry invariance and',checked,'independent FK/MoveIt endpoint comparisons passed')
