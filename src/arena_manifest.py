"""Predetermined asset/layout cohort; no grasp outcomes enter scene generation."""
import json,itertools
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
inventory=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
NAMES=['mustard','raisin','hidden_tuna','bowl','banana','sugar','soup','mug']
def layout(name,seed):
    rng=np.random.default_rng(seed);names=[name]+[NAMES[(NAMES.index(name)+i)%8] for i in range(1,6)]
    yaw=rng.uniform(-np.pi,np.pi,6);target=np.array([.5,0])+rng.uniform(-.015,.015,2)
    ext=[]
    for n,y in zip(names,yaw):
        bounds=np.array(inventory[n]['bounds']);R=Rotation.from_euler('z',y).as_matrix()
        ext.append(np.abs(R)@(bounds[1]-bounds[0])/2)
    for attempt in range(10000):
        positions=[target];angles=np.deg2rad([18,90,162,234,306])+rng.uniform(-.12,.12,5)
        for i,angle in enumerate(angles,1):
            d=np.array([np.cos(angle),np.sin(angle)])
            # Separate enclosing AABBs on at least one horizontal axis.
            radius=min((ext[0][j]+ext[i][j]+rng.uniform(.015,.035))/abs(d[j]) for j in range(2) if abs(d[j])>.01)
            positions.append(target+d*(radius+rng.uniform(0,.07)))
        valid=all(any(abs(positions[i][j]-positions[k][j])>=ext[i][j]+ext[k][j]+.010 for j in range(2)) for i,k in itertools.combinations(range(6),2))
        valid &= all(.15+.003<=p[0]-e[0] and p[0]+e[0]<=.85-.003 and -.497<=p[1]-e[1] and p[1]+e[1]<=.497 for p,e in zip(positions,ext))
        if valid:break
    else:raise RuntimeError((name,seed,'layout infeasible'))
    objects=[]
    for n,p,y,e in zip(names,positions,yaw,ext):
        b=np.array(inventory[n]['bounds']);c=b.mean(0);R=Rotation.from_euler('z',y).as_matrix();q=np.roll(Rotation.from_matrix(R).as_quat(),1)
        origin=np.r_[p,e[2]+.003]-R@c
        objects.append(dict(asset=n,position=origin.tolist(),quaternion_wxyz=q.tolist()))
    return dict(seed=seed,target=name,objects=objects,layout_attempts=attempt+1)
if __name__=='__main__':
    episodes=[layout(n,1000+5*i+j) for i,n in enumerate(NAMES) for j in range(5)]
    protocol=dict(experiment='Frozen v2 Arena non-box generalization',formal_count=40,background='maple_table_robolab',classes=NAMES,objects_per_scene=6,top_k=20,margin_m=.001,min_pad_coverage=.045,lift_gain_scale=2.,inference='unchanged infer.py, one capture per episode',pad_gate='unchanged opposing OBB face projection algorithm with GT asset size, not actual surface coverage',planning_target='GT render triangle mesh, identical world and attached geometry',planning_non_target='GT world AABB plus 1 mm per face',physics='authored convex decomposition; original friction 0.8/0.7; authored mass if valid, otherwise Arena default 0.2 kg',episodes=episodes)
    (ROOT/'ARENA_COMPLEX_PROTOCOL.json').write_text(json.dumps(protocol,indent=2))
