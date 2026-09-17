"""Freeze the final AnyGrasp-first/family-fallback paired protocol before trials."""
from __future__ import annotations
import hashlib,json,shutil
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
NEW=ROOT/'assets/family_final_v1'
FORMAL=ROOT/'assets/family_formal'
ARENA=ROOT/'assets/arena_complex'
OUT=ROOT/'assets/family_final_formal';OUT.mkdir(parents=True,exist_ok=True)
REQUESTED_UNSEEN=[
 'hot3d__megaphone','hot3d__potato_masher','hot3d__storage_box',
 'ycb__coffee_can','ycb__scissors','vomp__milkjug_a01__milkjug_a01',
 'vomp__utilityjug_a03__utilityjug_a03','vomp__whitepackerbottle_a01__whitepackerbottle_a01',
]
REGRESSION=['vomp__serving_bowl__serving_bowl','mug','soup','banana','sugar','mustard','hot3d__coffee_pot']
CLUTTER=['raisin','soup','mustard','sugar','hidden_tuna']
new=json.loads((NEW/'inventory.json').read_text());formal=json.loads((FORMAL/'inventory.json').read_text());arena=json.loads((ARENA/'inventory.json').read_text())
UNSEEN=[];structural_exclusions={}
for name in REQUESTED_UNSEEN:
 if name not in new:
  structural_exclusions[name]='USD rigid-body/collider probe rejected asset';continue
 bounds=np.asarray(new[name]['bounds'],float);extent=np.ptp(bounds,axis=0)
 if max(extent[:2])>.32 or extent[2]>.32:
  structural_exclusions[name]=f'outside frozen tabletop size envelope: {extent.tolist()}';continue
 UNSEEN.append(name)
if len(UNSEEN)<5:raise RuntimeError(f'fewer than five structurally valid unseen targets: {UNSEEN}, {structural_exclusions}')
inventory={'table':formal['table']}
for name in UNSEEN:
 inventory[name]=new[name];shutil.copy2(NEW/f'{name}_mesh.npz',OUT/f'{name}_mesh.npz')
for name in sorted(set(REGRESSION+CLUTTER)):
 source=formal if name in formal else arena
 inventory[name]=source[name];shutil.copy2(source/f'{name}_mesh.npz',OUT/f'{name}_mesh.npz')
(OUT/'inventory.json').write_text(json.dumps(inventory,indent=2))

def upright(name,center,yaw):
 bounds=np.asarray(inventory[name]['bounds'],float);local=bounds.mean(0);R=Rotation.from_euler('z',yaw).as_matrix();root=np.asarray(center,float)-R@local;root[2]=-bounds[0,2]+.001
 q=Rotation.from_euler('z',yaw).as_quat();return dict(asset=name,position=root.tolist(),quaternion_wxyz=np.roll(q,1).tolist())
def radius(name):
 e=np.ptp(np.asarray(inventory[name]['bounds'],float),axis=0);return float(np.linalg.norm(e[:2])/2)

rng=np.random.default_rng(20260915);targets=[('unseen',x) for x in UNSEEN]+[('regression',x) for x in REGRESSION]
assign=[]
while len(assign)<45:assign.extend(targets)
assign=assign[:45];episodes=[]
for i,(cohort,target) in enumerate(assign):
 seed=6000+i;repeat=sum(1 for e in episodes if e['target']==target)
 center=np.array([rng.uniform(.47,.52),rng.uniform(-.025,.025),0.]);objects=[upright(target,center,rng.uniform(-np.pi,np.pi))]
 choices=[x for x in CLUTTER if x!=target];rng.shuffle(choices);phase=rng.uniform(-.20,.20)
 for j,obstacle in enumerate(choices[:3]):
  angle=phase+j*2*np.pi/3;distance=radius(target)+radius(obstacle)+.035
  p=center+np.array([distance*np.cos(angle),distance*np.sin(angle),0.]);objects.append(upright(obstacle,p,rng.uniform(-.25,.25)))
 episodes.append(dict(seed=seed,cohort=cohort,repeat=repeat,target=target,objects=objects,paired_modes=['BASELINE','ANYGRASP_FIRST_FAMILY_FALLBACK']))
protocol=dict(version='grasp_family_final_v1',created_before_physical_trials=True,selection_rule='explicit Arena candidates; one rigid root, collider present, <=0.32 m tabletop envelope; no grasp outcomes',requested_unseen_targets=REQUESTED_UNSEEN,structural_exclusions=structural_exclusions,unseen_targets=UNSEEN,regression_targets=REGRESSION,scene_pairs=45,physical_episodes=90,random_seed=20260915,seed_range=[6000,6044],policy=dict(anygrasp_attempt_budget=3,family_fallback_attempt_budget=2,family_is_lazy_fallback=True),episodes=episodes)
(ROOT/'FAMILY_FINAL_PROTOCOL.json').write_text(json.dumps(protocol,indent=2))
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(OUT.iterdir()) if p.is_file() and not p.name.endswith('_scene.usdc')}
(ROOT/'FAMILY_FINAL_ASSET_HASHES.json').write_text(json.dumps(hashes,indent=2))
print(json.dumps(dict(unseen=UNSEEN,excluded=structural_exclusions,pairs=len(episodes),physical=2*len(episodes)),indent=2))
