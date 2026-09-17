"""Deterministic asset/pose split, created before any held-out grasp execution."""
import hashlib,json
from pathlib import Path
import numpy as np
from support_aware import CONFIG
ROOT=Path(__file__).resolve().parents[1]
folder=ROOT/'assets/unseen_v1'
inventory=json.loads((folder/'inventory.json').read_text())
old=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
targets=[x for x in inventory if 'registry_relative' in inventory[x]]
assert len(targets)==10,targets
obstacles=['raisin','soup','mustard']
for name in obstacles:inventory[name]=old[name]
(folder/'inventory.json').write_text(json.dumps(inventory,indent=2))
episodes=[]
for index,name in enumerate(targets):
    bounds=np.array(inventory[name]['bounds']);radius=np.linalg.norm((bounds[1]-bounds[0])[:2])/2
    for repeat in range(5):
        seed=3000+index*5+repeat;rng=np.random.default_rng(seed)
        xy=np.array([.5,0])+rng.uniform(-.025,.025,2);yaw=rng.uniform(-np.pi,np.pi)
        objects=[dict(asset=name,position=[*xy,-bounds[0,2]+.001],quaternion_wxyz=[np.cos(yaw/2),0,0,np.sin(yaw/2)])]
        for k,asset in enumerate(obstacles):
            b=np.asarray(inventory[asset]['bounds']);rr=radius+np.linalg.norm((b[1]-b[0])[:2])/2+.04+rng.uniform(0,.015)
            theta=np.deg2rad([-65,0,65][k])+rng.uniform(-.04,.04);oyaw=rng.uniform(-.2,.2)
            objects.append(dict(asset=asset,position=[*(xy+rr*np.array([np.cos(theta),np.sin(theta)])),-b[0,2]+.001],quaternion_wxyz=[np.cos(oyaw/2),0,0,np.sin(oyaw/2)]))
        episodes.append(dict(seed=seed,target=name,objects=objects,order=['GT','ANYGRASP'] if repeat%2==0 else ['ANYGRASP','GT']))
protocol=dict(version='unseen_support_v1',targets=targets,episodes=episodes,episode_count=100,
    modes=['GT','ANYGRASP'],definition_unseen='Excluded from executor development; AnyGrasp pretraining membership unknown',
    support=CONFIG,force_total_N=30.,pad_target_mu=.7,arm_gain_scale_after_close=2.,collision_margin_m=.001,
    anygrasp_top_k=20,max_actual_grasps=3,refinement_samples=256,refinement_seed=20260911,refinement_moveit_limit=12,
    oracle=dict(method='GT mesh antipodal pairs with URDF palm hull prescreen; class-independent geometric oracle, not proven optimal',seed=20260912,surface_samples=1024,mesh_check_limit=512,approach_tilts_deg=[0,-25,25],palm_clearance_m=.005,top_k=20),
    non_target_contact_N=.05,disturbance_translation_m=.002,disturbance_rotation_deg=2.,
    height_reference='World Z of the GT mesh local bounding-box center; unchanged from root height for centered development assets',
    post_failure_observation_s=2.2,tracking_watchdog_steps=150,
    success='GT reference-center net lift >=0.08 m, full mesh clear of table, bilateral contact and stable hold >=2 s',
    invalidation='If an implementation bug changes formal execution or measurement, keep old batch and restart all 100 episodes')
(ROOT/'UNSEEN_PROTOCOL.json').write_text(json.dumps(protocol,indent=2))
print('FROZEN_LAYOUTS',len(episodes),'TARGETS',len(targets))
