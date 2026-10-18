"""Archived actual contact-frame replay. No new collision tolerances or surfaces."""
import json,gzip,pickle,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from xarm7_native_gripper import NativeGripperGeometry
from xarm7_native_planning import PAD_FRAMES
from native_g1_hybrid_contact import PanelScene
p=Path(sys.argv[1]);idx=json.loads((p/'telemetry/index.json').read_text());rr=[]
for c in idx['chunks'][-2:]:
 with gzip.open(p/'telemetry'/c['file'],'rb') as f:rr+=pickle.load(f)
native=NativeGripperGeometry();raw={}
for n,l in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]:
 m=native.meshes[n].copy();m.apply_transform(np.linalg.inv(PAD_FRAMES[n]));raw[l]=m
s=PanelScene(SimpleNamespace(),raw);results=[];current=rr[-1];previous=rr[-2]
pre={b['name']:b for b in previous['wrist_ft']['body_states']};post={b['name']:b for b in current['wrist_ft']['body_states']}
P={l:np.asarray(pre[n]['T_world_body'])@PAD_FRAMES[n] for n,l in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]}
for c in current['contacts']:
 if c['force_n']<=0:continue
 l=c['finger'];point=np.asarray(c['contact_point_world_m']);assert s.on_cap(l,point,P,c['separation_m'])
 # Real points and the original cap/roundoff pass. A corresponding root/back
 # point remains rejected using exactly the same physical separation.
 local=(np.linalg.inv(P[l])@np.r_[point,1])[:3];wrong=local.copy();wrong[1]-=.01;wrong=(P[l]@np.r_[wrong,1])[:3]
 assert not s.on_cap(l,wrong,P,c['separation_m'])
 results.append(dict(finger=l,real_pre_step_cap_accepted=True,root_10mm_away_rejected=True,native_separation_m=c['separation_m']))
result=dict(native_240hz_step=current['t']-previous['t'],contacts=results,physical_geometry_changed=False,contact_tolerance_changed=False,physics_actors=0)
(p/'CONTACT_TIMING_REGRESSION.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
