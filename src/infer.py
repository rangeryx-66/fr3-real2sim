"""Standalone consumer of the unmodified AnyGrasp SDK, in its own environment."""
import argparse
import json
import os
import sys
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('--input', required=True)
p.add_argument('--output', required=True)
p.add_argument('--sdk', default=os.environ.get('ANYGRASP_SDK_ROOT', '/data1/home/rangeryx/anygrasp_sdk/grasp_detection'))
p.add_argument('--top-k', type=int, default=20)
a=p.parse_args()
a.input=os.path.abspath(a.input);a.output=os.path.abspath(a.output)
os.chdir(a.sdk)
sys.path.insert(0,a.sdk)
from gsnet import create_detector
from types import SimpleNamespace
cfg=SimpleNamespace(checkpoint_path=os.path.join(a.sdk,'log/checkpoint_detection.tar'), max_gripper_width=0.08, gripper_height=0.03)
data=np.load(a.input)
detector=create_detector(cfg)
if detector is None: raise RuntimeError('NO_GRASP: detector initialization failed')
gg=detector.get_grasp(data['points'].astype(np.float32), {'dense_grasp':False,'collision_detection':True,'region_steering':data['mask'].astype(bool),'approach_steering':None,'approach_thresh':np.pi})
out=[]
if gg is not None:
    gg=gg.nms().sort_by_score()
    for i,g in enumerate(gg[:a.top_k]):
        out.append(dict(rank=i,score=float(g.score),width=float(g.width),depth=float(g.depth),rotation=g.rotation_matrix.tolist(),translation=g.translation.tolist()))
with open(a.output,'w') as f: json.dump(dict(frame='camera_optical',T_B_C=data['T_B_C'].tolist(),grasps=out),f,indent=2)
print('ANYGRASP_CANDIDATES',len(out),flush=True)
