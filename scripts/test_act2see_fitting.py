"""Small blind-model regression: noisy rotation, translation, insufficient motion."""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from interaction_identification.fitting import fit_articulation,evaluate
rng=np.random.default_rng(23);axis=np.array([.2,-.1,.97]);axis/=np.linalg.norm(axis)
c=np.array([.3,.2,.1]);radial=np.cross(axis,[0,1,0]);radial=.4*radial/np.linalg.norm(radial)
R0=Rotation.from_euler('xyz',[.4,.2,-.7]).as_matrix();rows=[]
for theta in np.linspace(0,.06,200):
 R=Rotation.from_rotvec(axis*theta).as_matrix();T=np.eye(4)
 T[:3,3]=c+R@radial+rng.normal(0,5e-6,3)
 T[:3,:3]=Rotation.from_rotvec(rng.normal(0,2e-5,3)).as_matrix()@R@R0;rows.append(T)
f=fit_articulation(rows);e=evaluate(f,{'joint_type':'revolute','axis_world':axis,'origin_world':c},rows[0][:3,3])
assert f['joint_type']=='revolute',f
assert e['axis_angular_error_deg']<1,e
assert e['axis_line_distance_m']<.004,e
line=[]
for distance in np.linspace(0,.025,200):
 T=np.eye(4);T[:3,:3]=R0;T[:3,3]=c+distance*axis+rng.normal(0,5e-6,3);line.append(T)
assert fit_articulation(line)['joint_type']=='prismatic'
assert fit_articulation([np.eye(4)]*20)['joint_type']=='UNOBSERVABLE'
print('PASS noisy small arc, linear motion, and insufficient excitation',e)
