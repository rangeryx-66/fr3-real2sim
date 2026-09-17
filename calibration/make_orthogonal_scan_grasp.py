"""Rotate an existing stable target-relative grasp for an orthogonal scan pass."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--degrees',type=float,default=90.);a=p.parse_args()
    d=json.loads(a.source.read_text());O=np.asarray(d['T_B_target']);H=np.asarray(d['T_B_TCP']);X=np.linalg.inv(O)@H
    R=np.eye(4);R[:3,:3]=Rotation.from_euler('z',a.degrees,degrees=True).as_matrix();H2=O@R@X
    out={**d,'T_B_TCP':H2.tolist(),'scan_regrasp':{'source':str(a.source),'rotation_about_target_support_axis_deg':a.degrees,'purpose':'orthogonal gripper occlusion; not a reconstruction prior'}}
    a.output.write_text(json.dumps(out,indent=2));print(a.output)
if __name__=='__main__':main()
