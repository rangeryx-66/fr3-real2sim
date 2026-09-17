"""Audit the common BundleSDF object frame without consulting GT."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def mean_transform(transforms):
    out=np.eye(4);out[:3,3]=np.mean([x[:3,3] for x in transforms],axis=0)
    out[:3,:3]=Rotation.from_matrix([x[:3,:3] for x in transforms]).mean().as_matrix();return out


def audit(dataset:Path,reconstruction:Path):
    groups={}
    for p in sorted((dataset/'poses').glob('*.json')):
        row=json.loads(p.read_text());track=reconstruction/'ob_in_cam'/f'{p.stem}.txt'
        if not track.exists():continue
        T_B_C=np.asarray(row['T_B_camera']);T_B_TCP=np.asarray(row['T_B_TCP']);T_C_O=np.loadtxt(track)
        groups.setdefault(int(row['pass_id']),[]).append(np.linalg.inv(T_B_TCP)@T_B_C@T_C_O)
    result={'schema':'fr3_bundlesdf_pass_alignment/v1','object_frame':'single BundleSDF reconstruction frame','gt_used':False,'passes':{}}
    means={}
    for pid,Ts in groups.items():
        M=mean_transform(Ts);means[pid]=M;dt=[np.linalg.norm((np.linalg.inv(M)@x)[:3,3]) for x in Ts];dr=[np.degrees(Rotation.from_matrix((np.linalg.inv(M)@x)[:3,:3]).magnitude()) for x in Ts]
        result['passes'][str(pid)]={'frames':len(Ts),'T_TCP_BundleSDF_object':M.tolist(),'within_grasp_translation_rms_mm':float(np.sqrt(np.mean(np.square(dt)))*1000),'within_grasp_rotation_rms_deg':float(np.sqrt(np.mean(np.square(dr))))}
    if len(means)>1:
        keys=sorted(means);D=np.linalg.inv(means[keys[0]])@means[keys[1]]
        result['regrasp_change']={'translation_mm':float(np.linalg.norm(D[:3,3])*1000),'rotation_deg':float(np.degrees(Rotation.from_matrix(D[:3,:3]).magnitude()))}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('reconstruction',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();r=audit(a.dataset,a.reconstruction)
    if a.output:a.output.write_text(json.dumps(r,indent=2))
    print(json.dumps(r,indent=2))
