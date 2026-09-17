"""Express identified TCP-frame inertia in the reconstructed object frame."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np


def transform(inertial_tcp:Path,scan_manifest:Path,ob_in_cam_dir:Path,output:Path):
    estimate=json.loads(Path(inertial_tcp).read_text());manifest=json.loads(Path(scan_manifest).read_text())
    frame=manifest['frames'][0];T_B_C=np.asarray(frame['T_B_camera']);T_B_TCP=np.asarray(frame['T_B_TCP'])
    tracks=sorted(Path(ob_in_cam_dir).glob('*.txt'))
    if not tracks:raise FileNotFoundError('BundleSDF ob_in_cam tracking is empty')
    T_C_O=np.loadtxt(tracks[0]).reshape(4,4);T_C_TCP=np.linalg.inv(T_B_C)@T_B_TCP;T_TCP_O=np.linalg.inv(T_C_TCP)@T_C_O
    R_T_O=T_TCP_O[:3,:3];p_T_O=T_TCP_O[:3,3];R_O_T=R_T_O.T
    com_T=np.asarray(estimate['center_of_mass']);I_T=np.asarray(estimate['inertia_matrix'])
    result={**estimate,'center_of_mass':(R_O_T@(com_T-p_T_O)).tolist(),'inertia_matrix':(R_O_T@I_T@R_O_T.T).tolist(),
            'expressed_in':'BundleSDF_object','T_TCP_object':T_TCP_O.tolist(),
            'frame_source':'robot T_B_camera/T_B_TCP plus BundleSDF ob_in_cam; no GT'}
    Path(output).write_text(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('inertial_tcp',type=Path);p.add_argument('scan_manifest',type=Path);p.add_argument('ob_in_cam',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(transform(a.inertial_tcp,a.scan_manifest,a.ob_in_cam,a.output),indent=2))
