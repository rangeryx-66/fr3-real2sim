"""Map a GT-free reconstructed-frame NBV rotation into an executable grasp path."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('source_grasp',type=Path);p.add_argument('coverage_report',type=Path);p.add_argument('pass_alignment',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    source=json.loads(a.source_grasp.read_text());coverage=json.loads(a.coverage_report.read_text());alignment=json.loads(a.pass_alignment.read_text())
    O=np.asarray(source['T_B_target']);H=np.asarray(source['T_B_TCP']);T_target_tcp=np.linalg.inv(O)@H
    T_tcp_common=np.asarray(alignment['passes']['0']['T_TCP_BundleSDF_object']);T_target_common=T_target_tcp@T_tcp_common
    Q=np.asarray(coverage['recommended_regrasp']['rotation_common_object']);R=T_target_common[:3,:3]@Q@T_target_common[:3,:3].T
    D=np.eye(4);D[:3,:3]=R;H_next=O@D@T_target_tcp
    out={**source,'T_B_TCP':H_next.tolist(),'scan_regrasp':{'source':str(a.source_grasp),'selection':'GT-free reconstructed surface coverage next-best-view','coverage_report':str(a.coverage_report),'axis_common_object':coverage['recommended_regrasp']['axis_common_object'],'angle_deg':coverage['recommended_regrasp']['angle_deg'],'GT_policy':'GT transform used only to execute the selected relative regrasp; GT geometry/pose not used to score views'}}
    a.output.write_text(json.dumps(out,indent=2));print(a.output)


if __name__=='__main__':main()
