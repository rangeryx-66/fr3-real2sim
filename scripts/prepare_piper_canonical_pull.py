"""Reuse unchanged PiPER exact IK to prepare a 2 mm tangential diagnostic pull."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--asset-root',type=Path,required=True);p.add_argument('--measured-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads(a.measured_report.read_text());s=json.loads((a.source/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',a.asset_root,s);q=np.array(r['actual_closure']['robot_q'])[:6];base=r['base_final'];T=model.poses(q,base)['tcp_link'];rows=[]
for mm in [.25,.5,1.,2.]:
 target=T.copy();target[:3,3]+=T[:3,0]*mm/1000
 solution=model.ik(target,base,seed=q,starts=5)
 if solution is None:raise RuntimeError('canonical pull exact IK failure')
 if model.margin(solution)<=.05:raise RuntimeError('canonical pull joint margin failure')
 rows.append({'displacement_mm':mm,'q':solution.tolist(),'margin_rad':model.margin(solution),'T':target.tolist()});q=solution
output={'axis':'TCP +X, tangential to the pad normal; no hinge information','base':base,'IK_configuration_unchanged':True,'waypoints':rows}
a.output.write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
