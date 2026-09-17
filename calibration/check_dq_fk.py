"""Independent URDF FK against recorded TCP motion; no object information."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real2sim.fr3_robot_calibration import make_fr3_arm_urdf,_make_drake_plant,DEFAULT_FR3_URDF
out=Path(sys.argv[1]);urdf=make_fr3_arm_urdf(DEFAULT_FR3_URDF,out/'fk_check.urdf');plant,model=_make_drake_plant(urdf);ctx=plant.CreateDefaultContext();frame=plant.GetFrameByName('fr3_hand_tcp');result={}
for path in sorted(out.glob('pose_*.npz')):
 d=np.load(path);Ts=[]
 for q in d['q_actual']:
  plant.SetPositions(ctx,model,q);Ts.append(plant.CalcRelativeTransform(ctx,plant.world_frame(),frame).GetAsMatrix4())
 Ts=np.asarray(Ts);p=Ts[:,:3,3];actual=d['tcp_position'];rot=Rotation.from_matrix(Ts[:,:3,:3]);obs=Rotation.from_quat(np.roll(d['tcp_quat_wxyz'],-1,axis=1));keep=d['t']-d['t'][0]>=1
 result[path.stem]={'translation_change_agreement_max_m':float(np.max(np.linalg.norm((p-p[0])-(actual-actual[0]),axis=1))),'rotation_change_agreement_max_rad':float(np.max(((rot[0].inv()*rot).inv()*(obs[0].inv()*obs)).magnitude())),'fk_settled_translation_span_m':np.ptp(p[keep],axis=0).tolist(),'fk_settled_rotation_from_first_max_rad':float(np.max((rot[keep][0].inv()*rot[keep]).magnitude()))}
(out/'fk_consistency.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
