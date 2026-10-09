"""Post-run pose errors. Never imported by perception or physical execution."""
import argparse, json, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
job=json.loads((a.run/'job.json').read_text());v=json.loads((a.run/'rgbd/initial/visual_handle.json').read_text())
m=Model(ROOT/'config/piper.urdf',Path(job['asset_root']),json.loads((Path(job['source'])/'report.json').read_text()))
meta=m.manifest['interaction_geometry']['selection'];section=min(meta['sections'],key=lambda x:abs(x['fraction']))
gt_anchor=(m.asset_T@np.r_[section['anchor_root_m'],1.])[:3]
gt_axis=m.asset_T[:3,:3]@np.asarray(meta['axis_root']);gt_axis/=np.linalg.norm(gt_axis)
gt_normal=m.asset_T[:3,:3]@np.asarray(meta['outward_normal_root']);gt_normal/=np.linalg.norm(gt_normal)
axis=np.asarray(v['axis_world']);axis/=np.linalg.norm(axis);normal=np.asarray(v['outward_normal_world']);normal/=np.linalg.norm(normal)
delta=np.asarray(v['anchor_world_m'])-gt_anchor
result={'classification':'OFFLINE_GT_CONTACT_FRAME_EVALUATION_ONLY','not_consumed_by_execution':True,'GT_reference':'authored approximate interaction-proxy central section; not a hardware or exact visual-mesh ground truth','anchor_error_m':float(np.linalg.norm(delta)),'anchor_delta_world_m':delta.tolist(),'long_axis_error_unsigned_deg':float(np.rad2deg(np.arccos(np.clip(abs(axis@gt_axis),-1,1)))),'outward_normal_error_deg':float(np.rad2deg(np.arccos(np.clip(normal@gt_normal,-1,1)))),'GT_anchor_world_m':gt_anchor.tolist(),'GT_axis_world':gt_axis.tolist(),'GT_outward_normal_world':gt_normal.tolist(),'note':'Raw estimated handle frame is distinct from the selected grasp, which may intentionally use a surface hypothesis, side-face orientation or along-handle offset.'}
(a.run/'offline_GT_contact_frame_errors.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:result[k] for k in ['anchor_error_m','long_axis_error_unsigned_deg','outward_normal_error_deg']}))
