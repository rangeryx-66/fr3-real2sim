"""GT evaluator only. Its output is never consumed by visual estimation/execution."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from interaction_identification.contact_probe import robot_only_model
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
j=json.loads((a.run/'job.json').read_text());reference=json.loads(Path(j['plan']).read_text())['trial_candidates'][0];GT=np.asarray(reference['T']);plan_path=a.run/('visual_plan.json' if (a.run/'visual_plan.json').exists() else 'visual_shadow_plan.json');plan=json.loads(plan_path.read_text());visual=json.loads((a.run/'rgbd/initial/visual_handle.json').read_text())
rows=[]
for c in plan['rows']:
 T=np.asarray(c['T']);delta=T[:3,3]-GT[:3,3]
 angles=[np.rad2deg(Rotation.from_matrix(T[:3,:3]@(GT[:3,:3]@Rotation.from_euler('z',v,degrees=True).as_matrix()).T).magnitude()) for v in [0,180]]
 rows.append({'candidate_index':c['candidate_index'],'anchor_hypothesis':c.get('visual_anchor_hypothesis','cross_section_center'),'position_error_to_saved_GT_grasp_m':float(np.linalg.norm(delta)),'orientation_error_deg':float(angles[0]),'parallel_jaw_symmetry_orientation_error_deg':float(min(angles)),'translation_components_GT_grasp_frame_m':(GT[:3,:3].T@delta).tolist(),'IK_feasible':c['status']!='NO_IK','accessibility':c['status'],'T_visual':T.tolist()})
result={'classification':'OFFLINE_GT_EVALUATION_ONLY','reference':'saved GT-derived baseline grasp pose; other valid positions along the handle need not coincide','not_consumed_by_controller':True,'plan_sha256':hashlib.sha256(plan_path.read_bytes()).hexdigest(),'reference_GT_T':GT.tolist(),'rows':rows,'selected_visual_candidate':plan['trial_candidates'][0] if plan['trial_candidates'] else None,'fully_GT_free_execution':False}
correction=a.run/'visual_correction.json'
if correction.exists():
 T=np.asarray(json.loads(correction.read_text())['T_after']);angles=[np.rad2deg(Rotation.from_matrix(T[:3,:3]@(GT[:3,:3]@Rotation.from_euler('z',v,degrees=True).as_matrix()).T).magnitude()) for v in [0,180]]
 result['after_visual_correction']={'position_error_to_saved_GT_grasp_m':float(np.linalg.norm(T[:3,3]-GT[:3,3])),'orientation_error_deg':float(angles[0]),'parallel_jaw_symmetry_orientation_error_deg':float(min(angles))}
(a.run/'offline_GT_grasp_errors.json').write_text(json.dumps(result,indent=2));print(json.dumps({'candidate_count':len(rows),'feasible_count':sum(c['IK_feasible'] for c in rows),'selected':next((r for r in rows if r['accessibility']=='PENDING_REAL_CLOSURE'),None)},indent=2))
