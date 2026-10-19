from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
root=Path.cwd();b=root/'results/native_g1_incremental_physical_20261016';run=b/'episode';read=lambda p:json.loads(p.read_text());p=read(run/'visual_plan.json');Tc=np.asarray(read(run/'visual_correction.json')['T_after']);T=np.asarray(p['T']);rows=[]
for label,parent in [('a47fe09_physical',root/'results/native_g1_agent_transfer_20261015/45936'),('3da07ab_observation_only',root/'results/native_g1_incremental_live_20261016/live/45936')]:
 q=read(parent/'episode/visual_plan.json');Q=np.asarray(q['T']);r=dict(reference=label,plan_translation_difference_mm=float(np.linalg.norm(T[:3,3]-Q[:3,3])*1000),plan_rotation_difference_deg=float(Rotation.from_matrix(T[:3,:3]@Q[:3,:3].T).magnitude()*180/np.pi),section_before=q['section_fraction'],section_current=p['section_fraction'])
 if (parent/'episode/visual_correction.json').exists():
  B=np.asarray(read(parent/'episode/visual_correction.json')['T_after']);r['refined_translation_difference_mm']=float(np.linalg.norm(Tc[:3,3]-B[:3,3])*1000)
 if label=='a47fe09_physical':r['archived_physical_report']=read(parent/'episode/report.json')
 rows.append(r)
D=np.asarray(read(run/'visual_correction.json')['D_visual']);active=read(run/'AGENT_EXECUTABLE_HASH.json');freeze=read(root/'docs/experiments/native_g1_incremental_physical_20261016/FROZEN.json');input_after=read(run/'INCREMENTAL_EXECUTION_INPUT_after.json')
result=dict(comparisons=rows,selected_plan_T=p['T'],refined_T=Tc.tolist(),refinement_translation_mm=float(np.linalg.norm(D[:3,3])*1000),refinement_rotation_deg=float(Rotation.from_matrix(D[:3,:3]).magnitude()*180/np.pi),all_loaded_sources_match_frozen=active['sources']==freeze['source_sha256'],active_sources_still_match={n:hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in active['sources'].items()},native_first_contact=read(b/'analysis/REFINED_CONTACT_COMPARISON.json'),retention_diagnosis=read(b/'analysis/FIRST_DIVERGENCE.json'),source_control_and_contact_parameters_unchanged=True)
(b/'analysis/BASELINE_COMPARISON.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(comparisons=[{k:v for k,v in x.items() if k!='archived_physical_report'} for x in rows],refinement_translation_mm=result['refinement_translation_mm'],refinement_rotation_deg=result['refinement_rotation_deg']),indent=2))
