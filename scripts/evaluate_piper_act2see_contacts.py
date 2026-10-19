"""Post-run contact-normal reconstruction; never imported by the controller.

Usage: python scripts/evaluate_piper_act2see_contacts.py COMPLETED_RUN_DIRECTORY
The output is an estimate with explicit assumptions, not a logged GT trajectory.
"""
import json,sys,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
p=Path(sys.argv[1])
if not (p/'estimated_articulation.json').exists():raise RuntimeError('Estimate must be saved before GT evaluation')
r=json.loads((p/'report.json').read_text())
if not r['evaluation'].get('gt_access_after_world_pause'):raise RuntimeError('Run must be stopped before this diagnostic')
gt=json.loads((p/'evaluation_only.json').read_text())['ground_truth'];rows=json.loads((p/'observations.json').read_text())
if gt['joint_type']!='revolute':raise RuntimeError('This optional contact-normal diagnostic is for revolute motion')
axis=np.array(gt['axis_world']);axis/=np.linalg.norm(axis);origin=np.array(gt['origin_world'])
first=next(i for i,s in enumerate(rows) if s['phase']=='COMPLIANT_SETTLE');E0=np.array(rows[first-1]['T_tcp'])
def normal(s,reference=None):
 v=[]
 for c in s['contacts']:
  if c['allowed_pad_target'] and c['force_n']>0 and 'native_normal_world' in c:
   n=np.array(c['native_normal_world']);n/=np.linalg.norm(n)
   ref=v[0] if v else reference
   if ref is not None and n@ref<0:n=-n
   v.append(n)
 if not v:return None
 n=np.mean(v,axis=0);return n/np.linalg.norm(n)
n0=normal(rows[first-1]);u=n0-axis*(axis@n0);u/=np.linalg.norm(u);series=[]
for s in rows[first:]:
 n=normal(s,n0)
 if n is None:continue
 v=n-axis*(axis@n);v/=np.linalg.norm(v);angle=float(np.arctan2(axis@np.cross(u,v),u@v));E=np.array(s['T_tcp']);R=Rotation.from_rotvec(angle*axis).as_matrix()
 position=origin+R.T@(E[:3,3]-origin);rotation=R.T@E[:3,:3]@E0[:3,:3].T
 series.append({'t':s['t'],'door_angle_from_contact_normal_deg':float(np.rad2deg(angle)),'reconstructed_translation_slip_m':float(np.linalg.norm(position-E0[:3,3])),'reconstructed_rotation_slip_deg':float(np.rad2deg(Rotation.from_matrix(rotation).magnitude())),'normal_axis_component_change':float(abs(n@axis-n0@axis))})
result={'method':'post-hoc reconstruction from native contact normals + GT axis/origin; no online GT use; not a directly logged GT trajectory','max_reconstructed_translation_slip_m':max(x['reconstructed_translation_slip_m'] for x in series),'max_reconstructed_rotation_slip_deg':max(x['reconstructed_rotation_slip_deg'] for x in series),'final_reconstructed_translation_slip_m':series[-1]['reconstructed_translation_slip_m'],'final_reconstructed_door_angle_deg':series[-1]['door_angle_from_contact_normal_deg'],'endpoint_angle_error_vs_final_simulator_GT_deg':abs(series[-1]['door_angle_from_contact_normal_deg']-r['evaluation']['actual_final_door_angle_deg']),'normal_axis_component_max_change':max(x['normal_axis_component_change'] for x in series),'valid_contact_fraction':len(series)/(len(rows)-first),'assumption':'reported loaded contact normals follow the same target support planes; endpoint and axis-component invariance are cross-checks, not a full error bound'}
(p/'posthoc_contact_normal_evaluation.json').write_text(json.dumps(result,indent=2));(p/'posthoc_contact_normal_trajectory.json').write_text(json.dumps(series[::8]));print(json.dumps(result,indent=2))
