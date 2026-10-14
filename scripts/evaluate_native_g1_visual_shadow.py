"""Offline GT/oracle comparison only. Never imported by the visual runner."""
from pathlib import Path
import argparse,json,numpy as np,trimesh
from scipy.spatial.transform import Rotation
from xarm7_native_planning import NativeModel
from handle_contact_quality import handle_surfaces
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'results/native_g1_visual_input_20261014';old=ROOT/'results/native_g1_robust_rank_20261013'
parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=['shadow','physical'],default='shadow');args=parser.parse_args()
rows=[]
for asset in ['45671','45638','45600','45606']:
 folder=base/asset/(args.phase+'_01');row={'asset':asset,'comparison_role':'DIAGNOSTIC_ONLY; no admission threshold or pose correction'}
 if not (folder/'visual_plan.json').exists():
  row.update(visual_plan=False,reason=json.loads((folder/'report.json').read_text()).get('status') if (folder/'report.json').exists() else 'shadow incomplete');rows.append(row);continue
 job=json.loads((old/asset/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source);export=json.loads(Path(job['collision_export']).read_text());selection=job['grasp_geometry_candidates'][0]
 from revolute10_fixed import authored_handle_colliders
 allowed=authored_handle_colliders(export,selection);gt,_=handle_surfaces(model,export,allowed,selection)
 visual=json.loads((folder/'visual_plan.json').read_text());oracle=json.loads((old/asset/'planning/candidate.json').read_text());T=np.asarray(visual['T']);O=np.asarray(oracle['T']);obs=json.loads((folder/'visual_planning/robust_geometry_audit.json').read_text())
 init=next((folder/'rgbd').glob('initial*/handle_points_world.npy'));P=np.load(init)
 if len(P)>3000:P=P[np.random.default_rng(61).choice(len(P),3000,replace=False)]
 closest,distance,tri=trimesh.proximity.closest_point(gt,P)
 row.update(visual_plan=True,station_same=np.allclose(visual['base'],oracle['base']),planned_range_deg=float(np.rad2deg(visual['planned_reached_state'])),observed_to_GT_visual_surface_rmse_mm=float(np.sqrt(np.mean(distance**2))*1000),observed_to_GT_visual_surface_median_mm=float(np.median(distance)*1000),observed_to_GT_visual_surface_p95_mm=float(np.quantile(distance,.95)*1000),grasp_translation_difference_from_oracle_mm=float(np.linalg.norm(T[:3,3]-O[:3,3])*1000),grasp_orientation_difference_from_oracle_deg=float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@O[:3,:3].T).magnitude())),different_valid_poses_need_not_match_oracle=True,visual_scores={k:visual['contact_geometry_audit']['robust_contact'][k] for k in ['acquisition_score','retention_score','task_score','robust_combined_score','chosen_expected_aperture_m']},oracle_scores={k:oracle['contact_geometry_audit']['robust_contact'][k] for k in ['acquisition_score','retention_score','task_score','robust_combined_score','chosen_expected_aperture_m']},plan_wall_s=json.loads((folder/'visual_planning/planning_summary.json').read_text())['wall_s'],GT_used_only_in_this_post_planning_evaluator=True)
 rows.append(row)
(base/('SHADOW_COMPARISON.json' if args.phase=='shadow' else 'PHYSICAL_GT_DIAGNOSTICS.json')).write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
