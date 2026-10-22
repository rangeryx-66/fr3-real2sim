from pathlib import Path
import json,numpy as np,trimesh
from xarm7_native_planning import NativeModel
from revolute10_fixed import authored_handle_colliders
from handle_contact_quality import handle_surfaces,ray_surface
B=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_visual_reliability_20261015');O=B.parent/'native_g1_visual_input_20261014';rows=[]
for a in ['45600','45606','45671','45638']:
 p=json.loads((B/'offline'/a/'visual_plan.json').read_text());job=json.loads((O/a/'job.json').read_text());m=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()));export=json.loads((B/'offline'/a/'cooked_initial.json').read_text());allowed=authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);gt,native=handle_surfaces(m,export,allowed,job['grasp_geometry_candidates'][0]);folder=B/'diagnostics/observed'/('45671_retry' if a=='45671' else a);P=np.load(folder/'handle_points_world.npy');_,dist,_=trimesh.proximity.closest_point(gt,P);T=np.array(p['T']);width=p['contact_geometry_audit']['robust_contact']['chosen_expected_aperture_m'];poses=m.poses(m.home,p['base'],width=width);D=T@np.linalg.inv(poses['tcp_link']);pairs={}
 for n in ['gripper_link1','gripper_link2']:
  F=D@poses[n];b=m.pads[n];grid=np.array([[x,y,b[0,2]] for x in np.linspace(*b[:,0],9) for y in np.linspace(*b[:,1],7)]);orig=grid@F[:3,:3].T+F[:3,3];direction=-F[:3,2];gap=np.min([ray_surface(mesh,orig-.1*direction,direction)[0]-.1 for mesh in native],axis=0);v=gap[np.isfinite(gap)];pairs[n]=dict(hit_fraction=float(np.isfinite(gap).mean()),gap_quantiles_mm=(np.quantile(v,[0,.5,1])*1000).tolist() if len(v) else None)
 rows.append(dict(asset=a,GT_ONLY_OFFLINE=True,updated_point_surface_p95_mm=float(np.quantile(dist,.95)*1000),updated_point_surface_median_mm=float(np.median(dist)*1000),selected={k:p[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']},planned_range_deg=float(np.rad2deg(p['planned_reached_state'])),expected_aperture_mm=width*1000,native_ray_diagnostic=pairs,score={k:p['contact_geometry_audit']['robust_contact'][k] for k in ['acquisition_score','retention_score','task_score','robust_combined_score']},GT_admission_gate=False))
 print(rows[-1],flush=True)
(B/'diagnostics/REFINED_GEOMETRY_COMPARISON.json').write_text(json.dumps(rows,indent=2))
