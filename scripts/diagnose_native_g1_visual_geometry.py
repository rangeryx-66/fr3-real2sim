"""Offline observed-vs-GT surface/contact forensic evaluation; never a pose source."""
from pathlib import Path
import argparse,json,gzip,pickle,numpy as np,trimesh
from scipy.spatial.transform import Rotation
from xarm7_native_planning import NativeModel,LOGICAL
from revolute10_fixed import authored_handle_colliders
from handle_contact_quality import handle_surfaces,ray_surface
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
rows=[]
for asset in ['45671','45638','45600','45606']:
 case=a.root/asset;scene=case/'physical_01' if (case/'physical_01/visual_plan.json').exists() else case/'shadow_01';plan=json.loads((scene/'visual_plan.json').read_text());folder=next((scene/'rgbd').glob('initial*/observed_geometry.json')).parent;data=json.loads((folder/'observed_geometry.json').read_text());P=np.load(folder/'handle_points_world.npy');job=json.loads((case/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source);export=json.loads((scene/'cooked_initial.json').read_text());allowed=authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);gt,native=handle_surfaces(model,export,allowed,job['grasp_geometry_candidates'][0]);normal=np.array(data['normal_world']);axis=np.array(data['axis_world']);side=np.cross(normal,axis);F=np.column_stack([axis,side,normal]);origin=np.array(data['legacy_estimate']['front_surface_anchor_world_m'])
 observed=trimesh.util.concatenate([trimesh.Trimesh(x['vertices'],x['faces'],process=False) for x in data['parts']]);panel=np.array(data['panel']['vertices']);panel_projection=float(np.max(panel@normal));h=P@normal-panel_projection
 _,distance,_=trimesh.proximity.closest_point(gt,P);local_gt=(gt.vertices-origin)@F
 row=dict(asset=asset,GT_ONLY_OFFLINE=True,observed_points=len(P),observed_dimensions=data['dimensions_m'],GT_visual_projected_dimensions=np.ptp(local_gt,axis=0).tolist(),height_above_observed_panel_quantiles_mm=(np.quantile(h,[0,.1,.25,.5,.75,.9,1])*1000).tolist(),points_within_3mm_of_panel_fraction=float((abs(h)<.003).mean()),point_to_GT_visual_surface_quantiles_mm=(np.quantile(distance,[0,.25,.5,.75,.9,.95,1])*1000).tolist(),selection={k:plan[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']},predicted_first_contact=plan['contact_geometry_audit']['first_contacts'])
 def scan(T,width):
  P0=model.poses(model.home,plan['base'],width=width);D=T@np.linalg.inv(P0['tcp_link']);result={}
  for n in LOGICAL:
   frame=D@P0[n];b=model.pads[n];grid=np.array([[x,y,b[0,2]] for x in np.linspace(*b[:,0],9) for y in np.linspace(*b[:,1],7)]);points=grid@frame[:3,:3].T+frame[:3,3];direction=-frame[:3,2];r={}
   for label,meshes in [('observed',[observed]),('GT_native_offline',native)]:
    rays=[ray_surface(m,points-.1*direction,direction)[0]-.1 for m in meshes];gap=np.min(rays,axis=0);valid=np.isfinite(gap);r[label]=dict(hit_fraction=float(valid.mean()),signed_gap_quantiles_mm=(np.quantile(gap[valid],[0,.25,.5,.75,1])*1000).tolist() if valid.any() else None,within_1mm_fraction=float((abs(gap)<.001).mean()))
   result[n]=r
  return result
 T=np.asarray(plan['T']);width=plan['contact_geometry_audit']['robust_contact']['chosen_expected_aperture_m'];row['predicted_expected_aperture_mm']=width*1000;row['same_pose_contact_comparison']=scan(T,width)
 if (case/'physical_01/observations_chunks/index.json').exists():
  d=case/'physical_01/observations_chunks';idx=json.loads((d/'index.json').read_text());obs=sum([pickle.load(gzip.open(d/c['file'],'rb')) for c in idx['chunks']],[]);close=[r for r in obs if r['phase'] in ['SLOW_CLOSE','CENTER','LOW_PRELOAD']]
  if close:
   last=close[-1];first=next((r for r in close if max(r['forces_n'].values())>=.05),close[-1]);row['actual_first_contact_s']=first['t'];row['actual_last_closure_loads_n']=last['forces_n'];row['actual_last_aperture_mm']=last['aperture_m']*1000;row['last_TCP_from_command_distance_mm']=float(np.linalg.norm(np.array(last['T_tcp'])[:3,3]-T[:3,3])*1000);row['measured_pose_contact_comparison']=scan(np.array(last['T_tcp']),last['aperture_m'])
 correction=json.loads((scene/'visual_correction.json').read_text()) if (scene/'visual_correction.json').exists() else {};row['registration']=dict(translation_mm=float(np.linalg.norm(np.array(correction['D_visual'])[:3,3])*1000),rotation_deg=float(Rotation.from_matrix(np.array(correction['D_visual'])[:3,:3]).magnitude()*180/np.pi)) if correction else None
 rows.append(row);print(asset,json.dumps(row),flush=True)
(a.output/'GEOMETRY_FORENSICS.json').write_text(json.dumps(rows,indent=2))
