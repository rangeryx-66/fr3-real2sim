from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from handle_contact_quality import handle_surfaces,patch_quality
from revolute10_fixed import Model,authored_handle_colliders,physical_scene_class
from plan_oracle_door import closing_quality
out=ROOT/'results/45176_contact_transmission_20261011';out.mkdir(exist_ok=True);all_results=[]
for asset,old in [('45176',ROOT/'results/oracle_door_development_20261011/45176'),('45415',ROOT/'results/45415_precontact_continuity_20261011')]:
 job=json.loads((old/'job.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()));plan=json.loads((Path(job['whole_task_plan'])).read_text());export=json.loads(Path(job['collision_export']).read_text());s=job['grasp_geometry_candidates'][0];allowed=authored_handle_colliders(export,s);surfaces=handle_surfaces(model,export,allowed,s);base=plan['base'];Told=np.array(plan['T_grasp']);joint=model.manifest['joint_name'];link=model.manifest['moving_link'];M0=model.asset_T@model.asset.root_to_link(link,{joint:0.});D=model.asset_T@model.asset.root_to_link(link,{joint:1e-4})@np.linalg.inv(M0)
 def tangent(T):return (D@T)[:3,3]-T[:3,3]
 old_q=patch_quality(model,surfaces,Told,base,tangent(Told));results=[]
 n=model.asset_T[:3,:3]@s['outward_normal_root'];axis=model.asset_T[:3,:3]@s['axis_root'];n-=axis*(n@axis);n/=np.linalg.norm(n);P=model.poses(model.home,base,width=.1);offset=np.mean([(np.linalg.inv(P['tcp_link'])@P[f]@np.r_[model.pads[f].mean(0),1])[:3] for f in ['gripper_link1','gripper_link2']],axis=0);depth=np.diff(model.pads['gripper_link1'][:,1])[0]/4;sc=physical_scene_class(ROOT)(export,model,allowed)
 for section in s['sections']:
  anchor=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3];R=np.column_stack((axis,np.cross(-n,axis),-n))
  for insertion in [0.,-depth,depth,-2*depth,2*depth,-3*depth]:
   for pitch in [0.,-15.,15.]:
    T=np.eye(4);T[:3,:3]=R@Rotation.from_euler('x',pitch,degrees=True).as_matrix();T[:3,3]=anchor+n*insertion-T[:3,:3]@offset
    q=patch_quality(model,surfaces,T,base,tangent(T));c=closing_quality(model,sc,T,base,.017)
    results.append({'section':section['fraction'],'insertion_m':float(insertion),'pitch_deg':pitch,'T':T.tolist(),'patch':q,'closing_rank':c['rank'],'root':c['root_contact_count'],'other':c['non_handle_closing_collision']})
 results.sort(key=lambda r:(r['closing_rank'],r['patch']['score']),reverse=True)
 all_results.append({'asset':asset,'old_patch':old_q,'old_plan':{'section':plan['section_fraction'],'insertion':plan['insertion_m']},'ranked':results})
 print(asset,'OLD',old_q,flush=True)
 for r in results[:6]:print('BEST',asset,{k:v for k,v in r.items() if k not in ['T','patch']},'score',r['patch']['score'],flush=True)
(out/'PATCH_DIAGNOSIS_AND_COMPATIBILITY.json').write_text(json.dumps(all_results,indent=2))
