"""Read-only candidate and frozen 45415 compatibility; no Isaac initialization."""
from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from revolute10_fixed import Model,PinkIK,authored_handle_colliders,physical_scene_class,at_base,calibration,camera_clearance
from handle_contact_quality import handle_surfaces,patch_quality
from plan_oracle_door import closing_quality
from articulated_demo.kinematics import transform
out=ROOT/'results/45176_contact_transmission_20261011';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();job=json.loads((out/'planning_job.json').read_text());candidate=json.loads((out/'planning/candidate.json').read_text());(out/'whole_plan.json').write_text(json.dumps(candidate,indent=2));(out/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':candidate['base'],'T':candidate['T_grasp'],'preplan':candidate['preplan'],'approach':candidate['approach']}]},indent=2))
# Normal selected-station scene initialization; not a mid-run base change.
source=Path(job['source'])/'report.json';src=json.loads(source.read_text());src['robot_base_pose']=candidate['base'];source.write_text(json.dumps(src,indent=2));job['output']=str(out/'physical_01');(out/'job.json').write_text(json.dumps(job,indent=2))
records=[]
for label,j,p in [('45176_new',job,candidate),('45415_frozen',json.loads((ROOT/'results/45415_precontact_continuity_20261011/job.json').read_text()),json.loads((ROOT/'results/45415_precontact_continuity_20261011/whole_plan.json').read_text()))]:
 model=Model(ROOT/'config/piper.urdf',j['asset_root'],json.loads((Path(j['source'])/'report.json').read_text()));export=json.loads(Path(j['collision_export']).read_text());e=next(e for e in export['shapes'] if e.get('rigid_body_path','').endswith('/base_link'));E=np.array(e['rigid_body_world_transform']);original=[*E[:3,3],float(np.rad2deg(np.arctan2(E[1,0],E[0,0])))];base=p['base'];selection=j['grasp_geometry_candidates'][0];allowed=authored_handle_colliders(export,selection);sc=physical_scene_class(ROOT)(at_base(export,original,base),model,allowed);cal=calibration(j['camera_calibration']);ik=PinkIK(model.urdf,model.home);model.ik=ik.solve
 failures=[];camera_failures=[];fk_error=[];margins=[]
 for phase,path,w,handle in [('opening',[x['q'] for x in p['path']],min(.1,selection['dimensions_m'][1]),True),('approach',p['approach'],.1,False),('home',p['preplan'],.1,False)]:
  for index,q in enumerate(path):
   q=np.array(q);P=model.poses(q,base,width=w);moving=sc.moving_reference
   if phase=='opening':
    joint=model.manifest['joint_name'];link=model.manifest['moving_link'];M0=model.asset_T@model.asset.root_to_link(link,{joint:0.});D=model.asset_T@model.asset.root_to_link(link,{joint:p['path'][index]['state']})@np.linalg.inv(M0);moving=D@sc.moving_reference;fk_error.append(float(np.linalg.norm(P['tcp_link'][:3,3]-np.array(p['path'][index]['T_tcp'])[:3,3])))
   ok,why=sc.check(P,moving,handle)
   if not ok:failures.append({'phase':phase,'index':index,'reason':why})
   ok,why=camera_clearance(sc,P,cal)
   if not ok:camera_failures.append({'phase':phase,'index':index,'reason':why})
   margins.append(model.margin(q))
 T=np.array(p['T_grasp']);M0=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:0.});D=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:1e-4})@np.linalg.inv(M0);patch=patch_quality(model,handle_surfaces(model,export,allowed,selection),T,base,(D@T)[:3,3]-T[:3,3]);closing=closing_quality(model,sc,T,base,.1)
 records.append({'case':label,'asset_id':model.manifest['asset_id'],'mesh':selection['mesh'],'planned_range_deg':float(np.rad2deg(p['planned_reached_state'])),'minimum_margin_rad':min(margins),'maximum_FK_error_m':max(fk_error),'collision_failures':failures,'camera_failures':camera_failures,'closing':closing,'patch':patch,'base':base,'section':p['section_fraction'],'insertion_m':p['insertion_m'],'roll':p.get('roll_deg'),'pitch':p.get('pitch_deg'),'max_adjacent_opening_q_increment_rad':float(np.max(abs(np.diff(np.array([x['q'] for x in p['path']]),axis=0))))})
runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py';pink=Path(job['pink_runner']).read_text();opening='def body_at(theta):'+pink.split('def body_at(theta):',1)[1].split('initial_grasp=False',1)[0];unchanged=all(sha(p)==h for p,h in json.loads((out/'FROZEN_INPUT_HASHES.json').read_text()).items());proof={'classification':'OFFLINE_ONLY_NO_PHYSICAL_TRIAL','physical_trials_consumed':0,'runner_path':str(runner),'runner_sha256':sha(runner),'active_closure_sha256':sha(job['active_closure_source']),'helper_sha256':sha(ROOT/'scripts/precontact_trajectory.py'),'unchanged_opening_controller_excerpt_sha256':hashlib.sha256(opening.encode()).hexdigest(),'frozen_inputs_unchanged':unchanged,'cases':records,'offline_supported':unchanged and sha(runner)==job['expected_runner_sha256'],'source_config_hashes':{str(p):sha(p) for p in [ROOT/'scripts/plan_contact_transmission_door.py',ROOT/'scripts/handle_contact_quality.py',out/'whole_plan.json',out/'grasp_plan.json',out/'job.json']},'new_execution_gates':False}
(out/'PREFLIGHT.json').write_text(json.dumps(proof,indent=2));print(json.dumps({k:v for k,v in proof.items() if k!='cases'},indent=2))
for c in records:print('OFFLINE_CASE',{k:v for k,v in c.items() if k not in ['closing','patch']},'patch_score',c['patch']['score'])
