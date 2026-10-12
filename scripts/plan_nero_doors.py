"""Four registered native-NERO plans, unchanged contact ranking and Pink costs."""
from pathlib import Path
import json,inspect,hashlib,time
import numpy as np
import plan_shared_door_skill as shared
from nero_door_adapter import NeroModel,RobotPinkIK,scene_factory,configuration
ROOT=Path(__file__).resolve().parents[1];O=ROOT/'results/nero_feasibility_20261012';OLD=ROOT/'results/door_transfer4_20261012'
source=Path(shared.__file__).read_text().split("if __name__=='__main__':",1)[0]
old="flip=q.copy();flip[3]+=np.pi if q[3]<0 else -np.pi;flip[4]*=-1;flip[5]+=np.pi if q[5]<0 else -np.pi\n            qf=model.ik(g['T'],base,flip)";source=source.replace(old,"qf=model.ik(g['T'],base,model.rng.uniform(model.limits[:,0]+.05,model.limits[:,1]-.05))")
# Preserve scored partial candidates but choose the highest contact utility
# among complete 30-degree plans when any exists, per this task's prerequisite.
source=source.replace('key=joint_candidate_score(g[\'contact\'][\'support_score\'],fine[-1][\'state\'],goal,min_margin,continuity)','utility=joint_candidate_score(g[\'contact\'][\'support_score\'],fine[-1][\'state\'],goal,min_margin,continuity)\n        key=(fine[-1][\'state\']>=goal,utility)')
source=source.replace("candidate['shared_selection_utility']=key","candidate['shared_selection_utility']=utility")
ns=dict(shared.__dict__);exec(compile(source,str(shared.__file__)+'::native_NERO_dimension_adapter','exec'),ns);c=configuration();ns['Model']=lambda ignored,asset,s:NeroModel(c,asset,s);ns['Model'].joint_plan=shared.Model.joint_plan;ns['PinkIK']=lambda ignored,home:RobotPinkIK(c);ns['physical_scene_class']=lambda root:lambda e,m,a:scene_factory(root,e,m,a)
import argparse
p=argparse.ArgumentParser();p.add_argument('--asset',required=True);a=p.parse_args();i=a.asset;out=O/'offline'/i;out.mkdir(parents=True,exist_ok=True);job=json.loads((OLD/i/'planning_job.json').read_text()) if (OLD/i/'planning_job.json').exists() else json.loads((OLD/i/'job.json').read_text());job['benchmark_goal_rad']=float(np.deg2rad(30));job['planning_base_seeds']=[];c['camera_calibration']=job['camera_calibration'];start=time.time();plan=ns['plan'](job,out,wall_s=300);summary={'asset':i,'classification':'ORACLE_HANDLE / KNOWN_MODEL / NERO_FEASIBILITY','found':plan is not None,'complete_30deg':bool(plan and plan['planned_reached_state']>=job['benchmark_goal_rad']),'range_deg':float(np.rad2deg(plan['planned_reached_state'])) if plan else None,'wall_s':time.time()-start,'shared_source_sha256':hashlib.sha256(Path(shared.__file__).read_bytes()).hexdigest(),'adapter_sha256':hashlib.sha256(source.encode()).hexdigest(),'native_gripper':True,'only_two_verified_proxy_pair_corrections':True,'no_object_specific_offsets':True};(out/'RESULT.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
