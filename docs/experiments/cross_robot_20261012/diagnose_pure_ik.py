"""Joint-only counterfactual, explicitly NOT collision-free or executable.
No task controller or collision protection is modified. This separates IK
limits from the foreign collision/SRDF integration blockers.
"""
from pathlib import Path
import json,numpy as np
from cross_robot_adapter import RobotModel,RobotPinkIK
from cross_robot_offline import arc
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/cross_robot_20261012';old=ROOT/'results/door_transfer4_20261012';result=[]
class NoSpatialAssessment:
 robot=[];scene=[];moving_reference=np.eye(4)
 def check(self,*a):return True,'NO_SPATIAL_ASSESSMENT_DIAGNOSTIC_ONLY'
for r,c in json.loads((O/'ROBOT_MODELS.json').read_text()).items():
 for a in ['45134','45443','45176','45415']:
  job=json.loads((old/a/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());c['camera_calibration']=job['camera_calibration'];m=RobotModel(c,job['asset_root'],source);plan=json.loads((old/a/'whole_plan.json').read_text());m.study_width=float(job['grasp_geometry_candidates'][0]['dimensions_m'][1]);solver=RobotPinkIK(c);base=plan['base'];T=np.array(plan['T_grasp']);x=arc(m,solver,NoSpatialAssessment(),T,base)
  result.append({'robot':r,'asset':a,'mode':'PURE_IK_NOT_A_PLAN_OR_PHYSICAL_RESULT','collision_checks':False,'never_used_for_execution':True,'station':base,'same_grasp':T.tolist(),'range_deg':x['range_deg'],'initial_ik':x['initial_ik'],'blocker':x['blocker'],'limiting':x['limiting'],'path':x['path']})
(O/'PURE_IK_DIAGNOSTIC.json').write_text(json.dumps(result,indent=2));print([(r['robot'],r['asset'],r['range_deg']) for r in result])
