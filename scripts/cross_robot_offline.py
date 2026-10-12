"""Bounded cross-robot analysis around the existing shared grasp planner.
No Isaac contact episode or actuator substitution is performed here.
"""
from pathlib import Path
import argparse,json,time,inspect,hashlib
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
from cross_robot_adapter import RobotModel,RobotPinkIK,scene_factory
from articulated_demo.kinematics import transform
from interactive_twin_recovery.mobile import at_base,candidate_bases
import plan_shared_door_skill as shared
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');OUT=ROOT/'results/cross_robot_20261012';OLD=ROOT/'results/door_transfer4_20261012'

def limiting(model,q):
 m=np.minimum(q-model.limits[:,0],model.limits[:,1]-q);i=int(m.argmin());return {'joint':model.names[i],'margin_rad':float(m[i]),'q_rad':float(q[i]),'limit_rad':model.limits[i].tolist()}

def arc_once(model,solver,scene,T,base,seed=None):
 goal=np.deg2rad(30);q=solver.solve(T,base,seed,starts=4 if seed is None else 1);rows=[];why='NO_INITIAL_PINK_IK';fail=None
 if q is None:return {'path':rows,'range_deg':0.,'initial_ik':False,'blocker':why,'solver_last_error':solver.last_error,'limiting':None}
 for degree in range(31):
  X=model.door_target(T,np.deg2rad(degree));qn=solver.solve(X,base,q)
  if qn is None:why='NO_CONTINUOUS_PINK_IK';fail={'angle_deg':degree,'last_configuration':limiting(model,q),'solver_last_error':solver.last_error};break
  q=qn;ok,why=scene.check(model.poses(q,base,width=model.study_width),model.door_target(scene.moving_reference,np.deg2rad(degree)),True)
  if ok:ok,why=shared.camera_clearance(scene,model.poses(q,base,width=model.study_width),shared.calibration(model.config['camera_calibration']))
  if model.margin(q)<.05:ok=False;why='EXISTING_JOINT_MARGIN'
  if not ok:fail={'angle_deg':degree,'configuration':limiting(model,q)};break
  rows.append({'angle_deg':degree,'q':q.tolist(),'T_tcp':X.tolist(),'margin_rad':model.margin(q),'limiting':limiting(model,q)})
 return {'path':rows,'range_deg':rows[-1]['angle_deg'] if rows else 0.,'initial_ik':True,'blocker':None if len(rows)==31 else why,'first_blocked':fail,'minimum_margin_rad':min((r['margin_rad'] for r in rows),default=None),'limiting':limiting(model,q),'solver_calls':solver.calls}

def arc(model,solver,scene,T,base,seed=None):
 # Same two-branch budget for every robot/mode. A second explicit seed avoids
 # the home-seed duplication in the initial dimension-only adapter.
 candidates=[]
 for initial in [seed,solver.rng.uniform(model.limits[:,0]+.05,model.limits[:,1]-.05)]:
  r=arc_once(model,solver,scene,T,base,initial);candidates.append(r)
 best=max(candidates,key=lambda r:(bool(r['path']),r['range_deg'],r.get('minimum_margin_rad') or 0.))
 best['branch_audit']=[{'range_deg':r['range_deg'],'initial_ik':r['initial_ik'],'blocker':r['blocker'],'limiting':r['limiting']} for r in candidates]
 return best

def approach_check(model,solver,scene,T,base,arc_result):
 if not arc_result['path']:return {'feasible':False,'reason':'NO_COLLISION_FREE_INITIAL_GRASP','steps_checked':0}
 pre=T.copy();pre[:3,3]-=.04*T[:3,2];q=solver.solve(pre,base,np.array(arc_result['path'][0]['q']),starts=2)
 if q is None:return {'feasible':False,'reason':'NO_PREGRASP_IK','steps_checked':0}
 qpre=q.copy();steps=[]
 for f in np.linspace(0,1,21):
  X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];q=solver.solve(X,base,q)
  if q is None:return {'feasible':False,'reason':'APPROACH_NO_IK','steps_checked':len(steps)}
  ok,why=scene.check(model.poses(q,base,width=.1),scene.moving_reference,False)
  if ok:ok,why=shared.camera_clearance(scene,model.poses(q,base,width=.1),shared.calibration(model.config['camera_calibration']))
  if not ok:return {'feasible':False,'reason':why,'steps_checked':len(steps)}
  steps.append(q.tolist())
 return {'feasible':True,'reason':None,'steps_checked':len(steps),'minimum_margin_rad':min(model.margin(np.array(q)) for q in steps),'q_pregrasp':qpre.tolist(),'approach':steps,'home_route':'only independently optimized full plans include RRT home-to-pregrasp validation'}

def same_grasp_search(model,config,job,plan,export,out):
 T=np.array(plan['T_grasp']);base=plan['base'];allowed=shared.authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);sc=scene_factory(ROOT,at_base(export,job['source_base'],base),model,allowed);solver=RobotPinkIK(config)
 first=arc(model,solver,sc,T,base,np.array(plan['path'][0]['q']) if config['robot']=='piper' else None);first['approach']=approach_check(model,solver,sc,T,base,first)
 (out/'same_grasp_same_station.json').write_text(json.dumps({'mode':'SAME_GRASP_SAME_PIPER_STATION','base':base,'T_grasp':T.tolist(),**first},indent=2))
 normal=model.asset_T[:3,:3]@np.array(job['grasp_geometry_candidates'][0]['outward_normal_root']);pool=candidate_bases(base,{'anchor_world_m':T[:3,3].tolist(),'outward_normal_world':normal.tolist()},job['wrist_experiment']['mobile_scan']);start=time.time();audit=[];best=None
 # The same 300 s ceiling and candidate order are used for all robot platforms.
 for b in [base]+pool:
  if time.time()-start>300:break
  s=scene_factory(ROOT,at_base(export,job['source_base'],b),model,allowed);r=arc(model,solver,s,T,b);audit.append({'base':b,'range_deg':r['range_deg'],'initial_ik':r['initial_ik'],'blocker':r['blocker'],'limiting':r['limiting']})
  if r['path'] and (best is None or (r['range_deg'],r['minimum_margin_rad'])>(best['range_deg'],best['minimum_margin_rad'])):best={'base':b,'T_grasp':T.tolist(),**r}
  if best and best['range_deg']>=30:break
 if best:best['approach']=approach_check(model,solver,scene_factory(ROOT,at_base(export,job['source_base'],best['base']),model,allowed),T,best['base'],best)
 result={'mode':'SAME_GRASP_OPTIMIZED_STATION','best':best,'candidate_audit':audit,'wall_s':time.time()-start,'search_ceiling_s':300,'collision_arm_proxy':'official URDF hulls' if config['robot']!='piper' else 'frozen native PiPER','mount_status':config['mount_status']};(out/'same_grasp_station_search.json').write_text(json.dumps(result,indent=2));return result

def own_plan(config,job,out):
 # Compile a dimension-only variant of the actual shared planner. Geometric
 # scores, approach checks and collision/camera policy remain literal.
 text=Path(shared.__file__).read_text();text=text.split("if __name__=='__main__':",1)[0]
 old="flip=q.copy();flip[3]+=np.pi if q[3]<0 else -np.pi;flip[4]*=-1;flip[5]+=np.pi if q[5]<0 else -np.pi\n            qf=model.ik(g['T'],base,flip)"
 if old not in text:raise ValueError('offline source anchor changed')
 text=text.replace(old,"qf=model.ik(g['T'],base,model.rng.uniform(model.limits[:,0]+.05,model.limits[:,1]-.05))")
 ns=dict(shared.__dict__);exec(compile(text,str(shared.__file__)+'::dimension_adapter','exec'),ns)
 ns['Model']=lambda ignored,asset,source:RobotModel(config,asset,source);ns['PinkIK']=lambda ignored,home:RobotPinkIK(config);ns['physical_scene_class']=lambda root:lambda export,model,allowed:scene_factory(root,export,model,allowed)
 original_joint_plan=shared.Model.joint_plan
 class PlanProxy:
  joint_plan=staticmethod(original_joint_plan)
 ns['Model'].joint_plan=original_joint_plan
 start=time.time();plan=ns['plan'](job,out/'own_plan',wall_s=300)
 return {'found':plan is not None,'path':None if plan is None else str(out/'own_plan/candidate.json'),'wall_s':time.time()-start,'mount_status':config['mount_status'],'shared_source_sha256':hashlib.sha256(Path(shared.__file__).read_bytes()).hexdigest(),'dimension_adapter_sha256':hashlib.sha256(text.encode()).hexdigest(),'same_gripper':True}

def ablation(config,model,job,plan,export,out):
 if config['dof']!=7:return None
 chosen=json.loads((out/'same_grasp_station_search.json').read_text())['best']
 fallback=chosen is None
 if fallback:
  source=json.loads((out/'same_grasp_same_station.json').read_text());chosen={'base':source['base'],'T_grasp':source['T_grasp'],**source}
 T=np.array(chosen['T_grasp']);b=chosen['base'];allowed=shared.authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);seed=np.array(chosen['path'][0]['q']) if chosen['path'] else np.array(config['home_q']);jn=config['arm_joint_names'][2]
 values=[(-.6,'predefined'),(0.,'predefined'),(.6,'predefined')]
 if chosen['path']:values.append((float(seed[2]),'matched_full7_initial_configuration'))
 variants=[]
 for value,kind in values:
  solver=RobotPinkIK(config,{jn:value});sc=scene_factory(ROOT,at_base(export,job['source_base'],b),model,allowed);r=arc(model,solver,sc,T,b,seed);r['approach']=approach_check(model,solver,sc,T,b,r);variants.append({'joint':jn,'fixed_rad':value,'value_source':kind,'configuration_dof':solver.robot.nq,'same_base':b,'same_grasp':T.tolist(),**r})
 result={'label':'REDUNDANCY_APPROXIMATION_NOT_PIPER_MODEL','base':b,'T_grasp':T.tolist(),'full7_range_deg':chosen['range_deg'],'full7':chosen,'fixed_joint_ordinal':3,'fallback_to_registered_PiPER_station_due_to_no_full7_feasible_station':fallback,'variants':variants};(out/'redundancy_ablation.json').write_text(json.dumps(result,indent=2));return result

def main():
 p=argparse.ArgumentParser();p.add_argument('--robot',required=True);p.add_argument('--asset',required=True);a=p.parse_args();config=json.loads((OUT/'ROBOT_MODELS.json').read_text())[a.robot];job=json.loads((OLD/a.asset/'planning_job.json').read_text()) if (OLD/a.asset/'planning_job.json').exists() else json.loads((OLD/a.asset/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());config['camera_calibration']=job['camera_calibration'];job['source_base']=source['robot_base_pose'];job['benchmark_goal_rad']=float(np.deg2rad(30));job['planning_base_seeds']=[];job['collision_export']=job.get('collision_export',str(OLD/a.asset/'cooked_initial.json'));model=RobotModel(config,job['asset_root'],source);plan=json.loads((OLD/a.asset/'whole_plan.json').read_text());export=json.loads(Path(job['collision_export']).read_text());model.study_width=plan.get('contact_geometry_audit',{}).get('physical_aperture_m',float(job['grasp_geometry_candidates'][0]['dimensions_m'][1]));out=OUT/'offline'/a.robot/a.asset;out.mkdir(parents=True,exist_ok=True);start=time.time()
 same=same_grasp_search(model,config,job,plan,export,out);own=own_plan(config,job,out);abl=ablation(config,model,job,plan,export,out)
 summary={'robot':a.robot,'asset':a.asset,'classification':'ORACLE_HANDLE / KNOWN_MODEL / CROSS_ROBOT_ABLATION','same_grasp_fixed_station':json.loads((out/'same_grasp_same_station.json').read_text())['range_deg'],'same_grasp_optimized_station':None if same['best'] is None else same['best']['range_deg'],'own_plan':own,'ablation_complete':abl is not None,'wall_s':time.time()-start,'physical_validation':'NOT_OFFLINE_EVIDENCE','mount_status':config['mount_status']};(out/'SUMMARY.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
