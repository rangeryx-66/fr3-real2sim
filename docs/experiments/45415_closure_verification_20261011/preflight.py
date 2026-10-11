"""No Isaac/robot actuation: exact source loading, archived replay and sweep audit."""
from pathlib import Path
import json,hashlib,pickle,gzip,importlib.util,numpy as np,ast,fcl,sys
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
out=ROOT/'results/45415_closure_verification_20261011';job=json.loads((out/'job.json').read_text());old=ROOT/'results/oracle_door_development_20261011/45415/development_02'
load=lambda p,n:importlib.util.spec_from_file_location(n,p)
classes=[]
for name in ['original_closure','active_closure']:
 p=out/'provenance'/(name+'.py');s=load(p,name);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);classes.append(m.JawCenteredClosure)
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import physical_scene_class
from revolute10_fixed import authored_handle_colliders
model=Model(ROOT/'config/piper.urdf',job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
policy=json.loads((old/'sensor_metadata.json').read_text())['original_control_policy'];controllers=None;counts=[0,0];last=[None,None];peak=[0.,0.];samples=0;transient=[];prev=None;maximum_speed=0.;peak_row=None
for f in sorted((old/'observations_chunks').glob('*.gz')):
 for r in pickle.load(gzip.open(f,'rb')):
  if 5.38<=r['t']<=5.46:transient.append(r)
  if prev is not None and r['phase'] in ['SLOW_CLOSE','CENTER']:
   v=float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(prev['T_tcp'])[:3,3])/(r['t']-prev['t']))
   if v>maximum_speed:maximum_speed=v;peak_row={'t':r['t'],'loads_n':r['forces_n'],'command_increment_rad':float(np.max(abs(np.array(r['command_q_arm'])-prev['command_q_arm']))),'command_velocity_rad_s':r['telemetry']['command_q_velocity_arm']}
  if r['phase'] in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD']:
   if controllers is None:controllers=[c(np.array(r['T_tcp']),policy) for c in classes]
   centers=[(np.array(r['finger_world_poses'][n])@np.r_[model.pads[n].mean(0),1])[:3] for n in ['gripper_link1','gripper_link2']]
   for i,c in enumerate(controllers):
    cmd=c.update(r['T_tcp'],r['aperture_m'],r['telemetry']['pad_load_filtered_n'],centers,1/240)
    if cmd['mode']=='position':
     w=cmd['opening_m']
     if last[i] is not None and w>last[i]+1e-7:counts[i]+=1;peak[i]=max(peak[i],w-last[i])
     last[i]=w
   samples+=1
  prev=r
export=json.loads((old/'cooked_initial.json').read_text());selection=model.manifest['interaction_geometry']['selection'];allowed=authored_handle_colliders(export,selection);sc=physical_scene_class(ROOT)(export,model,allowed)
collision_failures=[];minimum_gripper_object_distance=float('inf')
# Actual archived q/finger feedback, plus substeps between adjacent samples.
for a,b in zip(transient,transient[1:]):
 for f in np.linspace(0,1,5):
  q=np.array(a['q'])*(1-f)+np.array(b['q'])*f;P=model.poses(q[:6],json.loads((out/'whole_plan.json').read_text())['base'],finger_q=q[6:]);ok,why=sc.check(P,np.array(a['T_moving_link']),False)
  if not ok:collision_failures.append(why)
  for rr in sc.robot:
   if rr.body not in ['gripper_link1','gripper_link2','gripper_base','flange_link']:continue
   for ss in sc.scene:
    if '/articulated_pose/' in ss.path:minimum_gripper_object_distance=min(minimum_gripper_object_distance,max(0.,float(fcl.distance(rr.object,ss.object))))
positions=np.array([r['T_tcp'] for r in transient])[:,:3,3];excursion=float(np.max(np.linalg.norm(positions-positions[0],axis=1)))
runner=ROOT/'scripts/run_oracle_door_closure_continuity.py';hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [runner,out/'provenance/active_closure.py',Path(job['baseline_runner']),Path(job['pink_runner'])]}
opening=Path(job['pink_runner']).read_text();opening='def body_at(theta):'+opening.split('def body_at(theta):',1)[1].split('initial_grasp=False',1)[0]
result={'classification':'PRELAUNCH_OFFLINE_ONLY','runner_path':str(runner),'runner_sha256':hashes[str(runner)],'active_closure_path':str(out/'provenance/active_closure.py'),'active_closure_sha256':hashes[str(out/'provenance/active_closure.py')],'loaded_update_code_file':controllers[1].update.__code__.co_filename,'replay':{'inputs':samples,'original_reopenings':counts[0],'fixed_reopenings':counts[1],'original_max_reopening_m':peak[0],'fixed_max_reopening_m':peak[1],'not_physical_validation':True},'unchanged_opening_controller_excerpt_sha256':hashlib.sha256(opening.encode()).hexdigest(),'frozen_dependency_hashes':hashes,'transient':{'maximum_actual_TCP_speed_m_s':maximum_speed,'peak_context':peak_row,'actual_TCP_excursion_over_window_m':excursion,'window_s':[transient[0]['t'],transient[-1]['t']],'archived_swept_collision_failures':collision_failures,'minimum_cooked_gripper_object_distance_m':minimum_gripper_object_distance,'all_archived_pad_loads_zero':all(max(r['forces_n'].values())==0 for r in transient),'interpretation':'Real pre-contact position/velocity-feedforward equilibrium reset, not sensor noise. Keep unchanged to isolate the aperture repair; record it. Existing contact/collision and loaded-speed protections are unchanged. No new acceptance tolerance or physical execution gate.'},'physical_trials_consumed':0}
if result['runner_sha256']!=job['expected_runner_sha256'] or result['active_closure_sha256']!=job['expected_active_closure_sha256'] or counts[1]!=0:raise RuntimeError('PRELAUNCH_SOURCE_OR_REPLAY_CHECK_FAILED_NO_PHYSICS')
(out/'PREFLIGHT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
