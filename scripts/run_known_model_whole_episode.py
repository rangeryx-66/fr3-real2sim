"""Independent KNOWN_MODEL_DIAGNOSTIC using the existing contact baseline.
No attachment, object force, execution-time object-state setter or geometry edit.
"""
from pathlib import Path
import sys,json,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
# Bind diagnostic helpers before the legacy loader adjusts module search paths.
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration


def source():
 text=(ROOT/'scripts/run_piper_contact_baseline.py').read_text()
 def replace(a,b):
  nonlocal text
  if text.count(a)!=1:raise RuntimeError('KNOWN_DIAGNOSTIC_HOOK_CHANGED:'+a[:100])
  text=text.replace(a,b)
 replace(" a=parser().parse_args();a.output.mkdir(parents=True,exist_ok=True)",''' from types import SimpleNamespace
 p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);args=p.parse_args();job=json.loads(args.job.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text())
 a=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),plan=Path(job['plan']),output=Path(job['output']),gpu=job['gpu'],trial=True,candidate=0,no_operation=False,deadline_shanghai=job['deadline_shanghai'],ownership=None)
 a.output.mkdir(parents=True,exist_ok=True)
 kind=job['skill']['joint_type'];max_state=0.;first_divergence=None;command_state=0.;planned_reference=None
''')
 replace("scene=bootstrap(a,base);", "from articulated_interaction_skill.scene import install\n install()\n from interactive_twin_refinement.assembly import install_setup_capture\n install_setup_capture()\n from interactive_twin.plant import bootstrap_job\n scene=bootstrap_job(a,base,job);")
 replace("chosen=plan['trial_candidates'][0]", "chosen=plan['trial_candidates'][0]")
 # Preserve all robot/object setup, gripper closure and position servo settings.
 replace(" variants=[(0,0,0),(0,-.004,0),(0,.004,0),(-.01,0,0),(.01,0,0),(0,0,-5),(0,0,5),(-.01,-.004,0),(.01,-.004,0),(-.01,.004,0),(.01,.004,0),(0,-.004,5)]", " variants=[(0,0,0)]")
 replace("   loss_s>.15", "   loss_s>.15") if False else None
 replace("  if slip_s>.1:raise RuntimeError('SUSTAINED_RELATIVE_SLIP')", "  if slip_s>.1:s['relative_motion_event']='GRASP_RELATIVE_MOTION' # diagnostic only")
 replace(" policy=json.loads", " from wrist_reconstruction.force_policy import TemporalForceGuard\n force_guard=TemporalForceGuard(job['wrist_experiment']['force_policy'])\n policy=json.loads")
 replace("  nonlocal tick,loss_s,slip_s", "  nonlocal tick,loss_s,slip_s,max_state,first_divergence")
 replace("'door_angle_deg':float(np.rad2deg(scene['articulation'].get_joint_positions()[0]))", "'door_angle_deg':float(np.rad2deg(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])) if kind=='revolute' else float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])")
 replace("rows.append(s);tick+=1", "s['force_event']=force_guard.update(s['forces_n'],dt,s['t']);s['planned_articulation_state']=command_state;s['planned_T_tcp']=None if planned_reference is None else planned_reference.tolist();rows.append(s);tick+=1\n  max_state=max(max_state,s['door_angle_deg'])\n  if planned_reference is not None and first_divergence is None:\n   error=float(np.linalg.norm(np.asarray(s['T_tcp'])[:3,3]-planned_reference[:3,3]));state_error=abs(s['door_angle_deg']-(np.rad2deg(command_state) if kind=='revolute' else command_state))\n   if error>.003 or state_error>(2. if kind=='revolute' else .003):first_divergence={'t':s['t'],'phase':phase,'planned_state':command_state,'actual_state':s['door_angle_deg'],'TCP_position_error_m':error,'definition':'diagnostic first >3mm TCP or >2deg/3mm object-state discrepancy; never a manipulation veto'}")
 replace(" and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']:raise RuntimeError('EXISTING_LOW_PRELOAD_FORCE_LIMIT')", " and s['force_event']['status'].startswith('HARD_FORCE_STOP'):raise RuntimeError(s['force_event']['status'])")
 # Same current safety caps; task speed stays below the original 5mm/s cap.
 replace("  if reference is not None:\n   loss_s=", "  if reference is not None:\n   if np.any(abs(np.asarray(robot.get_joint_velocities())[arm])>vel/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')\n   if len(rows)>1 and np.linalg.norm(np.asarray(rows[-1]['T_tcp'])[:3,3]-np.asarray(rows[-2]['T_tcp'])[:3,3])/dt>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')\n   loss_s=")
 start=text.index('   E0=tcp();D0=moving();theta0=')
 end=text.index('\n except BaseException as error:',start)
 text=text[:start]+'''   E0=tcp();D0=moving();theta0=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]);seed=np.asarray(robot.get_joint_positions())[arm]
   origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0});arc=[]
   # A small measured closure alignment is propagated through the entire known
   # path, checked before task execution; no new local-only station search.
   for saved in whole['path']:
    s=max(theta0,float(saved['state']));body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:s});D=body@np.linalg.inv(origin)
    target=D@E0;q=ik(target,seed)
    if q is None:raise RuntimeError('ALIGNED_WHOLE_PATH_NO_IK')
    if np.max(abs(q-seed))>.2:raise RuntimeError('ALIGNED_IK_BRANCH_DISCONTINUITY')
    for f in np.linspace(0,1,max(2,int(np.max(abs(q-seed))/.025)+2)):
     mq=(1-f)*seed+f*q;ok,why=collision.check(model.poses(mq,base,finger_q=np.asarray(robot.get_joint_positions())[fingers]),D@D0,True)
     if not ok:raise RuntimeError('ALIGNED_WHOLE_PATH_'+why)
    from wrist_reconstruction.planner import camera_clearance
    from wrist_reconstruction.geometry import calibration
    ok,why=camera_clearance(collision,model.poses(q,base,finger_q=np.asarray(robot.get_joint_positions())[fingers]),calibration(job['camera_calibration']))
    if not ok:raise RuntimeError(why)
    arc.append({'q':q.tolist(),'state':s,'T_tcp':target.tolist(),'margin_rad':model.margin(q)});seed=q
   preflight={'known_model_whole_path':arc,'min_joint_margin_rad':min(x['margin_rad'] for x in arc),'planning_goal':whole['goal_state'],'GT_planning':True,'end_retreat':whole['end_retreat']};(a.output/'aligned_whole_path.json').write_text(json.dumps(preflight,indent=2))
   previous=E0.copy()
   for waypoint in arc:
    phase='OPEN_5_DEG';command_state=waypoint['state'];planned_reference=np.asarray(waypoint['T_tcp'])
    distance=np.linalg.norm(planned_reference[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)
    if force_guard.pending:
     phase='FORCE_HOLD';hold(.25)
     if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
     force_guard.pending=False;duration*=2
    move(waypoint['q'],duration);previous=planned_reference.copy()
   # Smooth spline ends at zero arm velocity. Gripper effort/gains are retained.
   phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.);ready,detail=grip_window()
   actual_goal=np.rad2deg(whole['goal_state']) if kind=='revolute' else whole['goal_state'];success=max_state>=actual_goal-(.5 if kind=='revolute' else .002);status='KNOWN_MODEL_RANGE_REACHED' if success else 'KNOWN_MODEL_ACTUAL_RANGE_BELOW_PLAN'
''' +text[end:]
 replace("'status':status,'success':success,'mode':'known-model physical-contact baseline'", "'status':status,'success':success,'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'GT_collision_scene':True,'unknown_structure_identification_success':False,'autonomous_reconstruction_success':False,'maximum_actual_state':max_state,'actual_units':'degrees' if kind=='revolute' else 'meters','first_planned_actual_divergence':first_divergence,'whole_task_plan':job['whole_task_plan'],'grip_control_continuous':True,'end_release_retreat_planned':True,'base_prepositioned_before_grasp':True")
 replace("cv2.putText(image,f'KNOWN-MODEL / PHYSICAL CONTACT |", "cv2.putText(image,f'KNOWN_MODEL_DIAGNOSTIC / PHYSICAL CONTACT |")
 ast.parse(text);return text

if __name__=='__main__':
 text=source();exec(compile(text,str(ROOT/'scripts/run_known_model_whole_episode.py'),'exec'),globals())
