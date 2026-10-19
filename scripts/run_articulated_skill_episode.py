"""Guarded independent multistate entry; inherited physical prefix unchanged."""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_paper_structure_episode import source as parent

def source():
 text=parent('LOG_ONLY')
 def replace(a,b):
  nonlocal text
  if text.count(a)!=1:raise RuntimeError('SKILL_HOOK_CHANGED:'+a[:80])
  text=text.replace(a,b)
 replace(' from interactive_twin_refinement.assembly import install_setup_capture',' from articulated_interaction_skill.scene import install\n install()\n from interactive_twin_refinement.assembly import install_setup_capture')
 replace(' if job.get("active_structure"):'," if job['skill']['joint_type']=='prismatic':\n  from articulated_interaction_skill.memory import PrismaticMemory\n  InteractionMemory=PrismaticMemory\n elif job.get(\"active_structure\"):")
 replace(" video=subprocess.Popen", " from articulated_interaction_skill.capture import CaptureRecorder\n from articulated_interaction_skill.effort import EffortRecorder\n from articulated_interaction_skill.force_sensor import ForceSensor\n skill_capture=CaptureRecorder(scene,a.output,job);skill_effort=EffortRecorder(a.output);skill_effort.sensor=ForceSensor(scene,export,a.output)\n video=subprocess.Popen")
 replace("  if tick%8==0:evaluation_writer.append(s['t'],phase,s['T_tcp'])", "  if tick%8==0:\n   evaluation_writer.append(s['t'],phase,s['T_tcp'])\n   if drive is not None:skill_effort.append(s,dt,drive.direction,None if memory is None else memory.estimate)")
 begin=text.index('   if a.no_operation:',text.index("   phase='COMPLIANT_SETTLE';hold(.5)"));end=text.index('\n except BaseException as error:',begin)
 text=text[:begin]+'''   from articulated_interaction_skill.session import run as run_skill
   def skill_phase(value):
    nonlocal phase
    phase=value
   def increment_safe(direction,ds):
    E=tcp();target=E.copy();target[:3,3]+=direction*ds
    if job['skill']['joint_type']=='revolute':
     rr=memory.estimate['revolute'];axis=np.asarray(rr['axis']);radius=max(np.linalg.norm(np.cross(axis,E[:3,3]-rr['point_on_axis'])),.001)
     target[:3,:3]=Rotation.from_rotvec(memory.follow_sign*axis*ds/radius).as_matrix()@E[:3,:3]
    q=np.asarray(robot.get_joint_positions());solution=ik(target,q[arm])
    if solution is None:return False,'NO_INCREMENTAL_IK'
    if model.margin(solution)<=.05:return False,'LOW_JOINT_MARGIN'
    return collision.check(model.poses(solution,base,finger_q=q[fingers]),target@np.linalg.inv(grasp_tcp)@moving_initial,True)
   runtime=SimpleNamespace(policy=job['skill'],memory=memory,drive=drive,tcp=tcp,time=lambda:tick*dt,tick=lambda:tick,step=step,hold=hold,phase=skill_phase,grip=grip_window,capture=skill_capture,effort=skill_effort,base=base,grasp=grasp_tcp,increment_safe=increment_safe)
   skill_result=run_skill(runtime);fit=memory.final_fit();following=True;status=skill_result['status'];success=False
''' +text[end:]
 replace("'estimated_angle_deg':0. if memory is None else memory.angle_deg(E)", "'estimated_articulation_state':0. if memory is None else memory.state(E) if job['skill']['joint_type']=='prismatic' else memory.angle_deg(E),'estimated_articulation_units':'m' if job['skill']['joint_type']=='prismatic' else 'deg','estimated_angle_deg':0. if memory is None else memory.angle_deg(E)")
 replace('INTERACTIVE TWIN / SIM SURROGATE |', 'ARTICULATED MULTISTATE / PHYSICAL CONTACT SIM |')
 replace('EE estimate={s["estimated_angle_deg"]:.2f} deg', 'EE state={s["estimated_articulation_state"]:.4f} {s["estimated_articulation_units"]}')
 replace('  evaluation_writer.close()',"  evaluation_writer.close()\n  skill_effort.close()\n  skill_capture.flush(status)")
 replace("scene['articulation'].get_joint_positions()[0]", "scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]")
 # Legacy reporting assumed radians/door degrees. Only change post-stop units.
 replace("  actual_angle=float(np.rad2deg(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]));evaluation=evaluate(fit,gt,np.asarray(fit_poses[0])[:3,3],fit_poses) if fit_poses else {'type_correct':False}",
 "  actual_joint=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]);actual_angle=float(np.rad2deg(actual_joint)) if job['skill']['joint_type']=='revolute' else actual_joint\n  evaluation=evaluate(fit,gt,np.asarray(fit_poses[0])[:3,3],fit_poses) if fit_poses else {'type_correct':False}")
 replace('  displacement=actual_angle-float(np.rad2deg(job.get(\'initial_articulation_rad\',0.)))',
 "  displacement=actual_angle-float(np.rad2deg(job.get('initial_articulation_rad',0.))) if job['skill']['joint_type']=='revolute' else actual_joint-float(job.get('initial_articulation_rad',0.))")
 replace('  evaluation.update(actual_door_displacement_deg=displacement,actual_final_door_angle_deg=actual_angle,gt_access_after_saved_fit=True,gt_access_after_world_pause=True)',
 "  evaluation.update(actual_joint_displacement=displacement,actual_final_joint_state=actual_angle,joint_state_units='degrees' if job['skill']['joint_type']=='revolute' else 'meters',gt_access_after_saved_fit=True,gt_access_after_world_pause=True)")
 replace("  report.update(experiment='interactive_twin'", "  report.update(skill_capture_states=len(skill_capture.states),skill_goal=job['skill']['goal'],requested_joint_type=job['skill']['joint_type'])\n  report.update(experiment='interactive_twin'")
 ast.parse(text);return text

if __name__=='__main__':exec(compile(source(),str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
