"""Same physical prefix; independent evaluation logger and internal ablations."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_operational_structure_episode import source as original


def source(method='LOG_ONLY'):
    text=original()
    def replace(a,b):
        nonlocal text
        if text.count(a)!=1:raise RuntimeError('PAPER_HOOK_CHANGED:'+a[:100])
        text=text.replace(a,b)
    replace(' gt_gate=GroundTruthGate();gt_gate.protect(scene)'," from paper_structure.evaluation_logger import EvaluationLogger\n evaluation_writer=EvaluationLogger(scene,a.output/'evaluation_private')\n gt_gate=GroundTruthGate();gt_gate.protect(scene)")
    replace('rows.append(s);tick+=1',"rows.append(s);\n  if tick%8==0:evaluation_writer.append(s['t'],phase,s['T_tcp'])\n  tick+=1")
    replace(' finally:\n  world.pause()', ' finally:\n  world.pause()\n  evaluation_writer.close()')
    replace("   validate_robot_jacobian('grasp')", "   validate_robot_jacobian('grasp')\n   (a.output/'observable_grasp_start.json').write_text(json.dumps({'q':np.asarray(robot.get_joint_positions()).tolist(),'T_ee':tcp().tolist(),'grip':detail,'GT_read':False}))")
    if method=='LOG_ONLY':return text
    if method not in ('B0','B1','B2'):raise ValueError(method)
    if method=='B0':
        start=text.index('   if a.no_operation:');end=text.index('\n except BaseException as error:',start)
        # Model-free baseline uses the EXISTING constrained drive and four
        # observed-frame probes. No global hinge fit/tangent enters commands.
        text=text[:start]+'''   ap=job['active_structure']['refinement'];direction=None
   for attempt in range(4):
    direction=memory.begin_attempt(attempt,tick*dt);drive.set_direction(direction,tcp());drive.active=True
    phase='EXPLORATORY';attempt_start=tick*dt;reason='UNOBSERVABLE_ATTEMPT_TIMEOUT'
    for j in range(int(35/dt)):
     step()
     if j%8==0:
      memory.observe(tcp());elapsed=tick*dt-attempt_start
      if memory.travel()>=.005:reason='USEFUL_MODEL_FREE_MOTION';break
      if elapsed>=12. and memory.travel(memory.start_index)<.002:reason='SAFE_LOW_EXCITATION';break
      if memory.travel()>.020:reason='TOTAL_PROBE_EXCITATION_BUDGET';break
    memory.finish_attempt(tick*dt,reason)
    if reason in ('USEFUL_MODEL_FREE_MOTION','TOTAL_PROBE_EXCITATION_BUDGET'):break
    phase='PROBE_HOLD';drive.active=False;hold(1.)
   if memory.travel()<.005:raise RuntimeError('UNOBSERVABLE')
   phase='ESTIMATED_FOLLOW';refinement_start=tick*dt;path_length=0.;last=tcp()[:3,3].copy()
   for segment_index in range(ap['maximum_segments']):
    if tick*dt-refinement_start>=ap['maximum_sim_s'] or path_length>=ap['maximum_path_m']:break
    E=tcp();target=E.copy();target[:3,3]+=direction*ap['segment_m'];q=np.asarray(robot.get_joint_positions());solution=ik(target,q[arm])
    if solution is None or model.margin(solution)<=.05:raise RuntimeError('MODEL_FREE_NO_SAFE_IK')
    ok,why=collision.check(model.poses(solution,base,finger_q=q[fingers]),target@np.linalg.inv(grasp_tcp)@moving_initial,True)
    if not ok:raise RuntimeError('MODEL_FREE_'+why)
    drive.refresh_tangent(direction,E);drive.active=True;start=E[:3,3].copy()
    for k in range(int(ap['segment_timeout_s']/dt)):
     step()
     if k%8==0:
      E=tcp();path_length+=float(np.linalg.norm(E[:3,3]-last));last=E[:3,3].copy()
      if np.linalg.norm(E[:3,3]-start)>=ap['segment_m']:break
      if tick*dt-refinement_start>=ap['maximum_sim_s'] or path_length>=ap['maximum_path_m']:break
    # Same measured-orientation 6 degree stopping budget; no GT joint angle.
    if np.rad2deg(Rotation.from_matrix(tcp()[:3,:3]@E0[:3,:3].T).magnitude())>=ap['target_angle_deg']:break
   phase='HELDOUT_MANIPULATION';heldout_start=len(rows);drive.active=True
   (a.output/'heldout_start.json').write_text(json.dumps({'time_s':tick*dt,'T_ee':tcp().tolist(),'no_GT':True,'model':'none','fit_usage':'never'}))
   hold(structure_policy['heldout']['active_s']);phase='HELDOUT_DWELL';drive.active=False;hold(structure_policy['heldout']['dwell_s'])
   (a.output/'heldout_completion.json').write_text(json.dumps({'completed':True,'fit_updated':False,'GT_inputs':False}))
   phase='FINAL_HOLD';hold(2.);ready,detail=grip_window();success=bool(ready and retention.drift_m<=policy['max_slip_m']);status='MODEL_FREE_HOLD_COMPLETE' if success else 'MODEL_FREE_HOLD_FAILED'
''' +text[end:]
    else:
        if method=='B1':
            replace('from operational_structure.confidence import OperationalMemory as ProvisionalMemory','from paper_structure.memory import OnceMemory as ProvisionalMemory')
            replace('     selected=refine(rows,discovery_snapshot,structure_policy,a.output)',"     selected={'accepted':False,'operational_predictive_accepted':False,'high_fidelity_accepted':False,'refined':discovery_snapshot,'status':'ONE_SHOT_FROZEN'}\n     (a.output/'structure_selection.json').write_text(json.dumps(selected))")
        # Validation selects model before the separate held-out action. Reject
        # update, not all previously completed physical operation.
        replace("     if not selected['accepted']:raise RuntimeError(selected['status'])\n     adopt(memory,selected);fit=memory.estimate;memory.monitor_anchor=tcp().copy()",'''     if selected['accepted']:
      adopt(memory,selected)
      selected['controller_model']='accepted_refined'
     else:
      old_axis=np.asarray(memory.estimate['revolute']['axis']);new_axis=np.asarray(discovery_snapshot['revolute']['axis'])
      if old_axis@new_axis<0:memory.follow_sign*=-1.
      memory.estimate=json.loads(json.dumps(discovery_snapshot));memory.save()
      selected['controller_model']='discovery_fallback';selected['rejected_update_not_physical_failure']=True
     fit=memory.estimate;memory.monitor_anchor=tcp().copy()
     (a.output/'structure_selection.json').write_text(json.dumps(selected,indent=2))''')
    replace("  report.update(structure_protocol=bool(structure_policy)","  report.update(paper_internal_baseline='"+method+"',evaluation_trajectory_is_private=True)\n  report.update(structure_protocol=bool(structure_policy)")
    return text

if __name__=='__main__':
    import json,argparse
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args()
    method=json.loads(a.job.read_text()).get('paper_method','LOG_ONLY')
    exec(compile(source(method),str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
