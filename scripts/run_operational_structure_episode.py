"""Independent model-confidence scheduling; physical baseline remains unchanged."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_active_structure_episode import source as prior_source


def source():
    text=prior_source()
    def replace(old,new):
        nonlocal text
        if text.count(old)!=1:raise RuntimeError('OPERATIONAL_HOOK_CHANGED:'+old[:90])
        text=text.replace(old,new)
    replace(' from active_structure.refinement import refine',' from operational_structure.refinement import refine')
    replace('  from active_structure.discovery import ProvisionalMemory','  from operational_structure.confidence import OperationalMemory as ProvisionalMemory')
    replace("'HELDOUT_MANIPULATION','HELDOUT_DWELL') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']",
            "'HELDOUT_MANIPULATION','HELDOUT_DWELL','OPERATIONAL_CONTINUATION') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']")
    replace("s['relative_translation_slip_m']=retention.drift_m;s['constrained_drive']=torque_diagnostics;",
            "s['relative_translation_slip_m']=retention.drift_m;\n  if hasattr(memory,'monitor_anchor'):\n   from active_structure.discovery import ProvisionalMemory\n   s['global_reconstruction_consistency_error_m']=ProvisionalMemory.consistency_error(memory,np.asarray(s['T_tcp']))\n  s['constrained_drive']=torque_diagnostics;")
    replace("max(s['relative_translation_slip_m'],s['articulation_consistency_error_m'] or 0.)>policy['max_slip_m']",
            "s['relative_translation_slip_m']>policy['max_slip_m']")
    replace("   if slip_s>.1:raise RuntimeError('SUSTAINED_RELATIVE_SLIP')",
    """   if slip_s>.1:raise RuntimeError('SUSTAINED_CONTACT_PLANE_DRIFT')
   if hasattr(memory,'monitor') and memory.estimate is not None:
    if memory.monitor(s['articulation_consistency_error_m'],dt,tick*dt,phase):
     if phase=='ESTIMATED_FOLLOW':drive.active=False
     else:
      # Never refit validation/held-out observations. Test local prediction with
      # the frozen model, rather than mistaking a reference offset for slip.
      from operational_structure.prediction import prediction
      X=np.asarray([r['T_tcp'] for r in rows[-240::8]])
      pp=prediction(X,memory.estimate);cp=memory.policy['confidence']
      explained=pp['maximum_position_error_m']<=cp['explanation_position_m'] and pp['rotation_rmse_rad']<=cp['explanation_rotation_rad']
      memory.events.append({'event':'FROZEN_MODEL_LOCAL_CHECK','t':tick*dt,'phase':phase,'explained':bool(explained),'prediction':pp,'heldout_fitted':False,'physical_failure':False})
      memory.pending=False;memory.inconsistent_s=0.;memory.count_window(explained,X);memory.save_events()
      if memory.failed_refits>=cp['maximum_unexplained_refits']:raise RuntimeError('MODEL_CONFIDENCE_UNRESOLVED')""")
    replace("        if np.linalg.norm(E[:3,3]-start)>=ap['segment_m']:break",
            "        if memory.pending:break\n        if np.linalg.norm(E[:3,3]-start)>=ap['segment_m']*memory.step_scale:break")
    replace("      memory.refresh()", """      if memory.pending:
       drive.active=False
       for pause_tick in range(int(job['active_structure']['confidence']['pause_s']/dt)):
        step()
        if pause_tick%8==0:memory.observe(tcp())
       memory.resolve(tick*dt)
      else:memory.refresh()""")
    replace("      if segment_log[-1]['actual_displacement_m']<.0001:break",
            "      if segment_log[-1]['actual_displacement_m']<.0001 and not memory.events:break")
    replace("     adopt(memory,selected);fit=memory.estimate",
            "     adopt(memory,selected);fit=memory.estimate;memory.monitor_anchor=tcp().copy()")
    # No withheld poses are observed/refitted. Physical continuation is an
    # additional requirement, not implied by an accepted predictive model.
    replace("     phase='HELDOUT_DWELL';drive.active=False;hold(structure_policy['heldout']['dwell_s'])",
    """     phase='HELDOUT_DWELL';drive.active=False;hold(structure_policy['heldout']['dwell_s'])
     # A validated model must actually guide continued action. Keep the held-out
     # protocol unchanged, then spend only the remaining frozen motion budget.
     phase='OPERATIONAL_CONTINUATION';cp=job['active_structure']['continuation'];continuation_start=tick*dt;continuation_steps=[]
     for ci in range(cp['maximum_segments']):
      if memory.angle_deg(tcp())>=cp['target_angle_deg']:break
      if tick*dt-continuation_start>=cp['maximum_s'] or tick*dt-refinement_start>=ap['maximum_sim_s']:break
      measured=np.asarray([x['T_tcp'] for ii,x in enumerate(rows) if ii%8==0 and x['t']>=refinement_start])
      total_path=float(np.linalg.norm(np.diff(measured[:,:3,3],axis=0),axis=1).sum())
      if total_path>=ap['maximum_path_m']:break
      E=tcp();direction=memory.tangent(E);ds=cp['segment_m']*memory.step_scale
      axis=np.asarray(memory.estimate['revolute']['axis']);center=np.asarray(memory.estimate['revolute']['point_on_axis']);radius=max(np.linalg.norm(np.cross(axis,E[:3,3]-center)),.001)
      target=E.copy();target[:3,3]+=direction*ds;target[:3,:3]=Rotation.from_rotvec(memory.follow_sign*axis*ds/radius).as_matrix()@E[:3,:3]
      q=np.asarray(robot.get_joint_positions());solution=ik(target,q[arm])
      if solution is None or model.margin(solution)<=.05:raise RuntimeError('OPERATIONAL_CONTINUATION_NO_SAFE_IK')
      ok,why=collision.check(model.poses(solution,base,finger_q=q[fingers]),target@np.linalg.inv(grasp_tcp)@moving_initial,True)
      if not ok:raise RuntimeError('OPERATIONAL_CONTINUATION_'+why)
      drive.refresh_tangent(direction,E);drive.active=True;start=E[:3,3].copy()
      for cj in range(int(cp['segment_timeout_s']/dt)):
       step()
       if np.linalg.norm(tcp()[:3,3]-start)>=ds:break
       if tick*dt-continuation_start>=cp['maximum_s'] or tick*dt-refinement_start>=ap['maximum_sim_s']:break
      continuation_steps.append({'index':ci,'actual_displacement_m':float(np.linalg.norm(tcp()[:3,3]-start)),'estimated_angle_deg':memory.angle_deg(tcp())})
      (a.output/'operational_continuation.json').write_text(json.dumps({'steps':continuation_steps,'fit_updated':False,'GT_used':False,'heldout_protocol_unchanged':True,'time_s':tick*dt-continuation_start,'total_path_m':total_path}))
     drive.active=False
     ready_now,_=grip_window()
     selected['operational_structure']=bool(selected['operational_predictive_accepted'] and ready_now and memory.angle_deg(tcp())>=5.)
     selected['safe_continuation_estimated_angle_deg']=memory.angle_deg(tcp())
     (a.output/'structure_selection.json').write_text(json.dumps(selected,indent=2))""")
    return text


if __name__=='__main__':exec(compile(source(),str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
