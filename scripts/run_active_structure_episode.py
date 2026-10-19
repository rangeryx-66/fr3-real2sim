"""Independent provisional -> bounded refinement, frozen physical baseline."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from run_cross_object_structure_episode import source as prior_source


def source():
    text=prior_source()
    def replace(old,new):
        nonlocal text
        if text.count(old)!=1:raise RuntimeError('ACTIVE_STRUCTURE_HOOK_CHANGED:'+old[:100])
        text=text.replace(old,new)
    replace(' from cross_object_structure.refinement import refine,adopt',
            ' from cross_object_structure.refinement import adopt\n from active_structure.refinement import refine')
    replace(' from interaction_identification.act2see_loop import ConstrainedDrive,InteractionMemory,TactileRetention',
            ' from interaction_identification.act2see_loop import ConstrainedDrive,InteractionMemory,TactileRetention\n if job.get("active_structure"):\n  from active_structure.discovery import ProvisionalMemory\n  InteractionMemory=lambda T,out:ProvisionalMemory(T,out,job["active_structure"])')
    replace("    fit=memory.estimate;probe_metrics=", "    if job.get('active_structure'):\n     (a.output/'discovery_decision.json').write_text(json.dumps({'status':memory.discovery_status,'evidence':memory.last_evidence,'not_final_identification':memory.discovery_status=='PROVISIONAL_REVOLUTE','GT_used':False},indent=2))\n    fit=memory.estimate;probe_metrics=")
    begin=text.index("    drive.active=True\n    phase='ESTIMATED_FOLLOW'")
    end=text.index("    if structure_policy:",begin)
    original=text[begin:end]
    active="""    if job.get('active_structure') and structure_policy:
     ap=job['active_structure']['refinement'];phase='ESTIMATED_FOLLOW';drive.active=False
     refinement_start=tick*dt;refinement_start_angle=memory.angle_deg(tcp());path_length=0.;segment_log=[];last_position=tcp()[:3,3].copy()
     (a.output/'refinement_entered.json').write_text(json.dumps({'discovery_status':memory.discovery_status,'time_s':tick*dt,'budget':ap,'GT_used':False}))
     for segment_index in range(ap['maximum_segments']):
      E=tcp();q=np.asarray(robot.get_joint_positions());ordered_q=np.r_[q[arm],q[fingers]]
      direction,candidates=memory.choose_segment(E,ordered_q,model,collision,base,moving(),grasp_tcp,moving_initial)
      if direction is None:
       (a.output/'refinement_step_failure.json').write_text(json.dumps({'candidates':candidates,'segment':segment_index}))
       raise RuntimeError('REFINEMENT_NO_SAFE_INCREMENT')
      drive.refresh_tangent(direction,E);drive.active=True;start=E[:3,3].copy();start_time=tick*dt
      for k in range(int(ap['segment_timeout_s']/dt)):
       step()
       if k%8==0:
        E=tcp();memory.observe(E);path_length+=float(np.linalg.norm(E[:3,3]-last_position));last_position=E[:3,3].copy()
        if np.linalg.norm(E[:3,3]-start)>=ap['segment_m']:break
        if path_length>=ap['maximum_path_m'] or tick*dt-refinement_start>=ap['maximum_sim_s']:break
      memory.refresh()
      safety_updates.append({'time_s':tick*dt,'fit':json.loads(json.dumps(memory.estimate)),'follow_sign':memory.follow_sign,'initial_ee':memory.initial.tolist()})
      segment_log.append({'index':segment_index,'direction':direction.tolist(),'candidates':candidates,'actual_displacement_m':float(np.linalg.norm(tcp()[:3,3]-start)),'path_m':path_length,'estimated_angle_deg':memory.angle_deg(tcp()),'duration_s':tick*dt-start_time})
      (a.output/'active_refinement.json').write_text(json.dumps({'entered':True,'segments':segment_log,'path_m':path_length,'duration_s':tick*dt-refinement_start,'start_angle_deg':refinement_start_angle,'current_angle_deg':memory.angle_deg(tcp()),'GT_used':False},indent=2))
      if memory.angle_deg(tcp())>=ap['target_angle_deg']:break
      if path_length>=ap['maximum_path_m'] or tick*dt-refinement_start>=ap['maximum_sim_s']:break
      if segment_log[-1]['actual_displacement_m']<.0001:break
     drive.active=False
     # A reverse is optional. Current protocol uses a separate safe forward
     # validation action, so no unavailable reverse can block all refinement.
    else:
"""+''.join(' '+line+'\n' for line in original.rstrip().splitlines())
    text=text[:begin]+active+text[end:]
    replace("     adopt(memory,selected);fit=memory.estimate", "     if not selected['accepted']:raise RuntimeError(selected['status'])\n     adopt(memory,selected);fit=memory.estimate")
    return text


if __name__=='__main__':
    exec(compile(source(),str(ROOT/'scripts/run_interactive_twin_refinement_episode.py'),'exec'),globals())
