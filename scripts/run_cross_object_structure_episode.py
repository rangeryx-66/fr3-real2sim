"""Independent structure protocol; reuse frozen grasp, safety and native stepping.

Explicit checked insertion points leave baseline files byte-for-byte unchanged.
Only post-discovery action scheduling and structured-memory refinement are added.
"""
import hashlib
from pathlib import Path


def source():
    base=Path(__file__).with_name('run_interactive_twin_refinement_episode.py')
    text=base.read_text()
    def replace(old,new):
        nonlocal text
        if text.count(old)!=1: raise RuntimeError('FROZEN_STRUCTURE_HOOK_CHANGED:'+old[:60])
        text=text.replace(old,new)
    replace("job=json.loads(parser().parse_args().job.read_text())", "job=json.loads(parser().parse_args().job.read_text())\n from cross_object_structure.refinement import refine,adopt\n structure_policy=job.get('structure_protocol');discovery_snapshot=None")
    replace(" install_setup_capture()", " install_setup_capture()\n from cross_object_structure.scene import install_no_fixture_guard\n install_no_fixture_guard()")
    replace("protocol=None;protocol_completed=False;issued=[];replay_last_compliant=None;calibration_comp_fraction=1.",
            "protocol=None;protocol_completed=False;issued=[];replay_last_compliant=None;calibration_comp_fraction=1.\n safety_updates=[];schedule=json.loads(Path(job['safety_schedule']).read_text()) if job.get('safety_schedule') else [];schedule_index=0")
    replace("  native.clear()", """  nonlocal schedule_index
  while schedule_index<len(schedule) and tick*dt>=schedule[schedule_index]['time_s']-1e-9:
   update=schedule[schedule_index]
   if memory is None:memory=InteractionMemory(np.asarray(update['initial_ee']),a.output)
   memory.estimate=update['fit'];memory.follow_sign=update['follow_sign'];schedule_index+=1
  native.clear()""")
    # New action labels must retain the exact existing force cap. Changing a
    # logging phase must never bypass the baseline preload safety check.
    replace("'PHYSICS_HOLD','ROBOT_CALIBRATION') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']",
            "'PHYSICS_HOLD','ROBOT_CALIBRATION','REFINEMENT_REVERSE','REFINEMENT_REVERSAL_SETTLE','STRUCTURE_VALIDATION_SETTLE','STRUCTURE_VALIDATION','STRUCTURE_MODEL_SELECTION','HELDOUT_MANIPULATION','HELDOUT_DWELL') and max(s['forces_n'].values(),default=0)>policy['max_pad_load_n']")
    replace("    if job.get('mode') in ('physics_reference','sensitivity_reference'):",
            "    safety_updates.append({'time_s':tick*dt,'fit':json.loads(json.dumps(memory.estimate)),'follow_sign':memory.follow_sign,'initial_ee':memory.initial.tolist()})\n    if structure_policy:\n     discovery_snapshot=json.loads(json.dumps(fit));(a.output/'discovery_articulation.json').write_text(json.dumps(discovery_snapshot,indent=2))\n    if job.get('mode') in ('physics_reference','sensitivity_reference'):")
    replace("       memory.try_fit();last_refine=tick*dt", "       memory.try_fit();last_refine=tick*dt\n       safety_updates.append({'time_s':tick*dt,'fit':json.loads(json.dumps(memory.estimate)),'follow_sign':memory.follow_sign,'initial_ee':memory.initial.tolist()})")
    marker="    phase='FINAL_HOLD';drive.active=False;hold(2.);ready,detail=grip_window();"
    added="""    if structure_policy:
     # Independent complete forward validation action. A sub-mm reverse is
     # retained as unavailable evidence, never manufactured or required.
     phase='STRUCTURE_VALIDATION_SETTLE';drive.active=False;hold(1.)
     phase='STRUCTURE_VALIDATION';start_position=tcp()[:3,3].copy();segment_start=start_position.copy()
     drive.set_direction(memory.tangent(tcp()),tcp());drive.active=True
     for j in range(int(structure_policy['validation']['maximum_s']/dt)):
      step()
      if j%8==0:
       E=tcp()
       if np.linalg.norm(E[:3,3]-segment_start)>=.001:
        drive.refresh_tangent(memory.tangent(E),E);segment_start=E[:3,3].copy()
       if np.linalg.norm(E[:3,3]-start_position)>=structure_policy['validation']['forward_travel_m']:break
     phase='STRUCTURE_MODEL_SELECTION';drive.active=False
     selected=refine(rows,discovery_snapshot,structure_policy,a.output)
     adopt(memory,selected);fit=memory.estimate
     safety_updates.append({'time_s':tick*dt,'fit':json.loads(json.dumps(memory.estimate)),'follow_sign':memory.follow_sign,'initial_ee':memory.initial.tolist()})
     # Entire held-out command is preregistered. No observations from this
     # action are sent to either fitter or model-selection routine.
     phase='HELDOUT_MANIPULATION';heldout_start=len(rows);segment_start=tcp()[:3,3].copy()
     (a.output/'heldout_start.json').write_text(json.dumps({'time_s':tick*dt,'T_ee':tcp().tolist(),'selected_estimate':fit,'no_GT':True}))
     drive.set_direction(memory.tangent(tcp()),tcp());drive.active=True
     for j in range(int(structure_policy['heldout']['active_s']/dt)):
      step()
      if j%8==0:
       E=tcp()
       if np.linalg.norm(E[:3,3]-segment_start)>=.001:
        drive.refresh_tangent(memory.tangent(E),E);segment_start=E[:3,3].copy()
     phase='HELDOUT_DWELL';drive.active=False;hold(structure_policy['heldout']['dwell_s'])
     (a.output/'heldout_completion.json').write_text(json.dumps({'completed':True,'start_index':heldout_start,'end_index':len(rows),'model':'T2 robust SE3','fit_updated':False,'GT_inputs':False}))
"""
    replace(marker,added+marker)
    replace("  gt_gate.open=True", "  online_success=success\n  (a.output/'online_outcome.json').write_text(json.dumps({'status':status,'online_success':online_success,'GT_used':False,'interaction_stopped':True}))\n  gt_gate.open=True")
    replace("  (a.output/'command_tape.json').write_text(json.dumps(issued))", "  (a.output/'command_tape.json').write_text(json.dumps(issued))\n  (a.output/'safety_schedule.json').write_text(json.dumps(safety_updates))")
    replace("  report.update(experiment='interactive_twin'", "  report.update(structure_protocol=bool(structure_policy),online_success=online_success,success_is_post_episode_evaluation=True,frozen_runner_sha256='"+hashlib.sha256(base.read_bytes()).hexdigest()+"')\n  report.update(experiment='interactive_twin'")
    return text


if __name__=='__main__':
    exec(compile(source(),str(Path(__file__).with_name('run_interactive_twin_refinement_episode.py')),'exec'),globals())
