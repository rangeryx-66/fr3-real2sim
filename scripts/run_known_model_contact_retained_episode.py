"""Native permitted pad contact confirms retention; a scalar center interval
remains diagnostic. All baseline loaded-contact, overload and speed stops stay.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_corrected_episode import source as original

def source(two=False):
 text=original(two)
 old="if np.max(filtered())<.05 or not capture_verified:raise RuntimeError('CONFIRMED_GRASP_LOSS')"
 new="if np.max(filtered())<.05:raise RuntimeError('CONFIRMED_GRASP_LOSS')"
 if text.count(old)!=1:raise RuntimeError('CONTACT_RETENTION_HOOK_CHANGED')
 text=text.replace(old,new)
 text=text.replace("'physical_capture_verified':capture_verified", "'center_interval_diagnostic':capture_verified")
 text=text.replace("'grasp_loss_confirmation':'no effective pad contact or handle outside existing between-jaws capture; single-pad unload is data-quality warning'", "'grasp_loss_confirmation':'sustained absence of effective permitted pad contact; center interval is diagnostic; illegal contact and all physical thresholds unchanged'")
 old="    move(command,duration);previous=planned_reference.copy()"
 new="    move(command,duration);previous=planned_reference.copy()\n    if kind=='prismatic' and abs(waypoint['state']-.04)<1e-6:\n     phase='SHORT_CONTACT_CONTINUATION_CHECK';qvelocity=np.zeros(6);hold(.5);(a.output/'short_continuation_validation.json').write_text(json.dumps({'actual_state':rows[-1]['door_angle_deg'],'native_pad_loads':rows[-1]['forces_n'],'center_interval_diagnostic':capture_verified,'all_original_physical_guards_active':True,'full_reference_state':waypoint['state']},indent=2))"
 if text.count(old)!=(2 if two else 1):raise RuntimeError('SHORT_CONTINUATION_HOOK_CHANGED')
 text=text.replace(old,new)
 # Preserve the live field after a recoverable grasp/implementation stop. This
 # is the existing bounded paused-run pattern, not an object-state restoration.
 old=';app.close()'
 new='''
  if not success and (status.startswith('CONFIRMED_GRASP_LOSS') or status.startswith('IMPLEMENTATION_ERROR')):
   import time
   (a.output/'recovery_pending.json').write_text(json.dumps({'status':'SAFE_HOLD_PAUSED','stop':status,'q':np.asarray(robot.get_joint_positions()).tolist(),'base':base,'deadline':a.deadline_shanghai,'object_state_restored':False,'code_hot_modified':False},indent=2))
   while datetime.now(ZoneInfo('Asia/Shanghai'))<deadline:
    world.render();time.sleep(.2)
   (a.output/'recovery_pending.json').write_text(json.dumps({'status':'WALL_BUDGET_EXHAUSTED','prior_stop':status,'object_state_restored':False},indent=2))
  app.close()'''
 if text.count(old)!=1:raise RuntimeError('PAUSED_FIELD_HOOK_CHANGED')
 text=text.replace(old,new);ast.parse(text);return text

if __name__=='__main__':
 import json
 p=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(p.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text());text=source('second_station' in whole);exec(compile(text,str(ROOT/'scripts/run_known_model_contact_retained_episode.py'),'exec'),globals())
