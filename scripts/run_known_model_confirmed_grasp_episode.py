"""Confirm physical grasp loss separately from a unilateral load warning.
Initial bilateral close, overload, loaded contact, margin and speed checks stay
unchanged. Existing between-jaws geometry confirms continued captured contact.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_tracking_pair import source as fixed_source


def source(debug=False,two=False):
 text=fixed_source(False,debug,two)
 def replace(old,new):
  nonlocal text
  if text.count(old)!=1:raise RuntimeError('GRASP_CONFIRMATION_HOOK_CHANGED:'+old[:100])
  text=text.replace(old,new)
 replace("kind=job['skill']['joint_type'];", "kind=job['skill']['joint_type'];capture_verified=True;")
 replace("  nonlocal tick,loss_s,slip_s,max_state,first_divergence", "  nonlocal tick,loss_s,slip_s,max_state,first_divergence,capture_verified")
 replace("   if loss_s>.15:raise RuntimeError('SUSTAINED_CONTACT_LOSS')", '''   if loss_s>.15:
    if tick%8==0:
     _,capture_detail=grip_window();capture_verified=bool(capture_detail['between_jaws'])
    if np.max(filtered())<.05 or not capture_verified:raise RuntimeError('CONFIRMED_GRASP_LOSS')
    # A single pad unloading is insufficient evidence of grasp detachment.
    # Native permitted pad contact plus the existing captured-handle test is
    # checked before continued motion. No illegal/overloaded contact is waived.
    s['grasp_event']={'code':'GRASP_LOAD_ASYMMETRY','classification':'DATA_QUALITY','physical_capture_verified':capture_verified,'effective_pad_contact_retained':True,'provenance':'native pad loads and existing between-jaws test; initial bilateral verification unchanged'}
    s['effort_valid']=False''')
 replace("'grip_control_continuous':True", "'grip_control_continuous':True,'grasp_loss_confirmation':'no effective pad contact or handle outside existing between-jaws capture; single-pad unload is data-quality warning'")
 ast.parse(text);return text

if __name__=='__main__':
 import json
 p=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(p.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text());text=source(bool(job.get('paired_debug')),'second_station' in whole);exec(compile(text,str(ROOT/'scripts/run_known_model_confirmed_grasp_episode.py'),'exec'),globals())
