"""Short paired control-transition checks; robot-only boundary initialization.
These independent debug scenes never count as closed-to-open task progress.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_whole_episode import source as original

def source():
 text=original()
 def replace(a,b):
  nonlocal text
  if text.count(a)!=1:raise RuntimeError('CONTROL_PAIR_HOOK_CHANGED:'+a[:100])
  text=text.replace(a,b)
 replace("qtarget[fingers]=[.05,-.05];qvelocity=", "qtarget[fingers]=[.05,-.05];\n boundary=job['paired_debug'];qinit=np.asarray(boundary['q'],float);opening=float(boundary['aperture_m'])+.004;qinit[fingers]=[opening/2,-opening/2];robot.set_joint_positions(qinit);qtarget=qinit.copy();qtarget[arm]=boundary['command_q_arm'];\n qvelocity=")
 a=text.index("   phase='PREGRASP'");b=text.index("   closure=JawCenteredClosure",a)
 text=text[:a]+"   phase='PAIRED_DEBUG_BOUNDARY_SETTLE';hold(.5)\n"+text[b:]
 a=text.index("   E0=tcp();D0=moving();theta0=");b=text.index("\n except BaseException as error:",a)
 text=text[:a]+'''   phase='PAIRED_HOLD_OR_ZERO';measured=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-measured
   (a.output/'transition_state.json').write_text(json.dumps({'q_measured':measured.tolist(),'command_q':qtarget[arm].tolist(),'bias':bias.tolist(),'forces_n':rows[-1]['forces_n'],'test':job['pair_test']},indent=2))
   if job['pair_test']=='HOLD_COMMAND':hold(2.)
   elif job['pair_test']=='MEASURED_ZERO':move(measured,.25);hold(1.)
   elif job['pair_test']=='CONTINUOUS_ZERO':move(measured+bias,.25);hold(1.)
   else:raise RuntimeError('UNKNOWN_PAIRED_TEST')
   ready,detail=grip_window();success=bool(ready);status='PAIRED_HOLD_PASS' if success else 'PAIRED_HOLD_LOSS'
''' +text[b:]
 replace("'mode':'KNOWN_MODEL_DIAGNOSTIC'", "'mode':'KNOWN_MODEL_DIAGNOSTIC_DEBUG','task_progress_eligible':False,'robot_boundary_initialized':True,'pair_test':job['pair_test']")
 ast.parse(text);return text

if __name__=='__main__':
 text=source();exec(compile(text,str(ROOT/'scripts/run_known_model_control_pair.py'),'exec'),globals())
