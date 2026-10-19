"""Short paired tracking checks; frozen scene/servo/preload, robot boundary init.
Only continuous loaded tracking bias is varied. Debug progress is excluded.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_continuous_episode import source as continuous


def source(adaptive=False,debug=True,two=False):
 text=continuous(two)
 if adaptive:raise RuntimeError("ADAPTIVE_CONTROLLER_NOT_PART_OF_THIS_SCOPE")
 if debug:
  old="qtarget[fingers]=[.05,-.05];qvelocity="
  new="qtarget[fingers]=[.05,-.05];\n boundary=job['paired_debug'];qinit=np.asarray(boundary['q'],float);opening=float(boundary['aperture_m'])+.004;qinit[fingers]=[opening/2,-opening/2];robot.set_joint_positions(qinit);qtarget=qinit.copy();qtarget[arm]=boundary['command_q_arm'];\n qvelocity="
  if text.count(old)!=1:raise RuntimeError('TRACKING_BOUNDARY_HOOK_CHANGED')
  text=text.replace(old,new)
  a=text.index("   phase='PREGRASP'");b=text.index("   closure=JawCenteredClosure",a);text=text[:a]+"   phase='PAIRED_DEBUG_BOUNDARY_SETTLE';hold(.5)\n"+text[b:]
  text=text.replace("'mode':'KNOWN_MODEL_DIAGNOSTIC'", "'mode':'KNOWN_MODEL_DIAGNOSTIC_DEBUG','task_progress_eligible':False,'robot_boundary_initialized':True,'tracking_pair':job['tracking_pair']")
 text=text.replace("'loaded_arm_command_continuous':True", "'loaded_arm_command_continuous':True,'loaded_tracking_bias_adaptive':"+repr(adaptive))
 ast.parse(text);return text

if __name__=='__main__':
 import json
 p=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(p.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text());text=source(job.get('tracking_pair')=='ADAPTIVE_BIAS',bool(job.get('paired_debug')),'second_station' in whole);exec(compile(text,str(ROOT/'scripts/run_known_model_tracking_pair.py'),'exec'),globals())
