"""Paired intentional-release test: explicitly clear latched jaw effort only
when changing from grasp effort mode to release position mode. Debug only.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_control_pair import source as original


def source(clear=False):
 text=original()
 if clear:
  old='ArticulationAction(joint_positions=qtarget[fingers],joint_indices=fingers) if mode=='
  new='ArticulationAction(joint_positions=qtarget[fingers],joint_efforts=np.zeros(2),joint_indices=fingers) if mode=='
  if text.count(old)!=1:raise RuntimeError('RELEASE_EFFORT_HOOK_CHANGED')
  text=text.replace(old,new)
 a=text.index("   phase='PAIRED_HOLD_OR_ZERO';");b=text.index('\n except BaseException as error:',a)
 text=text[:a]+'''   reference=None;phase='PAIRED_RELEASE';effort=0.;mode='position'
   kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kp[fingers]=1000.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
   present=rows[-1]['aperture_m'];opening=min(.1,present+float(job.get('release_test_extra_m',.020)))
   for w in np.linspace(present,opening,max(2,int((opening-present)/(.001*dt))+1)):
    qtarget[fingers]=[w/2,-w/2];step()
   hold(.3);error=rows[-1]['aperture_m']-opening
   (a.output/'release_measurement.json').write_text(json.dumps({'target_aperture_m':opening,'actual_aperture_m':rows[-1]['aperture_m'],'error_m':error,'pad_loads_n':rows[-1]['forces_n'],'clear_effort_on_position_action':job['clear_release_effort']},indent=2))
   if abs(error)>.001:raise RuntimeError('RELEASE_APERTURE_TRACKING_IMPLEMENTATION_ERROR')
   if np.max(filtered())>.05:raise RuntimeError('RELEASE_CONTACT_REMAINS')
   # Same local collision model, measured released aperture; short 2cm exit.
   phase='PAIRED_LOCAL_EXIT';E=tcp();sq=np.asarray(robot.get_joint_positions())[arm]
   for amount in np.linspace(0,float(job.get('local_exit_m',.02)),5)[1:]:
    target=E.copy();target[:3,3]-=amount*E[:3,2];q=ik(target,sq)
    if q is None:raise RuntimeError('PAIRED_LOCAL_EXIT_NO_IK')
    ok,why=collision.check(model.poses(q,base,width=opening),moving(),False)
    if not ok:raise RuntimeError(why)
    from wrist_reconstruction.planner import camera_clearance
    from wrist_reconstruction.geometry import calibration
    ok,why=camera_clearance(collision,model.poses(q,base,width=opening),calibration(job['camera_calibration']))
    if not ok:raise RuntimeError(why)
    move(q,1.25);sq=q
   hold(.3);success=True;status='PAIRED_RELEASE_AND_LOCAL_EXIT_PASS'
''' +text[b:]
 text=text.replace("'pair_test':job['pair_test']", "'pair_test':job['pair_test'],'clear_release_effort':job['clear_release_effort']")
 ast.parse(text);return text

if __name__=='__main__':
 import json
 p=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(p.read_text());text=source(job['clear_release_effort']);exec(compile(text,str(ROOT/'scripts/run_known_model_release_pair.py'),'exec'),globals())
