"""Frozen whole plan: real approach/close, explicit release actuator transition.
Robot starts at home with planned open aperture, object remains normally closed.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_confirmed_grasp_episode import source as confirmed

def source(two=False):
 text=confirmed(False,two)
 old='ArticulationAction(joint_positions=qtarget[fingers],joint_indices=fingers) if mode=='
 new='ArticulationAction(joint_positions=qtarget[fingers],joint_efforts=np.zeros(2),joint_indices=fingers) if mode=='
 if text.count(old)!=1:raise RuntimeError('RELEASE_EFFORT_HOOK_CHANGED')
 text=text.replace(old,new)
 old='qtarget[fingers]=[.05,-.05];qvelocity='
 new="initial_opening=min(.1,float(whole['grasp_aperture_reference_m'])+float(job.get('pregrasp_extra_aperture_m',.020)));qtarget[fingers]=[initial_opening/2,-initial_opening/2];robot.set_joint_positions(qtarget);qvelocity="
 if text.count(old)!=1:raise RuntimeError('INITIAL_APERTURE_HOOK_CHANGED')
 text=text.replace(old,new).replace('width=.1','width=initial_opening')
 text=text.replace('1.5*distance/.003','1.5*distance/min(.004,float(job.get("reference_speed_m_s",.003)))')
 text=text.replace("'base_prepositioned_before_grasp':True", "'base_prepositioned_before_grasp':True,'initial_gripper_aperture_m':initial_opening,'initial_arm_home_unchanged':True,'explicit_zero_effort_in_position_mode':True")
 ast.parse(text);return text

if __name__=='__main__':
 import json
 p=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(p.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text());text=source('second_station' in whole);exec(compile(text,str(ROOT/'scripts/run_known_model_corrected_episode.py'),'exec'),globals())
