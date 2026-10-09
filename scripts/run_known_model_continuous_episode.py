"""Known whole-task trial with continuous loaded arm position command.
Only the measured-to-command bias at the transition is retained; robot gains,
jaw preload, path geometry, and every original safety limit stay unchanged.
"""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_whole_episode import source as one_station
from run_known_model_two_station_episode import source as two_station

def source(two=False):
 text=two_station() if two else one_station()
 old=";seed=np.asarray(robot.get_joint_positions())[arm]\n   origin="
 new=";seed=np.asarray(robot.get_joint_positions())[arm];command_bias=qtarget[arm]-seed\n   (a.output/('command_bias_'+str(regrasp_count)+'.json' if 'regrasp_count' in locals() else 'command_bias.json')).write_text(json.dumps({'q_measured':seed.tolist(),'command_q':qtarget[arm].tolist(),'bias':command_bias.tolist(),'purpose':'retain the loaded position-command equilibrium at path transition; unchanged gains and jaw preload'},indent=2))\n   origin="
 if text.count(old)!=(2 if two else 1):raise RuntimeError('CONTINUITY_TRANSITION_HOOK_CHANGED')
 text=text.replace(old,new)
 old="move(waypoint['q'],duration);previous=planned_reference.copy()"
 new="command=np.asarray(waypoint['q'])+command_bias\n    if model.margin(command)<=.05:raise RuntimeError('COMMAND_MARGIN_BELOW_EXISTING_LIMIT')\n    move(command,duration);previous=planned_reference.copy()"
 if text.count(old)!=(2 if two else 1):raise RuntimeError('CONTINUITY_MOVE_HOOK_CHANGED')
 text=text.replace(old,new)
 text=text.replace("'grip_control_continuous':True", "'grip_control_continuous':True,'loaded_arm_command_continuous':True")
 ast.parse(text);return text

if __name__=='__main__':
 import json
 path=Path(sys.argv[sys.argv.index('--job')+1]);job=json.loads(path.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text());text=source('second_station' in whole);exec(compile(text,str(ROOT/'scripts/run_known_model_continuous_episode.py'),'exec'),globals())
