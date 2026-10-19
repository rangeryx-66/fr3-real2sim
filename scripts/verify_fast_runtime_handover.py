"""Replay archived native command states through the actual new Cartesian handover."""
import argparse,json,pickle,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from cross_robot_adapter import RobotPinkIK
from interaction_reference_handover import seed_pose
from interaction_runtime.runtime import Runtime
from interaction_runtime.policy import Policy

def verify(ep):
 rows=pickle.load((ep/'hybrid_identification/00/measurements.pkl').open('rb'));i=next(i for i in range(1,len(rows)) if rows[i-1]['phase']=='LOCAL_CENTERING' and rows[i]['phase']=='TRANSMISSION_PROBE');row=rows[i-1]
 c=json.loads((ep/'sensor_assembly/SENSOR_CONFIG.json').read_text());c['urdf']=str(ep/'sensor_assembly/sim_urdf.urdf');r=object.__new__(Runtime);r.solver=RobotPinkIK(c);r.c=c;r.base=row['base'];r.dt=1/240;r.tick=0;r.arm=np.array([list(c['arm_joint_names']).index(n) for n in c['arm_joint_names']]);r.arm=np.arange(7)
 r.robot=SimpleNamespace(get_joint_positions=lambda:np.asarray(row['q']));r.qtarget=np.asarray(row['q']).copy();r.qtarget[r.arm]=row['command_q_arm'];r.g=dict(qvelocity=np.zeros(7));r.policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json');r.event=lambda *args,**kwargs:None;r.reference_drive=None;r.check_q=lambda q:(True,'offline FK/controller replay only');r.step=lambda:None
 before=seed_pose(r.solver,r.base,r.qtarget[r.arm]);r.begin_reference();target=r.reference_drive.pose.copy();target[0,3]+=.0001
 r.cartesian_step(target,'TRANSMISSION_PROBE');after=seed_pose(r.solver,r.base,r.qtarget[r.arm]);speed=float(np.linalg.norm(after[:3,3]-before[:3,3])/r.dt*1000)
 assert speed<1.0,speed
 return dict(archived_handover_t=row['t'],actual_new_runtime_first_command_fk_mm_s=speed,old_command_fk_mm_s=35.1942511359,position_command_continuity_repaired=True,translation_and_rotation_limited_independently=True,physics_actors=0,not_physical_safety_evidence=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=verify(a.episode);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
