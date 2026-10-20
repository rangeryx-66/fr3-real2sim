"""Offline reference-seed diagnostic at an archived stopped pull; no physics.

Reconstructs the declared Cartesian goal from allowed measured rail state and
the recorded progress reference. The first stored sample approximates handover
bias; this is not an executed controller A/B or a motor-output measurement.
"""
import argparse,json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns
from update_probing_v2_model import matrix
from cross_robot_adapter import RobotPinkIK
from interaction_reference_handover import seed_pose


def compare(folder):
    p=Path(folder);a,index=columns(p)
    config=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())
    job=json.loads((p/'job.json').read_text())
    base=json.loads((Path(job['source'])/'report.json').read_text())['robot_base_pose']
    arms=config['arm_indices'];solver=RobotPinkIK(config['gripper_config'])
    axis=np.array(json.loads((p/'ARTICULATION_INTERFACE.json').read_text())['prior']['axis_world'])
    s=index['segments'][0];ix=np.flatnonzero(a['segment']==s['id']);first,last=ix[0],ix[-2]
    start=matrix(a['tcp_pose'][first]);goal=start.copy()
    goal[:3,3]+=axis*(a['reference_q'][last]-a['joint_q'][first])
    q=a['q'][last,arms];command=a['command_q'][last]
    bias=a['command_q'][first]-a['q'][first,arms]
    measured=solver.velocity_step(goal,base,q,1/240)+bias
    nominal=solver.velocity_step(goal,base,command-bias,1/240)+bias
    origin=seed_pose(solver,base,command)[:3,3]
    return dict(diagnostic_only=True,not_executed=True,no_hidden_dynamics=True,
        recorded_reference_lead_mm=float((a['reference_q'][last]-a['joint_q'][last])*1000),
        command_measured_rail_offset_mm=float((a['command_TCP_pose'][last,:3]-a['tcp_pose'][last,:3])@axis*1000),
        reconstructed_clock_goal_gain_mm=float((goal[:3,3]-start[:3,3])@axis*1000),
        measured_seed_next_requested_FK_change_mm=float((seed_pose(solver,base,measured)[:3,3]-origin)@axis*1000),
        frozen_nominal_seed_next_requested_FK_change_mm=float((seed_pose(solver,base,nominal)[:3,3]-origin)@axis*1000),
        limitations=['first recorded sample approximates handover bias','one QP comparison, before existing command acceleration/collision validation','native execution is required to establish changed physical transmission'])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=compare(args.folder);args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))
