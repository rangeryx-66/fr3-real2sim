"""Short unloaded native-actuator exit fixture, not a manipulation result.

An archived observation joint state is a diagnostic destination only. Fresh
native planning and actual actuator motion must reach it; never initialize or
teleport the robot there. The original parked-state verifier owns completion.
"""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_v751 import PrismaticRuntimeV751,PrismaticOperationsV751
from interaction_runtime.prismatic_v73 import audit_frozen
from agent_command_protocol_v12 import atomic_json


def serve_exit(r):
    from diagnose_probing_v3 import columns
    from native_g1_safe_park import route_to_joint,plan_exit
    ops=PrismaticOperationsV751(r)
    try:
        source=Path(r.job['unloaded_exit_observation_fixture'])
        old,index=columns(source)
        config=json.loads((source/'EFFECTIVE_RUNTIME.json').read_text())
        target=old['q'][-1,np.asarray(config['arm_indices'])].astype(float)
        if np.max(old['pad_load'][-1])>.05:raise ValueError('ARCHIVED_STATE_NOT_UNLOADED')
        r.g['normal']=np.asarray(json.loads((source/'agent_geometry/observed_geometry.json').read_text())['normal_world'])
        r.g['phase']='SETTLE';r.hold(.5)
        measured=np.asarray(r.robot.get_joint_positions())[r.arm]
        route,reason=route_to_joint(r.g,measured,target)
        if route is None:return dict(status='OUTBOUND_FIXTURE_UNREACHABLE',reason=reason,physical_grasps=0)
        r.move_path(route[1:],'PREGRASP');r.stop_command();r.hold(.3)
        reached=np.asarray(r.robot.get_joint_positions())[r.arm]
        if np.max(abs(reached-target))>.025:raise RuntimeError('FIXTURE_STATE_NOT_PHYSICALLY_REACHED')
        if max(r.filtered())>.05 or not r.supervisor.force.settled():raise RuntimeError('FIXTURE_STATE_NOT_UNLOADED')
        original=plan_exit(r.g)
        returned=ops.release_park()
        result=dict(status='UNLOADED_EXIT_PHYSICAL_FIXTURE_COMPLETE',physical_grasps=0,
            archived_pose_input=True,GT_grasp_input=False,robot_teleport=False,
            target_q=target.tolist(),actual_difficult_q=reached.tolist(),outbound_route_method=reason,
            original_cartesian_exit_valid=original['valid'],original_reason=original.get('reason'),
            original_planning_wall_s=original.get('planning_wall_s'),safe_exit=returned,
            final_q=np.asarray(r.robot.get_joint_positions())[r.arm].tolist(),
            final_pad_N=list(r.filtered()),final_object_q=r.actual())
        atomic_json(r.output/'PHYSICAL_UNLOADED_EXIT.json',result)
        return result
    finally:ops.close_workers()


if __name__=='__main__':
    audit_frozen(ROOT)
    sys.exit(main(PrismaticRuntimeV751,serve_exit,extra_sources=('scripts/run_prismatic_exit_v751.py',)))
