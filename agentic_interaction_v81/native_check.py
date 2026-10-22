"""Short unloaded native mount check; not a contact/research episode.

Uses the prepared V8 job/resource pipeline. No Agent pose, object motion or
grasp is generated. The first check measures stationary mount/FK consistency.
"""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);a=p.parse_args()
    job=json.loads(a.job.read_text())
    from agentic_interaction_v8.cli import audit
    audit()
    import xarm7_native_gripper
    xarm7_native_gripper.OUT=Path(job['v8_robot_io'])
    from interaction_runtime.probing_v7 import RuntimeV7
    from .runtime import runtime_class
    from run_efficient_probing_v2 import main as run
    from agent_command_protocol_v12 import atomic_json
    import numpy as np
    def check(r):
        r.g['phase']='SETTLE';r.hold(.5)
        measured=r.tcp();predicted=r.model.poses(np.asarray(r.robot.get_joint_positions())[r.arm],r.base,
            finger_q=np.asarray(r.robot.get_joint_positions())[r.fingers])['tcp_link']
        delta=float(np.linalg.norm(measured[:3,3]-predicted[:3,3]))
        caps=r.robot._articulation_view.get_max_efforts()[0]
        result=dict(status='STATIONARY_MOBILE_MOUNT_CHECK',base=r.base.tolist(),
            base_joint_positions=np.asarray(r.robot.get_joint_positions())[r.base_indices].tolist(),
            base_caps=np.asarray(caps)[r.base_indices].tolist(),arm_caps=np.asarray(caps)[r.arm].tolist(),
            TCP_FK_translation_error_m=delta,wrist_valid=all(row['wrist_ft']['valid'] for row in r.rows),
            acquisitions=0,object_joint_commands=False,base_motion_executed=False)
        atomic_json(r.output/'V81_STATIONARY_MOUNT_CHECK.json',result)
        if delta>.001:raise RuntimeError('MOUNT_FK_INCONSISTENT')
        return result
    # Retain the original full-loop owner and normal structured shutdown.
    sys.argv=[sys.argv[0],'--job',str(a.job)]
    return run(runtime_class(RuntimeV7),check,extra_sources=('agentic_interaction_v81/mobile_mount.py','agentic_interaction_v81/runtime.py'))


if __name__=='__main__':sys.exit(main())
