"""Short unloaded native mount check; not a contact/research episode.

Uses the prepared V8 job/resource pipeline. No Agent pose, object motion or
grasp is generated. The first check measures stationary mount/FK consistency.
"""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path)
    p.add_argument('--move-xy',nargs=2,type=float,help='Short engineering fixture only, <=30 mm displacement')
    p.add_argument('--dwell-diagnostics',action='store_true',help='Two seconds of actual unloaded240Hz joint/pose samples before any base motion')
    a=p.parse_args()
    if a.move_xy and (not all(__import__('math').isfinite(v) for v in a.move_xy) or __import__('math').hypot(*a.move_xy)>.03):
        raise ValueError('BOUNDED_UNLOADED_INTERFACE_MOTION_ONLY')
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
        if a.dwell_diagnostics:
            rows=[]
            for _ in range(round(2/r.dt)):
                r.hold(r.dt);row=r.rows[-1]
                rows.append((row['t'],np.asarray(row['q'])[r.arm].copy(),
                    np.asarray(row['joint_velocities'])[r.arm].copy(),row['T_tcp'].copy(),r.actual()))
            t=np.array([x[0]for x in rows]);q=np.array([x[1]for x in rows]);dq=np.array([x[2]for x in rows]);T=np.array([x[3]for x in rows])
            np.savez_compressed(r.output/'UNLOADED_STATION_240HZ.npz',t=t,q=q,dq_reported=dq,T_tcp=T,object_q=np.array([x[4]for x in rows]))
            sample=dict(native_reported_velocity_max_rad_s=np.max(abs(dq),axis=0).tolist(),
                measured_joint_excursion_rad=np.ptp(q,axis=0).tolist(),
                configuration_velocity_max_rad_s=np.max(abs(np.diff(q,axis=0)/np.diff(t)[:,None]),axis=0).tolist(),
                measured_motion_dwell=r.station_motion.latest,
                no_actuator_or_physics_change=True,acquisitions=0)
            atomic_json(r.output/'UNLOADED_STATION_DIAGNOSTIC.json',sample)
            if not r.station_motion.ready(t[-1],q[-1]):
                raise RuntimeError('MEASURED_CONFIGURATIONS_NOT_STATIONARY_NO_BASE_MOTION')
        if a.move_xy:
            initial=r.base.copy();target=initial[:2]+np.asarray(a.move_xy)
            forward=r.move_station(target)
            if not forward.get('reached'):raise RuntimeError('CHECKED_BASE_FORWARD_NOT_REACHED:'+str(forward.get('status')))
            reverse=r.move_station(initial[:2])
            if not reverse.get('reached'):raise RuntimeError('CHECKED_BASE_RETURN_NOT_REACHED')
            result.update(status='UNLOADED_BASE_ROUND_TRIP_CHECK',forward=forward,reverse=reverse,
                base_motion_executed=True,actual_outward_distance_m=float(np.linalg.norm(np.asarray(forward['actual_base'])[:2]-initial[:2])))
            atomic_json(r.output/'V81_BASE_ROUND_TRIP_CHECK.json',result)
        return result
    # Retain the original full-loop owner and normal structured shutdown.
    sys.argv=[sys.argv[0],'--job',str(a.job)]
    return run(runtime_class(RuntimeV7),check,extra_sources=('agentic_interaction_v81/mobile_mount.py','agentic_interaction_v81/runtime.py'))


if __name__=='__main__':sys.exit(main())
